# Copyright (c) 2018-2024, NVIDIA CORPORATION. All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto. Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.
#

import asyncio
import json
import socket
import sys
import threading
import time

import carb
import omni.ext

from . import executor, ui_builder


def _get_event_loop() -> asyncio.AbstractEventLoop:
    """Backward compatible function for getting the event loop"""
    try:
        return asyncio.get_event_loop()
    except RuntimeError:
        return asyncio.get_event_loop_policy().get_event_loop()


class Extension(omni.ext.IExt):
    def on_startup(self, ext_id):
        self._globals = {**globals()}

        # get extension settings
        settings = carb.settings.get_settings()
        self._socket_host = settings.get("/exts/omni.isaac.vscode/host")
        self._socket_port = settings.get("/exts/omni.isaac.vscode/port")
        self._publish_carb_logs = settings.get("/exts/omni.isaac.vscode/carb_logs")

        # ui components
        self._ui_builder = ui_builder.UIBuilder("Window/VS Code", self._socket_host, self._socket_port)
        self._ui_builder.startup()

        # carb logs
        if self._publish_carb_logs:
            # log listener
            self._logging = carb.logging.acquire_logging()
            self._logger_handle = self._logging.add_logger(self._carb_logging)
            self._logging_levels = {
                # carb.logging.LEVEL_VERBOSE: "Verbose",
                carb.logging.LEVEL_INFO: "Info",
                carb.logging.LEVEL_WARN: "Warning",
                carb.logging.LEVEL_ERROR: "Error",
                carb.logging.LEVEL_FATAL: "Fatal",
            }

            # create UDP socket (carb logs)
            self._udp_server = None
            self._udp_clients = []
            self._udp_server_running = False
            threading.Thread(target=self._create_udp_socket).start()

        # create socket (code execution)
        self._server = None
        _get_event_loop().create_task(self._create_socket())

    def on_shutdown(self):
        self._ui_builder.shutdown()
        # close socket
        if self._server:
            self._server.close()
            _get_event_loop().run_until_complete(self._server.wait_closed())
        # carb logs
        if self._publish_carb_logs:
            self._logging.remove_logger(self._logger_handle)
            # close the UDP socket
            self._logging = None
            self._logger_handle = None
            self._udp_server = None
            self._udp_clients = []
            # wait for the UDP socket to close
            trial_count = 0
            while self._udp_server_running:
                time.sleep(0.1)
                trial_count += 1
                if trial_count > 10:
                    break
            self._udp_server_running = False

    def _carb_logging(self, source, level, filename, lineNumber, message):
        if level in self._logging_levels and self._udp_server and self._udp_clients:
            data = f"[{self._logging_levels[level]}][{source}] {message}".encode()
            for client in self._udp_clients:
                try:
                    self._udp_server.sendto(data, client)
                except Exception as e:
                    carb.log_error(f"{e} len:{len(data)}")

    def _create_udp_socket(self) -> None:
        """Create an UDP socket for broadcasting carb logging"""
        self._udp_clients = []
        self._udp_server_running = True
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as _server:
            try:
                _server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
                _server.bind((self._socket_host, self._socket_port))
                _server.setblocking(False)
                _server.settimeout(0.25)
            except Exception as e:
                self._udp_server = None
                self._udp_clients = []
                carb.log_error(str(e))
                self._udp_server_running = False
                return

            self._udp_server = _server
            while self._udp_server:
                try:
                    _, addr = _server.recvfrom(1024)
                    if addr not in self._udp_clients:
                        self._udp_clients.append(addr)
                except socket.timeout:
                    pass
                except Exception as e:
                    carb.log_warn("UDP server error: {}".format(e))
                    break
            self._udp_server = None
            self._udp_clients = []
            self._udp_server_running = False

    async def _create_socket(self) -> None:
        """Create a socket server to listen for incoming connections"""

        class ServerProtocol(asyncio.Protocol):
            def __init__(self, parent) -> None:
                super().__init__()
                self._parent = parent

            def connection_made(self, transport) -> None:
                carb.log_info(f"Connection from {transport.get_extra_info('peername')}")
                self.transport = transport

            def data_received(self, data) -> None:
                asyncio.run_coroutine_threadsafe(
                    self._parent._process_code(data.decode(), self.transport), _get_event_loop()
                )

        try:
            self._server = await _get_event_loop().create_server(
                protocol_factory=lambda: ServerProtocol(self),
                host=self._socket_host,
                port=self._socket_port,
                family=socket.AF_INET,
                reuse_port=None if sys.platform == "win32" else True,
            )
            carb.log_info(f"Serving at {self._socket_host}:{self._socket_port}")
            await self._server.start_serving()
        except Exception as e:
            carb.log_error(str(e))
            self._server = None

    async def _process_code(self, source: str, transport: asyncio.Transport) -> None:
        """Execute the source code in the Kit Python scope and send the result back to the client"""
        _executor = executor.Executor(self._globals, self._globals)
        output, exception, trace = await _executor.execute(source)
        if output.endswith("\n"):
            output = output[:-1]
        # build reply
        reply = {"status": "ok" if exception is None else "error", "output": output}
        if exception is not None:
            reply["traceback"] = [trace]
            reply["ename"] = str(type(exception).__name__)
            reply["evalue"] = str(exception)
        # send the reply to the client
        reply = json.dumps(reply, separators=(",", ":"))
        transport.write(reply.encode())
        # close the connection
        transport.close()
