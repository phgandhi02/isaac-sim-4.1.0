# Copyright (c) 2018-2024, NVIDIA CORPORATION. All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto. Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.
#

import asyncio
import glob
import json
import os
import socket
import subprocess
import sys

import carb
import jedi
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

        # get extension path
        self._extension_path = omni.kit.app.get_app().get_extension_manager().get_extension_path(ext_id)

        # get extension settings
        settings = carb.settings.get_settings()
        self._socket_host = settings.get("/exts/omni.isaac.jupyter_notebook/host")
        self._socket_port = settings.get("/exts/omni.isaac.jupyter_notebook/port")

        self._notebook_ip = settings.get("/exts/omni.isaac.jupyter_notebook/notebook_ip")
        self._notebook_port = settings.get("/exts/omni.isaac.jupyter_notebook/notebook_port")
        self._notebook_token = settings.get("/exts/omni.isaac.jupyter_notebook/notebook_token")
        self._notebook_dir = settings.get("/exts/omni.isaac.jupyter_notebook/notebook_dir")
        self._command_line_options = settings.get("/exts/omni.isaac.jupyter_notebook/command_line_options")
        self._classic_notebook_interface = settings.get("/exts/omni.isaac.jupyter_notebook/classic_notebook_interface")

        kill_processes_with_port_in_use = settings.get(
            "/exts/omni.isaac.jupyter_notebook/kill_processes_with_port_in_use"
        )

        # ensure port is free
        if kill_processes_with_port_in_use:
            # windows
            if sys.platform == "win32":
                pids = []
                cmd = ["netstat", "-ano"]
                p = subprocess.Popen(cmd, stdout=subprocess.PIPE)
                for line in p.stdout:
                    if f":{self._socket_port}".encode() in line or f":{self._notebook_port}".encode() in line:
                        if "listening".encode() in line.lower():
                            carb.log_info(f"Open port: {line.strip().decode()}")
                            pids.append(line.strip().split(b" ")[-1].decode())
                p.wait()
                for pid in pids:
                    if not pid.isnumeric():
                        continue
                    carb.log_warn(f"Forced process shutdown with PID {pid}")
                    cmd = ["taskkill", "/PID", pid, "/F"]
                    subprocess.Popen(cmd).wait()
            # linux
            elif sys.platform == "linux":
                pids = []
                cmd = ["netstat", "-ltnup"]
                try:
                    p = subprocess.Popen(cmd, stdout=subprocess.PIPE)
                    for line in p.stdout:
                        if f":{self._socket_port}".encode() in line or f":{self._notebook_port}".encode() in line:
                            carb.log_info(f"Open port: {line.strip().decode()}")
                            pids.append(
                                [chunk for chunk in line.strip().split(b" ") if b"/" in chunk][-1]
                                .decode()
                                .split("/")[0]
                            )
                    p.wait()
                except FileNotFoundError as e:
                    carb.log_warn(f"Command (netstat) not available. Install it using `apt install net-tools`")
                for pid in pids:
                    if not pid.isnumeric():
                        continue
                    carb.log_warn(f"Forced process shutdown with PID {pid}")
                    cmd = ["kill", "-9", pid]
                    subprocess.Popen(cmd).wait()

        # shutdown stream subscription
        self._shutdown_subscription = (
            omni.kit.app.get_app()
            .get_shutdown_event_stream()
            .create_subscription_to_pop(self._on_shutdown_event, name="omni.isaac.jupyter_notebook", order=0)
        )

        # ui components
        self._ui_builder = ui_builder.UIBuilder(
            "Window/Jupyter Notebook", self._notebook_ip, self._notebook_port, self._get_display_url
        )
        self._ui_builder.startup()

        # create socket (code execution)
        self._server = None
        _get_event_loop().create_task(self._create_socket())

        # jedi (autocompletion and introspection)
        # application root path
        app_folder = settings.get_as_string("/app/folder")
        if not app_folder:
            app_folder = carb.tokens.get_tokens_interface().resolve("${app}")
        path = os.path.normpath(os.path.join(app_folder, os.pardir))
        # get extension paths
        folders = [
            "exts",
            "extscache",
            "extsPhysics",
            os.path.join("kit", "exts"),
            os.path.join("kit", "extscore"),
            os.path.join("kit", "extensions"),
            os.path.join("kit", "extsPhysics"),
            os.path.join("kit", "kernel", "py"),
        ]
        added_sys_path = []
        for folder in folders:
            sys_paths = glob.glob(os.path.join(path, folder, "*"))
            for sys_path in sys_paths:
                if os.path.isdir(sys_path):
                    added_sys_path.append(sys_path)
        # python environment
        python_exe = "python.exe" if sys.platform == "win32" else "bin/python3"
        environment_path = os.path.join(path, "kit", "python", python_exe)
        # jedi project
        carb.log_info("Autocompletion: jedi.Project")
        carb.log_info(f"  |-- path: {path}")
        carb.log_info(f"  |-- added_sys_path: {len(added_sys_path)} items")
        carb.log_info(f"  |-- environment_path: {environment_path}")
        self._jedi_project = jedi.Project(
            path=path, environment_path=environment_path, added_sys_path=added_sys_path, load_unsafe_extensions=False
        )

        # run jupyter in a separate process
        self._launch_jupyter_process()

    def on_shutdown(self):
        self._shutdown_subscription = None
        self._ui_builder.shutdown()
        # close socket
        if self._server is not None:
            self._server.close()
            _get_event_loop().run_until_complete(self._server.wait_closed())
            self._server = None
        # end jupyter notebook (external process)
        if self._process is not None:
            process_pid = self._process.pid
            try:
                self._process.terminate()  # .kill()
            except OSError as e:
                if sys.platform == "win32":
                    if e.winerror != 5:
                        raise
                else:
                    from errno import ESRCH

                    if not isinstance(e, ProcessLookupError) or e.errno != ESRCH:
                        raise
            # make sure the process is not running anymore in Windows
            if sys.platform == "win32":
                subprocess.call(["taskkill", "/F", "/T", "/PID", str(process_pid)])
            # wait for the process to terminate
            self._process.wait()
            self._process = None

    def _on_shutdown_event(self, event):
        if event.type == omni.kit.app.POST_QUIT_EVENT_TYPE:
            self.on_shutdown()

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
                source = data.decode()
                # completion
                if source[:3] == "%!c":
                    source = source[3:]
                    asyncio.run_coroutine_threadsafe(
                        self._parent._complete_code_async(source, self.transport), _get_event_loop()
                    )
                # introspection
                elif source[:3] == "%!i":
                    source = source[3:]
                    pos = source.find("%")
                    line, column = [int(i) for i in source[:pos].split(":")]
                    source = source[pos + 1 :]
                    asyncio.run_coroutine_threadsafe(
                        self._parent._introspect_code_async(source, line, column, self.transport), _get_event_loop()
                    )
                # execution
                else:
                    asyncio.run_coroutine_threadsafe(
                        self._parent._process_code(source, self.transport), _get_event_loop()
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

    async def _complete_code_async(self, source: str, transport: asyncio.Transport) -> None:
        """Complete objects under the cursor and send the result back to the client"""
        # generate completions
        script = jedi.Script(source, project=self._jedi_project)
        completions = script.complete()
        delta = completions[0].get_completion_prefix_length() if completions else 0
        # build reply
        reply = {"matches": [c.name for c in completions], "delta": delta}
        # send the reply to the client
        reply = json.dumps(reply)
        transport.write(reply.encode())
        # close the connection
        transport.close()

    async def _introspect_code_async(self, source: str, line: int, column: int, transport: asyncio.Transport) -> None:
        """Introspect code under the cursor and send the result back to the client"""
        # generate introspection
        script = jedi.Script(source, project=self._jedi_project)
        definitions = script.infer(line=line, column=column)
        # build reply
        reply = {"found": False, "data": "TODO"}
        if len(definitions):
            reply["found"] = True
            reply["data"] = definitions[0].docstring()
        # send the reply to the client
        reply = json.dumps(reply)
        transport.write(reply.encode())
        # close the connection
        transport.close()

    # Jupyter Notebook methods

    def _launch_jupyter_process(self) -> None:
        """Launch the Jupyter notebook in a separate process"""
        # get packages path
        paths = [p for p in sys.path if "pip3-envs" in p]
        packages_txt = os.path.join(self._extension_path, "data", "launchers", "packages.txt")
        with open(packages_txt, "w") as f:
            f.write("\n".join(paths))

        if sys.platform == "win32":
            executable_path = os.path.abspath(os.path.join(os.path.dirname(os.__file__), "..", "python.exe"))
        else:
            executable_path = os.path.abspath(os.path.join(os.path.dirname(os.__file__), "..", "..", "bin", "python3"))

        cmd = [
            executable_path,
            os.path.join(self._extension_path, "data", "launchers", "jupyter_launcher.py"),
            self._notebook_ip,
            str(self._notebook_port),
            self._notebook_token,
            str(self._classic_notebook_interface),
            self._notebook_dir,
            self._command_line_options,
        ]

        carb.log_info("Starting Jupyter server in separate process")
        carb.log_info("  |-- command: " + " ".join(cmd))
        try:
            self._process = subprocess.Popen(cmd, cwd=os.path.join(self._extension_path, "data", "launchers"))
        except Exception as e:
            carb.log_error("Error starting Jupyter server: {}".format(e))
            self._process = None

    def _get_display_url(self) -> str:
        """Get the Jupyter notebook app.display_url"""
        display_url = ""
        if self._process is not None:
            notebook_txt = os.path.join(self._extension_path, "data", "launchers", "notebook.txt")
            if os.path.exists(notebook_txt):
                with open(notebook_txt, "r") as f:
                    urls = f.readlines()
                    display_url = "\n".join([url.strip() for url in urls])
        return display_url
