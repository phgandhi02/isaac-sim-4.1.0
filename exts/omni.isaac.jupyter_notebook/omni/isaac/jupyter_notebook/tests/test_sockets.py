# Copyright (c) 2018-2023, NVIDIA CORPORATION. All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto. Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.
#

import asyncio
import json

import carb
import omni.kit.test

message = "Hello World!"


class TestSockets(omni.kit.test.AsyncTestCase):
    # Before running each test
    async def setUp(self):
        settings = carb.settings.get_settings()
        self._socket_port = settings.get("/exts/omni.isaac.jupyter_notebook/port")

    # After running each test
    async def tearDown(self):
        pass

    async def test_tcp_socket(self):
        # open TCP socket (code execution)
        reader, writer = await asyncio.open_connection("127.0.0.1", self._socket_port)
        writer.write(f'print("{message}")'.encode())
        # wait for code execution
        for _ in range(10):
            await asyncio.sleep(0.1)
        # parse response
        data = await reader.read()
        data = json.loads(data.decode())
        # validate output
        print("response:", data)
        self.assertEqual("ok", data.get("status", None))
        self.assertEqual(message, data.get("output", None))
