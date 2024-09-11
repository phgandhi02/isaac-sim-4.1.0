# Copyright (c) 2018-2023, NVIDIA CORPORATION. All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto. Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.
#

import asyncio
import gc

import omni.graph.core as og
import omni.graph.core.tests as ogts
import omni.kit.test
from omni.isaac.core.utils.stage import create_new_stage_async


class TestRos2Service(ogts.OmniGraphTestCase):
    # Before running each test
    async def setUp(self):
        import rclpy

        await omni.usd.get_context().new_stage_async()
        self._timeline = omni.timeline.get_timeline_interface()

        ext_manager = omni.kit.app.get_app().get_extension_manager()
        ext_manager.get_enabled_extension_id("omni.isaac.ros2_bridge")
        await omni.kit.app.get_app().next_update_async()

        await create_new_stage_async()
        rclpy.init()

    # After running each test
    async def tearDown(self):
        import rclpy

        while omni.usd.get_context().get_stage_loading_status()[2] > 0:
            print("tearDown, assets still loading, waiting to finish...")
            await asyncio.sleep(1.0)

        self._timeline = None
        rclpy.shutdown()
        gc.collect()

    # ----------------------------------------------------------------------
    async def test_service(self):
        import builtin_interfaces.msg
        import rclpy

        self._timeline.play()
        await omni.kit.app.get_app().next_update_async()

        # define graph
        (test_graph, new_nodes, _, _) = og.Controller.edit(
            {"graph_path": "/ActionGraph", "evaluator_name": "execution"},
            {
                og.Controller.Keys.CREATE_NODES: [
                    ("OnPlaybackTick", "omni.graph.action.OnPlaybackTick"),
                    ("ServerRequest", "omni.isaac.ros2_bridge.OgnROS2ServiceServerRequest"),
                    ("ServerResponse", "omni.isaac.ros2_bridge.OgnROS2ServiceServerResponse"),
                    ("Client", "omni.isaac.ros2_bridge.OgnROS2ServiceClient"),
                ],
                og.Controller.Keys.SET_VALUES: [
                    ("ServerRequest.inputs:serviceName", "/custom_service"),
                    ("ServerRequest.inputs:messagePackage", "example_interfaces"),
                    ("ServerRequest.inputs:messageSubfolder", "srv"),
                    ("ServerRequest.inputs:messageName", "AddTwoInts"),
                    ("ServerResponse.inputs:messagePackage", "example_interfaces"),
                    ("ServerResponse.inputs:messageSubfolder", "srv"),
                    ("ServerResponse.inputs:messageName", "AddTwoInts"),
                    ("Client.inputs:serviceName", "/custom_service"),
                    ("Client.inputs:messagePackage", "example_interfaces"),
                    ("Client.inputs:messageSubfolder", "srv"),
                    ("Client.inputs:messageName", "AddTwoInts"),
                ],
                og.Controller.Keys.CONNECT: [
                    ("OnPlaybackTick.outputs:tick", "Client.inputs:execIn"),
                    ("OnPlaybackTick.outputs:tick", "ServerRequest.inputs:execIn"),
                    ("ServerRequest.outputs:onReceived", "ServerResponse.inputs:onReceived"),
                    ("ServerRequest.outputs:serverHandle", "ServerResponse.inputs:serverHandle"),
                ],
            },
        )

        await og.Controller.evaluate(test_graph)
        await omni.kit.app.get_app().next_update_async()
        server_req_node = new_nodes[1]
        server_res_node = new_nodes[2]
        client_node = new_nodes[3]

        og.Controller.attribute("inputs:Request:a", client_node).set(11)
        og.Controller.attribute("inputs:Request:b", client_node).set(10)

        # wait for the client to executes and send the request
        await omni.kit.app.get_app().next_update_async()
        await og.Controller.evaluate(test_graph)
        a = og.Controller.attribute("outputs:Request:a", server_req_node).get()
        b = og.Controller.attribute("outputs:Request:b", server_req_node).get()

        await omni.kit.app.get_app().next_update_async()
        server_result = a + b
        og.Controller.attribute("inputs:Response:sum", server_res_node).set(server_result)

        # wait for the server to executes and send the response
        await omni.kit.app.get_app().next_update_async()
        await omni.kit.app.get_app().next_update_async()

        client_result = og.Controller.attribute("outputs:Response:sum", client_node).get()
        print("server response = ", server_result)
        print("client response = ", client_result)
        self.assertEqual(client_result, 21)
        self._timeline.stop()
