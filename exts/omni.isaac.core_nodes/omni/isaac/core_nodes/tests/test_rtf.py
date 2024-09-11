# Copyright (c) 2021-2024, NVIDIA CORPORATION. All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto. Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.
#


import carb
import omni.graph.core as og
import omni.graph.core.tests as ogts
import omni.kit.test
from omni.isaac.core.robots import Robot
from omni.isaac.core.utils.physics import simulate_async
from omni.isaac.core.utils.stage import open_stage_async
from omni.isaac.core.utils.viewports import get_viewport_names
from omni.isaac.core_nodes.bindings import _omni_isaac_core_nodes
from omni.isaac.nucleus import get_assets_root_path


class TestRealTimeFactor(ogts.OmniGraphTestCase):
    async def setUp(self):
        """Set up  test environment, to be torn down when done"""
        await omni.usd.get_context().new_stage_async()
        self._timeline = omni.timeline.get_timeline_interface()
        self._core_nodes = _omni_isaac_core_nodes.acquire_interface()

        # add franka robot for test
        assets_root_path = get_assets_root_path()
        if assets_root_path is None:
            carb.log_error("Could not find Isaac Sim assets folder")
            return
        (result, error) = await open_stage_async(assets_root_path + "/Isaac/Robots/Franka/franka.usd")

    # ----------------------------------------------------------------------
    async def tearDown(self):
        """Get rid of temporary data used by the test"""
        await omni.kit.stage_templates.new_stage_async()

    # ----------------------------------------------------------------------
    async def test_rtf(self):
        graph_path = "/ActionGraph"
        nodeName = "isaac_test_node"

        (test_graph, new_nodes, _, _) = og.Controller.edit(
            {"graph_path": graph_path, "evaluator_name": "execution"},
            {
                og.Controller.Keys.CREATE_NODES: [
                    ("OnPlaybackTick", "omni.graph.action.OnPlaybackTick"),
                    (nodeName, "omni.isaac.core_nodes.IsaacTestNode"),
                    ("toString", "omni.graph.nodes.ToString"),
                    ("rtfNode", "omni.isaac.core_nodes.IsaacRealTimeFactor"),
                ],
                og.Controller.Keys.CONNECT: [
                    ("OnPlaybackTick.outputs:tick", nodeName + ".inputs:execIn"),
                    ("rtfNode.outputs:rtf", "toString.inputs:value"),
                    ("toString.outputs:converted", nodeName + ".inputs:input"),
                ],
            },
        )

        node_attribute_path = graph_path + "/" + nodeName + ".outputs:output"

        # check for valid RTF after evaluate
        self._timeline.play()
        await omni.kit.app.get_app().next_update_async()
        await omni.kit.app.get_app().next_update_async()
        await omni.kit.app.get_app().next_update_async()

        self.assertGreater(float(og.Controller.get(og.Controller.attribute(node_attribute_path))), 0.0)
