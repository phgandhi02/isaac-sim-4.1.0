# Copyright (c) 2018-2023, NVIDIA CORPORATION. All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto. Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.
#


import asyncio
from re import I

import carb
import omni.graph.core as og
import omni.graph.core.tests as ogts
import omni.kit.test
from omni.isaac.core.robots import Robot
from omni.isaac.core.utils.physics import simulate_async
from omni.isaac.core.utils.stage import open_stage_async
from omni.isaac.nucleus import get_assets_root_path_async


class TestArticulationControllerNode(ogts.OmniGraphTestCase):
    async def setUp(self):
        """Set up  test environment, to be torn down when done"""
        await omni.usd.get_context().new_stage_async()
        self._timeline = omni.timeline.get_timeline_interface()
        # add franka robot for test
        assets_root_path = await get_assets_root_path_async()
        if assets_root_path is None:
            carb.log_error("Could not find Isaac Sim assets folder")
            return
        (result, error) = await open_stage_async(assets_root_path + "/Isaac/Robots/Franka/franka.usd")

    # ----------------------------------------------------------------------
    async def tearDown(self):
        """Get rid of temporary data used by the test"""
        await omni.kit.app.get_app().next_update_async()
        while omni.usd.get_context().get_stage_loading_status()[2] > 0:
            print("tearDown, assets still loading, waiting to finish...")
            await asyncio.sleep(1.0)
        await omni.kit.app.get_app().next_update_async()
        return

    # ----------------------------------------------------------------------
    async def test_joint_name_ogn(self):
        (test_graph, new_nodes, _, _) = og.Controller.edit(
            {"graph_path": "/ActionGraph", "evaluator_name": "execution"},
            {
                og.Controller.Keys.CREATE_NODES: [
                    ("OnPlaybackTick", "omni.graph.action.OnPlaybackTick"),
                    ("JointNameArray", "omni.graph.nodes.ConstructArray"),
                    ("JointCommandArray", "omni.graph.nodes.ConstructArray"),
                    ("ArticulationController", "omni.isaac.core_nodes.IsaacArticulationController"),
                ],
                og.Controller.Keys.SET_VALUES: [
                    ("JointNameArray.inputs:arraySize", 2),
                    ("JointNameArray.inputs:arrayType", "token[]"),
                    ("JointNameArray.inputs:input0", "panda_joint2"),
                    ("JointNameArray.inputs:input1", "panda_joint3"),
                    ("JointCommandArray.inputs:arraySize", 2),
                    ("JointCommandArray.inputs:arrayType", "double[]"),
                    ("JointCommandArray.inputs:input0", -1.0),
                    ("JointCommandArray.inputs:input1", 1.2),
                    ("ArticulationController.inputs:robotPath", "/panda"),
                ],
                og.Controller.Keys.CREATE_ATTRIBUTES: [
                    ("JointNameArray.inputs:input1", "token"),
                    ("JointCommandArray.inputs:input1", "double"),
                ],
                og.Controller.Keys.CONNECT: [
                    ("OnPlaybackTick.outputs:tick", "ArticulationController.inputs:execIn"),
                    ("JointNameArray.outputs:array", "ArticulationController.inputs:jointNames"),
                    ("JointCommandArray.outputs:array", "ArticulationController.inputs:positionCommand"),
                ],
            },
        )

        await og.Controller.evaluate(test_graph)

        # check where the joints are after evaluate
        robot = Robot(prim_path="/panda", name="franka")
        self._timeline.play()
        await simulate_async(2)
        robot.initialize()
        print(robot.get_joint_positions())

        self.assertAlmostEqual(robot.get_joint_positions()[1], -1.0, delta=0.001)
        self.assertAlmostEqual(robot.get_joint_positions()[2], 1.2, delta=0.001)

    # ----------------------------------------------------------------------
    async def test_joint_index_ogn(self):
        (test_graph, new_nodes, _, _) = og.Controller.edit(
            {"graph_path": "/ActionGraph", "evaluator_name": "execution"},
            {
                og.Controller.Keys.CREATE_NODES: [
                    ("OnPlaybackTick", "omni.graph.action.OnPlaybackTick"),
                    ("Joint1Index", "omni.graph.nodes.ConstantInt"),
                    ("Joint2Index", "omni.graph.nodes.ConstantInt"),
                    ("JointIndexArray", "omni.graph.nodes.MakeArray"),
                    ("Joint1Position", "omni.graph.nodes.ConstantDouble"),
                    ("Joint2Position", "omni.graph.nodes.ConstantDouble"),
                    ("JointCommandArray", "omni.graph.nodes.MakeArray"),
                    ("ArticulationController", "omni.isaac.core_nodes.IsaacArticulationController"),
                ],
                og.Controller.Keys.SET_VALUES: [
                    ("Joint1Index.inputs:value", 1),
                    ("Joint2Index.inputs:value", 2),
                    ("Joint1Position.inputs:value", -1.0),
                    ("Joint2Position.inputs:value", 1.2),
                    ("JointIndexArray.inputs:arraySize", 2),
                    ("JointCommandArray.inputs:arraySize", 2),
                    ("ArticulationController.inputs:robotPath", "/panda"),
                ],
                og.Controller.Keys.CONNECT: [
                    ("OnPlaybackTick.outputs:tick", "ArticulationController.inputs:execIn"),
                    ("Joint1Index.inputs:value", "JointIndexArray.inputs:a"),
                    ("Joint2Index.inputs:value", "JointIndexArray.inputs:b"),
                    ("JointIndexArray.outputs:array", "ArticulationController.inputs:jointIndices"),
                    ("Joint1Position.inputs:value", "JointCommandArray.inputs:a"),
                    ("Joint2Position.inputs:value", "JointCommandArray.inputs:b"),
                    ("JointCommandArray.outputs:array", "ArticulationController.inputs:positionCommand"),
                ],
            },
        )

        await og.Controller.evaluate(test_graph)

        # check where the joints are after evaluate
        robot = Robot(prim_path="/panda", name="franka")
        self._timeline.play()
        await simulate_async(2)
        robot.initialize()
        print(robot.get_joint_positions())

        self.assertAlmostEqual(robot.get_joint_positions()[1], -1.0, delta=0.001)
        self.assertAlmostEqual(robot.get_joint_positions()[2], 1.2, delta=0.001)

    # ----------------------------------------------------------------------
    async def test_full_array_no_index_ogn(self):
        (test_graph, new_nodes, _, _) = og.Controller.edit(
            {"graph_path": "/ActionGraph", "evaluator_name": "execution"},
            {
                og.Controller.Keys.CREATE_NODES: [
                    ("OnPlaybackTick", "omni.graph.action.OnPlaybackTick"),
                    ("Joint1Position", "omni.graph.nodes.ConstantDouble"),
                    ("Joint2Position", "omni.graph.nodes.ConstantDouble"),
                    ("JointCommandArray", "omni.graph.nodes.MakeArray"),
                    ("ArticulationController", "omni.isaac.core_nodes.IsaacArticulationController"),
                ],
                og.Controller.Keys.SET_VALUES: [
                    ("Joint1Position.inputs:value", -1.0),
                    ("Joint2Position.inputs:value", 1.2),
                    ("JointCommandArray.inputs:arraySize", 9),
                    ("ArticulationController.inputs:robotPath", "/panda"),
                ],
                og.Controller.Keys.CONNECT: [
                    ("OnPlaybackTick.outputs:tick", "ArticulationController.inputs:execIn"),
                    ("Joint1Position.inputs:value", "JointCommandArray.inputs:b"),
                    ("Joint2Position.inputs:value", "JointCommandArray.inputs:c"),
                    ("JointCommandArray.outputs:array", "ArticulationController.inputs:positionCommand"),
                ],
            },
        )

        await og.Controller.evaluate(test_graph)

        # check where the joints are after evaluate
        robot = Robot(prim_path="/panda", name="franka")
        self._timeline.play()
        await simulate_async(8)
        robot.initialize()
        print(robot.get_joint_positions())

        self.assertAlmostEqual(robot.get_joint_positions()[1], -1.0, delta=0.01)
        self.assertAlmostEqual(robot.get_joint_positions()[2], 1.2, delta=0.01)

    # ----------------------------------------------------------------------
    async def test_single_joint_name_ogn(self):
        (test_graph, new_nodes, _, _) = og.Controller.edit(
            {"graph_path": "/ActionGraph", "evaluator_name": "execution"},
            {
                og.Controller.Keys.CREATE_NODES: [
                    ("OnPlaybackTick", "omni.graph.action.OnPlaybackTick"),
                    ("Joint1Name", "omni.graph.nodes.ConstantToken"),
                    ("JointNameArray", "omni.graph.nodes.MakeArray"),
                    ("Joint1Position", "omni.graph.nodes.ConstantDouble"),
                    ("JointCommandArray", "omni.graph.nodes.MakeArray"),
                    ("ArticulationController", "omni.isaac.core_nodes.IsaacArticulationController"),
                ],
                og.Controller.Keys.SET_VALUES: [
                    ("Joint1Name.inputs:value", "panda_joint3"),
                    ("Joint1Position.inputs:value", 1.7),
                    ("JointNameArray.inputs:arraySize", 1),
                    ("JointCommandArray.inputs:arraySize", 1),
                    ("ArticulationController.inputs:robotPath", "/panda"),
                ],
                og.Controller.Keys.CONNECT: [
                    ("OnPlaybackTick.outputs:tick", "ArticulationController.inputs:execIn"),
                    ("Joint1Name.inputs:value", "JointNameArray.inputs:a"),
                    ("JointNameArray.outputs:array", "ArticulationController.inputs:jointNames"),
                    ("Joint1Position.inputs:value", "JointCommandArray.inputs:a"),
                    ("JointCommandArray.outputs:array", "ArticulationController.inputs:positionCommand"),
                ],
            },
        )

        await og.Controller.evaluate(test_graph)

        # check where the joints are after evaluate
        robot = Robot(prim_path="/panda", name="franka")
        self._timeline.play()
        await simulate_async(2)
        robot.initialize()
        print(robot.get_joint_positions())

        self.assertAlmostEqual(robot.get_joint_positions()[2], 1.7, delta=0.001)
        self.assertGreater(abs(robot.get_joint_positions()[3] - 1.7), 0.1)

    # ----------------------------------------------------------------------
    async def test_single_joint_index_ogn(self):
        (test_graph, new_nodes, _, _) = og.Controller.edit(
            {"graph_path": "/ActionGraph", "evaluator_name": "execution"},
            {
                og.Controller.Keys.CREATE_NODES: [
                    ("OnPlaybackTick", "omni.graph.action.OnPlaybackTick"),
                    ("Joint1Index", "omni.graph.nodes.ConstantInt"),
                    ("JointIndexArray", "omni.graph.nodes.MakeArray"),
                    ("Joint1Position", "omni.graph.nodes.ConstantDouble"),
                    ("JointCommandArray", "omni.graph.nodes.MakeArray"),
                    ("ArticulationController", "omni.isaac.core_nodes.IsaacArticulationController"),
                ],
                og.Controller.Keys.SET_VALUES: [
                    ("Joint1Index.inputs:value", 2),
                    ("Joint1Position.inputs:value", 1.7),
                    ("JointIndexArray.inputs:arraySize", 1),
                    ("JointCommandArray.inputs:arraySize", 1),
                    ("ArticulationController.inputs:robotPath", "/panda"),
                ],
                og.Controller.Keys.CONNECT: [
                    ("OnPlaybackTick.outputs:tick", "ArticulationController.inputs:execIn"),
                    ("Joint1Index.inputs:value", "JointIndexArray.inputs:a"),
                    ("JointIndexArray.outputs:array", "ArticulationController.inputs:jointIndices"),
                    ("Joint1Position.inputs:value", "JointCommandArray.inputs:a"),
                    ("JointCommandArray.outputs:array", "ArticulationController.inputs:positionCommand"),
                ],
            },
        )

        await og.Controller.evaluate(test_graph)

        # check where the joints are after evaluate
        robot = Robot(prim_path="/panda", name="franka")
        self._timeline.play()
        await simulate_async(2)
        robot.initialize()
        print(robot.get_joint_positions())

        self.assertAlmostEqual(robot.get_joint_positions()[2], 1.7, delta=0.001)
        self.assertGreater(abs(robot.get_joint_positions()[3] - 1.7), 0.1)
