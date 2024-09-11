# Copyright (c) 2018-2023, NVIDIA CORPORATION. All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto. Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.
#


import asyncio

import carb
import numpy as np
import omni.graph.core as og
import omni.graph.core.tests as ogts
import omni.kit.test
from omni.isaac.core.robots import Robot
from omni.isaac.core.utils.physics import simulate_async
from omni.isaac.core.utils.stage import open_stage_async
from omni.isaac.nucleus import get_assets_root_path_async


class TestArticulationStateNode(ogts.OmniGraphTestCase):
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
                    ("ArticulationState", "omni.isaac.core_nodes.IsaacArticulationState"),
                ],
                og.Controller.Keys.SET_VALUES: [
                    ("JointNameArray.inputs:arraySize", 2),
                    ("JointNameArray.inputs:arrayType", "token[]"),
                    ("JointNameArray.inputs:input0", "panda_joint2"),
                    ("JointNameArray.inputs:input1", "panda_joint3"),
                    ("ArticulationState.inputs:robotPath", "/panda"),
                ],
                og.Controller.Keys.CREATE_ATTRIBUTES: [
                    ("JointNameArray.inputs:input1", "token"),
                ],
                og.Controller.Keys.CONNECT: [
                    ("OnPlaybackTick.outputs:tick", "ArticulationState.inputs:execIn"),
                    ("JointNameArray.outputs:array", "ArticulationState.inputs:jointNames"),
                ],
            },
        )

        # set a specific robot state
        robot = Robot(prim_path="/panda", name="franka")
        self._timeline.play()
        await simulate_async(1)
        robot.initialize()
        robot.set_joint_positions([-1.0, 1.2], joint_indices=[1, 2])
        await omni.kit.app.get_app().next_update_async()
        await omni.kit.app.get_app().next_update_async()

        # check where the joints are after evaluate
        await og.Controller.evaluate(test_graph)
        articulation_state_node = new_nodes[-1]
        joint_names = og.Controller.attribute("outputs:jointNames", articulation_state_node).get()
        joint_positions = og.Controller.attribute("outputs:jointPositions", articulation_state_node).get()
        joint_velocities = og.Controller.attribute("outputs:jointVelocities", articulation_state_node).get()
        joint_efforts = og.Controller.attribute("outputs:measuredJointEfforts", articulation_state_node).get()
        joint_forces = og.Controller.attribute("outputs:measuredJointForces", articulation_state_node).get()
        joint_torques = og.Controller.attribute("outputs:measuredJointTorques", articulation_state_node).get()
        print(joint_names)
        print(joint_positions)
        print(joint_velocities)
        print(joint_efforts)
        print(joint_forces)
        print(joint_torques)

        robot_joint_positions = robot.get_joint_positions()
        robot_joint_velocities = robot.get_joint_velocities()
        robot_measured_joint_efforts = robot.get_measured_joint_efforts()

        self.assertEqual(len(joint_names), 2)
        self.assertEqual(len(joint_positions), 2)
        self.assertEqual(len(joint_velocities), 2)
        self.assertEqual(len(joint_efforts), 2)
        self.assertEqual(len(joint_forces), 2)
        self.assertEqual(len(joint_torques), 2)
        assert_array_almost_equal = np.testing.assert_array_almost_equal
        self.assertIsNone(assert_array_almost_equal(joint_positions, robot_joint_positions[[1, 2]], decimal=3))
        self.assertIsNone(assert_array_almost_equal(joint_velocities, robot_joint_velocities[[1, 2]], decimal=3))
        self.assertIsNone(assert_array_almost_equal(joint_efforts, robot_measured_joint_efforts[[1, 2]], decimal=3))

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
                    ("ArticulationState", "omni.isaac.core_nodes.IsaacArticulationState"),
                ],
                og.Controller.Keys.SET_VALUES: [
                    ("Joint1Index.inputs:value", 1),
                    ("Joint2Index.inputs:value", 2),
                    ("JointIndexArray.inputs:arraySize", 2),
                    ("ArticulationState.inputs:robotPath", "/panda"),
                ],
                og.Controller.Keys.CONNECT: [
                    ("OnPlaybackTick.outputs:tick", "ArticulationState.inputs:execIn"),
                    ("Joint1Index.inputs:value", "JointIndexArray.inputs:a"),
                    ("Joint2Index.inputs:value", "JointIndexArray.inputs:b"),
                    ("JointIndexArray.outputs:array", "ArticulationState.inputs:jointIndices"),
                ],
            },
        )

        # set a specific robot state
        robot = Robot(prim_path="/panda", name="franka")
        self._timeline.play()
        await simulate_async(1)
        robot.initialize()
        robot.set_joint_positions([-1.0, 1.2], joint_indices=[1, 2])
        await omni.kit.app.get_app().next_update_async()
        await omni.kit.app.get_app().next_update_async()

        # check where the joints are after evaluate
        await og.Controller.evaluate(test_graph)
        articulation_state_node = new_nodes[-1]
        joint_names = og.Controller.attribute("outputs:jointNames", articulation_state_node).get()
        joint_positions = og.Controller.attribute("outputs:jointPositions", articulation_state_node).get()
        joint_velocities = og.Controller.attribute("outputs:jointVelocities", articulation_state_node).get()
        joint_efforts = og.Controller.attribute("outputs:measuredJointEfforts", articulation_state_node).get()
        joint_forces = og.Controller.attribute("outputs:measuredJointForces", articulation_state_node).get()
        joint_torques = og.Controller.attribute("outputs:measuredJointTorques", articulation_state_node).get()
        print(joint_names)
        print(joint_positions)
        print(joint_velocities)
        print(joint_efforts)
        print(joint_forces)
        print(joint_torques)

        robot_joint_positions = robot.get_joint_positions()
        robot_joint_velocities = robot.get_joint_velocities()
        robot_measured_joint_efforts = robot.get_measured_joint_efforts()

        self.assertEqual(len(joint_names), 2)
        self.assertEqual(len(joint_positions), 2)
        self.assertEqual(len(joint_velocities), 2)
        self.assertEqual(len(joint_efforts), 2)
        self.assertEqual(len(joint_forces), 2)
        self.assertEqual(len(joint_torques), 2)
        assert_array_almost_equal = np.testing.assert_array_almost_equal
        self.assertIsNone(assert_array_almost_equal(joint_positions, robot_joint_positions[[1, 2]], decimal=3))
        self.assertIsNone(assert_array_almost_equal(joint_velocities, robot_joint_velocities[[1, 2]], decimal=3))
        self.assertIsNone(assert_array_almost_equal(joint_efforts, robot_measured_joint_efforts[[1, 2]], decimal=3))

    # ----------------------------------------------------------------------
    async def test_full_array_no_index_ogn(self):
        (test_graph, new_nodes, _, _) = og.Controller.edit(
            {"graph_path": "/ActionGraph", "evaluator_name": "execution"},
            {
                og.Controller.Keys.CREATE_NODES: [
                    ("OnPlaybackTick", "omni.graph.action.OnPlaybackTick"),
                    ("ArticulationState", "omni.isaac.core_nodes.IsaacArticulationState"),
                ],
                og.Controller.Keys.SET_VALUES: [
                    ("ArticulationState.inputs:robotPath", "/panda"),
                ],
                og.Controller.Keys.CONNECT: [
                    ("OnPlaybackTick.outputs:tick", "ArticulationState.inputs:execIn"),
                ],
            },
        )

        # set a specific robot state
        robot = Robot(prim_path="/panda", name="franka")
        self._timeline.play()
        await simulate_async(1)
        robot.initialize()
        robot.set_joint_positions([-1.0, 1.2], joint_indices=[1, 2])
        await omni.kit.app.get_app().next_update_async()
        await omni.kit.app.get_app().next_update_async()

        # check where the joints are after evaluate
        await og.Controller.evaluate(test_graph)
        articulation_state_node = new_nodes[-1]
        joint_names = og.Controller.attribute("outputs:jointNames", articulation_state_node).get()
        joint_positions = og.Controller.attribute("outputs:jointPositions", articulation_state_node).get()
        joint_velocities = og.Controller.attribute("outputs:jointVelocities", articulation_state_node).get()
        joint_efforts = og.Controller.attribute("outputs:measuredJointEfforts", articulation_state_node).get()
        joint_forces = og.Controller.attribute("outputs:measuredJointForces", articulation_state_node).get()
        joint_torques = og.Controller.attribute("outputs:measuredJointTorques", articulation_state_node).get()
        print(joint_names)
        print(joint_positions)
        print(joint_velocities)
        print(joint_efforts)
        print(joint_forces)
        print(joint_torques)

        robot_joint_positions = robot.get_joint_positions()
        robot_joint_velocities = robot.get_joint_velocities()
        robot_measured_joint_efforts = robot.get_measured_joint_efforts()

        self.assertEqual(len(joint_names), 9)
        self.assertEqual(len(joint_positions), 9)
        self.assertEqual(len(joint_velocities), 9)
        self.assertEqual(len(joint_efforts), 9)
        self.assertEqual(len(joint_forces), 9)
        self.assertEqual(len(joint_torques), 9)
        assert_array_almost_equal = np.testing.assert_array_almost_equal
        self.assertIsNone(assert_array_almost_equal(joint_positions, robot_joint_positions, decimal=3))
        self.assertIsNone(assert_array_almost_equal(joint_velocities, robot_joint_velocities, decimal=3))
        self.assertIsNone(assert_array_almost_equal(joint_efforts, robot_measured_joint_efforts, decimal=3))

    # ----------------------------------------------------------------------
    async def test_single_joint_name_ogn(self):
        (test_graph, new_nodes, _, _) = og.Controller.edit(
            {"graph_path": "/ActionGraph", "evaluator_name": "execution"},
            {
                og.Controller.Keys.CREATE_NODES: [
                    ("OnPlaybackTick", "omni.graph.action.OnPlaybackTick"),
                    ("Joint1Name", "omni.graph.nodes.ConstantToken"),
                    ("JointNameArray", "omni.graph.nodes.MakeArray"),
                    ("ArticulationState", "omni.isaac.core_nodes.IsaacArticulationState"),
                ],
                og.Controller.Keys.SET_VALUES: [
                    ("Joint1Name.inputs:value", "panda_joint3"),
                    ("JointNameArray.inputs:arraySize", 1),
                    ("ArticulationState.inputs:robotPath", "/panda"),
                ],
                og.Controller.Keys.CONNECT: [
                    ("OnPlaybackTick.outputs:tick", "ArticulationState.inputs:execIn"),
                    ("Joint1Name.inputs:value", "JointNameArray.inputs:a"),
                    ("JointNameArray.outputs:array", "ArticulationState.inputs:jointNames"),
                ],
            },
        )

        # set a specific robot state
        robot = Robot(prim_path="/panda", name="franka")
        self._timeline.play()
        await simulate_async(1)
        robot.initialize()
        robot.set_joint_positions([-1.0], joint_indices=[2])
        await omni.kit.app.get_app().next_update_async()
        await omni.kit.app.get_app().next_update_async()

        # check where the joints are after evaluate
        await og.Controller.evaluate(test_graph)
        articulation_state_node = new_nodes[-1]
        joint_names = og.Controller.attribute("outputs:jointNames", articulation_state_node).get()
        joint_positions = og.Controller.attribute("outputs:jointPositions", articulation_state_node).get()
        joint_velocities = og.Controller.attribute("outputs:jointVelocities", articulation_state_node).get()
        joint_efforts = og.Controller.attribute("outputs:measuredJointEfforts", articulation_state_node).get()
        joint_forces = og.Controller.attribute("outputs:measuredJointForces", articulation_state_node).get()
        joint_torques = og.Controller.attribute("outputs:measuredJointTorques", articulation_state_node).get()
        print(joint_names)
        print(joint_positions)
        print(joint_velocities)
        print(joint_efforts)
        print(joint_forces)
        print(joint_torques)

        robot_joint_positions = robot.get_joint_positions()
        robot_joint_velocities = robot.get_joint_velocities()
        robot_measured_joint_efforts = robot.get_measured_joint_efforts()

        self.assertEqual(len(joint_names), 1)
        self.assertEqual(len(joint_positions), 1)
        self.assertEqual(len(joint_velocities), 1)
        self.assertEqual(len(joint_efforts), 1)
        self.assertEqual(len(joint_forces), 1)
        self.assertEqual(len(joint_torques), 1)
        assert_array_almost_equal = np.testing.assert_array_almost_equal
        self.assertIsNone(assert_array_almost_equal(joint_positions, robot_joint_positions[[2]], decimal=3))
        self.assertIsNone(assert_array_almost_equal(joint_velocities, robot_joint_velocities[[2]], decimal=3))
        self.assertIsNone(assert_array_almost_equal(joint_efforts, robot_measured_joint_efforts[[2]], decimal=3))

    # ----------------------------------------------------------------------
    async def test_single_joint_index_ogn(self):
        (test_graph, new_nodes, _, _) = og.Controller.edit(
            {"graph_path": "/ActionGraph", "evaluator_name": "execution"},
            {
                og.Controller.Keys.CREATE_NODES: [
                    ("OnPlaybackTick", "omni.graph.action.OnPlaybackTick"),
                    ("Joint1Index", "omni.graph.nodes.ConstantInt"),
                    ("JointIndexArray", "omni.graph.nodes.MakeArray"),
                    ("ArticulationState", "omni.isaac.core_nodes.IsaacArticulationState"),
                ],
                og.Controller.Keys.SET_VALUES: [
                    ("Joint1Index.inputs:value", 2),
                    ("JointIndexArray.inputs:arraySize", 1),
                    ("ArticulationState.inputs:robotPath", "/panda"),
                ],
                og.Controller.Keys.CONNECT: [
                    ("OnPlaybackTick.outputs:tick", "ArticulationState.inputs:execIn"),
                    ("Joint1Index.inputs:value", "JointIndexArray.inputs:a"),
                    ("JointIndexArray.outputs:array", "ArticulationState.inputs:jointIndices"),
                ],
            },
        )

        # set a specific robot state
        robot = Robot(prim_path="/panda", name="franka")
        self._timeline.play()
        await simulate_async(1)
        robot.initialize()
        robot.set_joint_positions([-1.0], joint_indices=[2])
        await omni.kit.app.get_app().next_update_async()
        await omni.kit.app.get_app().next_update_async()

        # check where the joints are after evaluate
        await og.Controller.evaluate(test_graph)
        articulation_state_node = new_nodes[-1]
        joint_names = og.Controller.attribute("outputs:jointNames", articulation_state_node).get()
        joint_positions = og.Controller.attribute("outputs:jointPositions", articulation_state_node).get()
        joint_velocities = og.Controller.attribute("outputs:jointVelocities", articulation_state_node).get()
        joint_efforts = og.Controller.attribute("outputs:measuredJointEfforts", articulation_state_node).get()
        joint_forces = og.Controller.attribute("outputs:measuredJointForces", articulation_state_node).get()
        joint_torques = og.Controller.attribute("outputs:measuredJointTorques", articulation_state_node).get()
        print(joint_names)
        print(joint_positions)
        print(joint_velocities)
        print(joint_efforts)
        print(joint_forces)
        print(joint_torques)

        robot_joint_positions = robot.get_joint_positions()
        robot_joint_velocities = robot.get_joint_velocities()
        robot_measured_joint_efforts = robot.get_measured_joint_efforts()

        self.assertEqual(len(joint_names), 1)
        self.assertEqual(len(joint_positions), 1)
        self.assertEqual(len(joint_velocities), 1)
        self.assertEqual(len(joint_efforts), 1)
        self.assertEqual(len(joint_forces), 1)
        self.assertEqual(len(joint_torques), 1)
        assert_array_almost_equal = np.testing.assert_array_almost_equal
        self.assertIsNone(assert_array_almost_equal(joint_positions, robot_joint_positions[[2]], decimal=3))
        self.assertIsNone(assert_array_almost_equal(joint_velocities, robot_joint_velocities[[2]], decimal=3))
        self.assertIsNone(assert_array_almost_equal(joint_efforts, robot_measured_joint_efforts[[2]], decimal=3))
