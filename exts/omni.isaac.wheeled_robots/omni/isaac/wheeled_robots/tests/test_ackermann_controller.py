import asyncio
import sys

import carb
import numpy as np
import omni.graph.core as og
import omni.graph.core.tests as ogts
import omni.kit.test
import usdrt.Sdf
from omni.isaac.core import World
from omni.isaac.core.robots import Robot
from omni.isaac.core.utils.physics import simulate_async
from omni.isaac.core.utils.prims import delete_prim, get_prim_at_path
from omni.isaac.core.utils.stage import create_new_stage_async
from omni.isaac.nucleus import get_assets_root_path_async
from omni.isaac.wheeled_robots.controllers.ackermann_controller import AckermannController
from omni.isaac.wheeled_robots.robots import WheeledRobot


class TestAckermannController(omni.kit.test.AsyncTestCase):
    async def setUp(self):
        pass

    # ----------------------------------------------------------------------

    async def tearDown(self):
        pass

    # ----------------------------------------------------------------------

    async def test_ackermann_drive_velocity_control(self):
        wheel_base = 1.65
        track_width = 1.25
        turning_wheel_radius = 0.25

        controller = AckermannController("test_controller", wheel_base, track_width, turning_wheel_radius)

        steering_angle = 0.717
        speed = 2.0

        command = [steering_angle, speed]

        actions = controller.forward(command)

        expected_wheel_velocity = speed / turning_wheel_radius

        self.assertEquals(actions.joint_positions[0], None)
        self.assertEquals(actions.joint_positions[1], None)
        self.assertAlmostEquals(actions.joint_positions[2], 0.916, delta=0.001)
        self.assertAlmostEquals(actions.joint_positions[3], 0.580, delta=0.001)

        self.assertAlmostEquals(actions.joint_velocities[0], 8.0, delta=0.001)
        self.assertAlmostEquals(actions.joint_velocities[1], 8.0, delta=0.001)
        self.assertEquals(actions.joint_velocities[2], None)
        self.assertEquals(actions.joint_velocities[3], None)

    async def test_ackermann_drive_acceleration_control(self):
        wheel_base = 1.65
        track_width = 1.25
        turning_wheel_radius = 0.25

        controller = AckermannController(
            "test_controller", wheel_base, track_width, turning_wheel_radius, use_acceleration=True
        )

        steering_angle = 0.717
        speed = 2.0
        current_linear_velocity = [5.0, 0.0, 0.0]
        acceleration = 10
        delta_time = 0.01667

        command = [steering_angle, speed, current_linear_velocity[0], delta_time, acceleration]

        actions = controller.forward(command)

        expected_wheel_velocity = (current_linear_velocity[0] + (acceleration * delta_time)) / turning_wheel_radius

        self.assertEquals(actions.joint_positions[0], None)
        self.assertEquals(actions.joint_positions[1], None)
        self.assertAlmostEquals(actions.joint_positions[2], 0.916, delta=0.001)
        self.assertAlmostEquals(actions.joint_positions[3], 0.580, delta=0.001)

        self.assertAlmostEquals(actions.joint_velocities[0], expected_wheel_velocity, delta=0.001)
        self.assertAlmostEquals(actions.joint_velocities[1], expected_wheel_velocity, delta=0.001)
        self.assertEquals(actions.joint_velocities[2], None)
        self.assertEquals(actions.joint_velocities[3], None)


class TestAckermannControllerOgn(ogts.OmniGraphTestCase):
    async def setUp(self):
        """Set up  test environment, to be torn down when done"""

        await create_new_stage_async()
        await self.setup_environment()
        await self.setup_ogn()

        await self.my_world.initialize_simulation_context_async()

    # ----------------------------------------------------------------------

    async def setup_environment(self):
        self.my_world = World(stage_units_in_meters=1.0, physics_dt=1.0 / 60, rendering_dt=1.0 / 60)
        self.my_world.scene.add_default_ground_plane(z_position=-0.03)

        self._timeline = omni.timeline.get_timeline_interface()

        self._assets_root_path = await get_assets_root_path_async()
        if self._assets_root_path is None:
            carb.log_error("Could not find Isaac Sim assets folder")
            return

        self.stage = omni.usd.get_context().get_stage()
        await omni.kit.app.get_app().next_update_async()

    # ----------------------------------------------------------------------
    async def setup_ogn(self):
        self.graph_path = "/ActionGraph"
        self.prim_path = "/World/Forklift"

        if get_prim_at_path(self.graph_path):
            delete_prim(self.graph_path)

    # ----------------------------------------------------------------------

    async def tearDown(self):
        """Get rid of temporary data used by the test"""
        await omni.kit.stage_templates.new_stage_async()
        self._timeline = None

    # ----------------------------------------------------------------------

    async def test_ackermann_controller_node_reset(self):
        (test_acker_graph, [acker_node, play_node], _, _) = og.Controller.edit(
            {"graph_path": "/ActionGraph", "evaluator_name": "execution"},
            {
                og.Controller.Keys.CREATE_NODES: [
                    ("AckermannController", "omni.isaac.wheeled_robots.AckermannController"),
                    ("OnPlaybackTick", "omni.graph.action.OnPlaybackTick"),
                ],
                og.Controller.Keys.CONNECT: [
                    ("OnPlaybackTick.outputs:tick", "AckermannController.inputs:execIn"),
                ],
                og.Controller.Keys.SET_VALUES: [
                    ("AckermannController.inputs:wheelBase", 1.65),
                    ("AckermannController.inputs:trackWidth", 1.25),
                    ("AckermannController.inputs:turningWheelRadius", 0.25),
                    ("AckermannController.inputs:steeringAngle", 0.717),
                    ("AckermannController.inputs:speed", 2.0),
                ],
            },
        )

        await og.Controller.evaluate(test_acker_graph)

        self.my_world.play()
        await omni.kit.app.get_app().next_update_async()

        self.assertAlmostEqual(
            og.Controller(og.Controller.attribute("outputs:leftWheelAngle", acker_node)).get(), 0.916, delta=0.001
        )
        self.assertAlmostEqual(
            og.Controller(og.Controller.attribute("outputs:rightWheelAngle", acker_node)).get(), 0.580, delta=0.001
        )
        self.assertAlmostEqual(
            og.Controller(og.Controller.attribute("outputs:wheelRotationVelocity", acker_node)).get(),
            8.0,
            delta=0.001,
        )

        self.my_world.stop()
        await omni.kit.app.get_app().next_update_async()

        self.assertEqual(og.Controller(og.Controller.attribute("outputs:leftWheelAngle", acker_node)).get(), 0.0)
        self.assertEqual(og.Controller(og.Controller.attribute("outputs:rightWheelAngle", acker_node)).get(), 0.0)
        self.assertEqual(og.Controller(og.Controller.attribute("outputs:wheelRotationVelocity", acker_node)).get(), 0.0)

    # ----------------------------------------------------------------------

    async def test_ackermann_controller_acceleration_enabled(self):
        self._forklift = self.my_world.scene.add(
            WheeledRobot(
                prim_path="/World/Forklift",
                name="forklift",
                wheel_dof_names=[
                    "left_back_wheel_joint",
                    "right_back_wheel_joint",
                    "left_rotator_joint",
                    "right_rotator_joint",
                ],
                create_robot=True,
                usd_path=self._assets_root_path + "/Isaac/Robots/Forklift/forklift_c.usd",
            )
        )
        # ensuring correct calculations when acceleration is disabled
        (test_acker_graph, [acker_node, play_node, compute_odom, _, _, _], _, _) = og.Controller.edit(
            {"graph_path": "/ActionGraph", "evaluator_name": "execution"},
            {
                og.Controller.Keys.CREATE_NODES: [
                    ("AckermannController", "omni.isaac.wheeled_robots.AckermannController"),
                    ("OnPlaybackTick", "omni.graph.action.OnPlaybackTick"),
                    ("ComputeOdom", "omni.isaac.core_nodes.IsaacComputeOdometry"),
                    ("ArticulationController", "omni.isaac.core_nodes.IsaacArticulationController"),
                    ("WheelAnglesArray", "omni.graph.nodes.ConstructArray"),
                    ("WheelVelocityArray", "omni.graph.nodes.ConstructArray"),
                ],
                og.Controller.Keys.SET_VALUES: [
                    ("AckermannController.inputs:wheelBase", 1.65),
                    ("AckermannController.inputs:trackWidth", 1.25),
                    ("AckermannController.inputs:turningWheelRadius", 0.25),
                    ("AckermannController.inputs:steeringAngle", 0.717),
                    ("AckermannController.inputs:useAcceleration", True),
                    ("AckermannController.inputs:acceleration", 5.0),
                    ("ComputeOdom.inputs:chassisPrim", [usdrt.Sdf.Path("/World/Forklift")]),
                    ("ArticulationController.inputs:robotPath", "/World/Forklift"),
                    (
                        "ArticulationController.inputs:jointNames",
                        [
                            "left_back_wheel_joint",
                            "right_back_wheel_joint",
                            "left_rotator_joint",
                            "right_rotator_joint",
                        ],
                    ),
                    ("WheelAnglesArray.inputs:arraySize", 4),
                    ("WheelAnglesArray.inputs:arrayType", "double[]"),
                    ("WheelVelocityArray.inputs:arraySize", 4),
                    ("WheelVelocityArray.inputs:arrayType", "double[]"),
                ],
                og.Controller.Keys.CREATE_ATTRIBUTES: [
                    ("WheelAnglesArray.inputs:input1", "double"),
                    ("WheelVelocityArray.inputs:input1", "double"),
                    ("WheelAnglesArray.inputs:input2", "double"),
                    ("WheelVelocityArray.inputs:input2", "double"),
                    ("WheelAnglesArray.inputs:input3", "double"),
                    ("WheelVelocityArray.inputs:input3", "double"),
                ],
                og.Controller.Keys.CONNECT: [
                    ("OnPlaybackTick.outputs:tick", "AckermannController.inputs:execIn"),
                    ("OnPlaybackTick.outputs:tick", "ComputeOdom.inputs:execIn"),
                    ("OnPlaybackTick.outputs:deltaSeconds", "AckermannController.inputs:DT"),
                    ("ComputeOdom.outputs:linearVelocity", "AckermannController.inputs:currentLinearVelocity"),
                    ("AckermannController.outputs:leftWheelAngle", "WheelAnglesArray.inputs:input2"),
                    ("AckermannController.outputs:rightWheelAngle", "WheelAnglesArray.inputs:input3"),
                    ("WheelVelocityArray.outputs:array", "ArticulationController.inputs:velocityCommand"),
                    ("WheelAnglesArray.outputs:array", "ArticulationController.inputs:positionCommand"),
                    ("AckermannController.outputs:wheelRotationVelocity", "WheelVelocityArray.inputs:input0"),
                    ("AckermannController.outputs:wheelRotationVelocity", "WheelVelocityArray.inputs:input1"),
                    ("WheelVelocityArray.outputs:array", "ArticulationController.inputs:velocityCommand"),
                    ("AckermannController.outputs:execOut", "ArticulationController.inputs:execIn"),
                ],
            },
        )

        self.my_world.play()

        # run 10 frames
        for i in range(10):
            await omni.kit.app.get_app().next_update_async()

        self.assertAlmostEqual(
            og.Controller(og.Controller.attribute("outputs:leftWheelAngle", acker_node)).get(), 0.916, delta=0.001
        )
        self.assertAlmostEqual(
            og.Controller(og.Controller.attribute("outputs:rightWheelAngle", acker_node)).get(), 0.580, delta=0.001
        )
        self.assertAlmostEqual(
            og.Controller(og.Controller.attribute("outputs:wheelRotationVelocity", acker_node)).get(),
            0.5,
            delta=0.1,
        )

    # ----------------------------------------------------------------------

    async def test_ackermann_controller_wheel_angle(self):
        # test if left wheel is turning at larger angle than right wheel, check if angles make sense
        (test_acker_graph, [acker_node, play_node], _, _) = og.Controller.edit(
            {"graph_path": "/ActionGraph", "evaluator_name": "execution"},
            {
                og.Controller.Keys.CREATE_NODES: [
                    ("AckermannController", "omni.isaac.wheeled_robots.AckermannController"),
                    ("OnPlaybackTick", "omni.graph.action.OnPlaybackTick"),
                ],
                og.Controller.Keys.SET_VALUES: [
                    ("AckermannController.inputs:wheelBase", 1.65),
                    ("AckermannController.inputs:trackWidth", 1.25),
                    ("AckermannController.inputs:turningWheelRadius", 0.25),
                    ("AckermannController.inputs:steeringAngle", 0.717),
                    ("AckermannController.inputs:speed", 2.0),
                    ("AckermannController.inputs:invertSteeringAngle", True),
                ],
                og.Controller.Keys.CONNECT: [
                    ("OnPlaybackTick.outputs:tick", "AckermannController.inputs:execIn"),
                ],
            },
        )

        self.my_world.play()
        await omni.kit.app.get_app().next_update_async()

        left_wheel = og.Controller.attribute("outputs:leftWheelAngle", acker_node).get()
        self.assertAlmostEqual(left_wheel, -0.580, delta=0.001)

        right_wheel = og.Controller.attribute("outputs:rightWheelAngle", acker_node).get()
        self.assertAlmostEqual(right_wheel, -0.916, delta=0.001)
        self.assertAlmostEqual(
            og.Controller(og.Controller.attribute("outputs:wheelRotationVelocity", acker_node)).get(),
            8.0,
            delta=0.001,
        )
        # left wheel angle must be greater than right wheel angle
        self.assertTrue(left_wheel > right_wheel)

    # ----------------------------------------------------------------------

    async def test_ackermann_controller_robot(self):
        # create forklift class
        self._forklift = self.my_world.scene.add(
            WheeledRobot(
                prim_path="/World/Forklift",
                name="forklift",
                wheel_dof_names=[
                    "left_back_wheel_joint",
                    "right_back_wheel_joint",
                    "left_rotator_joint",
                    "right_rotator_joint",
                ],
                create_robot=True,
                usd_path=self._assets_root_path + "/Isaac/Robots/Forklift/forklift_c.usd",
            )
        )
        self._timeline = omni.timeline.get_timeline_interface()
        (test_acker_graph, [acker_node, _, _, _, play_node], _, _) = og.Controller.edit(
            {"graph_path": "/ActionGraph", "evaluator_name": "execution"},
            {
                og.Controller.Keys.CREATE_NODES: [
                    ("AckermannController", "omni.isaac.wheeled_robots.AckermannController"),
                    ("WheelAnglesArray", "omni.graph.nodes.ConstructArray"),
                    ("WheelVelocityArray", "omni.graph.nodes.ConstructArray"),
                    ("ArticulationController", "omni.isaac.core_nodes.IsaacArticulationController"),
                    ("OnPlaybackTick", "omni.graph.action.OnPlaybackTick"),
                ],
                og.Controller.Keys.CONNECT: [
                    ("OnPlaybackTick.outputs:tick", "AckermannController.inputs:execIn"),
                    ("OnPlaybackTick.outputs:deltaSeconds", "AckermannController.inputs:DT"),
                    ("AckermannController.outputs:execOut", "ArticulationController.inputs:execIn"),
                    ("AckermannController.outputs:leftWheelAngle", "WheelAnglesArray.inputs:input2"),
                    ("AckermannController.outputs:rightWheelAngle", "WheelAnglesArray.inputs:input3"),
                    ("WheelAnglesArray.outputs:array", "ArticulationController.inputs:positionCommand"),
                    ("AckermannController.outputs:wheelRotationVelocity", "WheelVelocityArray.inputs:input0"),
                    ("AckermannController.outputs:wheelRotationVelocity", "WheelVelocityArray.inputs:input1"),
                    ("WheelVelocityArray.outputs:array", "ArticulationController.inputs:velocityCommand"),
                ],
                og.Controller.Keys.CREATE_ATTRIBUTES: [
                    ("WheelAnglesArray.inputs:input1", "double"),
                    ("WheelVelocityArray.inputs:input1", "double"),
                    ("WheelAnglesArray.inputs:input2", "double"),
                    ("WheelVelocityArray.inputs:input2", "double"),
                    ("WheelAnglesArray.inputs:input3", "double"),
                    ("WheelVelocityArray.inputs:input3", "double"),
                ],
                og.Controller.Keys.SET_VALUES: [
                    ("WheelAnglesArray.inputs:arraySize", 4),
                    ("WheelAnglesArray.inputs:arrayType", "double[]"),
                    ("WheelVelocityArray.inputs:arraySize", 4),
                    ("WheelVelocityArray.inputs:arrayType", "double[]"),
                    ("AckermannController.inputs:wheelBase", 1.65),
                    ("AckermannController.inputs:trackWidth", 1.25),
                    ("AckermannController.inputs:turningWheelRadius", 0.25),
                    ("AckermannController.inputs:steeringAngle", 0.500),
                    ("AckermannController.inputs:speed", 1.5),
                    ("ArticulationController.inputs:robotPath", "/World/Forklift"),
                    (
                        "ArticulationController.inputs:jointNames",
                        [
                            "left_back_wheel_joint",
                            "right_back_wheel_joint",
                            "left_rotator_joint",
                            "right_rotator_joint",
                        ],
                    ),
                ],
            },
        )

        robot = Robot(prim_path="/World/Forklift", name="Forklift")
        self._timeline.play()
        await omni.kit.app.get_app().next_update_async()
        robot.initialize()
        await simulate_async(1)

        # get current velocities of left and right back wheels; ensure they are non-zero, and close to the intended wheel velocity
        wheel_velocity = og.Controller.attribute("outputs:wheelRotationVelocity", acker_node).get()

        joint_vel = robot.get_joint_velocities()
        self.assertNotEqual(joint_vel[5], 0)
        self.assertNotEqual(joint_vel[6], 0)
        self.assertAlmostEqual(joint_vel[5], wheel_velocity, delta=0.5)
        self.assertAlmostEqual(joint_vel[6], wheel_velocity, delta=0.5)

        # get current left and right back wheel angles; ensure they are non-zero and the left wheel has a large angle than the right wheel
        joint_angle = robot.get_joint_positions()
        self.assertNotEqual(joint_angle[2], 0)
        self.assertNotEqual(joint_angle[3], 0)
        self.assertTrue(joint_angle[2] > joint_angle[3])

        # get angular velocity and robot and ensure it is greater than zero
        joint_angular = robot.get_angular_velocity()
        joint_angular[2] = abs(joint_angular[2])
        self.assertTrue(joint_angular[2] > 0.05)

    # ----------------------------------------------------------------------

    async def test_ackermann_controller_motionless_robot(self):
        # create forklift class
        self._forklift = self.my_world.scene.add(
            WheeledRobot(
                prim_path="/World/Forklift",
                name="forklift",
                wheel_dof_names=[
                    "left_back_wheel_joint",
                    "right_back_wheel_joint",
                    "left_rotator_joint",
                    "right_rotator_joint",
                ],
                create_robot=True,
                usd_path=self._assets_root_path + "/Isaac/Robots/Forklift/forklift_c.usd",
            )
        )
        self._timeline = omni.timeline.get_timeline_interface()
        (test_acker_graph, [acker_node, _, _, _, play_node], _, _) = og.Controller.edit(
            {"graph_path": "/ActionGraph", "evaluator_name": "execution"},
            {
                og.Controller.Keys.CREATE_NODES: [
                    ("AckermannController", "omni.isaac.wheeled_robots.AckermannController"),
                    ("WheelAnglesArray", "omni.graph.nodes.ConstructArray"),
                    ("WheelVelocityArray", "omni.graph.nodes.ConstructArray"),
                    ("ArticulationController", "omni.isaac.core_nodes.IsaacArticulationController"),
                    ("OnPlaybackTick", "omni.graph.action.OnPlaybackTick"),
                ],
                og.Controller.Keys.SET_VALUES: [
                    ("WheelAnglesArray.inputs:arraySize", 4),
                    ("WheelAnglesArray.inputs:arrayType", "double[]"),
                    ("WheelVelocityArray.inputs:arraySize", 4),
                    ("WheelVelocityArray.inputs:arrayType", "double[]"),
                    ("AckermannController.inputs:wheelBase", 1.65),
                    ("AckermannController.inputs:trackWidth", 1.25),
                    ("AckermannController.inputs:turningWheelRadius", 0.25),
                    ("AckermannController.inputs:steeringAngle", 0.0),
                    ("AckermannController.inputs:speed", 0.0),
                    ("ArticulationController.inputs:robotPath", "/World/Forklift"),
                    (
                        "ArticulationController.inputs:jointNames",
                        [
                            "left_back_wheel_joint",
                            "right_back_wheel_joint",
                            "left_rotator_joint",
                            "right_rotator_joint",
                        ],
                    ),
                ],
                og.Controller.Keys.CREATE_ATTRIBUTES: [
                    ("WheelAnglesArray.inputs:input1", "double"),
                    ("WheelVelocityArray.inputs:input1", "double"),
                    ("WheelAnglesArray.inputs:input2", "double"),
                    ("WheelVelocityArray.inputs:input2", "double"),
                    ("WheelAnglesArray.inputs:input3", "double"),
                    ("WheelVelocityArray.inputs:input3", "double"),
                ],
                og.Controller.Keys.CONNECT: [
                    ("OnPlaybackTick.outputs:tick", "AckermannController.inputs:execIn"),
                    ("OnPlaybackTick.outputs:deltaSeconds", "AckermannController.inputs:DT"),
                    ("AckermannController.outputs:execOut", "ArticulationController.inputs:execIn"),
                    ("AckermannController.outputs:leftWheelAngle", "WheelAnglesArray.inputs:input2"),
                    ("AckermannController.outputs:rightWheelAngle", "WheelAnglesArray.inputs:input3"),
                    ("WheelAnglesArray.outputs:array", "ArticulationController.inputs:positionCommand"),
                    ("AckermannController.outputs:wheelRotationVelocity", "WheelVelocityArray.inputs:input0"),
                    ("AckermannController.outputs:wheelRotationVelocity", "WheelVelocityArray.inputs:input1"),
                    ("WheelVelocityArray.outputs:array", "ArticulationController.inputs:velocityCommand"),
                ],
            },
        )

        robot = Robot(prim_path="/World/Forklift", name="Forklift")
        self._timeline.play()
        await omni.kit.app.get_app().next_update_async()
        robot.initialize()
        await omni.kit.app.get_app().next_update_async()

        # ensure wheel velocity, angles, and angular velocity equal none when robot is not in motion
        joint_vel_2 = robot.get_joint_velocities()
        self.assertAlmostEqual(joint_vel_2[5], 0.0, delta=1e-5)
        self.assertAlmostEqual(joint_vel_2[6], 0.0, delta=1e-5)

        joint_angle_2 = robot.get_joint_positions()
        self.assertAlmostEqual(joint_angle_2[2], 0.0, delta=1e-5)
        self.assertAlmostEqual(joint_angle_2[3], 0.0, delta=1e-5)

        joint_angular_2 = robot.get_angular_velocity()
        self.assertAlmostEqual(joint_angular_2[2], 0.0, delta=1e-5)
