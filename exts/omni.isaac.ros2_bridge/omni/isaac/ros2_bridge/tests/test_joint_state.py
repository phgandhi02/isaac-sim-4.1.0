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

import carb
import omni.graph.core as og

# Import extension python module we are testing with absolute import path, as if we are external user (other extension)
import omni.kit.commands

# NOTE:
#   omni.kit.test - std python's unittest module with additional wrapping to add suport for async/await tests
#   For most things refer to unittest docs: https://docs.python.org/3/library/unittest.html
import omni.kit.test
import omni.kit.usd
import usdrt.Sdf
from numpy import pi as PI
from omni.isaac.core.articulations import Articulation
from omni.isaac.core.utils.physics import simulate_async
from omni.isaac.core.utils.stage import open_stage_async
from omni.isaac.nucleus import get_assets_root_path_async

from .common import get_qos_profile, set_joint_drive_parameters


class TestRos2JointStatePublisher(omni.kit.test.AsyncTestCase):
    # Before running each test
    async def setUp(self):
        import rclpy

        await omni.usd.get_context().new_stage_async()
        self._timeline = omni.timeline.get_timeline_interface()

        ext_manager = omni.kit.app.get_app().get_extension_manager()
        ext_id = ext_manager.get_enabled_extension_id("omni.isaac.ros2_bridge")
        self._ros_extension_path = ext_manager.get_extension_path(ext_id)

        self._assets_root_path = await get_assets_root_path_async()
        if self._assets_root_path is None:
            carb.log_error("Could not find Isaac Sim assets folder")
            return
        kit_folder = carb.tokens.get_tokens_interface().resolve("${kit}")

        self._physics_rate = 60
        carb.settings.get_settings().set_bool("/app/runLoops/main/rateLimitEnabled", True)
        carb.settings.get_settings().set_int("/app/runLoops/main/rateLimitFrequency", int(self._physics_rate))
        carb.settings.get_settings().set_int("/persistent/simulation/minFrameRate", int(self._physics_rate))
        await omni.kit.app.get_app().next_update_async()
        rclpy.init()

        ## load asset and setup ROS bridge
        # open simple_articulation asset (with one drivable revolute and one drivable prismatic joint)
        self._assets_root_path = await get_assets_root_path_async()
        await omni.kit.app.get_app().next_update_async()
        self.usd_path = self._assets_root_path + "/Isaac/Robots/Simple/articulation_3_joints.usd"
        (result, error) = await open_stage_async(self.usd_path)
        await omni.kit.app.get_app().next_update_async()
        self.assertTrue(result)  # Make sure the stage loaded
        self._stage = omni.usd.get_context().get_stage()
        # ROS-ify asset by adding a joint state publisher
        try:
            og.Controller.edit(
                {"graph_path": "/ActionGraph", "evaluator_name": "execution"},
                {
                    og.Controller.Keys.CREATE_NODES: [
                        ("OnPlaybackTick", "omni.graph.action.OnPlaybackTick"),
                        ("PublishJointState", "omni.isaac.ros2_bridge.ROS2PublishJointState"),
                        ("ReadSimTime", "omni.isaac.core_nodes.IsaacReadSimulationTime"),
                    ],
                    og.Controller.Keys.SET_VALUES: [
                        (
                            "PublishJointState.inputs:targetPrim",
                            [usdrt.Sdf.Path("/Articulation")],
                        ),
                    ],
                    og.Controller.Keys.CONNECT: [
                        ("OnPlaybackTick.outputs:tick", "PublishJointState.inputs:execIn"),
                        ("ReadSimTime.outputs:simulationTime", "PublishJointState.inputs:timeStamp"),
                    ],
                },
            )
        except Exception as e:
            print(e)

        pass

    # After running each test
    async def tearDown(self):
        import rclpy

        while omni.usd.get_context().get_stage_loading_status()[2] > 0:
            print("tearDown, assets still loading, waiting to finish...")
            await asyncio.sleep(1.0)

        self._timeline = None
        rclpy.shutdown()
        gc.collect()
        pass

    async def test_joint_state_position_publisher(self):
        import rclpy
        from sensor_msgs.msg import JointState

        # setup ROS listener of the joint_state topic
        self.js_ros = JointState()

        def js_callback(data: JointState):
            self.js_ros.position = data.position
            self.js_ros.velocity = data.velocity
            self.js_ros.effort = data.effort

        node = rclpy.create_node("isaac_sim_test_joint_state_pub_sub")
        js_sub = node.create_subscription(JointState, "joint_states", js_callback, get_qos_profile())

        def spin():
            rclpy.spin_once(node, timeout_sec=0.1)

        default_position = [-80 * PI / 180.0, 0.4, 30 * PI / 180.0]

        self._timeline.play()
        await omni.kit.app.get_app().next_update_async()
        await simulate_async(2, 60, spin)
        received_position = self.js_ros.position

        print("\n received_position", received_position)

        self.assertAlmostEqual(received_position[0], default_position[0], delta=1e-3)
        self.assertAlmostEqual(received_position[1], default_position[1], delta=1e-3)
        self.assertAlmostEqual(received_position[2], default_position[2], delta=1e-3)

        self._timeline.stop()
        spin()

        pass

    async def test_joint_state_velocity_publisher(self):
        import rclpy
        from sensor_msgs.msg import JointState

        # setup ROS listener of the joint_state topic
        self.js_ros = JointState()

        def js_callback(data: JointState):
            self.js_ros.velocity = data.velocity

        node = rclpy.create_node("isaac_sim_test_joint_state_pub_sub")
        js_sub = node.create_subscription(JointState, "joint_states", js_callback, get_qos_profile())

        def spin():
            rclpy.spin_once(node, timeout_sec=0.1)

        joint_paths = [
            "/Articulation/Arm/CenterRevoluteJoint",
            "/Articulation/Slider/PrismaticJoint",
            "/Articulation/DistalPivot/DistalRevoluteJoint",
        ]

        joint_types = ["angular", "linear", "angular"]
        test_velocities = [5, 0.1, -2.5]
        joint_stiffness = 0
        joint_damping = 1e4
        num_joints = 3

        # # set the stiffness and damping parameters accordingly for position control
        for i in range(num_joints):
            set_joint_drive_parameters(
                joint_paths[i], joint_types[i], "velocity", test_velocities[i], joint_stiffness, joint_damping
            )

        # tick and get ros sub
        # # set the stiffness and damping parameters accordingly for position control
        self._timeline.play()

        await simulate_async(2, 60, spin)
        received_velocity = self.js_ros.velocity

        comp_velocity = [5 * PI / 180.0, 0.1, -2.5 * PI / 180.0]

        # print("test_velocities_radian", comp_velocity)

        self.assertAlmostEqual(received_velocity[0], comp_velocity[0], delta=1e-3)
        self.assertAlmostEqual(received_velocity[1], comp_velocity[1], delta=1e-3)
        self.assertAlmostEqual(received_velocity[2], comp_velocity[2], delta=1e-3)
        self._timeline.stop()
        spin()


class TestRos2JointStateSubscriber(omni.kit.test.AsyncTestCase):
    # Before running each test
    async def setUp(self):
        import rclpy

        await omni.usd.get_context().new_stage_async()
        self._timeline = omni.timeline.get_timeline_interface()

        ext_manager = omni.kit.app.get_app().get_extension_manager()
        ext_id = ext_manager.get_enabled_extension_id("omni.isaac.ros2_bridge")
        self._ros_extension_path = ext_manager.get_extension_path(ext_id)

        self._assets_root_path = await get_assets_root_path_async()
        if self._assets_root_path is None:
            carb.log_error("Could not find Isaac Sim assets folder")
            return
        kit_folder = carb.tokens.get_tokens_interface().resolve("${kit}")

        self._physics_rate = 60
        carb.settings.get_settings().set_bool("/app/runLoops/main/rateLimitEnabled", True)
        carb.settings.get_settings().set_int("/app/runLoops/main/rateLimitFrequency", int(self._physics_rate))
        carb.settings.get_settings().set_int("/persistent/simulation/minFrameRate", int(self._physics_rate))
        await omni.kit.app.get_app().next_update_async()
        rclpy.init()

        ## load asset and setup ROS bridge
        # open simple_articulation asset (with one drivable revolute and one drivable prismatic joint)
        self._assets_root_path = await get_assets_root_path_async()
        await omni.kit.app.get_app().next_update_async()
        self.usd_path = self._assets_root_path + "/Isaac/Robots/Simple/articulation_3_joints.usd"
        (result, error) = await open_stage_async(self.usd_path)
        await omni.kit.app.get_app().next_update_async()
        self.assertTrue(result)  # Make sure the stage loaded
        self._stage = omni.usd.get_context().get_stage()
        # ROS-ify asset by adding a joint state publisher
        # setup the graph
        try:
            (test_graph, new_nodes, _, _) = og.Controller.edit(
                {"graph_path": "/ActionGraph", "evaluator_name": "execution"},
                {
                    og.Controller.Keys.CREATE_NODES: [
                        ("OnPlaybackTick", "omni.graph.action.OnPlaybackTick"),
                        ("SubscribeJointState", "omni.isaac.ros2_bridge.ROS2SubscribeJointState"),
                        ("ArticulationController", "omni.isaac.core_nodes.IsaacArticulationController"),
                    ],
                    og.Controller.Keys.CONNECT: [
                        ("OnPlaybackTick.outputs:tick", "SubscribeJointState.inputs:execIn"),
                        ("OnPlaybackTick.outputs:tick", "ArticulationController.inputs:execIn"),
                        (
                            "SubscribeJointState.outputs:positionCommand",
                            "ArticulationController.inputs:positionCommand",
                        ),
                        (
                            "SubscribeJointState.outputs:velocityCommand",
                            "ArticulationController.inputs:velocityCommand",
                        ),
                        ("SubscribeJointState.outputs:effortCommand", "ArticulationController.inputs:effortCommand"),
                    ],
                    og.Controller.Keys.SET_VALUES: [
                        ("ArticulationController.inputs:targetPrim", [usdrt.Sdf.Path("/Articulation")]),
                    ],
                },
            )

            self.subscriber_node = new_nodes[1]

        except Exception as e:
            print(e)

        pass

    # After running each test
    async def tearDown(self):
        import rclpy

        while omni.usd.get_context().get_stage_loading_status()[2] > 0:
            print("tearDown, assets still loading, waiting to finish...")
            await asyncio.sleep(1.0)

        self._timeline = None
        rclpy.shutdown()
        gc.collect()
        pass

    async def test_joint_state_subscriber_node(self):
        """
        test if the joint state subscriber node is able to receive the joint state commands
        """
        import rclpy
        from sensor_msgs.msg import JointState

        ros2_publisher = None
        ros2_node = rclpy.create_node("isaac_sim_test_joint_state_sub")
        ros2_publisher = ros2_node.create_publisher(JointState, "joint_command", 10)

        # test position drive
        js_position = JointState()
        js_position.name = ["CenterRevoluteJoint", "PrismaticJoint", "DistalRevoluteJoint"]
        js_position.position = [45 * PI / 180.0, 0.2, -120 * PI / 180.0]
        js_position.velocity = [5 * PI / 180.0, 0.1, -2.5 * PI / 180.0]
        js_position.effort = [0.4, -0.2, 0.3]

        await omni.kit.app.get_app().next_update_async()
        self._timeline.play()
        await simulate_async(0.5)

        # publish value
        ros2_publisher.publish(js_position)
        await simulate_async(0.5)

        # get the value from the subscriber node
        joint_names = og.Controller.attribute("outputs:jointNames", self.subscriber_node).get()
        positions_received = og.Controller.attribute("outputs:positionCommand", self.subscriber_node).get()
        velocities_received = og.Controller.attribute("outputs:velocityCommand", self.subscriber_node).get()
        efforts_received = og.Controller.attribute("outputs:effortCommand", self.subscriber_node).get()

        self.assertAlmostEqual(positions_received[0], js_position.position[0], delta=1e-3)
        self.assertAlmostEqual(positions_received[1], js_position.position[1], delta=1e-3)
        self.assertAlmostEqual(positions_received[2], js_position.position[2], delta=1e-3)
        self.assertAlmostEqual(velocities_received[0], js_position.velocity[0], delta=1e-3)
        self.assertAlmostEqual(velocities_received[1], js_position.velocity[1], delta=1e-3)
        self.assertAlmostEqual(velocities_received[2], js_position.velocity[2], delta=1e-3)
        self.assertAlmostEqual(efforts_received[0], js_position.effort[0], delta=1e-3)
        self.assertAlmostEqual(efforts_received[1], js_position.effort[1], delta=1e-3)
        self.assertAlmostEqual(efforts_received[2], js_position.effort[2], delta=1e-3)

    async def test_joint_state_subscriber(self):
        """
        test if the joint state subscriber is able to move the robot as expected
        """
        import rclpy
        from sensor_msgs.msg import JointState

        ros2_publisher = None
        ros2_node = rclpy.create_node("isaac_sim_test_joint_state_sub")
        ros2_publisher = ros2_node.create_publisher(JointState, "joint_command", 10)

        test_position = [45 * PI / 180.0, 0.2, -120 * PI / 180.0]
        test_velocity = [5 * PI / 180.0, 0.1, -2.5 * PI / 180.0]
        test_effort = [0.4, -0.2, 0.3]

        # test position drive
        js_position = JointState()
        js_position.name = ["CenterRevoluteJoint", "PrismaticJoint", "DistalRevoluteJoint"]
        js_position.position = test_position

        self._timeline.play()
        await simulate_async(0.5)

        # get the articulation review for the asset
        art_handle = Articulation("/Articulation")
        art_handle.initialize()
        await simulate_async(0.5)

        def reset_robot():
            # reset the robot to 0s
            art_handle.set_joint_positions([0, 0, 0])
            # give it a second to move
            post_reset = art_handle.get_joint_positions()
            self.assertAlmostEqual(post_reset[0], 0, delta=1e-3)
            self.assertAlmostEqual(post_reset[1], 0, delta=1e-3)
            self.assertAlmostEqual(post_reset[2], 0, delta=1e-3)

        # publish value
        ros2_publisher.publish(js_position)
        # give it a second to move
        await simulate_async(1)

        joint_command_received = art_handle.get_joint_positions()
        print("joint_command_received", joint_command_received)

        self.assertAlmostEqual(joint_command_received[0], test_position[0], delta=1e-3)
        self.assertAlmostEqual(joint_command_received[1], test_position[1], delta=1e-3)
        self.assertAlmostEqual(joint_command_received[2], test_position[2], delta=1e-3)

        # test velocity drive
        print("test velocity drive")
        reset_robot()

        # change joint drives for velocity drive test
        joint_paths = [
            "/Articulation/Arm/CenterRevoluteJoint",
            "/Articulation/Slider/PrismaticJoint",
            "/Articulation/DistalPivot/DistalRevoluteJoint",
        ]

        joint_types = ["angular", "linear", "angular"]
        target_velocities = [5, 0.1, -2.5]
        joint_stiffness = 0
        joint_damping = 1e4
        num_joints = 3

        # # set the stiffness and damping parameters accordingly for position control
        for i in range(num_joints):
            set_joint_drive_parameters(
                joint_paths[i], joint_types[i], "velocity", target_velocities[i], joint_stiffness, joint_damping
            )

        # test velocity drive
        js_velocity = JointState()
        js_velocity.name = ["CenterRevoluteJoint", "PrismaticJoint", "DistalRevoluteJoint"]
        js_velocity.velocity = test_velocity

        # publish value
        ros2_publisher.publish(js_velocity)
        # give it a second to move
        await simulate_async(1)

        joint_command_received = art_handle.get_joint_velocities()
        print("joint_velocity_received", joint_command_received)

        self.assertAlmostEqual(joint_command_received[0], test_velocity[0], delta=1e-3)
        self.assertAlmostEqual(joint_command_received[1], test_velocity[1], delta=1e-3)
        self.assertAlmostEqual(joint_command_received[2], test_velocity[2], delta=1e-3)

        # test mixed drive
        print("test mixed drive")
        reset_robot()

        # change prismatic joint back to postion drive
        set_joint_drive_parameters(joint_paths[1], joint_types[1], "position", 0.2, 1e5, 1e4)

        js_mixed = JointState()
        js_mixed.name = ["CenterRevoluteJoint", "PrismaticJoint", "DistalRevoluteJoint"]
        js_mixed.position = [float("nan"), 0.4, float("nan")]
        js_mixed.velocity = [0.5, float("nan"), -2.5]

        ros2_publisher.publish(js_mixed)
        # give it a second to move
        await simulate_async(2)

        joint_position_received = art_handle.get_joint_positions()
        joint_velocity_received = art_handle.get_joint_velocities()
        print("joint_position_received", joint_position_received)
        print("joint_velocity_received", joint_velocity_received)

        self.assertAlmostEqual(joint_position_received[1], 0.4, delta=1e-2)

        self.assertAlmostEqual(joint_velocity_received[0], 0.5, delta=1e-2)
        self.assertAlmostEqual(joint_velocity_received[2], -2.5, delta=1e-2)
        self.assertAlmostEqual(joint_velocity_received[1], 0, delta=1e-2)

    async def test_joint_state_subscriber_with_names(self):
        """
        test if the joint state subscriber is able to move the robot as expected
        """

        # add the connection between joint names from subscriber and the controller
        graph_handle = og.get_graph_by_path("/ActionGraph")
        og.Controller.connect(
            "/ActionGraph/SubscribeJointState.outputs:jointNames",
            "/ActionGraph/ArticulationController.inputs:jointNames",
        )
        await og.Controller.evaluate(graph_handle)

        import rclpy
        from sensor_msgs.msg import JointState

        ros2_publisher = None
        ros2_node = rclpy.create_node("isaac_sim_test_joint_state_sub")
        ros2_publisher = ros2_node.create_publisher(JointState, "joint_command", 10)

        test_position = [45 * PI / 180.0, 0.2, -120 * PI / 180.0]
        test_velocity = [5 * PI / 180.0, 0.1, -2.5 * PI / 180.0]
        test_effort = [0.4, -0.2, 0.3]

        # test position drive
        js_position = JointState()
        js_position.name = ["CenterRevoluteJoint", "PrismaticJoint", "DistalRevoluteJoint"]
        js_position.position = test_position

        self._timeline.play()
        await simulate_async(0.5)

        # get the articulation review for the asset
        art_handle = Articulation("/Articulation")
        art_handle.initialize()
        await simulate_async(0.5)

        def reset_robot():
            # reset the robot to 0s
            art_handle.set_joint_positions([0, 0, 0])
            # give it a second to move
            post_reset = art_handle.get_joint_positions()
            self.assertAlmostEqual(post_reset[0], 0, delta=1e-3)
            self.assertAlmostEqual(post_reset[1], 0, delta=1e-3)
            self.assertAlmostEqual(post_reset[2], 0, delta=1e-3)

        # publish value
        ros2_publisher.publish(js_position)
        # give it a second to move
        await simulate_async(1)

        joint_command_received = art_handle.get_joint_positions()
        print("joint_command_received", joint_command_received)

        self.assertAlmostEqual(joint_command_received[0], test_position[0], delta=1e-3)
        self.assertAlmostEqual(joint_command_received[1], test_position[1], delta=1e-3)
        self.assertAlmostEqual(joint_command_received[2], test_position[2], delta=1e-3)

        # test velocity drive
        print("test velocity drive")
        reset_robot()

        # change joint drives for velocity drive test
        joint_paths = [
            "/Articulation/Arm/CenterRevoluteJoint",
            "/Articulation/Slider/PrismaticJoint",
            "/Articulation/DistalPivot/DistalRevoluteJoint",
        ]

        joint_types = ["angular", "linear", "angular"]
        target_velocities = [5, 0.1, -2.5]
        joint_stiffness = 0
        joint_damping = 1e4
        num_joints = 3

        # # set the stiffness and damping parameters accordingly for position control
        for i in range(num_joints):
            set_joint_drive_parameters(
                joint_paths[i], joint_types[i], "velocity", target_velocities[i], joint_stiffness, joint_damping
            )

        # test velocity drive
        js_velocity = JointState()
        js_velocity.name = ["CenterRevoluteJoint", "PrismaticJoint", "DistalRevoluteJoint"]
        js_velocity.velocity = test_velocity

        # publish value
        ros2_publisher.publish(js_velocity)
        # give it a second to move
        await simulate_async(1)

        joint_command_received = art_handle.get_joint_velocities()
        print("joint_velocity_received", joint_command_received)

        self.assertAlmostEqual(joint_command_received[0], test_velocity[0], delta=1e-3)
        self.assertAlmostEqual(joint_command_received[1], test_velocity[1], delta=1e-3)
        self.assertAlmostEqual(joint_command_received[2], test_velocity[2], delta=1e-3)

        # test mixed drive
        print("test mixed drive")
        reset_robot()

        # change prismatic joint back to postion drive
        set_joint_drive_parameters(joint_paths[1], joint_types[1], "position", 0.2, 1e5, 1e4)

        js_mixed = JointState()
        js_mixed.name = ["CenterRevoluteJoint", "PrismaticJoint", "DistalRevoluteJoint"]
        js_mixed.position = [float("nan"), 0.4, float("nan")]
        js_mixed.velocity = [0.5, float("nan"), -2.5]

        ros2_publisher.publish(js_mixed)
        # give it a second to move
        await simulate_async(2)

        joint_position_received = art_handle.get_joint_positions()
        joint_velocity_received = art_handle.get_joint_velocities()
        print("joint_position_received", joint_position_received)
        print("joint_velocity_received", joint_velocity_received)

        self.assertAlmostEqual(joint_position_received[1], 0.4, delta=1e-2)

        self.assertAlmostEqual(joint_velocity_received[0], 0.5, delta=1e-2)
        self.assertAlmostEqual(joint_velocity_received[2], -2.5, delta=1e-2)
        self.assertAlmostEqual(joint_velocity_received[1], 0, delta=1e-2)
