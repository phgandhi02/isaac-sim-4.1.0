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
import random
import time

import carb
import numpy as np
import omni.graph.core as og

# Import extension python module we are testing with absolute import path, as if we are external user (other extension)
import omni.kit.commands

# NOTE:
#   omni.kit.test - std python's unittest module with additional wrapping to add suport for async/await tests
#   For most things refer to unittest docs: https://docs.python.org/3/library/unittest.html
import omni.kit.test
import omni.kit.usd
import usdrt.Sdf
from omni.isaac.core.articulations import Articulation
from omni.isaac.core.objects import DynamicCuboid
from omni.isaac.core.utils.physics import simulate_async

# from omni.isaac.benchmark.services.utils import wait_until_stage_is_fully_loaded_async
from omni.isaac.core.utils.stage import open_stage_async
from omni.isaac.core.utils.xforms import get_world_pose
from omni.isaac.nucleus import get_assets_root_path_async
from pxr import Gf, Sdf, UsdGeom, UsdPhysics

from .common import get_qos_profile


class TestRos2Subscribers(omni.kit.test.AsyncTestCase):
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
        self.sub_data = []
        self.prev_seq = 0
        self.MAX_COUNT = 100
        self.MAX_OFFSET = abs(self.MAX_COUNT - 95)
        self.queue_size = 10

        carb.settings.get_settings().set_bool("/app/runLoops/main/rateLimitEnabled", True)
        carb.settings.get_settings().set_int("/app/runLoops/main/rateLimitFrequency", int(self._physics_rate))
        carb.settings.get_settings().set_int("/persistent/simulation/minFrameRate", int(self._physics_rate))
        await omni.kit.app.get_app().next_update_async()
        rclpy.init()

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

    def spin(self):
        seq = og.Controller.get(og.Controller.attribute(self.sub_node_time_attribute_path))
        if isinstance(seq, np.ndarray):
            seq = seq[0] if seq is not None else self.prev_seq

        if self.prev_seq != seq:
            self.sub_data.append(seq)
            self.prev_seq = seq

    def reset_queue_size(self, path, queue_size):
        self.sub_data = []
        self.prev_seq = 0
        og.Controller.set(og.Controller.attribute(path), queue_size)
        self.queue_size = queue_size

    def choose_queue_size(self):
        queue_size = random.randint(1, self.MAX_COUNT - self.MAX_OFFSET)
        print("Choosing queue size of", queue_size)
        return queue_size

    async def test_joint_state_subscriber_queue(self):

        import rclpy
        from builtin_interfaces.msg import Time
        from sensor_msgs.msg import JointState

        self._stage = omni.usd.get_context().get_stage()

        node = rclpy.create_node("isaac_sim_test_joint_state_sub_queue")
        ros_topic = "joint_sub"
        test_pub = node.create_publisher(JointState, ros_topic, 1)

        self.graph_path = "/ActionGraph"

        # We are using timestamp to store message sequence
        self.sub_node_time_attribute_path = self.graph_path + "/SubscribeJointState.outputs:timeStamp"
        sub_node_queue_attribute_path = self.graph_path + "/SubscribeJointState.inputs:queueSize"

        try:
            og.Controller.edit(
                {"graph_path": self.graph_path, "evaluator_name": "execution"},
                {
                    og.Controller.Keys.CREATE_NODES: [
                        ("OnPlaybackTick", "omni.graph.action.OnPlaybackTick"),
                        ("SubscribeJointState", "omni.isaac.ros2_bridge.ROS2SubscribeJointState"),
                    ],
                    og.Controller.Keys.SET_VALUES: [
                        (
                            "SubscribeJointState.inputs:topicName",
                            ros_topic,
                        ),
                    ],
                    og.Controller.Keys.CONNECT: [
                        ("OnPlaybackTick.outputs:tick", "SubscribeJointState.inputs:execIn"),
                    ],
                },
            )
        except Exception as e:
            print(e)

        # Set Queue Size in subscriber OG node and reset test variables

        self.reset_queue_size(sub_node_queue_attribute_path, queue_size=self.choose_queue_size())

        self._timeline.play()
        await omni.kit.app.get_app().next_update_async()
        await omni.kit.app.get_app().next_update_async()
        await omni.kit.app.get_app().next_update_async()

        def publish_data():
            # We are using timestamp to store message sequence
            for count in range(1, self.MAX_COUNT):
                msg = JointState()
                time_obj = Time()
                time_obj.sec = count
                msg.header.stamp = time_obj
                msg.name = ["test1", "test2"]
                msg.position = [0.0, 0.0]
                test_pub.publish(msg)
                time.sleep(0.01)

        publish_data()

        await simulate_async(2, 60, self.spin)

        self._timeline.stop()

        self.assertTrue(self.sub_data == [self.MAX_COUNT - self.queue_size + i for i in range(self.queue_size)])

        ## Change queue size to random value
        self.reset_queue_size(sub_node_queue_attribute_path, queue_size=self.choose_queue_size())

        self._timeline.play()
        await omni.kit.app.get_app().next_update_async()
        await omni.kit.app.get_app().next_update_async()
        await omni.kit.app.get_app().next_update_async()

        publish_data()

        await simulate_async(2, 60, self.spin)

        self._timeline.stop()

        self.assertTrue(self.sub_data == [self.MAX_COUNT - self.queue_size + i for i in range(self.queue_size)])

        pass

    async def test_clock_subscriber_queue(self):

        import rclpy
        from builtin_interfaces.msg import Time
        from rosgraph_msgs.msg import Clock

        self._stage = omni.usd.get_context().get_stage()

        node = rclpy.create_node("isaac_sim_test_clock_sub_queue")
        ros_topic = "clock_sub"
        test_pub = node.create_publisher(Clock, ros_topic, 1)

        self.graph_path = "/ActionGraph"

        # We are using timestamp to store message sequence
        self.sub_node_time_attribute_path = self.graph_path + "/SubscribeClock.outputs:timeStamp"
        sub_node_queue_attribute_path = self.graph_path + "/SubscribeClock.inputs:queueSize"

        try:
            og.Controller.edit(
                {"graph_path": self.graph_path, "evaluator_name": "execution"},
                {
                    og.Controller.Keys.CREATE_NODES: [
                        ("OnPlaybackTick", "omni.graph.action.OnPlaybackTick"),
                        ("SubscribeClock", "omni.isaac.ros2_bridge.ROS2SubscribeClock"),
                    ],
                    og.Controller.Keys.SET_VALUES: [
                        (
                            "SubscribeClock.inputs:topicName",
                            ros_topic,
                        ),
                    ],
                    og.Controller.Keys.CONNECT: [
                        ("OnPlaybackTick.outputs:tick", "SubscribeClock.inputs:execIn"),
                    ],
                },
            )
        except Exception as e:
            print(e)

        # Set random Queue Size in subscriber OG node and reset test variables

        self.reset_queue_size(sub_node_queue_attribute_path, queue_size=self.choose_queue_size())

        self._timeline.play()
        await omni.kit.app.get_app().next_update_async()
        await omni.kit.app.get_app().next_update_async()
        await omni.kit.app.get_app().next_update_async()

        def publish_data():
            # We are using timestamp to store message sequence
            for count in range(1, self.MAX_COUNT):
                msg = Clock()
                time_obj = Time()
                time_obj.sec = count
                msg.clock = time_obj
                test_pub.publish(msg)
                time.sleep(0.01)

        publish_data()

        await simulate_async(2, 60, self.spin)

        self._timeline.stop()

        self.assertTrue(self.sub_data == [self.MAX_COUNT - self.queue_size + i for i in range(self.queue_size)])

        ## Change queue size to random value
        self.reset_queue_size(sub_node_queue_attribute_path, queue_size=self.choose_queue_size())

        self._timeline.play()
        await omni.kit.app.get_app().next_update_async()
        await omni.kit.app.get_app().next_update_async()
        await omni.kit.app.get_app().next_update_async()

        publish_data()

        await simulate_async(2, 60, self.spin)

        self._timeline.stop()
        self.assertTrue(self.sub_data == [self.MAX_COUNT - self.queue_size + i for i in range(self.queue_size)])

        pass

    async def test_twist_subscriber_queue(self):

        import rclpy
        from geometry_msgs.msg import Twist

        self._stage = omni.usd.get_context().get_stage()

        node = rclpy.create_node("isaac_sim_test_twist_sub_queue")
        ros_topic = "twist_sub"
        test_pub = node.create_publisher(Twist, ros_topic, 1)

        self.graph_path = "/ActionGraph"

        # We are using timestamp to store message sequence
        self.sub_node_time_attribute_path = self.graph_path + "/SubscribeTwist.outputs:linearVelocity"
        sub_node_queue_attribute_path = self.graph_path + "/SubscribeTwist.inputs:queueSize"

        try:
            og.Controller.edit(
                {"graph_path": self.graph_path, "evaluator_name": "execution"},
                {
                    og.Controller.Keys.CREATE_NODES: [
                        ("OnPlaybackTick", "omni.graph.action.OnPlaybackTick"),
                        ("SubscribeTwist", "omni.isaac.ros2_bridge.ROS2SubscribeTwist"),
                    ],
                    og.Controller.Keys.SET_VALUES: [
                        (
                            "SubscribeTwist.inputs:topicName",
                            ros_topic,
                        ),
                    ],
                    og.Controller.Keys.CONNECT: [
                        ("OnPlaybackTick.outputs:tick", "SubscribeTwist.inputs:execIn"),
                    ],
                },
            )
        except Exception as e:
            print(e)

        # Set random Queue Size in subscriber OG node and reset test variables

        self.reset_queue_size(sub_node_queue_attribute_path, queue_size=self.choose_queue_size())

        self._timeline.play()
        await omni.kit.app.get_app().next_update_async()
        await omni.kit.app.get_app().next_update_async()
        await omni.kit.app.get_app().next_update_async()

        def publish_data():
            # We are using linear.x to store message sequence
            for count in range(1, self.MAX_COUNT):
                msg = Twist()
                msg.linear.x = float(count)
                test_pub.publish(msg)
                time.sleep(0.01)

        publish_data()

        await simulate_async(2, 60, self.spin)

        self._timeline.stop()

        self.assertTrue(self.sub_data == [self.MAX_COUNT - self.queue_size + i for i in range(self.queue_size)])

        ## Change queue size to random value
        self.reset_queue_size(sub_node_queue_attribute_path, queue_size=self.choose_queue_size())

        self._timeline.play()
        await omni.kit.app.get_app().next_update_async()
        await omni.kit.app.get_app().next_update_async()
        await omni.kit.app.get_app().next_update_async()

        publish_data()

        await simulate_async(2, 60, self.spin)

        self._timeline.stop()
        self.assertTrue(self.sub_data == [self.MAX_COUNT - self.queue_size + i for i in range(self.queue_size)])

        pass

    async def test_ackermann_subscriber_queue(self):

        import rclpy
        from ackermann_msgs.msg import AckermannDriveStamped
        from builtin_interfaces.msg import Time

        self._stage = omni.usd.get_context().get_stage()

        node = rclpy.create_node("isaac_sim_test_AckermannDrive_sub_queue")
        ros_topic = "ackermann_sub"
        test_pub = node.create_publisher(AckermannDriveStamped, ros_topic, 1)

        self.graph_path = "/ActionGraph"

        # We are using timestamp to store message sequence
        self.sub_node_time_attribute_path = self.graph_path + "/SubscribeAckermannDrive.outputs:timeStamp"
        sub_node_queue_attribute_path = self.graph_path + "/SubscribeAckermannDrive.inputs:queueSize"

        try:
            og.Controller.edit(
                {"graph_path": self.graph_path, "evaluator_name": "execution"},
                {
                    og.Controller.Keys.CREATE_NODES: [
                        ("OnPlaybackTick", "omni.graph.action.OnPlaybackTick"),
                        ("SubscribeAckermannDrive", "omni.isaac.ros2_bridge.ROS2SubscribeAckermannDrive"),
                    ],
                    og.Controller.Keys.SET_VALUES: [
                        (
                            "SubscribeAckermannDrive.inputs:topicName",
                            ros_topic,
                        ),
                    ],
                    og.Controller.Keys.CONNECT: [
                        ("OnPlaybackTick.outputs:tick", "SubscribeAckermannDrive.inputs:execIn"),
                    ],
                },
            )
        except Exception as e:
            print(e)

        # Set random Queue Size in subscriber OG node and reset test variables

        self.reset_queue_size(sub_node_queue_attribute_path, queue_size=self.choose_queue_size())

        self._timeline.play()
        await omni.kit.app.get_app().next_update_async()
        await omni.kit.app.get_app().next_update_async()
        await omni.kit.app.get_app().next_update_async()

        def publish_data():
            # We are using linear.x to store message sequence
            for count in range(1, self.MAX_COUNT):
                msg = AckermannDriveStamped()
                time_obj = Time()
                time_obj.sec = count
                msg.header.stamp = time_obj
                test_pub.publish(msg)
                time.sleep(0.01)

        publish_data()

        await simulate_async(2, 60, self.spin)

        self._timeline.stop()

        self.assertTrue(self.sub_data == [self.MAX_COUNT - self.queue_size + i for i in range(self.queue_size)])

        ## Change queue size to random value
        self.reset_queue_size(sub_node_queue_attribute_path, queue_size=self.choose_queue_size())

        self._timeline.play()
        await omni.kit.app.get_app().next_update_async()
        await omni.kit.app.get_app().next_update_async()
        await omni.kit.app.get_app().next_update_async()

        publish_data()

        await simulate_async(2, 60, self.spin)

        self._timeline.stop()
        self.assertTrue(self.sub_data == [self.MAX_COUNT - self.queue_size + i for i in range(self.queue_size)])

        pass

    async def test_transform_tree_subscriber(self):

        import rclpy
        from geometry_msgs.msg import TransformStamped
        from tf2_msgs.msg import TFMessage

        self._stage = omni.usd.get_context().get_stage()

        # Create a node to subscribe to TFs
        node = rclpy.create_node("isaac_sim_test_transform_tree_sub_queue")
        ros_topic = "tf_sub"
        test_pub = node.create_publisher(TFMessage, ros_topic, 1)

        self.graph_path = "/ActionGraph"

        try:
            og.Controller.edit(
                {"graph_path": self.graph_path, "evaluator_name": "execution"},
                {
                    og.Controller.Keys.CREATE_NODES: [
                        ("OnPlaybackTick", "omni.graph.action.OnPlaybackTick"),
                        ("SubscribeTransformTree", "omni.isaac.ros2_bridge.ROS2SubscribeTransformTree"),
                    ],
                    og.Controller.Keys.SET_VALUES: [
                        ("SubscribeTransformTree.inputs:topicName", ros_topic),
                        ("SubscribeTransformTree.inputs:frameNamesMap", ["/World", "world", "/World/cube", "cube"]),
                    ],
                    og.Controller.Keys.CONNECT: [
                        ("OnPlaybackTick.outputs:tick", "SubscribeTransformTree.inputs:execIn"),
                    ],
                },
            )
        except Exception as e:
            print(e)

        # Create our scene
        UsdGeom.Xform.Define(self._stage, "/World")

        scene = UsdPhysics.Scene.Define(self._stage, Sdf.Path("/World/physicsScene"))
        scene.CreateGravityDirectionAttr().Set(Gf.Vec3f(0.0, 0.0, -1.0))
        scene.CreateGravityMagnitudeAttr().Set(9.81)

        cube = DynamicCuboid(prim_path="/World/cube", position=np.array([0, 0, 5]))

        self._timeline.play()
        await omni.kit.app.get_app().next_update_async()
        await omni.kit.app.get_app().next_update_async()
        await omni.kit.app.get_app().next_update_async()

        for i in range(10):
            msg = TransformStamped()
            msg.header.stamp = node.get_clock().now().to_msg()
            msg.child_frame_id = "cube"
            msg.header.frame_id = "world"

            pos = np.array([float(i) / 5.0, float(i) * float(i) / 25.0, 1.0])

            msg.transform.translation.x = pos[0]
            msg.transform.translation.y = pos[1]
            msg.transform.translation.z = pos[2]

            # Rotation of i radians around (1,1,1)
            a = np.sin(float(i) / 2.0) / np.sqrt(3.0)

            rot = [np.cos(float(i) / 2.0), a, a, a]
            msg.transform.rotation.x = rot[1]
            msg.transform.rotation.y = rot[2]
            msg.transform.rotation.z = rot[3]
            msg.transform.rotation.w = rot[0]

            tf_msg = TFMessage()
            tf_msg.transforms.append(msg)

            test_pub.publish(tf_msg)

            time.sleep(0.01)

            await simulate_async(1.0, 60)
            await omni.kit.app.get_app().next_update_async()

            x, r = get_world_pose("/World/cube")

            # NOTE : a quaterion q and -q represent the same rotation
            self.assertTrue(np.linalg.norm(x - pos) < 1e-5)
            self.assertTrue(np.linalg.norm(r - rot) < 1e-5 or np.linalg.norm(r + rot) < 1e-5)

        self._timeline.stop()

        pass

    async def test_transform_tree_subscriber_nova_carter(self):

        import rclpy
        from geometry_msgs.msg import TransformStamped
        from tf2_msgs.msg import TFMessage

        # Load our Nova Carter ROS stage
        stage_path = "/Isaac/Robots/Carter/nova_carter_sensors.usd"
        await open_stage_async(self._assets_root_path + stage_path)

        self._stage = omni.usd.get_context().get_stage()

        # Create a node to subscribe to TFs
        node = rclpy.create_node("isaac_sim_test_transform_tree_sub_nova_carter")
        ros_topic = "tf_sub"
        test_pub = node.create_publisher(TFMessage, ros_topic, 1)

        self.graph_path = "/ActionGraph"

        try:
            og.Controller.edit(
                {"graph_path": self.graph_path, "evaluator_name": "execution"},
                {
                    og.Controller.Keys.CREATE_NODES: [
                        ("OnPlaybackTick", "omni.graph.action.OnPlaybackTick"),
                        ("SubscribeTransformTree", "omni.isaac.ros2_bridge.ROS2SubscribeTransformTree"),
                    ],
                    og.Controller.Keys.SET_VALUES: [
                        ("SubscribeTransformTree.inputs:topicName", ros_topic),
                        (
                            "SubscribeTransformTree.inputs:frameNamesMap",
                            ["/nova_carter/chassis_link", "base_link", "/nova_carter", "odom"],
                        ),
                        ("SubscribeTransformTree.inputs:articulationRoots", ["/nova_carter"]),
                    ],
                    og.Controller.Keys.CONNECT: [
                        ("OnPlaybackTick.outputs:tick", "SubscribeTransformTree.inputs:execIn"),
                    ],
                },
            )
        except Exception as e:
            print(e)

        self._timeline.play()
        await omni.kit.app.get_app().next_update_async()
        await omni.kit.app.get_app().next_update_async()
        await omni.kit.app.get_app().next_update_async()

        for i in range(10):
            msg = TransformStamped()
            msg.header.stamp = node.get_clock().now().to_msg()
            msg.child_frame_id = "base_link"
            msg.header.frame_id = "odom"

            pos = np.array([float(i) / 5.0, float(i) * float(i) / 25.0, 1.0])

            msg.transform.translation.x = pos[0]
            msg.transform.translation.y = pos[1]
            msg.transform.translation.z = pos[2]

            # Rotation of i radians around (1,1,1)
            a = np.sin(float(i) / 2.0) / np.sqrt(3.0)

            rot = [np.cos(float(i) / 2.0), a, a, a]
            msg.transform.rotation.x = rot[1]
            msg.transform.rotation.y = rot[2]
            msg.transform.rotation.z = rot[3]
            msg.transform.rotation.w = rot[0]

            tf_msg = TFMessage()
            tf_msg.transforms.append(msg)

            test_pub.publish(tf_msg)

            time.sleep(0.01)

            await simulate_async(1.0, 60)
            await omni.kit.app.get_app().next_update_async()

            x, r = get_world_pose("/nova_carter/chassis_link")

            # NOTE : a quaterion q and -q represent the same rotation
            self.assertTrue(np.linalg.norm(x - pos) < 1e-5)
            self.assertTrue(np.linalg.norm(r - rot) < 1e-5 or np.linalg.norm(r + rot) < 1e-5)

        self._timeline.stop()

        pass
