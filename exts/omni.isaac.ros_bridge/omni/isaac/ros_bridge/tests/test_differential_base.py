# Copyright (c) 2018-2024, NVIDIA CORPORATION. All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto. Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.
#

import asyncio
import gc

# NOTE:
#   omni.kit.test - std python's unittest module with additional wrapping to add suport for async/await tests
#   For most things refer to unittest docs: https://docs.python.org/3/library/unittest.html
from re import A, I

import carb
import omni.graph.core as og

# Import extension python module we are testing with absolute import path, as if we are external user (other extension)
import omni.kit.commands
import omni.kit.test
import omni.kit.usd
import usdrt.Sdf
from omni.isaac.core.utils.physics import simulate_async
from omni.isaac.nucleus import get_assets_root_path_async
from pxr import Gf, Sdf

from .common import add_carter, add_carter_ros, set_rotate, set_translate, wait_for_rosmaster_async


# Having a test class dervived from omni.kit.test.AsyncTestCase declared on the root of module will make it auto-discoverable by omni.kit.test
class TestRosDifferentialBase(omni.kit.test.AsyncTestCase):
    # Before running each test
    async def setUp(self):
        import rospy
        from omni.isaac.ros_bridge.scripts.roscore import Roscore

        await omni.usd.get_context().new_stage_async()
        self._timeline = omni.timeline.get_timeline_interface()

        ext_manager = omni.kit.app.get_app().get_extension_manager()
        ext_id = ext_manager.get_enabled_extension_id("omni.isaac.ros_bridge")
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

        self._roscore = Roscore()
        await wait_for_rosmaster_async()
        await omni.kit.app.get_app().next_update_async()

        try:
            rospy.init_node("isaac_sim_test_diff_drive", anonymous=True, disable_signals=True, log_level=rospy.ERROR)
        except rospy.exceptions.ROSException as e:
            print("Node has already been initialized, do nothing")

        pass

    # After running each test
    async def tearDown(self):
        while omni.usd.get_context().get_stage_loading_status()[2] > 0:
            print("tearDown, assets still loading, waiting to finish...")
            await asyncio.sleep(1.0)
        # rospy.signal_shutdown("test_complete")
        self._roscore.shutdown()
        self._roscore = None
        self._timeline = None
        gc.collect()
        pass

    async def test_differential_base(self):
        from copy import deepcopy

        import rospy
        import tf
        from geometry_msgs.msg import Twist
        from nav_msgs.msg import Odometry
        from pxr import UsdGeom

        await add_carter_ros()
        stage = omni.usd.get_context().get_stage()

        meters_per_unit = UsdGeom.GetStageMetersPerUnit(stage)

        # add an odom prim to carter
        odom_prim = stage.DefinePrim("/Carter/chassis_link/odom", "Xform")

        graph_path = "/Carter/ActionGraph"

        # add an tf publisher for world->odom
        try:
            og.Controller.edit(
                graph_path,
                {
                    og.Controller.Keys.CREATE_NODES: [("PublishTF", "omni.isaac.ros_bridge.ROS1PublishTransformTree")],
                    og.Controller.Keys.SET_VALUES: [
                        ("PublishTF.inputs:topicName", "/tf"),
                        ("PublishTF.inputs:targetPrims", [usdrt.Sdf.Path("/Carter/chassis_link/odom")]),
                    ],
                    og.Controller.Keys.CONNECT: [
                        (graph_path + "/on_playback_tick.outputs:tick", "PublishTF.inputs:execIn"),
                        (
                            graph_path + "/isaac_read_simulation_time.outputs:simulationTime",
                            "PublishTF.inputs:timeStamp",
                        ),
                    ],
                },
            )
        except Exception as e:
            print(e)

        # move carter off origin
        carter_prim = stage.GetPrimAtPath("/Carter")
        new_translate = Gf.Vec3d(1.00, -3.00, 0.25)
        new_rotate = Gf.Rotation(Gf.Vec3d(0, 0, 1), 45)
        set_translate(carter_prim, new_translate)
        set_rotate(carter_prim, new_rotate)

        self._odom_data = None

        def odom_callback(data: Odometry):
            self._odom_data = data.pose.pose

        self._tf_listener = tf.TransformListener()

        odom_sub = rospy.Subscriber("/odom", Odometry, odom_callback)
        cmd_vel_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)

        def move_cmd_msg(x, y, z, ax, ay, az):
            msg = Twist()
            msg.linear.x = x
            msg.linear.y = y
            msg.linear.z = z
            msg.angular.x = ax
            msg.angular.y = ay
            msg.angular.z = az
            return msg

        self._timeline.play()
        await omni.kit.app.get_app().next_update_async()
        await simulate_async(2)

        def check_odom_initial_pose():
            odom_data = deepcopy(self._odom_data)
            (tf_trans, tf_rot) = self._tf_listener.lookupTransform("/world", "/odom", rospy.Time())

            self.assertIsNotNone(self._odom_data)
            self.assertAlmostEqual(tf_trans[0], 1.0 * meters_per_unit, 2)
            self.assertAlmostEqual(tf_trans[1], -3.0 * meters_per_unit, 2)
            self.assertAlmostEqual(tf_rot[0], 0, 2)
            self.assertAlmostEqual(tf_rot[1], 0, 2)
            self.assertAlmostEqual(tf_rot[2], 0.38268, 2)
            self.assertAlmostEqual(tf_rot[3], 0.9238, 2)
            self.assertAlmostEqual(odom_data.position.x, 0, 1)
            self.assertAlmostEqual(odom_data.position.y, 0, 1)
            self.assertAlmostEqual(odom_data.orientation.x, 0, 1)
            self.assertAlmostEqual(odom_data.orientation.y, 0, 1)
            self.assertAlmostEqual(odom_data.orientation.z, 0, 1)
            self.assertAlmostEqual(odom_data.orientation.w, 1, 1)

        # check 0: is carter initial tf position and odometry position
        check_odom_initial_pose()

        # straight forward
        move_cmd = move_cmd_msg(0.1, 0.0, 0.0, 0.0, 0.0, 0.0)
        cmd_vel_pub.publish(move_cmd)
        self._timeline.play()
        await omni.kit.app.get_app().next_update_async()
        await simulate_async(3)

        # stop
        move_cmd = move_cmd_msg(0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
        cmd_vel_pub.publish(move_cmd)
        self._timeline.play()
        await omni.kit.app.get_app().next_update_async()
        await simulate_async(2)

        # check 1: location using default param
        odom_data = deepcopy(self._odom_data)
        (tf_trans, tf_rot) = self._tf_listener.lookupTransform("/world", "/odom", rospy.Time())
        self.assertAlmostEqual(round(tf_trans[0], 2), 1.21 * meters_per_unit, 2)
        self.assertAlmostEqual(round(tf_trans[1], 2), -2.79 * meters_per_unit, 2)
        self.assertAlmostEqual(tf_rot[0], 0, 2)
        self.assertAlmostEqual(tf_rot[1], 0, 2)
        self.assertAlmostEqual(tf_rot[2], 0.38268, 1)
        self.assertAlmostEqual(tf_rot[3], 0.9238, 1)
        self.assertAlmostEqual(round(odom_data.position.x, 1), 0.3, 1)
        self.assertAlmostEqual(round(odom_data.position.y, 1), 0, 1)
        self.assertAlmostEqual(odom_data.orientation.x, 0, 1)
        self.assertAlmostEqual(odom_data.orientation.y, 0, 1)
        self.assertAlmostEqual(odom_data.orientation.z, 0, 1)
        self.assertAlmostEqual(odom_data.orientation.w, 1, 1)

        self._timeline.stop()
        await omni.kit.app.get_app().next_update_async()

        # change wheel rotation and wheel base
        og.Controller.set(og.Controller.attribute(graph_path + "/differential_controller.inputs:wheelRadius"), 0.1)
        og.Controller.set(og.Controller.attribute(graph_path + "/differential_controller.inputs:wheelDistance"), 0.5)

        self._timeline.play()
        await omni.kit.app.get_app().next_update_async()
        await simulate_async(1)

        # check 3: is carter initial tf position and odometry position
        check_odom_initial_pose()

        # straight forward
        move_cmd = move_cmd_msg(0.1, 0.0, 0.0, 0.0, 0.0, 0.0)
        cmd_vel_pub.publish(move_cmd)
        self._timeline.play()
        await omni.kit.app.get_app().next_update_async()
        await simulate_async(3)

        # stop
        move_cmd = move_cmd_msg(0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
        cmd_vel_pub.publish(move_cmd)
        self._timeline.play()
        await omni.kit.app.get_app().next_update_async()
        await simulate_async(1)

        # check 4: location after change radius
        odom_data = deepcopy(self._odom_data)
        (tf_trans, tf_rot) = self._tf_listener.lookupTransform("/world", "/odom", rospy.Time())
        self.assertAlmostEqual(round(tf_trans[0], 2), 1.51 * meters_per_unit, 2)
        self.assertAlmostEqual(round(tf_trans[1], 2), -2.49 * meters_per_unit, 2)
        self.assertAlmostEqual(tf_rot[0], 0, 2)
        self.assertAlmostEqual(tf_rot[1], 0, 2)
        self.assertAlmostEqual(tf_rot[2], 0.3846, 1)
        self.assertAlmostEqual(tf_rot[3], 0.9230, 1)
        self.assertAlmostEqual(round(odom_data.position.x, 1), 0.7, 1)
        self.assertAlmostEqual(round(odom_data.position.y, 1), 0, 1)
        self.assertAlmostEqual(odom_data.orientation.x, 0, 1)
        self.assertAlmostEqual(odom_data.orientation.y, 0, 1)
        self.assertAlmostEqual(odom_data.orientation.z, 0, 1)
        self.assertAlmostEqual(odom_data.orientation.w, 1, 1)

        self._timeline.stop()

        odom_sub.unregister()
        cmd_vel_pub.unregister()
        pass

    # add carter and ROS topic from scratch
    async def test_differential_base_scratch(self):
        from copy import deepcopy

        import rospy
        from geometry_msgs.msg import Twist
        from nav_msgs.msg import Odometry

        await add_carter()

        self._odom_data = None

        def odom_callback(data: Odometry):
            self._odom_data = data.pose.pose

        odom_sub = rospy.Subscriber("/odom", Odometry, odom_callback)
        cmd_vel_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)

        def move_cmd_msg(x, y, z, ax, ay, az):
            msg = Twist()
            msg.linear.x = x
            msg.linear.y = y
            msg.linear.z = z
            msg.angular.x = ax
            msg.angular.y = ay
            msg.angular.z = az
            return msg

        graph_path = "/ActionGraph"
        (graph_id, created_nodes) = self.add_differential_drive(graph_path)
        og.Controller.attribute(graph_path + "/diffController.inputs:wheelRadius").set(0.1)

        self._timeline.play()
        await omni.kit.app.get_app().next_update_async()
        await simulate_async(3.0)

        # check 0: is carter initially stationary
        odom_data = deepcopy(self._odom_data)
        self.assertIsNotNone(self._odom_data)
        self.assertAlmostEqual(odom_data.position.x, 0, 1)
        self.assertAlmostEqual(odom_data.position.y, 0, 1)
        self.assertAlmostEqual(odom_data.orientation.x, 0, 1)
        self.assertAlmostEqual(odom_data.orientation.y, 0, 1)
        self.assertAlmostEqual(odom_data.orientation.z, 0, 1)
        self.assertAlmostEqual(odom_data.orientation.w, 1, 1)

        # rotate
        move_cmd = move_cmd_msg(0.0, 0.0, 0.0, 0.0, 0.0, 0.2)
        cmd_vel_pub.publish(move_cmd)
        self._timeline.play()
        await omni.kit.app.get_app().next_update_async()
        await simulate_async(2)

        # stop
        move_cmd = move_cmd_msg(0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
        cmd_vel_pub.publish(move_cmd)
        self._timeline.play()
        await omni.kit.app.get_app().next_update_async()
        await simulate_async(1)

        # check 1: location using default param
        odom_data = deepcopy(self._odom_data)
        self.assertAlmostEqual(odom_data.position.x, 0, 1)
        self.assertAlmostEqual(odom_data.position.y, 0, 1)
        self.assertAlmostEqual(odom_data.orientation.x, 0, 1)
        self.assertAlmostEqual(odom_data.orientation.y, 0, 1)
        self.assertAlmostEqual(odom_data.orientation.z, 0.40, 1)
        self.assertAlmostEqual(odom_data.orientation.w, 0.916, 1)

        # change wheel rotation and wheel base
        omni.kit.commands.execute("DeletePrims", paths=[graph_path])

        (graph_id, created_nodes) = self.add_differential_drive(graph_path)
        og.Controller.attribute(graph_path + "/diffController.inputs:wheelRadius").set(0.1)
        og.Controller.attribute(graph_path + "/diffController.inputs:wheelDistance").set(0.8)

        self._timeline.play()
        await omni.kit.app.get_app().next_update_async()
        await simulate_async(1)

        # rotate back
        move_cmd = move_cmd_msg(0.0, 0.0, 0.0, 0.0, 0.0, -0.2)
        cmd_vel_pub.publish(move_cmd)
        self._timeline.play()
        await omni.kit.app.get_app().next_update_async()
        await simulate_async(2)

        # stop
        move_cmd = move_cmd_msg(0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
        cmd_vel_pub.publish(move_cmd)
        self._timeline.play()
        await omni.kit.app.get_app().next_update_async()
        await simulate_async(1)

        # check 3: location after change radius
        odom_data = deepcopy(self._odom_data)
        self.assertAlmostEqual(odom_data.position.x, 0, 1)
        self.assertAlmostEqual(odom_data.position.y, 0, 1)
        self.assertAlmostEqual(odom_data.orientation.x, 0, 1)
        self.assertAlmostEqual(odom_data.orientation.y, 0, 1)
        self.assertAlmostEqual(odom_data.orientation.z, -0.61, 1)
        self.assertAlmostEqual(odom_data.orientation.w, 0.79, 1)

        self._timeline.stop()

        odom_sub.unregister()
        cmd_vel_pub.unregister()
        pass

    def add_differential_drive(self, graph_path):

        try:
            keys = og.Controller.Keys
            (graph, nodes, _, _) = og.Controller.edit(
                {"graph_path": graph_path, "evaluator_name": "execution"},
                {
                    keys.CREATE_NODES: [
                        ("OnPlaybackTick", "omni.graph.action.OnPlaybackTick"),
                        ("ReadSimTime", "omni.isaac.core_nodes.IsaacReadSimulationTime"),
                        # Added nodes used for Odometry publisher
                        ("computeOdom", "omni.isaac.core_nodes.IsaacComputeOdometry"),
                        ("publishOdom", "omni.isaac.ros_bridge.ROS1PublishOdometry"),
                        ("publishRawTF", "omni.isaac.ros_bridge.ROS1PublishRawTransformTree"),
                        # Added nodes used for Twist subscriber, differential drive
                        ("subscribeTwist", "omni.isaac.ros_bridge.ROS1SubscribeTwist"),
                        ("breakLinVel", "omni.graph.nodes.BreakVector3"),
                        ("breakAngVel", "omni.graph.nodes.BreakVector3"),
                        ("diffController", "omni.isaac.wheeled_robots.DifferentialController"),
                        ("artController", "omni.isaac.core_nodes.IsaacArticulationController"),
                    ],
                    keys.CONNECT: [
                        ("OnPlaybackTick.outputs:tick", "computeOdom.inputs:execIn"),
                        ("OnPlaybackTick.outputs:tick", "publishOdom.inputs:execIn"),
                        ("OnPlaybackTick.outputs:tick", "publishRawTF.inputs:execIn"),
                        ("ReadSimTime.outputs:simulationTime", "publishOdom.inputs:timeStamp"),
                        ("ReadSimTime.outputs:simulationTime", "publishRawTF.inputs:timeStamp"),
                        ("computeOdom.outputs:angularVelocity", "publishOdom.inputs:angularVelocity"),
                        ("computeOdom.outputs:linearVelocity", "publishOdom.inputs:linearVelocity"),
                        ("computeOdom.outputs:orientation", "publishOdom.inputs:orientation"),
                        ("computeOdom.outputs:position", "publishOdom.inputs:position"),
                        ("computeOdom.outputs:orientation", "publishRawTF.inputs:rotation"),
                        ("computeOdom.outputs:position", "publishRawTF.inputs:translation"),
                        ("OnPlaybackTick.outputs:tick", "subscribeTwist.inputs:execIn"),
                        ("OnPlaybackTick.outputs:tick", "artController.inputs:execIn"),
                        ("subscribeTwist.outputs:execOut", "diffController.inputs:execIn"),
                        ("subscribeTwist.outputs:linearVelocity", "breakLinVel.inputs:tuple"),
                        ("breakLinVel.outputs:x", "diffController.inputs:linearVelocity"),
                        ("subscribeTwist.outputs:angularVelocity", "breakAngVel.inputs:tuple"),
                        ("breakAngVel.outputs:z", "diffController.inputs:angularVelocity"),
                        ("diffController.outputs:velocityCommand", "artController.inputs:velocityCommand"),
                    ],
                    og.Controller.Keys.SET_VALUES: [
                        ("diffController.inputs:wheelRadius", 0.24),
                        ("diffController.inputs:wheelDistance", 0.5),
                        ("artController.inputs:jointNames", ["left_wheel", "right_wheel"]),
                        ("computeOdom.inputs:chassisPrim", [usdrt.Sdf.Path("/carter")]),
                        ("artController.inputs:targetPrim", [usdrt.Sdf.Path("/carter")]),
                    ],
                },
            )
        except Exception as e:
            print(e)
        return graph, nodes
