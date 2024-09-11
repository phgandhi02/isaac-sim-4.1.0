# Copyright (c) 2018-2023, NVIDIA CORPORATION. All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto. Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.
#

# NOTE:
#   omni.kit.test - std python's unittest module with additional wrapping to add suport for async/await tests
#   For most things refer to unittest docs: https://docs.python.org/3/library/unittest.html

import asyncio
import math

import carb.tokens
import numpy as np
import omni
import omni.graph.core as og
import omni.kit
import omni.kit.commands
import omni.kit.test
import omni.replicator.core as rep
import usdrt.Sdf
from omni.isaac.core.utils.physics import simulate_async
from omni.isaac.core.utils.viewports import add_aov_to_viewport

# Import extension python module we are testing with absolute import path, as if we are external user (other extension)
from omni.isaac.sensor import _sensor
from omni.kit.viewport.utility import get_active_viewport
from omni.syntheticdata import sensors
from pxr import Gf, Sdf, Usd, UsdGeom, UsdPhysics

from .common import fields_to_dtype, get_qos_profile


def add_cube(stage, path, scale, offset, physics=False):
    cubeGeom = UsdGeom.Cube.Define(stage, path)
    cubePrim = stage.GetPrimAtPath(path)
    cubeGeom.CreateSizeAttr(1.0)
    cubeGeom.AddTranslateOp().Set(offset)
    cubeGeom.AddScaleOp().Set(scale)
    if physics:
        rigid_api = UsdPhysics.RigidBodyAPI.Apply(cubePrim)
        rigid_api.CreateRigidBodyEnabledAttr(True)

    UsdPhysics.CollisionAPI.Apply(cubePrim)
    return cubePrim


# Having a test class dervived from omni.kit.test.AsyncTestCase declared on the root of module will make it auto-discoverable by omni.kit.test
class TestROS2RTXSensor(omni.kit.test.AsyncTestCase):
    # Before running each test
    async def setUp(self):
        import rclpy

        await omni.usd.get_context().new_stage_async()
        # This needs to be set so that kit updates match physics updates
        self._physics_rate = 60
        self._sensor_rate = 120
        carb.settings.get_settings().set_bool("/app/runLoops/main/rateLimitEnabled", True)
        carb.settings.get_settings().set_int("/app/runLoops/main/rateLimitFrequency", int(self._physics_rate))
        carb.settings.get_settings().set_int("/persistent/simulation/minFrameRate", int(self._physics_rate))
        rclpy.init()

        self._is = _sensor.acquire_imu_sensor_interface()
        self._timeline = omni.timeline.get_timeline_interface()

        ext_manager = omni.kit.app.get_app().get_extension_manager()
        ext_id = ext_manager.get_enabled_extension_id("omni.isaac.ros2_bridge")
        self._extension_path = ext_manager.get_extension_path(ext_id)

        pass

    async def tearDown(self):
        import rclpy

        self._timeline.stop()
        await omni.kit.app.get_app().next_update_async()
        while omni.usd.get_context().get_stage_loading_status()[2] > 0:
            print("tearDown, assets still loading, waiting to finish...")
            await asyncio.sleep(1.0)
        # rospy.signal_shutdown("test_complete")
        self._timeline = None
        rclpy.shutdown()
        pass

    async def test_rtx_lidar_point_cloud(self):
        stage = omni.usd.get_context().get_stage()

        viewport_api = get_active_viewport()
        # in order for the sensor to generate data properly we let the viewport know that it should create a buffer for the associated render variable.
        add_aov_to_viewport(viewport_api, "RtxSensorCpu")

        cube_prim = add_cube(stage, "/World/cube_1", (1, 20, 1), (5, 0, 0), physics=False)
        cube_prim = add_cube(stage, "/World/cube_2", (1, 20, 1), (-5, 0, 0), physics=False)
        cube_prim = add_cube(stage, "/World/cube_3", (20, 1, 1), (0, 5, 0), physics=False)
        cube_prim = add_cube(stage, "/World/cube_4", (20, 1, 1), (0, -5, 0), physics=False)

        await omni.syntheticdata.sensors.next_sensor_data_async(viewport_api)

        writer = rep.writers.get("RtxLidar" + "ROS2PublishPointCloud")
        writer.initialize()
        writer.attach([viewport_api.get_render_product_path()])

        await omni.syntheticdata.sensors.next_sensor_data_async(viewport_api)

        _, sensor = omni.kit.commands.execute("IsaacSensorCreateRtxLidar", path="/sensor", parent=None)
        await omni.kit.app.get_app().next_update_async()
        viewport_api.set_active_camera(sensor.GetPath().pathString)

    async def test_rtx_radar_point_cloud(self):
        stage = omni.usd.get_context().get_stage()

        viewport_api = get_active_viewport()
        # in order for the sensor to generate data properly we let the viewport know that it should create a buffer for the associated render variable.
        add_aov_to_viewport(viewport_api, "RtxSensorCpu")
        render_product_path = viewport_api.get_render_product_path()

        cube_prim = add_cube(stage, "/World/cube_1", (1, 20, 5), (5, 0, 0), physics=False)
        cube_prim = add_cube(stage, "/World/cube_2", (1, 20, 1), (-5, 0, 0), physics=False)
        cube_prim = add_cube(stage, "/World/cube_3", (20, 1, 1), (0, 5, 0), physics=False)
        cube_prim = add_cube(stage, "/World/cube_4", (20, 1, 1), (0, -5, 0), physics=False)

        await omni.syntheticdata.sensors.next_sensor_data_async(viewport_api)

        _, sensor = omni.kit.commands.execute("IsaacSensorCreateRtxRadar", path="/sensor", parent=None)
        viewport_api.set_active_camera(sensor.GetPath().pathString)

        writer = rep.writers.get("RtxRadar" + "ROS2PublishPointCloud")
        writer.initialize()
        writer.attach([render_product_path])
        await omni.kit.app.get_app().next_update_async()
        rv = "RtxRadar"
        writer_2 = rep.writers.get(rv + "DebugDrawPointCloud")
        writer_2.attach([render_product_path])

        await omni.kit.app.get_app().next_update_async()

    pass

    async def test_rtx_lidar(self):
        stage = omni.usd.get_context().get_stage()
        viewport_api = get_active_viewport()
        # in order for the sensor to generate data properly we let the viewport know that it should create a buffer for the associated render variable.
        add_aov_to_viewport(viewport_api, "RtxSensorCpu")
        render_product_path = viewport_api.get_render_product_path()

        config = "Example_Rotary"
        _, sensor = omni.kit.commands.execute(
            "IsaacSensorCreateRtxLidar",
            path="/sim_lidar",
            parent=None,
            config=config,
            orientation=Gf.Quatd(1.0, 0.0, 0.0, 0.0),
        )
        viewport_api.set_active_camera(sensor.GetPath().pathString)
        # in order for the sensor to generate data properly we let the viewport know that it should create a buffer for the associated render variable.
        # add_aov_to_viewport(viewport_api, "RtxSensorCpu")

        cube_prim = add_cube(stage, "/World/cube_1", (1, 20, 1), (5, 0, 0), physics=False)
        cube_prim = add_cube(stage, "/World/cube_2", (1, 20, 1), (-5, 0, 0), physics=False)
        cube_prim = add_cube(stage, "/World/cube_3", (20, 1, 1), (0, 5, 0), physics=False)
        cube_prim = add_cube(stage, "/World/cube_4", (20, 1, 1), (0, -5, 0), physics=False)
        cube_prim = add_cube(stage, "/World/cube_5", (1, 1, 1), (0, -4, 0), physics=False)

        try:
            og.Controller.edit(
                {"graph_path": "/ActionGraph", "evaluator_name": "execution"},
                {
                    og.Controller.Keys.CREATE_NODES: [
                        ("OnPlaybackTick", "omni.graph.action.OnPlaybackTick"),
                        ("PCLPublish", "omni.isaac.ros2_bridge.ROS2RtxLidarHelper"),
                        ("LaserScanPublish", "omni.isaac.ros2_bridge.ROS2RtxLidarHelper"),
                        ("PublishTF", "omni.isaac.ros2_bridge.ROS2PublishTransformTree"),
                        ("SimTime", "omni.isaac.core_nodes.IsaacReadSimulationTime"),
                    ],
                    og.Controller.Keys.SET_VALUES: [
                        ("SimTime.inputs:resetOnStop", True),
                        ("PCLPublish.inputs:renderProductPath", render_product_path),
                        ("PCLPublish.inputs:topicName", "point_cloud"),
                        ("PCLPublish.inputs:type", "point_cloud"),
                        ("PCLPublish.inputs:resetSimulationTimeOnStop", True),
                        ("PCLPublish.inputs:fullScan", False),
                        ("LaserScanPublish.inputs:fullScan", False),
                        ("LaserScanPublish.inputs:renderProductPath", render_product_path),
                        ("LaserScanPublish.inputs:topicName", "laser_scan"),
                        ("LaserScanPublish.inputs:type", "laser_scan"),
                        ("LaserScanPublish.inputs:resetSimulationTimeOnStop", True),
                        ("PublishTF.inputs:targetPrims", [usdrt.Sdf.Path("/sim_lidar")]),
                        ("PublishTF.inputs:parentPrim", [usdrt.Sdf.Path("/sim_lidar")]),
                    ],
                    og.Controller.Keys.CONNECT: [
                        ("OnPlaybackTick.outputs:tick", "PCLPublish.inputs:execIn"),
                        ("OnPlaybackTick.outputs:tick", "PublishTF.inputs:execIn"),
                        ("OnPlaybackTick.outputs:tick", "LaserScanPublish.inputs:execIn"),
                        ("SimTime.outputs:simulationTime", "PublishTF.inputs:timeStamp"),
                    ],
                },
            )
        except Exception as e:
            print(e)

        # enable debug rendering for test purposes
        rv = "RtxLidar"
        writer = rep.writers.get(rv + "DebugDrawPointCloud")
        writer.attach([render_product_path])

        import rclpy

        await omni.kit.app.get_app().next_update_async()

        from sensor_msgs.msg import LaserScan, PointCloud2

        self._pcl_data = None
        self._scan_data = None

        def pcl_callback(data):
            self._pcl_data = data

        def scan_callback(data):
            self._scan_data = data

        node = rclpy.create_node("rtx_lidar_tester")
        pcl_sub = node.create_subscription(PointCloud2, "point_cloud", pcl_callback, get_qos_profile())
        scan_sub = node.create_subscription(LaserScan, "laser_scan", scan_callback, get_qos_profile())

        def spin():
            rclpy.spin_once(node, timeout_sec=0.1)

        await asyncio.sleep(2.0)

        import time

        system_time = time.time()

        self._timeline.play()
        await omni.kit.app.get_app().next_update_async()
        await simulate_async(1.5, callback=spin)
        for _ in range(10):
            if self._pcl_data is None:
                await simulate_async(1, callback=spin)

        self.assertIsNotNone(self._scan_data)
        self.assertIsNotNone(self._pcl_data)

        self.assertGreaterEqual(self._scan_data.header.stamp.sec, 1)
        self.assertLess(self._scan_data.header.stamp.sec, system_time / 2.0)
        self.assertGreaterEqual(self._pcl_data.header.stamp.sec, 1)
        self.assertLess(self._pcl_data.header.stamp.sec, system_time / 2.0)

        # check laser scan
        self.assertGreater(len(self._scan_data.ranges), 10)

        for r in self._scan_data.ranges:
            if r < 0.0:
                continue  # invalid scan
            self.assertGreater(r, 3.49)  # this is the distance to the inset cube
            self.assertLess(r, 6.37)  # should be less than sqrt(4.5&2+4.5^2) ~ 6.3639

        ff = fields_to_dtype(self._pcl_data.fields, self._pcl_data.point_step)
        arr = np.frombuffer(self._pcl_data.data, ff)

        # print(arr)
        self.assertGreater(len(arr), 10)
        for p in arr:
            self.assertGreater(p[2], -4.5)
            self.assertLess(p[2], 4.5)

        self._timeline.stop()
        # make sure all previous messages are cleared
        await omni.kit.app.get_app().next_update_async()
        spin()
        await asyncio.sleep(2.0)

        self._pcl_data = None
        self._scan_data = None

        # Turn on SystemTime for timestamp of all lidar publishers. And retry test.
        og.Controller.attribute("/ActionGraph/PCLPublish" + ".inputs:useSystemTime").set(True)
        og.Controller.attribute("/ActionGraph/LaserScanPublish" + ".inputs:useSystemTime").set(True)

        await omni.kit.app.get_app().next_update_async()
        system_time = time.time()
        self._timeline.play()
        await omni.kit.app.get_app().next_update_async()
        await simulate_async(2, callback=spin)
        for _ in range(10):
            if self._pcl_data is None:
                await simulate_async(1, callback=spin)

        self.assertIsNotNone(self._scan_data)
        self.assertIsNotNone(self._pcl_data)

        self.assertGreaterEqual(self._pcl_data.header.stamp.sec, system_time)
        self.assertGreaterEqual(self._scan_data.header.stamp.sec, system_time)

        # check laser scan
        self.assertGreater(len(self._scan_data.ranges), 10)

        for r in self._scan_data.ranges:
            if r < 0.0:
                continue  # invalid scan
            self.assertGreater(r, 3.49)  # this is the distance to the inset cube
            self.assertLess(r, 6.37)  # should be less than sqrt(4.5&2+4.5^2) ~ 6.3639

        ff = fields_to_dtype(self._pcl_data.fields, self._pcl_data.point_step)
        arr = np.frombuffer(self._pcl_data.data, ff)

        # print(arr)
        self.assertGreater(len(arr), 10)
        for p in arr:
            self.assertGreater(p[2], -4.5)
            self.assertLess(p[2], 4.5)

        self._timeline.stop()

        # make sure all previous messages are cleared
        await omni.kit.app.get_app().next_update_async()
        spin()
        await asyncio.sleep(2.0)

        self._pcl_data = None
        self._scan_data = None

        # Turn off SystemTime for timestamp for lidar point_cloud publisher with full scan enabled.
        # And run test to check timestamp.
        og.Controller.attribute("/ActionGraph/PCLPublish" + ".inputs:useSystemTime").set(False)
        og.Controller.attribute("/ActionGraph/PCLPublish" + ".inputs:fullScan").set(True)

        await omni.kit.app.get_app().next_update_async()
        system_time = time.time()
        self._timeline.play()
        await omni.kit.app.get_app().next_update_async()
        await simulate_async(1.5, callback=spin)
        for _ in range(10):
            if self._pcl_data is None:
                await simulate_async(1, callback=spin)

        self.assertIsNotNone(self._pcl_data)
        self.assertIsNotNone(self._scan_data)

        self.assertGreaterEqual(self._pcl_data.header.stamp.sec, 1)
        self.assertLess(self._pcl_data.header.stamp.sec, system_time / 2.0)

        ff = fields_to_dtype(self._pcl_data.fields, self._pcl_data.point_step)
        arr = np.frombuffer(self._pcl_data.data, ff)

        # print(arr)
        self.assertGreater(len(arr), 10)
        for p in arr:
            self.assertGreater(p[2], -4.5)
            self.assertLess(p[2], 4.5)

        self._timeline.stop()

        # make sure all previous messages are cleared
        await omni.kit.app.get_app().next_update_async()
        spin()
        await asyncio.sleep(2.0)

        self._pcl_data = None
        self._scan_data = None

        # Turn on SystemTime for timestamp for lidar point_cloud publisher with full scan enabled.
        # And run test to check timestamp.
        # Turn off systemTime for laser_scan publisher

        og.Controller.attribute("/ActionGraph/PCLPublish" + ".inputs:useSystemTime").set(True)
        og.Controller.attribute("/ActionGraph/PCLPublish" + ".inputs:fullScan").set(True)

        og.Controller.attribute("/ActionGraph/LaserScanPublish" + ".inputs:useSystemTime").set(False)

        await omni.kit.app.get_app().next_update_async()
        system_time = time.time()
        self._timeline.play()
        await omni.kit.app.get_app().next_update_async()
        await simulate_async(1.5, callback=spin)
        for _ in range(10):
            if self._pcl_data is None:
                await simulate_async(1, callback=spin)

        self.assertIsNotNone(self._pcl_data)
        self.assertIsNotNone(self._scan_data)

        self.assertGreaterEqual(self._pcl_data.header.stamp.sec, system_time)

        self.assertGreaterEqual(self._scan_data.header.stamp.sec, 1)
        self.assertLess(self._scan_data.header.stamp.sec, system_time / 2.0)

        ff = fields_to_dtype(self._pcl_data.fields, self._pcl_data.point_step)
        arr = np.frombuffer(self._pcl_data.data, ff)

        # print(arr)
        self.assertGreater(len(arr), 10)
        for p in arr:
            self.assertGreater(p[2], -4.5)
            self.assertLess(p[2], 4.5)

        node.destroy_node()
