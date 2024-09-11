# Copyright (c) 2018-2023, NVIDIA CORPORATION. All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto. Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.
#

import asyncio
import copy
import gc

import carb

# Import extension python module we are testing with absolute import path, as if we are external user (other extension)
import omni.kit.commands

# NOTE:
#   omni.kit.test - std python's unittest module with additional wrapping to add suport for async/await tests
#   For most things refer to unittest docs: https://docs.python.org/3/library/unittest.html
import omni.kit.test
import omni.kit.usd
import usdrt.Sdf
from omni.isaac.core.utils.physics import simulate_async
from omni.isaac.core.utils.stage import open_stage_async
from omni.isaac.nucleus import get_assets_root_path_async
from pxr import Gf, Sdf

from .common import add_carter_ros, add_cube, get_qos_profile


# Having a test class dervived from omni.kit.test.AsyncTestCase declared on the root of module will make it auto-discoverable by omni.kit.test
class TestRos2Lidar(omni.kit.test.AsyncTestCase):
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

    # TODO: Carter V1 uses RTX lidar now
    # async def test_lidar(self):
    #     import rclpy
    #     from sensor_msgs.msg import LaserScan

    #     await add_carter_ros()
    #     await add_cube("/cube", 0.75, (2.00, 0, 0.75))

    #     self._lidar_data = None
    #     self._lidar_data_prev = None

    #     def lidar_callback(data: LaserScan):
    #         self._lidar_data = data

    #     node = rclpy.create_node("lidar_tester")
    #     subscriber = node.create_subscription(LaserScan, "scan", lidar_callback, get_qos_profile())

    #     def standard_checks():
    #         self.assertIsNotNone(self._lidar_data)
    #         self.assertGreater(self._lidar_data.angle_max, self._lidar_data.angle_min)
    #         self.assertEqual(self._lidar_data.intensities[0], 0.0)
    #         self.assertEqual(len(self._lidar_data.intensities), 900)
    #         self.assertEqual(self._lidar_data.intensities[450], 255.0)

    #     omni.kit.commands.execute(
    #         "ChangeProperty", prop_path=Sdf.Path("/Carter/chassis_link/carter_lidar.rotationRate"), value=0.0, prev=None
    #     )

    #     def spin():
    #         rclpy.spin_once(node, timeout_sec=0.1)

    #     # 0.0 Hz Lidar rotation
    #     self._timeline.play()
    #     await omni.kit.app.get_app().next_update_async()
    #     await omni.kit.app.get_app().next_update_async()
    #     await simulate_async(1, 60, spin)

    #     standard_checks()
    #     self.assertEqual(self._lidar_data.time_increment, 0)

    #     self._timeline.stop()
    #     await omni.kit.app.get_app().next_update_async()

    #     self._lidar_data_prev = copy.deepcopy(self._lidar_data)
    #     self._lidar_data = None

    #     omni.kit.commands.execute(
    #         "ChangeProperty",
    #         prop_path=Sdf.Path("/Carter/chassis_link/carter_lidar.rotationRate"),
    #         value=121.0,
    #         prev=None,
    #     )

    #     await omni.kit.app.get_app().next_update_async()
    #     # 121.0 Hz Lidar rotation
    #     self._timeline.play()
    #     await omni.kit.app.get_app().next_update_async()
    #     await omni.kit.app.get_app().next_update_async()
    #     await simulate_async(1, 60, spin)
    #     spin()

    #     standard_checks()

    #     self.assertGreater(self._lidar_data.header.stamp.sec, self._lidar_data_prev.header.stamp.sec)
    #     self.assertEqual(len(self._lidar_data.intensities), len(self._lidar_data_prev.intensities))
    #     self.assertEqual(self._lidar_data.intensities, self._lidar_data_prev.intensities)
    #     self.assertGreater(self._lidar_data.time_increment, 0.0)

    #     self._timeline.stop()
    #     await omni.kit.app.get_app().next_update_async()

    #     self._lidar_data_prev = copy.deepcopy(self._lidar_data)
    #     self._lidar_data = None

    #     omni.kit.commands.execute(
    #         "ChangeProperty",
    #         prop_path=Sdf.Path("/Carter/chassis_link/carter_lidar.rotationRate"),
    #         value=201.0,
    #         prev=None,
    #     )

    #     # 201.0 Hz Lidar rotation
    #     self._timeline.play()
    #     await omni.kit.app.get_app().next_update_async()
    #     await omni.kit.app.get_app().next_update_async()

    #     await simulate_async(1, 60, spin)

    #     standard_checks()

    #     self.assertGreater(self._lidar_data.header.stamp.sec, self._lidar_data_prev.header.stamp.sec)
    #     self.assertEqual(len(self._lidar_data.intensities), len(self._lidar_data_prev.intensities))
    #     self.assertEqual(self._lidar_data.intensities, self._lidar_data_prev.intensities)

    #     self.assertGreater(self._lidar_data_prev.time_increment, self._lidar_data.time_increment)

    #     self._timeline.stop()
    #     spin()

    #     pass

    async def test_lidar_buffer(self):
        # Test Lidar buffer with replicator activated
        import omni.graph.core as og
        import rclpy
        from sensor_msgs.msg import LaserScan

        (result, error) = await open_stage_async(
            self._assets_root_path + "/Isaac/Environments/Simple_Room/simple_room.usd"
        )

        # Make sure the stage loaded
        stage = omni.usd.get_context().get_stage()
        await omni.kit.app.get_app().next_update_async()

        # Add lidar
        result, lidar = omni.kit.commands.execute(
            "RangeSensorCreateLidar",
            path="/World/Lidar",
            parent=None,
            min_range=0.4,
            max_range=100.0,
            draw_points=True,
            draw_lines=True,
            horizontal_fov=360.0,
            vertical_fov=30.0,
            horizontal_resolution=0.4,
            vertical_resolution=4.0,
            rotation_rate=0.0,
            high_lod=False,
            yaw_offset=0.0,
            enable_semantics=True,
        )
        lidarPath = str(lidar.GetPath())
        lidar.GetPrim().GetAttribute("xformOp:translate").Set(Gf.Vec3d(0.0, -0.5, 0.5))

        # Setup a camera_helper to activate replicator to test PhysX Lidar buffer
        viewport_api = omni.kit.viewport.utility.get_active_viewport()
        render_product_path = viewport_api.get_render_product_path()
        # Set viewport resolution, changes will occur on next frame
        viewport_api.set_texture_resolution((1280, 720))
        await omni.kit.app.get_app().next_update_async()

        try:
            keys = og.Controller.Keys
            (graph, nodes, _, _) = og.Controller.edit(
                {"graph_path": "/ActionGraph", "evaluator_name": "execution"},
                {
                    keys.CREATE_NODES: [
                        ("OnTick", "omni.graph.action.OnTick"),
                        ("LidarLaserScanNode", "omni.isaac.range_sensor.IsaacReadLidarBeams"),
                        ("LaserScanPublisher", "omni.isaac.ros2_bridge.ROS2PublishLaserScan"),
                        ("CameraHelper", "omni.isaac.ros2_bridge.ROS2CameraHelper"),
                    ],
                    og.Controller.Keys.SET_VALUES: [
                        ("LidarLaserScanNode.inputs:lidarPrim", [usdrt.Sdf.Path(lidarPath)]),
                        ("CameraHelper.inputs:renderProductPath", render_product_path),
                    ],
                    keys.CONNECT: [
                        ("OnTick.outputs:tick", "LidarLaserScanNode.inputs:execIn"),
                        ("OnTick.outputs:tick", "CameraHelper.inputs:execIn"),
                        ("LidarLaserScanNode.outputs:execOut", "LaserScanPublisher.inputs:execIn"),
                        ("LidarLaserScanNode.outputs:azimuthRange", "LaserScanPublisher.inputs:azimuthRange"),
                        ("LidarLaserScanNode.outputs:depthRange", "LaserScanPublisher.inputs:depthRange"),
                        ("LidarLaserScanNode.outputs:horizontalFov", "LaserScanPublisher.inputs:horizontalFov"),
                        (
                            "LidarLaserScanNode.outputs:horizontalResolution",
                            "LaserScanPublisher.inputs:horizontalResolution",
                        ),
                        ("LidarLaserScanNode.outputs:intensitiesData", "LaserScanPublisher.inputs:intensitiesData"),
                        ("LidarLaserScanNode.outputs:linearDepthData", "LaserScanPublisher.inputs:linearDepthData"),
                        ("LidarLaserScanNode.outputs:numCols", "LaserScanPublisher.inputs:numCols"),
                        ("LidarLaserScanNode.outputs:numRows", "LaserScanPublisher.inputs:numRows"),
                        ("LidarLaserScanNode.outputs:rotationRate", "LaserScanPublisher.inputs:rotationRate"),
                    ],
                },
            )
        except Exception as e:
            print(e)

        self._lidar_data = None
        self._lidar_data_prev = None

        def lidar_callback(data: LaserScan):
            self._lidar_data = data

        node = rclpy.create_node("lidar_tester")
        subscriber = node.create_subscription(LaserScan, "scan", lidar_callback, get_qos_profile())

        def standard_checks():
            self.assertIsNotNone(self._lidar_data)
            self.assertGreater(self._lidar_data.angle_max, self._lidar_data.angle_min)
            self.assertEqual(len(self._lidar_data.intensities), 900)

        omni.kit.commands.execute(
            "ChangeProperty", prop_path=Sdf.Path(lidarPath + ".rotation_rate"), value=0.0, prev=None
        )

        def spin():
            rclpy.spin_once(node, timeout_sec=0.1)

        # 0.0 Hz Lidar rotation
        self._timeline.play()
        await omni.kit.app.get_app().next_update_async()
        await omni.kit.app.get_app().next_update_async()
        await simulate_async(2.0, 60, spin)

        standard_checks()
        self.assertEqual(self._lidar_data.time_increment, 0)

        self._timeline.stop()
        await omni.kit.app.get_app().next_update_async()

        self._lidar_data_prev = copy.deepcopy(self._lidar_data)
        self._lidar_data = None

        # 21.0 Hz Lidar rotation
        omni.kit.commands.execute(
            "ChangeProperty",
            prop_path=Sdf.Path(lidarPath + ".rotationRate"),
            value=21.0,
            prev=None,
        )

        await omni.kit.app.get_app().next_update_async()
        self._timeline.play()
        await omni.kit.app.get_app().next_update_async()
        await omni.kit.app.get_app().next_update_async()
        await simulate_async(2.0, 60, spin)

        standard_checks()

        self.assertEqual(len(self._lidar_data.intensities), len(self._lidar_data_prev.intensities))
        self.assertEqual(self._lidar_data.intensities, self._lidar_data_prev.intensities)
        self.assertGreater(self._lidar_data.time_increment, 0.0)

        self.assertEqual(len(self._lidar_data.ranges), len(self._lidar_data_prev.ranges))
        self.assertEqual(self._lidar_data.ranges, self._lidar_data_prev.ranges)

        self._timeline.stop()
        await omni.kit.app.get_app().next_update_async()

        self._lidar_data_prev = copy.deepcopy(self._lidar_data)
        self._lidar_data = None

        # 201.0 Hz Lidar rotation
        omni.kit.commands.execute(
            "ChangeProperty",
            prop_path=Sdf.Path(lidarPath + ".rotationRate"),
            value=201.0,
            prev=None,
        )

        self._timeline.play()
        await omni.kit.app.get_app().next_update_async()
        await omni.kit.app.get_app().next_update_async()
        await simulate_async(2.0, 60, spin)

        standard_checks()

        self.assertEqual(len(self._lidar_data.intensities), len(self._lidar_data_prev.intensities))
        self.assertEqual(self._lidar_data.intensities, self._lidar_data_prev.intensities)

        self.assertGreater(self._lidar_data_prev.time_increment, self._lidar_data.time_increment)

        self.assertEqual(len(self._lidar_data.ranges), len(self._lidar_data_prev.ranges))
        self.assertEqual(self._lidar_data.ranges, self._lidar_data_prev.ranges)

        self._timeline.stop()
        spin()
        pass
