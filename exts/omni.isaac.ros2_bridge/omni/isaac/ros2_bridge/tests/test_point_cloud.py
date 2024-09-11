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
import numpy as np
import omni.graph.core as og

# Import extension python module we are testing with absolute import path, as if we are external user (other extension)
import omni.kit.commands

# NOTE:
#   omni.kit.test - std python's unittest module with additional wrapping to add suport for async/await tests
#   For most things refer to unittest docs: https://docs.python.org/3/library/unittest.html
import omni.kit.test
import omni.kit.usd
import omni.kit.viewport.utility
import usdrt.Sdf
from omni.isaac.core.utils.physics import simulate_async
from omni.isaac.core.utils.stage import open_stage_async
from omni.isaac.nucleus import get_assets_root_path_async
from pxr import Gf, Sdf

from .common import add_carter, add_carter_ros, add_cube, fields_to_dtype, get_qos_profile


# Having a test class dervived from omni.kit.test.AsyncTestCase declared on the root of module will make it auto-discoverable by omni.kit.test
class TestRos2PointCloud(omni.kit.test.AsyncTestCase):
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

    async def test_3D_point_cloud(self):
        import rclpy
        from sensor_msgs.msg import PointCloud2

        await add_carter()
        await add_cube("/cube", 0.80, (1.60, 0.10, 0.50))

        # Add Point Cloud publisher
        graph_path = "/ActionGraph"

        try:
            keys = og.Controller.Keys
            (graph, nodes, _, _) = og.Controller.edit(
                {"graph_path": graph_path, "evaluator_name": "execution"},
                {
                    keys.CREATE_NODES: [
                        ("OnPlaybackTick", "omni.graph.action.OnPlaybackTick"),
                        ("ReadSimTime", "omni.isaac.core_nodes.IsaacReadSimulationTime"),
                        # Added nodes used for Point Cloud Publisher
                        ("ReadLidarPCL", "omni.isaac.range_sensor.IsaacReadLidarPointCloud"),
                        ("PublishPCL", "omni.isaac.ros2_bridge.ROS2PublishPointCloud"),
                    ],
                    keys.SET_VALUES: [
                        ("ReadLidarPCL.inputs:lidarPrim", [usdrt.Sdf.Path("/carter/chassis_link/carter_lidar")])
                    ],
                    keys.CONNECT: [
                        ("OnPlaybackTick.outputs:tick", "ReadLidarPCL.inputs:execIn"),
                        ("ReadLidarPCL.outputs:execOut", "PublishPCL.inputs:execIn"),
                        ("ReadLidarPCL.outputs:data", "PublishPCL.inputs:data"),
                        ("ReadSimTime.outputs:simulationTime", "PublishPCL.inputs:timeStamp"),
                    ],
                },
            )
        except Exception as e:
            print(e)

        # Enable highLod for Lidar
        omni.kit.commands.execute(
            "ChangeProperty", prop_path=Sdf.Path("/carter/chassis_link/carter_lidar.highLod"), value=True, prev=None
        )

        self._point_cloud_data = None

        def point_cloud_callback(data: PointCloud2):
            self._point_cloud_data = data

        node = rclpy.create_node("point_cloud_tester")
        lidar_sub = node.create_subscription(PointCloud2, "point_cloud", point_cloud_callback, get_qos_profile())

        def spin():
            rclpy.spin_once(node, timeout_sec=0.1)

        self._timeline.play()
        await omni.kit.app.get_app().next_update_async()
        await simulate_async(1, 60, spin)

        # If 3D point cloud (highLOD enabled)
        self.assertIsNotNone(self._point_cloud_data)
        self.assertEqual(self._point_cloud_data.height, 1)
        self.assertGreater(self._point_cloud_data.width, 1)
        self.assertEqual(
            self._point_cloud_data.row_step / self._point_cloud_data.point_step, self._point_cloud_data.width
        )
        self.assertEqual(
            len(self._point_cloud_data.data) / self._point_cloud_data.row_step, self._point_cloud_data.height
        )

        ff = fields_to_dtype(self._point_cloud_data.fields, self._point_cloud_data.point_step)
        arr = np.frombuffer(self._point_cloud_data.data, ff)

        self.assertAlmostEqual(arr[100][0], -45.083733, delta=0.01)
        self.assertAlmostEqual(arr[100][1], -7.949485, delta=0.01)
        self.assertAlmostEqual(arr[100][2], -0.7990794, delta=0.01)
        self.assertEqual(self._point_cloud_data.fields[0].datatype, 7)
        self.assertEqual(self._point_cloud_data.fields[1].datatype, 7)
        self.assertEqual(self._point_cloud_data.fields[2].datatype, 7)

        self._timeline.stop()
        spin()

        pass

    async def test_flat_point_cloud(self):
        import rclpy
        from sensor_msgs.msg import PointCloud2

        await add_carter()
        await add_cube("/cube", 0.80, (1.60, 0.10, 0.50))

        # Add Point Cloud publisher
        graph_path = "/ActionGraph"

        try:
            keys = og.Controller.Keys
            (graph, nodes, _, _) = og.Controller.edit(
                {"graph_path": graph_path, "evaluator_name": "execution"},
                {
                    keys.CREATE_NODES: [
                        ("OnPlaybackTick", "omni.graph.action.OnPlaybackTick"),
                        ("ReadSimTime", "omni.isaac.core_nodes.IsaacReadSimulationTime"),
                        # Added nodes used for Point Cloud Publisher
                        ("ReadLidarPCL", "omni.isaac.range_sensor.IsaacReadLidarPointCloud"),
                        ("PublishPCL", "omni.isaac.ros2_bridge.ROS2PublishPointCloud"),
                    ],
                    keys.SET_VALUES: [
                        ("ReadLidarPCL.inputs:lidarPrim", [usdrt.Sdf.Path("/carter/chassis_link/carter_lidar")])
                    ],
                    keys.CONNECT: [
                        ("OnPlaybackTick.outputs:tick", "ReadLidarPCL.inputs:execIn"),
                        ("ReadLidarPCL.outputs:execOut", "PublishPCL.inputs:execIn"),
                        ("ReadLidarPCL.outputs:data", "PublishPCL.inputs:data"),
                        ("ReadSimTime.outputs:simulationTime", "PublishPCL.inputs:timeStamp"),
                    ],
                },
            )
        except Exception as e:
            print(e)

        self._point_cloud_data = None

        def point_cloud_callback(data: PointCloud2):
            self._point_cloud_data = data

        node = rclpy.create_node("flat_point_cloud_tester")
        lidar_sub = node.create_subscription(PointCloud2, "point_cloud", point_cloud_callback, get_qos_profile())

        def spin():
            rclpy.spin_once(node, timeout_sec=0.1)

        self._timeline.play()
        await omni.kit.app.get_app().next_update_async()
        await simulate_async(1, 60, spin)

        # If flat point cloud (highLOD disabled)
        self.assertIsNotNone(self._point_cloud_data)
        self.assertEqual(self._point_cloud_data.height, 1)
        self.assertGreater(self._point_cloud_data.width, 1)
        self.assertEqual(len(self._point_cloud_data.data), self._point_cloud_data.row_step)
        self.assertEqual(
            self._point_cloud_data.row_step / self._point_cloud_data.point_step, self._point_cloud_data.width
        )

        ff = fields_to_dtype(self._point_cloud_data.fields, self._point_cloud_data.point_step)
        arr = np.frombuffer(self._point_cloud_data.data, ff)

        self.assertAlmostEqual(arr[50][0], 1.257611, delta=0.01)
        self.assertAlmostEqual(arr[50][1], 0.149961, delta=0.01)
        self.assertAlmostEqual(arr[50][2], -0.000000, delta=0.01)

        self.assertEqual(self._point_cloud_data.fields[0].datatype, 7)
        self.assertEqual(self._point_cloud_data.fields[1].datatype, 7)
        self.assertEqual(self._point_cloud_data.fields[2].datatype, 7)

        self._timeline.stop()
        spin()

        pass

    async def test_depth_to_point_cloud(self):
        import rclpy
        from sensor_msgs.msg import PointCloud2

        await add_carter_ros()
        await add_cube("/cube", 0.80, (1.60, 0.10, 0.50))

        graph_path = "/Carter/ROS_Cameras"

        # Disabling left camera rgb image publisher
        og.Controller.attribute(graph_path + "/isaac_create_render_product_left.inputs:enabled").set(False)

        # Add Point Cloud publisher in ROS Camera
        try:
            keys = og.Controller.Keys
            (graph, nodes, _, _) = og.Controller.edit(
                graph_path,
                {
                    keys.CREATE_NODES: [("depthToPCL", "omni.isaac.ros2_bridge.ROS2CameraHelper")],
                    keys.CONNECT: [
                        (graph_path + "/isaac_create_render_product_left.outputs:execOut", "depthToPCL.inputs:execIn"),
                        (graph_path + "/camera_frameId_left.inputs:value", "depthToPCL.inputs:frameId"),
                        (
                            graph_path + "/isaac_create_render_product_left.outputs:renderProductPath",
                            "depthToPCL.inputs:renderProductPath",
                        ),
                    ],
                    og.Controller.Keys.SET_VALUES: [
                        ("depthToPCL.inputs:topicName", "/point_cloud_left"),
                        ("depthToPCL.inputs:type", "depth_pcl"),
                    ],
                },
            )
        except Exception as e:
            print(e)

        # Enable left camera pipeline
        og.Controller.set(
            og.Controller.attribute(graph_path + "/isaac_create_render_product_left.inputs:enabled"), True
        )

        # acquire the viewport window
        viewport_api = omni.kit.viewport.utility.get_active_viewport()
        # Set viewport resolution, changes will occur on next frame
        viewport_api.set_texture_resolution((1280, 720))

        self._point_cloud_data = None

        def point_cloud_callback(data: PointCloud2):
            self._point_cloud_data = data

        node = rclpy.create_node("depth_point_cloud_tester")
        camera_sub = node.create_subscription(PointCloud2, "point_cloud_left", point_cloud_callback, get_qos_profile())

        def spin():
            rclpy.spin_once(node, timeout_sec=0.1)

        self._timeline.play()
        await omni.kit.app.get_app().next_update_async()
        await simulate_async(1, 60, spin)

        self.assertIsNotNone(self._point_cloud_data)
        self.assertGreater(self._point_cloud_data.width, 1)
        self.assertEqual(
            self._point_cloud_data.row_step / self._point_cloud_data.point_step, self._point_cloud_data.width
        )
        self.assertEqual(
            len(self._point_cloud_data.data) / self._point_cloud_data.row_step, self._point_cloud_data.height
        )

        self.assertEqual(self._point_cloud_data.data[526327], 190)
        self.assertEqual(self._point_cloud_data.data[712187], 63)
        self.assertEqual(self._point_cloud_data.fields[0].datatype, 7)
        self.assertEqual(self._point_cloud_data.fields[1].datatype, 7)
        self.assertEqual(self._point_cloud_data.fields[2].datatype, 7)

        self._timeline.stop()
        spin()

        pass

    async def test_flat_point_cloud_buffer(self):
        import rclpy
        from sensor_msgs.msg import PointCloud2

        (result, error) = await open_stage_async(
            self._assets_root_path + "/Isaac/Environments/Simple_Room/simple_room.usd"
        )

        # Make sure the stage loaded
        stage = omni.usd.get_context().get_stage()
        await omni.kit.app.get_app().next_update_async()

        HORIZONTAL_FOV = 360.0

        HORIZONTAL_RESOLUTION = 0.4

        # Add lidar
        result, lidar = omni.kit.commands.execute(
            "RangeSensorCreateLidar",
            path="/World/Lidar",
            parent=None,
            min_range=0.4,
            max_range=100.0,
            draw_points=True,
            draw_lines=True,
            horizontal_fov=HORIZONTAL_FOV,
            vertical_fov=30.0,
            horizontal_resolution=HORIZONTAL_RESOLUTION,
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

        # Add Point Cloud publisher
        graph_path = "/ActionGraph"

        try:
            keys = og.Controller.Keys
            (graph, nodes, _, _) = og.Controller.edit(
                {"graph_path": graph_path, "evaluator_name": "execution"},
                {
                    keys.CREATE_NODES: [
                        ("OnPlaybackTick", "omni.graph.action.OnPlaybackTick"),
                        ("ReadSimTime", "omni.isaac.core_nodes.IsaacReadSimulationTime"),
                        # Added nodes used for Point Cloud Publisher
                        ("ReadLidarPCL", "omni.isaac.range_sensor.IsaacReadLidarPointCloud"),
                        ("PublishPCL", "omni.isaac.ros2_bridge.ROS2PublishPointCloud"),
                        ("CameraHelper", "omni.isaac.ros2_bridge.ROS2CameraHelper"),
                    ],
                    keys.SET_VALUES: [
                        ("ReadLidarPCL.inputs:lidarPrim", [usdrt.Sdf.Path(lidarPath)]),
                        ("CameraHelper.inputs:renderProductPath", render_product_path),
                    ],
                    keys.CONNECT: [
                        ("OnPlaybackTick.outputs:tick", "ReadLidarPCL.inputs:execIn"),
                        ("ReadLidarPCL.outputs:execOut", "PublishPCL.inputs:execIn"),
                        ("ReadLidarPCL.outputs:data", "PublishPCL.inputs:data"),
                        ("OnPlaybackTick.outputs:tick", "CameraHelper.inputs:execIn"),
                        ("ReadSimTime.outputs:simulationTime", "PublishPCL.inputs:timeStamp"),
                    ],
                },
            )
        except Exception as e:
            print(e)

        self._point_cloud_data = None

        def point_cloud_callback(data: PointCloud2):
            self._point_cloud_data = data

        node = rclpy.create_node("depth_point_cloud_tester")
        camera_sub = node.create_subscription(PointCloud2, "point_cloud", point_cloud_callback, get_qos_profile())

        def spin():
            rclpy.spin_once(node, timeout_sec=0.1)

        def standard_checks():
            # If flat point cloud (highLOD disabled)
            self.assertIsNotNone(self._point_cloud_data)
            self.assertEqual(self._point_cloud_data.height, 1)
            self.assertGreater(self._point_cloud_data.width, 1)
            self.assertEqual(len(self._point_cloud_data.data), self._point_cloud_data.row_step)
            self.assertEqual(
                self._point_cloud_data.row_step / self._point_cloud_data.point_step, self._point_cloud_data.width
            )

            ff = fields_to_dtype(self._point_cloud_data.fields, self._point_cloud_data.point_step)
            arr = np.frombuffer(self._point_cloud_data.data, ff)

            self.assertEqual(self._point_cloud_data.fields[0].datatype, 7)
            self.assertEqual(self._point_cloud_data.fields[1].datatype, 7)
            self.assertEqual(self._point_cloud_data.fields[2].datatype, 7)

            def fix_data(arr):
                dat = []
                for x in arr:
                    dat.append((x[0], x[1], x[2]))
                return dat

            arr = fix_data(arr)

            # Check to see if number of points matches number of beams in full scan
            self.assertEqual(len(set(arr)), HORIZONTAL_FOV / HORIZONTAL_RESOLUTION)

        # Check with lidar 0.0 rotation rate
        omni.kit.commands.execute(
            "ChangeProperty",
            prop_path=Sdf.Path(lidarPath + ".rotationRate"),
            value=0.0,
            prev=None,
        )

        self._timeline.play()
        await omni.kit.app.get_app().next_update_async()
        await simulate_async(2.0, 60, spin)

        standard_checks()

        self._timeline.stop()
        await omni.kit.app.get_app().next_update_async()

        # 21.0 Hz Lidar rotation
        omni.kit.commands.execute(
            "ChangeProperty",
            prop_path=Sdf.Path(lidarPath + ".rotationRate"),
            value=21.0,
            prev=None,
        )

        self._timeline.play()
        await omni.kit.app.get_app().next_update_async()
        await omni.kit.app.get_app().next_update_async()
        await simulate_async(2.0, 60, spin)

        standard_checks()
        self._timeline.stop()
        spin()
        pass
