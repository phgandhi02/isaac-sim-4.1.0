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

import carb
import carb.tokens
import numpy as np
import omni
import omni.hydratexture
import omni.isaac.core.utils.numpy.rotations as rot_utils
import omni.kit
import omni.kit.commands
import omni.kit.test
import omni.replicator.core as rep
import omni.usd
from omni.isaac.core.objects import VisualCuboid
from omni.isaac.core.utils.prims import delete_prim
from omni.isaac.core.utils.stage import create_new_stage_async, update_stage_async
from omni.isaac.sensor import LidarRtx


# Having a test class dervived from omni.kit.test.AsyncTestCase declared on the root of module will make it auto-discoverable by omni.kit.test
class TestRTXSolidStateLidar(omni.kit.test.AsyncTestCase):
    # Before running each test
    async def setUp(self):
        self._settings = carb.settings.acquire_settings_interface()
        await create_new_stage_async()

        await update_stage_async()
        # This needs to be set so that kit updates match physics updates
        self._physics_rate = 60
        self._sensor_rate = 120
        self._settings.set_bool("/app/runLoops/main/rateLimitEnabled", True)
        self._settings.set_int("/app/runLoops/main/rateLimitFrequency", int(self._physics_rate))
        self._settings.set_int("/persistent/simulation/minFrameRate", int(self._physics_rate))

        pass

    async def tearDown(self):
        await omni.kit.app.get_app().next_update_async()
        while omni.usd.get_context().get_stage_loading_status()[2] > 0:
            # print("tearDown, assets still loading, waiting to finish...")
            await asyncio.sleep(1.0)
        await omni.kit.app.get_app().next_update_async()

        self._settings = None

    async def test_rtx_solid_state_lidar_point_cloud(self):
        VisualCuboid(prim_path="/World/cube1", position=np.array([5, 0, 0]), scale=np.array([1, 20, 1]))
        VisualCuboid(prim_path="/World/cube2", position=np.array([-5, 0, 0]), scale=np.array([1, 20, 1]))
        VisualCuboid(prim_path="/World/cube3", position=np.array([0, 5, 0]), scale=np.array([20, 1, 1]))
        VisualCuboid(prim_path="/World/cube4", position=np.array([0, -5, 0]), scale=np.array([20, 1, 1]))
        await update_stage_async()

        config = "Example_Solid_State"
        _, sensor = omni.kit.commands.execute("IsaacSensorCreateRtxLidar", path="/sensor", parent=None, config=config)
        texture = rep.create.render_product(sensor.GetPath().pathString, resolution=[1, 1])
        render_product_path = texture.path
        rv = "RtxLidar"
        writer = rep.writers.get(rv + "DebugDrawPointCloud")
        writer.attach([render_product_path])
        await update_stage_async()
        await update_stage_async()
        omni.timeline.get_timeline_interface().play()
        await omni.syntheticdata.sensors.next_render_simulation_async(render_product_path, 60)
        # cleanup and shutdown
        omni.timeline.get_timeline_interface().stop()
        writer.detach()
        await update_stage_async()
        delete_prim(sensor.GetPath())
        await update_stage_async()
        texture.destroy()

    async def test_rtx_solid_state_lidar_point_cloud_in_cube(self):
        """
        Tests RTX lidar point cloud returns correct range for all azimuth/elevation pairs across multiple frames.
        """
        from math import floor

        # Create a cube of specified edge length
        edge_length = 10.0
        VisualCuboid(prim_path="/World/cube", position=np.array([0, 0, 0]), scale=edge_length * np.ones(3))
        await update_stage_async()

        # Place RTX lidar in the cube, automatically creating point cloud and flat scan annotators
        sensor = LidarRtx(
            prim_path="/sensor",
            position=np.array([0.0, 0.0, 0.0]),
            orientation=rot_utils.euler_angles_to_quats(np.array([0, 0, 0]), degrees=True),
            config_file_name="Example_Solid_State",
        )
        sensor.initialize()
        sensor.add_range_data_to_frame()
        sensor.add_elevation_data_to_frame()
        sensor.add_azimuth_data_to_frame()
        await update_stage_async()

        omni.timeline.get_timeline_interface().play()
        for _ in range(6):
            await omni.kit.app.get_app().next_update_async()

            frame = sensor.get_current_frame()
            num_points = len(frame["range"])
            self.assertEqual(num_points, len(frame["elevation"]))
            self.assertEqual(num_points, len(frame["azimuth"]))

            for p in range(num_points):
                az = frame["azimuth"][p]
                el = frame["elevation"][p]
                r = frame["range"][p]

                # Adjust azimuth to appropriate angle in [-45, 45], then compute expected range to face, edge, or corner of cube
                az_adj = az + (1 - floor((3 * np.pi / 4.0 + az) / (np.pi / 2.0))) * np.pi / 2.0
                range_expected = edge_length / (2.0 * np.cos(az_adj) * np.cos(el))

                self.assertAlmostEqual(r, range_expected, delta=range_expected * 1e-2)

        omni.timeline.get_timeline_interface().stop()

    async def test_rtx_solid_state_lidar_flat_scan_in_cube(self):
        """
        Tests RTX lidar flat scan returns correct range for all azimuth/elevation pairs across multiple frames.
        """
        from math import floor

        # Create a cube of specified edge length
        edge_length = 10.0
        VisualCuboid(prim_path="/World/cube", position=np.array([0, 0, 0]), scale=edge_length * np.ones(3))

        # Place RTX lidar in the cube, automatically creating point cloud and flat scan annotators
        # For the flat scan test, we're using a solid state lidar config that has a row of evenly-spaced emitters
        sensor = LidarRtx(
            prim_path="/sensor",
            position=np.array([0.0, 0.0, 0.0]),
            orientation=rot_utils.euler_angles_to_quats(np.array([0, 0, 0]), degrees=True),
            config_file_name="Simple_Example_Solid_State",
        )
        sensor.initialize()
        sensor.add_linear_depth_data_to_frame()
        sensor.add_azimuth_range_to_frame()
        sensor.add_horizontal_resolution_to_frame()

        omni.timeline.get_timeline_interface().play()
        for i in range(7):
            await omni.kit.app.get_app().next_update_async()

        frame = sensor.get_current_frame()
        linear_depth_data = frame["linear_depth_data"]
        min_azimuth = frame["azimuth_range"][0]
        max_azimuth = frame["azimuth_range"][1]
        horizontal_resolution = frame["horizontal_resolution"]

        self.assertAlmostEqual(min_azimuth, -1.5, delta=horizontal_resolution)
        self.assertAlmostEqual(max_azimuth, 1.5, delta=horizontal_resolution)
        self.assertAlmostEqual(horizontal_resolution, 1.0)

        bad_beam_count = 0
        for p in range(len(linear_depth_data)):
            depth = linear_depth_data[p]
            if depth < 0:
                bad_beam_count = bad_beam_count + 1
                continue
            az = np.deg2rad(min_azimuth + horizontal_resolution * p)

            # Adjust azimuth to appropriate angle in [-45, 45], then compute expected range to face, edge, or corner of cube
            # Note flat scan projects scan at minimum elevation angle (in this case, -0.32 deg) vertically up along
            # cube face, so we don't need to account for elevation angle when computing expected range.
            az_adj = az + (1 - floor((3 * np.pi / 4.0 + az) / (np.pi / 2.0))) * np.pi / 2.0
            range_expected = edge_length / (2.0 * np.cos(az_adj))

            self.assertAlmostEqual(depth, range_expected, delta=range_expected * 1e-2)
        self.assertLess(bad_beam_count, 2)
        omni.timeline.get_timeline_interface().stop()

    pass
