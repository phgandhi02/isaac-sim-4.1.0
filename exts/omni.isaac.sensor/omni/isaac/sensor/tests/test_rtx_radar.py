# Copyright (c) 2024, NVIDIA CORPORATION. All rights reserved.
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
import numpy as np
import omni.kit
import omni.kit.commands
import omni.kit.test
import omni.replicator.core as rep
import omni.usd
from omni.isaac.core.objects import VisualCuboid
from omni.isaac.core.utils.stage import create_new_stage_async, update_stage_async


# Having a test class dervived from omni.kit.test.AsyncTestCase declared on the root of module will make it auto-discoverable by omni.kit.test
class TestRTXRadar(omni.kit.test.AsyncTestCase):
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

    async def test_rtx_radar_point_cloud(self):
        from math import floor

        # Create a cube of specified edge length
        edge_length = 10.0
        VisualCuboid(prim_path="/World/cube", position=np.array([0, 0, 0]), scale=edge_length * np.ones(3))
        await update_stage_async()

        config = "Example"
        _, sensor = omni.kit.commands.execute("IsaacSensorCreateRtxRadar", path="/sensor", parent=None, config=config)
        render_product_path = rep.create.render_product(sensor.GetPath().pathString, resolution=[1, 1]).path
        point_cloud_annotator = rep.AnnotatorRegistry.get_annotator("RtxSensorCpu" + "IsaacComputeRTXRadarPointCloud")
        point_cloud_annotator.attach([render_product_path])

        def data_acquisition_callback(event: carb.events.IEvent):
            data = point_cloud_annotator.get_data()
            radial_distance = data["radialDistance"]
            azimuth = data["azimuth"]
            elevation = data["elevation"]

            num_points = len(radial_distance)
            self.assertEqual(num_points, elevation)
            self.assertEqual(num_points, azimuth)

            for p in range(num_points):
                az = azimuth[p]
                el = elevation[p]
                r = radial_distance[p]

                # Adjust azimuth to appropriate angle in [-45, 45], then compute expected range to face, edge, or corner of cube
                az_adj = az + (1 - floor((3 * np.pi / 4.0 + az) / (np.pi / 2.0))) * np.pi / 2.0
                range_expected = edge_length / (2.0 * np.cos(az_adj) * np.cos(el))

                self.assertAlmostEqual(r, range_expected, delta=range_expected * 1e-2)

            return

        omni.kit.app.get_app_interface().get_update_event_stream().create_subscription_to_pop(data_acquisition_callback)
        omni.timeline.get_timeline_interface().play()
        for _ in range(6):
            await omni.kit.app.get_app().next_update_async()
        omni.timeline.get_timeline_interface().stop()

        point_cloud_annotator.detach()
        pass
