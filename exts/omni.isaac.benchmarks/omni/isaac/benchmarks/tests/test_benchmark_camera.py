# Copyright (c) 2018-2024, NVIDIA CORPORATION. All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto. Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.
#

import numpy as np
import omni.kit.test
from omni.isaac.benchmark.services import BaseIsaacBenchmarkAsync
from omni.isaac.core.utils.rotations import euler_angles_to_quat
from omni.isaac.core.utils.stage import is_stage_loading
from omni.isaac.sensor import Camera
from omni.kit.viewport.utility import get_active_viewport

TEST_NUM_APP_UPDATES = 60 * 10


class TestBenchmarkCamera(BaseIsaacBenchmarkAsync):
    async def setUp(self):
        await super().setUp()
        pass

    async def tearDown(self):
        await super().tearDown()
        pass

    async def benchmark_camera(self, n_camera, resolution):
        self.benchmark_name = f"cameras_{n_camera}_resolution_{resolution[0]}_{resolution[1]}"
        self.set_phase("loading")

        scene_path = "/Isaac/Environments/Simple_Warehouse/full_warehouse.usd"
        await self.fully_load_stage(self.assets_root_path + scene_path)

        timeline = omni.timeline.get_timeline_interface()
        timeline.play()
        cameras = []

        for i in range(n_camera):
            render_product_path = None
            if i == 0:
                viewport_api = get_active_viewport()
                render_product_path = viewport_api.get_render_product_path()
            cameras.append(
                Camera(
                    prim_path="/Cameras/Camera_" + str(i),
                    position=np.array([-8, 13, 2.0]),
                    resolution=resolution,
                    orientation=euler_angles_to_quat([90, 0, 90 + i * 360 / n_camera], degrees=True),
                    render_product_path=render_product_path,
                )
            )

            await omni.kit.app.get_app().next_update_async()
            cameras[i].initialize()

        # make sure scene is loaded in all viewports
        while is_stage_loading():
            print("asset still loading, waiting to finish")
            await omni.kit.app.get_app().next_update_async()
        await omni.kit.app.get_app().next_update_async()

        await self.store_measurements()

        # perform benchmark
        self.set_phase("benchmark")

        for _ in range(1 if self.test_mode else TEST_NUM_APP_UPDATES):
            await omni.kit.app.get_app().next_update_async()

        await self.store_measurements()

        timeline.stop()
        cameras = None

    # ----------------------------------------------------------------------
    async def test_benchmark_1_camera_720p(self):
        await self.benchmark_camera(1, [1280, 720])

    async def test_benchmark_2_camera_720p(self):
        await self.benchmark_camera(2, [1280, 720])

    async def test_benchmark_4_camera_720p(self):
        await self.benchmark_camera(4, [1280, 720])

    async def test_benchmark_8_camera_720p(self):
        await self.benchmark_camera(8, [1280, 720])
