# Copyright (c) 2022-2023, NVIDIA CORPORATION. All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto. Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.
#

import omni.kit.test
import omni.replicator.core as rep
from omni.isaac.core.utils.render_product import *


class TestStage(omni.kit.test.AsyncTestCase):
    # Before running each test
    async def setUp(self):
        await omni.usd.get_context().new_stage_async()
        pass

    # After running each test
    async def tearDown(self):
        pass

    async def test_hydra_texture(self):
        await omni.kit.app.get_app().next_update_async()
        hydra_texture = rep.create.render_product("/OmniverseKit_Persp", [512, 512], name="Isaac")
        await omni.kit.app.get_app().next_update_async()
        self.assertEqual(get_camera_prim_path(hydra_texture.path), "/OmniverseKit_Persp")
        add_aov(hydra_texture.path, "RtxSensorCpu")
        await omni.kit.app.get_app().next_update_async()
        camera_prim_path = get_camera_prim_path(hydra_texture.path)
        set_camera_prim_path(hydra_texture.path, "/OmniverseKit_Top")
        await omni.kit.app.get_app().next_update_async()
        self.assertEqual(get_camera_prim_path(hydra_texture.path), "/OmniverseKit_Top")
        self.assertEqual(get_resolution(hydra_texture.path), (512, 512))
        await omni.kit.app.get_app().next_update_async()
        set_resolution(hydra_texture.path, (1024, 1024))
        await omni.kit.app.get_app().next_update_async()
        self.assertEqual(get_resolution(hydra_texture.path), (1024, 1024))
        hydra_texture = None
        await omni.kit.app.get_app().next_update_async()
