# Copyright (c) 2018-2023, NVIDIA CORPORATION. All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto. Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.
#

import asyncio

import carb
import numpy as np
import omni.kit

# NOTE:
#   omni.kit.test - std python's unittest module with additional wrapping to add suport for async/await tests
#   For most things refer to unittest docs: https://docs.python.org/3/library/unittest.html
import omni.kit.test
from omni.isaac.core.utils.stage import create_new_stage_async, is_stage_loading, update_stage_async

# Import extension python module we are testing with absolute import path, as if we are external user (other extension)
from omni.isaac.examples.bin_filling import BinFilling


class TestBinFillingExampleExtension(omni.kit.test.AsyncTestCase):

    # Before running each test
    async def setUp(self):
        await create_new_stage_async()
        await update_stage_async()
        self._sample = BinFilling()
        self._sample.set_world_settings(physics_dt=1.0 / 60.0, stage_units_in_meters=1.0)
        await self._sample.load_world_async()
        await update_stage_async()
        while is_stage_loading():
            await update_stage_async()
        settings = carb.settings.get_settings()
        settings.set("/app/player/useFixedTimeStepping", False)
        settings.set("/app/runLoops/main/rateLimitEnabled", False)
        return

    # After running each test
    async def tearDown(self):
        # In some cases the test will end before the asset is loaded, in this case wait for assets to load
        while is_stage_loading():
            print("tearDown, assets still loading, waiting to finish...")
            await asyncio.sleep(1.0)
        await self._sample.clear_async()
        await update_stage_async()
        self._sample = None
        pass

    # Run all functions with simulation enabled
    async def test_bin_filling(self):
        await self._sample.reset_async()
        await update_stage_async()
        world = self._sample.get_world()
        ur10_task = world.get_task(name="bin_filling")
        task_params = ur10_task.get_params()
        my_ur10 = world.scene.get_object(task_params["robot_name"]["value"])
        bin = world.scene.get_object(task_params["bin_name"]["value"])
        await self._sample.on_fill_bin_event_async()
        await update_stage_async()
        # run for 1500 frames and print time
        for i in range(1500):
            await update_stage_async()
            my_ur10.gripper.update()
            if self._sample._controller.get_current_event() in [4, 5]:
                self.assertTrue(my_ur10.gripper.is_closed())
            if self._sample._controller.get_current_event() == 5:
                self.assertGreater(bin.get_world_pose()[0][-1], 0.15)
            if self._sample._controller.get_current_event() > 5:
                if not my_ur10.gripper.is_closed():
                    break
        if my_ur10.gripper.is_closed():
            bin.set_linear_velocity(np.array([0.0, 0.0, -15.0]))

        for i in range(100):
            await update_stage_async()
        self.assertTrue(not my_ur10.gripper.is_closed())
        self.assertLess(bin.get_world_pose()[0][-1], 0.15)
        pass

    async def test_reset(self):
        await self._sample.reset_async()
        await update_stage_async()
        await self._sample.on_fill_bin_event_async()
        await update_stage_async()
        for i in range(2500):
            await update_stage_async()
        await self._sample.reset_async()
        await update_stage_async()
        await self._sample.on_fill_bin_event_async()
        await update_stage_async()
        for i in range(2500):
            await update_stage_async()
        pass
