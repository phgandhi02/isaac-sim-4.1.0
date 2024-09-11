# Copyright (c) 2018-2023, NVIDIA CORPORATION. All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto. Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.
#

import asyncio

import numpy as np
import omni.kit

# NOTE:
#   omni.kit.test - std python's unittest module with additional wrapping to add suport for async/await tests
#   For most things refer to unittest docs: https://docs.python.org/3/library/unittest.html
import omni.kit.test
from omni.isaac.core.utils.stage import create_new_stage_async, is_stage_loading, update_stage_async

# Import extension python module we are testing with absolute import path, as if we are external user (other extension)
from omni.isaac.examples.path_planning import PathPlanning


class TestPathPlanningExampleExtension(omni.kit.test.AsyncTestCase):

    # Before running each test
    async def setUp(self):
        await create_new_stage_async()
        await update_stage_async()
        self._sample = PathPlanning()
        self._sample.set_world_settings(physics_dt=1.0 / 60.0, stage_units_in_meters=1.0)
        await self._sample.load_world_async()
        await update_stage_async()
        while is_stage_loading():
            await update_stage_async()
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
    async def test_follow_target(self):
        await self._sample.reset_async()
        await update_stage_async()
        await self._sample._on_follow_target_event_async()
        await update_stage_async()
        # run for 2500 frames and print time
        for i in range(500):
            await update_stage_async()
        pass

    # Run all functions with simulation enabled
    async def test_add_obstacle(self):
        await self._sample.reset_async()
        await update_stage_async()

        # run for 2500 frames and print time
        for i in range(500):
            await update_stage_async()
            if i % 50 == 0:
                self._sample._on_add_wall_event()
                await update_stage_async()
                await self._sample._on_follow_target_event_async()

        await update_stage_async()
        await self._sample.reset_async()
        await update_stage_async()
        pass

    async def test_reset(self):
        await self._sample.reset_async()
        await update_stage_async()
        pass
