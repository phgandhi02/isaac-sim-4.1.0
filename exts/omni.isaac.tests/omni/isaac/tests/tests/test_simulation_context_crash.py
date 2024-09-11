# Copyright (c) 2022-2024, NVIDIA CORPORATION. All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto. Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.
#

import asyncio

import omni.kit.test
from omni.isaac.core.robots.robot import Robot
from omni.isaac.core.utils.stage import add_reference_to_stage, create_new_stage_async, update_stage_async
from omni.isaac.core.world import World
from omni.isaac.nucleus import get_assets_root_path_async


# Having a test class derived from omni.kit.test.AsyncTestCase declared on the root of module will
# make it auto-discoverable by omni.kit.test
class TestSimulationContextCrash(omni.kit.test.AsyncTestCase):
    # Before running each test
    async def setUp(self):
        self._physics_dt = 1 / 60  # duration of physics frame in seconds

        self._timeline = omni.timeline.get_timeline_interface()

        await create_new_stage_async()
        await update_stage_async()

        pass

    # After running each test
    async def tearDown(self):
        self._timeline.stop()
        while omni.usd.get_context().get_stage_loading_status()[2] > 0:
            print("tearDown, assets still loading, waiting to finish...")
            await asyncio.sleep(1.0)
        await update_stage_async()
        World.clear_instance()
        pass

    async def test_simulation_context_crash(self):
        usd_path = await get_assets_root_path_async()
        usd_path += "/Isaac/Robots/Denso/cobotta_pro_900.usd"
        robot_prim_path = "/cobotta_pro_900"

        add_reference_to_stage(usd_path, robot_prim_path)

        self._timeline = omni.timeline.get_timeline_interface()

        # Start Simulation and wait
        self._timeline.play()
        await update_stage_async()
        self._robot = Robot(robot_prim_path)
        self._robot.initialize()
        # Initializing World after creating a robot will cause undefined behavior if timeline is playing
        world = World()
        # initializing causes timeline to stop if playing
        await world.initialize_simulation_context_async()
        await update_stage_async()
        self.assertEquals(world.is_playing(), False)
        # Make sure this call doesn't crash due to invalid physx handles
        self._robot.disable_gravity()
