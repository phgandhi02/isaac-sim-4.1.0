# Copyright (c) 2018-2023, NVIDIA CORPORATION. All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto. Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.
#
import gc
from abc import abstractmethod

from omni.isaac.core import World
from omni.isaac.core.scenes.scene import Scene
from omni.isaac.core.tasks.base_task import BaseTask
from omni.isaac.core.utils.stage import create_new_stage_async, update_stage_async
from omni.isaac.cortex.cortex_world import CortexWorld
from omni.isaac.examples import base_sample


class CortexBase(base_sample.BaseSample):
    async def load_world_async(self):
        """
        Function called when clicking load buttton.
        The difference between this class and Base Sample is that we initialize a CortexWorld specialization.
        """
        if CortexWorld.instance() is None:
            await create_new_stage_async()
            self._world = CortexWorld(**self._world_settings)
            await self._world.initialize_simulation_context_async()
            self.setup_scene()
        else:
            self._world = CortexWorld.instance()
        self._current_tasks = self._world.get_current_tasks()
        await self._world.reset_async()
        await self._world.pause_async()
        await self.setup_post_load()
        if len(self._current_tasks) > 0:
            self._world.add_physics_callback("tasks_step", self._world.step_async)
        return
