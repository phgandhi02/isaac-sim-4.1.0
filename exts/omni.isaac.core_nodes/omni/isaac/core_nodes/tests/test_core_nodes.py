# Copyright (c) 2018-2023, NVIDIA CORPORATION. All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto. Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.
#


import carb
import omni.kit.test
from omni.isaac.core.utils.stage import open_stage_async
from omni.isaac.core_nodes.bindings import _omni_isaac_core_nodes
from omni.isaac.nucleus import get_assets_root_path_async


class TestCoreNodes(omni.kit.test.AsyncTestCase):
    async def setUp(self):
        """Set up  test environment, to be torn down when done"""
        self._timeline = omni.timeline.get_timeline_interface()
        self._core_nodes = _omni_isaac_core_nodes.acquire_interface()
        # add franka robot for test
        assets_root_path = await get_assets_root_path_async()
        if assets_root_path is None:
            carb.log_error("Could not find Isaac Sim assets folder")
            return
        (result, error) = await open_stage_async(assets_root_path + "/Isaac/Robots/Franka/franka.usd")

    # ----------------------------------------------------------------------
    async def tearDown(self):
        """Get rid of temporary data used by the test"""
        await omni.kit.stage_templates.new_stage_async()

    # ----------------------------------------------------------------------
    async def test_simulation_time(self):
        await omni.kit.app.get_app().next_update_async()
        a = self._core_nodes.get_sim_time()
        b = self._core_nodes.get_sim_time_monotonic()
        c = self._core_nodes.get_system_time()
        await omni.kit.app.get_app().next_update_async()
        a = self._core_nodes.get_sim_time_at_swh_frame(0)
        b = self._core_nodes.get_sim_time_monotonic_at_swh_frame(0)
        c = self._core_nodes.get_system_time_at_swh_frame(0)
        self._timeline.play()
        await omni.kit.app.get_app().next_update_async()
        a = self._core_nodes.get_sim_time()
        b = self._core_nodes.get_sim_time_monotonic()
        c = self._core_nodes.get_system_time()
        a = self._core_nodes.get_sim_time_at_swh_frame(0)
        b = self._core_nodes.get_sim_time_monotonic_at_swh_frame(0)
        c = self._core_nodes.get_system_time_at_swh_frame(0)

    # ----------------------------------------------------------------------
    async def test_physics_num_steps(self):
        carb.settings.get_settings().set_bool("/app/runLoops/main/rateLimitEnabled", True)
        carb.settings.get_settings().set_int("/app/runLoops/main/rateLimitFrequency", 60)
        carb.settings.get_settings().set_int("/persistent/simulation/minFrameRate", 60)
        omni.timeline.get_timeline_interface().set_target_framerate(60)
        omni.timeline.get_timeline_interface().set_time_codes_per_second(60)

        steps = self._core_nodes.get_physics_num_steps()
        self.assertEqual(steps, 0)
        self._timeline.play()
        await omni.kit.app.get_app().next_update_async()
        steps = self._core_nodes.get_physics_num_steps()
        self.assertEqual(steps, 1)
        await omni.kit.app.get_app().next_update_async()
        steps = self._core_nodes.get_physics_num_steps()
        self.assertEqual(steps, 2)
        self._timeline.stop()
        await omni.kit.app.get_app().next_update_async()
        steps = self._core_nodes.get_physics_num_steps()
        self.assertEqual(steps, 0)
