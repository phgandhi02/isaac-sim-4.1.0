# Copyright (c) 2018-2023, NVIDIA CORPORATION. All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto. Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.
#

# This import is included for visualization of UI elements as demonstrated in testXYPlotWrapper
import asyncio

import numpy as np
import omni.kit.test
import omni.kit.ui_test as ui_test
import omni.timeline
import omni.ui as ui
from omni.isaac.core.articulations import Articulation
from omni.isaac.core.objects.cuboid import FixedCuboid, VisualCuboid
from omni.isaac.core.utils.stage import (
    add_reference_to_stage,
    create_new_stage,
    create_new_stage_async,
    update_stage_async,
)
from omni.isaac.core.world import World
from omni.isaac.nucleus import get_assets_root_path
from omni.isaac.ui.element_wrappers.core_connectors import LoadButton, ResetButton


# Having a test class dervived from omni.kit.test.AsyncTestCase declared on the root of module will make it auto-discoverable by omni.kit.test
class TestUICoreConnectors(omni.kit.test.AsyncTestCase):
    # Before running each test
    async def setUp(self):
        self._timeline = omni.timeline.get_timeline_interface()
        pass

    # After running each test
    async def tearDown(self):
        self._timeline.stop()
        while omni.usd.get_context().get_stage_loading_status()[2] > 0:
            print("tearDown, assets still loading, waiting to finish...")
            await asyncio.sleep(1.0)
        await update_stage_async()
        World.clear_instance()

        await create_new_stage_async()

    async def _create_window(self, title, width, height):
        window = ui.Window(
            title=title,
            width=width,
            height=height,
            visible=True,
            dockPreference=ui.DockPreference.LEFT_BOTTOM,
        )
        await update_stage_async()
        return window

    async def testLoadButton(self):
        window_title = "UI_Widget_Wrapper_Test_Window_LoadButton_Test"
        width = 500
        height = 200
        window = await self._create_window(window_title, width, height)

        self.setup_scene_called = False
        self.setup_post_load_called = False

        def setup_scene_fn():
            self.setup_scene_called = True

            robot_prim_path = "/ur10e"
            path_to_robot_usd = get_assets_root_path() + "/Isaac/Robots/UniversalRobots/ur10e/ur10e.usd"

            create_new_stage()
            add_reference_to_stage(path_to_robot_usd, robot_prim_path)

            art = Articulation(robot_prim_path)
            cuboid = FixedCuboid(
                "/Scenario/cuboid", position=np.array([0.3, 0.3, 0.5]), size=0.05, color=np.array([255, 0, 0])
            )

            world = World.instance()
            world.scene.add(art)
            world.scene.add(cuboid)

        def setup_post_load_fn():
            self.setup_post_load_called = True

            # Assert that the timeline is paused when this callback is called
            self.assertFalse(self._timeline.is_playing() or self._timeline.is_stopped())

        with window.frame:
            load_button = LoadButton(
                "LoadButton", "LOAD", setup_scene_fn=setup_scene_fn, setup_post_load_fn=setup_post_load_fn
            )

        button = ui_test.find(f"{window_title}//Frame/Frame[0]/HStack[0]/Button[0]")
        await button.click()
        await update_stage_async()

        # The LoadButton resets the Core World asynchronously, so it can take some time to get to the setup_post_load_fn
        await asyncio.sleep(1)

        self.assertTrue(self.setup_scene_called)
        self.assertTrue(self.setup_post_load_called)

    async def testResetButton(self):
        window_title = "UI_Widget_Wrapper_Test_Window_ResetButton_Test"
        width = 500
        height = 200
        window = await self._create_window(window_title, width, height)

        self.pre_reset_called = False
        self.post_reset_called = False

        def pre_reset_fn():
            self.pre_reset_called = True

            self.assertTrue(np.all(self.cuboid.get_world_pose()[0] == np.array([-1, 0, 0])))

        def post_reset_fn():
            self.post_reset_called = True

            # Assert that the timeline is paused when this callback is called
            self.assertFalse(self._timeline.is_playing() or self._timeline.is_stopped())

            self.assertTrue(np.all(self.cuboid.get_world_pose()[0] == np.array([1, 0, 0])))

        with window.frame:
            reset_button = ResetButton("ResetButton", "RESET", pre_reset_fn=pre_reset_fn, post_reset_fn=post_reset_fn)

        world = World()
        await world.initialize_simulation_context_async()

        self.cuboid = VisualCuboid("/cuboid", size=0.1, position=np.array([1, 0, 0]))
        world.scene.add(self.cuboid)

        self._timeline.play()
        await update_stage_async()
        self.cuboid.set_world_pose(np.array([-1, 0, 0]))

        button = ui_test.find(f"{window_title}//Frame/Frame[0]/HStack[0]/Button[0]")
        await button.click()
        await update_stage_async()

        # The ResetButton resets the Core World asynchronously, so it can take some time to get to the setup_post_load_fn
        await asyncio.sleep(0.25)

        self.assertTrue(self.pre_reset_called)
        self.assertTrue(self.post_reset_called)
