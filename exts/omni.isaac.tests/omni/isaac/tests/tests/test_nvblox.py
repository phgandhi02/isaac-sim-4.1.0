# Copyright (c) 2018-2023, NVIDIA CORPORATION. All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto. Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.
#


import carb

# NOTE:
#   omni.kit.test - std python's unittest module with additional wrapping to add suport for async/await tests
#   For most things refer to unittest docs: https://docs.python.org/3/library/unittest.html
import omni.kit.test
from omni.isaac.core.utils.stage import open_stage_async
from omni.isaac.nucleus import get_assets_root_path_async


# Having a test class dervived from omni.kit.test.AsyncTestCase declared on the root of module will make it auto-discoverable by omni.kit.test
class TestNvBloxScenes(omni.kit.test.AsyncTestCase):
    # Before running each test
    async def setUp(self):
        self._assets_root_path = await get_assets_root_path_async()
        if self._assets_root_path is None:
            carb.log_error("Could not find Isaac Sim assets folder")
            return
        self._timeline = omni.timeline.get_timeline_interface()
        pass

    # After running each test
    async def tearDown(self):
        await omni.kit.stage_templates.new_stage_async()
        self._timeline = None
        pass

    async def test_nvBlox_sample_scene(self):
        # open scene
        self.usd_path = self._assets_root_path + "/Isaac/Samples/NvBlox/nvblox_sample_scene.usd"
        (result, error) = await open_stage_async(self.usd_path)

        # Make sure the stage loaded
        self.assertTrue(result)
        await omni.kit.app.get_app().next_update_async()

        self._timeline.play()

        for i in range(10):
            await omni.kit.app.get_app().next_update_async()

        self._timeline.stop()
        return True
