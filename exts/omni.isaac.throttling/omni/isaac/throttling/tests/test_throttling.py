# Copyright (c) 2022-2023, NVIDIA CORPORATION. All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto. Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.
#

import carb
import carb.settings
import omni.ext
import omni.kit.test


class TestIsaacThrottling(omni.kit.test.AsyncTestCase):
    async def setUp(self):
        pass

    async def tearDown(self):
        pass

    async def test_on_stop_play_callback(self):
        self._settings = carb.settings.get_settings()
        self._settings.set("/rtx/ecoMode/enabled", True)
        self._timeline = omni.timeline.get_timeline_interface()
        self._timeline.play()
        await omni.kit.app.get_app().next_update_async()
        self.assertEqual(self._settings.get("/rtx/ecoMode/enabled"), False)
        self._timeline.stop()
        await omni.kit.app.get_app().next_update_async()
        self.assertEqual(self._settings.get("/rtx/ecoMode/enabled"), True)
        self._timeline.play()
        await omni.kit.app.get_app().next_update_async()
        self.assertEqual(self._settings.get("/rtx/ecoMode/enabled"), False)
        self._timeline.stop()
        await omni.kit.app.get_app().next_update_async()
        self.assertEqual(self._settings.get("/rtx/ecoMode/enabled"), True)
        pass
