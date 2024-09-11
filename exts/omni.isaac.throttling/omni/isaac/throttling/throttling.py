# Copyright (c) 2022-2023, NVIDIA CORPORATION. All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto. Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.
#


import carb
import omni.ext


class Extension(omni.ext.IExt):
    def on_startup(self, ext_id):
        # Enable the developer throttling settings when extension starts
        carb.settings.get_settings().set("/app/show_developer_preference_section", True)
        # Only create subscription if ecomode was enabled on startup
        curr_eco_mode = carb.settings.get_settings().get("/rtx/ecoMode/enabled")
        if curr_eco_mode:
            timeline = omni.timeline.get_timeline_interface()
            self.timeline_event_sub = timeline.get_timeline_event_stream().create_subscription_to_pop(
                self.on_stop_play, name="IsaacSimThrottlingEventHandler"
            )
        pass

    def on_stop_play(self, event: carb.events.IEvent):
        # Enable eco mode if playing sim, disable if stopped
        _settings = carb.settings.get_settings()
        if event.type == int(omni.timeline.TimelineEventType.PLAY):
            _settings.set("/rtx/ecoMode/enabled", False)
        elif event.type == int(omni.timeline.TimelineEventType.STOP):
            _settings.set("/rtx/ecoMode/enabled", True)
        pass

    def on_shutdown(self):
        self.timeline_event_sub = None
        pass
