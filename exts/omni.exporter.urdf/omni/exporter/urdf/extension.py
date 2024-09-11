# Copyright (c) 2018-2023, NVIDIA CORPORATION. All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto. Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.
#


import asyncio
import gc
import weakref

import omni.ext
import omni.kit.commands
import omni.timeline
import omni.ui as ui
import omni.usd
from omni.isaac.ui.menu import make_menu_item_description
from omni.kit.menu.utils import add_menu_items, remove_menu_items
from omni.usd import StageEventType

EXTENSION_TITLE = "USD to URDF Exporter"
from .ui_builder import UIBuilder


class Extension(omni.ext.IExt):
    def on_startup(self, ext_id: str):
        """Initialize extension and UI elements"""

        # Events
        self._usd_context = omni.usd.get_context()

        # Build Window
        self._window = ui.Window(
            title=EXTENSION_TITLE, width=600, height=500, visible=False, dockPreference=ui.DockPreference.LEFT_BOTTOM
        )
        self._window.set_visibility_changed_fn(self._on_window)

        # UI
        self._models = {}
        self._ext_id = ext_id
        self._menu_items = [
            make_menu_item_description(ext_id, EXTENSION_TITLE, lambda a=weakref.proxy(self): a._menu_callback())
        ]

        add_menu_items(self._menu_items, EXTENSION_TITLE)

        # Filled in with User Functions
        self.ui_builder = UIBuilder()

        # Events
        self._usd_context = omni.usd.get_context()
        self._stage_event_sub = None
        self._timeline = omni.timeline.get_timeline_interface()

    def on_shutdown(self):
        self._models = {}
        remove_menu_items(self._menu_items, EXTENSION_TITLE)
        if self._window:
            self._window = None
        self.ui_builder.cleanup()
        gc.collect()

    def _on_window(self, visible):
        if self._window.visible:
            # Subscribe to Stage and Timeline Events
            self._usd_context = omni.usd.get_context()
            events = self._usd_context.get_stage_event_stream()
            self._stage_event_sub = events.create_subscription_to_pop(self._on_stage_event)
            stream = self._timeline.get_timeline_event_stream()
            self._timeline_event_sub = stream.create_subscription_to_pop(self._on_timeline_event)

            self._build_ui()
        else:
            self._usd_context = None
            self._stage_event_sub = None
            self._timeline_event_sub = None
            self.ui_builder.cleanup()

    def _build_ui(self):
        with self._window.frame:
            with ui.VStack(spacing=5, height=0):
                self._build_extension_ui()

        async def dock_window():
            await omni.kit.app.get_app().next_update_async()

            def dock(space, name, location, pos=0.5):
                window = omni.ui.Workspace.get_window(name)
                if window and space:
                    window.dock_in(space, location, pos)
                return window

            tgt = ui.Workspace.get_window("Viewport")
            dock(tgt, EXTENSION_TITLE, omni.ui.DockPosition.LEFT, 0.33)
            await omni.kit.app.get_app().next_update_async()

        self._task = asyncio.ensure_future(dock_window())

    def _menu_callback(self):
        self._window.visible = not self._window.visible
        self.ui_builder.on_menu_callback()

    def _on_timeline_event(self, event):
        self.ui_builder.on_timeline_event(event)

    def _on_stage_event(self, event):
        if event.type == int(StageEventType.OPENED) or event.type == int(StageEventType.CLOSED):
            # stage was opened or closed, cleanup
            self.ui_builder.cleanup()

        self.ui_builder.on_stage_event(event)

    def _build_extension_ui(self):
        # Call user function for building UI
        self.ui_builder.build_ui()
