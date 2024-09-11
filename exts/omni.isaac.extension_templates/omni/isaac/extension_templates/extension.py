# Copyright (c) 2022-2023, NVIDIA CORPORATION. All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto. Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.
#

import asyncio
import gc
import os
import weakref

import carb
import omni
import omni.kit.commands
import omni.timeline
import omni.ui as ui
import omni.usd
from omni.isaac.ui.element_wrappers import CollapsableFrame, ScrollingWindow, TextBlock
from omni.isaac.ui.menu import make_menu_item_description
from omni.isaac.ui.ui_utils import btn_builder, get_style, setup_ui_headers, str_builder
from omni.kit.menu.utils import add_menu_items, remove_menu_items

from .template_generator import TemplateGenerator

EXTENSION_NAME = "Generate Extension Templates"


class Extension(omni.ext.IExt):
    def on_startup(self, ext_id: str):
        """Initialize extension and UI elements"""

        # Events
        self._usd_context = omni.usd.get_context()

        # Build Window
        self._window = ScrollingWindow(
            title=EXTENSION_NAME, width=600, height=500, visible=False, dockPreference=ui.DockPreference.LEFT_BOTTOM
        )
        self._window.set_visibility_changed_fn(self._on_window)

        # UI
        self._models = {}
        self._ext_id = ext_id
        self._menu_items = [
            make_menu_item_description(ext_id, EXTENSION_NAME, lambda a=weakref.proxy(self): a._menu_callback())
        ]
        # self._menu_items = [MenuItemDescription(name="Workflows", sub_menu=menu_items)]
        add_menu_items(self._menu_items, "Isaac Utils")

        self._template_generator = TemplateGenerator()

    def on_shutdown(self):
        self._models = {}
        remove_menu_items(self._menu_items, "Isaac Utils")
        if self._window:
            self._window = None
        gc.collect()

    def _on_window(self, visible):
        if self._window.visible:
            self._build_ui()

    def _menu_callback(self):
        self._window.visible = not self._window.visible

    def _build_ui(self):
        # if not self._window:
        with self._window.frame:
            with ui.VStack(spacing=5, height=0):

                self._build_info_ui()

                self._build_template_ui(
                    "Configuration Tooling Template", self._template_generator.generate_configuration_tooling_template
                )

                self._build_template_ui(
                    "Loaded Scenario Template", self._template_generator.generate_loaded_scenario_template
                )

                self._build_template_ui("Scripting Template", self._template_generator.generate_scripting_template)

                self._build_template_ui(
                    "UI Component Library", self._template_generator.generate_component_library_template
                )

                self._build_status_panel()

        async def dock_window():
            await omni.kit.app.get_app().next_update_async()

            def dock(space, name, location, pos=0.5):
                window = omni.ui.Workspace.get_window(name)
                if window and space:
                    window.dock_in(space, location, pos)
                return window

            tgt = ui.Workspace.get_window("Viewport")
            dock(tgt, EXTENSION_NAME, omni.ui.DockPosition.LEFT, 0.33)
            await omni.kit.app.get_app().next_update_async()

        self._task = asyncio.ensure_future(dock_window())

    def _build_info_ui(self):
        title = EXTENSION_NAME
        doc_link = (
            "https://docs.omniverse.nvidia.com/isaacsim/latest/advanced_tutorials/tutorial_extension_templates.html"
        )

        overview = (
            "Generate Extension Templates to get started building and programming standalone UI-based extensions in "
            + "Isaac Sim."
        )

        setup_ui_headers(self._ext_id, __file__, title, doc_link, overview)

    def _build_status_panel(self):
        self._status_frame = CollapsableFrame("Status Frame", collapsed=True, visible=False)
        with self._status_frame:
            self._status_block = TextBlock("Status", "", num_lines=3, include_copy_button=False)

    def _build_template_ui(self, template_name, generate_fun):
        frame = ui.CollapsableFrame(
            title=template_name,
            height=0,
            collapsed=True,
            style=get_style(),
            style_type_name_override="CollapsableFrame",
            horizontal_scrollbar_policy=ui.ScrollBarPolicy.SCROLLBAR_AS_NEEDED,
            vertical_scrollbar_policy=ui.ScrollBarPolicy.SCROLLBAR_ALWAYS_ON,
        )

        path_field = template_name + "_path"
        title_field = template_name + "_title"
        generate_btn = template_name + "_generate"
        description_field = template_name + "_description"

        with frame:
            with ui.VStack(style=get_style(), spacing=5, height=0):

                def control_generate_btn(model=None):
                    path = self._models[path_field].get_value_as_string()
                    title = self._models[title_field].get_value_as_string()

                    if path != "" and path[-1] != "/" and path[-1] != "\\" and title.strip(" ") != "":
                        self._models[generate_btn].enabled = True
                        self.write_status(f"Ready to Generate {template_name}")
                    else:
                        self._models[generate_btn].enabled = False
                        self.write_status(
                            "Cannot Generate Extension Template Without a Title and Valid Path.  The Path must not end in a '/'."
                        )

                self._models[path_field] = str_builder(
                    label="Extension Path",
                    tooltip="Directory where the extension template will be populated.  The path must not end in a slash",
                    use_folder_picker=True,
                    item_filter_fn=lambda item: item.is_folder,
                    folder_dialog_title="Select Path",
                    folder_button_title="Select",
                )
                self._models[path_field].add_value_changed_fn(control_generate_btn)

                self._models[title_field] = str_builder(
                    label="Extension Title",
                    default_val="",
                    tooltip="Title of Extension that will show up on Isaac Sim Toolbar",
                )
                self._models[title_field].add_value_changed_fn(control_generate_btn)

                self._models[description_field] = str_builder(
                    label="Extension Description", default_val="", tooltip="Short description of extension"
                )

                def on_generate_extension(model=None, val=None):
                    path = self._models[path_field].get_value_as_string()
                    title = self._models[title_field].get_value_as_string()
                    description = self._models[description_field].get_value_as_string()
                    generate_fun(path, title, description)

                    self.write_status(f"Created new extension '{title}' at {path} from {template_name}")

                self._models[generate_btn] = btn_builder(
                    label="Generate Extension",
                    text="Generate Extension",
                    tooltip=f"Generate {template_name}",
                    on_clicked_fn=on_generate_extension,
                )
                self._models[generate_btn].enabled = False

    def write_status(self, status, collapsed=False, visible=True):
        self._status_block.set_text(status)
        self._status_frame.collapsed = collapsed
        self._status_frame.visible = visible
