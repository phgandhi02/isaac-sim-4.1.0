# Copyright (c) 2022-2024, NVIDIA CORPORATION. All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto. Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.
#

import asyncio
import gc
import json
import os
import weakref

import carb.settings
import omni
import omni.kit.commands
import omni.physxsupportui.bindings._physxSupportUi as pxsupportui
import omni.timeline
import omni.ui as ui
import omni.usd
from omni.isaac.import_wizard.questionnaire import Questionnaire
from omni.isaac.import_wizard.ui_utils import (
    BUTTON_FONT,
    HIGHLIGHT_COLOR,
    Singleton,
    label_kwargs,
    large_btn_kwargs,
    medium_btn_kwargs,
    scroll_kwargs,
    small_btn_kwargs,
    text_kwargs,
    x_large_btn_kwargs,
)
from omni.isaac.ui.callbacks import on_docs_link_clicked, on_open_folder_clicked
from omni.isaac.ui.element_wrappers import ScrollingWindow
from omni.isaac.ui.menu import MenuItemDescription
from omni.isaac.ui.style import get_style
from omni.isaac.ui.ui_utils import add_folder_picker_icon
from omni.kit.menu.utils import add_menu_items, remove_menu_items
from omni.kit.notification_manager import NotificationStatus, post_notification
from omni.kit.window.extensions import SimpleCheckBox
from omni.kit.window.filepicker.dialog import FilePickerDialog
from omni.kit.window.popup_dialog import MessageDialog
from omni.kit.window.property.templates import LABEL_HEIGHT, LABEL_WIDTH

EXTENSION_TITLE = "Isaac Sim Import Wizard [alpha]"
EXTENSION_FOLDER_PATH = omni.kit.app.get_app().get_extension_manager().get_extension_path_by_module(__name__)


class Extension(omni.ext.IExt):
    def on_startup(self, ext_id: str):
        self.ext_id = ext_id

        # creating window for the wizard
        self._ext_window = ImportWizard(ext_id)

        action_registry = omni.kit.actions.core.get_action_registry()
        action_registry.register_action(
            ext_id,
            f"CreateUIExtension:{EXTENSION_TITLE}",
            self._menu_callback,
            description=f"Add {EXTENSION_TITLE} Extension to UI toolbar",
        )
        self._menu_items = [
            MenuItemDescription(name=EXTENSION_TITLE, onclick_action=(ext_id, f"CreateUIExtension:{EXTENSION_TITLE}"))
        ]

        add_menu_items(self._menu_items, "Isaac Utils")

    def on_shutdown(self):
        self._ext_window.on_shutdown()

        remove_menu_items(self._menu_items, "Isaac Utils")

        action_registry = omni.kit.actions.core.get_action_registry()
        action_registry.deregister_action(self.ext_id, f"CreateUIExtension:{EXTENSION_TITLE}")

    def _menu_callback(self):
        self._ext_window._window.visible = not self._ext_window._window.visible

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


@Singleton
class ImportWizard(object):
    def __init__(self, ext_id: str):
        self._ext_id = ext_id
        self._settings = carb.settings.get_settings()

        # params for initializing the wizard
        self._current_tool_name = None
        self.qa = None

        self._window = ScrollingWindow(
            title=EXTENSION_TITLE,
            width=400,
            height=500,
            visible=False,
            dockPreference=ui.DockPreference.LEFT_BOTTOM,
            horizontal_scrollbar_policy=ui.ScrollBarPolicy.SCROLLBAR_AS_NEEDED,
            vertical_scrollbar_policy=ui.ScrollBarPolicy.SCROLLBAR_ALWAYS_ON,
        )
        self._window.set_visibility_changed_fn(self._on_window)

    def on_shutdown(self):
        # close the wizard and shutdown the questionnaire popup if exist
        if self._window:
            self._window.visible = False
            self._window = None
        if self.qa is not None:
            self.qa.on_shutdown()

        gc.collect()

    def _on_window(self, visible):
        if visible:
            # if the window was closed by accident, it should open at the place it was closed, not from the beginning
            # the wizard instruction page should just be the opening page
            if self._current_tool_name and self._current_tool_name != "Isaac Sim Import Wizard":
                self._build_instructions_page(self._current_tool_name)
            else:
                self._build_welcome_page()
        else:
            # if wizard window closed, also close the questionnaire window if it's open
            if self.qa is not None:
                self.qa.on_shutdown()

    def reset(self):
        self.available_tools_list = []
        self.full_app_list = []
        self.user_tools_list = []
        self.selection_state = []
        self._current_app_name = ""
        self._current_tool_name = ""
        self._use_custom_file = False
        self._app_data = None
        self.pipeline_file = os.path.join(EXTENSION_FOLDER_PATH, "data", "pipeline.json")
        self.custom_file = os.path.join(EXTENSION_FOLDER_PATH, "data", "custom_pipeline.json")
        self.tools_file = os.path.join(EXTENSION_FOLDER_PATH, "data", "available_tools.json")

        # shutdown questionnaire if it's active
        if self.qa is not None:
            self.qa.on_shutdown()

    def _build_welcome_page(self):

        # opening welcome page resets all the internal parameters
        self.reset()

        # # get data for available pipelines
        with open(self.tools_file, "r") as file:
            tools_file = json.load(file)
        with open(self.pipeline_file, "r") as f:
            pipeline_data = json.load(f)

        # data needed for the welcome page
        self._get_data_from_file(self.pipeline_file, "Isaac Sim Import Wizard")
        self.available_tools_list = tools_file["Available Tools"]  # names of tools that are presented to users

        self.full_app_list = [
            pipeline_data[tool]["App Name"] for tool in self.available_tools_list
        ]  # names of apps that matches the extension name
        self.selection_state = [False] * len(self.full_app_list)

        # build the wizard welcome page ui
        with self._window.frame:
            with ui.Frame():
                with ui.VStack(height=0):
                    with ui.HStack():
                        ui.Label(
                            "Welcome!",
                            alignment=ui.Alignment.LEFT_TOP,
                            style={"font_size": 20, "color": 0xFFC7C7C7},
                        )
                    with ui.HStack():
                        ui.Spacer()
                        ui.Button(
                            "DO YOU NEED HELP GETTING STARTED?",
                            height=60,
                            width=ui.Percent(60),
                            word_wrap=True,
                            alignment=ui.Alignment.CENTER,
                            clicked_fn=self._build_questionnaire_window,  ## launch the "chatbot",
                            **large_btn_kwargs,
                        )
                        ui.Spacer()

                    with ui.VStack(height=0):
                        ui.Spacer(height=5)
                        ui.Line(style={"color": 0x338A8777}, width=ui.Fraction(1), alignment=ui.Alignment.CENTER)
                        ui.Spacer(height=10)
                        ui.Label(
                            "Isaac Sim Import Wizard [alpha]",
                            width=0,
                            alignment=ui.Alignment.LEFT_TOP,
                            style={"font_size": 20, "color": 0xFFC7C7C7},
                        )
                        ui.Spacer(height=10)
                        with ui.HStack():
                            ui.Label(
                                "Summary:",
                                **label_kwargs,
                            )
                            with ui.ZStack(style={"ZStack": {"margin": 0}}):
                                ui.Rectangle(height=ui.Fraction(1))
                                with ui.HStack(spacing=0):
                                    ui.Spacer(width=5)
                                    ui.Label(
                                        self._app_data["Summary"],
                                        **text_kwargs,
                                    )
                        ui.Spacer(height=10)
                        with ui.HStack():
                            ui.Label(
                                "Basic Instructions:",
                                **label_kwargs,
                            )
                            with ui.ScrollingFrame(
                                height=LABEL_HEIGHT * 15,
                                **scroll_kwargs,
                            ):
                                with ui.ZStack(style={"ZStack": {"margin": 10}}):
                                    ui.Rectangle()
                                    with ui.HStack(spacing=0):
                                        ui.Spacer(width=5)
                                        ui.Label(
                                            self._app_data["Basic Instructions"],
                                            **text_kwargs,
                                        )
                                        ui.Spacer(width=5)
                        ui.Spacer(height=10)
                        with ui.HStack():
                            ui.Label(
                                "Menu Location:",
                                **label_kwargs,
                                tooltip="Where to find this tool in the GUI",
                            )
                            with ui.ZStack(style={"ZStack": {"margin": 0}}):
                                ui.Rectangle(height=ui.Fraction(1))
                                with ui.HStack(spacing=0):
                                    ui.Spacer(width=5)
                                    ui.Label(
                                        self._app_data["Menu"],
                                        **text_kwargs,
                                    )
                        ui.Spacer(height=10)
                        with ui.HStack():
                            ui.Label(
                                "Additional Resources:",
                                width=LABEL_WIDTH / 2,
                                style_type_name_override="Label::label",
                                word_wrap=True,
                                alignment=ui.Alignment.LEFT_TOP,
                            )
                            with ui.VStack():
                                with ui.HStack():
                                    ui.Label(
                                        "Documentation:",
                                        style_type_name_override="Label::label",
                                        word_wrap=True,
                                        alignment=ui.Alignment.LEFT_TOP,
                                        width=LABEL_WIDTH,
                                    )
                                    with ui.Frame(tooltip="Link to Docs"):
                                        ui.Button(
                                            name="DocLink",
                                            width=24,
                                            height=24,
                                            clicked_fn=lambda: on_docs_link_clicked(self._links["Documentation Link"]),
                                            style=get_style()["IconButton.Image::OpenLink"],
                                            alignment=ui.Alignment.LEFT_TOP,
                                        )
                                with ui.HStack():
                                    ui.Label(
                                        "Forum:",
                                        style_type_name_override="Label::label",
                                        word_wrap=True,
                                        alignment=ui.Alignment.LEFT_TOP,
                                        width=LABEL_WIDTH,
                                    )
                                    with ui.Frame(tooltip="Link to Forum"):
                                        ui.Button(
                                            name="ForumLink",
                                            width=24,
                                            height=24,
                                            clicked_fn=lambda: on_docs_link_clicked(self._links["API Link"]),
                                            style=get_style()["IconButton.Image::OpenLink"],
                                            alignment=ui.Alignment.LEFT_TOP,
                                        )
                                with ui.HStack():
                                    ui.Label(
                                        "Pipeline Data Folder:",
                                        style_type_name_override="Label::label",
                                        word_wrap=True,
                                        alignment=ui.Alignment.LEFT_TOP,
                                        width=LABEL_WIDTH,
                                    )
                                    with ui.Frame(tooltip="Open Containing Folder"):
                                        ui.Button(
                                            name="IconButton",
                                            width=24,
                                            height=24,
                                            clicked_fn=lambda: on_open_folder_clicked(
                                                os.path.join(EXTENSION_FOLDER_PATH, "data")
                                            ),
                                            style=get_style()["IconButton.Image::OpenFolder"],
                                            alignment=ui.Alignment.LEFT_CENTER,
                                        )

                    self._build_checkboxes()

                    ui.Spacer(height=10)
                    # wizard footer
                    with ui.CollapsableFrame(
                        "Advanced Tips",
                        collapsed=True,
                        style={
                            "font_size": BUTTON_FONT,
                            "background_color": 0xFF2C2C2C,
                            "color": "0xFF00B976",
                        },
                        style_type_name_override="CollapsableFrame",
                    ):
                        with ui.ScrollingFrame(
                            height=LABEL_HEIGHT * 8,
                            **scroll_kwargs,
                        ):
                            with ui.HStack():
                                ui.Spacer(width=10)
                                ui.Label(
                                    self._app_data["Advanced Instructions"],
                                    **text_kwargs,
                                )
                    ui.Spacer(height=10)
                    with ui.HStack():
                        ui.Label(
                            "Jump To",
                            **label_kwargs,
                        )

                        combo_box = ui.ComboBox(
                            0, *self.available_tools_list, width=200, alignment=ui.Alignment.LEFT_CENTER
                        ).model

                        combo_box.add_item_changed_fn(self._on_clicked_dropdown)

                    ui.Spacer(height=10)
                    ui.Button(
                        "Next",
                        height=60,
                        clicked_fn=self._build_pipeline_page,
                        **large_btn_kwargs,
                    )
                    ui.Spacer(height=10)

    def _build_pipeline_page(self):
        """
        this page displays the list of tools of the pipeline before starting the process
        """
        # if the user wants to use a custom file, we get the user_tools_list from the custom file
        if self._use_custom_file:
            try:
                with open(self.custom_file, "r") as file:
                    user_data = json.load(file)
                self.user_tools_list = user_data[self.pipeline_name.model.get_value_as_string()]
            except:
                msg = "Invalid File name and/or Pipeline name. Please select a valid file and a valid pipeline."
                post_notification(msg, status=NotificationStatus.WARNING, duration=8)
                return

            # if it's a custom file, need to make sure the tool names listed are valid ones
            for tool in self.user_tools_list:
                if tool not in self.available_tools_list:
                    msg = (
                        tool
                        + " not a valid tool. Please check your pipeline list, and use the exact names appeared next to the checkboxes."
                    )
                    post_notification(msg, status=NotificationStatus.WARNING, duration=8)
                    return
        else:
            user_app_idx = [index for index, item in enumerate(self.selection_state) if item]
            self.user_tools_list = [self.available_tools_list[idx] for idx in user_app_idx]

        # if there is a user_tools_list, start at the beginning
        if self.user_tools_list:
            with self._window.frame:
                with ui.Frame(style=get_style()):
                    with ui.VStack(height=0):
                        ui.Spacer(height=15)
                        ui.Label(
                            "Pipeline Listed",
                            width=0,
                            name="title",
                            alignment=ui.Alignment.LEFT_TOP,
                            style={"font_size": 20, "color": 0xFFC7C7C7},
                        )
                        ui.Spacer(height=10)
                        with ui.ScrollingFrame(
                            height=LABEL_HEIGHT * 15,
                            **scroll_kwargs,
                        ):
                            with ui.ZStack(style={"ZStack": {"margin": 10}}):
                                ui.Rectangle()
                                ui.Label(
                                    "\n".join(self.user_tools_list),
                                    **text_kwargs,
                                )
                        with ui.HStack():
                            ui.Label(
                                "Instructions:",
                                **label_kwargs,
                            )
                            with ui.ZStack(style={"ZStack": {"margin": 10}}):
                                ui.Rectangle()
                                ui.Label(
                                    "Click 'Next' to start your process, 'Cancel' to Rebuild the Pipeline",
                                    **text_kwargs,
                                )
                        ui.Spacer(height=10)
                        with ui.HStack():
                            ui.Button(
                                "Next",
                                height=60,
                                clicked_fn=lambda: self._build_instructions_page(self.user_tools_list[0]),
                                **medium_btn_kwargs,
                            )
                            ui.Spacer(width=5)
                            ui.Button(
                                "Cancel",
                                height=60,
                                clicked_fn=self._build_welcome_page,
                                **medium_btn_kwargs,
                            )
        else:
            msg = "No tools selected. Please select at least one tool to include in the pipeline."
            post_notification(msg, status=NotificationStatus.WARNING)

    def _build_instructions_page(self, tool_name):
        """
        this sets up the wizard for the current app, by enable the extension if needed, build the UI, and open the current app window if there is one
        """

        self._prep_page(tool_name)
        self._get_data_from_file(self.pipeline_file, tool_name)  # this sets the parameters for the current app

        # check if this extension is enabled
        if self._current_app_name == "Articulation Inspector":
            # physX tool are not an extension so we skip this check
            pass
        else:
            extension_enabled = (
                omni.kit.app.get_app().get_extension_manager().is_extension_enabled(self._ext_folder_name)
            )
            if not extension_enabled:
                self._enable_extension_popup(self._ext_folder_name, self._current_app_name)

        # build the wizard ui to match the current app
        self._build_wizard_ui(self._app_data)

        # open the current app window if there is one (indicated in the json file)
        if self._ext_window:
            if self._current_app_name == "Articulation Inspector":
                self._settings.set_bool(pxsupportui.SETTINGS_PHYSICS_INSPECTOR_ENABLED, True)
            else:
                self._app_window = ui.Workspace.get_window(self._current_app_name)
                if self._app_window is not None:
                    self._app_window.visible = True

    def _build_wizard_ui(self, app_data):
        """
        this builds the UI for each instruction page
        """
        with self._window.frame:
            with ui.VStack():
                ui.Button(
                    "Start Over",
                    width=100,
                    height=40,
                    clicked_fn=self._build_welcome_page,
                    **small_btn_kwargs,
                )
                self._build_instruction_frame()
                self._build_wizard_footer()

    def _prep_page(self, tool_name):
        """
        close the previous app, setup parameters for this app

        """

        # before resetting the self._current_app_name, close the current tool's window if it has one. physX tool needs a special call
        if self._current_app_name == "Articulation Inspector":
            self._settings.set_bool(pxsupportui.SETTINGS_PHYSICS_INSPECTOR_ENABLED, False)
        elif self._current_app_name == "Isaac Sim Import Wizard":
            # do not close the wizard window
            pass
        else:
            self._app_window = ui.Workspace.get_window(self._current_app_name)
            if self._app_window is not None:
                self._app_window.visible = False

        # if the next_tool_name is in the user_tools_list, then we set the prev_tool_name to the current_tool_name from the user_tools_list
        if tool_name in self.user_tools_list:
            app_idx = self.user_tools_list.index(tool_name)
            self._prev_tool_name = self.user_tools_list[app_idx - 1] if app_idx != 0 else "At the Beginning"
            self._next_tool_name = (
                self.user_tools_list[app_idx + 1] if app_idx + 1 < len(self.user_tools_list) else "End of Pipeline"
            )

        else:
            # we set the prev and next tool names using the full available_tools list
            self.user_tools_list = []  # reset the user_tools_list
            app_idx = self.available_tools_list.index(tool_name)
            self._prev_tool_name = self.available_tools_list[app_idx - 1] if app_idx != 0 else "At the Beginning"
            self._next_tool_name = (
                self.available_tools_list[app_idx + 1]
                if app_idx + 1 < len(self.available_tools_list)
                else "End of Pipeline"
            )

    def _enable_extension_popup(self, ext_folder_name, ext_name):
        message = ext_name + " is not currently enabled. Click 'OK' to enable, 'Cancel' to skip tool"
        self._dialog = MessageDialog(
            title="Enable Extension Message",
            message=message,
            ok_handler=lambda a: self._enable_extension(ext_folder_name, ext_name),
            cancel_handler=lambda b: self._dialog.hide(),
        )
        self._dialog.show()

    def _enable_extension(self, ext_folder_name, ext_name):
        result = omni.kit.app.get_app().get_extension_manager().set_extension_enabled_immediate(ext_folder_name, True)
        self._dialog.hide()
        if result:
            self._sim_folder = (
                omni.kit.app.get_app().get_extension_manager().get_extension_path_by_module(ext_folder_name)
            )
        else:
            msg = f"Failed to enable extension {ext_name}"
            post_notification(msg, status=NotificationStatus.WARNING)

    def _on_clicked_stop(self):
        # close the last app
        self._app_window = ui.Workspace.get_window(self._current_app_name)
        if self._app_window is not None:
            self._app_window.visible = False

        # reset params collected or modified by the wizard
        self.reset()

        # close wizard window
        self._window.visible = False

    def _on_file_select_callback(self, file, path):
        self.custom_file = os.path.join(path, file)
        self.user_file_field.set_value(self.custom_file)
        self.folder_picker.hide()

    def _get_data_from_file(self, file, tool_name):
        with open(file, "r") as f:
            data = json.load(f)

        # build the gui for the current app
        self._app_data = data[tool_name]
        self._current_tool_name = tool_name
        self._current_app_name = data[tool_name]["App Name"]
        self._ext_folder_name = self._app_data["Extension"]
        self._ext_window = self._app_data[
            "Window"
        ]  # the parameter inside the json file that indicates if this tool has its own window
        self._links = self._app_data["Resources"]
        self._sim_folder = (
            omni.kit.app.get_app().get_extension_manager().get_extension_path_by_module(self._ext_folder_name)
        )

    def _on_clicked_dropdown(self, model, val):
        self._build_instructions_page(self.available_tools_list[model.get_item_value_model().as_int])

    def _build_instruction_frame(self):
        with ui.Frame(style=get_style(), spacing=10):
            with ui.VStack(height=0):
                ui.Spacer(height=5)
                ui.Line(style={"color": 0x338A8777}, width=ui.Fraction(1), alignment=ui.Alignment.CENTER)
                ui.Spacer(height=10)
                ui.Label(
                    self._current_app_name,
                    width=0,
                    alignment=ui.Alignment.LEFT_TOP,
                    style={"font_size": 20, "color": 0xFFC7C7C7},
                )
                ui.Spacer(height=10)
                with ui.HStack():
                    ui.Label(
                        "Summary:",
                        **label_kwargs,
                    )
                    with ui.ZStack(style={"ZStack": {"margin": 0}}):
                        ui.Rectangle(height=ui.Fraction(1))
                        with ui.HStack(spacing=0):
                            ui.Spacer(width=5)
                            ui.Label(
                                self._app_data["Summary"],
                                **text_kwargs,
                            )
                ui.Spacer(height=10)
                with ui.HStack():
                    ui.Label(
                        "Basic Instructions:",
                        **label_kwargs,
                    )
                    with ui.ScrollingFrame(
                        height=LABEL_HEIGHT * 15,
                        **scroll_kwargs,
                    ):
                        with ui.ZStack(style={"ZStack": {"margin": 10}}):
                            ui.Rectangle()
                            with ui.HStack(spacing=0):
                                ui.Spacer(width=5)
                                ui.Label(
                                    self._app_data["Basic Instructions"],
                                    **text_kwargs,
                                )
                                ui.Spacer(width=5)
                ui.Spacer(height=10)
                with ui.HStack():
                    ui.Label(
                        "Menu Location:",
                        **label_kwargs,
                        tooltip="Where to find this tool in the GUI",
                    )
                    with ui.ZStack(style={"ZStack": {"margin": 0}}):
                        ui.Rectangle(height=ui.Fraction(1))
                        with ui.HStack(spacing=0):
                            ui.Spacer(width=5)
                            ui.Label(
                                self._app_data["Menu"],
                                **text_kwargs,
                            )
                ui.Spacer(height=10)
                with ui.HStack():
                    ui.Label(
                        "Additional Resources:",
                        width=LABEL_WIDTH / 2,
                        style_type_name_override="Label::label",
                        word_wrap=True,
                        alignment=ui.Alignment.LEFT_TOP,
                    )
                    with ui.VStack():
                        with ui.HStack():
                            ui.Label(
                                "Documentation:",
                                style_type_name_override="Label::label",
                                word_wrap=True,
                                alignment=ui.Alignment.LEFT_TOP,
                                width=LABEL_WIDTH,
                            )
                            with ui.Frame(tooltip="Link to Docs"):
                                ui.Button(
                                    name="DocLink",
                                    width=18,
                                    height=18,
                                    clicked_fn=lambda: on_docs_link_clicked(self._links["Documentation Link"]),
                                    style=get_style()["IconButton.Image::OpenLink"],
                                    alignment=ui.Alignment.LEFT_TOP,
                                )
                        if "API Link" in self._links.keys():
                            with ui.HStack():
                                ui.Label(
                                    "Python API:",
                                    style_type_name_override="Label::label",
                                    word_wrap=True,
                                    alignment=ui.Alignment.LEFT_TOP,
                                    width=LABEL_WIDTH,
                                )
                                with ui.Frame(tooltip="Link to API Doc"):
                                    ui.Button(
                                        name="ApiDocLink",
                                        width=18,
                                        height=18,
                                        clicked_fn=lambda: on_docs_link_clicked(self._links["API Link"]),
                                        style=get_style()["IconButton.Image::OpenLink"],
                                        alignment=ui.Alignment.LEFT_TOP,
                                    )
                        if "Examples Link" in self._links.keys():
                            with ui.HStack():
                                ui.Label(
                                    "Examples:",
                                    style_type_name_override="Label::label",
                                    word_wrap=True,
                                    alignment=ui.Alignment.LEFT_TOP,
                                    width=LABEL_WIDTH,
                                )
                                with ui.Frame(tooltip="Link to Examples"):
                                    ui.Button(
                                        name="ExamplesLink",
                                        width=18,
                                        height=18,
                                        clicked_fn=lambda: on_docs_link_clicked(self._links["Examples Link"]),
                                        style=get_style()["IconButton.Image::OpenLink"],
                                        alignment=ui.Alignment.LEFT_TOP,
                                    )
                        if "Script Folder" in self._links.keys():
                            with ui.HStack():
                                ui.Label(
                                    "Extension Folder:",
                                    style_type_name_override="Label::label",
                                    word_wrap=True,
                                    alignment=ui.Alignment.LEFT_TOP,
                                    width=LABEL_WIDTH,
                                )
                                with ui.Frame(tooltip="Open Containing Folder"):
                                    ui.Button(
                                        name="IconButton",
                                        width=24,
                                        height=24,
                                        clicked_fn=lambda: on_open_folder_clicked(
                                            os.path.join(self._sim_folder, self._ext_folder_name)
                                        ),
                                        style=get_style()["IconButton.Image::OpenFolder"],
                                        alignment=ui.Alignment.LEFT_CENTER,
                                    )

    def _build_wizard_footer(self):
        with ui.Frame():
            with ui.VStack():
                with ui.CollapsableFrame(
                    "Advanced Tips",
                    collapsed=True,
                    style={
                        "font_size": BUTTON_FONT,
                        "background_color": 0xFF2C2C2C,
                        "color": "0xFF00B976",
                    },
                ):
                    with ui.ScrollingFrame(
                        height=LABEL_HEIGHT * 12,
                        **scroll_kwargs,
                    ):
                        with ui.HStack():
                            ui.Spacer(width=10)
                            ui.Label(
                                self._app_data["Advanced Instructions"],
                                **text_kwargs,
                            )
                            ui.Spacer(width=10)
                ui.Spacer(height=10)
                ui.Line(style={"color": 0x338A8777}, width=ui.Fraction(1), alignment=ui.Alignment.CENTER)
                ui.Spacer(height=10)
                with ui.HStack():
                    ui.Label(
                        "Jump To",
                        **label_kwargs,
                    )
                    combo_box = ui.ComboBox(0, *self.full_app_list, width=200, alignment=ui.Alignment.LEFT_CENTER).model
                    combo_box.add_item_changed_fn(self._on_clicked_dropdown)
                with ui.HStack(spacing=5, width=ui.Fraction(1)):
                    prepend_previous = "Previous: "
                    prepend_next = "Next: "
                    if self._prev_tool_name == "At the Beginning":
                        prev_btn_enabled = False
                        prepend_previous = ""
                    else:
                        prev_btn_enabled = True

                    if self._next_tool_name == "End of Pipeline":
                        next_btn_enabled = False
                        prepend_next = ""
                    else:
                        next_btn_enabled = True

                    self._prev_btn = ui.Button(
                        prepend_previous + self._prev_tool_name,
                        height=60,
                        clicked_fn=lambda: self._build_instructions_page(self._prev_tool_name),
                        enabled=prev_btn_enabled,
                        word_wrap=True,
                        **medium_btn_kwargs,
                    )
                    # ui.Spacer(width=5)
                    self._next_btn = ui.Button(
                        prepend_next + self._next_tool_name,
                        height=60,
                        clicked_fn=lambda: self._build_instructions_page(self._next_tool_name),
                        enabled=next_btn_enabled,
                        word_wrap=True,
                        **medium_btn_kwargs,
                    )
                    # ui.Spacer(width=5)
                    ui.Button(
                        "Done",
                        width=80,
                        height=60,
                        clicked_fn=self._on_clicked_stop,
                        **medium_btn_kwargs,
                    )

    def _build_checkboxes(self):

        self._func_list = []  # func_list attached to the checkboxes
        for i in range(len(self.full_app_list)):
            func_handle = SelectFunc(self, i)
            self._func_list.append(func_handle)

        with ui.Frame():
            with ui.VStack():
                with ui.HStack():
                    ui.Label(
                        "Use Custom Pipeline:",
                        **label_kwargs,
                    )
                    self.custom_file_cb = ui.SimpleBoolModel(default_value=False)
                    SimpleCheckBox(False, on_checked_fn=self._on_use_custom_pipeline, model=self.custom_file_cb)

                ui.Spacer(height=5)

                self._selection_frame = ui.Frame(height=200)
                with self._selection_frame:
                    with ui.VStack():
                        ui.Line(style={"color": 0x338A8777}, width=ui.Fraction(1), alignment=ui.Alignment.CENTER)
                        ui.Spacer(height=3)
                        ui.Label(
                            "Check all the tools you wish to include in the pipeline",
                            style={"font_size": 20, "color": 0xFFC7C7C7},
                        )
                        ui.Spacer(height=10)
                        with ui.ScrollingFrame(
                            height=LABEL_HEIGHT * 7,
                            **scroll_kwargs,
                        ):
                            with ui.VGrid(row_count=4, column_count=3):
                                for i in range(1, len(self.full_app_list)):
                                    with ui.HStack():
                                        this_cb_model = ui.SimpleBoolModel(default_value=False)
                                        SimpleCheckBox(
                                            self.selection_state[i],
                                            on_checked_fn=self._func_list[i],
                                            model=this_cb_model,
                                        )
                                        ui.Label(self.available_tools_list[i], word_wrap=True)

                self._select_file_frame = ui.Frame(visible=False)

    def _build_questionnaire_window(self):
        if self.qa is not None:
            self.qa._qa_window.visible = True
            self.qa.start()
        else:
            self.qa = Questionnaire(self, self.available_tools_list)
            self.qa.start()

    def _on_use_custom_pipeline(self, check_state):
        if check_state:
            # build the custom pipeline ui every time, so that it can catch saved pipelines from questionnaires
            self.folder_picker = weakref.proxy(
                FilePickerDialog(
                    title="File Picker",
                    click_apply_handler=weakref.proxy(self)._on_file_select_callback,
                )
            )
            self.folder_picker.hide()

            with self._select_file_frame:
                with ui.VStack():
                    with ui.HStack():
                        ui.Label("Select File", width=LABEL_WIDTH / 1.5)
                        # if saved file from the questionnaire exist, use it. otherwise use the default custom parameters
                        if self.qa is not None and self.qa.save_pipeline_file is not None:
                            self.custom_file = self.qa.save_pipeline_file
                            custom_pipeline_name = self.qa.save_pipeline_name
                        else:
                            custom_pipeline_name = "Import Pipeline"

                        self.user_file_field = ui.StringField(
                            name="StringField", height=0, alignment=ui.Alignment.LEFT_CENTER, read_only=True
                        ).model
                        self.user_file_field.set_value(self.custom_file)

                        add_folder_picker_icon(
                            self._on_file_select_callback,
                            dialog_title="Select User File",
                            button_title="Select File",
                        )

                    ui.Spacer(height=3)
                    with ui.HStack():
                        ui.Label("Pipeline Name", width=LABEL_WIDTH / 1.5)
                        pipeline_name_model = ui.SimpleStringModel(custom_pipeline_name)
                        self.pipeline_name = ui.StringField(model=pipeline_name_model)

        self._selection_frame.visible = not check_state
        self._select_file_frame.visible = check_state
        self._use_custom_file = check_state


class SelectFunc:
    """
    this class is used to attach the checkboxes so that the checkboxes UI can be iteratively created with the correct functions attached to them
    """

    def __init__(self, ext, idx):
        self.idx = idx
        self.ext = ext

    def __call__(self, check_state):
        self.ext.selection_state[self.idx] = check_state
