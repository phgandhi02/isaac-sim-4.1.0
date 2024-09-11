# Copyright (c) 2022-2024, NVIDIA CORPORATION. All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto. Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.
#


import json
import os
import weakref

import omni.kit.app
import omni.ui as ui
from omni.isaac.import_wizard.ui_utils import (
    HIGHLIGHT_COLOR,
    HOVER_COLOR,
    CheckBox,
    Singleton,
    header_kwargs,
    label_kwargs,
    scroll_kwargs,
    text_kwargs,
    tool_btn_kwargs,
    vertical_scroll_kwargs,
)
from omni.isaac.ui.element_wrappers import ScrollingWindow
from omni.isaac.ui.style import get_style
from omni.isaac.ui.ui_utils import add_folder_picker_icon
from omni.kit.notification_manager import NotificationStatus, post_notification
from omni.kit.window.extensions import SimpleCheckBox
from omni.kit.window.filepicker.dialog import FilePickerDialog
from omni.kit.window.popup_dialog.dialog import PopupDialog
from omni.kit.window.property.templates import LABEL_HEIGHT, LABEL_WIDTH
from omni.ui_query.query import OmniUIQuery

EXTENSION_FOLDER_PATH = omni.kit.app.get_app().get_extension_manager().get_extension_path_by_module(__name__)
WINDOW_NAME = "Getting Started with Isaac Sim Questionnaire"


@Singleton
class Questionnaire:
    def __init__(self, wizard_ext, available_tools):
        self.qa_file = os.path.join(EXTENSION_FOLDER_PATH, "data", "questionnaire.json")
        self.available_tools = available_tools
        self._wizard_ext = wizard_ext
        self.save_popup = None
        self.reset_params()

        # Build Window
        self._qa_window = ui.Window(
            title=WINDOW_NAME,
            width=700,
            height=450,
            visible=True,
            horizontal_scrollbar_policy=ui.ScrollBarPolicy.SCROLLBAR_AS_NEEDED,
            vertical_scrollbar_policy=ui.ScrollBarPolicy.SCROLLBAR_ALWAYS_ON,
        )

        self._qa_window.set_visibility_changed_fn(self.on_window)

        self._build_skeleton_ui()
        self._build_all_frames()

    def start(self):
        # if start is explicitly called, clear all the questions and tools and uncheck all the checkboxes
        self.clear_old_questions(self.qa_frame_list[0].name)
        self.clear_old_tools(self.tool_frame_list[0].name)
        for checkbox in self.checkbox_frame_dict["begin"]:
            checkbox.set_value(False)

        # start by showing the first question
        self.show_next_question("begin")

    def on_window(self, visible):
        if visible:
            self.reset_params()
            self._qa_window.visible = True
            self.start()
        else:
            self._qa_window.visible = False

    def on_shutdown(self):
        self._qa_window.visible = False
        if self.save_popup is not None:
            self.save_popup.on_shutdown()

    def reset_params(self):
        self.next_qa_key = "begin"
        self.save_pipeline_file = None
        self.save_pipeline_name = None
        self.data = json.load(open(self.qa_file, "r"))

    def _build_skeleton_ui(self):
        """
        setup the window layout with Questions on the left and Tools on the list. Both are empty at the start
        """
        self.qa_stack = None
        self.tool_stack = None
        # build the window
        with self._qa_window.frame:
            with ui.VStack():
                with ui.HStack():
                    with ui.VStack(height=ui.Fraction(0.8)):
                        ui.Label(
                            "Questions",
                            **header_kwargs,
                        )
                        ui.Spacer(height=20)
                        with ui.ScrollingFrame(**scroll_kwargs):
                            self.qa_stack = ui.VStack(spacing=5)
                    with ui.VStack(width=300):
                        ui.Label("Tools to Try", **header_kwargs)
                        ui.Spacer(height=20)
                        with ui.ScrollingFrame(**vertical_scroll_kwargs, height=ui.Fraction(1), width=ui.Percent(100)):
                            self.tool_stack = ui.VStack(height=0, spacing=5)

                with ui.HStack(height=40, spacing=20):
                    with ui.Frame(title="Notes", height=20, width=ui.Percent(70), word_wrap=True):
                        self.notes = ui.Label(
                            "",
                            alignment=ui.Alignment.LEFT,
                            style={"font_size": 16, "color": 0x0},
                            world_wrap=True,
                        )
                    ui.Button(
                        "  Save this Pipeline  ",
                        height=40,
                        width=0,
                        style={
                            "font_size": 20,
                            "color": HIGHLIGHT_COLOR,
                            "border_color": "0xFF555555",
                            "border_width": 4,
                            "border_radius": 5,
                            ":hovered": {"background_color": HOVER_COLOR},
                        },
                        clicked_fn=self._on_save_pipeline,
                    )
        return

    def _build_all_frames(self):
        """
        Build all the questions and tools into frames and add them to the respective stacks, keep them invisible at start
        """
        self.qa_frame_list = []
        self.tool_frame_list = []
        self.checkbox_frame_dict = {}

        # extract all the questions and build each one into a frame
        for qa_key in self.data.keys():
            if qa_key != "":
                question_frame = self._make_question_frame(qa_key)
                self.qa_frame_list.append(question_frame)

        # extract all the tools and build each one into a frame
        for tool_name in self.available_tools:
            tool_name = tool_name.replace(" ", "_")
            if tool_name != "":
                tool_frame = self._make_tool_btn_frame(tool_name)
                self.tool_frame_list.append(tool_frame)

        return

    def show_next_question(self, qa_key):
        """
        show the question by finding its frame in the qa_stack and make it visible
        """
        # find the frame via the name
        if qa_key == "end":
            self.update_notes("end")
        else:
            qa_stack_path = OmniUIQuery.get_widget_path(self._qa_window, self.qa_stack)
            frame_by_name = OmniUIQuery.find_widget(qa_stack_path + "/" + qa_key)
            frame_by_name.visible = True
            self.update_notes()

    def show_next_tool_btn(self, tool_name):
        """
        show the tool by finding its frame in the tool_stack and make it visible
        """
        if tool_name != "":
            tool_stack_path = OmniUIQuery.get_widget_path(self._qa_window, self.tool_stack)
            frame_by_name = OmniUIQuery.find_widget(tool_stack_path + "/" + tool_name)
            frame_by_name.visible = True

    def _make_question_frame(self, qa_key):
        # setup for build answer frame through a loop
        question_text = self.data[qa_key]["q_text"]
        answer_dict = self.data[qa_key]["a_options"]
        answer_texts = [item["a_text"] for item in answer_dict.values()]

        num_answers = len(answer_dict)

        # populate the checkbox callback first
        checkbox_callback_list = []
        for i in range(num_answers):
            checkbox_callback_list.append(ButtonMagic(self, i, qa_key, answer_dict))

        checkbox_frame_list = []
        question_frame = ui.Frame(height=0, style={"font_size": 18}, identifier=qa_key, visible=False, name=qa_key)
        with question_frame:
            with ui.VStack(style=get_style(), spacing=2):
                ui.Label(question_text, word_wrap=True)  # question
                ui.Spacer(height=2)
                for i in range(num_answers):
                    checkbox_frame = CheckBox(
                        answer_texts[i], default_value=False, on_click_fn=checkbox_callback_list[i]
                    )
                    checkbox_frame_list.append(checkbox_frame)

                ui.Spacer(height=2)

        self.qa_stack.add_child(question_frame)
        self.checkbox_frame_dict[qa_key] = checkbox_frame_list

        return question_frame

    def _make_tool_btn_frame(self, tool_name):
        tool_btn_func = lambda: self.open_wizard(tool_name)
        tool_frame = ui.Frame(name=tool_name, visible=False, identifier=tool_name, width=ui.Fraction(1))
        with tool_frame:
            with ui.HStack():
                ui.Spacer(width=20)
                ui.Button(
                    tool_name.replace("_", " "),
                    clicked_fn=tool_btn_func,
                    **tool_btn_kwargs,
                )
                ui.Spacer(width=20)

        self.tool_stack.add_child(tool_frame)

        return tool_frame

    def open_wizard(self, tool_name):
        """
        open the wizard to the page of the given tool
        """
        self._wizard_ext._build_instructions_page(
            tool_name.replace("_", " ")
        )  # need replace the underscore with space back

    def update_notes(self, end=None):
        if end:
            text = "Questionnaire Complete. \nSave the pipeline if you wish to use it for future workflows."
            note_font_size = 20
            note_color = HIGHLIGHT_COLOR
        else:
            text = "Click on the tools listed on the right for more information"
            note_font_size = 14
            note_color = "0xFFFFFFFF"

        self.notes.text = text
        self.notes.style = {"font_size": note_font_size, "color": note_color}
        # with ui.VStack():
        #     ui.Spacer(height=20)

    def _on_save_pipeline(self):
        # setup for save pipeline with defaults
        self.save_pipeline_file = os.path.join(EXTENSION_FOLDER_PATH, "data", "custom_pipeline.json")
        self.save_pipeline_name = "Saved Pipeline"

        # go through the tool stack and get the visible tools
        tool_stack_path = OmniUIQuery.get_widget_path(self._qa_window, self.tool_stack)
        find_all_frames = OmniUIQuery.find_widgets(tool_stack_path + "/*")
        visible_tools = [frame.name for frame in find_all_frames if frame.visible]
        save_array = [tool.replace("_", " ") for tool in visible_tools if tool != ""]
        if len(save_array) == 0:
            post_notification("No tools selected to save", status=NotificationStatus.INFO)
            return
        self.save_popup = SavePipelinePopup(
            title="Save Pipeline",
            ok_handler=self._on_ok_save,
            cancel_handler=self._on_cancel_save,
            save_array=save_array,
            save_file=self.save_pipeline_file,
            save_title=self.save_pipeline_name,
        )
        self.save_popup.show()

    def _on_ok_save(self, dialog):
        self.save_pipeline_file = self.save_popup.get_save_pipeline_file()
        self.save_pipeline_name = self.save_popup.get_save_pipeline_name()
        self.overwrite = self.save_popup.get_overwrite()
        save_dict = {self.save_pipeline_name: dialog.pipeline_array}

        if self.overwrite:
            with open(self.save_pipeline_file, "w") as f:
                json.dump(save_dict, f, indent=4)
        else:
            # append file if it exists, otherwise write a new one
            try:
                with open(self.save_pipeline_file, "r") as file:
                    data = json.load(file)
                    if self.save_pipeline_name in data.keys():
                        msg = "Pipeline with that name already exists, OVERWRITING"
                        post_notification(msg, status=NotificationStatus.WARNING)
                        data[self.save_pipeline_name] = dialog.pipeline_array

                    else:
                        data = {
                            **save_dict,
                            **data,
                        }  # appending the new pipeline to the beginning of the existing file for easy access
                    with open(self.save_pipeline_file, "w") as file:
                        json.dump(data, file, indent=4)
            except FileNotFoundError:
                msg = "File not found, creating new file"
                post_notification(msg, status=NotificationStatus.INFO)
                with open(self.save_pipeline_file, "w") as new_file:
                    json.dump(save_dict, new_file, indent=4)

        # if user saved the pipeline, automatically check the custom file checkbox in the main wizard window
        self._wizard_ext.custom_file_cb.set_value(True)
        self._wizard_ext._on_use_custom_pipeline(True)

        self.on_shutdown()

    def _on_cancel_save(self, dialog):
        self.save_pipeline_file = None
        self.save_pipeline_name = None
        self.ovewrite = False
        self.save_popup.on_shutdown()

    def clear_old_questions(self, qa_key):
        """
        if a user changes an answer to a question, clear all the questions below the one they clicked on
        """
        ## TODO: this assumes that the questions/frames are added in the order they should appear, probably not true if questions orders are not consistent or not in the order they should appear
        qa_stack_path = OmniUIQuery.get_widget_path(self._qa_window, self.qa_stack)
        find_all_frames = OmniUIQuery.find_widgets(qa_stack_path + "/*")
        frame_name_list = [frame.name for frame in find_all_frames]

        idx_clicked_qa = frame_name_list.index(qa_key)
        for i in range(len(find_all_frames) - 1, idx_clicked_qa, -1):
            find_all_frames[i].visible = False
            # uncheck all boxes for the frames that are made invisible
            for checkbox in self.checkbox_frame_dict[find_all_frames[i].name]:
                checkbox.set_value(False)
        return

    def clear_old_tools(self, tool_key):
        """
        if a user changes an answer to a question, clear all the irrelevant tools
        """
        tool_stack_path = OmniUIQuery.get_widget_path(self._qa_window, self.tool_stack)
        find_all_frames = OmniUIQuery.find_widgets(tool_stack_path + "/*")
        frame_name_list = [
            frame.name for frame in find_all_frames
        ]  # this should be the same as the available_tools, if the order of this is weird, use available_tools

        ## check for back tool_key
        if tool_key != "":
            idx_clicked_qa = frame_name_list.index(tool_key)
            for i in range(len(find_all_frames) - 1, idx_clicked_qa - 1, -1):
                find_all_frames[i].visible = False

        return


class ButtonMagic:
    def __init__(self, ext, idx, qa_key, answer_dict):
        self.idx = idx
        self.ext = ext
        self.qa_key = qa_key
        self.next_qa_key = [item["next_qa"] for item in answer_dict.values()][idx]
        self.tool_key = [item["tool"].replace(" ", "_") for item in answer_dict.values()][idx]
        self.num_answers = len(answer_dict)

    def __call__(self, check_state):
        # clear questions and tools below the current selection
        # TODO: this is going through ALL the frames each time, maybe could be more efficient if only clear the visible ones
        self.ext.clear_old_questions(self.qa_key)
        self.ext.clear_old_tools(self.tool_key)

        if check_state:
            for i in range(self.num_answers):
                # uncheck all the other boxes, only one answer allowed at a time
                if i != self.idx:
                    other_qa = self.ext.checkbox_frame_dict[self.qa_key][i]
                    other_qa.set_value(False)

            # show the tool assciated with the answer
            if self.tool_key != "":
                # TODO: potentially allow for multiple tools associated with each answer
                self.ext.show_next_tool_btn(self.tool_key)

            # move on to the next question
            self.ext.show_next_question(self.next_qa_key)


class SavePipelinePopup(PopupDialog):
    def __init__(self, title, ok_handler, cancel_handler, save_array, save_file, save_title):
        self.pipeline_array = save_array
        self.save_pipeline_file = save_file
        self.save_pipeline_name = save_title
        self.overwrite = False

        # get the folder picker ready
        self.folder_picker = weakref.proxy(
            FilePickerDialog(
                title="File Picker",
                click_apply_handler=weakref.proxy(self)._on_file_select_callback,
            )
        )
        self.folder_picker.hide()

        super().__init__(title=title, ok_handler=ok_handler, cancel_handler=cancel_handler)

    # overload the destory function to shutdown the popup as well
    def on_shutdown(self):
        self.folder_picker.hide()
        self.hide()

    # overloading the build_widget to add necessary UI
    def _build_widgets(self):
        # build the popup window
        with self._window.frame:
            with ui.VStack():
                # first list all the tools in the order they'll be saved in
                with ui.HStack():
                    ui.Label("Pipeline List", **label_kwargs)
                    ui.Label("\n".join(self.pipeline_array), **text_kwargs)
                ui.Spacer(height=10)

                # select file path and pipeline name to save
                with ui.HStack():
                    ui.Label("Select File", **label_kwargs)
                    self.user_file_field = ui.StringField(
                        name="StringField", height=0, alignment=ui.Alignment.LEFT_CENTER, read_only=True
                    ).model
                    self.user_file_field.set_value(self.save_pipeline_file)

                    add_folder_picker_icon(
                        self._on_file_select_callback,
                        dialog_title="Select File to Save Pipeline To",
                        button_title="Select File",
                    )

                ui.Spacer(height=3)
                with ui.HStack():
                    ui.Label("Pipeline Name", **label_kwargs)
                    pipeline_name_model = ui.SimpleStringModel(self.save_pipeline_name)
                    self.pipeline_name = ui.StringField(model=pipeline_name_model)

                with ui.HStack():
                    ui.Label(
                        "Overwrite File?",
                        width=0,
                    )
                    cb = ui.SimpleBoolModel(default_value=False)
                    SimpleCheckBox(False, on_checked_fn=self._on_overwrite, model=cb)

                self._build_ok_cancel_buttons()

    def _on_file_select_callback(self, file, path):
        self.custom_file = os.path.join(path, file)
        self.user_file_field.set_value(self.custom_file)
        self.folder_picker.hide()
        self.show()

    def _on_overwrite(self, check_state):
        self.overwrite = check_state

    def get_save_pipeline_file(self):
        return self.user_file_field.get_value_as_string()

    def get_save_pipeline_name(self):
        return self.pipeline_name.model.get_value_as_string()

    def get_overwrite(self):
        return self.overwrite
