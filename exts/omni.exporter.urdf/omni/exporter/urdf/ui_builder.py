# Copyright (c) 2022-2023, NVIDIA CORPORATION. All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto. Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.
#

import os
import pathlib
from pathlib import Path

import omni.timeline
import omni.ui as ui
from omni.isaac.core.utils.stage import open_stage
from omni.isaac.ui.element_wrappers import Button, CheckBox, CollapsableFrame, StringField
from omni.isaac.ui.ui_utils import get_style
from omni.usd import StageEventType

if os.name == "nt":
    file_dir = pathlib.Path(os.path.dirname(os.path.abspath(__file__)))
    exporter_urdf_dir = file_dir.joinpath(pathlib.Path("../../../pip_prebundle")).resolve()
    os.add_dll_directory(exporter_urdf_dir.__str__())

import nvidia.srl.tools.logger as logger
from nvidia.srl.from_usd.to_urdf import UsdToUrdf


class UIBuilder:
    def __init__(self):
        # Frames are sub-windows that can contain multiple UI elements
        self.frames = []
        # UI elements created using a UIElementWrapper instance
        self.wrapped_ui_elements = []

        # Get access to the timeline to control stop/pause/play programmatically
        self._timeline = omni.timeline.get_timeline_interface()

        self.log_level = logger.level_from_name("ERROR")

        self._on_init()

    def _on_init(self):
        self._data_params = dict()
        self._data_params["input_path"] = None
        self._data_params["output_path"] = None
        self._data_params["mesh_dir"] = None
        self._data_params["mesh_path_prefix"] = ""
        self._data_params["root"] = None
        self._data_params["visualize_collision_meshes"] = False

    def on_menu_callback(self):
        pass

    def on_timeline_event(self, event):
        if event.type == int(omni.timeline.TimelineEventType.STOP):
            self._scenario_state_btn.reset()
            self._scenario_state_btn.enabled = False

    def on_stage_event(self, event):
        if event.type == int(StageEventType.OPENED):
            # If the user opens a new stage, the extension should completely reset
            self._reset_extension()

    def cleanup(self):
        for ui_elem in self.wrapped_ui_elements:
            ui_elem.cleanup()

    def build_ui(self):
        def is_usd_or_urdf_path(file_path: str):
            # Filter file paths shown in the file picker to only be USD or Python files
            _, ext = os.path.splitext(file_path.lower())
            return ext == ".usd" or ext == ".urdf"

        with ui.VStack(style=get_style(), spacing=5, height=0):
            input_field = StringField(
                "USD Path",
                default_value="",
                tooltip="Path to the USD file to be exported, if empty use current stage",
                read_only=False,
                multiline_okay=False,
                on_value_changed_fn=self._on_input_field_value_changed_fn,
                use_folder_picker=True,
                item_filter_fn=is_usd_or_urdf_path,
                folder_dialog_title="select a USD file",
                folder_button_title="Select File",
            )
            self.wrapped_ui_elements.append(input_field)

            output_field = StringField(
                "Output File/Directory",
                default_value="",
                tooltip="Path to where the URDF file will be created",
                read_only=False,
                multiline_okay=False,
                on_value_changed_fn=self._on_output_field_value_changed_fn,
                use_folder_picker=True,
                item_filter_fn=is_usd_or_urdf_path,
                folder_dialog_title="Set Output File Or Directory",
                folder_button_title="Select File/Directory",
            )
            self.wrapped_ui_elements.append(output_field)

            button = Button("", "Export", on_click_fn=self._on_export_button_clicked_fn)

            self.wrapped_ui_elements.append(button)

            frame = CollapsableFrame(
                title="Advanced Options",
                collapsed=True,
            )
            self.wrapped_ui_elements.append(frame)
            with frame:
                with ui.VStack(style=get_style(), spacing=5, height=0):
                    mesh_field = StringField(
                        "Mesh Directory Path",
                        default_value="",
                        tooltip="Path to where directory where mesh files will be saved. Defaults to 'meshes'.",
                        read_only=False,
                        multiline_okay=False,
                        on_value_changed_fn=self._on_mesh_field_value_changed_fn,
                        use_folder_picker=True,
                        item_filter_fn=is_usd_or_urdf_path,
                    )
                    self.wrapped_ui_elements.append(mesh_field)

                    mesh_path_prefix_field = StringField(
                        "Mesh Path Prefix",
                        default_value="",
                        tooltip="Prefix to add to URDF mesh filename values (e.g. 'file://')",
                        read_only=False,
                        multiline_okay=False,
                        on_value_changed_fn=self._on_mesh_path_prefix_field_value_changed_fn,
                        use_folder_picker=False,
                        item_filter_fn=None,
                    )
                    self.wrapped_ui_elements.append(mesh_path_prefix_field)

                    root_path_field = StringField(
                        "Root Prim Path",
                        default_value="",
                        tooltip="Root prim path of the robot to be exported. Defaults to the default prim.",
                        read_only=False,
                        multiline_okay=False,
                        on_value_changed_fn=self._on_root_field_value_changed_fn,
                        use_folder_picker=False,
                        item_filter_fn=None,
                    )
                    self.wrapped_ui_elements.append(root_path_field)

                    stage_visualize_collisions_check_box = CheckBox(
                        "Visualize Collisions",
                        default_value=False,
                        tooltip="Visualization collider meshes even if their visibility is disabled.",
                        on_click_fn=self._on_visualize_collisions_check_box_click_fn,
                    )
                    self.wrapped_ui_elements.append(stage_visualize_collisions_check_box)

    def _on_input_field_value_changed_fn(self, new_value: str):
        self._data_params["input_path"] = new_value

    def _on_output_field_value_changed_fn(self, new_value: str):
        self._data_params["output_path"] = new_value

    def _on_mesh_path_prefix_field_value_changed_fn(self, new_value: str):
        self._data_params["mesh_path_prefix"] = new_value

    def _on_root_field_value_changed_fn(self, new_value: str):
        self._data_params["root"] = new_value

    def _on_mesh_field_value_changed_fn(self, new_value: str):
        self._data_params["mesh_dir"] = new_value

    def _on_visualize_collisions_check_box_click_fn(self, value: bool):
        self._data_params["visualize_collision_meshes"] = value

    def _on_export_button_clicked_fn(self):
        root = self._data_params["root"]
        if root == "":
            root = None

        usd_to_urdf_kwargs = {
            "node_names_to_remove": None,
            "edge_names_to_remove": None,
            "root": root,
            "parent_link_is_body_1": None,
            "log_level": self.log_level,
        }

        usd_path = self._data_params["input_path"]
        if usd_path == "":
            usd_path = None

        if usd_path is None:
            stage = omni.usd.get_context().get_stage()
            usd_to_urdf = UsdToUrdf(stage, **usd_to_urdf_kwargs)
        else:
            usd_to_urdf = UsdToUrdf.init_from_file(usd_path, **usd_to_urdf_kwargs)

        urdf_output_path = self._data_params["output_path"]
        if urdf_output_path == "" or urdf_output_path is None:
            if usd_path is None:
                raise ValueError("Must specify an URDF output path.")
            urdf_output_path = Path(usd_path).parent

        mesh_dir = self._data_params["mesh_dir"]
        if mesh_dir == "":
            mesh_dir = None

        output_path = usd_to_urdf.save_to_file(
            urdf_output_path=urdf_output_path,
            visualize_collision_meshes=self._data_params["visualize_collision_meshes"],
            mesh_dir=mesh_dir,
            mesh_path_prefix=self._data_params["mesh_path_prefix"],
        )

        if usd_path is None:
            input_path = stage.GetRootLayer().realPath
        else:
            input_path = usd_path
        print("Converted USD to URDF.")
        print(f"    Input: {input_path}")
        print(f"    Output: {output_path}")

    def _reset_extension(self):
        pass
