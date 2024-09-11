# Copyright (c) 2018-2023, NVIDIA CORPORATION. All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto. Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.
#

import gc
import weakref

import carb
import omni.ext
import omni.kit.commands
import omni.kit.utils
import omni.ui as ui
import omni.usd
from omni.isaac.ui.element_wrappers import ScrollingWindow
from omni.isaac.ui.menu import make_menu_item_description
from omni.isaac.ui.style import get_style
from omni.isaac.ui.ui_utils import btn_builder, cb_builder, combo_cb_str_builder, str_builder
from omni.kit.menu.utils import MenuItemDescription, add_menu_items, remove_menu_items
from pxr import Gf, Sdf, Usd, UsdGeom, UsdShade

from .mesh_merger import MeshMerger

EXTENSION_NAME = "Mesh Merge Tool"


class Extension(omni.ext.IExt):
    def on_startup(self, ext_id: str):
        """Called to load the extension"""

        self._stage = omni.usd.get_context().get_stage()
        self._window = ScrollingWindow(
            title=EXTENSION_NAME, width=600, height=400, visible=False, dockPreference=ui.DockPreference.LEFT_BOTTOM
        )
        self._window.deferred_dock_in("Console", omni.ui.DockPolicy.DO_NOTHING)
        self._window.set_visibility_changed_fn(self._on_window)
        self._menu_items = [
            make_menu_item_description(ext_id, EXTENSION_NAME, lambda a=weakref.proxy(self): a._menu_callback())
        ]
        add_menu_items(self._menu_items, "Isaac Utils")
        self.models = {}
        self.parent_xform = None

        self.mesh_merger = MeshMerger(self._stage)

    def build_ui(self):
        with self._window.frame:
            with ui.HStack(spacing=5):
                with ui.VStack(height=0, spacing=5):
                    input_frame = ui.CollapsableFrame(
                        title="Input",
                        style=get_style(),
                        style_type_name_override="CollapsableFrame",
                        horizontal_scrollbar_policy=ui.ScrollBarPolicy.SCROLLBAR_AS_NEEDED,
                        vertical_scrollbar_policy=ui.ScrollBarPolicy.SCROLLBAR_ALWAYS_ON,
                    )

                    def input_changed(model):
                        if input_frame.collapsed:
                            input_frame.title = "Input ({})".format(model.get_value_as_string())

                    def collapsed_changed_fn(frame, base_txt, model, collapsed):
                        if collapsed:
                            frame.title = base_txt + " ({})".format(model.get_value_as_string())
                        else:
                            frame.title = base_txt

                    with input_frame:
                        with ui.VStack(spacing=2, height=0):
                            self.models["input_mesh"] = str_builder("Source Prim", read_only=True)
                            self.models["input_mesh"].set_value("No Mesh Selected")
                            self.models["input_mesh"].add_value_changed_fn(lambda a: input_changed(a))
                            self.models["submesh"] = str_builder("Submeshes", read_only=True)
                            self.models["subset"] = str_builder("Geometry Subsets", read_only=True)
                            self.models["materials"] = str_builder("Materials", read_only=True)

                    input_frame.set_collapsed_changed_fn(
                        lambda c, f=input_frame, m=self.models["input_mesh"]: collapsed_changed_fn(f, "Input", m, c)
                    )
                    output_frame = ui.CollapsableFrame(
                        title="Output",
                        style=get_style(),
                        style_type_name_override="CollapsableFrame",
                        horizontal_scrollbar_policy=ui.ScrollBarPolicy.SCROLLBAR_AS_NEEDED,
                        vertical_scrollbar_policy=ui.ScrollBarPolicy.SCROLLBAR_ALWAYS_ON,
                    )
                    with output_frame:
                        with ui.VStack(spacing=2, height=0):
                            self.models["output_mesh"] = str_builder("Destination Prim", read_only=True)
                            self.models["output_mesh"].set_value("No Mesh Selected")

                            self.models["output_subset"] = str_builder("Geometry Subsets", read_only=True)
                    output_frame.set_collapsed_changed_fn(
                        lambda c, f=output_frame, m=self.models["output_mesh"]: collapsed_changed_fn(f, "Output", m, c)
                    )
                with ui.VStack(spacing=3, height=0):
                    self.parent_xform = cb_builder(
                        label="Clear Parent Transform",
                        tooltip="If selected, Creates merged mesh with origin at global orign, otherwise keeps origin at parent's origin",
                        on_clicked_fn=lambda a: setattr(self.mesh_merger, "clear_parent_xform", a),
                    )
                    self.deactivate_source = cb_builder(
                        label="Deactivate source assets",
                        tooltip="If selected, Deactivates all meshes used for creating the merged mesh",
                        on_clicked_fn=lambda a: setattr(self.mesh_merger, "deactivate_source", a),
                    )
                    self.override_looks_directory = combo_cb_str_builder(
                        "Combine Materials",
                        tooltip="If selected, replaces the path to all selected materials with the prim path provided.\n If the path doesn't exist, a Material scope will be created and all materials used will be moved to it",
                        on_clicked_fn=lambda a: self._on_stage_event(),
                        default_val=[False, ""],
                    )
                    self.override_looks_directory[1].add_value_changed_fn(
                        lambda a: setattr(self.mesh_merger, "materials_destination", a.get_value_as_string())
                    )
                    btn_builder(label="Merge Selected Prim", text="Merge", on_clicked_fn=self._merge_mesh)

    def _menu_callback(self):
        self._window.visible = not self._window.visible

    def _on_window(self, visible):
        if self._window.visible:
            self._usd_context = omni.usd.get_context()
            if self._usd_context is not None:
                self._selection = self._usd_context.get_selection()
                self._events = self._usd_context.get_stage_event_stream()
                self._stage_event_sub = self._events.create_subscription_to_pop(
                    self._on_stage_event, name="Mesh merge tool stage event"
                )
            self.build_ui()
            self._on_stage_event()
        else:
            self._stage_event_sub = None

    def _on_stage_event(self, event=None):  # Empty event is a forced update from UI
        if self._window.visible:
            self._stage = omni.usd.get_context().get_stage()
            if not event or event.type == int(omni.usd.StageEventType.SELECTION_CHANGED):
                selection = self._selection.get_selected_prim_paths()
                if selection:
                    self.mesh_merger.combine_materials = self.override_looks_directory[0].get_value_as_bool()
                    self.mesh_merger.materials_destination = self.override_looks_directory[1].get_value_as_string()
                    self.mesh_merger.update_selection(selection=selection, stage=self._stage)
                    curr_prim = self._stage.GetPrimAtPath(selection[0])

                    if len(selection) == 1:
                        self.models["input_mesh"].set_value(selection[0])
                    else:
                        self.models["input_mesh"].set_value(f"{selection[0]} (and {len(selection)-1} more)")
                    self.models["submesh"].set_value(self.mesh_merger.total_meshes)
                    self.models["subset"].set_value(self.mesh_merger.total_subsets)
                    self.models["materials"].set_value(self.mesh_merger.total_materials)
                    self.mesh_merger.output_mesh = "/Merged/" + str(curr_prim.GetName())
                    self.models["output_mesh"].set_value(self.mesh_merger.output_mesh)
                    self.models["output_subset"].set_value(self.mesh_merger.total_materials)
                else:
                    self.models["input_mesh"].set_value(f"None Selected")
                    self.models["submesh"].set_value(0)
                    self.models["subset"].set_value(0)
                    self.models["materials"].set_value(0)
                    self.models["output_mesh"].set_value("")
                    self.models["output_subset"].set_value(0)

    def _merge_mesh(self):
        self.mesh_merger.merge_meshes()

    def on_shutdown(self):
        """Called when the extesion us unloaded"""
        remove_menu_items(self._menu_items, "Isaac Utils")
        self._window = None
        gc.collect()
