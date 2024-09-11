# Copyright (c) 2020-2023, NVIDIA CORPORATION. All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto. Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.
#

# from omni.isaac.examples.ur10_palletizing.ur10_palletizing import BinStacking
import asyncio
import os

import omni
import omni.ui as ui
from omni.isaac.cortex.cortex_world import CortexWorld
from omni.isaac.examples.base_sample import BaseSampleExtension
from omni.isaac.examples.franka_cortex.franka_cortex import FrankaCortex
from omni.isaac.ui.ui_utils import btn_builder, cb_builder, dropdown_builder, get_style, str_builder


class FrankaCortexExtension(BaseSampleExtension):
    def on_startup(self, ext_id: str):
        super().on_startup(ext_id)
        ext_manager = omni.kit.app.get_app().get_extension_manager()
        sample_behaviors_id = ext_manager.get_enabled_extension_id("omni.isaac.cortex.sample_behaviors")
        behavior_path = (
            omni.kit.app.get_app().get_extension_manager().get_extension_path(sample_behaviors_id)
            + "/omni/isaac/cortex/sample_behaviors/franka"
        )

        self.behavior_map = {
            "Block Stacking": f"{behavior_path}/block_stacking_behavior.py",
            "Simple State Machine": f"{behavior_path}/simple/simple_state_machine.py",
            "Simple Decider Network": f"{behavior_path}/simple/simple_decider_network.py",
            "Peck State Machine": f"{behavior_path}/peck_state_machine.py",
            "Peck Decider Network": f"{behavior_path}/peck_decider_network.py",
            "Peck Game": f"{behavior_path}/peck_game.py",
        }
        self.selected_behavior = "Block Stacking"
        super().start_extension(
            menu_name="Cortex",
            submenu_name="",
            name="Franka Cortex Examples",
            title="Franka Cortex Examples",
            doc_link="https://docs.omniverse.nvidia.com/isaacsim/latest/cortex_tutorials/tutorial_cortex_4_franka_block_stacking.html#isaac-sim-app-tutorial-cortex-4-franka-block-stacking",
            overview="This Example shows how to Use Cortex for multiple behaviors robot and Cortex behaviors in Isaac Sim.\n\nPress the 'Open in IDE' button to view the source code.",
            sample=FrankaCortex(self.on_diagnostics),
            file_path=os.path.abspath(__file__),
            number_of_extra_frames=2,
        )
        self.task_ui_elements = {}
        frame = self.get_frame(index=0)
        self.build_task_controls_ui(frame)
        self.loaded = False
        return

    def _on_load_world(self):
        self._sample.behavior = self.get_behavior()
        self.loaded = True
        super()._on_load_world()

    def on_diagnostics(self, diagnostic, decision_stack):
        if diagnostic:
            self.diagostic_model.set_value(diagnostic)

        self.state_model.set_value(decision_stack)
        self.diagnostics_panel.visible = bool(diagnostic)

    def get_world(self):
        return CortexWorld.instance()

    def get_behavior(self):
        return self.behavior_map[self.selected_behavior]

    def _on_start_button_event(self):
        asyncio.ensure_future(self.sample.on_event_async())
        self.task_ui_elements["Start"].enabled = False
        return

    def post_reset_button_event(self):
        self.task_ui_elements["Start"].enabled = True
        return

    def post_load_button_event(self):
        self.task_ui_elements["Start"].enabled = True
        return

    def post_clear_button_event(self):
        self.task_ui_elements["Start"].enabled = False
        return

    def __on_selected_behavior_changed(self, selected_index):
        self.selected_behavior = selected_index
        if self.loaded:
            asyncio.ensure_future(self._sample.load_behavior(self.get_behavior()))
            self.on_diagnostics("", "")

    def build_task_controls_ui(self, frame):
        with self._controls_frame:
            with ui.VStack(style=get_style(), spacing=5, height=0):
                self.task_ui_elements["Selected Behavior"] = dropdown_builder(
                    "Selected Behavior",
                    items=[
                        "Block Stacking",
                        "Simple State Machine",
                        "Simple Decider Network",
                        "Peck State Machine",
                        "Peck Decider Network",
                        "Peck Game",
                    ],
                    on_clicked_fn=self.__on_selected_behavior_changed,
                )
                dict = {
                    "label": "Load World",
                    "type": "button",
                    "text": "Load",
                    "tooltip": "Load World and Task",
                    "on_clicked_fn": self._on_load_world,
                }
                self._buttons["Load World"] = btn_builder(**dict)
                self._buttons["Load World"].enabled = True
                dict = {
                    "label": "Reset",
                    "type": "button",
                    "text": "Reset",
                    "tooltip": "Reset robot and environment",
                    "on_clicked_fn": self._on_reset,
                }
                self._buttons["Reset"] = btn_builder(**dict)
                self._buttons["Reset"].enabled = False
        with frame:
            with ui.VStack(spacing=5):
                # Update the Frame Title
                frame.title = "Task Controls"
                frame.visible = True
                dict = {
                    "label": "Start",
                    "type": "button",
                    "text": "Start",
                    "tooltip": "Start",
                    "on_clicked_fn": self._on_start_button_event,
                }
                self.task_ui_elements["Start"] = btn_builder(**dict)
                self.task_ui_elements["Start"].enabled = False
        with self.get_frame(index=1):
            self.get_frame(index=1).title = "Diagnostics"
            self.get_frame(index=1).visible = True
            self._diagnostics = ui.VStack(spacing=5)
            # self._diagnostics.enabled = False
            with self._diagnostics:
                ui.Label("Decision Stack", height=20)
                self.state_model = ui.SimpleStringModel()
                ui.StringField(self.state_model, multiline=True, height=120)
                self.diagnostics_panel = ui.VStack(spacing=5)
                with self.diagnostics_panel:
                    ui.Label("Diagnostic message", height=20)
                    self.diagostic_model = ui.SimpleStringModel()
                    ui.StringField(self.diagostic_model, multiline=True, height=200)
