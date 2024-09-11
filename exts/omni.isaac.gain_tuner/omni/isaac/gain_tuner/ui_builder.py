# This software contains source code provided by NVIDIA Corporation.
# Copyright (c) 2024, NVIDIA CORPORATION.  All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto.  Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.
#

import math
from functools import partial

import carb
import numpy as np
import omni.timeline
import omni.ui as ui
from omni.isaac.core.utils.types import ArticulationAction
from omni.isaac.ui.element_wrappers import (
    Button,
    CheckBox,
    CollapsableFrame,
    DropDown,
    FloatField,
    StateButton,
    TextBlock,
    XYPlot,
)
from omni.isaac.ui.ui_utils import get_style, setup_ui_headers
from omni.usd import StageEventType

from .custom_ui_elements import LogFloatField
from .gains_tuner import GainsTestMode, GainTuner
from .global_variables import EXTENSION_TITLE


class UIBuilder:
    def __init__(self):
        # Frames are sub-windows that can contain multiple UI elements
        self.frames = []
        # UI elements created using a UIElementWrapper instance
        self.wrapped_ui_elements = []

        # Get access to the timeline to control stop/pause/play programmatically
        self._timeline = omni.timeline.get_timeline_interface()

        self._articulation = None
        self._gains_tuner = GainTuner()

        self._test_mode = None
        self._built_advanced_settings_frame = False

        self._reset_ui_next_frame = False
        self._make_plot_on_next_frame = False

    ###################################################################################
    #           The Functions Below Are Called Automatically By extension.py
    ###################################################################################

    def on_menu_callback(self):
        """Callback for when the UI is opened from the toolbar.
        This is called directly after build_ui().
        """
        # Reset internal state when UI window is closed and reopened
        self._invalidate_articulation()

        # Handles the case where the user loads their Articulation and
        # presses play before opening this extension
        if self._timeline.is_playing():
            self._articulation_menu.repopulate()
            self._stop_text.visible = False
        elif self._timeline.is_stopped():
            self._stop_text.visible = True

    def on_timeline_event(self, event):
        """Callback for Timeline events (Play, Pause, Stop)

        Args:
            event (omni.timeline.TimelineEventType): Event Type
        """
        pass

    def on_physics_step(self, step: float):
        """Callback for Physics Step.
        Physics steps only occur when the timeline is playing

        Args:
            step (float): Size of physics step
        """
        pass

    def on_render_step(self, e: carb.events.IEvent):
        """Render event set up to cancel physics subscriptions that run the gains test.

        Args:
            e (carb.events.IEvent): _description_
        """
        if self._reset_ui_next_frame:
            if self._make_plot_on_next_frame:
                if self._test_mode == GainsTestMode.SINUSOIDAL:
                    self._sinusoidal_plotting_frame.visible = True
                    self._sinusoidal_plotting_frame.rebuild()
                elif self._test_mode == GainsTestMode.STEP_FUNCTION:
                    self._step_plotting_frame.visible = True
                    self._step_plotting_frame.rebuild()
                self._test_mode = None

            self._advanced_settings_frame.enabled = True
            self._sinusoidal_gains_test_btn.enabled = True
            self._sinusoidal_gains_test_btn.reset()
            self._step_gains_test_btn.enabled = True
            self._step_gains_test_btn.reset()
            self._stiff_gains_btn.enabled = True
            self._hand_tuning_frame.enabled = True

            self._reset_ui_next_frame = False

    def on_stage_event(self, event):
        """Callback for Stage Events

        Args:
            event (omni.usd.StageEventType): Event Type
        """
        if event.type == int(omni.usd.StageEventType.ASSETS_LOADED):  # Any asset added or removed
            self._articulation_menu.repopulate()
        elif event.type == int(omni.usd.StageEventType.SIMULATION_START_PLAY):  # Timeline played
            self._articulation_menu.trigger_on_selection_fn_with_current_selection()
            self._stop_text.visible = False
        elif event.type == int(omni.usd.StageEventType.SIMULATION_STOP_PLAY):  # Timeline stopped
            # Ignore pause events
            if self._timeline.is_stopped():
                self._invalidate_articulation()
                self._articulation_menu.repopulate()

                self._sinusoidal_gains_test_btn.reset()
                self._gains_tuning_frame.enabled = False
                self._gains_tuning_frame.collapsed = True

                self._advanced_settings_frame.collapsed = True
                self._advanced_settings_frame.enabled = False
                self._stop_text.visible = True

    def cleanup(self):
        """
        Called when the stage is closed or the extension is hot reloaded.
        Perform any necessary cleanup such as removing active callback functions
        Buttons imported from omni.isaac.ui.element_wrappers implement a cleanup function that should be called
        """
        for ui_elem in self.wrapped_ui_elements:
            ui_elem.cleanup()

    def build_ui(self):
        self._build_info_ui()

        self._articulation_selection_frame = CollapsableFrame("Robot Selection", collapsed=False)

        with self._articulation_selection_frame:
            with ui.VStack(style=get_style(), spacing=5, height=0):
                self._articulation_menu = DropDown(
                    "Select Articulation",
                    tooltip="Select from Articulations found on the stage after the timeline has been played.",
                    on_selection_fn=self._on_articulation_selection,
                    keep_old_selections=True,
                )
                self._articulation_menu.set_populate_fn_to_find_all_usd_objects_of_type(
                    "articulation", repopulate=False
                )

                self._stop_text = TextBlock(
                    "README",
                    "Select an Articulation and click the PLAY button on the left to get started.",
                    include_copy_button=False,
                    num_lines=2,
                )

        self._gains_tuning_frame = CollapsableFrame("Tune Gains", collapsed=True, enabled=False)

        with self._gains_tuning_frame:
            with ui.VStack(style=get_style(), spacing=5, height=0):
                self._proportional_gains_fields = []
                self._damping_gains_fields = []
                self._hand_tuning_frame = CollapsableFrame(
                    "Hand Tune Gains", collapsed=True, enabled=True, build_fn=self._build_hand_tuning_frame
                )

                stiff_gains_tooltip = (
                    "Set all proportional gains to be 1e15 and all damping gains "
                    + "to be 1e5.  This will likely result in the best position tracking in the "
                    + "sinusoidal joint test."
                )
                self._stiff_gains_btn = Button(
                    "Set Stiff Gains",
                    "Set Stiff Gains",
                    tooltip=stiff_gains_tooltip,
                    on_click_fn=self._on_set_stiff_gains,
                )

                sinusoidal_gains_tooltip = (
                    "Send sinusoidal joint commands to each joint to asses the Articulation's "
                    + "ability to follow continuous joint commands."
                )
                self._sinusoidal_gains_test_btn = StateButton(
                    "Sinusoidal Gains Test",
                    "Test Gains",
                    "Testing Gains",
                    tooltip=sinusoidal_gains_tooltip,
                    on_a_click_fn=partial(self._on_run_gains_test, GainsTestMode.SINUSOIDAL),
                    physics_callback_fn=self._update_gains_test,
                )
                self.wrapped_ui_elements.append(self._sinusoidal_gains_test_btn)

                step_gains_tooltip = (
                    "Send non-continuous joint commands to each joint to asses the Articulation's "
                    + "ability to follow continuous joint commands."
                )
                self._step_gains_test_btn = StateButton(
                    "Step Function Gains Test",
                    "Test Gains",
                    "Testing Gains",
                    tooltip=step_gains_tooltip,
                    on_a_click_fn=partial(self._on_run_gains_test, GainsTestMode.STEP_FUNCTION),
                    physics_callback_fn=self._update_gains_test,
                )
                self.wrapped_ui_elements.append(self._sinusoidal_gains_test_btn)
                self.wrapped_ui_elements.append(self._step_gains_test_btn)

        self._sinusoidal_plotting_frame = CollapsableFrame("Sinusoidal Joint Plots", visible=False, collapsed=False)
        self._step_plotting_frame = CollapsableFrame("Step Function Joint Plots", visible=False, collapsed=False)

        def build_plotting_frame_fn(gains_test_mode):
            if self._articulation is None:
                return
            if not self._gains_tuner.is_data_ready():
                return
            with ui.VStack(style=get_style(), spacing=5, height=0):
                pos_rmse, vel_rmse = self._gains_tuner.compute_gains_test_error_terms()

                show_velocity_plots = gains_test_mode == GainsTestMode.SINUSOIDAL
                num_lines = self._articulation.num_dof * 2

                if gains_test_mode == GainsTestMode.SINUSOIDAL:
                    plot_results_text = self._get_human_readable_plot_error_text(self._articulation, pos_rmse, vel_rmse)
                    TextBlock("Results", plot_results_text, num_lines=num_lines)
                elif gains_test_mode == GainsTestMode.STEP_FUNCTION:
                    plot_readme = self._get_step_function_plot_readme(self._articulation)
                    TextBlock("README", plot_readme, num_lines=6)

                for joint_index in range(self._articulation.num_dof):
                    frame = CollapsableFrame(self._articulation.dof_names[joint_index], collapsed=False)
                    frame.set_build_fn(partial(self._build_joint_plot_frame, joint_index, show_velocity_plots))

        self._sinusoidal_plotting_frame.set_build_fn(partial(build_plotting_frame_fn, GainsTestMode.SINUSOIDAL))
        self._step_plotting_frame.set_build_fn(partial(build_plotting_frame_fn, GainsTestMode.STEP_FUNCTION))

        self._advanced_settings_frame = CollapsableFrame("Gains Test Settings", collapsed=True, enabled=False)

        def build_advanced_settings_frame_fn():
            # This build function can be called on a collapsed frame, and then all the joint frame
            # build functions will not be called.  This bool is set to true when a joint frame is
            # actually built.
            self._built_advanced_settings_frame = False

            self._joint_settings_frames = []

            if self._articulation is None:
                return

            self._reset_advanced_settings_fields()

            def on_set_test_duration(value):
                self._gains_tuner.set_test_duration(value)

            def on_joint_range_clipped(value):
                self._gains_tuner.set_joint_range_clipping_fraction(value)
                self._refresh_velocity()
                self._refresh_period()

            def on_set_joint_range_maximum(value):
                self._gains_tuner.set_joint_range_maximum(value)
                self._refresh_velocity()
                self._refresh_period()

            def on_set_joint_velocity(index, value):
                period = self._gains_tuner.get_period(value, index)
                self._joint_period_fields[index].set_value(period)

            def on_set_joint_period(index, value):
                velocity = self._gains_tuner.get_v_max(value, index)
                self._joint_max_velocity_fields[index].set_value(velocity)

            def build_joint_frame(joint_index):
                self._built_advanced_settings_frame = True
                if self._joint_cbs[joint_index] is not None:
                    joint_active = self._joint_cbs[joint_index].get_value()
                else:
                    joint_active = True

                with ui.VStack(style=get_style(), spacing=5, height=0):
                    cb = CheckBox(
                        "Include Joint in Gains Test",
                        default_value=joint_active,
                        tooltip="If unchecked, joint will be left in a fixed position.",
                        on_click_fn=lambda val: self._joint_settings_frames[joint_index].rebuild(),
                    )
                    self._joint_cbs[joint_index] = cb

                    if joint_active:
                        field = FloatField(
                            label=f"Max Velocity",
                            tooltip="Maximum joint velocity to be reached in the sinusoidal gains test.",
                            on_value_changed_fn=partial(on_set_joint_velocity, joint_index),
                        )
                        self._joint_max_velocity_fields[joint_index] = field
                        field = FloatField(
                            label="Period",
                            tooltip="Period of the sinusoid commanded in the sinusoidal gains test.",
                            on_value_changed_fn=partial(on_set_joint_period, joint_index),
                        )
                        self._joint_period_fields[joint_index] = field

                        self._joint_fixed_position_fields[joint_index] = None
                    else:
                        field = FloatField(
                            label=f"Fixed Joint Position", tooltip="Fixed position to be held throughout gains test."
                        )
                        self._joint_fixed_position_fields[joint_index] = field
                        self._joint_max_velocity_fields[joint_index] = None

                self._setup_advanced_settings_frames(single_index=joint_index)

            with ui.VStack(style=get_style(), spacing=5, height=0):
                TextBlock(
                    "README",
                    text=(
                        "Set parameters controling the exact behavior of joints in each type of "
                        + "gains test, including the range of motion and the maximum velocity "
                        + "reached.  Behavior in the sinusoidal gains test is over-specified by "
                        + "the parameters below, and so adjusting one parameter may cause others to "
                        + "be automatically changed.\n\nMax Velocity and Period are exclusive to the "
                        + "sinusoidal gains test. Period is bounded below by Max Velocity and Max "
                        + "Velocity is bounded above by the Articulation max_velocity and max_effort "
                        + "properties.  I.e. the gains test will not exceed a joint's maximum velocity "
                        + "or acceleration in the commanded sinusoid."
                    ),
                    num_lines=4,
                    include_copy_button=False,
                )

                ui.Spacer(height=15)

                self._test_duration_field = FloatField(
                    label="Test Duration (s)",
                    tooltip="Specify how long the gains test should be in seconds.",
                    default_value=self._gains_tuner.get_test_duration(),
                    lower_limit=0.05,
                    on_value_changed_fn=on_set_test_duration,
                )

                joint_range_maximum_tooltip = (
                    "Limit the range of motion of all joints to a maximum value to improve interpretability and plottability. "
                    + "If clipped from from this field, joints will move by +- the maximum range around their central position."
                )
                self._joint_range_maximum_field = FloatField(
                    label="Joint Range Maximum",
                    tooltip=joint_range_maximum_tooltip,
                    default_value=self._gains_tuner.get_joint_range_maximum(),
                    lower_limit=0.01,
                    on_value_changed_fn=on_set_joint_range_maximum,
                )

                joint_range_clipping_tooltip = (
                    "Fraction of the total range of motion used for gains tests. "
                    + "The range of motion is limited by `Maximum Joint Range` after applying this value."
                    "The default value of 0.9 allows the user to see overshoot if present without "
                    + "the articulation hitting hard joint limits."
                )
                self._joint_range_clipping_field = FloatField(
                    label="Joint Range Used (fraction)",
                    tooltip=joint_range_clipping_tooltip,
                    default_value=self._gains_tuner.get_joint_range_clipping_fraction(),
                    lower_limit=0.01,
                    upper_limit=1.0,
                    on_value_changed_fn=on_joint_range_clipped,
                )

                self._position_impulse_float_field = FloatField(
                    label="Initial Position Impulse",
                    tooltip="Initial error term between the commanded trajectory position and the initial robot position.",
                    default_value=self._gains_tuner.get_position_impulse(),
                    on_value_changed_fn=self._gains_tuner.set_position_impulse,
                )

                self._velocity_impulse_float_field = FloatField(
                    label="Initial Velocity Impulse",
                    tooltip="Initial error term between the commanded trajectory velocity and the initial robot velocity.",
                    default_value=self._gains_tuner.get_velocity_impulse(),
                    on_value_changed_fn=self._gains_tuner.set_velocity_impulse,
                )

                for i in range(self._articulation.num_dof):
                    joint_frame = CollapsableFrame(f"Joint {i}", collapsed=False)
                    self._joint_settings_frames.append(joint_frame)

                    joint_frame.set_build_fn(partial(build_joint_frame, i))

            self._setup_advanced_settings_frames()

        self._advanced_settings_frame.set_build_fn(build_advanced_settings_frame_fn)

    def _build_info_ui(self):
        title = EXTENSION_TITLE
        doc_link = "https://docs.omniverse.nvidia.com/isaacsim/"

        overview = "This utility is used to help tune the gains of an Articulation.  "
        overview += "Select the Articulation you would like to tune from the dropdown menu."
        overview += "\n\nGain tuning can be considered successful if near-perfect position tracking "
        overview += "is observed in the sinusoidal gains test at the maximum velocities intended for "
        overview += "your use case, and if reasonable behavior is observed in the step function "
        overview += "gains test. Try running both tests to understand more."

        ext_manager = omni.kit.app.get_app().get_extension_manager()
        ext_id = ext_manager.get_enabled_extension_id("omni.isaac.gain_tuner")

        setup_ui_headers(ext_id, __file__, title, doc_link, overview)

    def _refresh_period(self):
        # Refresh the period value on all joints
        num_dof = self._articulation.num_dof

        v_field_maxes = self._gains_tuner.get_v_max_from_robot_properties()

        for i in range(num_dof):
            v_float_field = self._joint_max_velocity_fields[i]

            if v_float_field is not None:
                v_max = v_float_field.get_value()

                period_float_field = self._joint_period_fields[i]
                period_float_field.set_lower_limit(self._gains_tuner.get_period(v_field_maxes[i], i))

                period_float_field.set_value(self._gains_tuner.get_period(v_max, i))

    def _refresh_velocity(self):
        # Refresh the velocity value on all joints
        num_dof = self._articulation.num_dof

        v_field_maxes = self._gains_tuner.get_v_max_from_robot_properties()

        for i in range(num_dof):
            v_float_field = self._joint_max_velocity_fields[i]
            period_float_field = self._joint_period_fields[i]

            if v_float_field is not None:
                # First update period field maximum in case it changed.
                period_float_field.set_lower_limit(self._gains_tuner.get_period(v_field_maxes[i], i))
                period = period_float_field.get_value()

                v_float_field = self._joint_max_velocity_fields[i]
                v_float_field.set_value(self._gains_tuner.get_v_max(period, i))

    def _reset_advanced_settings_fields(self):
        self._joint_max_velocity_fields = [None] * self._articulation.num_dof
        self._joint_period_fields = [None] * self._articulation.num_dof
        self._joint_fixed_position_fields = [None] * self._articulation.num_dof
        self._joint_cbs = [None] * self._articulation.num_dof

    def _setup_advanced_settings_frames(self, single_index: int = None):
        """Set up advanced settings frame with reasonable default values.

        Args:
            single_index (int, optional): If single_index is an int, only change the fields associated with that index. Defaults to None.
        """
        num_dof = self._articulation.num_dof
        dof_names = self._articulation.dof_names
        joint_positions = self._articulation.get_joint_positions()

        lower_joint_limits = self._articulation.dof_properties["lower"]
        upper_joint_limits = self._articulation.dof_properties["upper"]

        v_max, T = self._gains_tuner.get_default_tuning_test_parameters()

        v_field_maxes = self._gains_tuner.get_v_max_from_robot_properties()

        for i in range(num_dof):
            if single_index is not None:
                i = single_index

            frame = self._joint_settings_frames[i]
            if self._joint_max_velocity_fields[i] is not None:
                v_float_field = self._joint_max_velocity_fields[i]
                v_float_field.set_lower_limit(0.01)
                v_float_field.set_upper_limit(v_field_maxes[i])
                v_float_field.set_value(v_max[i])

                period_float_field = self._joint_period_fields[i]
                period_float_field.set_lower_limit(self._gains_tuner.get_period(v_field_maxes[i], i))
                period_float_field.set_value(T[i])

            elif self._joint_fixed_position_fields[i] is not None:
                position_float_field = self._joint_fixed_position_fields[i]
                position = joint_positions[i]

                position_float_field.set_value(position)
                position_float_field.set_upper_limit(upper_joint_limits[i])
                position_float_field.set_lower_limit(lower_joint_limits[i])

                def on_set_position(position, joint_index):
                    self._articulation.apply_action(ArticulationAction([position], joint_indices=[joint_index]))

                position_float_field.set_on_value_changed_fn(lambda val, index=i: on_set_position(val, index))

            # Write the human-readable names of each joint
            frame.title = dof_names[i]
            position = joint_positions[i]

            if single_index is not None:
                return

    def _refresh_gains_fields(self):
        if self._articulation is None:
            return
        if len(self._proportional_gains_fields) != self._articulation.num_dof:
            self._hand_tuning_frame.rebuild()
            return
        p, d = self._articulation.get_articulation_controller().get_gains()
        for i, p_field in enumerate(self._proportional_gains_fields):
            p_field.set_value(math.log10(p[i]) if p[i] != 0 else -25)
        for i, d_field in enumerate(self._damping_gains_fields):
            d_field.set_value(math.log10(d[i]) if d[i] != 0 else -25)

    def _build_hand_tuning_frame(self):
        self._proportional_gains_fields = []
        self._damping_gains_fields = []

        if self._articulation is None:
            return

        def on_scale_all_p_gains():
            p, _ = self._articulation.get_articulation_controller().get_gains()
            p *= 10 ** self._scale_p_gains_field.get_value()
            self._articulation.get_articulation_controller().set_gains(p, save_to_usd=True)
            self._refresh_gains_fields()

        def on_scale_all_d_gains():
            p, d = self._articulation.get_articulation_controller().get_gains()
            d *= 10 ** self._scale_d_gains_field.get_value()
            self._articulation.get_articulation_controller().set_gains(p, d, save_to_usd=True)
            self._refresh_gains_fields()

        def on_p_changed(index, value):
            p, _ = self._articulation.get_articulation_controller().get_gains()
            p[index] = 10**value
            self._articulation.get_articulation_controller().set_gains(p, save_to_usd=True)

        def on_d_changed(index, value):
            p, d = self._articulation.get_articulation_controller().get_gains()
            d[index] = 10**value
            self._articulation.get_articulation_controller().set_gains(p, d, save_to_usd=True)

        p, d = self._articulation.get_articulation_controller().get_gains()
        with ui.VStack(style=get_style(), spacing=5, height=0):
            TextBlock(
                "README: ",
                (
                    "All float values under `Hand Tune Gains` are in a log10 scale.\n"
                    + "E.g. a value of 3 corresponds to the number 1000."
                ),
                num_lines=2,
                include_copy_button=False,
            )

            ui.Spacer(height=15)

            self._scale_p_gains_field = LogFloatField(
                "Stiffness Multiplier",
                tooltip="Scalar for all stiffness gains (log10 scale)",
                lower_limit=-50,
                upper_limit=50,
                default_value=1.0,
            )
            self._scale_all_p_gains_btn = Button(
                "Scale Stiffness Gains",
                "Scale",
                tooltip="Apply scalar to all stiffness gains",
                on_click_fn=on_scale_all_p_gains,
            )

            self._scale_d_gains_field = LogFloatField(
                "Damping Multiplier",
                tooltip="Scalar for all damping gains (log10 scale)",
                lower_limit=-50,
                upper_limit=50,
                default_value=1.0,
            )
            self._scale_all_d_gains_btn = Button(
                "Scale Damping Gains",
                "Scale",
                tooltip="Apply scalar to all stiffness gains",
                on_click_fn=on_scale_all_d_gains,
            )

            for i, joint_name in enumerate(self._articulation.dof_names):
                CollapsableFrame(f"{joint_name}", collapsed=False)
                p_float_field = LogFloatField(
                    f"{joint_name} Stiffness log(kp) ",
                    tooltip=f"Log base 10 of the value of {joint_name} proportional gain.",
                    default_value=math.log10(p[i]) if p[i] != 0 else -25,
                    lower_limit=-25,
                    upper_limit=25,
                    on_value_changed_fn=partial(on_p_changed, i),
                )
                d_float_field = LogFloatField(
                    f"{joint_name} Damping log(kd) ",
                    tooltip=f"Log base 10 of the value of {joint_name} damping gain.",
                    default_value=math.log10(d[i]) if d[i] != 0 else -25,
                    lower_limit=-25,
                    upper_limit=25,
                    on_value_changed_fn=partial(on_d_changed, i),
                )

                self._proportional_gains_fields.append(p_float_field)
                self._damping_gains_fields.append(d_float_field)

    def _on_set_stiff_gains(self):
        if self._articulation is None:
            return
        self._gains_tuner.set_stiff_gains()
        self._refresh_gains_fields()

    def _get_human_readable_plot_error_text(self, articulation, pos_rmse, vel_rmse=None):
        text = f"Root Mean Squared Error (RMSE) for each Articulation Joint ({articulation.prim_path}):\n\n"
        for i, joint_name in enumerate(self._articulation.dof_names):
            text += f"{joint_name} Position RMSE: {np.round(pos_rmse[i], decimals=5)}\n"
        if vel_rmse is not None:
            text += "\n"
            for i, joint_name in enumerate(self._articulation.dof_names):
                text += f"{joint_name} Velocity RMSE: {np.round(vel_rmse[i], decimals=5)}\n"
        return text

    def _get_step_function_plot_readme(self, articulation):
        return (
            f"Step function joint position commands for Articulation ({articulation.prim_path}).\n\n"
            "Unlike the sinusoidal gains test, perfect position tracking is not expected in this test. "
            + "This test serves to validate whether an asset behaves reasonably when sent discontinuous joint commands. "
            + "The duration of time that each joint is commanded to be at its high position is determined by approximating "
            + "the time expected for that joint to get there if saturating either maximum velocity or effort. "
            + "The results can be interpretted as reasonable if the position plots are smooth and the maximum "
            + "positions are reached at roughly the same time as predicted.\n\n"
            + "The user should note that it is always better to send an Articulation a continuous path to fully control "
            + "desired behavior. The purpose of this test is to give an intuitive sense for how the asset "
            + "handles discontinuities that may arise through world collisions or programmatic mistakes. "
        )

    def _build_joint_plot_frame(self, joint_index, show_velocity_plots):
        (pos_cmd, vel_cmd, obs_pos, obs_vel, cmd_times) = self._gains_tuner.get_joint_states_from_gains_test(
            joint_index
        )
        if pos_cmd is None:
            return

        with ui.VStack(style=get_style(), spacing=5, height=0):
            XYPlot(
                "Joint Position",
                tooltip="Hold click over plot to display x,y values",
                x_label="Time (s)",
                y_label="Joint Position",
                show_legend=True,
                x_data=[cmd_times, cmd_times],
                y_data=[pos_cmd, obs_pos],
                legends=["Commanded Joint Position", "Observed Joint Position"],
            )

            if show_velocity_plots:
                XYPlot(
                    "Joint Velocity",
                    tooltip="Hold click over plot to display x,y values",
                    x_label="Time (s)",
                    y_label="Joint Velocity",
                    show_legend=True,
                    x_data=[cmd_times, cmd_times],
                    y_data=[vel_cmd, obs_vel],
                    legends=["Commanded Joint Velocity", "Observed Joint Velocity"],
                )

    def _invalidate_articulation(self):
        """
        This function handles the event that the existing articulation becomes invalid and there is
        not a new articulation to select.  It is called explicitly in the code when the timeline is
        stopped and when the DropDown menu finds no articulations on the stage.
        """
        self._articulation = None
        self._advanced_settings_frame.rebuild()
        self._test_mode = None

    def _on_articulation_selection(self, articulation_path):
        if articulation_path is None or self._timeline.is_stopped():
            self._invalidate_articulation()
            return

        self._gains_tuner = GainTuner()
        self._articulation = self._gains_tuner.setup(articulation_path)

        # UI management
        self._gains_tuning_frame.enabled = True
        self._gains_tuning_frame.collapsed = False

        self._hand_tuning_frame.rebuild()

        self._sinusoidal_gains_test_btn.reset()
        self._sinusoidal_gains_test_btn.enabled = True

        self._reset_advanced_settings_fields()
        self._advanced_settings_frame.rebuild()
        self._advanced_settings_frame.enabled = True

        self._test_mode = None

        self._reset_ui_next_frame = True

    def _update_gains_test(self, step: float):
        if self._test_mode is None:
            return
        done = self._gains_tuner.update_gains_test(step)
        if done:
            self._reset_ui_next_frame = True
            if self._gains_tuner.is_data_ready():
                self._make_plot_on_next_frame = True

    def _on_run_gains_test(self, gains_test_mode):
        """Disable all buttons until the gains test has finished."""
        self._sinusoidal_gains_test_btn.enabled = False
        self._step_gains_test_btn.enabled = False
        self._advanced_settings_frame.enabled = False
        self._hand_tuning_frame.collapsed = True
        self._hand_tuning_frame.enabled = False
        self._stiff_gains_btn.enabled = False

        v_max = []
        T = []
        joint_indices = []
        fixed_joint_positions = []

        default_v_max, default_T = self._gains_tuner.get_default_tuning_test_parameters()

        if self._built_advanced_settings_frame:
            for i in range(self._articulation.num_dof):
                if self._joint_max_velocity_fields[i] is not None:
                    v_float_field = self._joint_max_velocity_fields[i]
                    v_max.append(v_float_field.get_value())

                    period_float_field = self._joint_period_fields[i]
                    T.append(period_float_field.get_value())

                    joint_indices.append(i)
                elif self._joint_fixed_position_fields[i] is None:
                    v_max.append(default_v_max[i])
                    T.append(default_T[i])
                    joint_indices.append(i)
                else:
                    fixed_joint_positions.append(self._joint_fixed_position_fields[i].get_value())
        else:
            v_max, T = default_v_max, default_T
            joint_indices = np.arange(self._articulation.num_dof)

        self._gains_tuner.initialize_gains_test(gains_test_mode, v_max, T, joint_indices, fixed_joint_positions)
        self._test_mode = gains_test_mode
