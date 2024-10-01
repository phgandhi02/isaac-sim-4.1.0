# Copyright (c) 2022-2023, NVIDIA CORPORATION. All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto. Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.
#

import numpy as np
import omni.timeline
import omni.ui as ui
from omni.isaac.core.articulations import Articulation
from omni.isaac.core.utils.prims import get_prim_object_type
from omni.isaac.core.utils.types import ArticulationAction
from omni.isaac.ui.element_wrappers import CollapsableFrame, DropDown, FloatField, TextBlock
from omni.isaac.ui.ui_utils import get_style


class UIBuilder:
    def __init__(self):
        # Frames are sub-windows that can contain multiple UI elements
        self.frames = []

        # UI elements created using a UIElementWrapper from omni.isaac.ui.element_wrappers
        self.wrapped_ui_elements = []

        # Get access to the timeline to control stop/pause/play programmatically
        self._timeline = omni.timeline.get_timeline_interface()

        # Run initialization for the provided example
        self._on_init()

    ###################################################################################
    #           The Functions Below Are Called Automatically By extension.py
    ###################################################################################

    def on_menu_callback(self):
        """Callback for when the UI is opened from the toolbar.
        This is called directly after build_ui().
        """
        # Reset internal state when UI window is closed and reopened
        self._invalidate_articulation()

        self._selection_menu.repopulate()

        # Handles the case where the user loads their Articulation and
        # presses play before opening this extension
        if self._timeline.is_playing():
            self._stop_text.visible = False
        elif self._timeline.is_stopped():
            self._stop_text.visible = True

    def on_timeline_event(self, event):
        """Callback for Timeline events (Play, Pause, Stop)

        Args:
            event (omni.timeline.TimelineEventType): Event Type
        """
        pass

    def on_physics_step(self, step):
        """Callback for Physics Step.
        Physics steps only occur when the timeline is playing

        Args:
            step (float): Size of physics step
        """
        pass

    def on_stage_event(self, event):
        """Callback for Stage Events

        Args:
            event (omni.usd.StageEventType): Event Type
        """
        if event.type == int(omni.usd.StageEventType.ASSETS_LOADED):  # Any asset added or removed
            self._selection_menu.repopulate()
        elif event.type == int(omni.usd.StageEventType.SIMULATION_START_PLAY):  # Timeline played
            # Treat a playing timeline as a trigger for selecting an Articulation
            self._selection_menu.trigger_on_selection_fn_with_current_selection()
            self._stop_text.visible = False
        elif event.type == int(omni.usd.StageEventType.SIMULATION_STOP_PLAY):  # Timeline stopped
            # Ignore pause events
            if self._timeline.is_stopped():
                self._invalidate_articulation()
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
        """
        Build a custom UI tool to run your extension.
        This function will be called any time the UI window is closed and reopened.
        """
        selection_panel_frame = CollapsableFrame("Selection Panel", collapsed=False)

        with selection_panel_frame:
            with ui.VStack(style=get_style(), spacing=5, height=0):
                self._selection_menu = DropDown(
                    "Select Articulation",
                    tooltip="Select from Articulations found on the stage after the timeline has been played.",
                    on_selection_fn=self._on_articulation_selection,
                    keep_old_selections=True,
                    # populate_fn = self._find_all_articulations # Equivalent functionality to one-liner below
                )
                # This sets the populate_fn to find all USD objects of a certain type on the stage, overriding the populate_fn arg
                # Figure out the type of an object with get_prim_object_type(prim_path)
                self._selection_menu.set_populate_fn_to_find_all_usd_objects_of_type("articulation", repopulate=False)

                self._stop_text = TextBlock(
                    "README",
                    "Select an Articulation and click the PLAY button on the left to get started.",
                    include_copy_button=False,
                    num_lines=2,
                )

        self._robot_control_frame = CollapsableFrame("Robot Control Frame", collapsed=True, enabled=False)

        def build_robot_control_frame_fn():
            self._joint_control_frames = []
            self._joint_position_float_fields = []

            # Don't build the frame unless there is a valid Articulation.
            if self.articulation is None:
                return

            with ui.VStack(style=get_style(), spacing=5, height=0):
                for i in range(self.articulation.num_dof):
                    joint_frame = CollapsableFrame(f"Joint {i}", collapsed=False)
                    self._joint_control_frames.append(joint_frame)

                    # In each joint control frame, add controls to manage the robot joint
                    with joint_frame:
                        field = FloatField(label=f"Position Target", tooltip="Set joint position target")
                        field.set_on_value_changed_fn(
                            lambda value, index=i: self._on_set_joint_position_target(index, value)
                        )
                        self._joint_position_float_fields.append(field)
            self._setup_joint_control_frames()

        self._robot_control_frame.set_build_fn(build_robot_control_frame_fn)

    ######################################################################################
    # Functions Below This Point Support The Provided Example And Can Be Replaced/Deleted
    ######################################################################################

    def _on_init(self):
        self.articulation = None

    def _invalidate_articulation(self):
        """
        This function handles the event that the existing articulation becomes invalid and there is
        not a new articulation to select.  It is called explicitly in the code when the timeline is
        stopped and when the DropDown menu finds no articulations on the stage.
        """
        self.articulation = None
        self._robot_control_frame.collapsed = True
        self._robot_control_frame.enabled = False

    def _on_articulation_selection(self, selection: str):
        """
        This function is called whenever a new selection is made in the
        "Select Articulation" DropDown.  A new selection may also be
        made implicitly any time self._selection_menu.repopulate() is called
        since the Articulation they had selected may no longer be present on the stage.

        Args:
            selection (str): The item that is currently selected in the drop-down menu.
        """
        # If the timeline is stopped, the Articulation won't be usable.
        if selection is None or self._timeline.is_stopped():
            self._invalidate_articulation()
            return

        self.articulation = Articulation(selection)
        self.articulation.initialize()

        self._robot_control_frame.collapsed = False
        self._robot_control_frame.enabled = True
        self._robot_control_frame.rebuild()

    def _setup_joint_control_frames(self):
        """
        Once a robot has been chosen, update the UI to match robot properties:
            Make a frame visible for each robot joint.
            Rename each frame to match the human-readable name of the joint it controls.
            Change the FloatField for each joint to match the current robot position.
            Apply the robot's joint limits to each FloatField.
        """
        num_dof = self.articulation.num_dof
        dof_names = self.articulation.dof_names
        joint_positions = self.articulation.get_joint_positions()

        lower_joint_limits = self.articulation.dof_properties["lower"]
        upper_joint_limits = self.articulation.dof_properties["upper"]

        for i in range(num_dof):
            frame = self._joint_control_frames[i]
            position_float_field = self._joint_position_float_fields[i]

            # Write the human-readable names of each joint
            frame.title = dof_names[i]
            position = joint_positions[i]

            position_float_field.set_value(position)
            position_float_field.set_upper_limit(upper_joint_limits[i])
            position_float_field.set_lower_limit(lower_joint_limits[i])

    def _on_set_joint_position_target(self, joint_index: int, position_target: float):
        """
        This function is called when the user changes one of the float fields
        to control a robot joint position target.  The index of the joint and the new
        desired value are passed in as arguments.

        This function assumes that there is a guarantee it is called safely.
        I.e. A valid Articulation has been selected and initialized
        and the timeline is playing.  These gurantees are given by careful UI
        programming.  The joint control frames are only visible to the user when
        these guarantees are met.

        Args:
            joint_index (int): Index of robot joint that was modified
            position_target (float): New position target for robot joint
        """
        robot_action = ArticulationAction(
            joint_positions=np.array([position_target]),
            joint_velocities=np.array([0]),
            joint_indices=np.array([joint_index]),
        )
        self.articulation.apply_action(robot_action)

    # def _find_all_articulations(self):
    # #    Commented code left in to help a curious user gain a thorough understanding

    #     import omni.usd
    #     from pxr import Usd
    #     items = []
    #     stage = omni.usd.get_context().get_stage()
    #     if stage:
    #         for prim in Usd.PrimRange(stage.GetPrimAtPath("/")):
    #             path = str(prim.GetPath())
    #             # Get prim type get_prim_object_type
    #             type = get_prim_object_type(path)
    #             if type == "articulation":
    #                 items.append(path)
    #     return items
