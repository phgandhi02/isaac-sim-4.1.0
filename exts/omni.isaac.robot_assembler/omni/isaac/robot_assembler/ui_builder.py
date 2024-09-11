# Copyright (c) 2022-2024, NVIDIA CORPORATION. All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto. Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.
#

import asyncio
from typing import List

import carb
import numpy as np
import omni.kit.commands
import omni.timeline
import omni.ui as ui
import omni.usd
from omni.isaac.core.articulations import Articulation
from omni.isaac.core.prims import XFormPrim
from omni.isaac.core.utils.numpy.rotations import quats_to_rot_matrices, rot_matrices_to_quats
from omni.isaac.core.utils.prims import get_prim_at_path, get_prim_object_type
from omni.isaac.core.utils.stage import update_stage_async
from omni.isaac.core.utils.types import ArticulationAction
from omni.isaac.ui.element_wrappers import (
    Button,
    CheckBox,
    CollapsableFrame,
    DropDown,
    FloatField,
    Frame,
    StateButton,
    StringField,
    TextBlock,
)
from omni.isaac.ui.ui_utils import get_style, setup_ui_headers
from pxr import Usd, UsdGeom, UsdPhysics

from .global_variables import EXTENSION_TITLE
from .robot_assembler import RobotAssembler


class UIBuilder:

    AUTO_CREATE = "AUTO_CREATE_FRAME"

    def __init__(self):
        # Get access to the timeline to control stop/pause/play programmatically
        self._timeline = omni.timeline.get_timeline_interface()

        self.wrapped_ui_elements = []

        self._robot_assembler = RobotAssembler()

    ###################################################################################
    #           The Functions Below Are Called Automatically By extension.py
    ###################################################################################

    def on_menu_callback(self):
        """Callback for when the UI is opened from the toolbar.
        This is distinct from the creation of the UI in build_ui()
        because it can happen more than once if the user repeatedly
        closes and reopens the window.

        This callback happens after build_ui() when the extension is first opened
        """
        # Handles the edge case where the user loads their Articulation and
        # presses play before opening this extension
        if self._timeline.is_playing():
            self._repopulate_all_dropdowns()
            self.assembly_frame.enabled = True
        pass

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

    def on_stage_event(self, event: omni.usd.StageEventType):
        """Callback for Stage Events

        Args:
            event (omni.usd.StageEventType): Event Type
        """
        if event.type == int(omni.usd.StageEventType.ASSETS_LOADED):
            self._repopulate_all_dropdowns()
        elif event.type == int(omni.usd.StageEventType.SIMULATION_START_PLAY):  # Timeline played
            self.assembly_frame.enabled = True
            self._wait_and_reselect_articulations()
        elif event.type == int(omni.usd.StageEventType.SIMULATION_STOP_PLAY):  # Timeline played
            if self._timeline.is_stopped():
                self.assembly_frame.enabled = False
                self.assembly_frame.collapsed = True
                self._reselect_articulations()
                self._articulation_options = []

    def cleanup(self):
        """
        Called when the stage is closed or the extension is hot reloaded.
        Perform any necessary cleanup such as removing active callback functions
        Buttons imported from omni.isaac.ui.element_wrappers implement a cleanup function that should be called
        """
        for ui_elem in self.wrapped_ui_elements:
            ui_elem.cleanup()

    ######################################################################################################
    #                                           Build UI
    ######################################################################################################

    def build_ui(self):
        """
        Build a custom UI tool to run your extension.
        This function will be called once when your extension is opened.
        Closing and reopening the extension from the toolbar will maintain the state of the UI.
        If the user hot reloads this extension, this function will be called again.
        """
        names = ["Base Robot", "Attach Robot"]
        self._robot_frames = []
        self._robot_control_frames = []
        self._robot_dropdowns = []
        self._articulation_attach_point_dropdowns = []
        self._articulations = [None] * len(names)
        self._articulation_options = []
        self._articulation_options_pre_nest = []
        self._collapsable_robot_control_frames = [None] * len(names)
        self._show_art_cbs = []
        self._show_rigid_body_cbs = []

        self._converted_rigid_bodies = []

        self.wrapped_ui_elements = []

        self._joint_control_frames = []
        self._joint_position_float_fields = []

        self._articulations_nested = False
        self._assembled_robot = None

        self._make_info_frame()

        for idx in range(len(names)):
            robot_frame = CollapsableFrame(names[idx], collapsed=False)
            self._robot_frames.append(robot_frame)
            with robot_frame:
                with ui.VStack(style=get_style(), spacing=5, height=0):
                    show_art_cb = CheckBox(
                        "Show Articulations",
                        default_value=True,
                        tooltip="Show Articulations in the drop-down menu",
                        on_click_fn=lambda val, idx=idx: self._robot_dropdowns[idx].repopulate(),
                    )
                    self._show_art_cbs.append(show_art_cb)

                    show_rigid_body_cb = CheckBox(
                        "Show Rigid Bodies",
                        default_value=False,
                        tooltip="Show Rigid Bodies in the drop-down menu",
                        on_click_fn=lambda val, idx=idx: self._robot_dropdowns[idx].repopulate(),
                    )
                    self._show_rigid_body_cbs.append(show_rigid_body_cb)

                    selection_menu = DropDown(
                        "Select Prim",
                        tooltip="Select from Prims found on the stage after the timeline has been played.",
                        on_selection_fn=lambda selection, ind=idx: self._on_prim_selection(ind, selection),
                        keep_old_selections=True,
                        populate_fn=lambda ind=idx: self._dropdown_populate_fn(ind),
                    )
                    self._robot_dropdowns.append(selection_menu)
                    self.wrapped_ui_elements.append(selection_menu)

                    robot_control_frame = Frame(build_fn=lambda idx=idx: self._build_set_robot_position_frame(idx))
                    robot_control_frame.rebuild()
                    self._robot_control_frames.append(robot_control_frame)

        self._make_assemble_frame(names)
        self._make_assembly_summary_frame()

        self._make_convert_to_rigid_body_frame()

    def _make_info_frame(self):
        title = EXTENSION_TITLE
        doc_link = "https://docs.omniverse.nvidia.com/isaacsim/latest/advanced_tutorials/tutorial_advanced_assembling_robots.html"

        overview = "This Extension allows the user to assemble together one or more Articulations and Rigid Bodies."
        overview += "  This may include mounting a robot to a base, or mounting a gripper to an arm."
        overview += "  The current scope of this tool allows the user to design the scene or assembled body that they want, which they can save as a new USD file."
        overview += "  After assembly, this tool also provides a code snippet that allows the user to perform an identical assembly using Python code without the UI."
        overview += "\n\nTo use this extension, the user should select a base and attachable body.  These must be either Rigid Bodies or Articulations."
        overview += "  These are only selectable while the timeline is playing.  Once selected, the user may move on to the assembly frame."
        overview += "\n\nThe assembly frame allows the user to select attach points within the selected bodies, and then to click BEGIN ASSEMBLE."
        overview += "  In the following assembly screen, the user may modify the transform between the bodies being assembled by clicking SELECT ATTACH POINT"
        overview += " and modifying the attachment on the USD stage as desired.  Once assembled, the two resulting bodies will have a Fixed Joint constraint connecting them."
        overview += "\n\nFinally, the Convert Prim To Rigid Body frame may be used to ensure that a non-Articulation prim can be used as a mount."
        overview += "  This conversion applies rigid body physics to a USD object so that it can be included in an assembled body via a fixed joint."

        ext_manager = omni.kit.app.get_app().get_extension_manager()
        ext_id = ext_manager.get_enabled_extension_id("omni.isaac.robot_assembler")

        setup_ui_headers(ext_id, __file__, title, doc_link, overview)

    def _make_convert_to_rigid_body_frame(self):
        self._rigid_body_conversion_frame = CollapsableFrame("Convert Prim To Rigid Body", visible=True, collapsed=True)

        with self._rigid_body_conversion_frame:
            with ui.VStack(style=get_style(), spacing=5, height=0):
                self._rb_conversion_stringfield = StringField(
                    "Prim Path To Convert",
                    tooltip="Input a prim that should be converted to a rigid body.",
                )
                self._rb_conversion_button = Button(
                    "Convert To Rigid Body",
                    "CONVERT",
                    tooltip="Apply the Rigid Body API if possible to a prim, then create a fixed joint tying the rigid body to the stage.",
                    on_click_fn=self._on_rb_conversion,
                )

                def on_show_explanation():
                    self._rigid_body_explanation_frame.visible = True

                def on_hide_explanation():
                    self._rigid_body_explanation_frame.visible = False

                self._rb_frame_explanation_btn = StateButton(
                    "Explain This Frame",
                    "SHOW EXPLANATION",
                    "HIDE EXPLANATION",
                    "Show/Hide an explanation of this frame",
                    on_a_click_fn=on_show_explanation,
                    on_b_click_fn=on_hide_explanation,
                )

                self._rigid_body_explanation_frame = Frame(visible=False)
                with self._rigid_body_explanation_frame:
                    self._rigid_body_explanation_text = (
                        "This frame allows you to convert a USD prim to a Rigid Body with which a robot can be assembled.  "
                        + "A Rigid Body is a prim that has had the Rigid Body API applied to it.  USD prims cannot be assembled together via fixed joints"
                        + " unless they both have physics applied to them.\n\nIn addition, this button ensures that"
                        + " the physics:kinematicEnabled property is True.\n\nThese two conditions are necessary so that there can exist a Fixed Joint between"
                        + " the assembled bodies with no added constraints on the motion of the bodies beyond the fixed joint."
                        + "\n\nIt is not necessary or recommended to convert an Articulation to a rigid body using this frame."
                    )

                    self._rb_frame_explanation_block = TextBlock(
                        "Explanation", text=self._rigid_body_explanation_text, num_lines=10
                    )

    def _make_assembly_summary_frame(self):
        self.assembly_summary_frame = CollapsableFrame("Assembly Summary Frame", visible=False)

        with self.assembly_summary_frame:
            with ui.VStack(style=get_style(), spacing=5, height=0):
                self.undo_assemble_btn = Button(
                    "Undo Assembly",
                    "UNDO ASSEMBLE",
                    tooltip="Undo the last robot assembly",
                    on_click_fn=self._undo_last_assemble,
                )

                self.assembly_code_summary = TextBlock(
                    "Python Code",
                    "",
                    tooltip="A Code Block that replicates the result produced by the user in this UI tool",
                    num_lines=20,
                )

    def _make_assemble_frame(self, names):
        self.assembly_frame = CollapsableFrame("Assembly Frame", collapsed=True, enabled=False)
        with self.assembly_frame:
            with ui.VStack(style=get_style(), spacing=5, height=0):

                # Select Robot Attach Points
                for idx in range(len(names)):
                    self._articulation_attach_point_dropdowns.append(
                        DropDown(
                            f"{names[idx]} Attach Point",
                            tooltip="Select attach point for Articulation",
                            keep_old_selections=False,
                            populate_fn=lambda ind=idx: self._attach_point_populate_fn(ind),
                        )
                    )

                def on_begin_assemble_btn_clicked():
                    if (
                        self._robot_dropdowns[0].get_selection() in self._articulation_options
                        and self._robot_dropdowns[0].get_selection() in self._articulation_options
                    ):
                        self.single_robot_cb.visible = True
                    else:
                        self.single_robot_cb.visible = False

                    self._nest_prims()

                    self._assembler_frame.collapsed = False
                    self._assembler_frame.visible = True
                    self._rigid_body_conversion_frame.visible = False
                    self._rigid_body_conversion_frame.collapsed = True

                    self._begin_assemble_btn.enabled = False

                    for robot_frame in self._robot_frames:
                        robot_frame.visible = False

                    self.assembly_frame.visible = False

                self._begin_assemble_btn = Button(
                    "Begin Assembling Robots",
                    "BEGIN ASSEMBLE",
                    "Press this to begin assembling the selected robots",
                    on_begin_assemble_btn_clicked,
                )

        self._assembler_frame = CollapsableFrame("Robot Assembler", collapsed=True, visible=False)
        with self._assembler_frame:
            with ui.VStack(style=get_style(), spacing=5, height=0):

                def on_stop_assemble_btn_clicked():
                    async def async_cancel():
                        await self._undo_nest_prims()
                        await self._put_attached_art_back()

                    asyncio.ensure_future(async_cancel())

                    self._begin_assemble_btn.enabled = True
                    self._assembler_frame.visible = False
                    self._rigid_body_conversion_frame.visible = True

                    for robot_frame in self._robot_frames:
                        robot_frame.visible = True

                    self.assembly_frame.visible = True

                def on_assemble_btn_clicked(make_single_robot):
                    trans, rot = self._get_relative_attach_transform()

                    async def async_assemble():
                        await self._undo_nest_prims()
                        if (
                            self._art_1_path in self._articulation_options_pre_nest
                            and self._art_2_path in self._articulation_options_pre_nest
                        ):
                            self._assemble_robots(trans, rot, make_single_robot)
                        else:
                            self._assemble_rigid_bodies(trans, rot)

                    asyncio.ensure_future(async_assemble())

                    self._assembler_frame.collapsed = True
                    self._assembler_frame.visible = False
                    self._rigid_body_conversion_frame.visible = True
                    self._begin_assemble_btn.enabled = True

                    for robot_frame in self._robot_frames:
                        robot_frame.visible = True

                    self.assembly_frame.visible = True

                stop_assemble_btn = Button(
                    "Cancel Assemble",
                    "CANCEL ASSEMBLE",
                    "Press this to cancel the assembly.",
                    on_stop_assemble_btn_clicked,
                )
                select_attach_point_btn = Button(
                    "Select Attach Point Prim",
                    "SELECT ATTACH POINT",
                    "Select the attach point frame of the attach robot in order to specify the relative pose.",
                    self._select_attach_point_prim,
                )
                self.single_robot_cb = CheckBox(
                    "Assemble Into Single Robot",
                    default_value=False,
                    tooltip="When checked, the attached robots will be treated as a single Articulation that can be accessed at the prim path of the base robot."
                    + "  When not checked, the attached robots will still be treated as separate Articulations that are controlled independently.",
                )

                assemble_button = Button(
                    "Assemble Robots",
                    "ASSEMBLE",
                    "Assemble the selected robots",
                    lambda: on_assemble_btn_clicked(self.single_robot_cb.get_value()),
                )

    def _build_set_robot_position_frame(self, idx):
        if self._collapsable_robot_control_frames[idx] is None:
            collapsed = True
        else:
            collapsed = self._collapsable_robot_control_frames[idx].collapsed

        if self._articulations[idx] is None:
            Frame()
            self._collapsable_robot_control_frames[idx] = None
            return
        articulation = self._articulations[idx]
        num_dof = articulation.num_dof
        dof_names = articulation.dof_names
        joint_positions = articulation.get_joint_positions()

        lower_joint_limits = articulation.dof_properties["lower"]
        upper_joint_limits = articulation.dof_properties["upper"]

        robot_control_frame = CollapsableFrame("Set Robot Position", collapsed=collapsed)
        self._collapsable_robot_control_frames[idx] = robot_control_frame

        with robot_control_frame:
            # Stack the frames vertically so that they don't cover each other
            with ui.VStack(style=get_style(), spacing=6, height=0):

                for i in range(num_dof):
                    field = FloatField(
                        label=f"{dof_names[i]}",
                        tooltip="Set joint position target",
                        default_value=joint_positions[i],
                        lower_limit=lower_joint_limits[i],
                        upper_limit=upper_joint_limits[i],
                    )
                    field.set_on_value_changed_fn(
                        lambda value, index=i, robot_index=idx: self._on_set_joint_position_target(
                            robot_index, index, value
                        )
                    )

    ##########################################################################################
    #                            Convert To Rigid Body Frame Functions
    ##########################################################################################

    def _on_rb_conversion(self):
        prim_path = self._rb_conversion_stringfield.get_value()
        succ = self._robot_assembler.convert_prim_to_rigid_body(prim_path)
        if succ:
            self._converted_rigid_bodies.append(prim_path)

    ##########################################################################################
    #                               Assembler Summary Frame Functions
    ##########################################################################################

    def setup_assembler_summary_frame_robots(
        self, art_1_path, art_2_path, sel_1, sel_2, rel_trans, rel_orient, single_robot
    ):
        self.undo_assemble_btn.enabled = True
        self.assembly_summary_frame.visible = True
        self.assembly_summary_frame.collapsed = False

        def list_print(l):
            s = "np.array(["
            for i in range(len(l) - 1):
                s += str(round(l[i], 4)) + ","
            s += str(round(l[-1], 4)) + "])"
            return s

        self.assembly_code_summary.set_text(
            "from omni.isaac.robot_assembler import RobotAssembler,AssembledRobot \n"
            + "from omni.isaac.core.articulations import Articulation\n"
            + "import numpy as np\n\n"
            + f'base_robot_path = "{art_1_path}"\n'
            + f'attach_robot_path = "{art_2_path}"\n'
            + f'base_robot_mount_frame = "{sel_1}"\n'
            + f'attach_robot_mount_frame = "{sel_2}"\n'
            + f"fixed_joint_offset = {list_print(rel_trans)}\n"
            + f"fixed_joint_orient = {list_print(rel_orient)}\n"
            + f"single_robot = {single_robot}\n\n"
            + "robot_assembler = RobotAssembler()\n"
            + "assembled_robot = robot_assembler.assemble_articulations(\n\tbase_robot_path,\n\tattach_robot_path,\n\tbase_robot_mount_frame,\n\tattach_robot_mount_frame,\n\tfixed_joint_offset,\n\tfixed_joint_orient,\n\tmask_all_collisions = True,\n\tsingle_robot=single_robot\n)\n\n"
            + "# The fixed joint in a assembled robot is editable after the fact:\n"
            + "# offset,orient = assembled_robot.get_fixed_joint_transform()\n"
            + "# assembled_robot.set_fixed_joint_transform(np.array([.3,0,0]),np.array([1,0,0,0]))\n\n"
            + "# And the assembled robot can be disassembled, after which point the AssembledRobot object will no longer function.\n"
            + "# assembled_robot.disassemble()\n\n"
            + "# Controlling the resulting assembled robot is different depending on the single_robot flag\n"
            + "if single_robot:\n"
            + "\t# The robots will be considered to be part of a single Articulation at the base robot path\n"
            + "\tcontrollable_single_robot = Articulation(base_robot_path)\n"
            + "else:\n"
            + "\t# The robots are controlled independently from each other\n"
            + "\tbase_robot = Articulation(base_robot_path)\n"
            + "\tattach_robot = Articulation(attach_robot_path)\n"
        )

    def setup_assembler_summary_frame_rigid_bodies(self, art_1_path, art_2_path, sel_1, sel_2, rel_trans, rel_orient):
        self.undo_assemble_btn.enabled = True
        self.assembly_summary_frame.visible = True
        self.assembly_summary_frame.collapsed = False

        def list_print(l):
            s = "np.array(["
            for i in range(len(l) - 1):
                s += str(round(l[i], 4)) + ","
            s += str(round(l[-1], 4)) + "])"
            return s

        text = (
            "from omni.isaac.robot_assembler import RobotAssembler,AssembledBodies \n"
            + "from omni.isaac.core.articulations import Articulation\n"
            + "import numpy as np\n\n"
            + "robot_assembler = RobotAssembler()\n"
            + f'base_robot_path = "{art_1_path}"\n'
        )

        if art_1_path in self._converted_rigid_bodies:
            text += "robot_assembler.convert_prim_to_rigid_body(base_robot_path)\n"

        text += f'attach_robot_path = "{art_2_path}"\n'

        if art_2_path in self._converted_rigid_bodies:
            text += "robot_assembler.convert_prim_to_rigid_body(attach_robot_path)\n"

        text += (
            f'base_robot_mount_frame = "{sel_1}"\n'
            + f'attach_robot_mount_frame = "{sel_2}"\n'
            + f"fixed_joint_offset = {list_print(rel_trans)}\n"
            + f"fixed_joint_orient = {list_print(rel_orient)}\n"
            + "assembled_bodies = robot_assembler.assemble_rigid_bodies(\n\tbase_robot_path,\n\tattach_robot_path,\n\tbase_robot_mount_frame,\n\tattach_robot_mount_frame,\n\tfixed_joint_offset,\n\tfixed_joint_orient,\n\tmask_all_collisions = True\n)\n\n"
            + "# The fixed joint in a assembled body is editable after the fact:\n"
            + "# offset,orient = assembled_bodies.get_fixed_joint_transform()\n"
            + "# assembled_bodies.set_fixed_joint_transform(np.array([.3,0,0]),np.array([1,0,0,0]))\n\n"
            + "# And the assembled body can be disassembled, after which point the AssembledBodies object will no longer function.\n"
            + "# assembled_bodies.disassemble()\n\n"
        )
        self.assembly_code_summary.set_text(text)

    def _undo_last_assemble(self):
        if self._assembled_robot is not None and self._assembled_robot.is_assembled():
            self._assembled_robot.disassemble()
            self.undo_assemble_btn.enabled = False

            self._repopulate_all_dropdowns()

            # Reinitialize robots after messing with their physics parsing.
            self._wait_and_reselect_articulations()

    ##########################################################################################
    #                              Robot Assembler Frame Functions
    ##########################################################################################

    def _get_attach_point(self, ind):
        pt = self._articulation_attach_point_dropdowns[ind].get_selection()
        if pt == self.AUTO_CREATE:
            return ""
        else:
            return pt

    def _nest_prims(self):
        sel_1 = self._get_attach_point(0)
        sel_2 = self._get_attach_point(1)

        self._art_1_path = self._robot_dropdowns[0].get_selection()
        self._art_2_path = self._robot_dropdowns[1].get_selection()

        self._base_attach_frame = sel_1
        self._attached_art_attach_frame = sel_2

        if self._art_1_path is None or self._art_2_path is None:
            carb.log_error("Begin Assemble Button was Clicked before valid articulations were selected")

        self._articulations_nested = True

        self._articulation_options_pre_nest = self._articulation_options

        self._attached_art_default_pose = XFormPrim(self._art_2_path).get_world_pose()

        base_path = self._art_1_path + sel_1
        art_name = self._art_2_path[self._art_2_path.rfind("/") :]

        self._nested_articulation_path_from = self._art_2_path
        self._nested_articulation_path_to = base_path + art_name

        omni.kit.commands.execute("MovePrimCommand", path_from=self._art_2_path, path_to=base_path + art_name)

        nested_art_xform = XFormPrim(self._nested_articulation_path_to)
        nested_art_xform.set_local_pose(np.zeros(3), np.array([1, 0, 0, 0]))

        self._timeline.stop()

    def _select_attach_point_prim(self):
        omni.kit.commands.execute(
            "SelectPrimsCommand",
            old_selected_paths=[],
            new_selected_paths=[self._nested_articulation_path_to],
            expand_in_stage=False,
        )

    def _get_relative_attach_transform(self):
        sel_1 = self._base_attach_frame
        sel_2 = self._attached_art_attach_frame
        if sel_1 == self.AUTO_CREATE:
            sel_1 = ""
        if sel_2 == self.AUTO_CREATE:
            sel_2 = ""

        art_1_path = self._art_1_path

        base_attach_point_xform = XFormPrim(art_1_path + sel_1)

        attach_frame = sel_2
        nested_attach_frame_xform = XFormPrim(self._nested_articulation_path_to + attach_frame)

        nested_art_trans, nested_art_rot = nested_attach_frame_xform.get_world_pose()
        base_trans, base_rot = base_attach_point_xform.get_world_pose()
        base_rot, nested_art_rot = quats_to_rot_matrices(np.array([base_rot, nested_art_rot]))

        local_rot = base_rot.T @ nested_art_rot
        local_trans = (base_rot.T @ (nested_art_trans - base_trans)).reshape((3,))

        local_rot = rot_matrices_to_quats(local_rot)

        return local_trans, local_rot

    async def _undo_nest_prims(self):
        if not self._articulations_nested:
            return

        omni.kit.commands.execute(
            "MovePrimCommand", path_from=self._nested_articulation_path_to, path_to=self._nested_articulation_path_from
        )

        self._timeline.play()

        await update_stage_async()

        self._articulations_nested = False

    async def _put_attached_art_back(self):
        self._repopulate_all_dropdowns()

        self._robot_dropdowns[0].set_selection(self._art_1_path)

        self._robot_dropdowns[1].set_selection(self._art_2_path)
        self._on_prim_selection(1, self._art_2_path)

        attach_art = self._articulations[1]
        XFormPrim(attach_art.prim_path).set_world_pose(*self._attached_art_default_pose)

    def _assemble_rigid_bodies(self, rel_trans, rel_orient):
        sel_1 = self._base_attach_frame
        sel_2 = self._attached_art_attach_frame

        art_1_path = self._art_1_path
        art_2_path = self._art_2_path

        self._assembled_robot = self._robot_assembler.assemble_rigid_bodies(
            art_1_path,
            art_2_path,
            sel_1,
            sel_2,
            fixed_joint_offset=rel_trans,
            fixed_joint_orient=rel_orient,
            mask_all_collisions=True,
        )
        self.setup_assembler_summary_frame_rigid_bodies(art_1_path, art_2_path, sel_1, sel_2, rel_trans, rel_orient)

    def _assemble_robots(self, rel_trans, rel_orient, single_robot):
        sel_1 = self._base_attach_frame
        sel_2 = self._attached_art_attach_frame

        art_1_path = self._art_1_path
        art_2_path = self._art_2_path

        self._assembled_robot = self._robot_assembler.assemble_articulations(
            art_1_path,
            art_2_path,
            sel_1,
            sel_2,
            fixed_joint_offset=rel_trans,
            fixed_joint_orient=rel_orient,
            mask_all_collisions=True,
            single_robot=single_robot,
        )
        self.setup_assembler_summary_frame_robots(
            art_1_path, art_2_path, sel_1, sel_2, rel_trans, rel_orient, single_robot
        )

    ############################################################################################
    #                               Assembly Frame Functions
    ############################################################################################

    def _attach_point_populate_fn(self, art_ind: int) -> List[str]:
        selection = self._robot_dropdowns[art_ind].get_selection()
        if selection is None:
            return [self.AUTO_CREATE]
        attach_points = self._get_attach_points(selection)
        if art_ind == 0:
            attach_points.reverse()

        # Offer to auto-create a frame only if the object is NOT an Articulation
        if self._robot_dropdowns[art_ind].get_selection() not in self._articulation_options:
            attach_points.append(self.AUTO_CREATE)
        return attach_points

    def _get_attach_points(self, selection):
        stage = omni.usd.get_context().get_stage()
        paths = []
        if stage and selection is not None:
            for prim in Usd.PrimRange(stage.GetPrimAtPath(selection)):
                path = str(prim.GetPath())
                obj_type = get_prim_object_type(path)
                sub_path = path[len(selection) :]

                if (
                    sub_path.count("/") == 1
                    and (obj_type == "rigid_body" or obj_type == "xform" or obj_type == "articulation")
                    and not (prim.IsA(UsdGeom.Mesh))
                    and not (self._robot_assembler.is_root_joint(prim))
                ):
                    paths.append(sub_path)
        return paths

    ##########################################################################################
    #                            Robot Selection Frame Functions
    ##########################################################################################

    def _wait_and_reselect_articulations(self):
        # Certain physics things will occasionally take two frames to start working.
        async def wait_and_reselect():
            await update_stage_async()
            await update_stage_async()
            for dropdown in self._robot_dropdowns:
                dropdown.trigger_on_selection_fn_with_current_selection()

        asyncio.ensure_future(wait_and_reselect())

    def _reselect_articulations(self):
        for dropdown in self._robot_dropdowns:
            dropdown.trigger_on_selection_fn_with_current_selection()

    def _on_prim_selection(self, art_ind: int, selection: str):
        if selection is None or self._timeline.is_stopped() or selection not in self._articulation_options:
            self._articulations[art_ind] = None

        else:
            try:
                articulation = Articulation(selection)
                articulation.initialize()

                self._articulations[art_ind] = articulation
            except:
                self._articulations[art_ind] = None

        self._robot_control_frames[art_ind].rebuild()

        self._repopulate_all_dropdowns()

        self._articulation_attach_point_dropdowns[art_ind].repopulate()

    def _on_set_joint_position_target(self, robot_index: int, joint_index: int, position_target: float):
        articulation = self._articulations[robot_index]
        robot_action = ArticulationAction(
            joint_positions=np.array([position_target]),
            joint_velocities=np.array([0]),
            joint_indices=np.array([joint_index]),
        )
        articulation.apply_action(robot_action)

    def _repopulate_all_dropdowns(self):
        for d in self._robot_dropdowns:
            d.repopulate()

        # Repopulating articulation menus will recursively repopulate articulation_attach_point dropdowns

    def _dropdown_populate_fn(self, ind: int) -> List[str]:
        # Pick an articulation from the stage that has not been selected already
        selections = [d.get_selection() for d in self._robot_dropdowns[:ind]]
        options = []

        articulations = self._find_all_articulations()
        if self._show_art_cbs[ind].get_value():
            options.extend(articulations)
        if self._show_rigid_body_cbs[ind].get_value():
            rigid_bodies = self._find_all_rigid_bodies(articulations)
            options.extend(rigid_bodies)

        for selection in selections:
            if selection in options:
                options.remove(selection)

        if len(options) == 0:
            self.assembly_frame.enabled = False
        else:
            self.assembly_frame.enabled = True
        return options

    def _find_all_articulations(self):
        art_root_paths = []
        articulation_candidates = set()

        stage = omni.usd.get_context().get_stage()

        if not stage:
            return art_root_paths

        # Find all articulation root paths
        # Find all paths that are the maximal subpath of all prims connected by a fixed joint
        # I.e. a fixed joint connecting /ur10/link1 to /ur10/link0 would result in the path
        # /ur10.  The path /ur10 becomes a candidate Articulation.
        for prim in Usd.PrimRange(stage.GetPrimAtPath("/")):
            if (
                prim.HasAPI(UsdPhysics.ArticulationRootAPI)
                and prim.GetProperty("physxArticulation:articulationEnabled").IsValid()
                and prim.GetProperty("physxArticulation:articulationEnabled").Get()
            ):
                art_root_paths.append(tuple(str(prim.GetPath()).split("/")[1:]))
            elif UsdPhysics.Joint(prim):
                bodies = prim.GetProperty("physics:body0").GetTargets()
                bodies.extend(prim.GetProperty("physics:body1").GetTargets())
                if len(bodies) == 1:
                    continue
                base_path_split = str(bodies[0]).split("/")[1:]
                for body in bodies[1:]:
                    body_path_split = str(body).split("/")[1:]
                    for i in range(len(base_path_split)):
                        if len(body_path_split) < i or base_path_split[i] != body_path_split[i]:
                            base_path_split = base_path_split[:i]
                            break
                articulation_candidates.add(tuple(base_path_split))

        # Only keep candidates whose path is not a subset of another candidate's path
        unique_candidates = []
        for c1 in articulation_candidates:
            is_unique = True
            for c2 in articulation_candidates:
                if c1 == c2:
                    continue
                elif c2[: len(c1)] == c1:
                    is_unique = False
                    break
            if is_unique:
                unique_candidates.append(c1)

        # Only keep candidates that are a subset of exactly one articulation root
        art_base_paths = []
        for c in unique_candidates:
            subset_count = 0
            for root in art_root_paths:
                if root[: len(c)] == c:
                    subset_count += 1
            if subset_count == 1:
                art_path = ""
                for s in c:
                    art_path += "/" + s
                art_base_paths.append(art_path)

        # Keep memory of the Articulations in the DropDown menu to differentiate from Rigid Bodies
        self._articulation_options = art_base_paths

        return art_base_paths

    def _find_all_rigid_bodies(self, exclude_paths):
        items = []
        stage = omni.usd.get_context().get_stage()
        if stage:
            for prim in Usd.PrimRange(stage.GetPrimAtPath("/")):
                path = str(prim.GetPath())
                broke = False
                for exclude_path in exclude_paths:
                    if path[: len(exclude_path)] == exclude_path:
                        broke = True
                        break
                if broke:
                    continue

                # Get prim type get_prim_object_type
                type = get_prim_object_type(path)
                if type == "rigid_body":
                    items.append(path)
        return items
