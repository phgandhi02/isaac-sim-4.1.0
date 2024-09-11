# Copyright (c) 2020-2023, NVIDIA CORPORATION. All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto. Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.
#
import numpy as np
import omni.graph.core as og
from omni.isaac.core.articulations.articulation import Articulation
from omni.isaac.core.utils.types import ArticulationAction
from omni.isaac.core_nodes import BaseResetNode
from omni.isaac.manipulators.grippers.parallel_gripper import ParallelGripper
from omni.isaac.manipulators.ogn.OgnIsaacGripperControllerDatabase import OgnIsaacGripperControllerDatabase


class OgnIsaacGripperControllerInternalState(BaseResetNode):
    def __init__(self):
        self.STOPPED = 0
        self.INCREASING = 1
        self.DECREASING = 2
        self.STOPPING = 3

        self.robot_prim = None
        self.gripper_prim = None
        self.gripper_handle = None
        self.joint_names = None
        self.num_fingers = None
        self.open_position = None
        self.close_position = None
        self.next_position = None
        self.distance_per_frame = None  # default is to close the gripper instantaenously
        self.initialized = False
        self.gripper_state = self.STOPPED  # - 0-stopped, 1-opening, 2-closing, 3-stopping
        self.sign_flip = -1
        self.printed = False
        super().__init__(initialize=False)

    def initialize_controller(self):
        print("initializing robot")
        try:
            self.robot_handle = Articulation(self.robot_prim)
            self.robot_handle.initialize()
            self.joint_indices = [self.robot_handle.get_dof_index(name) for i, name in enumerate(self.joint_names)]

        except:
            print("WARNING: unable to initialize robot. check if joint names and robotPrim target are correct")

        # if both open and close limits are zeros, user did not put in a valid limit, default to joint limits
        if np.all(self.open_position == 0) and np.all(self.close_position == 0):
            self.open_position = np.array([])
            self.close_position = np.array([])
            print(f"\n WARNING: open and close positions defaulting to joint limit, open/close might be flipped.")
            for i in self.joint_indices:
                if self.robot_handle.dof_properties[i][1]:  # hasLimits is true
                    self.open_position = np.append(self.open_position, self.robot_handle.dof_properties[i][3])
                    self.close_position = np.append(self.close_position, self.robot_handle.dof_properties[i][2])
                else:
                    # no position limits found on joints
                    print(
                        f"no valid position limits found for {self.joint_names[i]}. Please either enter the open/close positions in gripper node, or set position limits to the relevant joints"
                    )

        print(f"\n Open position set to: {self.open_position}\n")
        print(f"\n Close position set to: {self.close_position}\n")

        print("initializing gripper")
        self.gripper_handle = ParallelGripper(
            self.gripper_prim,
            self.joint_names,
            self.open_position,
            self.close_position,
            self.distance_per_frame,
        )
        self.gripper_handle.initialize(
            physics_sim_view=None,
            articulation_apply_action_func=self.robot_handle.apply_action,
            get_joint_positions_func=self.robot_handle.get_joint_positions,
            set_joint_positions_func=self.robot_handle.set_joint_positions,
            dof_names=self.robot_handle.dof_names,
        )

        if (self.open_position - self.close_position)[0] > 0:
            print("NOTE: open/close joint limit flipped")
            self.sign_flip = -1
            self.upper_bound = self.open_position
            self.lower_bound = self.close_position
        else:
            self.sign_flip = 1
            self.upper_bound = self.close_position
            self.lower_bound = self.open_position
        self.initialized = True
        self.next_position = self.gripper_handle.get_joint_positions()
        self.apply_action()

    def gripper_commands(self):
        if self.gripper_state == self.INCREASING:
            self.next_position += self.distance_per_frame
            self.next_position = np.minimum(
                self.next_position, self.upper_bound
            )  # if stepped pass the limit, stop at limit
            if (self.next_position == self.upper_bound).all():
                self.gripper_state = self.STOPPED

        elif self.gripper_state == self.DECREASING:
            self.next_position -= self.distance_per_frame
            self.next_position = np.maximum(
                self.next_position, self.lower_bound
            )  # if stepped pass the limit, stop at limit
            if (self.next_position == self.lower_bound).all():
                self.gripper_state = self.STOPPED

        elif self.gripper_state == self.STOPPING:
            self.next_position = self.gripper_handle.get_joint_positions()
            self.fingers_stopped = self.num_fingers
            self.gripper_state = self.STOPPED

        # print("next position", self.next_position)

    def apply_action(self):
        if self.initialized:
            joint_actions = ArticulationAction()
            joint_actions.joint_indices = self.joint_indices
            joint_actions.joint_positions = self.next_position

            self.gripper_handle.apply_action(control_actions=joint_actions)

    def print_once(self, msg):
        if not self.printed:
            print(msg)
            self.printed = True
        else:
            pass

    def custom_reset(self):
        self.robot_prim = None
        self.gripper_prim = None
        self.gripper_handle = None
        self.joint_names = None
        self.open_position = None
        self.close_position = None
        self.next_position = None
        self.distance_per_frame = None  # default is to close the gripper instantaenously
        self.initialized = False
        self.robot_handle = None
        self.print_once = False
        pass


class OgnIsaacGripperController:
    """
    Node that prepare the inputs to ArticulationController to only control the gripper. It does not directly call actions. Assumes the gripper joints are position controlled.

    also assumes positive direction on the joint is opening, and negative position is closing
    """

    @staticmethod
    def internal_state():
        return OgnIsaacGripperControllerInternalState()

    @staticmethod
    def compute(db) -> bool:
        state = db.per_instance_state

        try:
            if not state.initialized:

                # find the robot articulation
                if len(db.inputs.articulationRootPrim) == 0:
                    db.log_error("No robot prim found for the articulation controller")
                    return False
                else:
                    state.robot_prim = db.inputs.articulationRootPrim[0].GetString()

                if len(db.inputs.gripperPrim) == 0:
                    db.log_error("No gripper prim found")
                    return False
                else:
                    state.gripper_prim = db.inputs.gripperPrim[0].GetString()

                ## setup the gripper parameters
                state.joint_names = db.inputs.jointNames
                n_fingers = len(db.inputs.jointNames)
                state.num_fingers = n_fingers
                # set the joint limits if open/close positions are set
                if db.inputs.openPosition is None:
                    state.open_position = np.zeros(n_fingers)
                else:
                    if db.inputs.openPosition.size == n_fingers:
                        state.open_position = db.inputs.openPosition
                    elif db.inputs.openPosition.size == 1:
                        state.open_position = np.repeat(db.inputs.openPosition, n_fingers)
                    else:
                        print("invalid open position input.")
                        state.open_position = np.zeros(n_fingers)

                if db.inputs.closePosition is None:
                    state.close_position = np.zeros(n_fingers)
                else:
                    if db.inputs.closePosition.size == n_fingers:
                        state.close_position = db.inputs.closePosition
                    elif db.inputs.closePosition.size == 1:
                        state.close_position = np.repeat(db.inputs.closePosition, n_fingers)
                    else:
                        print("invalid close position input.")
                        state.close_position = np.zeros(n_fingers)

                if db.inputs.gripperSpeed is None:
                    db.log_error("Must have Speed input")
                    return False
                elif (np.asarray(db.inputs.gripperSpeed) <= 0).any():  # if any of the entry is = or less than 0
                    db.log_error("Speed entries must be greater than 0")
                    return False
                else:
                    if db.inputs.gripperSpeed.size == n_fingers:
                        state.distance_per_frame = db.inputs.gripperSpeed
                    elif db.inputs.gripperSpeed.size == 1:
                        state.distance_per_frame = np.repeat(db.inputs.gripperSpeed, n_fingers)
                    else:
                        print("invalid speed array.")
                        return False

                state.initialize_controller()

            # set the state of the gripper
            if db.inputs.open:
                print("opening gripper")
                if state.sign_flip == 1:
                    state.gripper_state = state.DECREASING
                else:  # default is set to opening is increasing in joint position
                    state.gripper_state = state.INCREASING
            if db.inputs.close:
                print("closing gripper")
                if state.sign_flip == 1:
                    state.gripper_state = state.INCREASING
                else:
                    state.gripper_state = state.DECREASING
            if db.inputs.stop:
                print("stopping gripper")
                state.gripper_state = state.STOPPING

            # if the gripper is already moving, keep moving until it reaches goal
            if state.gripper_state != state.STOPPED:
                state.gripper_commands()

                ## if positionCommands not connecte to a downstream node, then move the robot, otherwise just send the position command on (assuming eventually there will be an articulation controller downstream to move the robot
                attr = db.node.get_attribute("outputs:positionCommands")
                downstream_nodes = attr.get_downstream_connections()
                ## TODO: if downstream node has "toString" involved, assume it's for debugging, move the robot AND pass on the data
                # get path of current graph
                # path to toString node
                # if string_node is in downstream_nodes: apply_action()
                if downstream_nodes == []:  # or string_node is in downstream_nodes:
                    state.apply_action()
                else:
                    state.print_once("!! Downstream nodes detected, so no commands sent to robot here!!")

            # setting up the outputs
            # pass joint names straight through
            db.outputs.joints = state.joint_names
            db.outputs.positionCommands = state.next_position

        except Exception as error:
            db.log_warn(str(error))
            return False

        return True

    @staticmethod
    def release(node):
        try:
            state = OgnIsaacGripperControllerDatabase.get_internal_state(node)
        except Exception:
            state = None
            pass

        if state is not None:
            state.reset()
