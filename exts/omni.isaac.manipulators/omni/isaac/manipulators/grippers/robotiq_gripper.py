# Copyright (c) 2021-2023, NVIDIA CORPORATION. All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto. Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.
#
from typing import Callable, List

import numpy as np
import omni.kit.app
from omni.isaac.core.utils.types import ArticulationAction
from omni.isaac.manipulators.grippers import SurfaceGripper


class RobotiqGripper(SurfaceGripper):
    """Provides high level functions to set/ get properties and actions of a Robotiq gripper
    (a gripper that has two fingers).

    Args:
        end_effector_prim_path (str): prim path of the Prim that corresponds to the gripper root/ end effector.
        joint_prim_names (List[str]): the left finger joint prim name and the right finger joint prim name respectively.
        joint_opened_positions (np.ndarray): joint positions of the left finger joint when opened.
        joint_closed_positions (np.ndarray): joint positions of the left finger joint when closed.
        action_deltas (np.ndarray, optional): deltas to apply for finger joint positions when openning or closing the gripper. Defaults to None.
    """

    def __init__(
        self,
        end_effector_prim_path: str,
        translate: float,
        joint_prim_name: str,
        joint_opened_position: np.ndarray|float,
        joint_closed_position: np.ndarray|float,
        action_delta: np.ndarray = None|float,
    ) -> None:
        SurfaceGripper.__init__(end_effector_prim_path=end_effector_prim_path,
                                translate = .01,
                                direction='x',
                                grip_threshold = .01,
                                force_limit= 1000000,
                                torque_limit= 10000,
                                bend_angle = 0,
                                kp = 100,
                                kd = 100,
                                disable_gravity= True
                                )
        self._joint_prim_name = joint_prim_name
        self._joint_opened_position = joint_opened_position
        self._joint_closed_position = joint_closed_position
        self._get_joint_position_func = None
        self._set_joint_position_func = None
        self._action_delta = action_delta
        self._articulation_num_dof = 1
        return
    
    # ---------------------------------------------------------------------------- #
    #                                  Properties                                  #
    # ---------------------------------------------------------------------------- #
    @property
    def joint_opened_position(self) -> np.ndarray:
        """
        Returns:
            np.ndarray: joint positions of the left finger joint when opened.
        """
        return self._joint_opened_position

    @property
    def joint_closed_positions(self) -> np.ndarray:
        """
        Returns:
            np.ndarray: joint positions of the left finger joint when closed.
        """
        return self._joint_closed_position

    @property
    def joint_prim_names(self) -> List[str]:
        """
        Returns:
            List[str]: the left finger joint prim name and the right finger joint prim name respectively.
        """
        return self._joint_prim_name
    
    # ---------------------------------------------------------------------------- #
    #                                Class Functions                               #
    # ---------------------------------------------------------------------------- #
    def initialize(
        self,
        articulation_apply_action_func: Callable,
        get_joint_positions_func: Callable,
        set_joint_positions_func: Callable,
        dof_names: List,
        physics_sim_view: omni.physics.tensors.SimulationView = None,
    ) -> None:
        """Create a physics simulation view if not passed and creates a rigid prim view using physX tensor api.
            This needs to be called after each hard reset (i.e stop + play on the timeline) before interacting with any
            of the functions of this class.

        Args:
            articulation_apply_action_func (Callable): apply_action function from the Articulation class.
            get_joint_positions_func (Callable): get_joint_positions function from the Articulation class.
            set_joint_positions_func (Callable): set_joint_positions function from the Articulation class.
            dof_names (List): dof names from the Articulation class.
            physics_sim_view (omni.physics.tensors.SimulationView, optional): current physics simulation view. Defaults to None

        Raises:
            Exception: _description_
        """
        SurfaceGripper.initialize(self, physics_sim_view=physics_sim_view)
        self._get_joint_position_func = get_joint_positions_func
        
        self._articulation_apply_action_func = articulation_apply_action_func
        current_joint_positions = get_joint_positions_func()
        if self._default_state is None:
            self._default_state = np.array(
                [
                    current_joint_positions
                ]
            )
        self._set_joint_position_func = set_joint_positions_func
        return

    def open(self) -> None:
        """Applies actions to the articulation that opens the gripper (ex: to release an object held)."""
        self._articulation_apply_action_func(self.forward(action="open"))
        return

    def close(self) -> None:
        """Applies actions to the articulation that closes the gripper (ex: to hold an object)."""
        self._articulation_apply_action_func(self.forward(action="close"))
        return

    def set_action_deltas(self, value: np.ndarray) -> None:
        """
        Args:
            value (np.ndarray): deltas to apply for finger joint positions when openning or closing the gripper.
                               [left, right]. Defaults to None.
        """
        self._action_delta = value
        return

    def get_action_deltas(self) -> np.ndarray:
        """
        Returns:
            np.ndarray: deltas that will be applied for finger joint positions when openning or closing the gripper.
                        [left, right]. Defaults to None.
        """
        return self._action_delta

    def set_default_state(self, joint_positions: np.ndarray) -> None:
        """Sets the default state of the gripper

        Args:
            joint_positions (np.ndarray): joint positions of the left finger joint.
        """
        self._default_state = joint_positions
        return

    def get_default_state(self) -> np.ndarray:
        """Gets the default state of the gripper

        Returns:
            np.ndarray: joint positions of the left finger joint.
        """
        return self._default_state

    def post_reset(self):
        Gripper.post_reset(self)
        self._set_joint_position_func(
            positions=self._default_state, joint_indices=self._joint_dof_index[0]
        )
        return

    def set_joint_positions(self, positions: np.ndarray) -> None:
        """
        Args:
            positions (np.ndarray): joint positions of the left finger joint.
        """
        self._set_joint_position_func(
            positions=positions, joint_indices=self._joint_dof_index[0]
        )
        return

    def get_joint_positions(self) -> np.ndarray:
        """
        Returns:
            np.ndarray: joint positions of the left finger joint.
        """
        return self._get_joint_position_func(joint_indices=self._joint_dof_index[0])

    def forward(self, action: str) -> ArticulationAction:
        """calculates the ArticulationAction for all of the articulation joints that corresponds to "open"
           or "close" actions.

        Args:
            action (str): "open" or "close" as an abstract action.

        Raises:
            Exception: _description_

        Returns:
            ArticulationAction: articulation action to be passed to the articulation itself
                                (includes all joints of the articulation).
        """
        if action == "open":
            target_joint_positions = None
            if self._action_delta is None:
                target_joint_positions = self._joint_opened_position[0]
            else:
                current_joint_position = self._get_joint_position_func()
                target_joint_positions = current_joint_position + self._action_delta
        elif action == "close":
            target_joint_positions = None
            if self._action_delta is None:
                target_joint_positions = self._joint_closed_position
            else:
                current_joint_position = self._get_joint_position_func()
                target_joint_positions= current_joint_position - self._action_delta
        else:
            raise Exception("action {} is not defined for ParallelGripper. Please input open or close as the action".format(action))
        return ArticulationAction(joint_positions=target_joint_positions)

    def apply_action(self, control_actions: ArticulationAction) -> None:
        """Applies actions to all the joints of an articulation that corresponds to the ArticulationAction of the finger joints only.

        Args:
            control_actions (ArticulationAction): ArticulationAction for the gripper joint.
        """
        joint_actions = ArticulationAction()
        if control_actions.joint_positions is not None:
            joint_actions.joint_positions = None
            joint_actions.joint_positions = control_actions.joint_positions
        if control_actions.joint_velocities is not None:
            joint_actions.joint_velocities = None
            joint_actions.joint_velocities = control_actions.joint_velocities
        if control_actions.joint_efforts is not None:
            joint_actions.joint_efforts = None
        self._articulation_apply_action_func(control_actions=joint_actions)
        return
