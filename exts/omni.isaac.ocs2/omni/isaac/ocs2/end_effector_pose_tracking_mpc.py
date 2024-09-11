# Copyright (c) 2022-2023, NVIDIA CORPORATION. All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto. Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.
#

"""Wrapper around python-bindings for mobile-manipulator MPC-SLQ."""

# python
import os
from typing import List, Union

# omniverse
import carb
import numpy as np
import omni.kit

# ocs2
from ocs2.mobile_manipulator import TargetTrajectories, mpc_interface, scalar_array, vector_array


class EndEffectorPoseTrackingMpc:
    """Thin wrapper around the python-bindings for the SLQ-MPC implementation for mobile manipulators.

    The SLQ-MPC considers a kinematic model for the robotic system. The state of the system is
    the kinematic state and the input to the system is the joint velocities.

    The MPC-SLQ formulates the optimal control problem's cost function as the end-effector trajectory
    to follow. Consequently the target state trajectory is the end-effector trajectory to follow.
    In between the discretized trajectory, the MPC performs linear interpolation for the position
    and SLERP interpolation for the orientation.

    The end-effector pose is expected as: (x, y, z, q_w, q_x, q_y, q_z) where (x, y, z) are cartesian
    position co-ordinates and (q_w, q_x, q_y, q_z) is the quaternion orientation.

    Reference:
        1. https://leggedrobotics.github.io/ocs2/robotic_examples.html#mobile-manipulator
    """

    def __init__(self, config_file: str, lib_folder: str, urdf_file: str):
        """Initializes the class variables.

        Note:
            If input paths are not absolute, they are resolved with respect to the path of the
            directory `omni.isaac.ocs2`.

        Args:
            config_file (str): The path to the configuration file.
            lib_folder (str): The path to the directory to generate CppAD code into.
            urdf_file (str): The path to the URDF of the robot.

        Raises:
            ValueError -- When input configuration file path is not valid.
            ValueError -- When input URDF file path is not valid.
        """
        # get extension directory
        ext_manager = omni.kit.app.get_app().get_extension_manager()
        ext_id = ext_manager.get_enabled_extension_id("omni.isaac.ocs2")
        self._ocs2_extension_path = ext_manager.get_extension_path(ext_id)

        # resolve the paths
        # config file
        if not os.path.isabs(config_file):
            config_file = os.path.abspath(os.path.join(self._ocs2_extension_path, config_file))
        # lib folder
        if not os.path.isabs(lib_folder):
            lib_folder = os.path.abspath(os.path.join(self._ocs2_extension_path, lib_folder))
        # urdf file
        if not os.path.isabs(urdf_file):
            urdf_file = os.path.abspath(os.path.join(self._ocs2_extension_path, urdf_file))

        # check that files exist
        if not os.path.exists(config_file):
            msg = f"[OCS2]: Configuration file not found: {config_file}"
            raise ValueError(msg)
        if not os.path.exists(urdf_file):
            msg = f"[OCS2]: Robot URDF file not found: {urdf_file}"
            raise ValueError(msg)

        # store inputs
        self._urdf_file = urdf_file
        self._config_file = config_file
        self._lib_folder = lib_folder
        # create MPC instance
        self._mpc = mpc_interface(config_file, lib_folder, urdf_file)
        # dimensions to the MPC
        self._stateDim = self._mpc.getStateDim()
        self._inputDim = self._mpc.getInputDim()
        # buffers to store optimal trajectories
        self._target_trajectory: TargetTrajectories = None
        # instantiate c++ std arrays to store results
        self._t_result = scalar_array()
        self._x_result = vector_array()
        self._u_result = vector_array()

    def __str__(self) -> str:
        """Returns: A string object for the class."""
        msg = "<class EndEffectorPoseTrackingMpc> object\n"
        msg += f"\tLoaded URDF file    : {self._urdf_file}\n"
        msg += f"\tLoaded config file  : {self._config_file}\n"
        msg += f"\tGenerated lib folder: {self._config_file}\n"
        msg += f"\tMPC State dimension : {self._stateDim}\n"
        msg += f"\tMPC Input dimension : {self._inputDim}"
        return msg

    """
    Properties
    """

    @property
    def state_dim(self) -> int:
        """
        Returns:
            int -- The dimension of MPC problem state.
        """
        return self._stateDim

    @property
    def input_dim(self) -> int:
        """
        Returns:
            int -- The dimension of MPC problem input/action.
        """
        return self._inputDim

    @property
    def mpc(self) -> mpc_interface:
        """
        Returns:
            mpc_interface -- The underlying MPC instance.
        """
        return self._mpc

    @property
    def target_trajectory(self) -> TargetTrajectories:
        """
        Returns:
            TargetTrajectories -- The OCS2 instance for target state-input trajectory.
        """
        return self._target_trajectory

    def get_optimal_time_traj(self) -> np.ndarray:
        """
        Returns:
            np.ndarray -- The timestamps corresponding to the optimal trajectory.
        """
        return np.array(self._t_result).squeeze()

    def get_optimal_state_traj(self) -> np.ndarray:
        """
        Returns:
            np.ndarray -- The robot states corresponding to the optimal trajectory
        """
        return np.array(self._x_result).squeeze()

    def get_optimal_input_traj(self) -> np.ndarray:
        """
        Returns:
            np.ndarray -- The robot inputs corresponding to the optimal trajectory
        """
        return np.array(self._u_result).squeeze()

    def get_current_cost(self) -> float:
        """
        Returns:
            float -- The current solution's cost.
        """
        if len(self._t_result) == 0:
            return float("inf")
        else:
            return self._mpc.cost(self._t_result[0], self._x_result[0], self._u_result[0])

    """
    Operations
    """

    def reset(self, curr_time: float, curr_state: np.ndarray):
        """Reset the MPC interface.

        Args:
            target_trajectory (TargetTrajectories) -- The desired trajectory for the end-effector of the robot.
        """
        # set observations
        self._set_observation(curr_time, curr_state)
        # set target trajectories
        # TODO: Demystify what does the mpc reset() function does.
        if self._target_trajectory is not None:
            self._mpc.reset(self._target_trajectory)
        else:
            carb.log_warn("Resetting MPC failed. No target trajectory present.")

    def set_target_trajectory(
        self,
        time_traj: List[float],
        state_traj: List[Union[np.ndarray, None]],
        input_traj: List[Union[np.ndarray, None]],
    ):
        """Sets the desired trajectory (t, x, u) for the robot to track.

        For kinematic end-effector tracking the `state_traj` corresponds to end-effector pose trajectory.
        The end-effector pose is expected as: (x, y, z, q_w, q_x, q_y, q_z) where (x, y, z) are cartesian
        position co-ordinates and (q_w, q_x, q_y, q_z) is the quaternion orientation.

        Note:
            If the lists for `state_traj` or `input_traj` contain :obj:`None`, the occurrences
            are replaced with zeros.

        Args:
            time_traj (List[float]) -- List of timestamps for the desired trajectory.
            state_traj (List[Union[np.ndarray, None]]) -- List of reference-states in the desired trajectory.
            input_traj (List[Union[np.ndarray, None]]) -- List of inputs in the desired trajectory.

        Raises:
            ValueError: If the input lists have unequal lengths.
        """
        # check that sizes are correct
        if not len(time_traj) == len(state_traj) == len(input_traj):
            msg = (
                f"The desired trajectory is invalid! Expected arguments list to be equal but found: "
                f"(time, state, input) = ({len(time_traj)}, {len(state_traj)}, {len(input_traj)})"
            )
            raise ValueError(msg)
        # define goal trajectories variables
        # Note: It is assumed that only a single reference state and input goal is provided.
        # time
        desired_time_traj = scalar_array()
        desired_time_traj.resize(len(time_traj))
        # input
        desired_input_traj = vector_array()
        desired_input_traj.resize(len(time_traj))
        # state
        desired_state_traj = vector_array()
        desired_state_traj.resize(len(time_traj))

        # fill up the trajectories
        for index in range(len(time_traj)):
            # time
            desired_time_traj[index] = float(time_traj[index])
            # state
            if state_traj[index] is not None:
                # convert quaternion from "wxyz" to "xyzw"
                state_traj[index][3:] = state_traj[index][[4, 5, 6, 3]]
                desired_state_traj[index] = np.array(state_traj[index], dtype=np.float64).reshape((7, 1))
            else:
                desired_state_traj[index] = np.zeros((7, 1))
            # input
            if input_traj[index] is not None:
                desired_input_traj[index] = np.array(input_traj[index], dtype=np.float64).reshape((self._inputDim, 1))
            else:
                desired_input_traj[index] = np.zeros((self._inputDim, 1), dtype=np.float64)
        # set trajectory
        self._target_trajectory = TargetTrajectories(desired_time_traj, desired_state_traj, desired_input_traj)

    def advance(self, curr_time: float, curr_state: np.ndarray) -> np.ndarray:
        """Run the MPC-SLQ algorithm to compute the actions in receding horizon control fashion.

        Args:
            curr_time (float): The current timestamp.
            curr_state (np.ndarray): The current state of the robot.

        Returns:
            np.ndarray -- The first command in the receding horizon control plan.

        Raises:
            RuntimeError: If no target trajectory is set.
        """
        if self._target_trajectory is None:
            msg = "No target trajectory provided. Please set trajectory using `set_target_trajectory(...)` method."
            raise RuntimeError(msg)
        # set target trajectories
        self._mpc.setTargetTrajectories(self._target_trajectory)
        # set the current observation
        self._set_observation(curr_time, curr_state)
        # advance MPC to compute new solution
        self._mpc.advanceMpc()
        # get MPC solution
        self._mpc.getMpcSolution(self._t_result, self._x_result, self._u_result)
        # convert output action to base and arm commands
        u_result_step = np.array(self._u_result).squeeze()[0]

        return u_result_step

    """
    Helpers
    """

    def _set_observation(self, curr_time: float, curr_state: np.ndarray):
        """Set the current observation to the MPC interface.

        Args:
            curr_time (float): The current timestamp.
            curr_state (np.ndarray): The current state of the robot.
        """
        # setup the current observation
        state = np.copy(curr_state).reshape((self._stateDim, 1))
        state = state.astype(np.float64)
        # set observation
        self._mpc.setObservation(curr_time, state, np.zeros(self._inputDim))
