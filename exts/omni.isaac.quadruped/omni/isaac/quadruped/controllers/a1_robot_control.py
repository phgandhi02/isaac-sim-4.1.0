# Copyright (c) 2022-2023, NVIDIA CORPORATION. All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto. Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.
#

from typing import Tuple

import carb.profiler
import numpy as np
import osqp  # used for solving QP
import scipy.sparse as sp
from omni.isaac.quadruped.utils.a1_ctrl_params import A1CtrlParams
from omni.isaac.quadruped.utils.a1_ctrl_states import A1CtrlStates
from omni.isaac.quadruped.utils.a1_desired_states import A1DesiredStates
from omni.isaac.quadruped.utils.rot_utils import skew


def _eval_quartic_bezier(nodes: np.ndarray, s: float) -> np.ndarray:
    """[summary]

    Evaluate a Bezier curve of degree 4 in N dimensions, where N is determined by the number of columns in `nodes`.

    Args:
        nodes {np.ndarray} -- 2D array containing 5 rows, each corresponding to one of the 5 nodes (points in N
            dimensions) defining the curve.
        s {float} -- s parameter ("time value") at which to evaluate the curve.
    """
    oms = 1.0 - s
    return (
        oms**4 * nodes[0, :]
        + 4 * s * oms**3 * nodes[1, :]
        + 6 * s**2 * oms**2 * nodes[2, :]
        + 4 * s**3 * oms * nodes[3, :]
        + s**4 * nodes[4, :]
    )


class A1RobotControl:
    """[summary]

    The A1 robot controller
    This class uses A1CtrlStates to save data. The control joint torque is generated
    using a QP controller
    """

    def __init__(self) -> None:
        """Initializes the class instance."""
        self.counter = 0
        self.grf = np.zeros((4, 3))
        pass

    """
    Operations
    """

    def update_plan(
        self, desired_states: A1DesiredStates, input_states: A1CtrlStates, input_params: A1CtrlParams, dt: float
    ) -> None:
        """[summary]

        update swing leg trajectory and several counters

        Args:
            desired_states {A1DesiredStates} -- the desired states
            input_states {A1CtrlStates} -- the control states
            input_params {A1CtrlParams}     -- the control parameters
            dt {float} -- The simulation time-step.

        """
        carb.profiler.begin(1, "update plan")
        self._update_gait_plan(input_states)
        self._update_foot_plan(desired_states, input_states, input_params, dt)
        # increase _counter
        input_states._counter += 1
        input_states._exp_time += dt

        input_states._gait_counter += input_states._gait_counter_speed
        input_states._gait_counter %= input_states._counter_per_gait
        carb.profiler.end(1)

    def generate_ctrl(
        self, desired_states: A1DesiredStates, input_states: A1CtrlStates, input_params: A1CtrlParams
    ) -> None:
        """[summary]

        main function, generate foot ground reaction force using QP and calculate joint torques

        Args:
            desired_states {A1DesiredStates} -- the desired states
            input_states {A1CtrlStates} -- the control states
            input_params {A1CtrlParams} -- the control parameters
        """
        carb.profiler.begin(2, "generate control")
        # first second, do nothing, wait sensor and stuff got stablized
        if input_states._exp_time < 0.1:
            return np.zeros(12)
        # initial control
        if input_states._init_transition == 0 and input_states._prev_transition == 0:
            input_params._kp_linear[0:2] = np.array([500, 500])

        # foot control
        foot_pos_final = input_states._foot_pos_target_rel
        foot_pos_cur = np.zeros([4, 3])

        for i, leg in enumerate(["FL", "FR", "RL", "RR"]):
            foot_pos_cur[i, :] = input_states._rot_mat_z.T @ input_states._foot_pos_abs[i, :]

        bezier_time = np.zeros(4)
        for i in range(4):
            if input_states._gait_counter[i] < input_states._counter_per_swing:
                bezier_time[i] = 0.0
                input_states._foot_pos_start_rel[i, :] = foot_pos_cur[i, :]
                input_states._early_contacts[i] = False
            else:
                bezier_time[i] = (
                    input_states._gait_counter[i] - input_states._counter_per_swing
                ) / input_states._counter_per_swing

        foot_pos_target = self._get_from_bezier_curve(input_states._foot_pos_start_rel, foot_pos_final, bezier_time)
        foot_pos_error = foot_pos_target - foot_pos_cur
        foot_forces_kin = (input_params._kp_foot * foot_pos_error).flatten()

        # detect early contacts
        # how to determine which foot is in contact: check gait counter
        for i in range(4):
            if not input_states._contacts[i] and input_states._gait_counter[i] <= input_states._counter_per_swing * 1.5:
                input_states._early_contacts[i] = False
            if (
                not input_states._contacts[i]
                and input_states._early_contacts[i] is False
                and input_states._foot_forces[i] > input_params._foot_force_low
                and input_states._gait_counter[i] > input_states._counter_per_swing * 1.5
            ):
                input_states._early_contacts[i] = True

        for i in range(4):
            input_states._contacts[i] = input_states._contacts[i] or input_states._early_contacts[i]

        # root control
        if self.counter == 0 or self.grf.all == 0:
            self.grf = self._compute_grf(desired_states, input_states, input_params)

        self.counter = (self.counter + 1) % 5

        grf_rel = self.grf @ input_states._rot_mat
        foot_forces_grf = -grf_rel.flatten()

        carb.profiler.begin(43, "convert to torque")
        # convert to torque
        carb.profiler.begin(50, "M matrix")
        M = np.kron(np.eye(4, dtype=int), input_params._km_foot)
        carb.profiler.end(50)
        # torques_init = input_states._j_foot.T @ foot_forces_init
        carb.profiler.begin(51, "torque kin")
        torques_kin = np.linalg.inv(input_states._j_foot) @ M @ foot_forces_kin
        carb.profiler.end(51)
        # torques_kin = input_states._j_foot.T @ foot_forces_kin
        carb.profiler.begin(52, "torque grf")
        torques_grf = input_states._j_foot.T @ foot_forces_grf
        carb.profiler.end(52)
        carb.profiler.end(43)
        carb.profiler.begin(44, "combine torque")
        # combine torques
        torques_init = np.zeros(12)
        for i in range(4):
            torques_init[3 * i : 3 * i + 3] = torques_grf[3 * i : 3 * i + 3]

        # combine torques
        torques = np.zeros(12)
        for i in range(4):
            if input_states._contacts[i]:
                torques[3 * i : 3 * i + 3] = torques_grf[3 * i : 3 * i + 3]
            else:
                torques[3 * i : 3 * i + 3] = torques_kin[3 * i : 3 * i + 3]

        torques = (1 - input_states._init_transition) * torques_init + input_states._init_transition * torques
        torques += input_params._torque_gravity
        carb.profiler.end(44)
        carb.profiler.end(2)
        return torques

    """
    Internal helpers.
    """

    def _update_gait_plan(self, input_states: A1CtrlStates) -> None:
        """[summary]

        update gait counters

        Args:
            input_states {A1CtrlStates} -- the control states
        """
        carb.profiler.begin(3, "Update gait plan")
        # initialize _counter
        if input_states._counter == 0 or input_states._gait_type != input_states._gait_type_last:
            if input_states._gait_type == 2:
                input_states._gait_counter = np.array([0.0, 120.0, 0.0, 120.0])
            elif input_states._gait_type == 1:
                input_states._gait_counter = np.array([0.0, 120.0, 120.0, 0.0])
            else:
                input_states._gait_counter = np.array([0.0, 0.0, 0.0, 0.0])

        # update _counter speed
        for i in range(4):
            if input_states._gait_type == 2:
                input_states._gait_counter_speed[i] = 1.4
            elif input_states._gait_type == 1:
                input_states._gait_counter_speed[i] = 1.4
            else:
                input_states._gait_counter_speed[i] = 0.0

            input_states._contacts[i] = input_states._gait_counter[i] < input_states._counter_per_swing

        input_states._gait_type_last = input_states._gait_type
        carb.profiler.end(3)

    def _update_foot_plan(
        self, desired_states: A1DesiredStates, input_states: A1CtrlStates, input_params: A1CtrlParams, dt: float
    ) -> None:
        """[summary]

        update foot swing target positions

        Args:
            input_states {A1DesiredStates} -- the desried states
            input_states {A1CtrlStates}    -- the control states
            input_params {A1CtrlParams}    -- the control parameters
            dt           {float}           -- delta time since last update
        """
        carb.profiler.begin(4, "update foot plan")
        # heuristic plan
        lin_pos = input_states._root_pos
        # lin_pos_rel = input_states._rot_mat_z.T @ lin_pos

        lin_pos_d = desired_states._root_pos_d
        # lin_pos_rel_d = input_states._rot_mat_z.T @ lin_pos_d

        lin_vel = input_states._root_lin_vel
        # body frame
        lin_vel_rel = input_states._rot_mat_z.T @ lin_vel

        input_states._foot_pos_target_rel = input_params._default_foot_pos.copy()
        for i in range(4):
            weight_y = np.square(np.abs(input_params._default_foot_pos[i, 2]) / 9.8)
            weight2 = input_states._counter_per_swing / input_states._gait_counter_speed[i] * dt / 2.0
            delta_x = weight_y * (lin_vel_rel[0] - desired_states._root_lin_vel_d[0]) + weight2 * lin_vel_rel[0]

            delta_y = weight_y * (lin_vel_rel[1] - desired_states._root_lin_vel_d[1]) + weight2 * lin_vel_rel[1]

            if delta_x < -0.1:
                delta_x = -0.1
            if delta_x > 0.1:
                delta_x = 0.1
            if delta_y < -0.1:
                delta_y = -0.1
            if delta_y > 0.1:
                delta_y = 0.1

            input_states._foot_pos_target_rel[i, 0] += delta_x
            input_states._foot_pos_target_rel[i, 1] += delta_y
        carb.profiler.end(4)

    def _get_from_bezier_curve(
        self, foot_pos_start: np.ndarray, foot_pos_final: np.ndarray, bezier_time: np.ndarray
    ) -> np.ndarray:
        """[summary]

        generate swing foot position target from a quartic bezier curve

        Args:
            foot_pos_start {np.ndarray} -- The curve start point
            foot_pos_final {np.ndarray} -- The curve end point
            bezier_time {np.ndarray} -- The curve interpolation time for each of the four legs; each should be
                within [0,1].
        """
        carb.profiler.begin(5, "generate swing foot pos from bezier curve")
        foot_pos_target = np.empty([4, 3])

        for i in range(4):
            bezier_nodes = np.array(
                [
                    foot_pos_start[i, :],
                    foot_pos_start[i, :],
                    foot_pos_final[i, :],
                    foot_pos_final[i, :],
                    foot_pos_final[i, :],
                ]
            )
            z_foot_clearance1 = 0.0
            z_foot_clearance2 = 0.5
            bezier_nodes[1, 2] += z_foot_clearance1
            bezier_nodes[2, 2] += z_foot_clearance2
            foot_pos_target[i, :] = _eval_quartic_bezier(bezier_nodes, bezier_time[i])
        carb.profiler.end(5)
        return foot_pos_target

    def _compute_grf(
        self,
        desired_states: A1DesiredStates,
        input_states: A1CtrlStates,
        input_params: A1CtrlParams,
        setup_optimizer: bool = True,
    ) -> np.ndarray:
        """[summary]

        main internal function, generate foot ground reaction force using QP

        Args:
            desired_states {A1DesiredStates} -- the desired states
            input_states {A1CtrlStates} -- the control states
            input_params {A1CtrlParams}     -- the control parameters

        Returns:
            grf {np.ndarray}
        """

        carb.profiler.begin(6, "generate foot ground force")

        self.solver = osqp.OSQP()
        inertia_inv, root_acc, acc_weight, u_weight = self._get_qp_params(desired_states, input_states, input_params)

        modified_contacts = np.array([True, True, True, True])
        if input_states._init_transition < 1.0:
            modified_contacts = np.array([True, True, True, True])
        else:
            modified_contacts = input_states._contacts
        carb.profiler.begin(41, "generate friction pyramid")
        mu = 0.2
        # use osqp
        Q = np.diag(np.square(acc_weight))
        R = u_weight
        F_min = 0
        F_max = 250.0
        hessian = np.identity(12) * R + inertia_inv.T @ Q @ inertia_inv
        gradient = -inertia_inv.T @ Q @ root_acc
        linearMatrix = np.zeros([20, 12])
        lowerBound = np.zeros(20)
        upperBound = np.zeros(20)
        for i in range(4):
            # extract F_zi
            linearMatrix[i, 2 + i * 3] = 1.0
            # friction pyramid
            # 1. F_xi < uF_zi
            linearMatrix[4 + i * 4, i * 3] = 1.0
            linearMatrix[4 + i * 4, 2 + i * 3] = -mu
            lowerBound[4 + i * 4] = -np.inf
            # 2. -F_xi > uF_zi
            linearMatrix[4 + i * 4 + 1, i * 3] = -1.0
            linearMatrix[4 + i * 4 + 1, 2 + i * 3] = -mu
            lowerBound[4 + i * 4 + 1] = -np.inf
            # 3. F_yi < uF_zi
            linearMatrix[4 + i * 4 + 2, 1 + i * 3] = 1.0
            linearMatrix[4 + i * 4 + 2, 2 + i * 3] = -mu
            lowerBound[4 + i * 4 + 2] = -np.inf
            # 4. -F_yi > uF_zi
            linearMatrix[4 + i * 4 + 3, 1 + i * 3] = -1.0
            linearMatrix[4 + i * 4 + 3, 2 + i * 3] = -mu
            lowerBound[4 + i * 4 + 3] = -np.inf

            c_flag = 1.0 if modified_contacts[i] else 0.0
            lowerBound[i] = c_flag * F_min
            upperBound[i] = c_flag * F_max
        carb.profiler.end(41)
        carb.profiler.begin(40, "create hessian")
        sparse_hessian = sp.csc_matrix(hessian)
        carb.profiler.end(40)
        carb.profiler.begin(7, "OSQP initialize")
        # initialize the OSQP solver
        self.solver.setup(
            P=sparse_hessian, q=gradient, A=sp.csc_matrix(linearMatrix), l=lowerBound, u=upperBound, verbose=False
        )
        carb.profiler.end(7)
        carb.profiler.begin(8, "OSQP solve")
        results = self.solver.solve()
        carb.profiler.end(8)
        # print("compare casadi with osqp")
        # print(grf_vec)
        # print(results.x)

        grf = results.x.reshape(4, 3)
        # print(str(grf))
        # print(results.x)
        # print(grf)
        carb.profiler.end(6)
        return grf

    def _get_qp_params(
        self, desired_states: A1DesiredStates, input_states: A1CtrlStates, input_params: A1CtrlParams
    ) -> Tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
        """[summary]
        main internal function, construct parameters of the QP problem

        Args:
            desired_states {A1DesiredStates} -- the desired states
            input_states {A1CtrlStates} -- the control states
            input_params {A1CtrlParams} -- the control parameters

        Returns:
            qp_params: {Tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray] -- inertia_inv, root_acc, acc_weight, u_weight}
        """
        carb.profiler.begin(9, "construct QP param")
        # continuous yaw error
        # reference: http://ltu.diva-portal.org/smash/get/diva2:1010947/FULLTEXT01.pdf
        euler_error = desired_states._euler_d - input_states._euler

        # limit euler error to pi/2
        if euler_error[2] > 3.1415926 * 1.5:  # eulerd 3.14 euler -3.14
            euler_error[2] = desired_states._euler_d[2] - 3.1415926 * 2 - input_states._euler[2]
        elif euler_error[2] < -3.1415926 * 1.5:
            euler_error[2] = desired_states._euler_d[2] + 3.1415926 * 2 - input_states._euler[2]

        root_acc = np.zeros(6)
        root_acc[0:3] = input_params._kp_linear * (desired_states._root_pos_d - input_states._root_pos)
        root_acc[0:3] += input_states._rot_mat_z @ (
            input_params._kd_linear
            * (desired_states._root_lin_vel_d - input_states._rot_mat_z.T @ input_states._root_lin_vel)
        )
        root_acc[3:6] = input_params._kp_angular * euler_error
        root_acc[3:6] += input_params._kd_angular * (
            desired_states._root_ang_vel_d - input_states._rot_mat_z.T @ input_states._root_ang_vel
        )

        # Add gravity
        mass = input_params._robot_mass
        root_acc[2] += mass * 9.8

        for i in range(6):
            if root_acc[i] < -500:
                root_acc[i] = -500
            if root_acc[i] > 500:
                root_acc[i] = 500

        # Create inverse inertia matrix
        inertia_inv = np.zeros([6, 12])
        inertia_inv[0:3] = np.tile(np.eye(3), 4)  # TODO: use the real inertia from URDF
        for i in range(4):
            skew_mat = skew(input_states._foot_pos_abs[i, :])
            inertia_inv[3:6, i * 3 : i * 3 + 3] = input_states._rot_mat_z.T @ skew_mat

        # QP weight
        acc_weight = np.array([1, 1, 1, 20, 20, 10])
        u_weight = 1e-3
        carb.profiler.end(9)
        return inertia_inv, root_acc, acc_weight, u_weight
