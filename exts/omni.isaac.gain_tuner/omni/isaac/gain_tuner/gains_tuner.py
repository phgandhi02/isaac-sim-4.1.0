# Copyright (c) 2024, NVIDIA CORPORATION. All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto. Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.
#

from enum import Enum

import carb
import numpy as np
import omni.timeline as timeline
from omni.isaac.core.articulations import Articulation
from omni.isaac.core.utils.types import ArticulationAction


class GainsTestMode(Enum):
    SINUSOIDAL = 1
    STEP_FUNCTION = 2


class GainTuner:
    def __init__(self):
        self._articulation = None
        self._robot_prim_path = None

        self._timeline = timeline.get_timeline_interface()

        self._joint_range_clipping_fraction = 0.9
        self._joint_range_maximum = 2 * np.pi
        self._test_duration = 5.0

        self._position_impulse = 0.0
        self._velocity_impulse = 0.0

        self._joint_position_commands = []
        self._joint_velocity_commands = []

        self._observed_joint_positions = []
        self._observed_joint_velocities = []
        self._command_times = []

        self._v_max = None
        self._T = None
        self._joint_indices = None

        self._data_ready = False

        self._test_timestep = 0

        self._gains_test_generator = None

    def setup(self, articulation_path):
        self._robot_prim_path = articulation_path
        self._articulation = Articulation(articulation_path)
        self._articulation.initialize()

        return self._articulation

    def get_articulation(self):
        return self._articulation

    def get_joint_range_clipping_fraction(self):
        return self._joint_range_clipping_fraction

    def get_joint_range_maximum(self):
        return self._joint_range_maximum

    def get_test_duration(self):
        return self._test_duration

    def get_position_impulse(self):
        return self._position_impulse

    def get_velocity_impulse(self):
        return self._velocity_impulse

    def is_data_ready(self):
        return self._data_ready

    def set_joint_range_clipping_fraction(self, fraction):
        self._joint_range_clipping_fraction = fraction

    def set_joint_range_maximum(self, maximum):
        self._joint_range_maximum = maximum

    def set_position_impulse(self, position_impulse):
        self._position_impulse = position_impulse

    def set_velocity_impulse(self, velocity_impulse):
        self._velocity_impulse = velocity_impulse

    def set_test_duration(self, duration):
        self._test_duration = duration

    def get_clipped_joint_ranges(self):
        lower_limit = self._articulation.dof_properties["lower"]
        upper_limit = self._articulation.dof_properties["upper"]

        l = np.copy(lower_limit)
        u = np.copy(upper_limit)

        fractional_clip = (1 - self._joint_range_clipping_fraction) / 2 * (u - l)
        l = l + fractional_clip
        u = u - fractional_clip

        d = u - l
        mask = d > self._joint_range_maximum
        if np.any(mask):
            midpoints = (u[mask] - l[mask]) / 2 + l[mask]
            l[mask] = midpoints - self._joint_range_maximum / 2
            u[mask] = midpoints + self._joint_range_maximum / 2

        return l, u

    def get_v_max_from_max_efforts(self):
        lower_joint_limits, upper_joint_limits = self.get_clipped_joint_ranges()
        max_efforts = np.copy(self._articulation.dof_properties["maxEffort"])

        # There is functionally no difference between this and higher values.
        # This avoids float overflow errors.
        clip_mask = max_efforts > 1e15
        max_efforts[clip_mask] = 1e15

        # Maximum velocity that could be reached by accelerating as fast as possible through half
        # the joint range. This is specific to the sinusoid test
        return np.sqrt(max_efforts * (upper_joint_limits - lower_joint_limits) / 2)

    def get_v_max_from_robot_properties(self):
        max_vel = self._articulation.dof_properties["maxVelocity"]

        max_vel_from_max_eff = self.get_v_max_from_max_efforts()

        return np.minimum(max_vel, max_vel_from_max_eff)

    def get_default_tuning_test_parameters(self):
        lower_joint_limits, upper_joint_limits = self.get_clipped_joint_ranges()

        v_upper_bounds = self.get_v_max_from_robot_properties()

        T = np.array([1.5 + 0.2 * i for i in range(len(lower_joint_limits) - 1, -1, -1)])
        v_max = (upper_joint_limits - lower_joint_limits) * np.pi / T

        mask = v_max > v_upper_bounds
        if np.any(mask):
            v_max[mask] = v_upper_bounds[mask]
            T[mask] = (upper_joint_limits[mask] - lower_joint_limits[mask]) * np.pi / v_max[mask]

        return v_max, T

    def get_period(self, v_max, joint_index):
        lower_joint_limits, upper_joint_limits = self.get_clipped_joint_ranges()
        lower_joint_limit = lower_joint_limits[joint_index]
        upper_joint_limit = upper_joint_limits[joint_index]

        return (upper_joint_limit - lower_joint_limit) * np.pi / v_max

    def get_v_max(self, period, joint_index):
        lower_joint_limits, upper_joint_limits = self.get_clipped_joint_ranges()
        lower_joint_limit = lower_joint_limits[joint_index]
        upper_joint_limit = upper_joint_limits[joint_index]

        return (upper_joint_limit - lower_joint_limit) * np.pi / period

    def get_sinusoidal_joint_commands(self, v_max, T, joint_indices=None):
        lower_joint_limits, upper_joint_limits = self.get_clipped_joint_ranges()

        if joint_indices is not None:
            lower_joint_limits = lower_joint_limits[joint_indices]
            upper_joint_limits = upper_joint_limits[joint_indices]

        p_0 = lower_joint_limits + (upper_joint_limits - lower_joint_limits) / 2

        position = lambda t: p_0 - v_max * T / (2 * np.pi) * np.cos(2 * np.pi * t / T)
        velocity = lambda t: v_max * np.sin(2 * np.pi * t / T)

        return position, velocity

    def get_step_joint_commands(self, joint_indices, test_duration):
        v_max = self.get_v_max_from_robot_properties()

        initial_step_time = 0.03

        lower_joint_limits, upper_joint_limits = self.get_clipped_joint_ranges()
        if joint_indices is not None:
            lower_joint_limits = lower_joint_limits[joint_indices]
            upper_joint_limits = upper_joint_limits[joint_indices]

            v_max = v_max[joint_indices]

        # The amount of time it takes for the Articulation to reach the joint targets is dependent
        # on the max_effort and max_velocity properties.
        steps = upper_joint_limits - lower_joint_limits
        expected_step_duration = steps / v_max

        # Add a safety margin for deceleration
        expected_step_duration *= 1.1

        # Do not go any shorter than .1 seconds
        expected_step_duration = np.maximum(np.full(expected_step_duration.shape, 0.1), expected_step_duration)

        # Make sure that the commanded step function fits within the test duration
        # clip_mask = expected_step_duration > (test_duration - initial_step_time) / 2
        # expected_step_duration[clip_mask] = (test_duration - initial_step_time) / 2

        def position_command(t, lower_joint_limits, upper_joint_limits, expected_step_duration):
            if t < initial_step_time:
                return np.copy(lower_joint_limits)
            r = np.copy(upper_joint_limits)
            second_step_mask = t > expected_step_duration + initial_step_time
            r[second_step_mask] = lower_joint_limits[second_step_mask]
            return r

        position = lambda t: position_command(t, lower_joint_limits, upper_joint_limits, expected_step_duration)
        velocity = lambda t: np.zeros_like(lower_joint_limits)

        return position, velocity

    def set_stiff_gains(self):
        p = np.full((self._articulation.num_dof,), 1e15, dtype=np.float64)
        d = np.full((self._articulation.num_dof,), 1e5, dtype=np.float64)
        self._articulation.get_articulation_controller().set_gains(p, d, save_to_usd=True)

    ############################ Run Gains Test ######################################
    def get_next_action(self, timestep, position_command_fn, velocity_command_fn, joint_indices):
        p = position_command_fn(timestep)
        v = velocity_command_fn(timestep)
        return ArticulationAction(p, v, joint_indices=joint_indices)

    def initialize_gains_test(self, test_mode: GainsTestMode, v_max, T, joint_indices, fixed_joint_positions):
        self._v_max = np.array(v_max)
        self._T = np.array(T)
        self._joint_indices = np.array(joint_indices)

        self._test_timestep = 0

        all_ind = np.arange(self._articulation.num_dof)
        # Fixed joint indices are every joint index that is not in self._joint_indices
        self._fixed_joint_indices = all_ind[~np.isin(all_ind, self._joint_indices)]
        self._fixed_positions = np.array(fixed_joint_positions)

        self._gains_test_generator = self.gains_test_script(test_mode)

        self._data_ready = False

    def _compute_gains_test_dof_error_terms(self, joint_index):
        if joint_index in self._joint_indices:
            remapped_joint_index = np.argmax(self._joint_indices == joint_index)
            pos_rmse = np.sqrt(
                np.mean(
                    np.square(
                        self._joint_position_commands[:, remapped_joint_index]
                        - self._observed_joint_positions[:, joint_index]
                    ),
                    axis=0,
                )
            )
            vel_rmse = np.sqrt(
                np.mean(
                    np.square(
                        self._joint_velocity_commands[:, remapped_joint_index]
                        - self._observed_joint_velocities[:, joint_index]
                    ),
                    axis=0,
                )
            )
        else:
            remapped_joint_index = np.argmax(self._fixed_joint_indices == joint_index)
            pos_rmse = np.sqrt(
                np.mean(
                    np.square(
                        self._fixed_positions[remapped_joint_index] - self._observed_joint_positions[:, joint_index]
                    ),
                    axis=0,
                )
            )
            vel_rmse = np.sqrt(np.mean(np.square(self._observed_joint_velocities[:, joint_index]), axis=0))
        return pos_rmse, vel_rmse

    def compute_gains_test_error_terms(self):
        pos_rmse = []
        vel_rmse = []
        for index in range(self._articulation.num_dof):
            dof_pos_rmse, dof_vel_rmse = self._compute_gains_test_dof_error_terms(index)
            pos_rmse.append(dof_pos_rmse)
            vel_rmse.append(dof_vel_rmse)
        return np.array(pos_rmse), np.array(vel_rmse)

    def update_gains_test(self, step: float):
        try:
            next(self._gains_test_generator)
            self._test_timestep += step
        except StopIteration:
            self._v_max = None
            self._T = None
            return True

    def gains_test_script(self, test_mode: GainsTestMode):
        if self._v_max is None:
            carb.log_error("Attempted to run gains test without first calling initialize_test()")
            return
        if len(self._v_max) == 0:
            carb.log_error("Attempted to run gains test without any moving joints")
            return

        self._joint_position_commands = []
        self._joint_velocity_commands = []

        self._observed_joint_positions = []
        self._observed_joint_velocities = []
        self._command_times = []

        if test_mode == GainsTestMode.SINUSOIDAL:
            position_command_fn, velocity_command_fn = self.get_sinusoidal_joint_commands(
                self._v_max, self._T, self._joint_indices
            )
        elif test_mode == GainsTestMode.STEP_FUNCTION:
            position_command_fn, velocity_command_fn = self.get_step_joint_commands(
                self._joint_indices, self._test_duration
            )

        self._articulation.set_joint_positions(position_command_fn(0) + self._position_impulse, self._joint_indices)
        self._articulation.set_joint_positions(
            self._fixed_positions + self._position_impulse, self._fixed_joint_indices
        )

        self._articulation.set_joint_velocities(velocity_command_fn(0) + self._velocity_impulse, self._joint_indices)
        self._articulation.set_joint_velocities(
            np.full(self._fixed_positions.shape, self._velocity_impulse), self._fixed_joint_indices
        )

        self._joint_position_commands.append(position_command_fn(0))
        self._joint_velocity_commands.append(velocity_command_fn(0))

        self._observed_joint_positions.append(self._articulation.get_joint_positions())
        self._observed_joint_velocities.append(self._articulation.get_joint_velocities())

        self._command_times.append(0)

        yield ()

        while self._test_timestep < self._test_duration:
            action = self.get_next_action(
                self._test_timestep, position_command_fn, velocity_command_fn, self._joint_indices
            )
            self._articulation.apply_action(action)
            if len(self._fixed_joint_indices) > 0:
                self._articulation.apply_action(
                    ArticulationAction(
                        self._fixed_positions,
                        np.zeros_like(self._fixed_positions),
                        joint_indices=self._fixed_joint_indices,
                    )
                )

            self._joint_position_commands.append(action.joint_positions)
            self._joint_velocity_commands.append(action.joint_velocities)
            self._command_times.append(self._test_timestep)

            yield ()  # One Physics Step Happens

            self._observed_joint_positions.append(self._articulation.get_joint_positions())
            self._observed_joint_velocities.append(self._articulation.get_joint_velocities())

        self._articulation.apply_action(
            ArticulationAction(self._observed_joint_positions[-1], np.zeros(self._articulation.num_dof))
        )

        self._joint_position_commands = np.array(self._joint_position_commands)
        self._joint_velocity_commands = np.array(self._joint_velocity_commands)
        self._observed_joint_positions = np.array(self._observed_joint_positions)
        self._observed_joint_velocities = np.array(self._observed_joint_velocities)
        self._command_times = np.array(self._command_times)

        self._data_ready = True

        return

    ############################ For Plotting #######################################

    def get_joint_states_from_gains_test(self, joint_index: int):
        if len(self._observed_joint_positions) == 0:
            return (None, None, None, None, None)

        if joint_index in self._joint_indices:
            remapped_joint_index = np.argmax(self._joint_indices == joint_index)
            return (
                self._joint_position_commands[:, remapped_joint_index],
                self._joint_velocity_commands[:, remapped_joint_index],
                self._observed_joint_positions[:, joint_index],
                self._observed_joint_velocities[:, joint_index],
                self._command_times,
            )
        else:
            remapped_joint_index = np.argmax(self._fixed_joint_indices == joint_index)
            data_len = len(self._command_times)
            return (
                np.full(data_len, self._fixed_positions[remapped_joint_index]),
                np.zeros(data_len),
                self._observed_joint_positions[:, joint_index],
                self._observed_joint_velocities[:, joint_index],
                self._command_times,
            )
