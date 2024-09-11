# Copyright (c) 2021-2023, NVIDIA CORPORATION. All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto. Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.
#

from typing import Optional, Tuple

import numpy as np
from omni.isaac.core.articulations import Articulation
from omni.isaac.motion_generation.articulation_kinematics_solver import ArticulationKinematicsSolver
from omni.isaac.robot_benchmark.benchmark_controllers.base_benchmark_controller import BaseBenchmarkController
from omni.isaac.universal_robots.controllers.rmpflow_controller import RMPFlowController as UR10RMPFlowController


class RMPFlowBenchmarkController(UR10RMPFlowController, BaseBenchmarkController):
    def __init__(self, name: str, robot_articulation: Articulation, physics_dt: float = 1.0 / 60.0):
        UR10RMPFlowController.__init__(self, name, robot_articulation, physics_dt)
        self._kinematics = self.rmp_flow.get_kinematics_solver()
        self._articulation_kinematics = ArticulationKinematicsSolver(
            robot_articulation, self._kinematics, self.rmp_flow.end_effector_frame_name
        )

    def set_robot_base_pose(
        self, robot_base_translation: np.ndarray, robot_base_orientation: Optional[np.ndarray] = None
    ) -> None:
        self.rmp_flow.set_robot_base_pose(robot_base_translation, robot_base_orientation)

    def get_current_end_effector_pose(self) -> Tuple[np.array, np.array]:
        return self._articulation_kinematics.compute_end_effector_pose()

    def get_articulation_subset(self):
        return self._articulation_motion_policy.get_active_joints_subset()
