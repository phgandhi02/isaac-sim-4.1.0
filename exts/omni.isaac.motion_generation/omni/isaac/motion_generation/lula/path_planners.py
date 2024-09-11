# Copyright (c) 2021-2023, NVIDIA CORPORATION. All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto. Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.
#

from typing import List, Union

import carb
import lula
import numpy as np
from omni.isaac.core import objects
from omni.isaac.core.utils.numpy.rotations import quats_to_rot_matrices

from ..path_planning_interface import PathPlanner
from .interface_helper import LulaInterfaceHelper


class RRT(LulaInterfaceHelper, PathPlanner):
    """RRT is a stochastic algorithm for quickly finding a feasible path in cspace to move a robot from a starting pose to a target pose.
    This class implements the PathPlanner interface, as well as exposing RRT-specific parameters.

    Args:
        robot_description_path (str): path to a robot description yaml file
        urdf_path (str): path to robot urdf
        rrt_config_path (str): path to an rrt parameter yaml file
        end_effector_frame_name (str): name of the robot end effector frame (must be present in the robot urdf)
    """

    def __init__(self, robot_description_path: str, urdf_path: str, rrt_config_path: str, end_effector_frame_name: str):

        robot_description = lula.load_robot(robot_description_path, urdf_path)
        self.end_effector_frame_name = end_effector_frame_name

        LulaInterfaceHelper.__init__(self, robot_description)

        world_view = self._world.add_world_view()

        self.rrt_config_path = rrt_config_path

        self._rrt = lula.create_motion_planner(self.rrt_config_path, self._robot_description, world_view)

        self._rrt.set_param("task_space_frame_name", self.end_effector_frame_name)
        self._seed = 123456

        self._plan = None

        self._cspace_target = None
        self._taskspace_target_position = None
        self._taskspace_target_rotation = None

    def compute_path(self, active_joint_positions, watched_joint_positions) -> np.array:
        __doc__ = PathPlanner.compute_path.__doc__

        active_joint_positions = active_joint_positions.astype(np.float64)
        if self._taskspace_target_position is None and self._cspace_target is not None:
            self._generate_plan_to_cspace_target(active_joint_positions)
        elif self._taskspace_target_position is None:
            self._plan = None
        else:
            self._generate_plan_to_taskspace_target(active_joint_positions)

        return self._plan

    def set_robot_base_pose(self, robot_position: np.array, robot_orientation: np.array) -> None:
        __doc__ = LulaInterfaceHelper.set_robot_base_pose.__doc__

        return LulaInterfaceHelper.set_robot_base_pose(self, robot_position, robot_orientation)

    def set_cspace_target(self, active_joint_targets: np.array) -> None:
        __doc__ = PathPlanner.set_cspace_target.__doc__

        self._cspace_target = active_joint_targets
        self._taskspace_target_position = None
        self._taskspace_target_rotation = None

    def set_end_effector_target(self, target_translation, target_orientation=None) -> None:
        __doc__ = PathPlanner.set_end_effector_target.__doc__

        if target_translation is not None:
            self._taskspace_target_position = (target_translation * self._meters_per_unit).astype(np.float64)
        else:
            self._taskspace_target_position = None

        if target_orientation is not None:
            self._taskspace_target_rotation = quats_to_rot_matrices(target_orientation)
        else:
            self._taskspace_target_rotation = None
        self._cspace_target = None

    def get_active_joints(self) -> List:
        __doc__ = PathPlanner.get_active_joints.__doc__

        return LulaInterfaceHelper.get_active_joints(self)

    def get_watched_joints(self) -> List:
        return LulaInterfaceHelper.get_watched_joints(self)

    def add_obstacle(self, obstacle: objects, static: bool = False) -> bool:
        __doc__ = PathPlanner.add_obstacle.__doc__
        return PathPlanner.add_obstacle(self, obstacle, static)

    def add_cuboid(
        self,
        cuboid: Union[objects.cuboid.DynamicCuboid, objects.cuboid.FixedCuboid, objects.cuboid.VisualCuboid],
        static: bool = False,
    ) -> bool:
        return LulaInterfaceHelper.add_cuboid(self, cuboid, static)

    def add_sphere(
        self, sphere: Union[objects.sphere.DynamicSphere, objects.sphere.VisualSphere], static: bool = False
    ) -> bool:
        return LulaInterfaceHelper.add_sphere(self, sphere, static)

    def add_capsule(
        self, capsule: Union[objects.capsule.DynamicCapsule, objects.capsule.VisualCapsule], static: bool = False
    ) -> bool:
        return LulaInterfaceHelper.add_capsule(self, capsule, static)

    def add_ground_plane(self, ground_plane: objects.ground_plane.GroundPlane) -> bool:
        return LulaInterfaceHelper.add_ground_plane(self, ground_plane)

    def disable_obstacle(self, obstacle: objects) -> bool:
        return LulaInterfaceHelper.disable_obstacle(self, obstacle)

    def enable_obstacle(self, obstacle: objects) -> bool:
        return LulaInterfaceHelper.enable_obstacle(self, obstacle)

    def remove_obstacle(self, obstacle: objects) -> bool:
        return LulaInterfaceHelper.remove_obstacle(self, obstacle)

    def update_world(self, updated_obstacles: List = None) -> None:
        LulaInterfaceHelper.update_world(self, updated_obstacles)
        self._rrt.update_world_view()

    def reset(self) -> None:
        LulaInterfaceHelper.reset(self)

        self._rrt = lula.create_motion_planner(
            self.rrt_config_path, self._robot_description, self._world.add_world_view()
        )

        self._rrt.set_param("task_space_frame_name", self.end_effector_frame_name)
        self._seed = 123456

    def set_max_iterations(self, max_iter: int) -> None:
        """Set the maximum number of iterations of RRT before a failure is returned

        Args:
            max_iter (int): Maximum number of iterations of RRT before a failure is returned.
                The time it takes to return a failure scales quadratically with max_iter
        """
        self._rrt.set_param("max_iterations", max_iter)

    def set_random_seed(self, random_seed: int) -> None:
        """Set the random seed that RRT uses to generate a solution

        Args:
            random_seed (int): Used to initialize random sampling. random_seed must be positive.
        """
        self._seed = random_seed

    def set_param(self, param_name: str, value: Union[np.array, float, int, str]) -> bool:
        """Set a parameter for the RRT algorithm.  The parameters and their appropriate values are enumerated below:

        `seed` (int):
            -Used to initialize random sampling.
            -`seed` must be positive.
            -This parameter may also be set through the set_random_seed() function

        `step_size` (float):
            -Step size for tree extension.
            -It is assumed that a straight path connecting two valid c-space configurations with
            separation distance <= `step_size` is a valid edge, where separation distance is defined
            as the L2-norm of the difference between the two configurations.
            -`step_size` must be positive.

        `max_iterations` (int)
            - Maximum number of iterations of tree extensions that will be attempted.
            - If `max_iterations` is reached without finding a valid path, the `Results` will
              indicate `path_found` is `false` and `path` will be an empty vector.
            - `max_iterations` must be positive.

        `distance_metric_weights` (np.array[np.float64[num_dof,]])
            - When selecting a node for tree extension, the closest node is defined using a weighted, squared L2-norm:
                distance = (q0 - q1)^T * W * (q0 - q1)
                where q0 and q1 represent two configurations and W is a diagonal matrix formed from
                `distance_metric_weights`.
            - The length of the `distance_metric_weights` must be equal to the number of c-space
              coordinates for the robot and each weight must be positive.

        `task_space_frame_name` (string)
            - Indicate the name (from URDF) of the frame to be used for task space planning.
            - With current implementation, setting a `task_space_frame_name` that is not found in the
              kinematics will throw an exception rather than failing gracefully.

        `task_space_limits` (np.array[np.float64[3,2]])
            - Task space limits define a bounding box used for sampling task space when planning
              a path to a task space target.
            - The specified `task_space_limits` should be a (3 x 2) matrix.  Rows correspond to the xyz
              dimensions of the bounding box, and columns 0 and 1 correspond to the lower and upper limit repectively.
            - Each upper limit must be >= the corresponding lower limit.

        `c_space_planning_params/exploration_fraction` (float)
            - The c-space planner uses RRT-Connect to try to find a path to a c-space target.
            - RRT-Connect attempts to iteratively extend two trees (one from the initial configuration and one from the target configuration)
                until the two trees can be connected. The
                configuration to which a tree is extended can be either a random sample
                (i.e., exploration) or a node on the tree to which connection is desired
                (i.e., exploitation). The `exploration_fraction` controls the fraction of steps that are
                exploration steps. It is generally recommended to set `exploration_fraction` in range
                [0.5, 1), where 1 corresponds to a single initial exploitation step followed by only
                exploration steps. Values of between [0, 0.5) correspond to more exploitation than
                exploration and are not recommended. If a value outside range [0, 1] is provided, a
                warning is logged and the value is clamped to range [0, 1].
            - A default value of 0.5 is recommended as a starting value for initial testing with a given
                system.

        `task_space_planning_params/translation_target_zone_tolerance` (float)
            - A configuration has reached the task space translation target when task space position has
              an L2 Norm within `translation_target_zone_tolerance` of the target.
            - It is assumed that a valid configuration within the target tolerance can be moved directly
              to the target configuration using an inverse kinematics solver and linearly stepping
              towards the solution.
            - In general, it is recommended that the size of the translation target zone be on the same
              order of magnitude as the translational distance in task-space corresponding to moving the
              robot in configuration space by one step with an L2 norm of `step_size`.

        `task_space_planning_params/orientation_target_zone_tolerance` (float)
            - A configuration has reached the task space pose target when task space orientation is
              within `orientation_target_zone_tolerance` radians and an L2 norm translation
              within `translation_target_zone_tolerance` of the target.
            - It is assumed that a valid configuration within the target tolerances can be moved
              directly to the target configuration using an inverse kinematics solver and linearly
              stepping towards the solution.
            - In general, it is recommended that the size of the orientation target zone be on the same
              order of magnitude as the rotational distance in task-space corresponding to moving the
              robot in configuration space by one step with an L2 norm of `step_size`.

         `task_space_planning_params/translation_target_final_tolerance` (float)
            - Once a path is found that terminates within `translation_target_zone_tolerance`, an IK
              solver is used to find a configuration space solution corresponding to the task space
              target. This solver terminates when the L2-norm of the corresponding task space position
              is within `translation_target_final_tolerance` of the target.
            - Note: This solver assumes that if a c-space configuration within
              `translation_target_zone_tolerance` is found then this c-space configuration can be
              moved linearly in cspace to the IK solution. If this assumption is NOT met, the returned
              path will not reach the task space target within the `translation_target_final_tolerance`
              and an error is logged.
            - The recommended default value is 1e-4, but in general this value should be set to a
              positive value that is considered "good enough" precision for the specific system.

         `task_space_planning_params/orientation_target_final_tolerance` (float)
            - For pose targets, once a path is found that terminates within
              `orientation_target_zone_tolerance` and `translation_target_zone_tolerance` of the target,
              an IK solver is used to find a configuration space solution corresponding to the task
              space target. This solver terminates when the L2-norm of the corresponding task space
              position is within `orientation_target_final_tolerance` and
              `translation_target_final_tolerance` of the target.
            - Note: This solver assumes that if a c-space configuration within the target zone
              tolerances is found then this c-space configuration can be moved linearly in cspace to the
              IK solution. If this assumption is NOT met, the returned path will not reach the task
              space target within the the final target tolerances and an error is logged.
            - The recommended default value is 1e-4, but in general this value should be set to a
              positive value that is considered "good enough" precision for the specific system.

         `task_space_planning_params/translation_gradient_weight` (float)
            - For pose targets, computed translation and orientation gradients are linearly weighted by
              `translation_gradient_weight` and `orientation_gradient_weight` to compute a combined
              gradient step when using the Jacobian Transpose method to guide tree expansion
              towards a task space target.
            - A default value of 1.0 is recommended as a starting value for initial testing with a given
              system.
            - Must be > 0.

         `task_space_planning_params/orientation_gradient_weight` (float)
            - For pose targets, computed translation and orientation gradients are linearly weighted by
              `translation_gradient_weight` and `orientation_gradient_weight` to compute a combined
              gradient step when using the Jacobian Transpose method to guide tree expansion
              towards a task space target.
            - A default value of 0.125 is recommended as a starting value for initial testing with a
              given system.
            - Must be > 0.

         `task_space_planning_params/nn_translation_distance_weight` (float)
            - For pose targets, nearest neighbor distances are computed by linearly weighting
              translation and orientation distance by `nn_translation_distance_weight` and
              `nn_orientation_distance_weight`.
            - Nearest neighbor search is used to select the node from which the tree of valid
              configurations will be expanded.
            - A default value of 1.0 is recommended as a starting value for initial testing with a given
              system.
            - Must be > 0.

         `task_space_planning_params/nn_orientation_distance_weight` (float)
            - For pose targets, nearest neighbor distances are computed by linearly weighting
              translation and orientation distance by `nn_translation_distance_weight` and
              `nn_orientation_distance_weight`.
            - Nearest neighbor search is used to select the node from which the tree of valid
              configurations will be expanded.
            - A default value of 0.125 is recommended as a starting value for initial testing with a
              given system.
            - Must be > 0.

        `task_space_planning_params/task_space_exploitation_fraction` (float)
            - Fraction of iterations for which tree is extended towards target position in task space.
            - Must be in range [0, 1]. Additionally, the sum of `task_space_exploitation_fraction` and
                `task_space_exploration_fraction` must be <= 1.
            - A default value of 0.4 is recommended as a starting value for initial testing with a given
                system.

        `task_space_planning_params/task_space_exploration_fraction` (float)
            - Fraction of iterations for which tree is extended towards random position in task space.
            - Must be in range [0, 1]. Additionally, the sum of `task_space_exploitation_fraction` and
                `task_space_exploration_fraction` must be <= 1.
            - A default value of 0.1 is recommended as a starting value for initial testing with a given
                system.

            The remaining fraction beyond `task_space_exploitation_fraction` and
            `task_space_exploration_fraction` is a `c_space_exploration_fraction` that is
            implicitly defined as:

            1 - (`task_space_exploitation_fraction` + `task_space_exploration_fraction`)

            In general, easier path searches will take less time with higher exploitation fraction
            while more difficult searches will waste time if the exploitation fraction is too high
            and benefit from greater combined exploration fraction.

        `task_space_planning_params/max_extension_substeps_away_from_target` (int)
            - Maximum number of Jacobian transpose gradient descent substeps that may be taken
              while the end effector is away from the task-space target.
            - The threshold for nearness is determined by the
              `extension_substep_target_region_scale_factor` parameter.
            - A default value of 6 is recommended as a starting value for initial testing with a given
              system.

        `task_space_planning_params/max_extension_substeps_near_target` (int)
            - Maximum number of Jacobian transpose gradient descent substeps that may be taken
              while the end effector is near the task-space target.
            - The threshold for nearness is determined by the
              `extension_substep_target_region_scale_factor` parameter.
            - A default value of 50 is recommended as a starting value for initial testing with a given
              system.

        `task_space_planning_params/extension_substep_target_region_scale_factor` : (float)
            - A scale factor used to determine whether the end effector is close enough to the target
              to change the amount of gradient descent substeps allowed when adding a node in RRT.
            - The `max_extension_substeps_near_target` parameter is used when the distance
              (i.e., L2 norm) between the end effector and target position is less than
              `extension_substep_target_region_scale_factor` * `x_zone_target_tolerance`.
            - Must be greater than or equal to 1.0; a value of 1.0 effectively disables the
             `max_extension_substeps_near_target` parameter.
            - A default value of 2.0 is recommended as a starting value for initial testing with a given
              system.

        Args:
            param_name (str): Name of parameter
            value (Union[np.ndarray[np.float64],float,int,str]): value of parameter

        Returns:
            bool: True if the parameter was set successfully

        """
        if param_name == "seed":
            self.set_random_seed(value)
            return True

        if param_name == "task_space_limits":
            value = [self._rrt.Limit(row[0], row[1]) for row in value]

        return self._rrt.set_param(param_name, value)

    def _generate_plan_to_cspace_target(self, joint_positions):
        if self._cspace_target is None:
            self._plan = None
            return
        plan = self._rrt.plan_to_cspace_target(joint_positions, self._cspace_target)
        if plan.path_found:
            self._plan = np.array(plan.path)
        else:
            self._plan = None

    def _generate_plan_to_taskspace_target(self, joint_positions):
        if self._taskspace_target_position is None:
            self._plan = None
            return

        trans_rel, rot_rel = LulaInterfaceHelper._get_pose_rel_robot_base(
            self, self._taskspace_target_position, self._taskspace_target_rotation
        )

        self._rrt.set_param("seed", self._seed)
        if rot_rel is not None:
            target_pose = lula.Pose3(lula.Rotation3(rot_rel), trans_rel)
            plan = self._rrt.plan_to_pose_target(joint_positions, target_pose, generate_interpolated_path=False)
        else:
            plan = self._rrt.plan_to_translation_target(joint_positions, trans_rel, generate_interpolated_path=False)

        if plan.path_found:
            self._plan = np.array(plan.path)
        else:
            self._plan = None
