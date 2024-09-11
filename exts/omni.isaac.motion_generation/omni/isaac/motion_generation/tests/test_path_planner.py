# Copyright (c) 2021-2023, NVIDIA CORPORATION. All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto. Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.
#

import asyncio
import json
import os

import numpy as np
import omni.isaac.motion_generation.interface_config_loader as interface_config_loader
import omni.kit.test
from omni.isaac.core.objects import FixedCuboid, VisualCuboid
from omni.isaac.core.objects.ground_plane import GroundPlane
from omni.isaac.core.prims import XFormPrim
from omni.isaac.core.robots import Robot
from omni.isaac.core.utils.numpy.rotations import euler_angles_to_quats, rot_matrices_to_quats
from omni.isaac.core.utils.stage import (
    add_reference_to_stage,
    create_new_stage_async,
    get_current_stage,
    update_stage_async,
)
from omni.isaac.core.utils.viewports import set_camera_view
from omni.isaac.core.world import World
from omni.isaac.motion_generation import ArticulationTrajectory

# Import extension python module we are testing with absolute import path, as if we are external user (other extension)
from omni.isaac.motion_generation.articulation_kinematics_solver import ArticulationKinematicsSolver
from omni.isaac.motion_generation.lula.kinematics import LulaKinematicsSolver
from omni.isaac.motion_generation.lula.path_planners import RRT
from omni.isaac.motion_generation.lula.trajectory_generator import LulaCSpaceTrajectoryGenerator
from omni.isaac.motion_generation.path_planner_visualizer import PathPlannerVisualizer
from omni.isaac.nucleus import get_assets_root_path_async
from pxr import Sdf, UsdLux


# Having a test class derived from omni.kit.test.AsyncTestCase declared on the root of module will
# make it auto-discoverable by omni.kit.test
class TestPathPlanner(omni.kit.test.AsyncTestCase):
    # Before running each test
    async def setUp(self):
        self._physics_fps = 60
        self._physics_dt = 1 / self._physics_fps  # duration of physics frame in seconds

        ext_manager = omni.kit.app.get_app().get_extension_manager()
        ext_id = ext_manager.get_enabled_extension_id("omni.isaac.motion_generation")
        self._articulation_policy_extension_path = ext_manager.get_extension_path(ext_id)

        self._polciy_config_dir = os.path.join(self._articulation_policy_extension_path, "motion_policy_configs")
        self.assertTrue(
            os.path.exists(os.path.join(self._polciy_config_dir, "policy_map.json")),
            f'{os.path.join(self._polciy_config_dir, "policy_map.json")}',
        )
        with open(os.path.join(self._polciy_config_dir, "policy_map.json")) as policy_map:
            self._policy_map = json.load(policy_map)

        await update_stage_async()
        robot_prim_path = "/panda"
        usd_path = await get_assets_root_path_async()
        usd_path += "/Isaac/Robots/Franka/franka.usd"
        await create_new_stage_async()

        await update_stage_async()
        add_reference_to_stage(usd_path, robot_prim_path)

        self._timeline = omni.timeline.get_timeline_interface()

        set_camera_view(
            eye=[0.7 * 2.95, 0.7 * 3.3, 0.7 * 5.5], target=[0, 0, 0], camera_prim_path="/OmniverseKit_Persp"
        )

        rrt_config = interface_config_loader.load_supported_path_planner_config("Franka", "RRT")
        rrt_config["robot_description_path"] = os.path.join(
            self._articulation_policy_extension_path,
            "omni",
            "isaac",
            "motion_generation",
            "tests",
            "test_assets",
            "franka_conservative_spheres_robot_description.yaml",
        )
        rrt = RRT(**rrt_config)
        rrt.set_max_iterations(10000)
        self._planner = rrt

        self._cspace_trajectory_planner = LulaCSpaceTrajectoryGenerator(
            rrt_config["robot_description_path"], rrt_config["urdf_path"]
        )

        # Start Simulation and wait
        self._timeline.play()
        await update_stage_async()

        self._robot = Robot(robot_prim_path)
        self._robot.initialize()

        await self.reset_robot(self._robot)

        kinematics_config = interface_config_loader.load_supported_lula_kinematics_solver_config("Franka")
        self._kinematics_solver = LulaKinematicsSolver(**kinematics_config)
        self._articulation_kinematics_solver = ArticulationKinematicsSolver(
            self._robot, self._kinematics_solver, "right_gripper"
        )

        self._planner_visualizer = PathPlannerVisualizer(self._robot, self._planner)

        self.PRINT_GOLDEN_VALUES = False
        self.TEST_FOR_DETERMINISM = False  # Right now RRT paths are not deterministic across different machines.  Later this will be fixed, and determinism will be tested

    # After running each test
    async def tearDown(self):
        self._timeline.stop()
        while omni.usd.get_context().get_stage_loading_status()[2] > 0:
            print("tearDown, assets still loading, waiting to finish...")
            await asyncio.sleep(1.0)
        await update_stage_async()
        self._articulation_policy = None
        await update_stage_async()
        World.clear_instance()
        pass

    async def _create_light(self):
        sphereLight = UsdLux.SphereLight.Define(get_current_stage(), Sdf.Path("/World/SphereLight"))
        sphereLight.CreateRadiusAttr(2)
        sphereLight.CreateIntensityAttr(100000)
        XFormPrim(str(sphereLight.GetPath().pathString)).set_world_pose([6.5, 0, 12])

    async def _prepare_stage(self, robot):
        # Set settings to ensure deterministic behavior
        # Initialize the robot
        # Play the timeline

        self._timeline.stop()

        world = World()

        await world.initialize_simulation_context_async()
        await self._create_light()

        self._timeline.play()
        await update_stage_async()

        robot.initialize()
        robot.disable_gravity()

        self._robot.post_reset()
        await update_stage_async()

    async def reset_robot(self, robot):
        """
        To make motion_generation outputs more deterministic, this method may be used to
        teleport the robot to specified position targets, setting velocity to 0

        This prevents changes in dynamic_control from affecting motion_generation tests
        """
        robot.post_reset()
        await self._prepare_stage(robot)
        await update_stage_async()
        pass

    async def test_rrt_set_params(self):
        self.assertTrue(self._planner.set_param("seed", 5))
        self.assertTrue(self._planner.set_param("step_size", 0.001))
        self.assertTrue(self._planner.set_param("max_iterations", 1000))
        self.assertTrue(self._planner.set_param("distance_metric_weights", np.ones(7, dtype=np.float64) * 0.8))
        self.assertTrue(self._planner.set_param("task_space_frame_name", "panda_hand"))
        self.assertTrue(
            self._planner.set_param("task_space_limits", np.array([[-1, 1], [-1, 1], [0, 1]], dtype=np.float64))
        )
        self.assertTrue(self._planner.set_param("c_space_planning_params/exploration_fraction", 0.6))
        self.assertTrue(self._planner.set_param("task_space_planning_params/translation_target_zone_tolerance", 0.02))
        self.assertTrue(self._planner.set_param("task_space_planning_params/orientation_target_zone_tolerance", 0.02))
        self.assertTrue(self._planner.set_param("task_space_planning_params/translation_target_final_tolerance", 1e-4))
        self.assertTrue(self._planner.set_param("task_space_planning_params/orientation_target_final_tolerance", 1e-4))
        self.assertTrue(self._planner.set_param("task_space_planning_params/translation_gradient_weight", 1.0))
        self.assertTrue(self._planner.set_param("task_space_planning_params/orientation_gradient_weight", 0.0125))
        self.assertTrue(self._planner.set_param("task_space_planning_params/nn_translation_distance_weight", 1.0))
        self.assertTrue(self._planner.set_param("task_space_planning_params/nn_orientation_distance_weight", 0.125))
        self.assertTrue(self._planner.set_param("task_space_planning_params/task_space_exploitation_fraction", 0.5))
        self.assertTrue(self._planner.set_param("task_space_planning_params/task_space_exploration_fraction", 0.2))
        self.assertTrue(
            self._planner.set_param("task_space_planning_params/max_extension_substeps_away_from_target", 8)
        )
        self.assertTrue(self._planner.set_param("task_space_planning_params/max_extension_substeps_near_target", 5))
        self.assertTrue(
            self._planner.set_param("task_space_planning_params/extension_substep_target_region_scale_factor", 2.0)
        )
        self.assertTrue(self._planner.set_param("task_space_planning_params/unexploited_nodes_culling_scalar", 1.0))
        self.assertTrue(self._planner.set_param("task_space_planning_params/gradient_substep_size", 0.03))
        self.assertFalse(self._planner.set_param("not a real parameter", 5))

        self._planner.reset()

    async def test_rrt_franka(self):
        target_translation = np.array([-0.4, 0.3, 0.5])
        target_orientation = np.array([0.6837119, -0.5746039, 0.14875382, -0.42454764])

        self._planner.set_end_effector_target(target_translation, target_orientation)

        # Check that this doesn't mess anything up
        self._planner.set_cspace_target(np.zeros(7))  # Should just be overridden
        self._planner.set_end_effector_target(target_translation, target_orientation)

        left_barrier = FixedCuboid(
            "/obstacles/left_barrier", size=1.0, scale=np.array([0.01, 0.5, 1]), position=np.array([0, 0.45, 0.5])
        )
        right_barrier = FixedCuboid(
            "/obstacles/right_barrier", size=1.0, scale=np.array([0.04, 0.5, 0.5]), position=np.array([0, -0.45, 0.35])
        )
        back_barrier = FixedCuboid(
            "/obstacles/back_barrier", size=1.0, scale=np.array([0.5, 0.01, 1]), position=np.array([-0.45, 0, 1])
        )
        top_barrier = FixedCuboid(
            "/obstacles/top_barrier", size=1.0, scale=np.array([0.25, 0.25, 0.01]), position=np.array([0, 0, 1.2])
        )
        ground_plane = GroundPlane("/ground")

        target_prim = VisualCuboid(
            "/target", size=1.0, scale=np.full((3,), 0.05), position=target_translation, color=np.array([1, 0, 0])
        )

        self._planner.add_obstacle(left_barrier)
        self._planner.add_obstacle(right_barrier)
        self._planner.add_obstacle(back_barrier)
        self._planner.add_obstacle(top_barrier)
        self._planner.add_obstacle(ground_plane)

        self._planner.update_world()

        actions = self._planner.compute_path(
            self._planner_visualizer.get_active_joints_subset().get_joint_positions(), []
        )
        # print("FK: ",
        #         rot_matrices_to_quats(
        #             self._kinematics_solver.compute_forward_kinematics(
        #                 self._articulation_kinematics_solver.get_end_effector_frame(),
        #                 actions[-1])[1]
        #         )
        #     )

        if self.PRINT_GOLDEN_VALUES:
            print("Number of actions: ", len(actions))
            print("Final action: ", end="")
            [print(actions[-1], ",", end="") for i in range(len(actions[-1]))]

        LOGGED_PATH_LEN = 11
        LOGGED_FINAL_POSITION = np.array(
            [
                -2.2235743574338285,
                1.2670535347824194,
                -1.5803078127051602,
                -2.044557783811974,
                -0.889700828512457,
                1.6705503159953106,
                0.41399271401981974,
                None,
                None,
            ]
        )

        if self.TEST_FOR_DETERMINISM:
            self.assertTrue(
                len(actions) == LOGGED_PATH_LEN,
                "Logged plan has length " + str(LOGGED_PATH_LEN) + "; this plan has length " + str(len(actions)),
            )
            await self.assertAlmostEqual(
                LOGGED_FINAL_POSITION,
                actions[-1],
                f"The final position in the path doesn't match the logged position: {LOGGED_FINAL_POSITION} != {actions[-1]}",
            )
        else:
            self.assertTrue(len(actions) > 0, f"{len(actions)}")

        await self.follow_plan(actions, 0.01)

        await self.reset_robot(self._robot)

        self._planner.set_end_effector_target(target_translation)

        actions = self._planner.compute_path(
            self._planner_visualizer.get_active_joints_subset().get_joint_positions(), []
        )

        self.assertTrue(len(actions) > 0, f"{len(actions)}")
        await self.follow_plan(actions, 0.01)

    async def test_rrt_franka_moving_base(self):
        target_translation = np.array([1.4, -0.1, 0.5])
        target_orientation = np.array([0.95, 0.05, 0, 0])

        self._planner.set_end_effector_target(target_translation, target_orientation)

        robot_base_position = np.array([1, 0, 0.2])
        robot_base_orientation = euler_angles_to_quats(np.array([0.1, 0, 0.3]))

        barrier = FixedCuboid(
            "/obstacles/barrier", size=1.0, scale=np.array([0.01, 0.5, 1]), position=np.array([1.2, -0.3, 0.5])
        )

        target_prim = VisualCuboid(
            "/target", size=1.0, scale=np.full((3,), 0.05), position=target_translation, color=np.array([1, 0, 0])
        )

        self._planner.add_obstacle(barrier)

        self._planner.set_robot_base_pose(robot_base_position, robot_base_orientation)
        self._kinematics_solver.set_robot_base_pose(robot_base_position, robot_base_orientation)
        self._robot.set_world_pose(robot_base_position, robot_base_orientation)

        self._planner.update_world()

        actions = self._planner.compute_path(
            self._planner_visualizer.get_active_joints_subset().get_joint_positions(), []
        )

        if self.PRINT_GOLDEN_VALUES:
            print("Number of actions: ", len(actions))
            print("Final action: ", end="")
            [print(actions[-1].joint_positions[i], ",", end="") for i in range(len(actions[-1].joint_positions))]

        LOGGED_PATH_LEN = 6
        LOGGED_FINAL_POSITION = np.array(
            [
                -1.287984743737736,
                -1.194971983321831,
                1.3341467119855843,
                -3.0448501997009876,
                0.229684493139643,
                3.1069385805619922,
                -1.3131528226307583,
                None,
                None,
            ]
        )

        if self.TEST_FOR_DETERMINISM:
            self.assertTrue(
                len(actions) == LOGGED_PATH_LEN,
                "Logged plan has length " + str(LOGGED_PATH_LEN) + "; this plan has length " + str(len(actions)),
            )
            await self.assertAlmostEqual(
                LOGGED_FINAL_POSITION,
                actions[-1].joint_positions,
                f"The final position in the path doesn't match the logged position: {LOGGED_FINAL_POSITION} != {actions[-1].joint_positions}",
            )
        else:
            self.assertTrue(len(actions) > 0, f"{len(actions)}")

        await self.follow_plan(actions, 0.01)

        await self.reset_robot(self._robot)
        self._robot.set_world_pose(robot_base_position, robot_base_orientation)

        self._planner.set_end_effector_target(target_translation)

        actions = self._planner.compute_path(
            self._planner_visualizer.get_active_joints_subset().get_joint_positions(), []
        )

        self.assertTrue(len(actions) > 0, f"{len(actions)}")
        await self.follow_plan(actions, 0.01)

    async def test_rrt_franka_cspace_target(self):
        cspace_target = np.array(
            [
                -2.2235743574338285,
                1.2670535347824194,
                -1.5803078127051602,
                -2.044557783811974,
                -0.889700828512457,
                1.6705503159953106,
                0.41399271401981974,
            ]
        )
        target_pose = np.array([-0.4, 0.3, 0.5])

        self._planner.set_cspace_target(cspace_target)

        # Check that this doesn't mess anything up
        self._planner.set_end_effector_target(np.zeros(3))  # Should just be overridden
        self._planner.set_cspace_target(cspace_target)

        left_barrier = FixedCuboid(
            "/obstacles/left_barrier", size=1.0, scale=np.array([0.01, 0.5, 1]), position=np.array([0, 0.45, 0.5])
        )
        right_barrier = FixedCuboid(
            "/obstacles/right_barrier", size=1.0, scale=np.array([0.04, 0.5, 0.5]), position=np.array([0, -0.45, 0.35])
        )
        back_barrier = FixedCuboid(
            "/obstacles/back_barrier", size=1.0, scale=np.array([0.5, 0.01, 1]), position=np.array([-0.45, 0, 1])
        )
        top_barrier = FixedCuboid(
            "/obstacles/top_barrier", size=1.0, scale=np.array([0.25, 0.25, 0.01]), position=np.array([0, 0, 1.2])
        )
        ground_plane = GroundPlane("/ground", z_position=-0.0305)

        target_prim = VisualCuboid(
            "/target", size=1.0, scale=np.full((3,), 0.05), position=target_pose, color=np.array([1, 0, 0])
        )

        self._planner.add_obstacle(left_barrier)
        self._planner.add_obstacle(right_barrier)
        self._planner.add_obstacle(back_barrier)
        self._planner.add_obstacle(top_barrier)
        self._planner.add_obstacle(ground_plane)

        self._planner.update_world()

        actions = self._planner.compute_path(
            self._planner_visualizer.get_active_joints_subset().get_joint_positions(), []
        )

        if self.PRINT_GOLDEN_VALUES:
            print("Number of actions: ", len(actions))
            print("Final action: ", end="")
            [print(actions[-1].joint_positions[i], ",", end="") for i in range(len(actions[-1].joint_positions))]

        LOGGED_PATH_LEN = 11
        LOGGED_FINAL_POSITION = np.array(
            [
                -2.2235743574338285,
                1.2670535347824194,
                -1.5803078127051602,
                -2.044557783811974,
                -0.889700828512457,
                1.6705503159953106,
                0.41399271401981974,
                None,
                None,
            ]
        )

        if self.TEST_FOR_DETERMINISM:
            self.assertTrue(
                len(actions) == LOGGED_PATH_LEN,
                "Logged plan has length " + str(LOGGED_PATH_LEN) + "; this plan has length " + str(len(actions)),
            )
            await self.assertAlmostEqual(
                LOGGED_FINAL_POSITION,
                actions[-1].joint_positions,
                f"The final position in the path doesn't match the logged position: {LOGGED_FINAL_POSITION} != {actions[-1].joint_positions}",
            )
        else:
            self.assertTrue(len(actions) > 0, f"{len(actions)}")

        await self.follow_plan(actions, 0.01)

    async def _test_traj_gen_with_rrt(self, rrt_plan, interpolation_max_dist, path_dist_thresh=0.01):
        # Pure Math RRT interpolation vs Generated Terajectory

        interpolated_plan = self._planner_visualizer.interpolate_path(rrt_plan, interpolation_max_dist)
        trajectory = self._cspace_trajectory_planner.compute_c_space_trajectory(interpolated_plan)
        self.assertTrue(trajectory is not None, "Failed to Generate Trajectory connecting RRT waypoints!")

        discretized_trajectory = np.array(
            [
                trajectory.get_joint_targets(t)[0]
                for t in np.arange(trajectory.start_time, trajectory.end_time, self._physics_dt)
            ]
        )
        min_path_dists = np.min(
            np.linalg.norm(interpolated_plan[:, None, :] - discretized_trajectory[None, :, :], axis=2), axis=1
        )

        print("Min Path Dists: [")
        for path_dist in min_path_dists:
            print(np.round(path_dist, decimals=3), ",", end="")
        print("\n]")

        self.assertTrue(np.all(min_path_dists < path_dist_thresh))

    async def follow_plan(self, plan, interpolation_max_dist, path_dist_thresh=0.02):
        interpolated_plan = self._planner_visualizer.interpolate_path(plan, interpolation_max_dist)
        trajectory = self._cspace_trajectory_planner.compute_c_space_trajectory(interpolated_plan)
        self.assertTrue(trajectory is not None, "Failed to Generate Trajectory connecting RRT waypoints!")

        discretized_trajectory = np.array(
            [
                trajectory.get_joint_targets(t)[0]
                for t in np.arange(trajectory.start_time, trajectory.end_time, self._physics_dt)
            ]
        )
        articulation_sequence = ArticulationTrajectory(self._robot, trajectory, self._physics_dt).get_action_sequence()

        robot_poses = []
        for action in articulation_sequence:
            robot_poses.append(self._planner_visualizer.get_active_joints_subset().get_joint_positions())
            self._robot.apply_action(action)
            await update_stage_async()

        robot_poses.append(self._planner_visualizer.get_active_joints_subset().get_joint_positions())
        robot_poses = np.array(robot_poses)

        # Distance of Ideal RRT Path to real robot path
        rrt_path_dists = np.min(np.linalg.norm(interpolated_plan[:, None, :] - robot_poses[None, :, :], axis=2), axis=1)

        # Distance of Generated Trajectory to real robot path
        traj_path_dists = np.min(
            np.linalg.norm(discretized_trajectory[:, None, :] - robot_poses[None, :, :], axis=2), axis=1
        )

        # Distance of Generated Trajectory to Ideal RRT Path
        traj_to_rrt_path_dists = np.min(
            np.linalg.norm(interpolated_plan[:, None, :] - discretized_trajectory[None, :, :], axis=2), axis=1
        )

        print("Max Distance To RRT Path:", np.max(rrt_path_dists))
        print("Max Distance To Generated Path:", np.max(traj_path_dists))
        print("Max Distance From Generated Path To RRT Path:", np.max(traj_to_rrt_path_dists))

        self.assertTrue(
            np.all(rrt_path_dists < path_dist_thresh),
            "Robot trajectory was too far from ideal RRT trajectory to be trusted",
        )
        self.assertTrue(
            np.all(traj_path_dists < path_dist_thresh), "Robot trajectory was too far from the commanded trajectory"
        )
        self.assertTrue(
            np.all(traj_to_rrt_path_dists < path_dist_thresh),
            "Generated Trajectory was too far from ideal RRT trajectory to be used",
        )

    async def assertAlmostEqual(self, a, b, dbg_msg=""):
        # overriding method because it doesn't support iterables
        a = np.array(a)
        b = np.array(b)
        self.assertFalse(np.any(abs((a[a != np.array(None)] - b[b != np.array(None)])) > 1e-3), dbg_msg)
        pass
