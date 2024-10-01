# Copyright (c) 2022-2023, NVIDIA CORPORATION. All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto. Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.
#

import numpy as np
from omni.isaac.core.articulations import Articulation
from omni.isaac.core.objects import DynamicCuboid, FixedCuboid, GroundPlane
from omni.isaac.core.prims import XFormPrim
from omni.isaac.core.utils import distance_metrics
from omni.isaac.core.utils.numpy.rotations import euler_angles_to_quats, quats_to_rot_matrices
from omni.isaac.core.utils.stage import add_reference_to_stage
from omni.isaac.core.utils.types import ArticulationAction
from omni.isaac.core.utils.viewports import set_camera_view
from omni.isaac.motion_generation import ArticulationMotionPolicy, RmpFlow
from omni.isaac.motion_generation.interface_config_loader import load_supported_motion_policy_config
from omni.isaac.nucleus import get_assets_root_path


class FrankaRmpFlowExampleScript:
    def __init__(self):
        self._rmpflow = None
        self._articulation_rmpflow = None

        self._articulation = None
        self._target = None

        self._script_generator = None

    def load_example_assets(self):
        """Load assets onto the stage and return them so they can be registered with the
        core.World.

        This function is called from ui_builder._setup_scene()

        The position in which things are loaded is also the position to which
        they will be returned on reset.
        """

        robot_prim_path = "/panda"
        path_to_robot_usd = get_assets_root_path() + "/Isaac/Robots/Franka/franka.usd"

        add_reference_to_stage(path_to_robot_usd, robot_prim_path)
        self._articulation = Articulation(robot_prim_path)

        add_reference_to_stage(get_assets_root_path() + "/Isaac/Props/UIElements/frame_prim.usd", "/World/target")
        self._target = XFormPrim(
            "/World/target",
            scale=[0.04, 0.04, 0.04],
            position=np.array([0.4, 0, 0.25]),
            orientation=euler_angles_to_quats([0, np.pi, 0]),
        )

        self._obstacles = [
            FixedCuboid(
                name="ob1",
                prim_path="/World/obstacle_1",
                scale=np.array([0.03, 1.0, 0.3]),
                position=np.array([0.25, 0.25, 0.15]),
                color=np.array([0.0, 0.0, 1.0]),
            ),
            FixedCuboid(
                name="ob2",
                prim_path="/World/obstacle_2",
                scale=np.array([0.5, 0.03, 0.3]),
                position=np.array([0.5, 0.25, 0.15]),
                color=np.array([0.0, 0.0, 1.0]),
            ),
        ]

        self._goal_block = DynamicCuboid(
            name="Cube",
            position=np.array([0.4, 0, 0.025]),
            prim_path="/World/pick_cube",
            size=0.05,
            color=np.array([1, 0, 0]),
        )
        self._ground_plane = GroundPlane("/World/Ground")

        # Return assets that were added to the stage so that they can be registered with the core.World
        return self._articulation, self._target, *self._obstacles, self._goal_block, self._ground_plane

    def setup(self):
        """
        This function is called after assets have been loaded from ui_builder._setup_scenario().
        """
        # Set a camera view that looks good
        set_camera_view(eye=[2, 0.8, 1], target=[0, 0, 0], camera_prim_path="/OmniverseKit_Persp")

        # Loading RMPflow can be done quickly for supported robots
        rmp_config = load_supported_motion_policy_config("Franka", "RMPflow")

        # Initialize an RmpFlow object
        self._rmpflow = RmpFlow(**rmp_config)

        for obstacle in self._obstacles:
            self._rmpflow.add_obstacle(obstacle)

        # Use the ArticulationMotionPolicy wrapper object to connect rmpflow to the Franka robot articulation.
        self._articulation_rmpflow = ArticulationMotionPolicy(self._articulation, self._rmpflow)

        # Create a script generator to execute my_script().
        self._script_generator = self.my_script()

    def reset(self):
        """
        This function is called when the reset button is pressed.
        In this example the core.World takes care of all necessary resetting
        by putting everything back in the position it was in when loaded.

        In more complicated scripts, e.g. scripts that modify or create USD properties
        or attributes at runtime, the user will need to implement necessary resetting
        behavior to ensure their script runs deterministically.
        """
        # Start the script over by recreating the generator.
        self._script_generator = self.my_script()

    """
    The following two functions demonstrate the mechanics of running code in a script-like way
    from a UI-based extension.  This takes advantage of Python's yield/generator framework.  

    The update() function is tied to a physics subscription, which means that it will be called
    one time on every physics step (usually 60 frames per second).  Each time it is called, it
    queries the script generator using next().  This makes the script generator execute until it hits
    a yield().  In this case, no value need be yielded.  This behavior can be nested into subroutines
    using the "yield from" keywords.
    """

    def update(self, step: float):
        try:
            result = next(self._script_generator)
        except StopIteration:
            return True

    def my_script(self):
        translation_target, orientation_target = self._target.get_world_pose()

        yield from self.close_gripper_franka(self._articulation)

        # Notice that subroutines can still use return statements to exit.  goto_position() returns a boolean to indicate success.
        success = yield from self.goto_position(
            translation_target, orientation_target, self._articulation, self._rmpflow, timeout=200
        )

        if not success:
            print("Could not reach target position")
            return

        yield from self.open_gripper_franka(self._articulation)

        # Visualize the new target.
        lower_translation_target = np.array([0.4, 0, 0.04])
        self._target.set_world_pose(lower_translation_target, orientation_target)

        success = yield from self.goto_position(
            lower_translation_target, orientation_target, self._articulation, self._rmpflow, timeout=250
        )

        yield from self.close_gripper_franka(self._articulation, close_position=np.array([0.02, 0.02]), atol=0.006)

        high_translation_target = np.array([0.4, 0, 0.4])
        self._target.set_world_pose(high_translation_target, orientation_target)

        success = yield from self.goto_position(
            high_translation_target, orientation_target, self._articulation, self._rmpflow, timeout=200
        )

        next_translation_target = np.array([0.4, 0.4, 0.4])
        self._target.set_world_pose(next_translation_target, orientation_target)

        success = yield from self.goto_position(
            next_translation_target, orientation_target, self._articulation, self._rmpflow, timeout=200
        )

        next_translation_target = np.array([0.4, 0.4, 0.25])
        self._target.set_world_pose(next_translation_target, orientation_target)

        success = yield from self.goto_position(
            next_translation_target, orientation_target, self._articulation, self._rmpflow, timeout=200
        )

        yield from self.open_gripper_franka(self._articulation)

    ################################### Functions

    def goto_position(
        self,
        translation_target,
        orientation_target,
        articulation,
        rmpflow,
        translation_thresh=0.01,
        orientation_thresh=0.1,
        timeout=500,
    ):
        """
        Use RMPflow to move a robot Articulation to a desired task-space position.
        Exit upon timeout or when end effector comes within the provided threshholds of the target pose.
        """

        articulation_motion_policy = ArticulationMotionPolicy(articulation, rmpflow, 1 / 60)
        rmpflow.set_end_effector_target(translation_target, orientation_target)

        for i in range(timeout):
            ee_trans, ee_rot = rmpflow.get_end_effector_pose(
                articulation_motion_policy.get_active_joints_subset().get_joint_positions()
            )

            trans_dist = distance_metrics.weighted_translational_distance(ee_trans, translation_target)
            rotation_target = quats_to_rot_matrices(orientation_target)
            rot_dist = distance_metrics.rotational_distance_angle(ee_rot, rotation_target)

            done = trans_dist < translation_thresh and rot_dist < orientation_thresh

            if done:
                return True

            rmpflow.update_world()
            action = articulation_motion_policy.get_next_articulation_action(1 / 60)
            articulation.apply_action(action)

            # If not done on this frame, yield() to pause execution of this function until
            # the next frame.
            yield ()

        return False

    def open_gripper_franka(self, articulation):
        open_gripper_action = ArticulationAction(np.array([0.04, 0.04]), joint_indices=np.array([7, 8]))
        articulation.apply_action(open_gripper_action)

        # Check in once a frame until the gripper has been successfully opened.
        while not np.allclose(articulation.get_joint_positions()[7:], np.array([0.04, 0.04]), atol=0.001):
            yield ()

        return True

    def close_gripper_franka(self, articulation, close_position=np.array([0, 0]), atol=0.001):
        # To close around the cube, different values are passed in for close_position and atol
        open_gripper_action = ArticulationAction(np.array(close_position), joint_indices=np.array([7, 8]))
        articulation.apply_action(open_gripper_action)

        # Check in once a frame until the gripper has been successfully closed.
        while not np.allclose(articulation.get_joint_positions()[7:], np.array(close_position), atol=atol):
            yield ()

        return True
