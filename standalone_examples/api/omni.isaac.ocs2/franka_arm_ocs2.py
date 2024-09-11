# Copyright (c) 2021-2023, NVIDIA CORPORATION. All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto. Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.
#

# python
import sys

if sys.platform == "win32":
    print("This example is not supported on windows, exiting")
    exit()

"""Launch Omniverse Toolkit first."""

# kit
from isaacsim import SimulationApp

simulation_app = SimulationApp({"headless": False})


"""Rest everything follows."""


import numpy as np
from omni.isaac.core.objects.sphere import VisualSphere
from omni.isaac.core.utils.extensions import enable_extension
from omni.isaac.core.utils.types import ArticulationAction
from omni.isaac.core.utils.viewports import set_camera_view

# isaac-core
from omni.isaac.core.world import World

# isaac-franka
from omni.isaac.franka import Franka

# isaac-ocs2
enable_extension("omni.isaac.ocs2")
from omni.isaac.ocs2.end_effector_pose_tracking_mpc import EndEffectorPoseTrackingMpc

# print settings
np.set_printoptions(formatter={"float_kind": "{:.2f}".format})


def main():
    """Sets the Franka control mode to "velocity" and tests the MPC."""
    # Add MPC
    config = {
        "urdf_path": "data/franka/urdf/panda.urdf",
        "lib_folder": "/tmp/ocs2/auto_generated/franka",
        "mpc_config_path": "data/franka/mpc/task.info",
    }
    mpc_interface = EndEffectorPoseTrackingMpc(config["mpc_config_path"], config["lib_folder"], config["urdf_path"])
    # Receive the number of arm dimensions
    arm_num_dof = mpc_interface.state_dim
    # print info about MPC
    print(mpc_interface)

    # Load kit helper
    my_world = World(stage_units_in_meters=1.0, physics_dt=0.01)
    # Set main camera
    set_camera_view([1.5, 1.5, 1.5], [0.0, 0.0, 0.0])
    # Spawn things into stage
    # -- ground
    my_world.scene.add_default_ground_plane()
    # -- robot
    robot = my_world.scene.add(Franka("/World/Robot"))
    # -- markers
    goal_vis_prim = my_world.scene.add(
        VisualSphere("/World/Vis/ee_goal", name="ee_goal", radius=0.01, color=np.asarray([1.0, 0.0, 0.0]))
    )
    ee_vis_prim = my_world.scene.add(
        VisualSphere("/World/Vis/ee_curr", name="ee_curr", radius=0.01, color=np.asarray([0.0, 0.0, 1.0]))
    )

    # Play the simulator
    my_world.reset()
    # Set control mode
    robot._articulation_view.switch_control_mode("velocity")
    robot.disable_gravity()

    # Now we are ready!
    print("[INFO]: Setup complete...")

    # Define simulation stepping
    dt = 0.01
    sim_time = 0.0
    # Define goals for the arm
    ee_goal_index = 0
    ee_goals = [
        [0.5, 0.5, 0.7, 0.707, 0, 0.707, 0],
        [0.5, -0.4, 0.6, 0.707, 0.707, 0.0, 0.0],
        [0.5, 0, 0.5, 0.0, 1.0, 0.0, 0.0],
    ]
    # Define a goal for the arm
    ee_goal_pose = np.array(ee_goals[ee_goal_index])

    # Obtain measurements
    arm_joint_pos = robot.get_joint_positions()[:arm_num_dof]
    ee_curr_pose = robot.end_effector.get_world_pose()
    ee_curr_pose = np.concatenate((ee_curr_pose[0], ee_curr_pose[1]), axis=0)
    # Update visualization
    goal_vis_prim.set_world_pose(ee_goal_pose[:3], ee_goal_pose[3:])
    ee_vis_prim.set_world_pose(ee_curr_pose[:3], ee_curr_pose[3:])

    # Define target trajectory
    mpc_interface.set_target_trajectory(
        time_traj=[sim_time, sim_time + 2], state_traj=[ee_curr_pose, ee_goal_pose], input_traj=[None, None]
    )
    # Reset the MPC
    mpc_interface.reset(sim_time, arm_joint_pos)

    # Simulate physics
    for count in range(100000):
        # obtain current measurements
        arm_joint_pos = robot.get_joint_positions()[:arm_num_dof]
        # compute arm's optimal control command
        arm_cmd = mpc_interface.advance(sim_time, arm_joint_pos)
        # print mpc cost
        # perform actions
        action = ArticulationAction(joint_velocities=arm_cmd, joint_indices=[range(arm_num_dof)])
        robot.apply_action(action)
        # perform step
        my_world.step()
        # update sim-time
        sim_time += dt
        # obtain new measurements
        ee_curr_pose = robot.end_effector.get_world_pose()
        ee_curr_pose = np.concatenate((ee_curr_pose[0], ee_curr_pose[1]), axis=0)
        # compute the waypoint error
        error = np.linalg.norm(ee_curr_pose[:3] - ee_goal_pose[:3])
        # update visualization
        ee_vis_prim.set_world_pose(ee_curr_pose[:3], ee_curr_pose[3:])
        # get next waypoint
        if error < 0.014:
            # print goal state
            print(
                f"\tMPC cost: { mpc_interface.get_current_cost()}\n",
                f"\tCurrent EE state:\n"
                f"\t\tI_r_IE    : {ee_curr_pose[:3]} \n"
                f"\t\tq_IE      : {ee_curr_pose[3:]} \n"
                f"\tGoal EE state:\n"
                f"\t\tI_r_IE_des: {ee_goals[ee_goal_index][:3]} \n"
                f"\t\tq_IE_des  : {ee_goals[ee_goal_index][3:]} \n"
                "----------------------------------------------",
            )
            # next goal
            ee_goal_index += 1
            if ee_goal_index >= len(ee_goals):
                ee_goal_index = 0
            # Define a goal for the arm
            ee_goal_pose = np.array(ee_goals[ee_goal_index])
            # Update prims
            goal_vis_prim.set_world_pose(ee_goal_pose[:3], ee_goal_pose[3:])
            # Define target trajectory
            mpc_interface.set_target_trajectory(
                time_traj=[sim_time, sim_time + 2], state_traj=[ee_curr_pose, ee_goal_pose], input_traj=[None, None]
            )


if __name__ == "__main__":
    # Run OCS2 example with Franka
    main()
    # Close the simulator
    simulation_app.close()

# EOF
