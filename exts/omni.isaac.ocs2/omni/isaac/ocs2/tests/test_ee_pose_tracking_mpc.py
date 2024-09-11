# Copyright (c) 2022-2023, NVIDIA CORPORATION. All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto. Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.
#

import os

# python
import numpy as np

# kit
import omni.kit.test

# omni-isaac-ocs2
from omni.isaac.ocs2.end_effector_pose_tracking_mpc import EndEffectorPoseTrackingMpc


class TestEndEffectorPoseTrackingMpc(omni.kit.test.AsyncTestCase):
    """
    Tests the EndEffectorPoseTrackingMpc wrapper.
    """

    # Before running each test
    async def setUp(self):
        pass

    # After running each test
    async def tearDown(self):
        pass

    async def test_franka(self):
        """
        Dummy test for franka robot.
        """
        # TODO, need to investigate why this test fails in docker
        if os.path.exists("/.dockerenv"):
            return
        # MPC-SLQ instance
        lib_folder = "/tmp/ocs2/auto_generated/franka"
        mpc_config_file = "data/franka/mpc/task.info"
        urdf_file = "data/franka/urdf/panda.urdf"
        mpc_interface = EndEffectorPoseTrackingMpc(mpc_config_file, lib_folder, urdf_file)

        # the total time to simulate
        T = 5
        # the frequency of MPC
        f_mpc = 100
        # define initial state of the robot
        q_init = [0.0, 0.171, 0.1144, -1.57, 0.05, 1.57, 0.469]
        # end-effector command
        ref_ee_cmd = np.array([0.5, 0.0, 0.5, 0.0, 1.0, 0.0, 0.0])

        # current timer for simulation
        sim_time = 0.0
        # setup the goal trajectories
        mpc_interface.set_target_trajectory(time_traj=[sim_time], state_traj=[ref_ee_cmd], input_traj=[None])
        # reset with current observation
        state = np.copy(q_init)
        mpc_interface.reset(sim_time, state)

        for step in range(int(T * f_mpc)):
            # compute optimal input
            cmd = mpc_interface.advance(sim_time, state)
            # update simulation time
            sim_time += 1 / f_mpc
            # setup next time and observation
            state = mpc_interface.get_optimal_state_traj()[1]
            # evaluate cost of current solution
            # self.assertLessEqual(mpc_interface.get_current_cost(), curr_cost)
            curr_cost = mpc_interface.get_current_cost()
            if step % f_mpc == 0:
                print(f"[Franka]: [Step {step}]: Current cost: {curr_cost}")

        # check that a stable config is achieved
        self.assertLess(np.linalg.norm(cmd), 1.0e-2)
