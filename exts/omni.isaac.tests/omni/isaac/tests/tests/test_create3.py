# Copyright (c) 2018-2023, NVIDIA CORPORATION. All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto. Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.
#

import math
import time

import carb
import carb.tokens
import numpy as np
import omni.graph.core as og

# NOTE:
#   omni.kit.test - std python's unittest module with additional wrapping to add suport for async/await tests
#   For most things refer to unittest docs: https://docs.python.org/3/library/unittest.html
import omni.kit.test
from omni.isaac.core import World
from omni.isaac.core.articulations.articulation import Articulation
from omni.isaac.core.utils.nucleus import get_assets_root_path
from omni.isaac.core.utils.prims import delete_prim
from omni.isaac.core.utils.stage import open_stage_async

from .robot_helpers import init_robot_sim, setup_robot_og


# Class derived from omni.kit.test.AsyncTestCase declared on the root of module will make it auto-discoverable by omni.kit.test
class TestCreate3(omni.kit.test.AsyncTestCase):
    # Before running each test
    async def setUp(self):

        self._assets_root_path = get_assets_root_path()
        if self._assets_root_path is None:
            carb.log_error("Could not find Isaac Sim assets folder")
            return

        # add in create3 (from nucleus)
        self.usd_path = self._assets_root_path + "/Isaac/Robots/iRobot/create_3.usd"
        (result, error) = await open_stage_async(self.usd_path)
        # Make sure the stage loaded
        self.assertTrue(result)

        await omni.kit.app.get_app().next_update_async()

        self.my_world = World(stage_units_in_meters=1.0)
        await self.my_world.initialize_simulation_context_async()

        # setup omnigraph
        self.graph_path = "/ActionGraph"
        graph, self.odom_node = setup_robot_og(
            self.graph_path, "left_wheel_joint", "right_wheel_joint", "/create3", 0.3575, 0.233
        )
        pass

    # After running each test
    async def tearDown(self):
        self.my_world.stop()
        self.my_world.clear_instance()
        await omni.kit.app.get_app().next_update_async()
        # In some cases the test will end before the asset is loaded, in this case wait for assets to load
        while omni.usd.get_context().get_stage_loading_status()[2] > 0:
            await omni.kit.app.get_app().next_update_async()

        pass

    # Actual test, notice it is "async" function, so "await" can be used if needed
    async def test_loading(self):

        delete_prim("/ActionGraph")
        # Start Simulation and wait
        self.my_world.play()
        await omni.kit.app.get_app().next_update_async()

        # get the create3
        self.ar = Articulation("/create3")
        self.ar._articulation_view.initialize()
        self.starting_pos, _ = self.ar.get_world_pose()

        left_wheel_joint_idx = self.ar._articulation_view.get_dof_index("left_wheel_joint")
        right_wheel_joint_idx = self.ar._articulation_view.get_dof_index("right_wheel_joint")
        self.ar._articulation_view.set_joint_velocity_targets(
            velocities=np.array([1.0, 1.0]), joint_indices=[left_wheel_joint_idx, right_wheel_joint_idx]
        )

        for i in range(60):
            await omni.kit.app.get_app().next_update_async()

        self.current_pos, _ = self.ar.get_world_pose()

        delta = np.linalg.norm(self.current_pos - self.starting_pos)
        self.assertTrue(delta > 0.02)

        pass

    # Building up speed tests
    async def test_accel(self):

        odom_velocity = og.Controller.attribute("outputs:linearVelocity", self.odom_node)
        odom_ang_vel = og.Controller.attribute("outputs:angularVelocity", self.odom_node)

        # Start sim and wait
        self.my_world.play()
        await omni.kit.app.get_app().next_update_async()

        await init_robot_sim("/create3")

        for x in range(1, 5):
            forward_velocity = x * 0.015
            og.Controller.attribute(self.graph_path + "/DifferentialController.inputs:linearVelocity").set(
                forward_velocity
            )
            for i in range(30):
                await omni.kit.app.get_app().next_update_async()
            if og.DataView.get(odom_ang_vel)[2] > 0.8:
                print("spinning out of control, linear velocity: " + str(forward_velocity))
                self.my_world.stop()
            else:
                self.assertAlmostEqual(og.DataView.get(odom_velocity)[0], forward_velocity, delta=1e-1)
            await omni.kit.app.get_app().next_update_async()

        self.my_world.stop()

        pass

    # braking from different init speeds
    async def test_brake(self):

        odom_velocity = og.Controller.attribute("outputs:linearVelocity", self.odom_node)
        odom_ang_vel = og.Controller.attribute("outputs:angularVelocity", self.odom_node)

        # Start Simulation and wait
        self.my_world.play()
        await omni.kit.app.get_app().next_update_async()

        await init_robot_sim("/create3")
        for x in range(1, 5):
            self.my_world.play()
            await omni.kit.app.get_app().next_update_async()
            forward_velocity = x * 0.15
            angular_velocity = x * 0.15
            og.Controller.attribute(self.graph_path + "/DifferentialController.inputs:linearVelocity").set(
                forward_velocity
            )
            og.Controller.attribute(self.graph_path + "/DifferentialController.inputs:angularVelocity").set(
                angular_velocity
            )
            for j in range(30):
                await omni.kit.app.get_app().next_update_async()
            og.Controller.attribute(self.graph_path + "/DifferentialController.inputs:linearVelocity").set(0.0)
            og.Controller.attribute(self.graph_path + "/DifferentialController.inputs:angularVelocity").set(0.0)
            for j in range(10):
                await omni.kit.app.get_app().next_update_async()
            self.assertAlmostEqual(og.DataView.get(odom_velocity)[0], 0.0, delta=5e-1)
            self.assertAlmostEqual(og.DataView.get(odom_ang_vel)[2], 0.0, delta=5e-1)

            self.my_world.stop()
            await omni.kit.app.get_app().next_update_async()
        pass

    async def test_spin(self):
        odom_ang_vel = og.Controller.attribute("outputs:angularVelocity", self.odom_node)

        for x in range(1, 6):
            # Start Simulation and wait
            self.my_world.play()
            await omni.kit.app.get_app().next_update_async()

            await init_robot_sim("/create3")

            angular_velocity = 0.01 * x
            og.Controller.attribute(self.graph_path + "/DifferentialController.inputs:angularVelocity").set(
                angular_velocity
            )

            # wait until const velocity reached
            for i in range(30):
                await omni.kit.app.get_app().next_update_async()

            curr_ang_vel = float(og.DataView.get(odom_ang_vel)[2])
            self.assertAlmostEqual(curr_ang_vel, angular_velocity, delta=2e-1)

            self.my_world.stop()

        pass

    # go in circle
    async def test_circle(self):
        odom_velocity = og.Controller.attribute("outputs:linearVelocity", self.odom_node)
        odom_ang_vel = og.Controller.attribute("outputs:angularVelocity", self.odom_node)
        odom_position = og.Controller.attribute("outputs:position", self.odom_node)

        # Start Simulation and wait
        self.my_world.play()
        await omni.kit.app.get_app().next_update_async()

        await init_robot_sim("/create3")
        forward_velocity = -1e-5
        angular_velocity = 12.87
        og.Controller.attribute(self.graph_path + "/DifferentialController.inputs:linearVelocity").set(forward_velocity)
        og.Controller.attribute(self.graph_path + "/DifferentialController.inputs:angularVelocity").set(
            angular_velocity
        )

        for j in range(600):
            await omni.kit.app.get_app().next_update_async()

        self.assertAlmostEqual(og.DataView.get(odom_position)[0], 0, delta=5e-2)
        self.assertAlmostEqual(og.DataView.get(odom_position)[1], 0, delta=5e-2)
        self.assertAlmostEqual(og.DataView.get(odom_velocity)[0], 0, delta=5e-2)
        self.assertTrue(og.DataView.get(odom_ang_vel)[2] > 1.2)

        await omni.kit.app.get_app().next_update_async()

        pass
