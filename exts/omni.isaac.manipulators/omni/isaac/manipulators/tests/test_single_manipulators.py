# Copyright (c) 2021-2024, NVIDIA CORPORATION. All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto. Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.
#

import numpy as np
import omni.kit.test
from omni.isaac.core import World
from omni.isaac.core.utils.stage import add_reference_to_stage, create_new_stage_async
from omni.isaac.manipulators import SingleManipulator
from omni.isaac.manipulators.grippers import SurfaceGripper
from omni.isaac.nucleus import get_assets_root_path_async


class TestSingleManipulators(omni.kit.test.AsyncTestCase):
    async def setUp(self):
        World.clear_instance()
        await create_new_stage_async()
        self._my_world = World()
        await self._my_world.initialize_simulation_context_async()
        self._assets_root_path = await get_assets_root_path_async()
        self._my_world.scene.add_default_ground_plane()

    async def test_single_manipulators(self):
        asset_path = self._assets_root_path + "/Isaac/Robots/UniversalRobots/ur10/ur10.usd"
        add_reference_to_stage(usd_path=asset_path, prim_path="/World/UR10")
        gripper_usd = self._assets_root_path + "/Isaac/Robots/UR10/Props/short_gripper.usd"
        add_reference_to_stage(usd_path=gripper_usd, prim_path="/World/UR10/ee_link")
        gripper = SurfaceGripper(end_effector_prim_path="/World/UR10/ee_link", translate=0.1611, direction="x")
        ur10 = self._my_world.scene.add(
            SingleManipulator(
                prim_path="/World/UR10", name="my_ur10", end_effector_prim_path="/World/UR10/ee_link", gripper=gripper
            )
        )
        ur10_2 = self._my_world.scene.add(
            SingleManipulator(
                prim_path="/World/UR10", name="my_ur10_2", end_effector_prim_name="ee_link", gripper=gripper
            )
        )
        ur10.set_joints_default_state(
            positions=np.array([-np.pi / 2, -np.pi / 2, -np.pi / 2, -np.pi / 2, np.pi / 2, 0])
        )
        ur10.gripper.set_default_state(opened=True)
        await self._my_world.reset_async()
        self.assertFalse(ur10.gripper is None)
        pass
