# Copyright (c) 2021-2023, NVIDIA CORPORATION. All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto. Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.
#

import asyncio

# NOTE:
#   omni.kit.test - std python's unittest module with additional wrapping to add support for async/await tests
#   For most things refer to unittest docs: https://docs.python.org/3/library/unittest.html
import omni.kit.test
import torch
from omni.isaac.core import World

# Import extension python module we are testing with absolute import path, as if we are external user (other extension)
from omni.isaac.core.articulations import Articulation
from omni.isaac.core.utils.prims import get_prim_at_path
from omni.isaac.core.utils.stage import (
    add_reference_to_stage,
    create_new_stage_async,
    get_current_stage,
    update_stage_async,
)
from omni.isaac.core.utils.types import ArticulationAction
from omni.isaac.nucleus import get_assets_root_path_async
from pxr import PhysxSchema, UsdPhysics


# Having a test class derived from omni.kit.test.AsyncTestCase declared on the root of module will make it auto-discoverable by omni.kit.test
class TestArticulation(omni.kit.test.AsyncTestCase):
    async def setUp(self, device="cpu"):
        World.clear_instance()
        await create_new_stage_async()
        self._my_world = World(stage_units_in_meters=1.0, backend="torch", device=device)
        await self._my_world.initialize_simulation_context_async()
        self._my_world.scene.add_default_ground_plane()
        pass

    async def tearDown(self):
        while omni.usd.get_context().get_stage_loading_status()[2] > 0:
            print("tearDown, assets still loading, waiting to finish...")
            await asyncio.sleep(1.0)
        await update_stage_async()
        World.clear_instance()
        pass

    async def test_get_applied_action(self, add_view_to_scene=True):
        assets_root_path = await get_assets_root_path_async()
        asset_path = assets_root_path + "/Isaac/Robots/Franka/franka_alt_fingers.usd"
        add_reference_to_stage(usd_path=asset_path, prim_path="/World/Franka")
        franka = self._my_world.scene.add(Articulation(prim_path="/World/Franka", name="franka"))
        await self._my_world.reset_async()
        franka.get_applied_action()
        await self._my_world.stop_async()
        self.assertTrue(franka.get_applied_action() is None)

    async def test_apply_partial_articulation(self, add_view_to_scene=True):
        World.clear_instance()
        await create_new_stage_async()
        self._my_world = World(stage_units_in_meters=1.0, backend="numpy", device="cpu")
        await self._my_world.initialize_simulation_context_async()
        self._my_world.scene.add_default_ground_plane()
        assets_root_path = await get_assets_root_path_async()
        asset_path = assets_root_path + "/Isaac/Robots/Franka/franka_alt_fingers.usd"
        add_reference_to_stage(usd_path=asset_path, prim_path="/World/Franka")
        my_franka = self._my_world.scene.add(Articulation(prim_path="/World/Franka", name="franka"))
        await self._my_world.reset_async()
        joint_positions = my_franka.get_joint_positions()
        joint_targets = [None] * 9
        joint_targets[7:] = joint_positions[7:]
        my_franka.apply_action(ArticulationAction(joint_positions=joint_targets))
        await update_stage_async()
        await update_stage_async()
        await self._my_world.stop_async()

    async def test_dof_efforts(self, add_view_to_scene=True):
        assets_root_path = await get_assets_root_path_async()
        self._my_world.set_simulation_dt(0.001)
        asset_path = assets_root_path + "/Isaac/Robots/Cartpole/cartpole.usd"
        self.stage = get_current_stage()
        add_reference_to_stage(usd_path=asset_path, prim_path="/World/cartpole")
        if self._my_world.get_physics_context()._physx_scene_api.GetSolverTypeAttr().Get() == "TGS":
            articulation_api = PhysxSchema.PhysxArticulationAPI.Apply(self.stage.GetPrimAtPath("/World/cartpole"))
            articulation_api.CreateSolverVelocityIterationCountAttr(0)
            articulation_api.CreateSolverPositionIterationCountAttr(8)
        robot = self._my_world.scene.add(Articulation(prim_path="/World/cartpole", name="franka"))
        await self._my_world.reset_async()
        efforts = torch.ones((robot.num_dof), device="cpu") * 1000
        robot.set_joint_efforts(efforts)
        self._my_world.step_async()
        await update_stage_async()
        current_efforts = robot.get_measured_joint_efforts()
        await self._my_world.stop_async()
        # print(efforts, current_efforts)
        self.assertTrue(torch.isclose(current_efforts, efforts, atol=1e-1).all())

    async def test_joint_forces(self, add_view_to_scene=True):
        assets_root_path = await get_assets_root_path_async()
        asset_path = assets_root_path + "/Isaac/Robots/Franka/franka_alt_fingers.usd"
        add_reference_to_stage(usd_path=asset_path, prim_path="/World/Franka")
        franka = self._my_world.scene.add(Articulation(prim_path="/World/Franka", name="franka"))
        await self._my_world.reset_async()
        efforts = torch.ones((franka.num_dof), device="cuda") * 100
        franka.set_joint_efforts(efforts)
        self._my_world.step_async()
        await update_stage_async()
        forces = franka.get_measured_joint_forces()
        await self._my_world.stop_async()
        self.assertEqual(forces.shape, torch.Size([franka._articulation_view.num_bodies, 6]))

    async def test_articulation_joint_signs(self):
        assets_root_path = await get_assets_root_path_async()
        asset_path = assets_root_path + "/Isaac/Robots/Simple/articulation_3_joints.usd"
        add_reference_to_stage(usd_path=asset_path, prim_path="/World/Articulation")
        test_art = self._my_world.scene.add(Articulation(prim_path="/World/Articulation", name="test_art"))
        await self._my_world.reset_async()
        # test_position = torch.Tensor([0.14, 0.5, -0.14])
        test_position = test_art.get_joint_positions()
        self._my_world.step_async()
        await update_stage_async()

        center_rev_drive = UsdPhysics.DriveAPI.Get(
            get_prim_at_path("/World/Articulation/Arm/CenterRevoluteJoint"), "angular"
        )
        center_rev_position = center_rev_drive.GetTargetPositionAttr().Get()

        distal_rev_drive = UsdPhysics.DriveAPI.Get(
            get_prim_at_path("/World/Articulation/DistalPivot/DistalRevoluteJoint"), "angular"
        )
        distal_rev_position = distal_rev_drive.GetTargetPositionAttr().Get()

        prismatic_drive = UsdPhysics.DriveAPI.Get(
            get_prim_at_path("/World/Articulation/Slider/PrismaticJoint"), "linear"
        )
        prismatic_position = prismatic_drive.GetTargetPositionAttr().Get()

        self.assertAlmostEqual(torch.rad2deg(test_position[0]), center_rev_position, delta=0.1)
        self.assertAlmostEqual(test_position[1], prismatic_position, delta=0.1)
        self.assertAlmostEqual(torch.rad2deg(test_position[2]), distal_rev_position, delta=0.1)
