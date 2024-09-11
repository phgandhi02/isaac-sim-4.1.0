# Copyright (c) 2021-2024, NVIDIA CORPORATION. All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto. Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.
#

import omni.isaac.core.utils.deformable_mesh_utils as DeformableMeshUtils
import omni.kit.test
import torch
from omni.isaac.core import World
from omni.isaac.core.materials.particle_material import ParticleMaterial
from omni.isaac.core.prims.soft.deformable_prim import DeformablePrim
from omni.isaac.core.tests.common import TestProperties

# NOTE:
#   omni.kit.test - std python's unittest module with additional wrapping to add support for async/await tests
#   For most things refer to unittest docs: https://docs.python.org/3/library/unittest.html
from omni.isaac.core.utils.prims import define_prim
from omni.isaac.core.utils.stage import create_new_stage_async, update_stage_async
from omni.physx.scripts import deformableUtils, physicsUtils

# Import extension python module we are testing with absolute import path, as if we are external user (other extension)
from pxr import Gf, Usd, UsdGeom


# Having a test class derived from omni.kit.test.AsyncTestCase declared on the root of module will make it auto-discoverable by omni.kit.test
class TestDeformablePrim(omni.kit.test.AsyncTestCase, TestProperties):
    async def setUp(self):
        World.clear_instance()
        await create_new_stage_async()
        self.my_world = World(backend="torch", device="cuda")
        await self.my_world.initialize_simulation_context_async()

    async def tearDown(self):
        self.my_world.clear_instance()
        await update_stage_async()

    async def test_deformable_prim(self):
        await update_stage_async()
        self.stage = omni.usd.get_context().get_stage()
        env_path = "/World/Env"
        env = UsdGeom.Xform.Define(self.stage, env_path)
        # set up the geometry
        deformable_path = env.GetPrim().GetPath().AppendChild("deformable")
        self.plane_mesh = UsdGeom.Mesh.Define(self.stage, deformable_path)
        tri_points, tri_indices = DeformableMeshUtils.createTriangleMeshCube(8)
        self.plane_mesh.GetPointsAttr().Set(tri_points)
        self.plane_mesh.GetFaceVertexIndicesAttr().Set(tri_indices)
        self.plane_mesh.GetFaceVertexCountsAttr().Set([3] * (len(tri_indices) // 3))
        physicsUtils.setup_transform_as_scale_orient_translate(self.plane_mesh)
        physicsUtils.set_or_add_translate_op(self.plane_mesh, Gf.Vec3f(2, 0.0, 2.0))
        physicsUtils.set_or_add_orient_op(self.plane_mesh, Gf.Rotation(Gf.Vec3d([1, 0, 0]), 15).GetQuat())
        # self.particle_material = ParticleMaterial(prim_path=particle_material_path, drag=0.1, lift=0.3, friction=0.6)
        self.deformable = DeformablePrim(prim_path=str(deformable_path))
        self.my_world.scene.add(self.deformable)
        await self.my_world.reset_async(soft=False)
        await self.my_world.stop_async()

        for timeline in [True, False]:
            await self.int_prop_test(
                self.deformable.get_solver_position_iteration_count,
                self.deformable.set_solver_position_iteration_count,
                is_stopped=timeline,
            )
            await self.bool_prop_test(
                self.deformable.get_self_collision, self.deformable.set_self_collision, is_stopped=timeline
            )
            await self.scalar_prop_test(
                self.deformable.get_self_collision_filter_distance,
                self.deformable.set_self_collision_filter_distance,
                is_stopped=timeline,
            )
            await self.scalar_prop_test(
                self.deformable.get_settling_threshold, self.deformable.set_settling_threshold, is_stopped=timeline
            )
            await self.scalar_prop_test(
                self.deformable.get_sleep_threshold, self.deformable.set_sleep_threshold, is_stopped=timeline
            )
            await self.scalar_prop_test(
                self.deformable.get_sleep_damping, self.deformable.set_sleep_damping, is_stopped=timeline
            )
            await self.scalar_prop_test(
                self.deformable.get_vertex_velocity_damping,
                self.deformable.set_vertex_velocity_damping,
                is_stopped=timeline,
            )

            if not self.my_world.is_playing():
                await self.my_world.play_async()
