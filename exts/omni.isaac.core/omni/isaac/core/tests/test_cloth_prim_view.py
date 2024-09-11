# Copyright (c) 2021-2023, NVIDIA CORPORATION. All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto. Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.
#

import asyncio

import numpy as np
import omni.kit.test
import torch
from omni.isaac.core import World
from omni.isaac.core.materials.particle_material import ParticleMaterial
from omni.isaac.core.prims.soft.cloth_prim import ClothPrim
from omni.isaac.core.prims.soft.cloth_prim_view import ClothPrimView
from omni.isaac.core.prims.soft.particle_system import ParticleSystem
from omni.isaac.core.utils.stage import create_new_stage_async, update_stage_async

# NOTE:
#   omni.kit.test - std python's unittest module with additional wrapping to add support for async/await tests
#   For most things refer to unittest docs: https://docs.python.org/3/library/unittest.html
from omni.isaac.core.utils.types import DynamicsViewState
from omni.physx.scripts import deformableUtils, physicsUtils
from pxr import Gf, Usd, UsdGeom


# Having a test class derived from omni.kit.test.AsyncTestCase declared on the root of module will make it auto-discoverable by omni.kit.test
class TestClothPrimView(omni.kit.test.AsyncTestCase):
    async def setUp(self):
        World.clear_instance()
        await create_new_stage_async()
        self.my_world = World(backend="torch", device="cuda")
        await self.my_world.initialize_simulation_context_async()
        self._test_cfg = dict()

    async def tearDown(self):
        self.my_world.clear_instance()
        await update_stage_async()

    async def test_cloth_prim_view_gpu_pipeline(self):
        self.isclose = torch.isclose
        self._array_container = lambda x: torch.tensor(x, device=self._device, dtype=torch.float32)
        await update_stage_async()
        self.stage = omni.usd.get_context().get_stage()
        await self._runner()

    async def _runner(self):

        await update_stage_async()
        self.num_envs = 10
        self.dimx = 5
        self.dimy = 5

        for i in range(self.num_envs):
            env_path = "/World/Env" + str(i)
            env = UsdGeom.Xform.Define(self.stage, env_path)
            # set up the geometry
            cloth_path = env.GetPrim().GetPath().AppendChild("cloth").pathString
            self.plane_mesh = UsdGeom.Mesh.Define(self.stage, cloth_path)
            tri_points, tri_indices = deformableUtils.create_triangle_mesh_square(
                dimx=self.dimx, dimy=self.dimy, scale=1.0
            )
            self.plane_mesh.GetPointsAttr().Set(tri_points)
            self.plane_mesh.GetFaceVertexIndicesAttr().Set(tri_indices)
            self.plane_mesh.GetFaceVertexCountsAttr().Set([3] * (len(tri_indices) // 3))
            physicsUtils.setup_transform_as_scale_orient_translate(self.plane_mesh)
            physicsUtils.set_or_add_translate_op(self.plane_mesh, Gf.Vec3f(i * 2, 0.0, 2.0))
            physicsUtils.set_or_add_orient_op(self.plane_mesh, Gf.Rotation(Gf.Vec3d([1, 0, 0]), 15 * i).GetQuat())
            particle_system_path = str(env.GetPrim().GetPath().AppendChild("particleSystem"))
            particle_material_path = str(env.GetPrim().GetPath().AppendChild("particleMaterial"))
            particle_material = ParticleMaterial(prim_path=particle_material_path, drag=0.1, lift=0.3, friction=0.6)
            radius = 0.5 * (0.6 / 5.0)
            restOffset = radius
            contactOffset = restOffset * 1.5
            particle_system = ParticleSystem(
                prim_path=particle_system_path,
                simulation_owner=self.my_world.get_physics_context().prim_path,
                rest_offset=restOffset,
                contact_offset=contactOffset,
                solid_rest_offset=restOffset,
                fluid_rest_offset=restOffset,
                particle_contact_offset=contactOffset,
            )
            cloth = ClothPrim(
                prim_path=cloth_path, particle_system=particle_system, particle_material=particle_material
            )

        # create a view to deal with all the cloths
        self.cloth_view = ClothPrimView(prim_paths_expr="/World/Env*/cloth", name="clothView1")
        self.my_world.scene.add(self.cloth_view)
        await update_stage_async()

        for indexed in [False, True]:
            self._test_cfg["indexed"] = indexed
            print(self._test_cfg)
            await self.position_test()
            await self.velocity_test()
            await self.spring_stiffness_test()
            await self.spring_damping_test()

        await self.my_world.stop_async()
        self.my_world.clear_instance()

    async def position_test(self):
        await self.my_world.reset_async()
        indices = [1, 2] if self._test_cfg["indexed"] else None
        prev_values = self.cloth_view.get_world_positions()
        new_values = prev_values[indices]
        self.cloth_view.set_world_positions(new_values, indices)
        cur_values = self.cloth_view.get_world_positions(indices)
        self.assertTrue(self.isclose(new_values, cur_values).all())
        mesh_points = self.plane_mesh.GetPointsAttr().Get()
        expected_shape = torch.Size(
            [len(indices) if self._test_cfg["indexed"] else self.cloth_view.count, len(mesh_points), 3]
        )
        print(expected_shape, cur_values.shape)
        self.assertTrue(cur_values.shape == expected_shape)

    async def velocity_test(self):
        await self.my_world.reset_async()
        indices = [1, 2] if self._test_cfg["indexed"] else None
        prev_values = self.cloth_view.get_velocities()
        new_values = prev_values[indices]
        self.cloth_view.set_velocities(new_values, indices)
        cur_values = self.cloth_view.get_velocities(indices)
        self.assertTrue(self.isclose(new_values, cur_values).all())
        expected_shape = torch.Size(
            [
                len(indices) if self._test_cfg["indexed"] else self.cloth_view.count,
                self.cloth_view.max_particles_per_cloth,
                3,
            ]
        )
        self.assertTrue(cur_values.shape == expected_shape)

    async def paticle_masses_test(self):
        await self.my_world.reset_async()
        indices = [1, 2] if self._test_cfg["indexed"] else None
        prev_values = self.cloth_view.get_particle_masses()
        new_values = prev_values[indices]
        self.cloth_view.set_particle_masses(new_values, indices)
        cur_values = self.cloth_view.get_particle_masses(indices)
        self.assertTrue(self.isclose(new_values, cur_values).all())
        expected_shape = torch.Size(
            [
                len(indices) if self._test_cfg["indexed"] else self.cloth_view.count,
                self.cloth_view.max_particles_per_cloth,
            ]
        )
        print(expected_shape, cur_values.shape)
        self.assertTrue(cur_values.shape == expected_shape)

    async def spring_stiffness_test(self):
        await self.my_world.reset_async()
        indices = [1, 2] if self._test_cfg["indexed"] else None
        prev_values = self.cloth_view.get_stretch_stiffnesses()
        new_values = prev_values[indices]
        self.cloth_view.set_stretch_stiffnesses(new_values, indices)
        cur_values = self.cloth_view.get_stretch_stiffnesses(indices)
        self.assertTrue(self.isclose(new_values, cur_values).all())
        expected_shape = torch.Size(
            [
                len(indices) if self._test_cfg["indexed"] else self.cloth_view.count,
                self.cloth_view.max_springs_per_cloth,
            ]
        )
        print(expected_shape, cur_values.shape)
        self.assertTrue(cur_values.shape == expected_shape)

    async def spring_damping_test(self):
        await self.my_world.reset_async()
        indices = [1, 2] if self._test_cfg["indexed"] else None
        prev_values = self.cloth_view.get_spring_dampings()
        new_values = prev_values[indices]
        self.cloth_view.set_spring_dampings(new_values, indices)
        cur_values = self.cloth_view.get_spring_dampings(indices)
        self.assertTrue(self.isclose(new_values, cur_values).all())
        expected_shape = torch.Size(
            [
                len(indices) if self._test_cfg["indexed"] else self.cloth_view.count,
                self.cloth_view.max_springs_per_cloth,
            ]
        )
        self.assertTrue(cur_values.shape == expected_shape)
        await self.my_world.stop_async()
