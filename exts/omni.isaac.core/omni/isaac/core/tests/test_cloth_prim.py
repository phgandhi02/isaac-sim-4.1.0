# Copyright (c) 2021-2023, NVIDIA CORPORATION. All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto. Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.
#

import omni.kit.test
import torch
from omni.isaac.core import World
from omni.isaac.core.materials.particle_material import ParticleMaterial
from omni.isaac.core.prims.soft.cloth_prim import ClothPrim
from omni.isaac.core.prims.soft.particle_system import ParticleSystem
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
class TestClothPrim(omni.kit.test.AsyncTestCase, TestProperties):
    async def setUp(self):
        World.clear_instance()
        await create_new_stage_async()
        self.my_world = World(backend="torch", device="cuda")
        await self.my_world.initialize_simulation_context_async()

    async def tearDown(self):
        self.my_world.clear_instance()
        await update_stage_async()

    async def test_cloth_prim(self):
        await update_stage_async()
        self.stage = omni.usd.get_context().get_stage()
        env_path = "/World/Env"
        env = UsdGeom.Xform.Define(self.stage, env_path)
        # set up the geometry
        cloth_path = env.GetPrim().GetPath().AppendChild("cloth")
        self.plane_mesh = UsdGeom.Mesh.Define(self.stage, cloth_path)
        tri_points, tri_indices = deformableUtils.create_triangle_mesh_square(dimx=10, dimy=10, scale=1.0)
        self.plane_mesh.GetPointsAttr().Set(tri_points)
        self.plane_mesh.GetFaceVertexIndicesAttr().Set(tri_indices)
        self.plane_mesh.GetFaceVertexCountsAttr().Set([3] * (len(tri_indices) // 3))
        physicsUtils.setup_transform_as_scale_orient_translate(self.plane_mesh)
        physicsUtils.set_or_add_translate_op(self.plane_mesh, Gf.Vec3f(0.0, 0.0, 2.0))
        physicsUtils.set_or_add_orient_op(self.plane_mesh, Gf.Rotation(Gf.Vec3d([1, 0, 0]), 10.0).GetQuat())
        particle_system_path = str(env.GetPrim().GetPath().AppendChild("particleSystem"))
        particle_material_path = str(env.GetPrim().GetPath().AppendChild("particleMaterial"))

        self.particle_material = ParticleMaterial(prim_path=particle_material_path, drag=0.1, lift=0.3, friction=0.6)
        radius = 0.5 * (0.6 / 5.0)
        restOffset = radius
        contactOffset = restOffset * 1.5
        self.particle_system = ParticleSystem(
            prim_path=particle_system_path,
            simulation_owner=self.my_world.get_physics_context().prim_path,
            rest_offset=restOffset,
            contact_offset=contactOffset,
            solid_rest_offset=restOffset,
            fluid_rest_offset=restOffset,
            particle_contact_offset=contactOffset,
        )
        self.particle_system.set_simulation_owner(self.my_world.get_physics_context().prim_path)

        self.cloth = ClothPrim(
            prim_path=str(cloth_path), particle_system=self.particle_system, particle_material=self.particle_material
        )
        self.my_world.scene.add(self.cloth)
        await self.my_world.reset_async(soft=False)

        input_1 = torch.rand(self.cloth._cloth_prim_view.max_springs_per_cloth).cuda()
        input_2 = torch.rand(self.cloth._cloth_prim_view.max_springs_per_cloth).cuda()

        await self.my_world.stop_async()
        await self.bool_prop_test(
            self.cloth.get_self_collision_filter, self.cloth.set_self_collision_filter, is_stopped=True
        )
        await self.bool_prop_test(self.cloth.get_self_collision, self.cloth.set_self_collision, is_stopped=True)
        await self.int_prop_test(self.cloth.get_particle_group, self.cloth.set_particle_group, is_stopped=True)
        # the mesh doesn't seem to be watertight? setting any nonzero pressure messes up the applied schema due to this
        await self.scalar_prop_test(self.cloth.get_pressure, self.cloth.set_pressure, set_value=0.0, is_stopped=True)
        await self.scalar_prop_test(
            self.cloth.get_cloth_stretch_stiffness, self.cloth.set_cloth_stretch_stiffness, is_stopped=True
        )
        await self.scalar_prop_test(
            self.cloth.get_cloth_bend_stiffness, self.cloth.set_cloth_bend_stiffness, is_stopped=True
        )
        await self.scalar_prop_test(
            self.cloth.get_cloth_shear_stiffness, self.cloth.set_cloth_shear_stiffness, is_stopped=True
        )
        await self.scalar_prop_test(self.cloth.get_cloth_damping, self.cloth.set_cloth_damping, is_stopped=True)
        await self.vector_prop_test(
            self.cloth.get_spring_damping,
            self.cloth.set_spring_damping,
            set_value_1=input_1,
            set_value_2=input_2,
            is_stopped=True,
        )
        await self.vector_prop_test(
            self.cloth.get_stretch_stiffness,
            self.cloth.set_stretch_stiffness,
            set_value_1=input_1,
            set_value_2=input_2,
            is_stopped=True,
        )

        await self.my_world.play_async()
        await self.bool_prop_test(
            self.cloth.get_self_collision_filter, self.cloth.set_self_collision_filter, is_stopped=False
        )
        await self.bool_prop_test(self.cloth.get_self_collision, self.cloth.set_self_collision, is_stopped=False)
        await self.int_prop_test(self.cloth.get_particle_group, self.cloth.set_particle_group, is_stopped=False)
        await self.scalar_prop_test(self.cloth.get_pressure, self.cloth.set_pressure, set_value=0.0, is_stopped=False)
        await self.scalar_prop_test(
            self.cloth.get_cloth_stretch_stiffness, self.cloth.set_cloth_stretch_stiffness, is_stopped=False
        )
        await self.scalar_prop_test(
            self.cloth.get_cloth_bend_stiffness, self.cloth.set_cloth_bend_stiffness, is_stopped=False
        )
        await self.scalar_prop_test(
            self.cloth.get_cloth_shear_stiffness, self.cloth.set_cloth_shear_stiffness, is_stopped=False
        )
        await self.scalar_prop_test(self.cloth.get_cloth_damping, self.cloth.set_cloth_damping, is_stopped=False)
        await self.vector_prop_test(
            self.cloth.get_spring_damping,
            self.cloth.set_spring_damping,
            set_value_1=input_1,
            set_value_2=input_2,
            is_stopped=False,
        )
        await self.vector_prop_test(
            self.cloth.get_stretch_stiffness,
            self.cloth.set_stretch_stiffness,
            set_value_1=input_1,
            set_value_2=input_2,
            is_stopped=False,
        )
