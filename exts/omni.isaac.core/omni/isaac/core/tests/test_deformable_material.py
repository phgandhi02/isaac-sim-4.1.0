# Copyright (c) 2023-2024, NVIDIA CORPORATION. All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto. Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto.  Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.
#

import omni.kit.test
from omni.isaac.core import World
from omni.isaac.core.materials.deformable_material import DeformableMaterial
from omni.isaac.core.tests.common import TestProperties

# NOTE:
#   omni.kit.test - std python's unittest module with additional wrapping to add support for async/await tests
#   For most things refer to unittest docs: https://docs.python.org/3/library/unittest.html
from omni.isaac.core.utils.prims import define_prim
from omni.isaac.core.utils.stage import create_new_stage_async, update_stage_async

# Import extension python module we are testing with absolute import path, as if we are external user (other extension)
from pxr import Gf, Usd, UsdGeom


# Having a test class derived from omni.kit.test.AsyncTestCase declared on the root of module will make it auto-discoverable by omni.kit.test
class TestDeformableMaterial(omni.kit.test.AsyncTestCase, TestProperties):
    async def setUp(self):
        World.clear_instance()
        await create_new_stage_async()
        self.my_world = World(backend="torch")  # , device="cuda")
        await self.my_world.initialize_simulation_context_async()
        self._test_cfg = dict()
        pass

    async def tearDown(self):
        World.clear_instance()
        await update_stage_async()
        pass

    async def test_deformable_prim(self):
        await update_stage_async()
        self.stage = omni.usd.get_context().get_stage()
        self.deformable_material = DeformableMaterial(
            prim_path="/deformableMaterial",
            dynamic_friction=0.5,
            youngs_modulus=5e4,
            poissons_ratio=0.4,
            damping_scale=0.1,
            elasticity_damping=0.1,
        )
        self.my_world.scene.add(self.deformable_material)
        await self.my_world.reset_async(soft=False)
        await self.my_world.stop_async()

        for timeline in [True, False]:
            await self.scalar_prop_test(
                self.deformable_material.get_dynamic_friction,
                self.deformable_material.set_dynamic_friction,
                is_stopped=timeline,
            )
            await self.scalar_prop_test(
                self.deformable_material.get_youngs_modululs,
                self.deformable_material.set_youngs_modululs,
                is_stopped=timeline,
            )
            await self.scalar_prop_test(
                self.deformable_material.get_poissons_ratio,
                self.deformable_material.set_poissons_ratio,
                is_stopped=timeline,
            )
            await self.scalar_prop_test(
                self.deformable_material.get_damping_scale,
                self.deformable_material.set_damping_scale,
                is_stopped=timeline,
            )
            await self.scalar_prop_test(
                self.deformable_material.get_elasticity_damping,
                self.deformable_material.set_elasticity_damping,
                is_stopped=timeline,
            )

            if not self.my_world.is_playing():
                await self.my_world.play_async()

        self.my_world.clear_instance()
