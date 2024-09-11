# Copyright (c) 2021-2024, NVIDIA CORPORATION. All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto. Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.
#

import asyncio

import numpy as np

# NOTE:
#   omni.kit.test - std python's unittest module with additional wrapping to add support for async/await tests
#   For most things refer to unittest docs: https://docs.python.org/3/library/unittest.html
import omni.kit.test
import torch
from omni.isaac.core import World
from omni.isaac.core.materials.deformable_material import DeformableMaterial
from omni.isaac.core.materials.deformable_material_view import DeformableMaterialView
from omni.isaac.core.utils.stage import create_new_stage_async, get_current_stage, update_stage_async


# Having a test class derived from omni.kit.test.AsyncTestCase declared on the root of module will make it auto-discoverable by omni.kit.test
class TestDeformableMaterialView(omni.kit.test.AsyncTestCase):
    async def setUp(self):
        World.clear_instance()
        await create_new_stage_async()
        self.my_world = World(backend="torch")  # , device="cuda")
        await self.my_world.initialize_simulation_context_async()
        await update_stage_async()
        self._test_cfg = dict()
        pass

    async def tearDown(self):
        self.my_world.clear_instance()
        await update_stage_async()
        pass

    async def test_deformable_material_view(self):
        self.isclose = torch.isclose
        self._array_container = lambda x: torch.tensor(x, device=self._device, dtype=torch.float32)
        self.stage = get_current_stage()
        await update_stage_async()
        await self._runner()
        pass

    async def _runner(self):
        self.num_envs = 10
        for i in range(self.num_envs):
            self.deformable_material = DeformableMaterial(
                prim_path="/World/deformableMaterial_" + str(i),
                dynamic_friction=0.5,
                youngs_modulus=5e4,
                poissons_ratio=0.4,
                damping_scale=0.1,
                elasticity_damping=0.1,
            )

        # create a view to deal with all the deformables
        self.deformable_material_view = DeformableMaterialView(prim_paths_expr="/World/deformableMaterial_*")
        self.my_world.scene.add(self.deformable_material_view)
        await update_stage_async()

        for indexed in [False, True]:
            self._test_cfg["indexed"] = indexed
            print(self._test_cfg)
            await self.dynamic_friction_test()
            await self.poissons_ratio_test()
            await self.youngs_modululs_test()
            await self.damping_scale_test()
            await self.elasticity_damping_test()

        await self.my_world.stop_async()

    async def dynamic_friction_test(self):
        await self.my_world.reset_async()
        indices = [1, 2] if self._test_cfg["indexed"] else None
        prev_values = self.deformable_material_view.get_dynamic_frictions(indices)
        new_values = prev_values + np.random.uniform(low=0.0, high=1.0, size=(prev_values.shape[0], 1)).astype(
            np.single
        )
        self.deformable_material_view.set_dynamic_frictions(new_values, indices)
        cur_values = self.deformable_material_view.get_dynamic_frictions(indices)
        self.assertTrue(self.isclose(new_values, cur_values).all())
        expected_shape = torch.Size(
            [len(indices) if self._test_cfg["indexed"] else self.deformable_material_view.count, 1]
        )
        self.assertTrue(cur_values.shape == expected_shape)
        print(expected_shape)

    async def poissons_ratio_test(self):
        await self.my_world.reset_async()
        indices = [1, 2] if self._test_cfg["indexed"] else None
        prev_values = self.deformable_material_view.get_poissons_ratios(indices) / 2
        new_values = prev_values + np.random.uniform(low=0.0, high=0.1, size=(prev_values.shape[0], 1)).astype(
            np.single
        )
        self.deformable_material_view.set_poissons_ratios(new_values, indices)
        cur_values = self.deformable_material_view.get_poissons_ratios(indices)
        self.assertTrue(self.isclose(new_values, cur_values).all())
        expected_shape = torch.Size(
            [len(indices) if self._test_cfg["indexed"] else self.deformable_material_view.count, 1]
        )
        self.assertTrue(cur_values.shape == expected_shape)
        print(expected_shape)

    async def youngs_modululs_test(self):
        await self.my_world.reset_async()
        indices = [1, 2] if self._test_cfg["indexed"] else None
        prev_values = self.deformable_material_view.get_youngs_moduli(indices)
        new_values = prev_values + np.random.uniform(low=0.0, high=1.0, size=(prev_values.shape[0], 1)).astype(
            np.single
        )
        self.deformable_material_view.set_youngs_moduli(new_values, indices)
        cur_values = self.deformable_material_view.get_youngs_moduli(indices)
        self.assertTrue(self.isclose(new_values, cur_values).all())
        expected_shape = torch.Size(
            [len(indices) if self._test_cfg["indexed"] else self.deformable_material_view.count, 1]
        )
        self.assertTrue(cur_values.shape == expected_shape)
        print(expected_shape)

    async def damping_scale_test(self):
        await self.my_world.reset_async()
        indices = [1, 2] if self._test_cfg["indexed"] else None
        prev_values = self.deformable_material_view.get_damping_scales(indices) / 2
        new_values = prev_values + np.random.uniform(low=0.0, high=0.3, size=(prev_values.shape[0], 1)).astype(
            np.single
        )
        self.deformable_material_view.set_damping_scales(new_values, indices)
        cur_values = self.deformable_material_view.get_damping_scales(indices)
        self.assertTrue(self.isclose(new_values, cur_values, atol=5e-5).all())
        expected_shape = torch.Size(
            [len(indices) if self._test_cfg["indexed"] else self.deformable_material_view.count, 1]
        )
        self.assertTrue(cur_values.shape == expected_shape)
        print(expected_shape)

    async def elasticity_damping_test(self):
        await self.my_world.reset_async()
        indices = [1, 2] if self._test_cfg["indexed"] else None
        prev_values = self.deformable_material_view.get_elasticity_dampings(indices)
        new_values = prev_values + np.random.uniform(low=0.0, high=1.0, size=(prev_values.shape[0], 1)).astype(
            np.single
        )
        self.deformable_material_view.set_elasticity_dampings(new_values, indices)
        cur_values = self.deformable_material_view.get_elasticity_dampings(indices)
        self.assertTrue(self.isclose(new_values, cur_values).all())
        expected_shape = torch.Size(
            [len(indices) if self._test_cfg["indexed"] else self.deformable_material_view.count, 1]
        )
        self.assertTrue(cur_values.shape == expected_shape)
        print(expected_shape)
