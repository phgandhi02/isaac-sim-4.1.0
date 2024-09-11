# Copyright (c) 2022-2023, NVIDIA CORPORATION. All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto. Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.
#

import omni.kit.test
from omni.isaac.core.materials.physics_material import PhysicsMaterial

# Import extension python module we are testing with absolute import path, as if we are external user (other extension)
from omni.isaac.core.prims.geometry_prim import GeometryPrim

# NOTE:
#   omni.kit.test - std python's unittest module with additional wrapping to add support for async/await tests
#   For most things refer to unittest docs: https://docs.python.org/3/library/unittest.html
from omni.isaac.core.utils.prims import define_prim
from pxr import UsdPhysics


# Having a test class derived from omni.kit.test.AsyncTestCase declared on the root of module will make it auto-discoverable by omni.kit.test
class TestGeometryPrim(omni.kit.test.AsyncTestCase):
    async def setUp(self):
        await omni.usd.get_context().new_stage_async()
        await omni.kit.app.get_app().next_update_async()
        pass

    async def tearDown(self):
        await omni.kit.app.get_app().next_update_async()
        pass

    async def test_collision_approximation(self):
        define_prim("/test", prim_type="cube")
        geometry_prim = GeometryPrim("/test", "test", collision=True)
        approximations = ["convexHull", "convexDecomposition"]
        for possible_approx in approximations:
            geometry_prim.set_collision_approximation(possible_approx)
            self.assertEqual(possible_approx, geometry_prim.get_collision_approximation())
        return

    async def test_collision_enabled(self):
        define_prim("/test", prim_type="cube")
        geometry_prim = GeometryPrim("/test", "test")
        api = UsdPhysics.CollisionAPI.Apply(geometry_prim.prim)
        api.GetCollisionEnabledAttr().Set(True)
        self.assertTrue(geometry_prim.get_collision_enabled())
        return

    async def test_physics_material(self):
        define_prim("/test", prim_type="cube")
        geometry_prim = GeometryPrim("/test", "test")
        physics_material = PhysicsMaterial(
            prim_path="/Physics_material_1", dynamic_friction=0.2, static_friction=0.2, restitution=0.0
        )
        geometry_prim.apply_physics_material(physics_material=physics_material)
        self.assertEqual(geometry_prim.get_applied_physics_material(), physics_material)
        return
