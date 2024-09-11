# Copyright (c) 2018-2024, NVIDIA CORPORATION. All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto. Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.
#

import carb
import carb.tokens
import numpy as np
import omni.graph.core as og

# NOTE:
#   omni.kit.test - std python's unittest module with additional wrapping to add suport for async/await tests
#   For most things refer to unittest docs: https://docs.python.org/3/library/unittest.html
import omni.kit.test
from omni.isaac.core import World
from omni.isaac.core.utils.extensions import get_extension_path_from_name
from omni.isaac.core.utils.prims import delete_prim
from omni.isaac.core.utils.stage import open_stage_async
from omni.isaac.nucleus import get_assets_root_path_async
from omni.physx.scripts.physicsUtils import add_ground_plane
from pxr import Gf

from .robot_helpers import init_robot_sim, set_physics_frequency, setup_robot_og


# Having a test class dervived from omni.kit.test.AsyncTestCase declared on the root of module will make it auto-discoverable by omni.kit.test
class TestO3dyn(omni.kit.test.AsyncTestCase):
    # Before running each test
    async def setUp(self):
        self._timeline = omni.timeline.get_timeline_interface()

        ext_manager = omni.kit.app.get_app().get_extension_manager()
        self._assets_root_path = await get_assets_root_path_async()
        if self._assets_root_path is None:
            carb.log_error("Could not find Isaac Sim assets folder")
            return
        self.my_world = None
        self._extension_path = get_extension_path_from_name("omni.isaac.tests")
        ## setup carter_v1:
        # open local carter_v1:
        # (result, error) = await omni.usd.get_context().open_stage_async(
        #     self._extension_path + "/data/tests/carter_v1.usd"
        # )

        # add in carter (from nucleus)
        self.usd_path = self._assets_root_path + "/Isaac/Robots/O3dyn/o3dyn.usd"

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

    async def test_loading(self):

        (result, error) = await open_stage_async(self.usd_path)

        stage = omni.usd.get_context().get_stage()

        # Make sure the stage loaded
        self.assertTrue(result)
        self.my_world = World(stage_units_in_meters=1.0)
        await self.my_world.initialize_simulation_context_async()
        self.my_world.play()
        for i in range(150):
            await omni.kit.app.get_app().next_update_async()
        pose = omni.usd.get_world_transform_matrix(
            stage.GetPrimAtPath(stage.GetDefaultPrim().GetPath().AppendPath("base_link"))
        )
        translate = pose.ExtractTranslation()
        self.assertAlmostEqual(translate[0], 0.00, delta=0.01)
        self.assertAlmostEqual(translate[1], 0.00, delta=0.01)

        self.assertAlmostEqual(translate[2], -0.01, delta=0.01)
        self.my_world.stop()
        pass

    # general, slowly building up speed testcase
    async def test_add_as_reference(self):
        omni.usd.get_context().new_stage()
        stage = omni.usd.get_context().get_stage()

        robot_prim = stage.DefinePrim(str(stage.GetDefaultPrim().GetPath()) + "/O3dyn", "Xform")

        robot_prim.GetReferences().AddReference(self.usd_path)
        self.my_world = World(stage_units_in_meters=1.0)
        add_ground_plane(stage, "/physics/groundPlane", "Z", 1000.0, Gf.Vec3f(0.0, 0, -0.12), Gf.Vec3f(1.0))
        await self.my_world.initialize_simulation_context_async()
        self.my_world.play()
        for i in range(250):
            await omni.kit.app.get_app().next_update_async()

        pose = omni.usd.get_world_transform_matrix(stage.GetPrimAtPath(robot_prim.GetPath().AppendPath("base_link")))
        translate = pose.ExtractTranslation()
        self.assertAlmostEqual(translate[0], 0.00, delta=0.01)
        self.assertAlmostEqual(translate[1], 0.00, delta=0.01)

        self.assertAlmostEqual(translate[2], -0.06, delta=0.01)
        self.my_world.stop()
        pass

    async def test_move_forward(self):
        omni.usd.get_context().new_stage()
        stage = omni.usd.get_context().get_stage()

        robot_prim = stage.DefinePrim(str(stage.GetDefaultPrim().GetPath()) + "/O3dyn", "Xform")

        robot_prim.GetReferences().AddReference(self.usd_path)
        self.my_world = World(stage_units_in_meters=1.0)
        add_ground_plane(stage, "/physics/groundPlane", "Z", 1000.0, Gf.Vec3f(0.0, 0, -0.12), Gf.Vec3f(1.0))
        await self.my_world.initialize_simulation_context_async()
        for prim in stage.GetPrimAtPath(robot_prim.GetPath().AppendPath("wheel_drive")).GetChildren():
            prim.GetAttribute("drive:angular:physics:targetVelocity").Set(100)
        self.my_world.play()
        for i in range(300):
            await omni.kit.app.get_app().next_update_async()

        pose = omni.usd.get_world_transform_matrix(stage.GetPrimAtPath(robot_prim.GetPath().AppendPath("base_link")))
        translate = pose.ExtractTranslation()
        self.assertGreater(translate[0], 1.0)
        self.assertAlmostEqual(translate[1], 0.00, delta=0.02)
        self.my_world.stop()

        pass

    async def test_move_sideways(self):
        omni.usd.get_context().new_stage()
        stage = omni.usd.get_context().get_stage()

        robot_prim = stage.DefinePrim(str(stage.GetDefaultPrim().GetPath()) + "/O3dyn", "Xform")

        robot_prim.GetReferences().AddReference(self.usd_path)
        self.my_world = World(stage_units_in_meters=1.0)
        add_ground_plane(stage, "/physics/groundPlane", "Z", 1000.0, Gf.Vec3f(0.0, 0, -0.12), Gf.Vec3f(1.0))
        await self.my_world.initialize_simulation_context_async()
        for prim in stage.GetPrimAtPath(robot_prim.GetPath().AppendPath("wheel_drive")).GetChildren():
            if prim.GetName() in ["wheel_fr_joint", "wheel_rl_joint"]:
                prim.GetAttribute("drive:angular:physics:targetVelocity").Set(100)
            else:
                prim.GetAttribute("drive:angular:physics:targetVelocity").Set(-100)
        self.my_world.play()
        for i in range(300):
            await omni.kit.app.get_app().next_update_async()

        pose = omni.usd.get_world_transform_matrix(stage.GetPrimAtPath(robot_prim.GetPath().AppendPath("base_link")))
        translate = pose.ExtractTranslation()
        self.assertAlmostEqual(translate[0], 0.00, delta=0.1)
        self.assertGreater(
            translate[1],
            1.00,
        )
        self.my_world.stop()

        pass

    async def test_rotate(self):
        omni.usd.get_context().new_stage()
        stage = omni.usd.get_context().get_stage()

        robot_prim = stage.DefinePrim(str(stage.GetDefaultPrim().GetPath()) + "/O3dyn", "Xform")

        robot_prim.GetReferences().AddReference(self.usd_path)
        self.my_world = World(stage_units_in_meters=1.0)
        add_ground_plane(stage, "/physics/groundPlane", "Z", 1000.0, Gf.Vec3f(0.0, 0, -0.12), Gf.Vec3f(1.0))
        await self.my_world.initialize_simulation_context_async()
        for prim in stage.GetPrimAtPath(robot_prim.GetPath().AppendPath("wheel_drive")).GetChildren():
            if prim.GetName() in ["wheel_fl_joint", "wheel_rl_joint"]:
                prim.GetAttribute("drive:angular:physics:targetVelocity").Set(150)
            else:
                prim.GetAttribute("drive:angular:physics:targetVelocity").Set(-150)
        self.my_world.play()
        for i in range(300):
            await omni.kit.app.get_app().next_update_async()

        pose = omni.usd.get_world_transform_matrix(stage.GetPrimAtPath(robot_prim.GetPath().AppendPath("base_link")))
        translate = pose.ExtractTranslation()
        # Robot origin is not at center of rotation, give it some slack on X/Y
        self.assertLess(
            abs(translate[0]),
            0.2,
        )
        self.assertLess(
            abs(translate[1]),
            0.2,
        )
        print(translate)
        pose.Orthonormalize()
        rotation = pose.ExtractRotation().GetAngle()
        self.assertGreater(
            rotation,
            45,
        )
        self.my_world.stop()
