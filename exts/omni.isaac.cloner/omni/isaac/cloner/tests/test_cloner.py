# Copyright (c) 2021-2024, NVIDIA CORPORATION. All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto. Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.
#

import unittest

import numpy as np
import omni.kit
from omni.isaac.cloner import Cloner, GridCloner
from omni.isaac.nucleus import get_assets_root_path_async
from pxr import Gf, Usd, UsdGeom, UsdPhysics, Vt


class TestSimpleCloner(omni.kit.test.AsyncTestCase):
    async def setUp(self):
        ext_manager = omni.kit.app.get_app().get_extension_manager()
        ext_id = ext_manager.get_enabled_extension_id("omni.isaac.cloner")
        self._extension_path = ext_manager.get_extension_path(ext_id)
        await omni.usd.get_context().new_stage_async()
        await omni.kit.app.get_app().next_update_async()

    async def test_simple_cloner(self):
        stage = omni.usd.get_context().get_stage()

        # create our base environment with one cube
        base_env_path = "/World/Cube_0"
        UsdGeom.Cube.Define(stage, base_env_path)

        # create a Cloner instance
        cloner = Cloner()

        # generate 4 paths that begin with "/World/Cube" - path will be appended with _{index}
        target_paths = cloner.generate_paths("/World/Cube", 4)

        cube_positions = np.array([[0, 0, 0], [3, 0, 0], [6, 0, 0], [9, 0, 0]])
        target_translations = []
        for pos in cube_positions:
            target_translations.append(Gf.Vec3d(*pos.tolist()))

        # clone the cube at target paths at specified positions
        cloner.clone(
            source_prim_path="/World/Cube_0", prim_paths=target_paths, positions=cube_positions, replicate_physics=False
        )

        for i in range(4):
            self.assertTrue(stage.GetPrimAtPath(f"/World/Cube_{i}") is not None)
            self.assertTrue(stage.GetPrimAtPath(f"/World/Cube_{i}").GetTypeName() == "Cube")
            self.assertTrue(
                stage.GetPrimAtPath(f"/World/Cube_{i}").GetAttribute("xformOp:translate").Get()
                == target_translations[i]
            )

    async def test_simple_cloner_physics_replication(self):
        stage = omni.usd.get_context().get_stage()

        # create our base environment with one cube
        base_env_path = "/World/Cube_0"
        cube = UsdGeom.Cube.Define(stage, base_env_path)
        UsdPhysics.RigidBodyAPI.Apply(cube.GetPrim())

        # create a Cloner instance
        cloner = Cloner()

        # generate 4 paths that begin with "/World/Cube" - path will be appended with _{index}
        target_paths = cloner.generate_paths("/World/Cube", 4)

        cube_positions = np.array([[0, 0, 0], [3, 0, 0], [6, 0, 0], [9, 0, 0]])
        target_translations = []
        for pos in cube_positions:
            target_translations.append(Gf.Vec3d(*pos.tolist()))

        # clone the cube at target paths at specified positions
        cloner.clone(
            source_prim_path="/World/Cube_0",
            prim_paths=target_paths,
            positions=cube_positions,
            replicate_physics=True,
            base_env_path="/World",
        )

        for i in range(4):
            self.assertTrue(stage.GetPrimAtPath(f"/World/Cube_{i}") is not None)
            self.assertTrue(stage.GetPrimAtPath(f"/World/Cube_{i}").GetTypeName() == "Cube")
            self.assertTrue(stage.GetPrimAtPath(f"/World/Cube_{i}").HasAPI(UsdPhysics.RigidBodyAPI))
            self.assertTrue(
                stage.GetPrimAtPath(f"/World/Cube_{i}").GetAttribute("xformOp:translate").Get()
                == target_translations[i]
            )

    async def test_simple_cloner_copy_randomization(self):
        stage = omni.usd.get_context().get_stage()

        # create our base environment with one cube
        base_env_path = "/World/Cube_0"
        UsdGeom.Cube.Define(stage, base_env_path)

        # create a Cloner instance
        cloner = Cloner()

        # generate 4 paths that begin with "/World/Cube" - path will be appended with _{index}
        target_paths = cloner.generate_paths("/World/Cube", 4)

        cube_positions = np.array([[0, 0, 0], [3, 0, 0], [6, 0, 0], [9, 0, 0]])
        target_translations = []
        for pos in cube_positions:
            target_translations.append(Gf.Vec3d(*pos.tolist()))

        # clone the cube at target paths at specified positions
        cloner.clone(
            source_prim_path="/World/Cube_0",
            prim_paths=target_paths,
            positions=cube_positions,
            replicate_physics=False,
            copy_from_source=True,
        )

        colors = [
            Vt.Vec3fArray(1, (Gf.Vec3f(1.0, 0.0, 0.0))),
            Vt.Vec3fArray(1, (Gf.Vec3f(0.0, 1.0, 0.0))),
            Vt.Vec3fArray(1, (Gf.Vec3f(0.0, 0.0, 1.0))),
            Vt.Vec3fArray(1, (Gf.Vec3f(0.5, 0.5, 0.5))),
        ]
        for i in range(4):
            stage.GetPrimAtPath(f"/World/Cube_{i}").GetAttribute("primvars:displayColor").Set(colors[i])

        for i in range(4):
            self.assertTrue(stage.GetPrimAtPath(f"/World/Cube_{i}") is not None)
            self.assertTrue(stage.GetPrimAtPath(f"/World/Cube_{i}").GetTypeName() == "Cube")
            self.assertTrue(
                stage.GetPrimAtPath(f"/World/Cube_{i}").GetAttribute("xformOp:translate").Get()
                == target_translations[i]
            )
            self.assertTrue(
                stage.GetPrimAtPath(f"/World/Cube_{i}").GetAttribute("primvars:displayColor").Get() == colors[i]
            )

    async def test_grid_cloner(self):
        stage = omni.usd.get_context().get_stage()

        # create our base environment with one cube
        base_env_path = "/World/Cube_0"
        UsdGeom.Cube.Define(stage, base_env_path)

        # create a GridCloner instance
        cloner = GridCloner(spacing=3)

        # generate 4 paths that begin with "/World/Cube" - path will be appended with _{index}
        target_paths = cloner.generate_paths("/World/Cube", 4)

        # clone the cube at target paths
        cloner.clone(source_prim_path="/World/Cube_0", prim_paths=target_paths, replicate_physics=False)

        target_translations = [
            Gf.Vec3d(1.5, -1.5, 0),
            Gf.Vec3d(1.5, 1.5, 0),
            Gf.Vec3d(-1.5, -1.5, 0),
            Gf.Vec3d(-1.5, 1.5, 0),
        ]
        for i in range(4):
            self.assertTrue(stage.GetPrimAtPath(f"/World/Cube_{i}") is not None)
            self.assertTrue(stage.GetPrimAtPath(f"/World/Cube_{i}").GetTypeName() == "Cube")
            self.assertTrue(
                stage.GetPrimAtPath(f"/World/Cube_{i}").GetAttribute("xformOp:translate").Get()
                == target_translations[i]
            )

    async def test_grid_cloner_physics_replication(self):
        stage = omni.usd.get_context().get_stage()

        # create our base environment with one cube
        base_env_path = "/World/Cube_0"
        cube = UsdGeom.Cube.Define(stage, base_env_path)
        UsdPhysics.RigidBodyAPI.Apply(cube.GetPrim())

        # create a GridCloner instance
        cloner = GridCloner(spacing=3)

        # generate 4 paths that begin with "/World/Cube" - path will be appended with _{index}
        target_paths = cloner.generate_paths("/World/Cube", 4)

        # clone the cube at target paths
        cloner.clone(
            source_prim_path="/World/Cube_0", prim_paths=target_paths, replicate_physics=True, base_env_path="/World"
        )

        target_translations = [
            Gf.Vec3d(1.5, -1.5, 0),
            Gf.Vec3d(1.5, 1.5, 0),
            Gf.Vec3d(-1.5, -1.5, 0),
            Gf.Vec3d(-1.5, 1.5, 0),
        ]
        for i in range(4):
            self.assertTrue(stage.GetPrimAtPath(f"/World/Cube_{i}") is not None)
            self.assertTrue(stage.GetPrimAtPath(f"/World/Cube_{i}").GetTypeName() == "Cube")
            self.assertTrue(stage.GetPrimAtPath(f"/World/Cube_{i}").HasAPI(UsdPhysics.RigidBodyAPI))
            self.assertTrue(
                stage.GetPrimAtPath(f"/World/Cube_{i}").GetAttribute("xformOp:translate").Get()
                == target_translations[i]
            )

    async def test_grid_cloner_articulation(self):
        stage = omni.usd.get_context().get_stage()

        # create our base environment with one cube
        base_env_path = "/World/envs"

        cloner = GridCloner(spacing=3)
        cloner.define_base_env(base_env_path + "/env_0")
        UsdGeom.Xform.Define(stage, base_env_path + "/env_0")
        prim = stage.DefinePrim(base_env_path + "/env_0/Ant", "Xform")
        asset_root_path = await get_assets_root_path_async()
        prim.GetReferences().AddReference(asset_root_path + "/Isaac/Robots/Ant/ant_instanceable.usd")

        target_paths = cloner.generate_paths("/World/envs/env", 100)

        # clone the cube at target paths
        target_translations = cloner.clone(
            source_prim_path="/World/envs/env_0", prim_paths=target_paths, replicate_physics=False
        )

        for i in range(100):
            self.assertTrue(stage.GetPrimAtPath(f"/World/envs/env_{i}") is not None)
            self.assertTrue(
                stage.GetPrimAtPath(f"/World/envs/env_{i}/Ant/torso").HasAPI(UsdPhysics.ArticulationRootAPI)
            )
            self.assertTrue(
                stage.GetPrimAtPath(f"/World/envs/env_{i}").GetAttribute("xformOp:translate").Get()
                == Gf.Vec3d(*target_translations[i])
            )

    async def test_grid_cloner_articulation_physics_replication(self):
        stage = omni.usd.get_context().get_stage()

        # create our base environment with one cube
        base_env_path = "/World/envs"

        cloner = GridCloner(spacing=3)
        cloner.define_base_env(base_env_path + "/env_0")
        UsdGeom.Xform.Define(stage, base_env_path + "/env_0")
        prim = stage.DefinePrim(base_env_path + "/env_0/Ant", "Xform")
        asset_root_path = await get_assets_root_path_async()
        prim.GetReferences().AddReference(asset_root_path + "/Isaac/Robots/Ant/ant_instanceable.usd")

        target_paths = cloner.generate_paths("/World/envs/env", 100)

        # clone the cube at target paths
        target_translations = cloner.clone(
            source_prim_path="/World/envs/env_0",
            prim_paths=target_paths,
            replicate_physics=True,
            base_env_path="/World/envs",
        )

        for i in range(100):
            self.assertTrue(stage.GetPrimAtPath(f"/World/envs/env_{i}") is not None)
            self.assertTrue(
                stage.GetPrimAtPath(f"/World/envs/env_{i}/Ant/torso").HasAPI(UsdPhysics.ArticulationRootAPI)
            )
            self.assertTrue(
                stage.GetPrimAtPath(f"/World/envs/env_{i}").GetAttribute("xformOp:translate").Get()
                == Gf.Vec3d(*target_translations[i])
            )

    async def test_grid_cloner_copy_addition(self):
        stage = omni.usd.get_context().get_stage()

        # create our base environment with one cube
        base_env_path = "/World/envs"

        cloner = GridCloner(spacing=3)
        cloner.define_base_env(base_env_path + "/env_0")
        UsdGeom.Xform.Define(stage, base_env_path + "/env_0")
        prim = stage.DefinePrim(base_env_path + "/env_0/Ant", "Xform")
        asset_root_path = await get_assets_root_path_async()
        prim.GetReferences().AddReference(asset_root_path + "/Isaac/Robots/Ant/ant_instanceable.usd")

        target_paths = cloner.generate_paths("/World/envs/env", 100)

        # clone the cube at target paths
        target_translations = cloner.clone(
            source_prim_path="/World/envs/env_0",
            prim_paths=target_paths,
            replicate_physics=True,
            base_env_path="/World/envs",
            copy_from_source=True,
        )

        for i in range(100):
            self.assertTrue(stage.GetPrimAtPath(f"/World/envs/env_{i}") is not None)
            self.assertTrue(
                stage.GetPrimAtPath(f"/World/envs/env_{i}/Ant/torso").HasAPI(UsdPhysics.ArticulationRootAPI)
            )
            self.assertTrue(
                stage.GetPrimAtPath(f"/World/envs/env_{i}").GetAttribute("xformOp:translate").Get()
                == Gf.Vec3d(*target_translations[i])
            )

        UsdGeom.Cube.Define(stage, "/World/envs/env_0/Cube")
        UsdGeom.Sphere.Define(stage, "/World/envs/env_1/Sphere")
        self.assertTrue(stage.GetPrimAtPath(f"/World/envs/env_0/Cube").IsValid() == True)
        self.assertTrue(stage.GetPrimAtPath(f"/World/envs/env_0/Sphere").IsValid() == False)
        self.assertTrue(stage.GetPrimAtPath(f"/World/envs/env_1/Cube").IsValid() == False)
        self.assertTrue(stage.GetPrimAtPath(f"/World/envs/env_1/Sphere").IsValid() == True)

    async def test_grid_cloner_inherit_addition(self):
        stage = omni.usd.get_context().get_stage()

        # create our base environment with one cube
        base_env_path = "/World/envs"

        cloner = GridCloner(spacing=3)
        cloner.define_base_env(base_env_path + "/env_0")
        UsdGeom.Xform.Define(stage, base_env_path + "/env_0")
        prim = stage.DefinePrim(base_env_path + "/env_0/Ant", "Xform")
        asset_root_path = await get_assets_root_path_async()
        prim.GetReferences().AddReference(asset_root_path + "/Isaac/Robots/Ant/ant_instanceable.usd")

        target_paths = cloner.generate_paths("/World/envs/env", 100)

        # clone the cube at target paths
        target_translations = cloner.clone(
            source_prim_path="/World/envs/env_0",
            prim_paths=target_paths,
            replicate_physics=True,
            base_env_path="/World/envs",
            copy_from_source=False,
        )

        for i in range(100):
            self.assertTrue(stage.GetPrimAtPath(f"/World/envs/env_{i}") is not None)
            self.assertTrue(
                stage.GetPrimAtPath(f"/World/envs/env_{i}/Ant/torso").HasAPI(UsdPhysics.ArticulationRootAPI)
            )
            self.assertTrue(
                stage.GetPrimAtPath(f"/World/envs/env_{i}").GetAttribute("xformOp:translate").Get()
                == Gf.Vec3d(*target_translations[i])
            )

        UsdGeom.Cube.Define(stage, "/World/envs/env_0/Cube")
        UsdGeom.Sphere.Define(stage, "/World/envs/env_1/Sphere")
        self.assertTrue(stage.GetPrimAtPath(f"/World/envs/env_0/Cube").IsValid() == True)
        self.assertTrue(stage.GetPrimAtPath(f"/World/envs/env_0/Sphere").IsValid() == False)
        self.assertTrue(stage.GetPrimAtPath(f"/World/envs/env_1/Cube").IsValid() == True)
        self.assertTrue(stage.GetPrimAtPath(f"/World/envs/env_1/Sphere").IsValid() == True)

    async def test_grid_cloner_offsets(self):
        stage = omni.usd.get_context().get_stage()

        # create our base environment with one cube
        base_env_path = "/World/envs"

        cloner = GridCloner(spacing=3)
        cloner.define_base_env(base_env_path + "/env_0")
        UsdGeom.Xform.Define(stage, base_env_path + "/env_0")
        prim = stage.DefinePrim(base_env_path + "/env_0/Ant", "Xform")
        asset_root_path = await get_assets_root_path_async()
        prim.GetReferences().AddReference(asset_root_path + "/Isaac/Robots/Ant/ant_instanceable.usd")

        target_paths = cloner.generate_paths("/World/envs/env", 100)

        position_offsets = [[0, 0, 1.0]] * 100
        orientation_offsets = [[0, 0, 0, 1.0]] * 100

        # clone the cube at target paths
        target_translations = cloner.clone(
            source_prim_path="/World/envs/env_0",
            prim_paths=target_paths,
            replicate_physics=True,
            base_env_path="/World/envs",
            copy_from_source=False,
            position_offsets=position_offsets,
            orientation_offsets=orientation_offsets,
        )

        for i in range(100):
            self.assertTrue(stage.GetPrimAtPath(f"/World/envs/env_{i}") is not None)
            self.assertTrue(
                stage.GetPrimAtPath(f"/World/envs/env_{i}/Ant/torso").HasAPI(UsdPhysics.ArticulationRootAPI)
            )
            self.assertTrue(
                stage.GetPrimAtPath(f"/World/envs/env_{i}").GetAttribute("xformOp:translate").Get()
                == Gf.Vec3d(*target_translations[i])
            )
            self.assertTrue(
                stage.GetPrimAtPath(f"/World/envs/env_{i}").GetAttribute("xformOp:translate").Get()[2] == 1.0
            )
            self.assertTrue(
                stage.GetPrimAtPath(f"/World/envs/env_{i}").GetAttribute("xformOp:orient").Get()
                == Gf.Quatd(0.0, Gf.Vec3d(0.0, 0.0, 1.0))
            )

    async def test_simple_cloner_on_stage_in_memory(self):
        stage = Usd.Stage.CreateInMemory()

        # create our base environment with one cube
        base_env_path = "/World/Cube_0"
        UsdGeom.Cube.Define(stage, base_env_path)

        # create a Cloner instance
        cloner = Cloner(stage=stage)

        # generate 4 paths that begin with "/World/Cube" - path will be appended with _{index}
        target_paths = cloner.generate_paths("/World/Cube", 4)

        cube_positions = np.array([[0, 0, 0], [3, 0, 0], [6, 0, 0], [9, 0, 0]])
        target_translations = []
        for pos in cube_positions:
            target_translations.append(Gf.Vec3d(*pos.tolist()))

        # clone the cube at target paths at specified positions
        cloner.clone(
            source_prim_path="/World/Cube_0", prim_paths=target_paths, positions=cube_positions, replicate_physics=False
        )

        for i in range(4):
            self.assertTrue(stage.GetPrimAtPath(f"/World/Cube_{i}") is not None)
            self.assertTrue(stage.GetPrimAtPath(f"/World/Cube_{i}").GetTypeName() == "Cube")
            self.assertTrue(
                stage.GetPrimAtPath(f"/World/Cube_{i}").GetAttribute("xformOp:translate").Get()
                == target_translations[i]
            )
