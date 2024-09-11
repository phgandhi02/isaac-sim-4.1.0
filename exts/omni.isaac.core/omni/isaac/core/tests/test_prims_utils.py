# Copyright (c) 2021-2023, NVIDIA CORPORATION. All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto. Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.
#

import numpy as np
import omni.kit.commands
import omni.kit.test
import torch
from omni.isaac.core.objects import DynamicCuboid, VisualCuboid
from omni.isaac.core.prims.xform_prim import XFormPrim
from omni.isaac.core.utils.prims import find_matching_prim_paths, get_all_matching_child_prims
from omni.isaac.core.utils.stage import add_reference_to_stage
from omni.isaac.nucleus import get_assets_root_path_async


class TestPrims(omni.kit.test.AsyncTestCase):
    # Before running each test
    async def setUp(self):
        await omni.usd.get_context().new_stage_async()
        await omni.kit.app.get_app().next_update_async()
        pass

    # After running each test
    async def tearDown(self):
        pass

    async def test_get_all_matching_child_prims(self):
        from omni.isaac.core.utils.prims import create_prim, get_prim_path
        from omni.isaac.core.utils.stage import clear_stage

        clear_stage()
        create_prim("/World/Floor")
        create_prim("/World/Floor/thefloor", "Cube", position=np.array([75, 75, -150.1]), attributes={"size": 300})
        create_prim("/World/Room", "Sphere", attributes={"radius": 1e3})

        result = get_all_matching_child_prims("/World")
        result = [get_prim_path(prim) for prim in result]
        self.assertListEqual(result, ["/World", "/World/Floor", "/World/Room", "/World/Floor/thefloor"])

    async def test_create_prim(self):
        from omni.isaac.core.utils.prims import create_prim, get_prim_path
        from omni.isaac.core.utils.stage import clear_stage

        clear_stage()
        create_prim("/World")
        create_prim(
            "/World/thebox", "Cube", position=[175, 75, 0.0], orientation=[0.0, 0.0, 0.0, 1.0], attributes={"size": 150}
        )
        create_prim(
            "/World/thechair1",
            "Cube",
            position=(-75, 75, 0.0),
            orientation=(0.0, 0.0, 0.0, 1.0),
            attributes={"size": 150},
        )
        create_prim("/World/thechair2", "Cube", position=np.array([75, 75, 0.0]), attributes={"size": 150})
        create_prim("/World/thetable", "Cube", position=torch.Tensor([-175, 75, 0.0]), attributes={"size": 150})

        result = get_all_matching_child_prims("/World")
        result = [get_prim_path(prim) for prim in result]
        self.assertListEqual(
            result, ["/World", "/World/thebox", "/World/thechair1", "/World/thechair2", "/World/thetable"]
        )

    async def test_is_prim_non_root_articulation_link(self):
        from omni.isaac.core.objects import DynamicCuboid
        from omni.isaac.core.utils.prims import is_prim_non_root_articulation_link
        from omni.isaac.core.utils.stage import clear_stage
        from omni.isaac.nucleus import get_assets_root_path_async

        clear_stage()
        add_reference_to_stage(usd_path="", prim_path="/World/Franka")
        assets_root_path = await get_assets_root_path_async()
        if assets_root_path is None:
            raise Exception("Asset root path doesn't exist")
        asset_path = assets_root_path + "/Isaac/Robots/Franka/franka_alt_fingers.usd"
        add_reference_to_stage(usd_path=asset_path, prim_path="/World/Franka")
        DynamicCuboid(prim_path="/World/Franka/panda_link1/Cube")
        XFormPrim(prim_path="/World/Franka/panda_link1/test_1")
        XFormPrim(prim_path="/World/Franka/test_1")
        self.assertFalse(is_prim_non_root_articulation_link(prim_path="/World/Franka"))
        self.assertTrue(is_prim_non_root_articulation_link(prim_path="/World/Franka/panda_link1"))
        self.assertTrue(is_prim_non_root_articulation_link(prim_path="/World/Franka/panda_link0"))
        self.assertFalse(is_prim_non_root_articulation_link(prim_path="/World/Franka/panda_link1/test_1"))
        self.assertFalse(is_prim_non_root_articulation_link(prim_path="/World/Franka/test_1"))

    async def test_get_articulation_root_api_prim_path(self):
        ext_manager = omni.kit.app.get_app().get_extension_manager()
        ext_manager.get_enabled_extension_id("omni.isaac.cloner")
        await omni.kit.app.get_app().next_update_async()

        from omni.isaac.cloner import GridCloner
        from omni.isaac.core.utils.prims import get_articulation_root_api_prim_path
        from omni.isaac.core.utils.stage import clear_stage, get_current_stage
        from omni.isaac.nucleus import get_assets_root_path_async
        from pxr import UsdGeom

        env_zero_path = "/World/envs/env_0"
        num_envs = 5

        clear_stage()
        # add asset
        assets_root_path = await get_assets_root_path_async()
        if assets_root_path is None:
            raise Exception("Asset root path doesn't exist")
        asset_path = assets_root_path + "/Isaac/Robots/Humanoid/humanoid.usd"
        add_reference_to_stage(usd_path=asset_path, prim_path=f"{env_zero_path}/articulation")
        # clone env
        cloner = GridCloner(spacing=1.5)
        cloner.define_base_env(env_zero_path)
        UsdGeom.Xform.Define(get_current_stage(), env_zero_path)
        cloner.clone(source_prim_path=env_zero_path, prim_paths=cloner.generate_paths("/World/envs/env", num_envs))

        self.assertListEqual(
            [
                get_articulation_root_api_prim_path("/World/envs/env_0"),
                get_articulation_root_api_prim_path("/World/envs/env_1/articulation"),
                get_articulation_root_api_prim_path("/World/envs/env_2/articulation/torso"),
                get_articulation_root_api_prim_path("/World/envs/.*/articulation"),
                get_articulation_root_api_prim_path("/World/envs/.*/articulation/torso"),
                get_articulation_root_api_prim_path("/World/.*/env_3/articulation"),
            ],
            [
                "/World/envs/env_0/articulation/torso",
                "/World/envs/env_1/articulation/torso",
                "/World/envs/env_2/articulation/torso",
                "/World/envs/.*/articulation/torso",
                "/World/envs/.*/articulation/torso",
                "/World/.*/env_3/articulation/torso",
            ],
        )

    async def test_find_matching_prim_paths(self):
        assets_root_path = await get_assets_root_path_async()
        if assets_root_path is None:
            raise Exception("Asset root path doesn't exist")
        asset_path = assets_root_path + "/Isaac/Robots/Franka/franka_alt_fingers.usd"
        add_reference_to_stage(usd_path=asset_path, prim_path="/World/Franka_1")
        add_reference_to_stage(usd_path=asset_path, prim_path="/World/Franka_2")
        XFormPrim(prim_path="/World/Franka_3")
        VisualCuboid(prim_path="/World/cube_1")
        DynamicCuboid(prim_path="/World/cube_2")
        DynamicCuboid(prim_path="/World/cube_3")
        self.assertTrue(len(find_matching_prim_paths("/World/cube_.*", "rigid_body")) == 2)
        self.assertTrue(len(find_matching_prim_paths("/World/cube_.*", "")) == 3)
        with self.assertRaises(ValueError):
            find_matching_prim_paths("/World/cube_.*", "geometry")
        self.assertTrue(len(find_matching_prim_paths("/World/cube_.*", "rigid_body")) == 2)
        self.assertTrue(len(find_matching_prim_paths("/World/cube_.*")) == 3)
        self.assertTrue(len(find_matching_prim_paths("/World/Franka_.*")) == 3)
        self.assertTrue(len(find_matching_prim_paths("/World/Franka_.*", "articulation")) == 2)
        pass
