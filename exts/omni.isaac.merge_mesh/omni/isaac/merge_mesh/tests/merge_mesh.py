# Copyright (c) 2018-2023, NVIDIA CORPORATION. All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto. Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.
#

# NOTE:
#   omni.kit.test - std python's unittest module with additional wrapping to add suport for async/await tests
#   For most things refer to unittest docs: https://docs.python.org/3/library/unittest.html
import omni.kit.test
from pxr import Sdf, UsdGeom, UsdShade

from ..mesh_merger import MeshMerger


# Having a test class dervived from omni.kit.test.AsyncTestCase declared on the root of module will make it auto-discoverable by omni.kit.test
class TestMergeMesh(omni.kit.test.AsyncTestCase):
    # Before running each test
    async def setUp(self):
        await omni.usd.get_context().new_stage_async()
        self._stage = omni.usd.get_context().get_stage()
        mtl_created_list = []
        # Add three cubes to the scene
        self.cubes_list = []
        for i in range(3):
            result, path = omni.kit.commands.execute("CreateMeshPrimCommand", prim_type="Cube")
            self.cubes_list.append(path)
            # Create a new material using OmniPBR.mdl
            omni.kit.commands.execute(
                "CreateAndBindMdlMaterialFromLibrary",
                mdl_name="OmniPBR.mdl",
                mtl_name="OmniPBR",
                mtl_created_list=mtl_created_list,
            )
            mtl_prim = self._stage.GetPrimAtPath(mtl_created_list[-1])
            # Get the path to the prim
            cube_prim = self._stage.GetPrimAtPath(path)
            # Bind the material to the prim
            cube_mat_shade = UsdShade.Material(mtl_prim)
            UsdShade.MaterialBindingAPI(cube_prim).Bind(cube_mat_shade, UsdShade.Tokens.strongerThanDescendants)

        await omni.kit.app.get_app().next_update_async()
        pass

    # After running each test
    async def tearDown(self):
        await omni.kit.app.get_app().next_update_async()
        pass

    # Run for 60 frames and make sure there were no errors loading
    async def test_startup(self):
        window = omni.ui.Workspace.get_window("Mesh Merge Tool")
        self.assertIsNotNone(window)
        window.visible = True
        for frame in range(60):
            await omni.kit.app.get_app().next_update_async()
        window.visible = False
        pass

    async def test_basic_merge(self):
        mesh_merger = MeshMerger(self._stage)
        mesh_merger.clear_parent_xform = False
        mesh_merger.deactivate_source = False
        mesh_merger.combine_materials = False
        mesh_merger.materials_destination = ""
        mesh_merger.update_selection(selection=self.cubes_list, stage=self._stage)
        mesh_merger.output_mesh = "/Merged/" + str(self._stage.GetPrimAtPath(self.cubes_list[0]).GetName())

        mesh_merger.merge_meshes()
        await omni.kit.app.get_app().next_update_async()
        merged = self._stage.GetPrimAtPath(mesh_merger.output_mesh)
        self.assertTrue(merged.IsValid())

        self.assertEqual(len(merged.GetChildren()), 3)
        pass

    async def test_material_combine_merge(self):
        mesh_merger = MeshMerger(self._stage)
        mesh_merger.clear_parent_xform = False
        mesh_merger.deactivate_source = False
        mesh_merger.combine_materials = True
        mesh_merger.materials_destination = "/World/Looks2"
        mesh_merger.update_selection(selection=self.cubes_list, stage=self._stage)
        mesh_merger.output_mesh = "/Merged/" + str(self._stage.GetPrimAtPath(self.cubes_list[0]).GetName())

        mesh_merger.merge_meshes()

        merged = self._stage.GetPrimAtPath(mesh_merger.output_mesh)
        self.assertTrue(merged.IsValid())

        self.assertEqual(len(merged.GetChildren()), 3)

        newLooks = self._stage.GetPrimAtPath("/World/Looks2")
        self.assertEqual(len(newLooks.GetChildren()), 3)
        pass

    async def test_deactivate_source(self):
        mesh_merger = MeshMerger(self._stage)
        mesh_merger.clear_parent_xform = False
        mesh_merger.deactivate_source = True
        mesh_merger.combine_materials = False
        mesh_merger.materials_destination = "/World/Looks2"
        mesh_merger.update_selection(selection=self.cubes_list, stage=self._stage)
        mesh_merger.output_mesh = "/Merged/" + str(self._stage.GetPrimAtPath(self.cubes_list[0]).GetName())

        mesh_merger.merge_meshes()

        merged = self._stage.GetPrimAtPath(mesh_merger.output_mesh)
        self.assertTrue(merged.IsValid())

        self.assertEqual(len(merged.GetChildren()), 3)

        for src in self.cubes_list:
            prim = self._stage.GetPrimAtPath(src)
            self.assertFalse(prim.IsActive())
        pass

    async def test_clear_parent_xform(self):
        mesh_merger = MeshMerger(self._stage)
        mesh_merger.clear_parent_xform = True
        mesh_merger.deactivate_source = True
        mesh_merger.combine_materials = False
        mesh_merger.materials_destination = "/World/Looks2"
        mesh_merger.update_selection(selection=self.cubes_list, stage=self._stage)
        mesh_merger.output_mesh = "/Merged/" + str(self._stage.GetPrimAtPath(self.cubes_list[0]).GetName())

        mesh_merger.merge_meshes()

        merged = self._stage.GetPrimAtPath(mesh_merger.output_mesh)
        self.assertTrue(merged.IsValid())

        pass

    async def test_merge_command(self):
        result, prim = omni.kit.commands.execute(
            "MergeMeshesCommand",
            source=self.cubes_list,
            clear_transform=False,
            deactivate_source=True,
            combine_materials=True,
            materials_destination="/World/Looks",
        )

        merged = self._stage.GetPrimAtPath(prim)
        self.assertTrue(merged.IsValid())
