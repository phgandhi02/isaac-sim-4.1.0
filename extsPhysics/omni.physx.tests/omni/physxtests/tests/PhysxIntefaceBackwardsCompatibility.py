from unittest import skip
from omni.physx import get_physx_interface, get_physx_simulation_interface
from omni.physxtests.utils.physicsBase import PhysicsKitStageAsyncTestCase, TestCategory
from omni.physx.scripts import physicsUtils
from omni.physxtests import utils as testUtils
from pxr import UsdPhysics, UsdUtils, Sdf, PhysxSchema
import omni.usd
import os


class PhysxInterfaceBackwardsCompatibilityTestKitStage(PhysicsKitStageAsyncTestCase):
    category = TestCategory.Kit
    async def _open_usd(self, filename):
        schema_folder = os.path.abspath(os.path.normpath(os.path.join(__file__, "../../../../data/")))
        schema_folder = schema_folder.replace("\\", "/") + "/"

        print(schema_folder + filename + ".usda")

        await omni.usd.get_context().open_stage_async(schema_folder + filename + ".usda")

        usd_context = omni.usd.get_context()
        self.assertIn(filename, usd_context.get_stage_url())

        return omni.usd.get_context().get_stage()

    async def _compat_test_base(self, name):
        stage = await self._open_usd(name)
        self.set_stage(stage)

        # check that we got a compatibility warning
        self.assertTrue(get_physx_interface().check_backwards_compatibility())

        # fix it up
        get_physx_interface().run_backwards_compatibility()

        # check that we don't got a compatibility warning anymore
        self.assertTrue(not get_physx_interface().check_backwards_compatibility())

    async def test_backward_compat_vehicles(self):
        await self._compat_test_base("vehicles_compat")

    async def test_backward_compat_base(self):
        await self._compat_test_base("base_compat")

    async def test_backward_compat_cct(self):
        await self._compat_test_base("cct_compat")

    async def test_backward_compat_physx_mesh_collision_api(self):
        await self._compat_test_base("physxMeshCollisionAPI_compat")

    async def test_backward_compat_trianglemesh_to_sdf(self):
        stage = await self._open_usd("trianglemesh_to_sdf_compat")
        self.set_stage(stage)

        meshPrim = stage.GetPrimAtPath(str(stage.GetDefaultPrim().GetPath()) + "/meshCube").GetPrim()

        params = ["physxTriangleMeshCollision:sdfResolution",
                  "physxTriangleMeshCollision:sdfBitsPerSubgridPixel",
                  "physxTriangleMeshCollision:sdfMargin",
                  "physxTriangleMeshCollision:sdfNarrowBandThickness",
                  "physxTriangleMeshCollision:sdfSubgridResolution"]

        oldvals = [meshPrim.GetAttribute(p).Get() for p in params]

        self.assertTrue(get_physx_interface().check_backwards_compatibility())
        get_physx_interface().run_backwards_compatibility()
        self.assertTrue(not get_physx_interface().check_backwards_compatibility())

        physxSDFMeshCollisionAPI = PhysxSchema.PhysxSDFMeshCollisionAPI(meshPrim)
        self.assertIsNotNone(physxSDFMeshCollisionAPI)
        newvals = [physxSDFMeshCollisionAPI.GetSdfResolutionAttr().Get(),
                   physxSDFMeshCollisionAPI.GetSdfBitsPerSubgridPixelAttr().Get(),
                   physxSDFMeshCollisionAPI.GetSdfMarginAttr().Get(),
                   physxSDFMeshCollisionAPI.GetSdfNarrowBandThicknessAttr().Get(),
                   physxSDFMeshCollisionAPI.GetSdfSubgridResolutionAttr().Get()]

        for o, n in zip(oldvals, newvals):
            self.assertEqual(o, n)

        for o in params:
            self.assertFalse(meshPrim.GetAttribute(o))

    # Test that the physxScene:asyncSimRender attribute (now in schema, no longer an attribute)
    # is properly handled and converted to the new schema value
    async def test_backward_compat_async_scene_simulation(self):
        stage = await self.new_stage()
        self.set_stage(stage)
        omni.usd.get_context()
        self.default_prim_path = str(stage.GetDefaultPrim().GetPath())

        scene = UsdPhysics.Scene.Define(stage, self.default_prim_path + "/physicsScene")
        scenePrim = scene.GetPrim()
        # apply physx scene schema
        PhysxSchema.PhysxSceneAPI.Apply(scenePrim)
        # Add the old attribute manually
        scenePrim.CreateAttribute("asyncSimRender", Sdf.ValueTypeNames.Bool, True).Set(True)

        # check that we got a compatibility warning
        self.assertTrue(get_physx_interface().check_backwards_compatibility())
        get_physx_interface().run_backwards_compatibility() # fix up the compatibility
        # now check that the backwards compatibility warning won't fire again
        self.assertFalse(get_physx_interface().check_backwards_compatibility())

    # Test that the physxsdfcollision:resolution attribute is properly handled
    async def test_backward_compat_sdf_resolution(self):
        stage = await self.new_stage()
        self.set_stage(stage)
        omni.usd.get_context()
        self.default_prim_path = stage.GetDefaultPrim().GetPath()

        meshCube = physicsUtils.create_mesh_cube(stage, self.default_prim_path.AppendChild("meshCube"), 20.0)
        UsdPhysics.RigidBodyAPI.Apply(meshCube.GetPrim())
        UsdPhysics.CollisionAPI.Apply(meshCube.GetPrim())
        UsdPhysics.MeshCollisionAPI.Apply(meshCube.GetPrim())

        # Add the api with old custom attribute manually
        PhysxSchema.PhysxTriangleMeshCollisionAPI.Apply(meshCube.GetPrim())
        oldAttrvalue = 8
        meshCube.GetPrim().CreateAttribute("physxsdfcollision:resolution", Sdf.ValueTypeNames.Int, True).Set(oldAttrvalue)

        # check that we got a compatibility warning
        self.assertTrue(get_physx_interface().check_backwards_compatibility())
        get_physx_interface().run_backwards_compatibility() # fix up the compatibility

        # now check that the backwards compatibility warning won't fire again
        self.assertFalse(get_physx_interface().check_backwards_compatibility())

        # and that the old attr is gone and the new one has the correct value and the new api is applied
        physxSDFMeshCollisionAPI = PhysxSchema.PhysxSDFMeshCollisionAPI(meshCube.GetPrim())
        self.assertIsNotNone(physxSDFMeshCollisionAPI)
        newAttrValue = physxSDFMeshCollisionAPI.GetSdfResolutionAttr().Get()
        self.assertEqual(oldAttrvalue, newAttrValue)
        self.assertFalse(meshCube.GetPrim().GetAttribute("physxsdfcollision:resolution"))

    # Test that the PhysxArticulationForceSensorAPI triggers a deprecation warning
    # when running a backwards compatibility check
    async def test_backward_compat_articulation_force_sensor(self):

        stage = await self._open_usd("articulationForceSensor_compat")
        self.set_stage(stage)

        # check that we got a compatibility warning.
        message = f"Physics backwardsCompatibility: PhysxArticulationForceSensorAPI has been removed. Prim (/World/Cube)."
        with testUtils.ExpectMessage(self, message):
            get_physx_interface().check_backwards_compatibility()

        # remove the PhysxArticulationForceSensorAPI from the Cube prim because it is no longer a valid schema.
        get_physx_interface().run_backwards_compatibility()

        # check that we get no compatibility warning.
        get_physx_interface().check_backwards_compatibility()

    # Test that the physxScene iteration counts rename works correctly
    async def test_backward_scene_iteration_count(self):
        stage = await self.new_stage()
        self.set_stage(stage)
        omni.usd.get_context()
        self.default_prim_path = str(stage.GetDefaultPrim().GetPath())

        # create the scene
        scene = UsdPhysics.Scene.Define(stage, self.default_prim_path + "/physicsScene")
        scenePrim = scene.GetPrim()
        sceneApi = PhysxSchema.PhysxSceneAPI.Apply(scenePrim)

        # Add the old attribute manually
        oldMaxValue = 130
        oldMinValue = 125
        scenePrim.CreateAttribute("physxScene:maxIterationCount", Sdf.ValueTypeNames.UInt, True).Set(oldMaxValue)
        scenePrim.CreateAttribute("physxScene:minIterationCount", Sdf.ValueTypeNames.UInt, True).Set(oldMinValue)

        # check that we got a compatibility warning
        self.assertTrue(get_physx_interface().check_backwards_compatibility())
        get_physx_interface().run_backwards_compatibility() # fix up the compatibility

        # now check that the backwards compatibility warning won't fire again
        self.assertFalse(get_physx_interface().check_backwards_compatibility())

        # and that the old attr is gone and the new one has the correct value
        self.assertFalse(scenePrim.GetAttribute("physxScene:maxIterationCount"))
        self.assertFalse(scenePrim.GetAttribute("physxScene:minIterationCount"))
        newMinValue = sceneApi.GetMinPositionIterationCountAttr().Get()
        newMaxValue = sceneApi.GetMaxPositionIterationCountAttr().Get()
        self.assertEqual(oldMinValue, newMinValue)
        self.assertEqual(oldMaxValue, newMaxValue)
