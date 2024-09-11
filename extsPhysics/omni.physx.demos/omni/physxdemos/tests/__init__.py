from omni.physxtestsvisual.utils import TestCase
from omni.physxtests.utils.physicsBase import PhysicsKitStageAsyncTestCase, TestCategory
import omni.physxdemos as demo
import carb.settings
import omni.kit.stage_templates

ASSET_DEMOS = [
    "FrankaNutBoltDemo",                # showroom
    "NutsAndBoltsDemo",                 # showroom
    "AnalogDigitalClockDemo",           # showroom
    "FrankaDeformableDemo",
    "ChairStackingDemo",
    "FrankaBrickStackDemo",
    "HoneyDemo",
    "ClothDeckChairDemo",
    # "FluidIsosurfaceGlassBoxDemo",    # Crashing OM-116136
    "MixerDemo",
    "TeddyOnIceDemo",
]

EXCLUDE_FROM_ALL = [
    "TriangleMeshMultiMaterialDemo",    # timeouting
    "DeformableHandDemo",               # timeouts
    "ParticleSamplerDemo",              # causes sporadic GPU crashes
    "RigidBodyCCDDemo",                 # intermittent ERROR_DEVICE_LOST on Linux TC, not reproable locally ..
    "ForceDemo",                        # failing on omni.graph.core dep?
    "LegoTechnicBuggy",                 # timeouts from remote content loading
    "MuseumDemo",                       # currently disabled due to issue OM-84698, perhaps enable once fixed
    "ParticlePostProcessingDemo",       # OM-115172
    "FluidIsosurfaceGlassBoxDemo",      # Crashing OM-116136
]

EXCLUDE_FROM_VISUAL = [
    "OverlapAny",                       # Render seems to be different each time
    "OverlapMeshDemo",                  # debug vis sometimes renders, sometimes not  
    "OverlapMultipleDemo",              # debug vis sometimes renders, sometimes not  
    "TriggerDemo",                      # debug vis sometimes renders, sometimes not
    "RaycastsDemo",                     # overlay is offset on linux
    "SweepsDemo",                       # overlay is offset on linux
    "DeformableBodyAttachmentsDemo",    # https://nvidia-omniverse.atlassian.net/browse/OM-45168
    "ParticleInflatableDemo",           # needs better camera angle (demo cam not set for some reason)
    "ParticleClothDemo",                # needs better camera angle (demo cam not set for some reason)
    "ParticleSamplerDemo",              # needs better camera angle (demo cam not set for some reason)
    "ParticlePostProcessingDemo",       # needs better camera angle (demo cam not set for some reason)
    "OverlapShapeDemo",                 # unstable overlay on VP2
]


class DirectImportTestDemo(demo.Base):
    category = "TestDemo"


class PhysxDemoBase(TestCase):
    category = TestCategory.Core
    thresholds = {"HairSimpleDemo":0.01}

    async def _visual_base(self, exclude_list):
        async def do_prepare(scene_class):
            print(scene_class.__module__)
            await self.setup_viewport_test()

        async def do_test(scene_class):
            await self.wait(60)
            await self.step(20, precise=True)
            demo_name = f"{scene_class.__module__.split('.')[-1]}"
            await self.do_visual_test(
                img_name="",
                img_suffix=demo_name,
                threshold=PhysxDemoBase.thresholds.get(demo_name, 0.0015),
            )

        await demo.test("omni.physxdemos", do_prepare, do_test, exclude_list, test_case=self, inspect_offset=2)

    async def _base(self, exclude_list):
        async def do_prepare(scene_class):
            print(scene_class.__module__)

        async def do_steps(scene_class):
            await self.wait(60)
            await self.step(20)

        await demo.test("omni.physxdemos", do_prepare, do_steps, exclude_list, test_case=self, inspect_offset=2)


class PhysxBaseDemosTest(PhysxDemoBase):
    category = TestCategory.Core

    async def test_physics_demos(self):
        exclude_list = EXCLUDE_FROM_ALL + ASSET_DEMOS
        await self._base(exclude_list)

    async def test_physics_visual_demos(self):
        exclude_list = EXCLUDE_FROM_ALL + EXCLUDE_FROM_VISUAL + ASSET_DEMOS
        await self._visual_base(exclude_list)

    async def test_physics_demos_perstage_settings(self):
        settings = carb.settings.get_settings()
        stageSharpness = 0.123456
        stagePickingForce = 123456

        # reset stage and check if the defaults didn't change to our values
        await self.new_stage()
        sharpness = settings.get("/rtx/post/aa/sharpness")
        pickingForce = settings.get("/physics/pickingForce")
        print(f"sharpness is {sharpness} and pickingForce is {pickingForce}")
        self.assertTrue(sharpness != stageSharpness and pickingForce != stagePickingForce)

        success = False

        async def do_steps(_):
            nonlocal success
            sharpness = settings.get("/rtx/post/aa/sharpness")
            pickingForce = settings.get("/physics/pickingForce")
            print(f"sharpness is {sharpness} and pickingForce is {pickingForce}")
            success = sharpness == stageSharpness and pickingForce == stagePickingForce
            await self.wait(60)
        
        demo.register("omni.physxdemos.tests.scenes")
        await demo.test("settingsDemoTest", None, do_steps)
        demo.unregister("omni.physxdemos.tests.scenes")
        self.assertTrue(success)

    async def test_physics_demos_direct_import(self):
        success = False
        async def do_steps(scene_class):
            print(scene_class.__name__)
            nonlocal success
            success = scene_class.__name__ == DirectImportTestDemo.__name__
            await self.wait(60)

        demo.register("omni.physxdemos.tests")
        await demo.test("omni.physxdemos.tests", None, do_steps)
        demo.unregister("omni.physxdemos.tests")
        self.assertTrue(success)


class PhysXVehicleTestDemos(PhysicsKitStageAsyncTestCase):
    category = TestCategory.Core

    async def test_physics_vehicle_demos(self):
        async def do_steps(scene_class):
            print(scene_class.__module__)
            await self.wait(60)
            await self.step(20)

        await demo.test("omni.physxvehicle", None, do_steps)


class PhysxXAssetDemosTest(PhysxDemoBase):
    async def test_physics_asset_demos(self):
        async def do_steps(scene_class):
            print(scene_class.__module__)
            await self.wait(60)
            await self.wait(240)

        assets_path = carb.settings.get_settings().get("physics/demoAssetsPath")
        print(f"Demo assets path is: {assets_path}")

        demos = ASSET_DEMOS
        for name in demos:
            with self.subTest(name) as include:
                if include:
                    await demo.test_demo(f"omni.physxdemos.scenes.{name}", None, do_steps)
