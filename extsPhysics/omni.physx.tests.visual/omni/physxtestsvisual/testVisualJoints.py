from omni.physx.scripts import physicsUtils
from omni.physxtests import utils
from omni.physxtestsvisual.utils import TestCase
from omni.physxtests.utils.physicsBase import TestCategory
from omni.kit.viewport.utility import frame_viewport_selection
import omni.kit.ui_test as ui_test
import omni.ui as ui
from pxr import Gf, UsdGeom, UsdPhysics, PhysxSchema
import carb.input
import carb.tokens
import omni.kit.app
from omni.physx.bindings._physx import (
    SETTING_DISPLAY_JOINTS,
)


class PhysxVisualJointsTest(TestCase):
    category = TestCategory.Core

    def create_cubes(self, stage):
        size = Gf.Vec3f(25.0)
        position = Gf.Vec3f(0.0, -100.0, 0.0)
        cube0 = physicsUtils.add_rigid_cube(stage, "/cubeActor0", size, position)
        rigidBodyAPI = UsdPhysics.RigidBodyAPI.Apply(cube0.GetPrim())
        rigidBodyAPI.CreateRigidBodyEnabledAttr(False)
        position = Gf.Vec3f(0.0, 100.0, 0.0)
        cube1 = physicsUtils.add_rigid_cube(stage, "/cubeActor1", size, position)

        self.body0prim = cube0.GetPrim()
        self.body1prim = cube1.GetPrim()

    def create_joint(self, stage, joint_type):
        if joint_type == "revolute":
            joint = UsdPhysics.RevoluteJoint.Define(self._stage, "/World/cubeActor0/revoluteJoint")
        elif joint_type == "prismatic":
            joint = UsdPhysics.PrismaticJoint.Define(self._stage, "/World/cubeActor0/prismaticJoint")
        elif joint_type == "fixed":
            joint = UsdPhysics.FixedJoint.Define(self._stage, "/World/cubeActor0/fixedJoint")
        elif joint_type == "d6":
            joint = UsdPhysics.Joint.Define(self._stage, "/World/cubeActor0/d6Joint")
        elif joint_type == "distance":
            joint = UsdPhysics.DistanceJoint.Define(self._stage, "/World/cubeActor0/distanceJoint")
        elif joint_type == "spherical":
            joint = UsdPhysics.SphericalJoint.Define(self._stage, "/World/cubeActor0/sphericalJoint")
        elif joint_type == "gear":
            joint = PhysxSchema.PhysxPhysicsGearJoint.Define(self._stage, "/World/cubeActor0/gearJoint")
        elif joint_type == "rack":
            joint = PhysxSchema.PhysxPhysicsRackAndPinionJoint.Define(self._stage, "/World/cubeActor0/rackJoint")
        val1 = [self.body0prim.GetPath()]
        val2 = [self.body1prim.GetPath()]
        joint.CreateBody0Rel().SetTargets(val1)
        joint.CreateBody1Rel().SetTargets(val2)
        joint.CreateLocalPos0Attr().Set(Gf.Vec3f(0.0, 4.0, 0.0))
        joint.CreateLocalPos1Attr().Set(Gf.Vec3f(0.0, -4.0, 0.0))
        self.joint = joint

    def setup_joint_scenario(self, stage, joint_type):
        self.create_cubes(stage)
        self.create_joint(stage, joint_type)

    async def test_physics_visual_joint_selection(self):
        stage = await self.new_stage()

        await self.setup_viewport_test()

        utils.physics_scene_setup(stage)
        self.setup_joint_scenario(stage, "revolute")

        await self.wait(1)

        usdContext = omni.usd.get_context()
        usdContext.get_selection().set_selected_prim_paths(["/World/cubeActor0/revoluteJoint"], True)

        await self.wait(20)

        await self.do_visual_test()
        await self.new_stage()

    async def test_physics_visual_joint_billboard(self):
        res = True

        for joint_type in ["revolute", "prismatic", "fixed", "d6", "distance", "spherical", "gear", "rack"]:
            stage = await self.new_stage()
            await self.setup_viewport_test()

            joint_vis = carb.settings.get_settings().get_as_bool(SETTING_DISPLAY_JOINTS)
            carb.settings.get_settings().set_bool(SETTING_DISPLAY_JOINTS, True)

            utils.physics_scene_setup(stage)
            self.setup_joint_scenario(stage, joint_type)

            res &= await self.do_visual_test(img_suffix=f"_{joint_type}", skip_assert=True, threshold=1e-5)

            carb.settings.get_settings().set_bool(SETTING_DISPLAY_JOINTS, joint_vis)
        self.assertTrue(res)
        await self.new_stage()

    async def test_physics_visual_joint_billboard_visibility(self):
        stage = await self.new_stage()

        await self.setup_viewport_test()

        utils.physics_scene_setup(stage)

        joint_vis = carb.settings.get_settings().get_as_bool(SETTING_DISPLAY_JOINTS)
        carb.settings.get_settings().set_bool(SETTING_DISPLAY_JOINTS, True)

        self.setup_joint_scenario(stage, "revolute")

        await omni.kit.app.get_app().next_update_async()

        usdContext = omni.usd.get_context()
        usdContext.get_selection().set_selected_prim_paths([], True)

        for _ in range(20):
            await omni.kit.app.get_app().next_update_async()

        imageable = UsdGeom.Imageable(self.joint)
        imageable.MakeInvisible()

        for _ in range(20):
            await omni.kit.app.get_app().next_update_async()

        await self.do_visual_test()       
        await self.new_stage()

        carb.settings.get_settings().set_bool(SETTING_DISPLAY_JOINTS, joint_vis)

    async def test_physics_visual_joint_billboard_parent_visibility(self):
        stage = await self.new_stage()

        await self.setup_viewport_test()

        utils.physics_scene_setup(stage)

        joint_vis = carb.settings.get_settings().get_as_bool(SETTING_DISPLAY_JOINTS)
        carb.settings.get_settings().set_bool(SETTING_DISPLAY_JOINTS, True)

        self.setup_joint_scenario(stage, "revolute")

        await omni.kit.app.get_app().next_update_async()

        usdContext = omni.usd.get_context()
        usdContext.get_selection().set_selected_prim_paths([], True)

        for _ in range(20):
            await omni.kit.app.get_app().next_update_async()

        print("Prim path: " + str(self.body0prim.GetPrimPath()))
        imageable = UsdGeom.Imageable(self.body0prim)
        imageable.MakeInvisible()

        for _ in range(20):
            await omni.kit.app.get_app().next_update_async()

        await self.do_visual_test()
        await self.new_stage()

        carb.settings.get_settings().set_bool(SETTING_DISPLAY_JOINTS, joint_vis)

    async def test_physics_visual_joint_align_workflow(self):
        stage = await self.new_stage()
        self.create_cubes(stage)
        SETTING_GIZMO_SCALE = "/persistent/app/viewport/gizmo/constantScale"
        joint_vis = carb.settings.get_settings().get_as_bool(SETTING_DISPLAY_JOINTS)
        gizmo_scale = carb.settings.get_settings().get_as_int(SETTING_GIZMO_SCALE)
        body_paths = [str(self.body0prim.GetPath()), str(self.body1prim.GetPath())]

        async def setup_it(body_id):
            await self.setup_viewport_test()
            carb.settings.get_settings().set_int(SETTING_GIZMO_SCALE, 30)
            carb.settings.get_settings().set_bool(SETTING_DISPLAY_JOINTS, True)
            omni.usd.get_context().get_selection().set_selected_prim_paths([body_paths[body_id]], True)
            await self.wait(10)
            frame_viewport_selection(force_legacy_api=True)
            await self.wait(10)

        def break_it():
            self.joint.GetLocalPos0Attr().Set(Gf.Vec3f(0.0, 0.0, 0.0))
            self.joint.GetLocalPos1Attr().Set(Gf.Vec3f(0.0, 0.0, 0.0))

        async def fix_it(identifier, text):
            window = ui.Workspace.get_window("Property")
            window.focus()
            omni.usd.get_context().get_selection().set_selected_prim_paths([str(self.joint.GetPrim().GetPath())], True)            
            await self.wait(2)

            pos0_widget = ui_test.find_all(f"Property//Frame/**/physprop_physics:{identifier}")[0]
            await pos0_widget.right_click()
            await ui_test.select_context_menu(f"Align {text}", offset=ui_test.Vec2(50, 5))          

            omni.usd.get_context().get_selection().set_selected_prim_paths([], True)
            await self.wait()

        async def test_it(suffix, joint_type):
            await self.wait(10)
            omni.usd.get_context().get_selection().set_selected_prim_paths([], True)
            await self.wait(10)

            return await self.do_visual_test(
                img_name=f"joint_align_workflow_{joint_type}",
                img_suffix=f"_{suffix}",
                threshold=1e-5,
                skip_assert=True
            )

        pos_fix_list = [
            ("localPos0", "Position to Body 0", 1),
            ("localPos1", "Position to Body 1", 0),
        ]

        tm_fix_list = [
            ("localPos0", "Transform to Body 0", 1),
            ("localRot0", "Transform to Body 0", 1),
            ("localPos1", "Transform to Body 1", 0),
            ("localRot1", "Transform to Body 1", 0),
        ]

        result = True

        for joint_type, fix_list in [
                ("revolute", pos_fix_list),
                ("spherical", pos_fix_list),
                ("fixed", tm_fix_list),
                ("prismatic", tm_fix_list),
        ]:
            self.create_joint(stage, joint_type)

            for identifier, text, select_body_id in fix_list:
                break_it()
                await setup_it(0)
                result &= await test_it(f"broken_{identifier}", joint_type)

                await fix_it(identifier, text)
                await setup_it(select_body_id)
                result &= await test_it(f"fixed_{identifier}", joint_type)

            stage.RemovePrim(self.joint.GetPrim().GetPrimPath())

        carb.settings.get_settings().set_bool(SETTING_DISPLAY_JOINTS, joint_vis)
        carb.settings.get_settings().set_int(SETTING_GIZMO_SCALE, gizmo_scale)
        await self.new_stage()
        self.assertTrue(result)
