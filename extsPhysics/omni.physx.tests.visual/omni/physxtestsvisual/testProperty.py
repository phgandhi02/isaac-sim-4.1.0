import carb
import unittest
import omni.kit.ui_test as ui_test
import omni.kit.window.property as p
import omni.ui as ui
import omni.usd
from omni.kit.property.physx import Manager, database, databaseUtils
from omni.kit.property.physx.widgets import MainFrameWidget, PhysicsMaterialBindingWidget
from omni.physx.scripts import particleUtils, physicsUtils
from omni.physxtests import utils
from omni.physxtests.tests import PhysicsUtils
from omni.physxtests.utils.physicsBase import TestCategory
from omni.physxtestsvisual.utils import TestCase
from pxr import Gf, PhysxSchema, UsdGeom, UsdPhysics, UsdShade


class PhysxPropertyTest(TestCase):
    category = TestCategory.Core

    async def _select_and_focus_property_window(self, paths):
        # select prim and focus property window so the widgets are built
        omni.usd.get_context().get_selection().set_selected_prim_paths(paths, False)
        window = ui.Workspace.get_window("Property")
        window.focus()
        await self.wait(20)

    async def _scroll_to_widget(self, widget_title):
        widget = p.get_window()._widgets_top[Manager.scheme][MainFrameWidget.name].get_subwidget_by_title(widget_title)
        widget._collapsable_frame.scroll_here_y()
        await self.wait(20)

    async def _test_physics_visual_component(self, prim, component_name):
        await self._select_and_focus_property_window([prim.GetPath().pathString])

        # resize window so that all components can fit without scrolling to get proper computed_height
        # (some invisible widgets are not counted towards the height)
        await self.setup_docked_test("Property", "Physics Debug", 790, 2048)
        await self.wait(3)

        # get target height and scroll offset and do a quickresize
        widget = p.get_window()._widgets_top[Manager.scheme][MainFrameWidget.name]
        target_height = widget._collapsable_frame.computed_height
        target_scroll = widget._collapsable_frame.screen_position_y

        self.force_resize_window("Property", 790, target_height)
        await self.wait(3)

        # and scroll and waaait for it to stabilize
        p.get_window()._window_frame.vertical_scrollbar_policy = ui.ScrollBarPolicy.SCROLLBAR_ALWAYS_OFF
        p.get_window()._window_frame.scroll_y = target_scroll
        await self.wait(20)

        # resize to correct width, this will refresh the scrollbar being off even with no-window
        self.force_resize_window("Property", 800, target_height)
        await self.wait(20)
        return await self.do_visual_test(img_suffix=component_name)

    # tries to find minimal set of prim+components needed to instantiate the component
    def _create_suitable_components(self, stage, component):
        try:
            cubePath = None

            prim = physicsUtils.add_capsule(stage, "/capsuleActor")

            # a bit of special treatment for tendons
            tendonAttachmentApis = {"PhysxTendonAttachmentAPI", "PhysxTendonAttachmentRootAPI", "PhysxTendonAttachmentLeafAPI"}
            if component.name in tendonAttachmentApis:
                api = getattr(PhysxSchema, component.name)
                if api is not None:
                    api.Apply(prim, component.name)

            databaseUtils.patch_refresh_cache_to_prim(prim)
            if not component.can_add(prim):
                utils.execute_and_check(self, "AddPhysicsComponent", usd_prim=prim, component="PhysicsRigidBodyAPI")
            else:
                return prim

            databaseUtils.patch_refresh_cache_to_prim(prim)
            if not component.can_add(prim):
                utils.execute_and_check(self, "AddPhysicsComponent", usd_prim=prim, component="PhysicsCollisionAPI")
            else:
                return prim

            databaseUtils.patch_refresh_cache_to_prim(prim)
            if not component.can_add(prim):
                path = omni.usd.get_stage_next_free_path(stage, "/particleSystem", True)
                particleSystem = particleUtils.add_physx_particle_system(stage, path)
                prim = particleSystem.GetPrim()
            else:
                return prim

            databaseUtils.patch_refresh_cache_to_prim(prim)
            if not component.can_add(prim):
                path = omni.usd.get_stage_next_free_path(stage, "/particleSet", True)
                particleSet = particleUtils.add_physx_particleset_points(stage, "/particleSet", [], [], [], "/particleSystem", self_collision=True, fluid=True, particle_group=0, particle_mass=0.001, density=0.0)
                prim = particleSet.GetPrim()
            else:
                return prim

            databaseUtils.patch_refresh_cache_to_prim(prim)
            if not component.can_add(prim):
                cubePath = utils.execute_and_check(self, "CreateMeshPrim", prim_type="Cube")
                prim = UsdGeom.Mesh.Get(stage, cubePath).GetPrim()
            else:
                return prim

            if component.name == "TriggerStateAPI":
                databaseUtils.patch_refresh_cache_to_prim(prim)
                if not component.can_add(prim):
                    utils.execute_and_check(self, "AddPhysicsComponent", usd_prim=prim, component="TriggerAPI")
                else:
                    return prim

            databaseUtils.patch_refresh_cache_to_prim(prim)
            if not component.can_add(prim):
                path = omni.usd.get_stage_next_free_path(stage, "/physicsMaterial", True)
                prim = UsdShade.Material.Define(stage, path).GetPrim()
            else:
                return prim

            databaseUtils.patch_refresh_cache_to_prim(prim)
            if not component.can_add(prim):
                path = omni.usd.get_stage_next_free_path(stage, "/joint", True)
                prim = UsdPhysics.Joint.Define(stage, path).GetPrim()
            else:
                return prim

            databaseUtils.patch_refresh_cache_to_prim(prim)
            if not component.can_add(prim):
                tendonApis = {"PhysxTendonAxisAPI", "PhysxTendonAxisRootAPI"}
                path = omni.usd.get_stage_next_free_path(stage, "/revoluteJoint", True)
                prim = UsdPhysics.RevoluteJoint.Define(stage, path).GetPrim()
                if component.name in tendonApis:
                    api = getattr(PhysxSchema, component.name)
                    if api is not None:
                        api.Apply(prim, component.name)
            else:
                return prim

            databaseUtils.patch_refresh_cache_to_prim(prim)
            if not component.can_add(prim):
                path = omni.usd.get_stage_next_free_path(stage, "/attachment", True)
                attachment = PhysxSchema.PhysxPhysicsAttachment.Define(stage, path)
                attachment.GetActor0Rel().SetTargets([cubePath])
                prim = attachment.GetPrim()
            else:
                return prim

            databaseUtils.patch_refresh_cache_to_prim(prim)
            if not component.can_add(prim):
                path = omni.usd.get_stage_next_free_path(stage, "/cloth", True)
                prim = UsdGeom.Mesh.Define(stage, path).GetPrim()
                PhysxSchema.PhysxParticleClothAPI.Apply(prim)
                PhysxSchema.PhysxAutoParticleClothAPI.Apply(prim)
            else:
                return prim

            databaseUtils.patch_refresh_cache_to_prim(prim)
            if not component.can_add(prim):
                meshIndices = [0]
                positions = [Gf.Vec3f(0.0)]
                orientations = [Gf.Quath(1.0)]
                linearVelocities = [Gf.Vec3f(0.0)]
                angularVelocities = [Gf.Vec3f(0.0)]
                path = omni.usd.get_stage_next_free_path(stage, "/pointInstancer", True)
                pointInstancer = UsdGeom.PointInstancer.Define(stage, path)
                meshList = pointInstancer.GetPrototypesRel()
                meshList.AddTarget(cubePath)
                pointInstancer.GetProtoIndicesAttr().Set(meshIndices)
                pointInstancer.GetPositionsAttr().Set(positions)
                pointInstancer.GetOrientationsAttr().Set(orientations)
                pointInstancer.GetVelocitiesAttr().Set(linearVelocities)
                pointInstancer.GetAngularVelocitiesAttr().Set(angularVelocities)
                prim = pointInstancer.GetPrim()
            else:
                return prim

            databaseUtils.patch_refresh_cache_to_prim(prim)
            if not component.can_add(prim):
                path = omni.usd.get_stage_next_free_path(stage, "/physicsScene", True)
                prim = UsdPhysics.Scene.Define(stage, path).GetPrim()
            else:
                return prim

        except Exception as e:
            errStr = f"Exception occurred while processing {component.name}: {e}!"
            carb.log_error(errStr)
            self.fail(errStr)
            prim = None

        return prim

    async def test_physics_visual_components(self):
        for component in database.components.values():
            with self.subTest(component.name) as include:
                if not include:
                    continue

                stage = await self.new_stage()

                # TODO FIXME MAData contain colons, skip them for now
                if component.name.find(":") != -1:
                    self.skipTest("Skipped")
                    continue

                # TODO FIXME TEMP HOTFIX - tendons are causing kit exit error code 3221226356
                if component.name in {"PhysxTendonAxisAPI", "PhysxTendonAxisRootAPI"}:
                    self.skipTest("Skipped")
                    continue

                # TODO FIXME - particle sampler is causing issues, fix as soon as we locked down particle components
                if component.name in {"PhysxParticleSamplingAPI", "PhysxParticleSetAPI", "PhysxParticleClothAPI"}:
                    self.skipTest("Skipped")
                    continue

                # TODO FIXME - 2px resolution diff from time to time :rage:
                if component.name == "PhysxDeformableBodyAPI":
                    self.skipTest("Skipped")
                    continue

                # auto-magic for creating the golden image does not work
                if component.name in {"PhysxVehicleSuspensionComplianceAPI", "PhysxVehicleSteeringAPI", "PhysxVehicleAckermannSteeringAPI"}:
                    self.skipTest("Skipped")
                    continue

                prim = self._create_suitable_components(stage, component)
                databaseUtils.patch_refresh_cache_to_prim(prim)
                if not component.can_add(prim):
                    errStr = f"Didn't find a way how to test {component.name}!"
                    carb.log_error(errStr)
                    self.fail(errStr)
                    continue

                if component.add_component_fn is not None:
                    component.add_component_fn(usd_prim=prim, component=component.name)
                else:
                    utils.execute_and_check(self, "AddPhysicsComponent", usd_prim=prim, component=component.name)

                await self.wait(20)
                self.assertTrue(await self._test_physics_visual_component(prim, "_" + component.name.lower()))

    async def test_physics_ui_rigid_cube_properties(self):
        stage = await self.new_stage()
        prim = physicsUtils.add_rigid_cube(stage, "/cubeActor", Gf.Vec3f(100.0))

        await self._select_and_focus_property_window([prim.GetPath().pathString])

        # find main Physics frame
        physics_frame = ui_test.find("Property//Frame/**/CollapsableFrame[*].title=='Physics'")

        # NOTE: OmniUIQuery.find_widgets does not support Frame inside an inner path, replacing by *
        top_stack = physics_frame.find("*/ZStack[0]/VStack[0]/*/VStack[0]")
        component_frames = top_stack.find_all("*/ZStack[0]/CollapsableFrame[*]")
        other_frames = top_stack.find_all("CollapsableFrame[*]")

        titles = [t.widget.title for t in component_frames + other_frames]

        # expect Rigid Body, Collider and Mass component frames and a Physics material binding frame
        expected_titles = [
            database.components[UsdPhysics.RigidBodyAPI.__name__].title,
            database.components[UsdPhysics.CollisionAPI.__name__].title,
            database.components[UsdPhysics.MassAPI.__name__].title,
            PhysicsMaterialBindingWidget.title,
        ]

        self.assertTrue(all(e in expected_titles for e in titles))

    async def test_physics_ui_add_rigid_component(self):
        stage = await self.new_stage()
        prim = physicsUtils.add_cube(stage, "/cubeActor", Gf.Vec3f(100.0))

        self.assertTrue(not prim.HasAPI(UsdPhysics.RigidBodyAPI))

        await self._select_and_focus_property_window([prim.GetPath().pathString])

        # add rigidbody component through the Add button
        add_button = ui_test.find("Property//Frame/**/Button[0].name=='add'")
        await add_button.click()
        await ui_test.select_context_menu("Physics/Rigid Body", offset=ui_test.Vec2(50, 5))

        self.assertTrue(prim.HasAPI(UsdPhysics.RigidBodyAPI))

    async def test_physics_ui_joint_euler_quat(self):
        from carb.input import KeyboardInput
        stage = await self.new_stage()

        path = "/Joint"
        joint = UsdPhysics.Joint.Define(stage, path)
        await self._select_and_focus_property_window([path])

        euler_widget = ui_test.find("Property//Frame/**/MultiFloatDragField[*].identifier=='physprop_physics:localRot0'")

        prev = joint.GetLocalRot0Attr().Get()

        await euler_widget.double_click(human_delay_speed=10)
        for s in "180":
            await ui_test.emulate_char_press(s)
            await ui_test.wait_n_updates(10)
        await ui_test.emulate_keyboard_press(KeyboardInput.ENTER)

        curr = joint.GetLocalRot0Attr().Get()
        self.assertNotEqual(prev, curr)

    @unittest.skip("Hair component hidden")
    async def test_physics_ui_add_hair_component_mesh(self):
        stage = await self.new_stage()
        prim = physicsUtils.create_mesh_cube(stage, "/meshActor", 1.0).GetPrim()

        self.assertFalse(prim.HasAPI(PhysxSchema.PhysxHairAPI))

        await self._select_and_focus_property_window([prim.GetPath().pathString])

        # add hair component through the Add button
        add_button = ui_test.find("Property//Frame/**/Button[0].name=='add'")
        await add_button.click()
        await ui_test.select_context_menu("Physics/Hair", offset=ui_test.Vec2(50, 5))

        self.assertTrue(prim.HasAPI(PhysxSchema.PhysxHairAPI))

    @unittest.skip("Hair component hidden")
    async def test_physics_ui_add_hair_component_mesh_subset(self):
        stage = await self.new_stage()
        mesh = physicsUtils.create_mesh_cube(stage, "/meshActor", 1.0)
        prim = PhysicsUtils.create_mesh_face_subset(mesh, [0, 1]).GetPrim()

        self.assertFalse(prim.HasAPI(PhysxSchema.PhysxHairAPI))

        await self._select_and_focus_property_window([prim.GetPath().pathString])

        # add hair component through the Add button
        add_button = ui_test.find("Property//Frame/**/Button[0].name=='add'")
        await add_button.click()
        await ui_test.select_context_menu("Physics/Hair", offset=ui_test.Vec2(30, 5))

        self.assertTrue(prim.HasAPI(PhysxSchema.PhysxHairAPI))

    @unittest.skip("Hair component hidden")
    async def test_physics_ui_add_hair_component_basiscurves(self):
        stage = await self.new_stage()
        prim = PhysicsUtils.create_hair_basiscurves(stage, "/curvesActor", 2, 8, Gf.Vec3f(1.0, 0.0, 0.0), Gf.Vec3f(0.1, 0.0, 0.0)).GetPrim()

        self.assertFalse(prim.HasAPI(PhysxSchema.PhysxHairAPI))

        await self._select_and_focus_property_window([prim.GetPath().pathString])

        # add hair component through the Add button
        add_button = ui_test.find("Property//Frame/**/Button[0].name=='add'")
        await add_button.click()
        await ui_test.select_context_menu("Physics/Hair", offset=ui_test.Vec2(50, 5))

        self.assertTrue(prim.HasAPI(PhysxSchema.PhysxHairAPI))

    @unittest.skip("Hair component hidden")
    async def test_physics_ui_add_hair_material(self):
        stage = await self.new_stage()

        # Click through the menu to create a physics hair material
        menu_widget = ui_test.get_menubar()
        await menu_widget.find_menu("Create").click()
        await menu_widget.find_menu("Physics").click()
        await menu_widget.find_menu("Physics Material").click()

        # the name of the dialog is 'Test'. We need to find the label, then click on the corresponding checkbox
        checkboxLabel = ui_test.find("Test//Frame/**/Label[0].text=='Hair Material'")
        await ui_test.emulate_mouse_move_and_click(checkboxLabel.position + ui_test.Vec2(-18, 5))

        ok_button = ui_test.find("Test//Frame/**/Button[0].text=='Ok'")
        await ok_button.click()

        prim = stage.GetPrimAtPath("/World/PhysicsMaterial")
        self.assertTrue(prim.HasAPI(PhysxSchema.PhysxHairMaterialAPI))

    async def test_physics_mainframe_visibility(self):
        stage = await self.new_stage()

        cube = physicsUtils.add_cube(stage, "/cube", Gf.Vec3f(100.0))

        await self._select_and_focus_property_window([cube.GetPath().pathString])
        physics_frame = ui_test.find("Property//Frame/**/CollapsableFrame[*].title=='Physics'")
        self.assertFalse(physics_frame.widget.visible)

        rigid_cube = physicsUtils.add_rigid_cube(stage, "/rigidCube", Gf.Vec3f(100.0))

        await self._select_and_focus_property_window([rigid_cube.GetPath().pathString])
        physics_frame = ui_test.find("Property//Frame/**/CollapsableFrame[*].title=='Physics'")
        self.assertTrue(physics_frame.widget.visible)

    async def test_physics_remove_component_button(self):
        stage = await self.new_stage()
        prim = physicsUtils.add_rigid_cube(stage, "/cubeActor", Gf.Vec3f(100.0))

        self.assertTrue(prim.HasAPI(UsdPhysics.RigidBodyAPI))

        await self._select_and_focus_property_window([prim.GetPath().pathString])
        await self._scroll_to_widget("Rigid Body")

        # remove rigidbody component through the Add button
        remove_button = ui_test.find("Property//Frame/**/Button[*].identifier=='remove_component Rigid Body'")
        self.assertIsNotNone(remove_button)
        await remove_button.click()
        yes_button = ui_test.find("Remove component?//Frame/**/Button[*].text=='Yes'")
        self.assertIsNotNone(yes_button)
        await yes_button.click()
        self.assertTrue(not prim.HasAPI(UsdPhysics.RigidBodyAPI))
