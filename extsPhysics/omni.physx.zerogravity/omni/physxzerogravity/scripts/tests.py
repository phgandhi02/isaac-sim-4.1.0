import math
import os
import unittest

import carb
import omni.ext
import omni.kit.app
import omni.kit.commands
import omni.kit.test
import omni.kit.ui_test as ui_test
import omni.timeline
from omni.physx.scripts import physicsUtils
from omni.physx.scripts import utils as baseUtils
from omni.physxtests import utils
from omni.physxtests.utils.physicsBase import PhysicsKitStageAsyncTestCase, PhysicsBaseAsyncTestCase
from omni.physxzerogravity import get_physx_zero_gravity_interface
from omni.physxzerogravity.scripts.action_bar import ButtonDef
from omni.physxzerogravity.scripts.constants import ZeroGravityConstants
from omni.physxzerogravity.scripts.settings import ZeroGravitySettings
from omni.physxzerogravity.scripts.toolbar_button import ToolButtonGroup
from pxr import Gf, Sdf, Usd, UsdGeom, UsdPhysics

SETTINGS_PLACEMENT_MODE_ENABLED = "/physics/placementModeEnabled"
SETTINGS_PLACEMENT_MODE_LAYER_IDENTIFIER = "/physics/placementModeLayerIdentifier"

SETTINGS_RATE_LIMIT_ENABLED = "/app/runLoops/main/rateLimitEnabled"
SETTINGS_RATE_LIMIT_FREQ = "/app/runLoops/main/rateLimitFrequency"

PHYSICS_FREQ = 60
DROP_WAIT_FRAMES = 280
ENABLED_WAIT_FRAMES = 10
ACTION_WAIT_FRAMES = 5

GEOMETRY = [
    "Cube",
    "Sphere",
    "Cylinder",
    "Capsule",
    "Cone",
    "ConeMesh",
    "CubeMesh",
    "CylinderMesh",
    "DiskMesh",
    "PlaneMesh",
    "SphereMesh",
    "TorusMesh",
]

DROP_POS = [
    52.5,
    52.5,
    52.5,
    52.5,
    52.5,
    52.5,
    52.5,
    52.5,
    40.13,
    40.13,
    51.1,
    46.15
]


def _euler_to_quaternion(euler_angles):
    """
        Converts euler angles in quaternion

        Args:
            euler_angles: iterable list of euler angles in degrees

        Returns:
            the Gf.Quatf quaternion representation
    """
    euler_radians = [angle for angle in euler_angles]
    axes = [Gf.Vec3d(axis) for axis in [(1, 0, 0), (0, 1, 0), (0, 0, 1)]]
    quaternion = Gf.Quatf(1.0, 0.0, 0.0, 0.0)
    for angle, axis in zip(euler_radians, axes):
        rotation = Gf.Rotation(axis, angle)
        quaternion *= Gf.Quatf(rotation.GetQuat())
    return quaternion


def _quaternion_to_euler(quaternion):
    """
        Converts quaternion to euler angles

        Args:
            quaternion: a Gf.Quatf quaternion

        Returns:
            a list of Euler angles in degrees
    """
    rotation = Gf.Rotation(quaternion)
    axes = [Gf.Vec3d.XAxis(), Gf.Vec3d.YAxis(), Gf.Vec3d.ZAxis()]
    euler_angles = rotation.Decompose(axes[0], axes[1], axes[2])
    return euler_angles


class ZeroGravityTests(PhysicsKitStageAsyncTestCase):
    async def setUp(self):
        await self._disable_zerog()  # make sure zerog is inactive
        await utils.new_stage_setup()  # Cleanup stage to avoid crashes due to memory not being deallocated correctly
        await super().setUp()
        self._rate_limit_freq_enabled = carb.settings.get_settings().get(SETTINGS_RATE_LIMIT_ENABLED)
        self._rate_limit_freq = carb.settings.get_settings().get(SETTINGS_RATE_LIMIT_FREQ)
        carb.settings.get_settings().set_bool(SETTINGS_RATE_LIMIT_ENABLED, int(True))
        carb.settings.get_settings().set_int(SETTINGS_RATE_LIMIT_FREQ, int(PHYSICS_FREQ))

        self._button_defs = dict()
        ButtonDef.define_buttons(self._button_defs, '')

    async def tearDown(self):
        self._button_defs = None
        await super().tearDown()
        carb.settings.get_settings().set_bool(SETTINGS_RATE_LIMIT_ENABLED, int(self._rate_limit_freq_enabled))
        carb.settings.get_settings().set_int(SETTINGS_RATE_LIMIT_FREQ, int(self._rate_limit_freq))

    async def _load_usd(self, filename):
        data_path = "../../../../data/tests/ZeroGravityTests"
        tests_folder = os.path.abspath(os.path.normpath(os.path.join(__file__, data_path)))
        tests_folder = tests_folder.replace("\\", "/") + "/"
        filepath = tests_folder + filename + ".usda"
        await omni.usd.get_context().open_stage_async(filepath)  # Note: this OPENS a new stage, close it afterwards!
        self._usd_context = omni.usd.get_context()
        self.assertIn(filename, self._usd_context.get_stage_url())
        self._stage = self._usd_context.get_stage()

    async def create_simple_test_stage(self):
        cube_size = 10
        ground_plane_size = 50
        vertical_offset = 25
        stage = await utils.new_stage_setup()
        up_axis = UsdGeom.GetStageUpAxis(stage)
        up_offset = baseUtils.getAxisAlignedVector(up_axis, vertical_offset)
        physicsUtils.add_cube(stage, "/Cube0", Gf.Vec3f(cube_size), up_offset)
        up_offset = baseUtils.getAxisAlignedVector(up_axis, 15)
        physicsUtils.add_rigid_cube(stage, "/CubeActor0", Gf.Vec3f(cube_size), up_offset)
        physicsUtils.add_ground_plane(stage, "/GroundPlane", up_axis, ground_plane_size, Gf.Vec3f(0.0), Gf.Vec3f(0.5))
        utils.physics_scene_setup(stage)
        self._stage = stage
        self.assertIsNotNone(self._stage)

    async def _close_stage(self):
        omni.usd.get_context().get_selection().clear_selected_prim_paths()
        self._stage = None
        await omni.usd.get_context().close_stage_async()
        await self.wait(ACTION_WAIT_FRAMES)

    async def _enable_zerog(self):
        '''We will not randomly wait for zerog init complete -
        when custom manipulator is ON, zerog placement mode is initialized'''
        omni.kit.commands.execute("ZeroGravitySetEnabled", enabled=True)
        on = False
        while on is False:
            await self.wait(1)
            on = carb.settings.get_settings().get_as_bool(ZeroGravitySettings.SETTINGS_CUSTOM_MANIPULATOR_ENABLED)

    async def _disable_zerog(self):
        omni.kit.commands.execute("ZeroGravitySetEnabled", enabled=False)
        off = False
        while off is False:
            await self.wait(1)
            off = not carb.settings.get_settings().get_as_bool(ZeroGravitySettings.SETTINGS_CUSTOM_MANIPULATOR_ENABLED)

    def _zerog_flush_all_changes(self):
        omni.kit.commands.execute("ZeroGravityFlushAllChanges")

    def _find_zerog_toolbar(self):
        toolbar = omni.ui.Workspace.get_window(ZeroGravityConstants.EXTENSION_DISPLAY_NAME)
        self.assertTrue(toolbar is not None)
        return toolbar

    async def _test_ui_icon(self, on_cmd, off_cmd, on_tooltip, off_tooltip, on_icon_name, off_icon_name):
        await self._enable_zerog()

        # execute the ON command
        on_cmd()
        await self.wait(ENABLED_WAIT_FRAMES)

        button = ui_test.find(
            f"{ZeroGravityConstants.EXTENSION_DISPLAY_NAME}//Frame/**/Button[*].tooltip=='{on_tooltip}'"
        )
        self.assertTrue(button)
        self.assertTrue(button._widget.enabled)
        self.assertTrue(on_icon_name in button._widget.image_url)

        # execute the OFF command
        off_cmd()
        await self.wait(ENABLED_WAIT_FRAMES)

        button = ui_test.find(
            f"{ZeroGravityConstants.EXTENSION_DISPLAY_NAME}//Frame/**/Button[*].tooltip=='{off_tooltip}'"
        )
        self.assertTrue(button)
        self.assertTrue(button._widget.enabled)
        self.assertTrue(off_icon_name in button._widget.image_url)

        await self._disable_zerog()

    async def test_ui_zerog_main_icon(self):
        toolbar = omni.kit.window.toolbar.get_instance()
        self.assertTrue(toolbar is not None)

        widget_instance = toolbar

        if hasattr(toolbar, '_widget_instance') and toolbar._widget_instance is not None:
            widget_instance = toolbar._widget_instance

        for _, widget_group in widget_instance._toolbar_widget_groups:
            if isinstance(widget_group, ToolButtonGroup):
                tool_button_group = widget_group
                break

        self.assertTrue(tool_button_group is not None)

        await self._enable_zerog()
        self.assertTrue(tool_button_group._enabled)
        self.assertTrue(tool_button_group._enabled_button.visible)
        self.assertTrue(tool_button_group._enabled_button.enabled)
        self.assertTrue(tool_button_group._enabled_button.checked)

        await self._disable_zerog()
        self.assertTrue(tool_button_group._enabled)
        self.assertTrue(tool_button_group._enabled_button.visible)
        self.assertTrue(tool_button_group._enabled_button.enabled)
        self.assertTrue(not tool_button_group._enabled_button.checked)

    async def test_ui_zerog_sweep_icon(self):
        await self._enable_zerog()
        await self._test_ui_icon(
            lambda: omni.kit.commands.execute("ZeroGravitySetSweepModeCommand", sweep_mode=True),
            lambda: omni.kit.commands.execute("ZeroGravitySetSweepModeCommand", sweep_mode=False),
            self._button_defs[ButtonDef.BUTTON_SWEEP_STOP][1],
            self._button_defs[ButtonDef.BUTTON_SWEEP_START][1],
            self._button_defs[ButtonDef.BUTTON_SWEEP_STOP][0],
            self._button_defs[ButtonDef.BUTTON_SWEEP_START][0],
        )
        await self._disable_zerog()

    async def test_ui_zerog_drop_icon(self):
        await self._enable_zerog()
        await self._test_ui_icon(
            lambda: omni.kit.commands.execute("ZeroGravitySetDropping", dropping=True),
            lambda: omni.kit.commands.execute("ZeroGravitySetDropping", dropping=False),
            self._button_defs[ButtonDef.BUTTON_DROP_STOP][1],
            self._button_defs[ButtonDef.BUTTON_DROP_START][1],
            self._button_defs[ButtonDef.BUTTON_DROP_STOP][0],
            self._button_defs[ButtonDef.BUTTON_DROP_START][0],
        )
        await self._disable_zerog()

    async def test_enabled(self):
        settings = carb.settings.get_settings()

        # load a stage with markers and enable zero gravity (placementmode)
        await self._load_usd("NoPhysicsWithMarkers")
        await self._enable_zerog()

        self._zerog_flush_all_changes()  # The NoPhysicsWithMarkers file has markers baked in the USD, wait for restore

        # check the floor plane has collision api now on it
        placement_layer_identifier = settings.get_as_string(SETTINGS_PLACEMENT_MODE_LAYER_IDENTIFIER)
        placement_layer = Sdf.Layer.Find(placement_layer_identifier)
        with Usd.EditContext(self._stage, placement_layer):
            plane_prim = self._stage.GetPrimAtPath("/World/Plane")
            has_api = plane_prim.HasAPI(UsdPhysics.CollisionAPI)
            self.assertTrue(has_api)
            # note: static objects will *not* have a rigid body so this is not needed
            # has_api = plane_prim.HasAPI(UsdPhysics.RigidBodyAPI)
            # self.assertTrue(has_api)
            for geometry in GEOMETRY:
                geometry_prim = self._stage.GetPrimAtPath(f"/World/{geometry}")
                has_api = geometry_prim.HasAPI(UsdPhysics.RigidBodyAPI)
                self.assertTrue(has_api)
                has_api = geometry_prim.HasAPI(UsdPhysics.CollisionAPI)
                self.assertTrue(has_api)
                await self.wait(1)

        await self._disable_zerog()
        await self._close_stage()

    # Test dropping _already marked_ prims. The prims are already marked (they embed
    # placement mode specific metadata) and have cooked colliders already in the USD file.
    # As soon as you select them and click 'drop', they should drop on the statically-marked
    # plane.
    async def test_drop(self):
        await self._load_usd("NoPhysicsWithMarkers")
        await self._enable_zerog()
        omni.kit.commands.execute("ZeroGravitySetSweepMode", sweep_mode=False)

        selected_paths = []
        for geometry in GEOMETRY:
            selected_paths.append(f"/World/{geometry}")
        self._usd_context.get_selection().set_selected_prim_paths(selected_paths, True)
        omni.kit.commands.execute("ZeroGravitySetDropping", dropping=True)
        await self.wait(DROP_WAIT_FRAMES)
        omni.kit.commands.execute("ZeroGravitySetDropping", dropping=False)
        await self.wait(ACTION_WAIT_FRAMES)
        i = 0
        for geometry in GEOMETRY:
            # FIXME(malesiani): these two won't work for now since we're applying forces
            # to drop objects, fix it when PX-4096 has a resolution
            if geometry == "Cylinder" or geometry == "Cone":
                i = i + 1
                continue
            geometry_prim = self._stage.GetPrimAtPath(f"/World/{geometry}")
            pos = geometry_prim.GetAttribute("xformOp:translate").Get()
            self.assertAlmostEqual(pos[1], DROP_POS[i], delta=0.1)
            i = i + 1

        await self._disable_zerog()
        await self._close_stage()

    async def test_set_markers_and_check(self):
        await self._load_usd("NoPhysicsNoMarkers")
        await self._enable_zerog()

        omni.kit.commands.execute("ZeroGravitySetSweepMode", sweep_mode=False)
        await self.wait(ENABLED_WAIT_FRAMES)
        self._usd_context.get_selection().set_selected_prim_paths(["/World/Plane"], True)
        omni.kit.commands.execute("ZeroGravitySetSelectedStatic")
        await self.wait(ACTION_WAIT_FRAMES)
        self._usd_context.get_selection().set_selected_prim_paths(["/World/Plane"], False)
        selected_paths = []
        for geometry in GEOMETRY:
            selected_paths.append(f"/World/{geometry}")
        self._usd_context.get_selection().set_selected_prim_paths(selected_paths, True)
        omni.kit.commands.execute("ZeroGravitySetSelectedDynamic")
        await self.wait(ACTION_WAIT_FRAMES)
        # check the floor plane has collision api now on it
        settings = carb.settings.get_settings()
        placement_layer_identifier = settings.get_as_string(SETTINGS_PLACEMENT_MODE_LAYER_IDENTIFIER)
        placement_layer = Sdf.Layer.Find(placement_layer_identifier)
        with Usd.EditContext(self._stage, placement_layer):
            plane_prim = self._stage.GetPrimAtPath("/World/Plane")
            has_api = plane_prim.HasAPI(UsdPhysics.CollisionAPI)
            self.assertTrue(has_api)
            # note that static objects *do not* have rigid bodies on them.. /World/Plane shouldn't have it
            has_api = plane_prim.HasAPI(UsdPhysics.RigidBodyAPI)
            self.assertFalse(has_api)
            for geometry in GEOMETRY:
                geometry_prim = self._stage.GetPrimAtPath(f"/World/{geometry}")
                has_api = geometry_prim.HasAPI(UsdPhysics.RigidBodyAPI)
                self.assertTrue(has_api)
                has_api = geometry_prim.HasAPI(UsdPhysics.CollisionAPI)
                self.assertTrue(has_api)

        await self._disable_zerog()
        await self._close_stage()

    async def test_set_markers_and_drop(self):
        await self._load_usd("NoPhysicsNoMarkers")
        await self._enable_zerog()

        omni.kit.commands.execute("ZeroGravitySetSweepMode", sweep_mode=False)

        self._usd_context.get_selection().set_selected_prim_paths(["/World/Plane"], True)
        omni.kit.commands.execute("ZeroGravitySetSelectedStatic")
        await self.wait(ACTION_WAIT_FRAMES)
        self._usd_context.get_selection().set_selected_prim_paths(["/World/Plane"], False)
        selected_paths = []
        for geometry in GEOMETRY:
            selected_paths.append(f"/World/{geometry}")
        self._usd_context.get_selection().set_selected_prim_paths(selected_paths, True)
        omni.kit.commands.execute("ZeroGravitySetSelectedDynamic")
        await self.wait(ACTION_WAIT_FRAMES)
        omni.kit.commands.execute("ZeroGravitySetDropping", dropping=True)
        await self.wait(DROP_WAIT_FRAMES)
        omni.kit.commands.execute("ZeroGravitySetDropping", dropping=False)
        await self.wait(ACTION_WAIT_FRAMES)
        i = 0
        for geometry in GEOMETRY:
            # FIXME(malesiani): these two won't work for now since we're applying forces
            # to drop objects, fix it when PX-4096 has a resolution
            if geometry == "Cylinder" or geometry == "Cone":
                i = i + 1
                continue
            geometry_prim = self._stage.GetPrimAtPath(f"/World/{geometry}")
            pos = geometry_prim.GetAttribute("xformOp:translate").Get()
            self.assertAlmostEqual(pos[1], DROP_POS[i], delta=0.1, msg=f"got a wrong ground pos for geometry {geometry}")
            i = i + 1
        omni.kit.commands.execute("ZeroGravityClearAll")

        await self._disable_zerog()
        await self._close_stage()

    async def test_clear_and_drop(self):
        await self._load_usd("ClearWithMarkers")
        await self._enable_zerog()

        omni.kit.commands.execute("ZeroGravitySetSweepMode", sweep_mode=False)
        await self.wait(ENABLED_WAIT_FRAMES)
        self._usd_context.get_selection().set_selected_prim_paths(["/World/CubeMesh", "/World/CubeMesh_01", "/World/CubeMesh_02"], True)
        omni.kit.commands.execute("ZeroGravityClearSelected")
        await self.wait(ACTION_WAIT_FRAMES)
        self._usd_context.get_selection().set_selected_prim_paths(["/World/CubeMesh", "/World/CubeMesh_01", "/World/CubeMesh_02"], False)
        self._usd_context.get_selection().set_selected_prim_paths(["/World/CylinderMesh"], True)
        omni.kit.commands.execute("ZeroGravitySetDropping", dropping=True)
        await self.wait(DROP_WAIT_FRAMES)
        omni.kit.commands.execute("ZeroGravitySetDropping", dropping=False)
        for _ in range(ACTION_WAIT_FRAMES):
            cylinder_prim2 = self._stage.GetPrimAtPath("/World/CylinderMesh")
            pos2 = cylinder_prim2.GetAttribute("xformOp:translate").Get()
            await omni.kit.app.get_app().next_update_async()
        cylinder_prim = self._stage.GetPrimAtPath("/World/CylinderMesh")
        pos = cylinder_prim.GetAttribute("xformOp:translate").Get()
        self.assertAlmostEqual(pos[1], 45.0, delta=0.1)

        await self._disable_zerog()
        await self._close_stage()

    async def test_clear_all_and_drop(self):
        await self._load_usd("ClearWithMarkers")
        await self._enable_zerog()

        omni.kit.commands.execute("ZeroGravitySetSweepMode", sweep_mode=False)
        await self.wait(ENABLED_WAIT_FRAMES)
        omni.kit.commands.execute("ZeroGravityClearAll")
        await self.wait(ACTION_WAIT_FRAMES)
        self._usd_context.get_selection().set_selected_prim_paths(["/World/CylinderMesh"], True)
        omni.kit.commands.execute("ZeroGravitySetDropping", dropping=True)
        await self.wait(ACTION_WAIT_FRAMES)
        omni.kit.commands.execute("ZeroGravitySetDropping", dropping=False)
        await self.wait(ACTION_WAIT_FRAMES)
        cylinder_prim = self._stage.GetPrimAtPath("/World/CylinderMesh")
        pos = cylinder_prim.GetAttribute("xformOp:translate").Get()
        self.assertAlmostEqual(pos[1], 60.0, delta=0.1)

        await self._disable_zerog()
        await self._close_stage()

    async def test_clear_all_mark_and_drop(self):
        await self._load_usd("ClearWithMarkers")
        await self._enable_zerog()

        omni.kit.commands.execute("ZeroGravitySetSweepMode", sweep_mode=False)
        await self.wait(ENABLED_WAIT_FRAMES)
        omni.kit.commands.execute("ZeroGravityClearAll")
        await self.wait(ACTION_WAIT_FRAMES)
        self._usd_context.get_selection().set_selected_prim_paths(["/World/Plane"], True)
        omni.kit.commands.execute("ZeroGravitySetSelectedStatic")
        await self.wait(ACTION_WAIT_FRAMES)
        self._usd_context.get_selection().set_selected_prim_paths(["/World/Plane"], False)
        self._usd_context.get_selection().set_selected_prim_paths(["/World/CylinderMesh"], True)
        omni.kit.commands.execute("ZeroGravitySetSelectedDynamic")
        await self.wait(ACTION_WAIT_FRAMES)
        omni.kit.commands.execute("ZeroGravitySetDropping", dropping=True)
        await self.wait(DROP_WAIT_FRAMES)
        omni.kit.commands.execute("ZeroGravitySetDropping", dropping=False)
        await self.wait(ACTION_WAIT_FRAMES)
        cylinder_prim = self._stage.GetPrimAtPath("/World/CylinderMesh")
        pos = cylinder_prim.GetAttribute("xformOp:translate").Get()
        self.assertAlmostEqual(pos[1], 45.0, delta=0.1)

        await self._disable_zerog()
        await self._close_stage()

    async def test_dynamic_drop(self):
        await self._load_usd("DynamicWithMarkers")
        await self._enable_zerog()

        omni.kit.commands.execute("ZeroGravitySetSweepMode", sweep_mode=False)
        await self.wait(ENABLED_WAIT_FRAMES)
        self._usd_context.get_selection().set_selected_prim_paths(["/World/CylinderMesh"], True)
        omni.kit.commands.execute("ZeroGravitySetDropping", dropping=True)
        await self.wait(DROP_WAIT_FRAMES)
        omni.kit.commands.execute("ZeroGravitySetDropping", dropping=False)
        await self.wait(ACTION_WAIT_FRAMES)
        prim = self._stage.GetPrimAtPath("/World/CylinderMesh")
        pos = prim.GetAttribute("xformOp:translate").Get()
        self.assertAlmostEqual(pos[1], 150.0, delta=1.0)

        await self._disable_zerog()
        await self._close_stage()

    async def test_physics_listener_disabled(self):
        stage = omni.usd.get_context().get_stage()
        dynamicPrim = physicsUtils.add_rigid_box(stage, "/dynamicActor")
        await self._enable_zerog()

        omni.usd.get_context().get_selection().set_selected_prim_paths([str(dynamicPrim.GetPrimPath())], True)
        omni.kit.commands.execute("ZeroGravitySetSelectedDynamic")

        await self.wait(ENABLED_WAIT_FRAMES)

        cube = physicsUtils.add_rigid_box(stage, "/cubeActor")

        omni.usd.get_context().get_selection().set_selected_prim_paths([str(cube.GetPrimPath())], True)
        omni.kit.commands.execute("ZeroGravitySetSelectedDynamic")

        fixedJoint = UsdPhysics.FixedJoint.Define(stage, "/World/fixedJoint")
        fixedJoint.CreateBody0Rel().SetTargets([cube.GetPrimPath()])

        await self.step(ENABLED_WAIT_FRAMES)
        omni.kit.commands.execute("ZeroGravityWaitForSimulationStepCompletedCommand")
        utils.check_stats(self, {"numBoxShapes": 2, "numDynamicRigids": 2, "numConstraints": 1})

        await self._disable_zerog()
        await self._close_stage()

    async def test_physics_clear_markers(self):
        stage = omni.usd.get_context().get_stage()

        dynamicPrim = physicsUtils.add_rigid_box(stage, "/dynamicActor")
        cube = physicsUtils.add_cube(stage, "/cubeActor")

        await self.step(ENABLED_WAIT_FRAMES)
        omni.kit.commands.execute("ZeroGravityWaitForSimulationStepCompletedCommand")
        utils.check_stats(self, {"numBoxShapes": 1, "numDynamicRigids": 1})
        await self.stop()

        # make sure sweep mode is inactive now
        omni.kit.commands.execute("ZeroGravitySetSweepMode", sweep_mode=False)
        # enable zero gravity
        await self._enable_zerog()

        # ZG should have cleared rigid body but not the original collider from the dynamicActor box
        omni.kit.commands.execute("ZeroGravityWaitForSimulationStepCompletedCommand")
        utils.check_stats(self, {"numBoxShapes": 1})
        utils.check_stats(self, {"numDynamicRigids": 0})

        # mark one cube as dynamic, the other as static
        omni.usd.get_context().get_selection().set_selected_prim_paths([str(dynamicPrim.GetPrimPath())], True)
        await self.wait(ACTION_WAIT_FRAMES)
        omni.kit.commands.execute("ZeroGravitySetSelectedDynamic")

        omni.usd.get_context().get_selection().set_selected_prim_paths([str(cube.GetPrimPath())], True)
        await self.wait(ACTION_WAIT_FRAMES)
        omni.kit.commands.execute("ZeroGravitySetSelectedStatic")

        await self.wait(ENABLED_WAIT_FRAMES)
        omni.kit.commands.execute("ZeroGravityWaitForSimulationStepCompletedCommand")
        utils.check_stats(self, {"numBoxShapes": 2, "numDynamicRigids": 1, "numKinematicBodies": 0})

        await self._disable_zerog()

        # enable zero gravity again, markers should still be set and restored from USD
        await self._enable_zerog()

        omni.kit.commands.execute("ZeroGravityWaitForSimulationStepCompletedCommand")
        utils.check_stats(self, {"numBoxShapes": 2, "numDynamicRigids": 1, "numKinematicBodies": 0})

        # clear all the markers
        get_physx_zero_gravity_interface().clear_all_markers()

        await self.wait(ENABLED_WAIT_FRAMES)

        # now see if we're just left with the collider box only (the rigidbody is removed
        # by zerog, but as soon as zerog is disabled it's restored by the anonymous layer),
        # exactly as what we started with
        omni.kit.commands.execute("ZeroGravityWaitForSimulationStepCompletedCommand")
        utils.check_stats(self, {"numBoxShapes": 1, "numDynamicRigids": 0})

        # Disable zerog
        await self._disable_zerog()
        await self._close_stage()

    async def test_physics_preserved(self):
        stage = omni.usd.get_context().get_stage()
        utils.execute_and_check(self, "AddPhysicsScene", stage=stage, path="/physicsScene")  # ensure we have a physicsscene
        _ = physicsUtils.add_rigid_box(stage, "/dynamicActor")  # this adds a +1 rigidbody

        # step frame with timeline, there should be one rigid body
        await self.step(ENABLED_WAIT_FRAMES)
        omni.kit.commands.execute("ZeroGravityWaitForSimulationStepCompletedCommand")
        utils.check_stats(self, {"numBoxShapes": 1, "numDynamicRigids": 1})
        await self.stop()

        # enable zero gravity
        await self._enable_zerog()

        # markers should not be read from the physics set, original collider will be retained
        omni.kit.commands.execute("ZeroGravityWaitForSimulationStepCompletedCommand")
        utils.check_stats(self, {"numDynamicRigids": 0})
        utils.check_stats(self, {"numBoxShapes": 1})

        await self._disable_zerog()

        # rigid body props should be set back
        # step frame with timeline, there should be one rigid body
        await self.step(ENABLED_WAIT_FRAMES)
        omni.kit.commands.execute("ZeroGravityWaitForSimulationStepCompletedCommand")
        utils.check_stats(self, {"numBoxShapes": 1, "numDynamicRigids": 1})
        await self.stop()

        await self._close_stage()

    # Startup simple test - test an activation with sweep mode on an empty stage
    async def test_sweep_mode_activation_on_empty_stage(self):
        omni.kit.commands.execute("ZeroGravitySetSweepModeCommand", sweep_mode=True)
        await self._enable_zerog()

        await self.wait(ENABLED_WAIT_FRAMES)

        await self._disable_zerog()
        await self._close_stage()

    async def test_physics_reset_transformation(self):
        stage = omni.usd.get_context().get_stage()
        dynamicPrim0 = physicsUtils.add_rigid_box(stage, "/dynamicActor0")

        # enable zero gravity
        await self._enable_zerog()

        # mark one cube as dynamic other as static
        omni.usd.get_context().get_selection().set_selected_prim_paths([str(dynamicPrim0.GetPrimPath())], True)
        omni.kit.commands.execute("ZeroGravitySetSelectedDynamic")

        await self.wait(ENABLED_WAIT_FRAMES)

        dynamicPrim1 = physicsUtils.add_rigid_box(stage, "/dynamicActor1")
        omni.usd.get_context().get_selection().set_selected_prim_paths([str(dynamicPrim1.GetPrimPath())], True)
        omni.kit.commands.execute("ZeroGravitySetSelectedDynamic")

        await self.wait(ENABLED_WAIT_FRAMES)

        dynamicPrim0.GetAttribute("xformOp:translate").Set(Gf.Vec3f(200.0))
        dynamicPrim1.GetAttribute("xformOp:translate").Set(Gf.Vec3f(-200.0))

        pos0 = dynamicPrim0.GetAttribute("xformOp:translate").Get()
        pos1 = dynamicPrim1.GetAttribute("xformOp:translate").Get()

        epsilon = 0.1
        self.assertTrue(Gf.IsClose(pos0, Gf.Vec3d(200), epsilon))
        self.assertTrue(Gf.IsClose(pos1, Gf.Vec3d(-200), epsilon))

        # finally select both cubes and restore their positions as before the zerog manipulations
        omni.usd.get_context().get_selection().set_selected_prim_paths(
            [str(dynamicPrim0.GetPrimPath()), str(dynamicPrim1.GetPrimPath())], True)

        await self.wait(ENABLED_WAIT_FRAMES)

        get_physx_zero_gravity_interface().mark_selection("restore", "")

        await self.wait(ENABLED_WAIT_FRAMES)

        pos0 = dynamicPrim0.GetAttribute("xformOp:translate").Get()
        pos1 = dynamicPrim1.GetAttribute("xformOp:translate").Get()

        self.assertTrue(Gf.IsClose(pos0, Gf.Vec3d(0.0), epsilon))
        self.assertTrue(Gf.IsClose(pos1, Gf.Vec3d(0.0), epsilon))

        await self._disable_zerog()
        await self._close_stage()

    # Creates two prims, activate the sweep mode and check that we have a dynamic markup for the selection and
    # one static markup for the swept nearby object
    async def test_sweep_mode_activation(self):
        stage = omni.usd.get_context().get_stage()
        cube0 = physicsUtils.add_rigid_box(stage, "/cubeActor0")
        cube1 = physicsUtils.add_rigid_box(stage, "/cubeActor1")
        cube1.GetAttribute("xformOp:translate").Set(Gf.Vec3f(110))

        # enable zero gravity
        await self._enable_zerog()
        # make sure sweep mode is inactive now
        omni.kit.commands.execute("ZeroGravitySetSweepMode", sweep_mode=False)

        await self.wait(ENABLED_WAIT_FRAMES)

        # activate sweep mode
        omni.kit.commands.execute("ZeroGravitySetSweepModeCommand", sweep_mode=True)

        await self.wait(ENABLED_WAIT_FRAMES)

        # check if the UI has properly updated

        self._usd_context = omni.usd.get_context()
        self._usd_context.get_selection().set_selected_prim_paths([str(cube0.GetPrimPath())], True)

        await self.wait(ENABLED_WAIT_FRAMES)

        omni.kit.commands.execute("ZeroGravityWaitForSimulationStepCompletedCommand")
        self._check_physx_object_counts({"numBoxShapes": 2, "numDynamicRigids": 1, "numStaticRigids": 1})

        await self._disable_zerog()
        await self._close_stage()

    # Creates two prims in the same place, select one and set the sweep mode as 'mark everything nearby as dynamic'. All dynamic
    # marked-up objects are expected.
    async def test_sweep_mode_mark_nearby_as_dynamic(self):
        stage = omni.usd.get_context().get_stage()
        dynamicPrim = physicsUtils.add_rigid_box(stage, "/dynamicActor")
        _ = physicsUtils.add_rigid_box(stage, "/cubeActor0")

        await self._enable_zerog()

        # activate sweep mode
        omni.kit.commands.execute("ZeroGravitySetSweepModeCommand", sweep_mode=True)
        omni.kit.commands.execute("ZeroGravityMarkSweepItemsDynamicCommand", use_dynamic_markers_for_swept_items=True)

        omni.usd.get_context().get_selection().set_selected_prim_paths([str(dynamicPrim.GetPrimPath())], True)

        await self.wait(ENABLED_WAIT_FRAMES)

        await self.wait(DROP_WAIT_FRAMES)

        omni.kit.commands.execute("ZeroGravityWaitForSimulationStepCompletedCommand")
        self._check_physx_object_counts({"numBoxShapes": 2, "numDynamicRigids": 2, "numStaticRigids": 0})

        await self._disable_zerog()
        await self._close_stage()

    async def test_manipulator(self):
        '''Checks if the zero gravity manipulator is registered correctly, works with rb/non-rb prims and
        hides when scale mode is active.'''
        from omni.kit.manipulator.selector.extension import get_manipulator_selector
        from omni.kit.manipulator.transform.settings_constants import c
        from omni.physxzerogravity.scripts.zerog_transform_manipulator import ZeroGravityTransformManipulator

        settings = carb.settings.get_settings()

        await self.create_simple_test_stage()
        await self.wait(ENABLED_WAIT_FRAMES)

        await self._enable_zerog()

        module_name = 'omni.physxzerogravity'
        context = omni.usd.get_context()
        selection = context.get_selection()

        def find_zerog_manipulator():
            selector = get_manipulator_selector("")
            for k, v in selector._manipulators.items():
                if k == module_name:
                    for m in v:
                        if isinstance(m, ZeroGravityTransformManipulator):
                            return m
            return None

        manipulator = find_zerog_manipulator()
        self.assertTrue(manipulator is not None, "zero gravity manipulator must exist and be registered to the system!")
        self.assertFalse(manipulator.enabled and manipulator._manipulator.enabled, "zero gravity manipulator is supposed to be not yet active!")

        # select a non rigid body and check current manipulator
        selection.set_selected_prim_paths(["/World/Cube0"], True)
        await self.wait(ACTION_WAIT_FRAMES)
        self.assertTrue(manipulator.enabled and manipulator._manipulator.enabled, "zero gravity manipulator is supposed to be active now!")

        # select a rigid body and check current manipulator
        selection.set_selected_prim_paths(["/World/CubeActor0"], True)
        await self.wait(ACTION_WAIT_FRAMES)
        self.assertTrue(manipulator.enabled and manipulator._manipulator.enabled, "zero gravity manipulator is supposed to be active now!")

        # make sure Move op is active
        settings.set(c.TRANSFORM_OP_SETTING, c.TRANSFORM_OP_MOVE)

        # check that gizmo disappears when SCALE op is active
        settings.set(c.TRANSFORM_OP_SETTING, c.TRANSFORM_OP_SCALE)
        await self.wait(ACTION_WAIT_FRAMES)
        self.assertFalse(manipulator.enabled and manipulator._manipulator.enabled, "zero gravity manipulator should not be active now!")
        settings.set(c.TRANSFORM_OP_SETTING, c.TRANSFORM_OP_ROTATE)
        await self.wait(ACTION_WAIT_FRAMES)
        self.assertTrue(manipulator.enabled and manipulator._manipulator.enabled, "zero gravity manipulator gizmo should be visible in rotation mode!")
        settings.set(c.TRANSFORM_OP_SETTING, c.TRANSFORM_OP_MOVE)
        await self.wait(ACTION_WAIT_FRAMES)
        self.assertTrue(manipulator.enabled and manipulator._manipulator.enabled, "zero gravity manipulator gizmo should be visible in translation mode!")

        await self._disable_zerog()

        selection.clear_selected_prim_paths()
        await self.wait(ACTION_WAIT_FRAMES)
        self.assertFalse(manipulator._enabled, "zero gravity manipulator should not be active now!")

        await self._close_stage()

    # Test a 1-sized Cube geom to be marked with sweep mode and be dropped onto a ground plane (with collider built-in).
    # Test it for stages with both Y-up and Z-up axis. It should work regardless.
    async def test_ground_plane_and_cube_interaction(self):
        for up_axis in ['Y', 'Z']:
            await utils.new_stage_setup()
            stage = omni.usd.get_context().get_stage()
            UsdGeom.SetStageUpAxis(stage, up_axis)
            ground_plane_size = 50
            up_axis = UsdGeom.GetStageUpAxis(stage)
            ground_plane_path = physicsUtils.add_ground_plane(stage, "/groundPlane", up_axis, ground_plane_size, Gf.Vec3f(0.0), Gf.Vec3f(0.5))
            ground_plane = stage.GetPrimAtPath(ground_plane_path)
            cube = physicsUtils.add_rigid_box(stage, "/cubeActor")

            # Move the cube up before the drop
            if up_axis == 'Y':
                cube.GetAttribute("xformOp:translate").Set(Gf.Vec3f(0, 100, 0))
            elif up_axis == 'Z':
                cube.GetAttribute("xformOp:translate").Set(Gf.Vec3f(0, 0, 100))

            await self._enable_zerog()
            omni.kit.commands.execute("ZeroGravitySetSweepModeCommand", sweep_mode=True)

            self._usd_context = omni.usd.get_context()
            self._usd_context.get_selection().set_selected_prim_paths([str(cube.GetPath())], True)
            omni.kit.commands.execute("ZeroGravitySetDropping", dropping=True)
            await self.wait(DROP_WAIT_FRAMES)

            # Check if cube is still on the surface of the ground plane
            cube_pos = cube.GetAttribute("xformOp:translate").Get()
            geom_cube = UsdGeom.Cube(cube)
            cube_half_extent = geom_cube.GetSizeAttr().Get() / 2
            ground_plane_pos = ground_plane.GetAttribute("xformOp:translate").Get()
            # Check that the dropped cube's center has hit the ground plane
            if up_axis == 'Y':
                self.assertTrue(Gf.IsClose(cube_pos[1] - cube_half_extent, ground_plane_pos[1], 0.1))
            elif up_axis == 'Z':
                self.assertTrue(Gf.IsClose(cube_pos[2] - cube_half_extent, ground_plane_pos[2], 0.1))

            await self._disable_zerog()
            await self._close_stage()

    # This test checks if switching selections works correctly in sweep mode. Three cubes are added
    # to the stage, and the test ensures that the selected cube is marked as dynamic while the others
    # nearby are marked as static. Being all at the same position means filtering will be enabled, we
    # do not care about that here.
    async def test_selection_switch(self):
        # Set up the stage and add three cubes
        stage = omni.usd.get_context().get_stage()
        cube0 = physicsUtils.add_rigid_box(stage, "/cubeActor0")
        cube1 = physicsUtils.add_rigid_box(stage, "/cubeActor1")
        cube2 = physicsUtils.add_rigid_box(stage, "/cubeActor2")

        # Enable zero gravity and sweep mode
        await self._enable_zerog()
        omni.kit.commands.execute("ZeroGravitySetSweepModeCommand", sweep_mode=True)

        # Select each cube in turn and check the dynamic and static rigid body counts
        for cube in [cube0, cube1, cube2]:
            omni.usd.get_context().get_selection().set_selected_prim_paths([str(cube.GetPrimPath())], True)
            await self.wait(ENABLED_WAIT_FRAMES)
            omni.kit.commands.execute("ZeroGravityWaitForSimulationStepCompletedCommand")
            self._check_physx_object_counts({"numBoxShapes": 3, "numDynamicRigids": 1, "numStaticRigids": 2})

        await self._disable_zerog()
        await self._close_stage()

    # If filtering works, marking an object and then marking another object directly overlapping the former should not
    # produce a small 'explosion', i.e. a forceful decompenetration due to simulation constraints.
    async def test_filtering_simple(self):
        # Set up the stage and add three cubes
        stage = omni.usd.get_context().get_stage()
        cube0 = physicsUtils.add_rigid_box(stage, "/cubeActor0")
        cube1 = physicsUtils.add_rigid_box(stage, "/cubeActor1")

        # Enable zero gravity and disable sweep mode
        await self._enable_zerog()
        omni.kit.commands.execute("ZeroGravitySetSweepModeCommand", sweep_mode=False)

        cube0_start_pos = cube0.GetAttribute("xformOp:translate").Get()
        cube1.GetAttribute("xformOp:translate").Set(Gf.Vec3f(0.2, 0, 0))
        cube1_start_pos = cube1.GetAttribute("xformOp:translate").Get()

        omni.usd.get_context().get_selection().set_selected_prim_paths([str(cube0.GetPrimPath())], True)
        omni.kit.commands.execute("ZeroGravitySetSelectedDynamic")
        omni.kit.commands.execute("ZeroGravityWaitForSimulationStepCompletedCommand")
        omni.usd.get_context().get_selection().set_selected_prim_paths([str(cube1.GetPrimPath())], True)
        omni.kit.commands.execute("ZeroGravitySetSelectedDynamic")
        omni.kit.commands.execute("ZeroGravityWaitForSimulationStepCompletedCommand")

        await self.wait(ACTION_WAIT_FRAMES)  # if filtering didn't work, there would probably be a small 'explosion' here

        cube0_end_pos = cube0.GetAttribute("xformOp:translate").Get()
        cube1_end_pos = cube1.GetAttribute("xformOp:translate").Get()

        self.assertTrue(Gf.IsClose(cube0_start_pos, cube0_end_pos, 0.001))
        self.assertTrue(Gf.IsClose(cube1_start_pos, cube1_end_pos, 0.001))

        await self._disable_zerog()
        await self._close_stage()

    # Tests filtering automatically disabling itself when a previously filtered object is no longer in collision after
    # a certain amount of time. Creates two cubes compenetrating: marks them both as dynamic - filtering should kick in.
    # Afterwards move one of the cubes away. Waits a bit. Filtering should disable itself for that box.
    # Then re-moves the box back into position and even beyond: now there should be a collision with the other box which
    # never left the scene origin.
    async def test_filtering_move_out_and_restore(self):
        # Set up the stage and add three cubes
        stage = omni.usd.get_context().get_stage()
        cube0 = physicsUtils.add_rigid_box(stage, "/cubeActor0")
        cube1 = physicsUtils.add_rigid_box(stage, "/cubeActor1")

        # Enable zero gravity and disable sweep mode (or nearby objects would be marked as static)
        await self._enable_zerog()
        omni.kit.commands.execute("ZeroGravitySetSweepModeCommand", sweep_mode=False)

        cube0_start_pos = cube0.GetAttribute("xformOp:translate").Get()
        cube1.GetAttribute("xformOp:translate").Set(Gf.Vec3f(0.2, 0, 0))  # Move cube1 a bit to facilitate 'explosion'
        cube1_start_pos = cube1.GetAttribute("xformOp:translate").Get()

        omni.usd.get_context().get_selection().set_selected_prim_paths([str(cube0.GetPrimPath())], True)
        omni.kit.commands.execute("ZeroGravitySetSelectedDynamic")
        omni.usd.get_context().get_selection().set_selected_prim_paths([str(cube1.GetPrimPath())], True)
        omni.kit.commands.execute("ZeroGravitySetSelectedDynamic")
        omni.kit.commands.execute("ZeroGravityWaitForSimulationStepCompletedCommand")

        await self.wait(ACTION_WAIT_FRAMES)  # if filtering didn't work, there would probably be a small 'explosion' here
        self._check_physx_object_counts({"numBoxShapes": 2, "numDynamicRigids": 2, "numStaticRigids": 0})

        # Check current positions, they should not have moved
        cube0_current_pos = cube0.GetAttribute("xformOp:translate").Get()
        cube1_current_pos = cube1.GetAttribute("xformOp:translate").Get()

        self.assertTrue(Gf.IsClose(cube0_start_pos, cube0_current_pos, 0.001))
        self.assertTrue(Gf.IsClose(cube1_start_pos, cube1_current_pos, 0.001))

        # Now move cube1 away from cube0, its filtering should disable itself and restore normal collisions flow
        for _ in range(50):
            get_physx_zero_gravity_interface().move(str(cube1.GetPath()), carb.Float3(0.1, 0, 0), True, False)
            # IMPORTANT: without waiting at least one frame, event loop and placement_mode.simulate() will not run
            # and stuff won't be moved as if the gizmo moved it
            await self.wait(ACTION_WAIT_FRAMES)

        # And finally move cube1 back, even beyond origin position: this should provoke a collision against cube0
        for _ in range(100):
            get_physx_zero_gravity_interface().move(str(cube1.GetPath()), carb.Float3(-0.1, 0, 0), True, False)
            await self.wait(ACTION_WAIT_FRAMES)

        cube0_end_pos = cube0.GetAttribute("xformOp:translate").Get()
        cube1_end_pos = cube1.GetAttribute("xformOp:translate").Get()

        # At this point we expect that a collision took place and moved somewhere the two cubes!
        self.assertFalse(Gf.IsClose(cube0_start_pos, cube0_end_pos, 0.001))
        self.assertFalse(Gf.IsClose(cube1_start_pos, cube1_end_pos, 0.001))

        await self._disable_zerog()
        await self._close_stage()


    # Given three cubes: cube0, cube1 and cube2 as in this picture seen from the top view,
    # activate ZG sweep mode, mark cube0 as dynamic (so cube1 and cube2 should be automatically
    # marked as static), then start rotating in increments of 15 degrees each cube0 around the Y
    # axis until cube0 can no longer be rotated because cube1 and cube2 will stop it.
    #
    #   -----                                             -----|----|
    #   |   | cube1         ^                             |   hit!  | <-----
    #   -----               |                             -----|    |
    #                       |                                  |    |
    #   -----------------------                                |    |
    #   |        cube0        |              ====>             |    |
    #   -----------------------                                |    |
    #     |                                                    |    |
    #     |               -----                                |    |-----
    #     v               |   | cube2                          |   hit!  |
    #                     -----                          ----> |----|-----
    #
    # Finally verify cube0's final position: it should roughly be the expected "stuck between cubes" one.
    async def test_cube_rotation(self):
        stage = omni.usd.get_context().get_stage()

        cube0 = physicsUtils.add_rigid_box(stage, "/World/cube0")

        # Scale on X so that it looks like 'a beam'
        cube0.GetAttribute("xformOp:scale").Set(Gf.Vec3f(6, 1, 1))

        cube1 = physicsUtils.add_rigid_box(stage, "/World/cube1")
        cube1.GetAttribute("xformOp:translate").Set(Gf.Vec3f(-1, 0, -2.5))

        cube2 = physicsUtils.add_rigid_box(stage, "/World/cube2")
        cube2.GetAttribute("xformOp:translate").Set(Gf.Vec3f(+1, 0, +2.5))

        await self._enable_zerog()
        omni.kit.commands.execute("ZeroGravitySetSweepModeCommand", sweep_mode=True)
        await self.wait(ENABLED_WAIT_FRAMES)

        omni.usd.get_context().get_selection().set_selected_prim_paths([str(cube0.GetPrimPath())], True)
        await self.wait(ACTION_WAIT_FRAMES)

        for _ in range(60):
            rotation_in_quaternions = _euler_to_quaternion([0, 15, 0])  # 15 degrees around Y axis
            rotation_in_quaternions_xyzw = carb.Float4(rotation_in_quaternions.GetImaginary()[0],
                                                       rotation_in_quaternions.GetImaginary()[1],
                                                       rotation_in_quaternions.GetImaginary()[2],
                                                       rotation_in_quaternions.GetReal())
            get_physx_zero_gravity_interface().rotate(str(cube0.GetPath()), str(cube0.GetPath()), rotation_in_quaternions_xyzw, True)
            await self.wait(ACTION_WAIT_FRAMES)

        cube0_end_rot = cube0.GetAttribute("xformOp:orient").Get()
        final_expected_rot = _euler_to_quaternion([0, 90, 0])

        vec4f1 = Gf.Vec4f(cube0_end_rot.GetImaginary()[0], cube0_end_rot.GetImaginary()[1], cube0_end_rot.GetImaginary()[2],
                          cube0_end_rot.GetReal())
        vec4f2 = Gf.Vec4f(final_expected_rot.GetImaginary()[0], final_expected_rot.GetImaginary()[1], final_expected_rot.GetImaginary()[2],
                          final_expected_rot.GetReal())

        self.assertTrue(Gf.IsClose(vec4f1, vec4f2, 0.01))

        await self._disable_zerog()
        await self._close_stage()

    # Test that ZG respects an authored collider when enabled, whatever the approximation type.
    # Only rigid body APIs should be stripped.
    async def test_authored_collider_retention(self):
        settings = carb.settings.get_settings()
        stage = omni.usd.get_context().get_stage()

        cubeGeom = physicsUtils.add_box(stage, "/World/cube0")
        UsdPhysics.CollisionAPI.Apply(cubeGeom.GetPrim())
        meshCollisionAPI = UsdPhysics.MeshCollisionAPI.Apply(cubeGeom.GetPrim())
        meshCollisionAPI.CreateApproximationAttr().Set("convexDecomposition")  # This is unused for dynamic markers
        massAPI = UsdPhysics.MassAPI.Apply(cubeGeom.GetPrim())
        massAPI.GetMassAttr().Set(5.0)
        UsdPhysics.RigidBodyAPI.Apply(cubeGeom.GetPrim())
        cubePrim = cubeGeom.GetPrim()

        has_api = cubePrim.HasAPI(UsdPhysics.CollisionAPI)
        self.assertTrue(has_api)

        await self._enable_zerog()
        omni.kit.commands.execute("ZeroGravitySetSweepModeCommand", sweep_mode=False)
        await self.wait(ENABLED_WAIT_FRAMES)

        self._zerog_flush_all_changes()

        # check the cube still has collision api even if not marked
        placement_layer_identifier = settings.get_as_string(SETTINGS_PLACEMENT_MODE_LAYER_IDENTIFIER)
        placement_layer = Sdf.Layer.Find(placement_layer_identifier)
        with Usd.EditContext(stage, placement_layer):
            has_api = cubePrim.HasAPI(UsdPhysics.CollisionAPI)
            self.assertTrue(has_api)
            # RB API should have been removed by ZG
            has_api = cubePrim.HasAPI(UsdPhysics.RigidBodyAPI)
            self.assertFalse(has_api)

        omni.usd.get_context().get_selection().set_selected_prim_paths([str(cubePrim.GetPrimPath())], True)
        omni.kit.commands.execute("ZeroGravitySetSelectedDynamic")
        await self.wait(ACTION_WAIT_FRAMES)
        self._zerog_flush_all_changes()

        with Usd.EditContext(stage, placement_layer):
            has_api = cubePrim.HasAPI(UsdPhysics.CollisionAPI)
            self.assertTrue(has_api)
            has_api = cubePrim.HasAPI(UsdPhysics.RigidBodyAPI)
            self.assertTrue(has_api)
            colliderType = UsdPhysics.MeshCollisionAPI(cubePrim).GetApproximationAttr().Get()
            self.assertTrue(colliderType == UsdPhysics.Tokens.convexDecomposition)

        await self._disable_zerog()
        await self._close_stage()

    # OM-109665 Regression test: makes sure ZG sweep thread is properly deactivated when ZG is disabled
    async def test_sweep_mode_thread_deactivation_at_zg_stop(self):
        # Create a couple of prims and activate sweep mode - this will also trigger precooking
        stage = omni.usd.get_context().get_stage()
        _ = physicsUtils.add_box(stage, "/World/cube0")
        _ = physicsUtils.add_box(stage, "/World/cube1")
        await self._enable_zerog()
        omni.kit.commands.execute("ZeroGravitySetSweepModeCommand", sweep_mode=True)
        # Shutting down immediately should also disable zerog sweep thread, even in precooking
        await self._disable_zerog()
        await self._close_stage()
