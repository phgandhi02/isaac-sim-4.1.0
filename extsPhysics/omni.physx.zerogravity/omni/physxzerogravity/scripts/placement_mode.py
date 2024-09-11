import asyncio

import carb
import carb.events
import carb.input
import omni.appwindow
import omni.ext
import omni.kit.app
import omni.kit.commands
import omni.kit.extensions
import omni.physx
import omni.physxzerogravity
import omni.physxzerogravity.bindings._physXZeroGravity as pxzerog
import omni.timeline
import omni.usd
from omni.physx.bindings._physx import SimulationEvent
from pxr import PhysicsSchemaTools, Sdf, Usd
from typing import Callable, List, Dict

from .action_bar import ActionBar
from .settings import ZeroGravitySettings
from .utils import is_rigid_body_placement_prim, refresh_manipulator_selector
from .zerog_sweep_area_progressbar import PhysXZeroGravityProgressView
from .zerog_transform_manipulator_registry import TransformManipulatorRegistry

SETTINGS_PICKING_MODE = "/persistent/app/viewport/pickingMode"
SETTINGS_PLACEMENT_MODE_MARK_CONTEXT_MENU_PRIM = "/physics/placementModeMarkContextMenuPrim"

PLACEMENT_MODE_LAYER = "ZeroGravity"
PICKING_MODE_MODELS = "models"


class PlacementModeTransformCommand(omni.kit.commands.Command):
    def __init__(self, physx_authoring, capture_session_id):
        """ Initialization of command"""
        self._run_once = False
        self._physx_authoring = physx_authoring
        self._capture_session_id = capture_session_id

    def do(self):
        """ First do() is in response after the the transform already occurred from simulation changes"""
        if not self._run_once:
            self._run_once = True
        else:
            self._physx_authoring.redo_capture_transform_changes(self._capture_session_id)

    def undo(self):
        """ Undo the previously watched fast updates changes in usd"""
        self._physx_authoring.undo_capture_transform_changes(self._capture_session_id)


SubscriptionId = int


def subscribe_to_zerog_enabled(callback: Callable) -> SubscriptionId:
    """
        Subscription system to get notified when ZG gets activated
    """
    subscription_id = PlacementMode._generate_subscription_id()
    PlacementMode._callbacks[subscription_id] = callback
    return subscription_id


def unsubscribe_from_zerog_enabled(subscription_id: SubscriptionId):
    if subscription_id in PlacementMode._callbacks:
        del PlacementMode._callbacks[subscription_id]


class PlacementMode:
    # Parts of ZG's activation notification system
    _callbacks: Dict[SubscriptionId, Callable] = {}
    _next_id: SubscriptionId = 0

    @classmethod
    def _generate_subscription_id(cls) -> SubscriptionId:
        cls._next_id += 1
        return cls._next_id

    def __init__(self, icon_folder):
        """ Initialization of placement mode handling class """
        self._usd_context = omni.usd.get_context()
        self._selection = self._usd_context.get_selection()
        self._selected_paths = self._selection.get_selected_prim_paths()
        self._placement_mode_layer = None
        self._stepping_time = 0.0
        # acquire physx and zero gravity interfaces
        self._physx = omni.physx.get_physx_interface()
        self._physx_cooking = omni.physx.get_physx_cooking_interface()
        self._physx_zero_gravity = omni.physxzerogravity.get_physx_zero_gravity_interface()
        self._physx_attached_to_stage = False

        # subscribe to physx simulation_event_stream_v2 to receive physx scene attached/detached events
        events = self._physx.get_simulation_event_stream_v2()
        self._simulation_event_sub = events.create_subscription_to_pop(self._on_simulation_event)

        # Sweep area progress bar overlay
        self._sweepAreaProgressView = PhysXZeroGravityProgressView()

        self._settings = carb.settings.get_settings()

        self._enabled = self._settings.get_as_bool(pxzerog.SETTINGS_PLACEMENT_MODE_ENABLED)
        self._placement_mode_enabled_before_play = None

        self._settings_subs = []
        self._settings_subs.append(omni.kit.app.SettingChangeSubscription(
            pxzerog.SETTINGS_PLACEMENT_MODE_ENABLED, self._on_placement_mode_enabled_setting_changed
        ))
        # setup settings change subscription for placement mode lock rotation
        self._settings_subs.append(omni.kit.app.SettingChangeSubscription(
            pxzerog.SETTINGS_PLACEMENT_MODE_LOCK_ROTATION, self._on_placement_mode_lock_rotation_setting_changed
        ))
        # setup settings change subscription for placement mode reset collision filterse
        self._settings_subs.append(omni.kit.app.SettingChangeSubscription(
            ZeroGravitySettings.SETTINGS_PLACEMENT_MODE_MARK_DEFAULT,
            lambda _, et: self._handle_menu_change_event(et, "default")
        ))
        # setup settings change subscription for placement mode mark default
        self._settings_subs.append(omni.kit.app.SettingChangeSubscription(
            ZeroGravitySettings.SETTINGS_PLACEMENT_MODE_MARK_CLEAR,
            lambda _, et: self._handle_menu_change_event(et, "clear")
        ))
        # setup settings change subscription for placement restore all transforms
        self._settings_subs.append(omni.kit.app.SettingChangeSubscription(
            ZeroGravitySettings.SETTINGS_PLACEMENT_MODE_RESTORE_ALL_TRANSFORMS,
            lambda _, et: self._handle_menu_change_event(et, "restore")
        ))
        # setup settings change subscription for placement mode mark convex decomposition
        self._settings_subs.append(omni.kit.app.SettingChangeSubscription(
            ZeroGravitySettings.SETTINGS_PLACEMENT_MODE_MARK_CONVEX_DECOMPOSITION,
            lambda _, et: self._handle_menu_change_event(et, "convex_decomposition")
        ))
        # setup settings change subscription for placement mode mark mesh simplification
        self._settings_subs.append(omni.kit.app.SettingChangeSubscription(
            ZeroGravitySettings.SETTINGS_PLACEMENT_MODE_MARK_MESH_SIMPLIFICATION,
            lambda _, et: self._handle_menu_change_event(et, "mesh_simplification")
        ))
        # setup settings change subscription for placement mode mark convex hull
        self._settings_subs.append(omni.kit.app.SettingChangeSubscription(
            ZeroGravitySettings.SETTINGS_PLACEMENT_MODE_MARK_CONVEX_HULL,
            lambda _, et: self._handle_menu_change_event(et, "convex_hull")
        ))
        # setup settings change subscription for placement mode mark triangle mesh
        self._settings_subs.append(omni.kit.app.SettingChangeSubscription(
            ZeroGravitySettings.SETTINGS_PLACEMENT_MODE_MARK_TRIANGLE_MESH,
            lambda _, et: self._handle_menu_change_event(et, "triangle_mesh")
        ))

        # setup settings changed subscription for picking mode
        self._picking_mode_models = self._settings.get(SETTINGS_PICKING_MODE) == PICKING_MODE_MODELS
        self._settings_subs.append(omni.kit.app.SettingChangeSubscription(
            SETTINGS_PICKING_MODE, self._on_picking_mode_setting_changed
        ))

        # setup stage event subscription
        self._stage_event_sub = self._usd_context.get_stage_event_stream().create_subscription_to_pop(
            self._on_stage_event, name="omni.physxzerogravity.PlacementMode._on_stage_event"
        )

        # setup app update subscription for frame events
        self._app = omni.kit.app.get_app_interface()
        self._app_update_sub = self._app.get_update_event_stream().create_subscription_to_pop(
            self._on_app_update_event, name="omni.physxzerogravity.PlacementMode._on_app_update_event"
        )

        self._manipulator_registry = None

        # setup for input handlers for raycast drag manipulation
        self._input = carb.input.acquire_input_interface()
        self._appwindow = omni.appwindow.get_default_app_window()
        self._keyboard = self._appwindow.get_keyboard()
        self._keyboard_sub = self._input.subscribe_to_keyboard_events(self._keyboard, self._on_keyboard_event)
        self._mouse = self._appwindow.get_mouse()
        self._dragging = False
        # locking rotation
        self._lock_rotation = self._settings.get_as_bool(pxzerog.SETTINGS_PLACEMENT_MODE_LOCK_ROTATION)
        self._translate_delta_xyz = None
        self._rotate_delta_xyzw = None
        self._scale_delta_xyz = None
        self._is_start = False
        self._is_stop = False
        # support for undo/redo
        omni.kit.commands.register(PlacementModeTransformCommand)
        self._is_capturing_transform_changes = False
        # action bar, the bar that appears in the scene
        self._action_bar = ActionBar(icon_folder, self._physx_zero_gravity)
        if self._enabled:
            self._action_bar.show()

    def clean(self):
        """ Cleanup of placement mode"""
        self._enabled = False
        if self._sweepAreaProgressView is not None:
            self._sweepAreaProgressView.on_shutdown()
            self._sweepAreaProgressView = None
        if self._manipulator_registry is not None:
            self._manipulator_registry.destroy()
            self._manipulator_registry = None
        self._settings_subs = []
        self._physx = None
        self._physx_zero_gravity = None
        self._physx_attached_to_stage = None
        self._simulation_event_sub = None
        self._placement_mode_layer = None
        self._stage_event_sub = None
        self._app_update_sub = None
        omni.kit.commands.unregister(PlacementModeTransformCommand)
        self._input.unsubscribe_to_keyboard_events(self._keyboard, self._keyboard_sub)
        self._keyboard_sub = None
        self._input = None
        self._action_bar.clean()
        for sub_id in list(PlacementMode._callbacks.keys()):
            unsubscribe_from_zerog_enabled(sub_id)
        PlacementMode._callbacks.clear()  # redundant but safe

    def enabled(self) -> bool:
        return self._enabled

    def get_action_bar(self):
        return self._action_bar

    def _flush_all_changes(self):
        self._physx_zero_gravity.force_all_changes_to_be_flushed()

    def should_show_gizmo(self, path) -> bool:
        return self._physx_zero_gravity.should_show_gizmo(PhysicsSchemaTools.sdfPathToInt(path))

    def use_custom_manipulator_for_prim(self, path) -> bool:
        stage = self._usd_context.get_stage()
        prim = stage.GetPrimAtPath(path)
        return is_rigid_body_placement_prim(prim)

    def _on_simulation_event(self, event):
        if event.type == int(SimulationEvent.ATTACHED_TO_STAGE):
            self._physx_attached_to_stage = True
        elif event.type == int(SimulationEvent.DETACHED_FROM_STAGE):
            self._physx_attached_to_stage = False
        elif event.type == int(SimulationEvent.RESUMED):
            if self._enabled:
                self._placement_mode_enabled_before_play = True
                self._settings.set_bool(pxzerog.SETTINGS_PLACEMENT_MODE_ENABLED, False)
        elif event.type == int(SimulationEvent.STOPPED):
            if self._placement_mode_enabled_before_play and not self._enabled:
                self._settings.set_bool(pxzerog.SETTINGS_PLACEMENT_MODE_ENABLED, True)
                self._placement_mode_enabled_before_play = False

    def _handle_menu_change_event(self, event_type, mark_action: str, setting: str = SETTINGS_PLACEMENT_MODE_MARK_CONTEXT_MENU_PRIM):
        """ Event handler for menu change"""
        if event_type == carb.settings.ChangeEventType.CHANGED and self._enabled:
            if self._physx_zero_gravity:
                self._mark_undo()
                contextMenuPrim = self._settings.get_as_string(setting)
                self._physx_zero_gravity.mark_selection(mark_action, contextMenuPrim)
                refresh_manipulator_selector()
                self._settings.set(setting, "")

    def _set_model_mode(self):
        if not self._enabled:
            return
        self._mark_undo()
        if self._physx_zero_gravity:
            self._physx_zero_gravity.set_model_mode(self._picking_mode_models)

    def _on_placement_mode_enabled_setting_changed(self, item, event_type):
        """ Event handler for placement mode enabled setting changed"""
        if event_type == carb.settings.ChangeEventType.CHANGED:
            enabled = self._settings.get_as_bool(pxzerog.SETTINGS_PLACEMENT_MODE_ENABLED)
            if enabled == self._enabled:
                return
            self._enabled = enabled

            if self._enabled:
                # Enable the placement mode!

                # Enable custom manipulator for ZeroG
                self._manipulator_registry = TransformManipulatorRegistry()

                # turn off simulation if it's running
                timeline = omni.timeline.get_timeline_interface()
                if timeline.is_playing() or not timeline.is_stopped():
                    timeline.stop()

                # In case the simulation was running, give it some time to restore the scene first
                async def __wait_for_scene_restore():
                    for i in range(5):
                        await omni.kit.app.get_app().next_update_async()
                asyncio.ensure_future(__wait_for_scene_restore())

                self._selected_paths = self._selection.get_selected_prim_paths()
                # create an anonymous physics placement mode layer if this hasn't been done already
                if self._placement_mode_layer is None:
                    self._placement_mode_layer = Sdf.Layer.CreateAnonymous(PLACEMENT_MODE_LAYER)
                    stage = self._usd_context.get_stage()
                    stage.GetSessionLayer().subLayerPaths.insert(0, self._placement_mode_layer.identifier)
                if self._physx_zero_gravity:
                    self._physx_zero_gravity.set_physics_authoring_mode(True, self._placement_mode_layer.identifier)
                self._set_model_mode()
                self._settings.set(ZeroGravitySettings.SETTINGS_PLACEMENT_MODE_LAYER_IDENTIFIER, self._placement_mode_layer.identifier)

                if not self._settings.get_as_bool(pxzerog.SETTINGS_PLACEMENT_MODE_HIDE_ACTION_BAR):
                    self._action_bar.show()
                else:
                    self._action_bar.hide()

                # enable sweep mode if requested
                if self._action_bar._sweep_mode:
                    self._action_bar.set_sweep_mode(True)

                # send out notifications (create a new list so callbacks can remove themselves without creating
                # loop iteration issues)
                for sub_id in list(PlacementMode._callbacks.keys()):
                    if sub_id in PlacementMode._callbacks:
                        PlacementMode._callbacks[sub_id]()

            else:
                # Disable the placement mode
                if self._physx_zero_gravity:
                    self._physx_zero_gravity.set_physics_authoring_mode(False, None)
                self._sweepAreaProgressView.set_enabled(False)
                if self._placement_mode_layer is not None:
                    # cleanup the physics placement mode layer
                    self._placement_mode_layer.Clear()
                    stage = self._usd_context.get_stage()
                    if stage:
                        stage.GetSessionLayer().subLayerPaths.remove(self._placement_mode_layer.identifier)
                    del self._placement_mode_layer
                    self._placement_mode_layer = None
                self._action_bar.hide()

                if self._manipulator_registry is not None:
                    self._manipulator_registry.destroy()
                    self._manipulator_registry = None

            self._settings.set_bool(ZeroGravitySettings.SETTINGS_CUSTOM_MANIPULATOR_ENABLED, self._enabled)

    def _on_placement_mode_lock_rotation_setting_changed(self, item, event_type):
        """ Event handler for picking mode setting changed"""
        if event_type == carb.settings.ChangeEventType.CHANGED and self._enabled:
            self._lock_rotation = self._settings.get(pxzerog.SETTINGS_PLACEMENT_MODE_LOCK_ROTATION)

    def _mark_undo(self):
        if self._physx_zero_gravity and self._enabled:
            capture_session_id = self._physx_zero_gravity.start_capture_transform_changes()
            omni.kit.commands.execute(
                "PlacementModeTransform", physx_authoring=self._physx_zero_gravity, capture_session_id=capture_session_id
            )

    def _on_picking_mode_setting_changed(self, item, event_type):
        """ Event handler for picking mode setting changed"""
        if event_type == carb.settings.ChangeEventType.CHANGED and self._enabled:
            self._picking_mode_models = self._settings.get(SETTINGS_PICKING_MODE) == PICKING_MODE_MODELS
            self._set_model_mode()

    def set_translate_delta_xyz(self, x, y, z):
        if self._enabled:
            self._translate_delta_xyz = carb.Float3(x, y, z)

    def set_rotate_delta_xyzw(self, x, y, z, w):
        if self._enabled:
            self._rotate_delta_xyzw = carb.Float4(x, y, z, w)

    def set_scale_delta_xyz(self, x, y, z):
        if self._enabled:
            self._scale_delta_xyz = carb.Float3(x, y, z)

    def set_active(self, active: bool):
        if not active:
            self._is_stop = True
        else:
            self._is_start = True

    def _transform_gizmo_changed(self):
        if not self._enabled:
            return
        if not self._is_capturing_transform_changes and self._physx_zero_gravity and self._physx_zero_gravity.has_transform_changes():
            self._is_capturing_transform_changes = True
            self._capture_session_id = self._physx_zero_gravity.start_capture_transform_changes()
        self._last_transform_change_time_s = self._app.get_time_since_start_s()

    def _on_stage_event(self, evt):
        """ Event handler for stage events like selection and closing stages"""
        if evt.type == int(omni.usd.StageEventType.SELECTION_CHANGED):
            if self._enabled:
                self._selected_paths = self._selection.get_selected_prim_paths()
                if self._physx_zero_gravity:
                    self._physx_zero_gravity.synchronize_placement_mode()
                if self._placement_mode_layer:
                    self._settings.set(ZeroGravitySettings.SETTINGS_PLACEMENT_MODE_LAYER_IDENTIFIER, self._placement_mode_layer.identifier)
        elif evt.type == int(omni.usd.StageEventType.CLOSING):
            if self._action_bar._sweep_mode:
                self._action_bar.set_sweep_mode(False)
        elif evt.type == int(omni.usd.StageEventType.OPENED):
            if self._enabled:
                self._settings.set(pxzerog.SETTINGS_PLACEMENT_MODE_ENABLED, False)
                self._settings.set(pxzerog.SETTINGS_PLACEMENT_MODE_ENABLED, True)

    def _apply_gizmo_transforms(self):
        if not self._enabled:
            return
        stage = omni.usd.get_context().get_stage()
        has_update = False
        if self._physx_zero_gravity:
            has_update = (
                self._is_stop
                or self._translate_delta_xyz is not None
                or self._rotate_delta_xyzw is not None
                or self._scale_delta_xyz is not None
                or self._physx_zero_gravity.is_raycast_dragging()
            )
        if not has_update:
            return
        with Usd.EditContext(stage, self._placement_mode_layer):
            if self._is_stop:
                for path in self._selected_paths:
                    self._physx_zero_gravity.cancel_move_rotate_scale(str(path))
                self._is_stop = False
            elif self._translate_delta_xyz:
                for path in self._selected_paths:
                    if self.use_custom_manipulator_for_prim(path):
                        self._transform_gizmo_changed()
                        self._physx_zero_gravity.move(
                            str(path), self._translate_delta_xyz, self._is_start, self._lock_rotation
                        )
                self._is_start = False
            elif self._rotate_delta_xyzw:
                for path in self._selected_paths:
                    if self.use_custom_manipulator_for_prim(path):
                        self._transform_gizmo_changed()
                        self._physx_zero_gravity.rotate(
                            str(path), str(self._selected_paths[-1]), self._rotate_delta_xyzw, self._is_start
                        )
                self._is_start = False
            elif self._scale_delta_xyz:
                for path in self._selected_paths:
                    if self.use_custom_manipulator_for_prim(path):
                        self._transform_gizmo_changed()
                        self._physx_zero_gravity.scale(
                            str(path),
                            str(self._selected_paths[-1]),
                            self._scale_delta_xyz,
                            self._is_start,
                            self._lock_rotation,
                        )
                self._is_start = False
            elif self._physx_zero_gravity.is_raycast_dragging():
                self._transform_gizmo_changed()
        self._translate_delta_xyz = None
        self._rotate_delta_xyzw = None
        self._scale_delta_xyz = None

    def _on_app_update_event(self, evt):
        """ Event handler app update events occuring every frame"""
        if self._enabled and self._physx_attached_to_stage:
            left_button_down = (
                self._input.get_mouse_button_flags(self._mouse, carb.input.MouseInput.LEFT_BUTTON)
                & carb.input.BUTTON_FLAG_DOWN
            ) == carb.input.BUTTON_FLAG_DOWN
            left_button_down = left_button_down & (
                not self._settings.get_as_bool(pxzerog.SETTINGS_PLACEMENT_MODE_ALLOW_DROP_ON_TRANSFORM_GIZMO_CHANGES)
            )
            if self._physx_zero_gravity:
                self._physx_zero_gravity.notify_gizmo_state(left_button_down)
                self._apply_gizmo_transforms()

                # Undo/redo: determine when can there are no more transform changes
                if self._is_capturing_transform_changes:
                    now_s = self._app.get_time_since_start_s()
                    elapsed_s = now_s - self._last_transform_change_time_s
                    if elapsed_s > 0.5:
                        self._is_capturing_transform_changes = False
                        # execute a command to apply the transform to usd.
                        omni.kit.commands.execute(
                            "PlacementModeTransform",
                            physx_authoring=self._physx_zero_gravity,
                            capture_session_id=self._capture_session_id,
                        )

                self._physx_zero_gravity.placement_simulate()

    def _on_keyboard_event(self, evt):
        """ Event handler for keyboard input to control drag manipulation"""
        if self._enabled:
            SETTINGS_TRANSFORM_OP = "/app/transform/operation"
            # SHIFT key down/up handler (for dragging with force) which hides the transform gizmo
            if evt.input == carb.input.KeyboardInput.LEFT_SHIFT or evt.input == carb.input.KeyboardInput.RIGHT_SHIFT:
                if evt.type == carb.input.KeyboardEventType.KEY_PRESS:
                    self._transform_op = self._settings.get_as_string(SETTINGS_TRANSFORM_OP)
                    self._settings.set_string(SETTINGS_TRANSFORM_OP, "select")
                    self._dragging = True
                elif evt.type == carb.input.KeyboardEventType.KEY_RELEASE:
                    if self._dragging:
                        self._settings.set_string(SETTINGS_TRANSFORM_OP, self._transform_op)
                        self._dragging = False
            # END key down/up handler for drop
            if evt.input == carb.input.KeyboardInput.END:
                if evt.type == carb.input.KeyboardEventType.KEY_PRESS:
                    if len(self._selected_paths) > 0:
                        self._transform_gizmo_changed()
                        self._action_bar.set_dropping(True)
                elif evt.type == carb.input.KeyboardEventType.KEY_RELEASE:
                    self._action_bar.set_dropping(False)
        return True
