from __future__ import annotations
import omni.physxzerogravity.bindings._physXZeroGravity
import typing
import carb._carb
import carb.events._events

__all__ = [
    "PhysxZeroGravity",
    "SETTINGS_LOGGING_ENABLED",
    "SETTINGS_LOGGING_ENABLED_DEFAULT",
    "SETTINGS_PLACEMENT_MODE_ALLOW_DROP_ON_TRANSFORM_GIZMO_CHANGES",
    "SETTINGS_PLACEMENT_MODE_ALLOW_DROP_ON_TRANSFORM_GIZMO_CHANGES_DEFAULT",
    "SETTINGS_PLACEMENT_MODE_DROP_CUSTOM_GRAVITY_DIRECTION",
    "SETTINGS_PLACEMENT_MODE_DROP_CUSTOM_GRAVITY_DIRECTION_DEFAULT",
    "SETTINGS_PLACEMENT_MODE_DROP_CUSTOM_GRAVITY_DIRECTION_ENABLED",
    "SETTINGS_PLACEMENT_MODE_DROP_CUSTOM_GRAVITY_DIRECTION_ENABLED_DEFAULT",
    "SETTINGS_PLACEMENT_MODE_DROP_SPEED",
    "SETTINGS_PLACEMENT_MODE_DROP_SPEED_DEFAULT",
    "SETTINGS_PLACEMENT_MODE_ENABLED",
    "SETTINGS_PLACEMENT_MODE_ENABLED_DEFAULT",
    "SETTINGS_PLACEMENT_MODE_HIDE_ACTION_BAR",
    "SETTINGS_PLACEMENT_MODE_HIDE_ACTION_BAR_DEFAULT",
    "SETTINGS_PLACEMENT_MODE_LOCK_ROTATION",
    "SETTINGS_PLACEMENT_MODE_LOCK_ROTATION_DEFAULT",
    "SETTINGS_PLACEMENT_MODE_SET_SWEPT_ITEMS_DYNAMIC",
    "SETTINGS_PLACEMENT_MODE_SET_SWEPT_ITEMS_DYNAMIC_DEFAULT",
    "SETTINGS_PLACEMENT_MODE_SHOW_DYNAMICS",
    "SETTINGS_PLACEMENT_MODE_SHOW_DYNAMICS_DEFAULT",
    "SETTINGS_PLACEMENT_MODE_SHOW_STATICS",
    "SETTINGS_PLACEMENT_MODE_SHOW_STATICS_DEFAULT",
    "SETTINGS_PLACEMENT_MODE_SWEEP_AREA_VISUALIZE",
    "SETTINGS_PLACEMENT_MODE_SWEEP_AREA_VISUALIZE_DEFAULT",
    "SETTINGS_ZEROG_SIMREADY_DND_ENABLED",
    "SETTINGS_ZEROG_SIMREADY_DND_ENABLED_DEFAULT",
    "SETTING_SUPPORTUI_PHYSICS_INSPECTOR_ENABLED",
    "SETTING_SUPPORTUI_PHYSICS_INSPECTOR_ENABLED_DEFAULT",
    "ZeroGravityEventType",
    "acquire_physx_zero_gravity_interface",
    "release_physx_zero_gravity_interface",
    "release_physx_zero_gravity_interface_scripting"
]


class PhysxZeroGravity():
    """
    This interface is the access PhysX ZeroGravity specific features
    """
    def cancel_move_rotate_scale(self, path: str) -> bool: 
        """
        Cancels any outstanding move/rotate/scale operations on the object specified

        Args:
            path:  Name of object to be have move/rotate/scale operations canceled
        """
    def clear_all_markers(self) -> None: 
        """
        Clear all zero gravity markers on a current stage
        """
    def enable_local_precooking(self, enabled: bool) -> None: 
        """
        Enable or disable the local precooking in the scene
        """
    def force_all_changes_to_be_flushed(self) -> None: 
        """
        Blocking version of synchronizePlacementMode that will ensure all changes are flushed before returning
        """
    def force_solid_angle_camera_prims_evaluation_now(self) -> None: 
        """
        Force a geo-priority evaluation of the prims most likely to be clicked by the user (in camera view).
        Synchronous.
        """
    def get_elapsed_time(self) -> float: ...
    def get_event_stream(self) -> carb.events._events.IEventStream: 
        """
        Event stream sending various events defined in ZeroGravityEvent enum.

        Returns:
            Event stream sending the events.
        """
    def get_solid_angle_camera_prim_path_at_pos(self, pos: int) -> str: 
        """
        Access the pos-th SDF prim path in the evaluated buffer. First one has a higher priority.
        """
    def get_solid_angle_camera_prims_num(self) -> int: 
        """
        Get the number of solid angle prims evaluated.
        """
    def has_physx_actor(self, path: str) -> bool: 
        """
        Returns true if the primitive path provided is associated with a PhysX SDK actor

        Args:
            path:  Path of the primitive to test
        """
    def has_transform_changes(self) -> bool: 
        """
        Returns true if there have been transform changes since the last time the undo stack was captured
        """
    def is_applying_properties(self) -> bool: ...
    def is_raycast_dragging(self) -> bool: 
        """
        Returns true if raycast dragging operation is currently running.
        """
    def mark_prim(self, prim_path: str, markType: str) -> bool: 
        """
        Marks prim with physics state
        """
    def mark_selection(self, markType: str, contextMenuPrimPath: str) -> int: 
        """
        Marks currently selected prims using this markup type
        """
    def move(self, path: str, delta_translation: carb._carb.Float3, start: bool, lock_rotation: bool) -> bool: 
        """
        Attempts to move a physics object to a new relative position

        Args:
            path:  Name of object to be moved
            delta_translation: Float3 of the delta to move X/Y/Z
            start: Indicates first update of move operation.
            lock_rotation: Whether rotation should be locked during move
        """
    def notify_gizmo_state(self, isactive: bool) -> None: 
        """
        Informs the placement mode of the current gizmo state, active or inactive and toggles dropping mode behavior accordingly
        """
    def placement_simulate(self) -> None: ...
    def redo_capture_transform_changes(self, arg0: int) -> None: ...
    def rotate(self, path: str, pivot_path: str, delta_rotation: carb._carb.Float4, start: bool) -> bool: 
        """
        Attempts to rotate a physics object by a delta as a quaternion, optionally relative to a pivot actor

        Args:
            path: Name of object to be rotated
            pivot_path : Path of object the rotation is relative to
            delta_rotation: Delta rotation as a quaternion X/Y/Z/W
            start: Indicates first update of rotate operation.
        """
    def scale(self, path: str, pivot_path: str, delta_scale: carb._carb.Float3, start: bool, lock_rotation: bool) -> bool: 
        """
        Attempts to move a physics object by scaling the distance to a pivot actor

        Args:
            path: Name of object whose distance to the pivot object is scaled
            pivot_path: Path of object the scaling is relative to - if equal to path, marked as kinematic
            delta_scale: Delta scale as vector X/Y/Z
            start: Indicates first update of scaling operation.
            lock_rotation: Whether rotation should be locked during scale
        """
    def set_dropping_on_prim(self, isDropping: bool, primPath: str) -> int: 
        """
        Sets the dropping mode on a rigid body at the specified prim path
        """
    def set_dropping_on_selected(self, isDropping: bool) -> int: 
        """
        Sets the dropping mode on currently selected rigid bodies
        """
    def set_model_mode(self, arg0: bool) -> None: ...
    def set_physics_authoring_mode(self, state: bool, layerName: str) -> None: 
        """
        Sets the physics authoring mode. When enabled this runs the custom
        collision filtering logic to prevent objects from exploding when they
        are intersecting in their rest state

        Args:
            state:  Enable or disable the custom physics authoring code
            layerName: Anonymous layer where zerog will operate with its placement mode
        """
    def set_sweep_mode(self, sweepActive: bool) -> None: 
        """
        Enables or disables sweep mode which marks nearby objects automatically
        """
    def set_swept_items_dynamic(self, useDynamicMarkersForSweptItems: bool) -> None: 
        """
        Sets whether sweep mode marks nearby objects as dynamic. If not, swept objects are marked as static.
        """
    def should_show_gizmo(self, arg0: int) -> bool: ...
    def start_capture_transform_changes(self) -> int: ...
    def sweep_area_visualize(self, visualizeAABB: bool) -> None: 
        """
        Show or hide a sweep area bounding box visualization.
        """
    def synchronize_placement_mode(self) -> None: 
        """
        Synchronizes the current USD stage for any possible changes which need to be reflected by physics placement mode
        """
    def undo_capture_transform_changes(self, arg0: int) -> None: ...
    def wait_for_simulation_step_completed(self) -> None: 
        """
        Blocks until this simulation step is completed. Call this before reading out simulation statistics (e.g. in unit tests)
        otherwise you'll read garbage data still in simulation and get errors.
        """
    pass
class ZeroGravityEventType():
    """
    ZeroGravity events used by event stream.

    Members:

      REFRESH_GIZMO : When omni.kit.manipulator.selector needs refresh; contains nothing in the dictionary.
                
    """
    def __eq__(self, other: object) -> bool: ...
    def __getstate__(self) -> int: ...
    def __hash__(self) -> int: ...
    def __index__(self) -> int: ...
    def __init__(self, value: int) -> None: ...
    def __int__(self) -> int: ...
    def __ne__(self, other: object) -> bool: ...
    def __repr__(self) -> str: ...
    def __setstate__(self, state: int) -> None: ...
    @property
    def name(self) -> str:
        """
        :type: str
        """
    @property
    def value(self) -> int:
        """
        :type: int
        """
    REFRESH_GIZMO: omni.physxzerogravity.bindings._physXZeroGravity.ZeroGravityEventType # value = <ZeroGravityEventType.REFRESH_GIZMO: 0>
    __members__: dict # value = {'REFRESH_GIZMO': <ZeroGravityEventType.REFRESH_GIZMO: 0>}
    pass
def acquire_physx_zero_gravity_interface(plugin_name: str = None, library_path: str = None) -> PhysxZeroGravity:
    pass
def release_physx_zero_gravity_interface(arg0: PhysxZeroGravity) -> None:
    pass
def release_physx_zero_gravity_interface_scripting(arg0: PhysxZeroGravity) -> None:
    pass
SETTINGS_LOGGING_ENABLED = '/persistent/physics/zerogLoggingEnabled'
SETTINGS_LOGGING_ENABLED_DEFAULT = '/defaults/persistent/physics/zerogLoggingEnabled'
SETTINGS_PLACEMENT_MODE_ALLOW_DROP_ON_TRANSFORM_GIZMO_CHANGES = '/physics/placementModeAllowDropOnTransformGizmoChanges'
SETTINGS_PLACEMENT_MODE_ALLOW_DROP_ON_TRANSFORM_GIZMO_CHANGES_DEFAULT = '/defaults/physics/placementModeAllowDropOnTransformGizmoChanges'
SETTINGS_PLACEMENT_MODE_DROP_CUSTOM_GRAVITY_DIRECTION = '/persistent/physics/placementModeDropCustomGravityDirection'
SETTINGS_PLACEMENT_MODE_DROP_CUSTOM_GRAVITY_DIRECTION_DEFAULT = '/defaults/persistent/physics/placementModeDropCustomGravityDirection'
SETTINGS_PLACEMENT_MODE_DROP_CUSTOM_GRAVITY_DIRECTION_ENABLED = '/persistent/physics/placementModeDropCustomGravityDirectionEnabled'
SETTINGS_PLACEMENT_MODE_DROP_CUSTOM_GRAVITY_DIRECTION_ENABLED_DEFAULT = '/defaults/persistent/physics/placementModeDropCustomGravityDirectionEnabled'
SETTINGS_PLACEMENT_MODE_DROP_SPEED = '/persistent/physics/placementModeDropSpeed'
SETTINGS_PLACEMENT_MODE_DROP_SPEED_DEFAULT = '/defaults/persistent/physics/placementModeDropSpeed'
SETTINGS_PLACEMENT_MODE_ENABLED = '/physics/placementModeEnabled'
SETTINGS_PLACEMENT_MODE_ENABLED_DEFAULT = '/defaults/physics/placementModeEnabled'
SETTINGS_PLACEMENT_MODE_HIDE_ACTION_BAR = '/physics/placementModeHideActionBar'
SETTINGS_PLACEMENT_MODE_HIDE_ACTION_BAR_DEFAULT = '/defaults/physics/placementModeHideActionBar'
SETTINGS_PLACEMENT_MODE_LOCK_ROTATION = '/persistent/physics/placementModeLockRotation'
SETTINGS_PLACEMENT_MODE_LOCK_ROTATION_DEFAULT = '/defaults/persistent/physics/placementModeLockRotation'
SETTINGS_PLACEMENT_MODE_SET_SWEPT_ITEMS_DYNAMIC = '/persistent/physics/placementModeSetSweptItemsDynamic'
SETTINGS_PLACEMENT_MODE_SET_SWEPT_ITEMS_DYNAMIC_DEFAULT = '/defaults/persistent/physics/placementModeSetSweptItemsDynamic'
SETTINGS_PLACEMENT_MODE_SHOW_DYNAMICS = '/persistent/physics/placementModeShowDynamics'
SETTINGS_PLACEMENT_MODE_SHOW_DYNAMICS_DEFAULT = '/defaults/persistent/physics/placementModeShowDynamics'
SETTINGS_PLACEMENT_MODE_SHOW_STATICS = '/persistent/physics/placementModeShowStatics'
SETTINGS_PLACEMENT_MODE_SHOW_STATICS_DEFAULT = '/defaults/persistent/physics/placementModeShowStatics'
SETTINGS_PLACEMENT_MODE_SWEEP_AREA_VISUALIZE = '/persistent/physics/placementModeSweepAreaVisualize'
SETTINGS_PLACEMENT_MODE_SWEEP_AREA_VISUALIZE_DEFAULT = '/defaults/persistent/physics/placementModeSweepAreaVisualize'
SETTINGS_ZEROG_SIMREADY_DND_ENABLED = '/persistent/physics/zeroGSimreadyDnDEnabled'
SETTINGS_ZEROG_SIMREADY_DND_ENABLED_DEFAULT = '/defaults/persistent/physics/zeroGSimreadyDnDEnabled'
SETTING_SUPPORTUI_PHYSICS_INSPECTOR_ENABLED = '/physics/supportUiPhysicsInspector/enabled'
SETTING_SUPPORTUI_PHYSICS_INSPECTOR_ENABLED_DEFAULT = '/defaults/physics/supportUiPhysicsInspector/enabled'
