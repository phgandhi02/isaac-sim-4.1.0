"""pybind11 carb.physxui bindings"""
from __future__ import annotations
import omni.physxui.bindings._physxUI
import typing
import carb._carb
import carb.input
import omni.ui_scene._scene

__all__ = [
    "IPhysxUI",
    "IPhysxUIPrivate",
    "PhysXUIOmniUISceneOverlay",
    "acquire_physx_ui_interface",
    "acquire_physx_ui_private_interface",
    "release_physx_ui_interface",
    "release_physx_ui_interface_scripting",
    "release_physx_ui_private_interface",
    "release_physx_ui_private_interface_scripting"
]


class IPhysxUI():
    def block_usd_notice_handler(self, arg0: bool) -> None: ...
    def enable_collision_mesh_visualization(self, arg0: bool) -> None: ...
    def enable_debug_visualization(self, arg0: bool) -> None: ...
    def enable_redraw_optimizations(self, arg0: bool) -> None: ...
    def explode_view_distance(self, arg0: float) -> None: ...
    def get_attachments(self, prim_path: str) -> dict: 
        """
        Get all attachments that are associated with a given primitive at path.
        Args:
            prim_path: Sdf.Path to primitive for which associated attachments are queried.
        Returns:
            dict with "attachments" entry containing a list of all attachment paths that are associated with the prim_path.
        """
    def get_vehicle_visualization(self, parameter: str) -> bool: 
        """
        Get state of vehicle debug visualization features.

        Args:
            parameter - Debug visualization feature string identifier (see set_vehicle_visualization
            for the supported identifiers)

        Returns:
            bool: True if the feature is set for visualization, else false.
        """
    def hide_attached_actor(self, attachment_path: str, actor_path: str, hide: bool) -> None: 
        """
        Hide and unhide an attached actor. Attachment needs to be enabled for visualization with
        set_attachment_visualization otherwise no effect takes place. 
        Args:
            attachment_path: Sdf.Path to the attachment.
            actor_path: Sdf.Path to attached actor.
            hide: Hides or unhides the attached actor.
        """
    def is_usd_notice_handler_enabled(self) -> bool: ...
    def refresh_attachment(self, attachment_path: str) -> None: 
        """
        Refresh deformable attachment.
        Args:
            attachment_path: Sdf.Path to attachment primitive.
        """
    def select_spatial_tendon_attachment_helper(self, link_body_path: str, instance_name: str) -> None: 
        """
        Trigger selection of spatial tendon attachment session layer helper geometry.

        The selection is queued in the spatial tendon visualization module, and the selection will fail
        quietly if the provided inputs are invalid, i.e. do not point to a valid attachment in the stage.

        Args:
            link_body_path: Sdf.Path to rigid body articulation link that the attachment API is applied to.
            instance_name: String instance name of attachment API (specific API type is not relevant)
        """
    def set_attachment_visualization(self, attachment_path: str, enable: bool) -> None: 
        """
        Enable/disable attachment visualization for an attachment.
        Args:
            attachment_path: Sdf.Path to attachment for which visualization is enabled or disabled.
            enable: Enables or disables attachment visualization.
        """
    def set_camera_pos(self, pos: carb._carb.Float3) -> None: 
        """
        Set camera world position. Used by DebugVisualizer of colliders.
        Args:
            pos: Camera world position.
        """
    def set_collision_mesh_type(self, arg0: str) -> None: ...
    def set_tendon_visualization_filter(self, arg0: str) -> None: ...
    def set_vehicle_visualization(self, parameter: str, enable: bool) -> None: 
        """
        Toggle individual vehicle debug visualization features.

        Args:
            parameter - Debug visualization feature string identifier, can be one of the following:
                {'Suspension'}
            enable - Bool to enable/disable the feature.
        """
    def set_visualization_distance(self, arg0: float) -> None: ...
    def update(self) -> None: ...
    def update_gizmo_transform(self, arg0: str, arg1: carb._carb.Float3, arg2: carb._carb.Float4) -> bool: ...
    pass
class IPhysxUIPrivate():
    def clear_input_manager(self) -> None: 
        """
        Unregister and clear all added actions from the input manager. Enables all disabled conflicting actions.
        """
    def register_gamepad_action(self, actionName: str, input: carb.input.GamepadInput, gamepad_index: int = 0) -> None: 
        """
        Registers a gamepad action mapping. The action will be active only during simulation mode in first person mode and conflicting actions would be automatically disabled.

        Args:
            action_name: Name of the action.
            input: Gamepad input to activate the action with.
            index: Gamepad index.
        """
    def register_keyboard_action(self, actionName: str, input: carb.input.KeyboardInput, modifiers: int) -> None: 
        """
        Registers a keyboard action mapping. The action will be active only during simulation mode in first person mode and conflicting actions would be automatically disabled.

        Args:
            action_name: Name of the action.
            input: Keyboard input to activate the action with.
            modifiers: Keyboard modifier.
        """
    def test_debug_vis_internal_state(self, phase: int) -> bool: 
        """
        Performs test of Debug Visualization's internal structures.
        Each phase tests specific stuff.
            Phase 0: initial camera position.
            Phase 1: far camera position with completely boxed debug visualization with either AABB or box/bounding box.
            Phase 2: midpoint where some objects are replaced with AABB and some are not.
        """
    def unregister_action(self, actionName: str) -> None: 
        """
        Unregisters an action mapping registered with either register_keyboard_action or register_gamepad_action.

        Args:
            action_name: Name of the action.
        """
    pass
class PhysXUIOmniUISceneOverlay(omni.ui_scene._scene.Manipulator, omni.ui_scene._scene.AbstractContainer, omni.ui_scene._scene.AbstractItem):
    def __init__(self, **kwargs) -> None: ...
    def set_enable_picking_fn(self, arg0: typing.Callable[[bool], None]) -> None: ...
    pass
def acquire_physx_ui_interface(plugin_name: str = None, library_path: str = None) -> IPhysxUI:
    pass
def acquire_physx_ui_private_interface(plugin_name: str = None, library_path: str = None) -> IPhysxUIPrivate:
    pass
def release_physx_ui_interface(arg0: IPhysxUI) -> None:
    pass
def release_physx_ui_interface_scripting(arg0: IPhysxUI) -> None:
    pass
def release_physx_ui_private_interface(arg0: IPhysxUIPrivate) -> None:
    pass
def release_physx_ui_private_interface_scripting(arg0: IPhysxUIPrivate) -> None:
    pass
