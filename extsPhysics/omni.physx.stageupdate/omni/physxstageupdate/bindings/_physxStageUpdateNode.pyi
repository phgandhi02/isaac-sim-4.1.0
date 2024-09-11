"""pybind11 carb.physx.stageupdate bindings"""
from __future__ import annotations
import omni.physxstageupdate.bindings._physxStageUpdateNode
import typing

__all__ = [
    "IPhysxStageUpdateNode",
    "acquire_physx_stage_update_node_interface",
    "release_physx_stage_update_node_interface",
    "release_physx_stage_update_node_scripting"
]


class IPhysxStageUpdateNode():
    def attach_node(self) -> None: 
        """
        Attach the PhysX StageUpdateNode to IStageUpdate
        """
    def block_timeline_events(self, arg0: bool) -> None: 
        """
        Blocks time line events (play, resume, stop)
        """
    def detach_node(self) -> None: 
        """
        Detach the PhysX StageUpdateNode to IStageUpdate
        """
    def is_node_attached(self) -> bool: 
        """
        Check if StageUpdateNode is attached to IStageUpdate

        Returns:
            bool: True if attached.
        """
    def timeline_events_blocked(self) -> bool: 
        """
        Check if time line events (play, resume, stop) are blocked

        Returns:
            bool: True if blocked.
        """
    pass
def acquire_physx_stage_update_node_interface(plugin_name: str = None, library_path: str = None) -> IPhysxStageUpdateNode:
    pass
def release_physx_stage_update_node_interface(arg0: IPhysxStageUpdateNode) -> None:
    pass
def release_physx_stage_update_node_scripting(arg0: IPhysxStageUpdateNode) -> None:
    pass
