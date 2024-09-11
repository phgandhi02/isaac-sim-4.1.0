"""pybind11 carb.physx.cct bindings"""
from __future__ import annotations
import omni.physxcct.bindings._physxCct
import typing
import carb._carb
import carb.events._events

__all__ = [
    "CctEvent",
    "IPhysxCct",
    "acquire_physx_cct_interface",
    "release_physx_cct_interface"
]


class CctEvent():
    """
            Cct events used by Cct event stream.
            

    Members:

      COLLISION_DOWN : Character controller collision down event, during the last cct move a collision below the CCT was found and the status changed; contains the following in a dictionary::

                    'cctPath':int2 - Usd path to the CCT decoded into two ints. PhysicsSchemaTools.decodeSdfPath will return SdfPath.                                
                    'collision':bool - Reports current collision with CCT.
                

      COLLISION_UP : Character controller collision down event, during the last cct move a collision above the CCT was found and the status changed; contains the following in a dictionary::

                    'cctPath':int2 - Usd path to the CCT decoded into two ints. PhysicsSchemaTools.decodeSdfPath will return SdfPath.                                
                    'collision':bool - Reports current collision with CCT.
                

      COLLISION_SIDES : Character controller collision down event, during the last cct move a collision on a side of the CCT was found and the status changed; contains the following in a dictionary::

                    'cctPath':int2 - Usd path to the CCT decoded into two ints. PhysicsSchemaTools.decodeSdfPath will return SdfPath.                                
                    'collision':bool - Reports current collision with CCT.
                
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
    COLLISION_DOWN: omni.physxcct.bindings._physxCct.CctEvent # value = <CctEvent.COLLISION_DOWN: 0>
    COLLISION_SIDES: omni.physxcct.bindings._physxCct.CctEvent # value = <CctEvent.COLLISION_SIDES: 2>
    COLLISION_UP: omni.physxcct.bindings._physxCct.CctEvent # value = <CctEvent.COLLISION_UP: 1>
    __members__: dict # value = {'COLLISION_DOWN': <CctEvent.COLLISION_DOWN: 0>, 'COLLISION_UP': <CctEvent.COLLISION_UP: 1>, 'COLLISION_SIDES': <CctEvent.COLLISION_SIDES: 2>}
    pass
class IPhysxCct():
    def activate_cct(self, arg0: str) -> None: 
        """
        Adds character controller to the manager. Use if not calling any of the other methods at least once.

        Args:
            path: Path of the controller's prim.
        """
    def disable_first_person(self, arg0: str) -> None: 
        """
        Disable first person camera support.

        Args:
            path: Path of the controller's prim.
        """
    def disable_gravity(self, arg0: str) -> None: 
        """
        Disables gravity set through apply_gravity or apply_custom_gravity.

        Args:
            path: Path of the controller's prim.
        """
    def enable_custom_gravity(self, arg0: str, arg1: carb._carb.Double3) -> None: 
        """
        Adds custom gravity to the controller's move vector.

        Args:
            path: Path of the controller's prim.
            gravity: Custom gravity vector.
        """
    def enable_first_person(self, arg0: str, arg1: str) -> None: 
        """
        Enable first person camera support for a controller. Hides mouse cursor, sets the controller's capsule as a guide and uses the controller's camera transformation when transforming move vector in local space mode. If you want to use multiple controllers at the same time use setViewportIndex.

        Args:
            path: Path of the controller's prim.
            camera_path: Camera path.
        """
    def enable_gravity(self, arg0: str) -> None: 
        """
        Adds current PhysicsScene's gravity to the controller's move vector.

        Args:
            path: Path of the controller's prim.
        """
    def enable_worldspace_move(self, arg0: str, arg1: bool) -> None: 
        """
        Sets if PhysxCharacterControllerAPI:MoveTarget attribute will be considered as a local or world space vector. Local space move vector is transformed with the controller's capsule transformation (or camera transformation in the case of first person mode).

        Args:
            use: World space is used when true, local space if false.
        """
    def get_cct_event_stream(self) -> carb.events._events.IEventStream: 
        """
        Simulation event stream sending various simulation events defined in SimulationEvent enum.

        Returns:
            Event stream sending the simulation events.
        """
    def get_controller_height(self, arg0: str) -> float: 
        """
        Gets controller's height.

        Args:
            path: Path of the controller's prim.

        Returns:
            float: The height of the controller.
        """
    def has_gravity_enabled(self, arg0: str) -> bool: 
        """
        Gets if gravity is being added to the controller's move vector.

        Args:
            path: Path of the controller's prim.
        """
    def remove_cct(self, arg0: str) -> None: 
        """
        Removes a character controller's data from the manager.

        Args:
            path: Path of the controller's prim.
        """
    def set_controller_height(self, arg0: str, arg1: float) -> None: 
        """
        Set the controller's height. This does not check for collisions.

        Args:
            path: Path of the controller's prim.
            val: New controller height.
        """
    def set_move(self, arg0: str, arg1: carb._carb.Float3) -> None: 
        """
        Move a controller by a given vector each frame (internally sets CharacterControllerAPI:MoveTarget). The vector is transformed from local to world space by either the controller's capsule or camera (if in first person mode) transformation. Use enable_worldspace_move to skip the local->world transform.

        Args:
            path: Path of the controller's prim.
            displacement: Displacement vector.
        """
    def set_position(self, arg0: str, arg1: carb._carb.Double3) -> None: 
        """
        Set the position of the center of the controller's collision. This does not check for collisions.

        Args:
            path: Path of the controller's prim.
            position: New center position of the controller.
        """
    pass
def acquire_physx_cct_interface(plugin_name: str = None, library_path: str = None) -> IPhysxCct:
    pass
def release_physx_cct_interface(arg0: IPhysxCct) -> None:
    pass
