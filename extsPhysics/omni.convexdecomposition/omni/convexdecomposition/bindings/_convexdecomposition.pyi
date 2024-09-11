"""pybind11 omni.convexdecomposition bindings"""
from __future__ import annotations
import omni.convexdecomposition.bindings._convexdecomposition
import typing

__all__ = [
    "ConvexDecomposition",
    "Parameters",
    "SimpleMesh",
    "VoxelFillMode",
    "acquire_convexdecomposition_interface",
    "release_convexdecomposition_interface",
    "release_convexdecomposition_interface_scripting"
]


class ConvexDecomposition():
    def apply_sphere_approximation(self, arg0: str, arg1: int) -> bool: ...
    def begin_vhacd(self, handle: int, smm: SimpleMesh, mp: Parameters) -> bool: ...
    def cancel_vhacd(self, handle: int) -> bool: ...
    def create_vhacd(self) -> int: ...
    def get_convex_hull(self, handle: int, index: int, smm: SimpleMesh) -> bool: ...
    def get_convex_hull_count(self, arg0: int) -> int: ...
    def is_complete(self, arg0: int) -> bool: ...
    def release_vhacd(self, handle: int) -> None: ...
    def save_obj(self, fname: str, smm: SimpleMesh, flip_winding_order: bool) -> bool: ...
    pass
class Parameters():
    def __init__(self) -> None: ...
    @property
    def error_percentage(self) -> float:
        """
        :type: float
        """
    @error_percentage.setter
    def error_percentage(self, arg0: float) -> None:
        pass
    @property
    def max_convex_hull_count(self) -> int:
        """
        :type: int
        """
    @max_convex_hull_count.setter
    def max_convex_hull_count(self, arg0: int) -> None:
        pass
    @property
    def max_hull_vertices(self) -> int:
        """
        :type: int
        """
    @max_hull_vertices.setter
    def max_hull_vertices(self, arg0: int) -> None:
        pass
    @property
    def notify_complete_callback(self) -> typing.Callable[[int], None]:
        """
        :type: typing.Callable[[int], None]
        """
    @notify_complete_callback.setter
    def notify_complete_callback(self, arg0: typing.Callable[[int], None]) -> None:
        pass
    @property
    def voxel_fill_mode(self) -> VoxelFillMode:
        """
        :type: VoxelFillMode
        """
    @voxel_fill_mode.setter
    def voxel_fill_mode(self, arg0: VoxelFillMode) -> None:
        pass
    @property
    def voxel_resolution(self) -> int:
        """
        :type: int
        """
    @voxel_resolution.setter
    def voxel_resolution(self, arg0: int) -> None:
        pass
    pass
class SimpleMesh():
    def __init__(self) -> None: ...
    @property
    def center(self) -> carb._carb.Double3:
        """
        :type: carb._carb.Double3
        """
    @center.setter
    def center(self, arg0: carb._carb.Double3) -> None:
        pass
    @property
    def indices(self) -> typing.List[int]:
        """
        :type: typing.List[int]
        """
    @indices.setter
    def indices(self, arg0: typing.List[int]) -> None:
        pass
    @property
    def vertices(self) -> typing.List[float]:
        """
        :type: typing.List[float]
        """
    @vertices.setter
    def vertices(self, arg0: typing.List[float]) -> None:
        pass
    @property
    def volume(self) -> float:
        """
        :type: float
        """
    @volume.setter
    def volume(self, arg0: float) -> None:
        pass
    pass
class VoxelFillMode():
    """
    Members:

      FLOOD_FILL

      SURFACE_ONLY

      RAYCAST_FILL
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
    FLOOD_FILL: omni.convexdecomposition.bindings._convexdecomposition.VoxelFillMode # value = <VoxelFillMode.FLOOD_FILL: 0>
    RAYCAST_FILL: omni.convexdecomposition.bindings._convexdecomposition.VoxelFillMode # value = <VoxelFillMode.RAYCAST_FILL: 2>
    SURFACE_ONLY: omni.convexdecomposition.bindings._convexdecomposition.VoxelFillMode # value = <VoxelFillMode.SURFACE_ONLY: 1>
    __members__: dict # value = {'FLOOD_FILL': <VoxelFillMode.FLOOD_FILL: 0>, 'SURFACE_ONLY': <VoxelFillMode.SURFACE_ONLY: 1>, 'RAYCAST_FILL': <VoxelFillMode.RAYCAST_FILL: 2>}
    pass
def acquire_convexdecomposition_interface(plugin_name: str = None, library_path: str = None) -> ConvexDecomposition:
    pass
def release_convexdecomposition_interface(arg0: ConvexDecomposition) -> None:
    pass
def release_convexdecomposition_interface_scripting(*args, **kwargs) -> typing.Any:
    pass
