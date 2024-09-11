# Copyright (c) 2021-2023, NVIDIA CORPORATION. All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto. Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.
#

# python
import typing

# omniverse
import carb
import numpy as np

# isaacsim
from omni.isaac.core.utils.prims import get_prim_at_path
from pxr import Gf, Usd, UsdGeom


def recompute_extents(
    prim: UsdGeom.Boundable, time: Usd.TimeCode = Usd.TimeCode.Default(), include_children: bool = False
) -> None:
    """Recomputes and overwrites the extents attribute for a UsdGeom.Boundable prim

    Args:
        prim (UsdGeom.Boundable): Input prim to recompute extents for
        time (Usd.TimeCode, optional): timecode to use for computing extents. Defaults to Usd.TimeCode.Default().
        include_children (bool, optional): include children of specified prim in calculation. Defaults to False.

    Raises:
        ValueError: If prim is not of UsdGeom.Boundable type

    Example:

    .. code-block:: python

        >>> import omni.isaac.core.utils.bounds as bounds_utils
        >>> import omni.isaac.core.utils.stage as stage_utils
        >>>
        >>> prim = stage_utils.get_current_stage().GetPrimAtPath("/World/Cube")
        >>> bounds_utils.recompute_extents(prim)
    """
    #
    def update_extents(prim: UsdGeom.Boundable, time: Usd.TimeCode = Usd.TimeCode.Default()):
        compute_prim = UsdGeom.Boundable(prim)
        if compute_prim:
            bounds = []
            mesh = UsdGeom.Mesh(compute_prim)
            if mesh:
                bounds = mesh.ComputeExtent(mesh.GetPointsAttr().Get())
            else:
                bounds = UsdGeom.Boundable.ComputeExtentFromPlugins(compute_prim, time)

            if compute_prim.GetExtentAttr().HasValue():
                compute_prim.GetExtentAttr().Set(bounds)
            else:
                compute_prim.CreateExtentAttr(bounds)
        else:
            raise ValueError(f"Input prim is not of type UsdGeom.Boundable, is instead {type(prim)}")

    if include_children:
        for p in Usd.PrimRange(prim.GetPrim()):
            try:
                update_extents(p, time)
            except ValueError:
                carb.log_info(f"Skipping {p}, not boundable")
    else:
        update_extents(prim, time)


def create_bbox_cache(time: Usd.TimeCode = Usd.TimeCode.Default(), use_extents_hint: bool = True) -> UsdGeom.BBoxCache:
    """Helper function to create a Bounding Box Cache object that can be used for computations

    Args:
        time (Usd.TimeCode, optional): time at which cache should be initialized. Defaults to Usd.TimeCode.Default().
        use_extents_hint (bool, optional): Use existing extents attribute on prim to compute bounding box. Defaults to True.

    Returns:
        UsdGeom.BboxCache: Initialized bbox cache

    Example:

    .. code-block:: python

        >>> import omni.isaac.core.utils.bounds as bounds_utils
        >>>
        >>> bounds_utils.create_bbox_cache()
        <pxr.UsdGeom.BBoxCache object at 0x7f6720b8bc90>
    """
    return UsdGeom.BBoxCache(time=time, includedPurposes=[UsdGeom.Tokens.default_], useExtentsHint=use_extents_hint)


def compute_aabb(bbox_cache: UsdGeom.BBoxCache, prim_path: str, include_children: bool = False) -> np.array:
    """Compute an Axis-Aligned Bounding Box (AABB) for a given ``prim_path``

    A combined AABB is computed if ``include_children`` is True

    Args:
        bbox_cache (UsdGeom.BboxCache): Existing Bounding box cache to use for computation
        prim_path (str): prim path to compute AABB for
        include_children (bool, optional): include children of specified prim in calculation. Defaults to False.

    Returns:
        np.array: Bounding box for this prim, [min x, min y, min z, max x, max y, max z]

    Example:

    .. code-block:: python

        >>> import omni.isaac.core.utils.bounds as bounds_utils
        >>>
        >>> # 1 stage unit length cube centered at (0.0, 0.0, 0.0)
        >>> cache = bounds_utils.create_bbox_cache()
        >>> bounds_utils.compute_aabb(cache, prim_path="/World/Cube")
        [-0.5 -0.5 -0.5  0.5  0.5  0.5]
        >>>
        >>> # the same cube rotated 45 degrees around the z-axis
        >>> cache = bounds_utils.create_bbox_cache()
        >>> bounds_utils.compute_aabb(cache, prim_path="/World/Cube")
        [-0.70710678  -0.70710678  -0.5  0.70710678  0.70710678  0.5]
    """
    total_bounds = Gf.BBox3d()
    prim = get_prim_at_path(prim_path)
    if include_children:
        for p in Usd.PrimRange(prim):
            total_bounds = Gf.BBox3d.Combine(
                total_bounds, Gf.BBox3d(bbox_cache.ComputeWorldBound(p).ComputeAlignedRange())
            )
    else:
        total_bounds = Gf.BBox3d(bbox_cache.ComputeWorldBound(prim).ComputeAlignedRange())

    range = total_bounds.GetRange()
    return np.array([*range.GetMin(), *range.GetMax()])


def compute_combined_aabb(bbox_cache: UsdGeom.BBoxCache, prim_paths: typing.List[str]) -> np.array:
    """Computes a combined Axis-Aligned Bounding Box (AABB) given a list of prim paths

    Args:
        bbox_cache (UsdGeom.BboxCache): Existing Bounding box cache to use for computation
        prim_paths (typing.List[str]): List of prim paths to compute combined AABB for

    Returns:
        np.array: Bounding box for input prims, [min x, min y, min z, max x, max y, max z]

    Example:

    .. code-block:: python

        >>> import omni.isaac.core.utils.bounds as bounds_utils
        >>>
        >>> # 1 stage unit length cube centered at (0.0, 0.0, 0.0)
        >>> # with a 1 stage unit diameter sphere centered at (-0.5, 0.5, 0.5)
        >>> cache = bounds_utils.create_bbox_cache()
        >>> bounds_utils.compute_combined_aabb(cache, prim_paths=["/World/Cube", "/World/Sphere"])
        [-1.  -0.5 -0.5  0.5  1.   1. ]
    """
    total_bounds = Gf.BBox3d()
    for prim_path in prim_paths:
        prim = get_prim_at_path(prim_path)
        bounds = bbox_cache.ComputeWorldBound(prim)
        total_bounds = Gf.BBox3d.Combine(total_bounds, Gf.BBox3d(bounds.ComputeAlignedRange()))
    range = total_bounds.GetRange()
    return np.array([*range.GetMin(), *range.GetMax()])


def compute_obb(bbox_cache: UsdGeom.BBoxCache, prim_path: str) -> typing.Tuple[np.ndarray, np.ndarray, np.ndarray]:
    """Computes the Oriented Bounding Box (OBB) of a prim

    .. note::

        * The OBB does not guarantee the smallest possible bounding box, it rotates and scales the default AABB.
        * The rotation matrix incorporates any scale factors applied to the object.
        * The `half_extent` values do not include these scaling effects.

    Args:
        bbox_cache (UsdGeom.BBoxCache): USD Bounding Box Cache object to use for computation
        prim_path (str): Prim path to compute OBB for

    Returns:
        Tuple[np.ndarray, np.ndarray, np.ndarray]: A tuple containing the following OBB information:
            - The centroid of the OBB as a NumPy array.
            - The axes of the OBB as a 2D NumPy array, where each row represents a different axis.
            - The half extent of the OBB as a NumPy array.

    Example:

    .. code-block:: python

        >>> import omni.isaac.core.utils.bounds as bounds_utils
        >>>
        >>> # 1 stage unit length cube centered at (0.0, 0.0, 0.0)
        >>> cache = bounds_utils.create_bbox_cache()
        >>> centroid, axes, half_extent = bounds_utils.compute_obb(cache, prim_path="/World/Cube")
        >>> centroid
        [0. 0. 0.]
        >>> axes
        [[1. 0. 0.]
         [0. 1. 0.]
         [0. 0. 1.]]
        >>> half_extent
        [0.5 0.5 0.5]
        >>>
        >>> # the same cube rotated 45 degrees around the z-axis
        >>> cache = bounds_utils.create_bbox_cache()
        >>> centroid, axes, half_extent = bounds_utils.compute_obb(cache, prim_path="/World/Cube")
        >>> centroid
        [0. 0. 0.]
        >>> axes
        [[ 0.70710678  0.70710678  0.        ]
         [-0.70710678  0.70710678  0.        ]
         [ 0.          0.          1.        ]]
        >>> half_extent
        [0.5 0.5 0.5]
    """
    # Compute the BBox3d for the prim
    prim = get_prim_at_path(prim_path)
    bound = bbox_cache.ComputeWorldBound(prim)

    # Compute the translated centroid of the world bound
    centroid = bound.ComputeCentroid()

    # Compute the axis vectors of the OBB
    # NOTE: The rotation matrix incorporates the scale factors applied to the object
    rotation_matrix = bound.GetMatrix().ExtractRotationMatrix()
    x_axis = rotation_matrix.GetRow(0)
    y_axis = rotation_matrix.GetRow(1)
    z_axis = rotation_matrix.GetRow(2)

    # Compute the half-lengths of the OBB along each axis
    # NOTE the size/extent values do not include any scaling effects
    half_extent = bound.GetRange().GetSize() * 0.5

    return np.array([*centroid]), np.array([[*x_axis], [*y_axis], [*z_axis]]), np.array(half_extent)


def get_obb_corners(centroid: np.ndarray, axes: np.ndarray, half_extent: np.ndarray) -> np.ndarray:
    """Computes the corners of the Oriented Bounding Box (OBB) from the given OBB information

    Args:
        centroid (np.ndarray): The centroid of the OBB as a NumPy array.
        axes (np.ndarray): The axes of the OBB as a 2D NumPy array, where each row represents a different axis.
        half_extent (np.ndarray): The half extent of the OBB as a NumPy array.

    Returns:
        np.ndarray: NumPy array of shape (8, 3) containing each corner location of the OBB

        :math:`c_0 = (x_{min}, y_{min}, z_{min})`
        |br| :math:`c_1 = (x_{min}, y_{min}, z_{max})`
        |br| :math:`c_2 = (x_{min}, y_{max}, z_{min})`
        |br| :math:`c_3 = (x_{min}, y_{max}, z_{max})`
        |br| :math:`c_4 = (x_{max}, y_{min}, z_{min})`
        |br| :math:`c_5 = (x_{max}, y_{min}, z_{max})`
        |br| :math:`c_6 = (x_{max}, y_{max}, z_{min})`
        |br| :math:`c_7 = (x_{max}, y_{max}, z_{max})`

    Example:

    .. code-block:: python

        >>> import omni.isaac.core.utils.bounds as bounds_utils
        >>>
        >>> cache = bounds_utils.create_bbox_cache()
        >>> centroid, axes, half_extent = bounds_utils.compute_obb(cache, prim_path="/World/Cube")
        >>> bounds_utils.get_obb_corners(centroid, axes, half_extent)
        [[-0.5 -0.5 -0.5]
         [-0.5 -0.5  0.5]
         [-0.5  0.5 -0.5]
         [-0.5  0.5  0.5]
         [ 0.5 -0.5 -0.5]
         [ 0.5 -0.5  0.5]
         [ 0.5  0.5 -0.5]
         [ 0.5  0.5  0.5]]
    """
    corners = [
        centroid - axes[0] * half_extent[0] - axes[1] * half_extent[1] - axes[2] * half_extent[2],
        centroid - axes[0] * half_extent[0] - axes[1] * half_extent[1] + axes[2] * half_extent[2],
        centroid - axes[0] * half_extent[0] + axes[1] * half_extent[1] - axes[2] * half_extent[2],
        centroid - axes[0] * half_extent[0] + axes[1] * half_extent[1] + axes[2] * half_extent[2],
        centroid + axes[0] * half_extent[0] - axes[1] * half_extent[1] - axes[2] * half_extent[2],
        centroid + axes[0] * half_extent[0] - axes[1] * half_extent[1] + axes[2] * half_extent[2],
        centroid + axes[0] * half_extent[0] + axes[1] * half_extent[1] - axes[2] * half_extent[2],
        centroid + axes[0] * half_extent[0] + axes[1] * half_extent[1] + axes[2] * half_extent[2],
    ]
    return np.array(corners)


def compute_obb_corners(bbox_cache: UsdGeom.BBoxCache, prim_path: str) -> np.ndarray:
    """Computes the corners of the Oriented Bounding Box (OBB) of a prim

    Args:
        bbox_cache (UsdGeom.BBoxCache): Bounding Box Cache object to use for computation
        prim_path (str): Prim path to compute OBB for

    Returns:
        np.ndarray: NumPy array of shape (8, 3) containing each corner location of the OBB

        :math:`c_0 = (x_{min}, y_{min}, z_{min})`
        |br| :math:`c_1 = (x_{min}, y_{min}, z_{max})`
        |br| :math:`c_2 = (x_{min}, y_{max}, z_{min})`
        |br| :math:`c_3 = (x_{min}, y_{max}, z_{max})`
        |br| :math:`c_4 = (x_{max}, y_{min}, z_{min})`
        |br| :math:`c_5 = (x_{max}, y_{min}, z_{max})`
        |br| :math:`c_6 = (x_{max}, y_{max}, z_{min})`
        |br| :math:`c_7 = (x_{max}, y_{max}, z_{max})`

    Example:

    .. code-block:: python

        >>> import omni.isaac.core.utils.bounds as bounds_utils
        >>>
        >>> cache = bounds_utils.create_bbox_cache()
        >>> bounds_utils.compute_obb_corners(cache, prim_path="/World/Cube")
        [[-0.5 -0.5 -0.5]
         [-0.5 -0.5  0.5]
         [-0.5  0.5 -0.5]
         [-0.5  0.5  0.5]
         [ 0.5 -0.5 -0.5]
         [ 0.5 -0.5  0.5]
         [ 0.5  0.5 -0.5]
         [ 0.5  0.5  0.5]]
    """
    centroid, axis, half_extent = compute_obb(bbox_cache, prim_path)
    return get_obb_corners(centroid, axis, half_extent)
