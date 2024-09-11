# Copyright (c) 2021-2023, NVIDIA CORPORATION. All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto. Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.
#

from pxr import UsdGeom

AXES_INDICES = {"X": 0, "x": 0, "Y": 1, "y": 1, "Z": 2, "z": 2}
"""Mapping from axis name to axis ID

Example:

.. code-block:: python

    >>> import omni.isaac.core.utils.constants as constants_utils
    >>>
    >>> # get the x-axis index
    >>> constants_utils.AXES_INDICES['x']
    0
    >>> constants_utils.AXES_INDICES['X']
    0
"""

AXES_TOKEN = {
    "X": UsdGeom.Tokens.x,
    "x": UsdGeom.Tokens.x,
    "Y": UsdGeom.Tokens.y,
    "y": UsdGeom.Tokens.y,
    "Z": UsdGeom.Tokens.z,
    "z": UsdGeom.Tokens.z,
}
"""Mapping from axis name to axis USD token

    >>> import omni.isaac.core.utils.constants as constants_utils
    >>>
    >>> # get the x-axis USD token
    >>> constants_utils.AXES_TOKEN['x']
    X
    >>> constants_utils.AXES_TOKEN['X']
    X
"""
