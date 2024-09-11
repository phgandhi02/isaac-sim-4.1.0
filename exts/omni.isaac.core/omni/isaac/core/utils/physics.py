# Copyright (c) 2021-2023, NVIDIA CORPORATION. All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto. Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.
#

# python
from typing import Callable, Union

import omni.kit
import omni.kit.commands

# isaacsim
from omni.isaac.core.utils.stage import get_current_stage

# omniverse
from pxr import Sdf


def get_rigid_body_enabled(prim_path: str) -> Union[bool, None]:
    """Get the ``physics:rigidBodyEnabled`` attribute from the USD Prim at the given path

    Args:
        prim_path (str): The path to the USD Prim

    Returns:
        Any: The value of ``physics:rigidBodyEnabled`` attribute if it exists, and None if it does not exist.

    Example:

    .. code-block:: python

        >>> import omni.isaac.core.utils.physics as physics_utils
        >>>
        >>> # prim without the Physics' Rigid Body property
        >>> physics_utils.get_rigid_body_enabled("/World/Cube")
        None
        >>> # prim with the physics Rigid Body property added and enabled
        >>> physics_utils.get_rigid_body_enabled("/World/Cube")
        True
    """
    stage = get_current_stage()
    return stage.GetPrimAtPath(prim_path).GetAttribute("physics:rigidBodyEnabled").Get()


def set_rigid_body_enabled(_value, prim_path):
    """If it exists, set the ``physics:rigidBodyEnabled`` attribute on the USD Prim at the given path

    .. note::

        If the prim does not have the physics Rigid Body property added, calling this function will have no effect

    Args:
        _value (Any): Value to set ``physics:rigidBodyEnabled`` attribute to
        prim_path (str): The path to the USD Prim

    Example:

    .. code-block:: python

        >>> import omni.isaac.core.utils.physics as physics_utils
        >>>
        >>> physics_utils.set_rigid_body_enabled(False, "/World/Cube")
    """
    omni.kit.commands.execute(
        "ChangeProperty", prop_path=Sdf.Path(f"{prim_path}.physics:rigidBodyEnabled"), value=_value, prev=None
    )


async def simulate_async(seconds: float, steps_per_sec: int = 60, callback: Callable = None) -> None:
    """Helper function to simulate async for ``seconds * steps_per_sec frames``.

    Args:
        seconds (float): time in seconds to simulate for
        steps_per_sec (int, optional): steps per second. Defaults to 60.
        callback (Callable, optional): optional function to run every step. Defaults to None.

    Example:

    .. code-block:: python

        >>> import asyncio
        >>> import omni.isaac.core.utils.physics as physics_utils
        >>> from omni.kit.async_engine import run_coroutine
        >>>
        >>> async def task():
        ...     # simulate 1 second with 120 steps per second
        ...     await physics_utils.simulate_async(1, steps_per_sec=120)
        ...
        >>> run_coroutine(task())

    .. code-block:: python

        >>> import asyncio
        >>> import omni.isaac.core.utils.physics as physics_utils
        >>> from omni.kit.async_engine import run_coroutine
        >>>
        >>> def callback(*args, **kwargs):
        ...     print("callback:", args, kwargs)
        ...
        >>> async def task():
        ...     # simulate 1 second with 120 steps per second and call the callback on each step
        ...     await physics_utils.simulate_async(1, 120, callback)
        ...
        >>> run_coroutine(task())
    """
    for _ in range(int(steps_per_sec * seconds)):
        await omni.kit.app.get_app().next_update_async()
        if callback is not None:
            callback()
