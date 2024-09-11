# Copyright (c) 2021-2023, NVIDIA CORPORATION. All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto. Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.
#

# python
from typing import Any

# omniverse
import carb


def set_carb_setting(carb_settings: carb.settings.ISettings, setting: str, value: Any) -> None:
    """Convenience to set the carb settings.

    Args:
        carb_settings (carb.settings.ISettings): The interface to carb settings.
        setting (str): Name of setting to change.
        value (Any): New value for the setting.

    Raises:
        TypeError: If the type of value does not match setting type.

    Example:

    .. code-block:: python

        >>> import carb
        >>> import omni.isaac.core.utils.carb as carb_utils
        >>>
        >>> settings = carb.settings.get_settings()
        >>> carb_utils.set_carb_setting(settings, "/physics/updateToUsd", True)
    """
    if isinstance(value, str):
        carb_settings.set_string(setting, value)
    elif isinstance(value, bool):
        carb_settings.set_bool(setting, value)
    elif isinstance(value, int):
        carb_settings.set_int(setting, value)
    elif isinstance(value, float):
        carb_settings.set_float(setting, value)
    else:
        raise TypeError(f"Value of type {type(value)} is not supported.")


def get_carb_setting(carb_settings: carb.settings.ISettings, setting: str) -> Any:
    """Convenience function to get settings.

    Args:
        carb_settings (carb.settings.ISettings): The interface to carb settings.
        setting (str): Name of setting to change.

    Returns:
        Any: Value for the setting.

    Example:

    .. code-block:: python

        >>> import carb
        >>> import omni.isaac.core.utils.carb as carb_utils
        >>>
        >>> settings = carb.settings.get_settings()
        >>> carb_utils.get_carb_setting(settings, "/physics/updateToUsd")
        False
    """
    return carb_settings.get(setting)
