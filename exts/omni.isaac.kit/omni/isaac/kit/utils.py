# Copyright (c) 2021-2023, NVIDIA CORPORATION. All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto. Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.
#

# The functions in this file are copies from omni.isaac.core to make the dependency structure cleaner.

from __future__ import annotations

from typing import Any

import carb
from omni.kit.usd import layers


def set_carb_setting(carb_settings: carb.settings.ISettings, setting: str, value: Any) -> None:
    """Convenience function to set settings.

    Arguments:
        setting (str): Name of setting to change.
        value (Any): New value for the setting.

    Raises:
        TypeError: If the type of value does not match setting type.
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


def open_stage(usd_path: str) -> bool:
    """
    Open the given usd file and replace currently opened stage
    Args:
        usd_path (str): Path to open
    """
    import omni.usd
    from pxr import Usd

    if not Usd.Stage.IsSupportedFile(usd_path):
        raise ValueError("Only USD files can be loaded with this method")
    usd_context = omni.usd.get_context()
    usd_context.disable_save_to_recent_files()
    result = omni.usd.get_context().open_stage(usd_path)
    usd_context.enable_save_to_recent_files()
    return result


def create_new_stage() -> Usd.Stage:
    """[summary]

    Returns:
        bool: [description]
    """
    import omni.usd

    return omni.usd.get_context().new_stage()


def save_stage(usd_path: str) -> bool:
    """
    Save usd file to path, it will be overwritten with the current stage
    Args:
        usd_path (str): Path to save the current stage to
    """
    import omni.usd
    from pxr import Usd

    if not Usd.Stage.IsSupportedFile(usd_path):
        raise ValueError("Only USD files can be saved with this method")
    result = omni.usd.get_context().save_as_stage(usd_path)
    return result


def set_livesync_stage(usd_path: str, enable: bool) -> bool:
    """[summary]

    Args:
        usd_path (str): path to enable live sync for, it will be overwritten with the current stage
        enable (bool): True to enable livesync, false to disable livesync

    Returns:
        bool: [description]
    """
    import omni.usd

    # TODO: Check that the provided usd_path exists
    if save_stage(usd_path):
        if enable:
            usd_path_split = usd_path.split("/")
            live_session = layers.get_live_syncing().find_live_session_by_name(usd_path, "Default")
            if live_session is None:
                live_session = layers.get_live_syncing().create_live_session(name="Default")
            result = layers.get_live_syncing().join_live_session(live_session)
            return True
        else:
            layers.get_live_syncing().stop_live_session(usd_path)
            return True
    else:
        return False


def is_stage_loading() -> bool:
    """
    bool: Convenience function to see if any files are being loaded. True if loading, False otherwise
    """
    import omni.usd

    context = omni.usd.get_context()
    if context is None:
        return False
    else:
        _, _, loading = context.get_stage_loading_status()
        return loading > 0
