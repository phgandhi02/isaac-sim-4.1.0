# Copyright (c) 2018-2023, NVIDIA CORPORATION. All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto. Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.
#

import asyncio
import json
import typing

import carb
import omni
import omni.client
from omni.client._omniclient import Result
from pxr import Usd


def build_server_list() -> typing.List:
    """Return list with all known servers to check

    Returns:
        all_servers (typing.List): List of servers found
    """
    mounted_drives = carb.settings.get_settings().get_settings_dictionary("/persistent/app/omniverse/mountedDrives")
    all_servers = []
    if mounted_drives is not None:
        mounted_dict = json.loads(mounted_drives.get_dict())
        for drive in mounted_dict.items():
            all_servers.append(drive[1])
    else:
        carb.log_warn("/persistent/app/omniverse/mountedDrives setting not found")

    return all_servers


async def check_server_async(server: str, path: str, timeout: float = 10.0) -> bool:
    """Check a specific server for a path (asynchronous version).

    Args:
        server (str): Name of Nucleus server
        path (str): Path to search
        timeout (float): Default value: 10 seconds

    Returns:
        bool: True if folder is found
    """
    carb.log_info("Checking path: {}{}".format(server, path))

    try:
        result, _ = await asyncio.wait_for(omni.client.stat_async("{}{}".format(server, path)), timeout)
        if result == Result.OK:
            carb.log_info("Success: {}{}".format(server, path))
            return True
        else:
            carb.log_info("Failure: {}{} not accessible".format(server, path))
            return False
    except asyncio.TimeoutError:
        carb.log_warn(f"check_server_async() timeout {timeout}")
        return False
    except Exception as ex:
        carb.log_warn(f"Exception: {type(ex).__name__}")
        return False


async def open_stage_async(usd_path: str) -> bool:
    """
    Open the given usd file and replace currently opened stage
    Args:
        usd_path (str): Path to open
    """
    if not Usd.Stage.IsSupportedFile(usd_path):
        raise ValueError("Only USD files can be loaded with this method")
    usd_context = omni.usd.get_context()
    usd_context.disable_save_to_recent_files()
    (result, error) = await omni.usd.get_context().open_stage_async(usd_path)
    usd_context.enable_save_to_recent_files()
    return (result, error)
