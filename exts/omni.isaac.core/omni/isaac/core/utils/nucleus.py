# Copyright (c) 2018-2023, NVIDIA CORPORATION. All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto. Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.
#

import typing

import carb
import omni.client
from omni.client._omniclient import CopyBehavior, Result
from omni.isaac.nucleus import *


def get_url_root(url: str) -> str:
    """Get root from URL or path
    Args:
        url (str): full http or omniverse path

    Returns:
        str: Root path or URL or Nucleus server
    """
    carb.log_warn("Deprecation warning: this function has been moved to omni.isaac.nucleus")
    return nucleus.get_url_root(url=url)


def create_folder(server: str, path: str) -> bool:
    """Create a folder on server

    Args:
        server (str): Name of Nucleus server
        path (str): Path to folder

    Returns:
        bool: True if folder is created successfully
    """
    carb.log_warn("Deprecation warning: this function has been moved to omni.isaac.nucleus")
    return nucleus.create_folder(server=server, path=path)


def delete_folder(server: str, path: str) -> bool:
    """Remove folder and all of its contents

    Args:
        server (str): Name of Nucleus server
        path (str): Path to folder

    Returns:
        bool: True if folder is deleted successfully
    """
    carb.log_warn("Deprecation warning: this function has been moved to omni.isaac.nucleus")
    return nucleus.delete_folder(server=server, path=path)


async def _list_files(url: str) -> typing.Tuple[str, typing.List]:
    """List files under a URL

    Args:
        url (str): URL of Nucleus server with path to folder

    Returns:
        root (str): Root of URL of Nucleus server
        paths (typing.List): List of path to each file
    """
    carb.log_warn("Deprecation warning: this function has been moved to omni.isaac.nucleus")
    results = await nucleus._list_files(url=url)

    return results


async def download_assets_async(
    src: str,
    dst: str,
    progress_callback,
    concurrency: int = 10,
    copy_behaviour: omni.client._omniclient.CopyBehavior = CopyBehavior.OVERWRITE,
    copy_after_delete: bool = True,
    timeout: float = 300.0,
) -> omni.client._omniclient.Result:
    """Download assets from S3 bucket

    Args:
        src (str): URL of S3 bucket as source
        dst (str): URL of Nucleus server to copy assets to
        progress_callback: Callback function to keep track of progress of copy
        concurrency (int): Number of concurrent copy operations. Default value: 3
        copy_behaviour (omni.client._omniclient.CopyBehavior): Behavior if the destination exists. Default value: OVERWRITE
        copy_after_delete (bool): True if destination needs to be deleted before a copy. Default value: True
        timeout (float): Default value: 300 seconds

    Returns:
        Result (omni.client._omniclient.Result): Result of copy
    """
    # omni.client is a singleton, import locally to allow to run with multiprocessing

    carb.log_warn("Deprecation warning: this function has been moved to omni.isaac.nucleus")

    result = await nucleus.download_assets_async(
        src=src,
        dst=dst,
        progress_callback=progress_callback,
        concurrency=concurrency,
        copy_behaviour=copy_behaviour,
        copy_after_delete=copy_after_delete,
        timeout=timeout,
    )

    return result


def check_server(server: str, path: str, timeout: float = 10.0) -> bool:
    """Check a specific server for a path

    Args:
        server (str): Name of Nucleus server
        path (str): Path to search
        timeout (float): Default value: 10 seconds

    Returns:
        bool: True if folder is found
    """
    carb.log_warn("Deprecation warning: this function has been moved to omni.isaac.nucleus")

    return nucleus.check_server(server=server, path=path, timeout=timeout)


async def check_server_async(server: str, path: str, timeout: float = 10.0) -> bool:
    """Check a specific server for a path (asynchronous version).

    Args:
        server (str): Name of Nucleus server
        path (str): Path to search
        timeout (float): Default value: 10 seconds

    Returns:
        bool: True if folder is found
    """
    carb.log_warn("Deprecation warning: this function has been moved to omni.isaac.nucleus")

    result = await nucleus.check_server_async(server=server, path=path, timeout=timeout)

    return result


def build_server_list() -> typing.List:
    """Return list with all known servers to check

    Returns:
        all_servers (typing.List): List of servers found
    """
    carb.log_warn("Deprecation warning: this function has been moved to omni.isaac.nucleus")

    return nucleus.build_server_list()


def find_nucleus_server(suffix: str) -> typing.Tuple[bool, str]:
    """Attempts to determine best Nucleus server to use based on existing mountedDrives setting and the
    default server specified in json config at "/persistent/isaac/asset_root/". Call is blocking

    Args:
        suffix (str): Path to folder to search for. Default value: /Isaac

    Returns:
        bool: True if Nucleus server with suffix is found
        url (str): URL of found Nucleus
    """
    carb.log_warn("find_nucleus_server() is deprecated. Use get_assets_root_path().")
    return False, ""


def get_server_path(suffix: str = "") -> typing.Union[str, None]:
    """Tries to find a Nucleus server with specific path

    Args:
        suffix (str): Path to folder to search for.

    Returns:
        url (str): URL of Nucleus server with path to folder.
        Returns None if Nucleus server not found.
    """
    carb.log_warn("Deprecation warning: this function has been moved to omni.isaac.nucleus")

    return nucleus.get_server_path(suffix=suffix)


async def get_server_path_async(suffix: str = "") -> typing.Union[str, None]:
    """Tries to find a Nucleus server with specific path (asynchronous version).

    Args:
        suffix (str): Path to folder to search for.

    Returns:
        url (str): URL of Nucleus server with path to folder.
        Returns None if Nucleus server not found.
    """
    carb.log_warn("Deprecation warning: this function has been moved to omni.isaac.nucleus")

    result = await nucleus.get_server_path_async(suffix=suffix)
    return result


def verify_asset_root_path(path: str) -> typing.Tuple[omni.client.Result, str]:
    """Attempts to determine Isaac assets version and check if there are updates.
    (asynchronous version)

    Args:
        path (str): URL or path of asset root to verify

    Returns:
        omni.client.Result: OK if Assets verified
        ver (str): Version of Isaac Sim assets
    """
    carb.log_warn("Deprecation warning: this function has been moved to omni.isaac.nucleus")

    return nucleus.verify_asset_root_path(path=path)


def get_full_asset_path(path: str) -> typing.Union[str, None]:
    """Tries to find the full asset path on connected servers

    Args:
        path (str): Path of asset from root to verify

    Returns:
        url (str): URL or full path to assets.
        Returns None if assets not found.
    """

    carb.log_warn("Deprecation warning: this function has been moved to omni.isaac.nucleus")

    return nucleus.get_full_asset_path(path=path)


async def get_full_asset_path_async(path: str) -> typing.Union[str, None]:
    """Tries to find the full asset path on connected servers (asynchronous version).

    Args:
        path (str): Path of asset from root to verify

    Returns:
        url (str): URL or full path to assets.
        Returns None if assets not found.
    """

    carb.log_warn("Deprecation warning: this function has been moved to omni.isaac.nucleus")

    result = await nucleus.get_full_asset_path_async(path=path)

    return result


def get_nvidia_asset_root_path() -> typing.Union[str, None]:
    """Tries to find the root path to the NVIDIA assets

    Returns:
        url (str): URL or root path to NVIDIA assets folder.
        Returns None if NVIDIA assets not found.
    """

    carb.log_warn("Deprecation warning: this function has been moved to omni.isaac.nucleus")

    return nucleus.get_nvidia_asset_root_path()


def get_isaac_asset_root_path() -> typing.Union[str, None]:
    """Tries to find the root path to the Isaac Sim assets

    Returns:
        url (str): URL or root path to Isaac Sim assets folder.
        Returns None if Isaac Sim assets not found.
    """

    carb.log_warn("Deprecation warning: this function has been moved to omni.isaac.nucleus")

    return nucleus.get_isaac_asset_root_path()


def get_assets_root_path() -> typing.Union[str, None]:
    """Tries to find the root path to the Isaac Sim assets on a Nucleus server

    Returns:
        url (str): URL of Nucleus server with root path to assets folder.
        Returns None if Nucleus server not found.
    """

    carb.log_warn("Deprecation warning: this function has been moved to omni.isaac.nucleus")

    return nucleus.get_assets_root_path()


async def get_assets_root_path_async() -> typing.Union[str, None]:
    """Tries to find the root path to the Isaac Sim assets on a Nucleus server (asynchronous version).

    Returns:
        url (str): URL of Nucleus server with root path to assets folder.
        Returns None if Nucleus server not found.
    """

    carb.log_warn("Deprecation warning: this function has been moved to omni.isaac.nucleus")

    results = await nucleus.get_assets_root_path_async()
    return results


def get_assets_server() -> typing.Union[str, None]:
    """Tries to find a server with the Isaac Sim assets

    Returns:
        url (str): URL of Nucleus server with the Isaac Sim assets
            Returns None if Nucleus server not found.
    """
    carb.log_warn("Deprecation warning: this function has been moved to omni.isaac.nucleus")

    return nucleus.get_assets_server()


async def _collect_files(url: str) -> typing.Tuple[str, typing.List]:
    """Collect files under a URL.

    Args:
        url (str): URL of Nucleus server with path to folder

    Returns:
        root (str): Root of URL of Nucleus server
        paths (typing.List): List of path to each file
    """
    carb.log_warn("Deprecation warning: this function has been moved to omni.isaac.nucleus")

    results = await nucleus._collect_files(url=url)

    return results


async def is_dir_async(path: str) -> bool:
    """Check if path is a folder

    Args:
        path (str): Path to folder

    Returns:
        bool: True if path is a folder
    """
    carb.log_warn("Deprecation warning: this function has been moved to omni.isaac.nucleus")

    results = await nucleus.is_dir_async(path=path)

    return results


async def is_file_async(path: str) -> bool:
    """Check if path is a file

    Args:
        path (str): Path to file

    Returns:
        bool: True if path is a file
    """

    carb.log_warn("Deprecation warning: this function has been moved to omni.isaac.nucleus")

    results = await nucleus.is_file_async(path=path)

    return results


def is_file(path: str) -> bool:
    """Check if path is a file

    Args:
        path (str): Path to file

    Returns:
        bool: True if path is a file
    """

    carb.log_warn("Deprecation warning: this function has been moved to omni.isaac.nucleus")

    return nucleus.is_file(path=path)


async def recursive_list_folder(path: str) -> typing.List:
    """Recursively list all files

    Args:
        path (str): Path to folder

    Returns:
        paths (typing.List): List of path to each file
    """
    carb.log_warn("Deprecation warning: this function has been moved to omni.isaac.nucleus")

    results = await nucleus.recursive_list_folder(path=path)

    return results


async def list_folder(path: str) -> typing.Tuple[typing.List, typing.List]:
    """List files and sub-folders from root path

    Args:
        path (str): Path to root folder

    Raises:
        Exception: When unable to find files under the path.

    Returns:
        files (typing.List): List of path to each file
        dirs (typing.List): List of path to each sub-folder
    """

    carb.log_warn("Deprecation warning: this function has been moved to omni.isaac.nucleus")

    results = await nucleus.list_folder(path=path)

    return results
