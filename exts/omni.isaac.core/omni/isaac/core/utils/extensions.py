# Copyright (c) 2021-2023, NVIDIA CORPORATION. All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto. Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.
#

import omni.kit.app


def get_extension_id(extension_name: str) -> str:
    """Get extension id for a loaded extension

    Args:
        extension_name (str): name of the extension

    Returns:
        str: Full extension id

    Example:

    .. code-block:: python

        >>> import omni.isaac.core.utils.extensions as extensions_utils
        >>>
        >>> extensions_utils.get_extension_id("omni.kit.window.stage")
        omni.kit.window.stage-2.4.3
    """
    extension_manager = omni.kit.app.get_app().get_extension_manager()
    return extension_manager.get_enabled_extension_id(extension_name)


def get_extension_path(ext_id: str) -> str:
    """Get extension path for a loaded extension by its full id

    Args:
        ext_id (str): full id of extension

    Returns:
        str: Path to loaded extension root directory

    Example:

    .. code-block:: python

        >>> import omni.isaac.core.utils.extensions as extensions_utils
        >>>
        >>> ext_id = extensions_utils.get_extension_id("omni.kit.window.stage")
        >>> extensions_utils.get_extension_path(ext_id)
        /home/user/.local/share/ov/pkg/isaac_sim-<version>/kit/exts/omni.kit.window.stage
    """
    extension_manager = omni.kit.app.get_app().get_extension_manager()
    return extension_manager.get_extension_path(ext_id)


def get_extension_path_from_name(extension_name: str) -> str:
    """Get extension path for a loaded extension by its name

    Args:
        extension_name (str): name of the extension

    Returns:
        str: Path to loaded extension root directory

    Example:

    .. code-block:: python

        >>> import omni.isaac.core.utils.extensions as extensions_utils
        >>>
        >>> extensions_utils.get_extension_path_from_name("omni.kit.window.stage")
        /home/user/.local/share/ov/pkg/isaac_sim-<version>/kit/exts/omni.kit.window.stage
    """
    extension_manager = omni.kit.app.get_app().get_extension_manager()
    return extension_manager.get_extension_path(get_extension_id(extension_name))


def enable_extension(extension_name: str) -> bool:
    """Load an extension from the extension manager.

    Args:
        extension_name (str): name of the extension

    Returns:
        bool: True if extension could be loaded, False otherwise

    Example:

    .. code-block:: python

        >>> import omni.isaac.core.utils.extensions as extensions_utils
        >>>
        >>> extensions_utils.enable_extension("omni.kit.window.stage")
        True
    """
    extension_manager = omni.kit.app.get_app().get_extension_manager()
    return extension_manager.set_extension_enabled_immediate(extension_name, True)


def disable_extension(extension_name: str) -> bool:
    """Unload an extension.

    Args:
        extension_name (str): name of the extension

    Returns:
        bool: True if extension could be unloaded, False otherwise

    Example:

    .. code-block:: python

        >>> import omni.isaac.core.utils.extensions as extensions_utils
        >>>
        >>> extensions_utils.disable_extension("omni.kit.window.stage")
        True
    """
    extension_manager = omni.kit.app.get_app().get_extension_manager()
    return extension_manager.set_extension_enabled_immediate(extension_name, False)
