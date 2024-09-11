# Copyright (c) 2024, NVIDIA CORPORATION. All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto. Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.
#
import os
import subprocess
import sys

import carb


class UIBuilder:
    """Manage extension UI"""

    def __init__(self, menu_path, host, port):
        self._menu = None
        self._editor_menu = None

        self._host = host
        self._port = port
        self._menu_path = menu_path

        # get application folder
        self._app_folder = carb.settings.get_settings().get_as_string("/app/folder")
        if not self._app_folder:
            self._app_folder = carb.tokens.get_tokens_interface().resolve("${app}")
        self._app_folder = os.path.normpath(os.path.join(self._app_folder, os.pardir))

    def startup(self):
        """Create menu item"""
        # menu item
        try:
            import omni.kit.ui
        except ImportError:
            self._editor_menu = None
        else:
            self._editor_menu = omni.kit.ui.get_editor_menu()
            if self._editor_menu:
                self._menu = self._editor_menu.add_item(self._menu_path, self._launch, toggle=False, value=False)

    def shutdown(self):
        """Clean up menu item"""
        if self._menu is not None:
            try:
                self._editor_menu.remove_item(self._menu)
            except:
                self._editor_menu.remove_item(self._menu_path)
        self._menu = None
        self._editor_menu = None

    def _launch(self, *args, **kwargs):
        """Launch a new VS Code window on the application path"""
        command = ["code", "-n", self._app_folder]
        if sys.platform.startswith("linux"):
            command = [" ".join(command)]
        carb.log_info(f"Launching VS Code: {command}")
        result = subprocess.run(command, shell=True, close_fds=True)
        # check process execution
        notification = f"Serving at {self._host}:{self._port}"
        if result.returncode:
            notification += f"\n\nUnable to launch VS Code (error code: {result.returncode})"
            if result.returncode == 1 or result.returncode == 127:
                notification += ".\nMake sure VS Code is installed and accessible on the system via the command 'code'"
            carb.log_warn(notification)
        # show notification in Kit window
        try:
            import omni.kit.notification_manager as notification_manager
        except ImportError:
            pass
        else:
            if result.returncode:
                status = notification_manager.NotificationStatus.WARNING
            else:
                status = notification_manager.NotificationStatus.INFO
            notification_manager.post_notification(
                notification,
                hide_after_timeout=not result.returncode,
                duration=3,
                status=status,
                button_infos=[notification_manager.NotificationButtonInfo("OK", on_complete=None)],
            )
