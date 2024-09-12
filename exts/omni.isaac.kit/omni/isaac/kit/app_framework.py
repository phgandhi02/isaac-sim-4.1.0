# Copyright (c) 2023, NVIDIA CORPORATION. All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto. Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.
#
import builtins
import os
import sys
import typing

import carb
import omni.kit.app


class AppFramework:
    """Minimal omniverse application that launches without any application config"""

    def __init__(self, name: str = "kit", argv=[]):

        builtins.ISAAC_LAUNCHED_FROM_TERMINAL = False

        self._framework = carb.get_framework()
        self._framework.load_plugins(
            loaded_file_wildcards=["omni.kit.app.plugin"],
            search_paths=[os.path.abspath(f'{os.environ["CARB_APP_PATH"]}/kernel/plugins')],
        )
        self._app = omni.kit.app.get_app()
        # Path to where kit was built to
        app_root = os.environ["CARB_APP_PATH"]

        # first arg should be the file
        argv.insert(0, os.path.abspath(__file__))

        # allow root so apps don't fail when running in docker/root environments
        if sys.platform.startswith("linux") and os.geteuid() == 0 and "--allow-root" not in argv:
            argv.append("--allow-root")
        self._app.startup(name, app_root, argv)

    def update(self) -> None:
        """
        Convenience function to step the application forward one frame
        """
        self._app.update()

    def close(self):
        """Close the running Omniverse Toolkit."""
        self._app.shutdown()
        self._framework.unload_all_plugins()

    @property
    def app(self) -> omni.kit.app.IApp:
        """
        omni.kit.app.IApp: omniverse kit application object
        """
        return self._app

    @property
    def framework(self) -> typing.Any:
        """
        omni.kit.app.IApp: omniverse kit application object
        """
        return self._framework
