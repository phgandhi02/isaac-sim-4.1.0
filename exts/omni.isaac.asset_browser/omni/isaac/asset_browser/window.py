# Copyright (c) 2022-2024, NVIDIA CORPORATION. All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto. Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.
#

import os

import carb.settings
import omni.ui as ui
from omni.kit.browser.core import TreeCategoryDelegate
from omni.kit.browser.folder.core import TreeFolderBrowserWidget

from .delegate import AssetDetailDelegate
from .model import AssetBrowserModel


class AssetBrowserWindow(ui.Window):
    """
    Represent a window to show Assets
    """

    WINDOW_TITLE = "Isaac Assets (Beta)"

    def __init__(self, visible=True):
        super().__init__(self.WINDOW_TITLE, visible=visible)

        self.frame.set_build_fn(self._build_ui)

        self._browser_model = None
        self._delegate = None
        self._widget = None

        # Dock it to the same space where Stage is docked.
        self.deferred_dock_in("Content")

    def _build_ui(self):
        preload_folder = os.path.abspath(carb.tokens.get_tokens_interface().resolve("${app}/../predownload"))
        self._browser_model = AssetBrowserModel()
        self._delegate = AssetDetailDelegate(self._browser_model)

        with self.frame:
            with ui.VStack(spacing=15):
                self._widget = TreeFolderBrowserWidget(
                    self._browser_model,
                    detail_delegate=self._delegate,
                    predownload_folder=preload_folder,
                )
