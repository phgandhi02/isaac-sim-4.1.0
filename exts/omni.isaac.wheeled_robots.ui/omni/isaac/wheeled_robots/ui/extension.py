# Copyright (c) 2018-2024, NVIDIA CORPORATION. All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto. Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.
#


import gc

import omni.ext
import omni.kit.commands
from omni.isaac.ui.menu import make_menu_item_description
from omni.isaac.wheeled_robots.bindings._omni_isaac_wheeled_robots import acquire_interface, release_interface
from omni.kit.menu.utils import MenuItemDescription, add_menu_items, remove_menu_items

from .menu_graphs import DifferentialRobotGraph


class Extension(omni.ext.IExt):
    def on_startup(self, ext_id: str):
        self.__interface = acquire_interface()

        controller_menu = [
            make_menu_item_description(ext_id, "Differential Controller", onclick_fun=self._open_differential_graph),
        ]
        self._menu_controller = [
            MenuItemDescription(
                name="Common Omnigraphs",
                sub_menu=controller_menu,
            )
        ]

        add_menu_items(self._menu_controller, "Isaac Utils")
        self._window = None

    def on_shutdown(self):
        remove_menu_items(self._menu_controller, "Isaac Utils")
        if self._window:
            self._window.visible = False
        release_interface(self.__interface)
        gc.collect()

    def _open_differential_graph(self):
        self._differential_graph = DifferentialRobotGraph()
        self._window = self._differential_graph.create_differential_robot_graph()
