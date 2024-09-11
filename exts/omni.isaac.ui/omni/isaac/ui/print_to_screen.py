# Copyright (c) 2021-2023, NVIDIA CORPORATION. All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto. Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.
#

import numpy as np
import omni.graph.core as og
from omni.isaac.core.utils.prims import delete_prim, is_prim_path_valid
from omni.isaac.core.utils.string import find_unique_string_name


class ScreenPrinter:
    """Print text to the viewport using the omni.graph.visualization.nodes.DrawScreenSpaceText node.

    Args:
        screen_pos_x (float): X position of the text on the screen, given as a percent of screen width with 0 refering to the left hand side. (Default: 78)
        screen_pos_y (float): Y position of the text on the screen, given as a percent of screen width with 0 refering to the top. (Default 95)
        text_size (float): Size of text (Default 14.0)
        max_width (float): Maximum width of text before wrapping around and continuing on a new line.  A value of 0 means there is no wraparound (Default 0)
        color (np.array): Color of text, given in a (4x1) np.array of the form [r,g,b,luminocity].  All four values have a minimum of 0.0 and a maximum of 2.0. (Default [1,1,1,1])
    """

    def __init__(
        self,
        screen_pos_x: float = 78,
        screen_pos_y: float = 95,
        text_size: float = 14.0,
        max_width: int = 0,
        color: np.array = np.ones(4),
    ) -> None:
        self._keys = og.Controller.Keys
        self._controller = og.Controller()

        self._graph_path = find_unique_string_name("/World/PrintActionGraph", lambda x: not is_prim_path_valid(x))

        (self.graph, self.nodes, _, _) = self._controller.edit(
            {"graph_path": self._graph_path, "evaluator_name": "push"},
            {
                self._keys.CREATE_NODES: [
                    ("tick", "omni.graph.action.OnTick"),
                    ("print_to_screen", "omni.graph.visualization.nodes.DrawScreenSpaceText"),
                ],
                self._keys.CONNECT: [("tick.outputs:tick", "print_to_screen.inputs:execIn")],
                self._keys.SET_VALUES: [
                    ("print_to_screen.inputs:position", [screen_pos_x, screen_pos_y]),
                    ("print_to_screen.inputs:text", ""),
                    ("print_to_screen.inputs:size", text_size),
                    ("print_to_screen.inputs:boxWidth", max_width),
                    ("print_to_screen.inputs:textColor", color),
                ],
            },
        )

        self._print_node = self.nodes[1]

    def set_text(self, text: str) -> None:
        """Set the text on the screen

        Args:
            text (str): Text to appear on the screen.
        """
        self._controller.edit(self.graph, {self._keys.SET_VALUES: (("inputs:text", self._print_node), text)})

    def set_text_position(self, screen_pos_x: float, screen_pos_y: float) -> None:
        """Set the x,y position of the text on the screen

        Args:
            screen_pos_x (float): X position of the text on the screen, given as a percent of screen width with 0 refering to the left hand side.
            screen_pos_y (float): Y position of the text on the screen, given as a percent of screen width with 0 refering to the top.
        """
        self._controller.edit(
            self.graph, {self._keys.SET_VALUES: (("inputs:position", self._print_node), [screen_pos_x, screen_pos_y])}
        )

    def set_text_size(self, size: float) -> None:
        """Set the size of the text.

        Args:
            size (float): Pixel height of a line of text
        """
        self._controller.edit(self.graph, {self._keys.SET_VALUES: (("inputs:size", self._print_node), size)})

    def set_text_max_width(self, max_width: int) -> None:
        """Set the maximum text width (in pixels) before wrap-around

        Args:
            max_width (int): Maximum width of text before wrapping around and continuing on a new line.  A value of 0 means there is no wrap-around
        """
        self._controller.edit(self.graph, {self._keys.SET_VALUES: (("inputs:boxWidth", self._print_node), max_width)})

    def set_text_color(self, color4f: np.array) -> None:
        """Set the color of the text

        Args:
            color4f (np.array): Color of text, given in a (4x1) np.array of the form [r,g,b,luminocity].  All four values have a minimum of 0.0 and a maximum of 2.0.
        """
        self._controller.edit(self.graph, {self._keys.SET_VALUES: (("inputs:textColor", self.nodes[1]), color4f)})

    def clear_text(self) -> None:
        """Clear the text from the screen."""
        self.set_text("")

    def exit(self) -> None:
        """Delete OmniGraph used by this ScreenPrinter.  After calling exit(), all subsequent function calls will fail."""
        delete_prim(self._graph_path)
