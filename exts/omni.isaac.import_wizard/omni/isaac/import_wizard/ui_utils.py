# Copyright (c) 2022-2023, NVIDIA CORPORATION. All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto. Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.

from typing import Callable

import omni.ui as ui
from omni.isaac.ui.element_wrappers.base_ui_element_wrappers import UIWidgetWrapper
from omni.kit.window.property.templates import LABEL_HEIGHT, LABEL_WIDTH


def Singleton(class_):
    """A singleton decorator"""
    instances = {}

    def getinstance(*args, **kwargs):
        if class_ not in instances:
            instances[class_] = class_(*args, **kwargs)
        return instances[class_]

    return getinstance


BUTTON_FONT = 16
BUTTON_BACKGROUND_COLOR = 0xFF343434
DISABLED_COLOR = 0xFF777777
HIGHLIGHT_COLOR = 0xFF00B976
HOVER_COLOR = 0x6600B976
TEXT_COLOR = 0xFFC7C7C7


label_kwargs = {
    "style_type_name_override": "Label::label",
    "word_wrap": True,
    "width": LABEL_WIDTH / 2,
    "height": LABEL_HEIGHT,
    "alignment": ui.Alignment.LEFT_TOP,
}
text_kwargs = {
    "style_type_name_override": "Label::label",
    "height": LABEL_HEIGHT,
    "alignment": ui.Alignment.LEFT_TOP,
    "style": {"font_size": 16, "color": TEXT_COLOR},
    "word_wrap": True,
}
scroll_kwargs = {
    "style_type_name_override": "ScrollingFrame",
    "alignment": ui.Alignment.LEFT_TOP,
    "horizontal_scrollbar_policy": ui.ScrollBarPolicy.SCROLLBAR_AS_NEEDED,
    "vertical_scrollbar_policy": ui.ScrollBarPolicy.SCROLLBAR_ALWAYS_ON,
}
vertical_scroll_kwargs = {
    "style_type_name_override": "ScrollingFrame",
    "alignment": ui.Alignment.LEFT_TOP,
    "horizontal_scrollbar_policy": ui.ScrollBarPolicy.SCROLLBAR_ALWAYS_OFF,
    "vertical_scrollbar_policy": ui.ScrollBarPolicy.SCROLLBAR_AS_NEEDED,
}

x_large_btn_kwargs = {
    "style": {
        "font_size": 26,
        "background_color": BUTTON_BACKGROUND_COLOR,
        "color": HIGHLIGHT_COLOR,
        "border_color": HIGHLIGHT_COLOR,
        "border_width": 3,
        "border_radius": 5,
        ":hovered": {"background_color": HOVER_COLOR},
    },
}


large_btn_kwargs = {
    "style": {
        "font_size": 24,
        "background_color": BUTTON_BACKGROUND_COLOR,
        "color": HIGHLIGHT_COLOR,
        "border_color": HIGHLIGHT_COLOR,
        "border_width": 3,
        "border_radius": 5,
        ":hovered": {"background_color": HOVER_COLOR},
        ":disabled": {
            "color": DISABLED_COLOR,
            "border_color": DISABLED_COLOR,
        },
    },
}
medium_btn_kwargs = {
    "style": {
        "font_size": 20,
        "background_color": BUTTON_BACKGROUND_COLOR,
        "color": HIGHLIGHT_COLOR,
        "border_color": HIGHLIGHT_COLOR,
        "border_width": 2,
        "border_radius": 5,
        ":hovered": {"background_color": HOVER_COLOR},
        ":disabled": {
            "color": DISABLED_COLOR,
            "border_color": DISABLED_COLOR,
        },
    },
}
small_btn_kwargs = {
    "style": {
        "font_size": BUTTON_FONT,
        "background_color": BUTTON_BACKGROUND_COLOR,
        "color": HIGHLIGHT_COLOR,
        "border_color": HIGHLIGHT_COLOR,
        "border_width": 1,
        "border_radius": 5,
        "margin": 5,
        ":hovered": {"background_color": HOVER_COLOR},
        ":disabled": {
            "color": DISABLED_COLOR,
            "border_color": DISABLED_COLOR,
        },
    },
}
header_kwargs = {
    "height": 18,
    "style": {
        "font_size": 24,
        "alignment": ui.Alignment.LEFT_TOP,
        "style_type_name_override": "Label::label",
        "word_wrap": True,
        "color": HIGHLIGHT_COLOR,
    },
}
tool_btn_kwargs = {
    "height": 20,
    "width": 260,
    "style": {
        "font_size": 16,
        "color": HIGHLIGHT_COLOR,
        "background_color": BUTTON_BACKGROUND_COLOR,
        "border_color": HIGHLIGHT_COLOR,
        "border_width": 0.5,
        "border_radius": 5,
        ":hovered": {"background_color": HOVER_COLOR},
    },
}


class CheckBox(UIWidgetWrapper):
    """Create a CheckBox UI Element

    Args:
        label (str): Short descriptive text to the left of the CheckBox
        default_value (bool, optional): If True, CheckBox will be checked. Defaults to False.
        tooltip (str, optional): Text to appear when the mouse hovers over the CheckBox.  Defaults to "".
        on_click_fn (_type_, optional): Callback function that will be called when the CheckBox is pressed.
            Function should take a single bool argument.  The return value will not be used.  Defaults to None.
    """

    def __init__(self, label: str, default_value: bool = False, tooltip="", on_click_fn=None):
        self._on_click_fn = on_click_fn

        checkbox_frame = self._create_ui_widget(label, bool(default_value), tooltip)
        super().__init__(checkbox_frame)

    @property
    def label(self) -> ui.Label:
        """
        Returns:
            omni.ui.Label: UI Label element that contains the descriptive text
        """
        return self._label

    @property
    def checkbox(self) -> ui.CheckBox:
        """
        Returns:
            omni.ui.CheckBox: UI CheckBox element
        """
        return self._checkbox

    def get_value(self) -> bool:
        """
        Returns:
            bool: Check box is checked
        """
        return self.checkbox.model.get_value_as_bool()

    def set_value(self, val: bool):
        """
        Args:
            val (bool): If True, set CheckBox to checked state
        """
        self.checkbox.model.set_value(bool(val))

    def set_on_click_fn(self, on_click_fn: Callable):
        """Set the function that will be called when the CheckBox is clicked.

        Args:
            on_click_fn (Callable): Callback function for when CheckBox is clicked.
                The function should take a single bool argument.  The return value will not be used.
        """
        self._on_click_fn = on_click_fn

    def _on_click_fn_wrapper(self, model):
        if self._on_click_fn is not None:
            self._on_click_fn(model.get_value_as_bool())

    def _create_ui_widget(self, label: str, default_value: bool, tooltip: str):
        containing_frame = ui.Frame()
        with containing_frame:
            with ui.HStack():
                model = ui.SimpleBoolModel()
                model.set_value(default_value)
                self._checkbox = ui.CheckBox(model=model, tooltip=tooltip, width=25)
                model.add_value_changed_fn(self._on_click_fn_wrapper)
                self._label = ui.Label(label, alignment=ui.Alignment.LEFT, word_wrap=True)

        return containing_frame
