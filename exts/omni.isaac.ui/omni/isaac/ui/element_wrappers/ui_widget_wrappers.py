# Copyright (c) 2023, NVIDIA CORPORATION. All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto. Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.
#
import sys
from cmath import inf
from collections.abc import Iterable
from typing import Callable, List, Optional, Tuple, Union

import carb
import numpy as np
import omni.physx as _physx
import omni.ui as ui
from omni.isaac.core.utils.prims import get_prim_object_type
from omni.isaac.ui.ui_utils import (
    BUTTON_WIDTH,
    LABEL_HEIGHT,
    LABEL_WIDTH,
    add_line_rect_flourish,
    add_separator,
    format_tt,
    get_style,
    on_copy_to_clipboard,
)
from omni.isaac.ui.widgets import DynamicComboBoxModel
from omni.kit.window.filepicker import FilePickerDialog
from omni.kit.window.property.templates import LABEL_HEIGHT, LABEL_WIDTH
from omni.usd import get_context
from pxr import Usd

from .base_ui_element_wrappers import UIWidgetWrapper

##########################################################################################
#                                 UI Frame Wrappers
##########################################################################################


class ScrollingWindow(ui.Window):
    def __init__(self, **kwargs):
        super().__init__(**kwargs)
        original_frame = ui.Window.frame.fget(self)
        with original_frame:
            self._scrolling_frame = ui.ScrollingFrame(vertical_scrollbar_policy=ui.ScrollBarPolicy.SCROLLBAR_ALWAYS_ON)

    @property
    def frame(self) -> ui.ScrollingFrame:
        """
        Returns:
            omni.ui.Frame: A UI Frame
        """
        return self._scrolling_frame


class Frame(UIWidgetWrapper):
    """Create a Frame UI element

    Args:
        enabled (bool, optional): Frame is enabled. Defaults to True.
        visible (bool, optional): Frame is visible. Defaults to True.
        build_fn (Callable, optional): A function that can be called to specify what should fill the Frame.
            Function should take no arguments.  Return values will not be used. Defaults to None.
    """

    def __init__(self, enabled: bool = True, visible: bool = True, build_fn: Callable = None):
        # Create a Frame UI element
        self._frame = self._create_frame(enabled, visible, build_fn)
        super().__init__(self.frame)

    @property
    def frame(self) -> ui.Frame:
        """
        Returns:
            omni.ui.Frame: A UI Frame
        """
        return self._frame

    def rebuild(self):
        """
        Rebuild the Frame using the specified build_fn
        """
        self.frame.rebuild()

    def set_build_fn(self, build_fn: Callable):
        """Set the build_fn to use when rebuilding the frame.

        Args:
            build_fn (Callable): Build function to use when rebuilding the frame.  Function should take
                no arguments.  Return values will not be used.
        """
        self.frame.set_build_fn(build_fn)

    def __enter__(self):
        self.frame.__enter__()

    def __exit__(self, *args):
        self.frame.__exit__(*args)

    def _create_frame(self, enabled: bool, visible: bool, build_fn: Callable) -> ui.CollapsableFrame:
        frame = ui.Frame(
            visible=visible,
            enabled=enabled,
            build_fn=build_fn,
            style=get_style(),
            style_type_name_override="Frame",
            horizontal_scrollbar_policy=ui.ScrollBarPolicy.SCROLLBAR_AS_NEEDED,
            vertical_scrollbar_policy=ui.ScrollBarPolicy.SCROLLBAR_ALWAYS_ON,
        )

        return frame


class CollapsableFrame(Frame):
    """Create a CollapsableFrame UI element

    Args:
        title (str): Title of Collapsable Frame
        collapsed (bool, optional): Frame is collapsed. Defaults to True.
        enabled (bool, optional): Frame is enabled. Defaults to True.
        visible (bool, optional): Frame is visible. Defaults to True.
        build_fn (Callable, optional): A function that can be called to specify what should fill the Frame.
            Function should take no arguments.  Return values will not be used. Defaults to None.
    """

    def __init__(
        self, title: str, collapsed: bool = True, enabled: bool = True, visible: bool = True, build_fn: Callable = None
    ):
        # Create a Frame UI element
        self._frame = self._create_frame(title, collapsed, enabled, visible, build_fn)
        UIWidgetWrapper.__init__(self, self.frame)

    @property
    def collapsed(self) -> bool:
        """
        Returns:
            bool: CollapsableFrame is collapsed
        """
        return self.frame.collapsed

    @collapsed.setter
    def collapsed(self, value: bool):
        self.frame.collapsed = value

    @property
    def title(self) -> str:
        """
        Returns:
            str: Title text of CollapsableFrame
        """
        return self.frame.title

    @title.setter
    def title(self, value: str):
        self.frame.title = value

    def _create_frame(
        self, title: str, collapsed: bool, enabled: bool, visible: bool, build_fn: Callable
    ) -> ui.CollapsableFrame:
        frame = ui.CollapsableFrame(
            title=title,
            name=title,
            height=0,
            collapsed=collapsed,
            visible=visible,
            enabled=enabled,
            build_fn=build_fn,
            style=get_style(),
            style_type_name_override="CollapsableFrame",
            horizontal_scrollbar_policy=ui.ScrollBarPolicy.SCROLLBAR_AS_NEEDED,
            vertical_scrollbar_policy=ui.ScrollBarPolicy.SCROLLBAR_ALWAYS_ON,
        )

        return frame


class ScrollingFrame(Frame):
    """Create a ScrollingFrame UI element with a specified size.

    Args:
        num_lines (int, optional): Determines height of ScrollingFrame element in terms of the
            typical line height of UI elements. If not specified, the ScrollingFrame will fill the space it can in the UI Window.
        enabled (bool, optional): Frame is enabled. Defaults to True.
        visible (bool, optional): Frame is visible. Defaults to True.
        build_fn (Callable, optional): A function that can be called to specify what should fill the Frame.
            Function should take no arguments.  Return values will not be used. Defaults to None.
    """

    def __init__(self, num_lines=None, enabled: bool = True, visible: bool = True, build_fn: Callable = None):
        self._bottom_line_buffer_size = 4

        # Create a Frame UI element
        self._frame = self._create_frame(num_lines, enabled, visible, build_fn)
        UIWidgetWrapper.__init__(self, self.frame)

    def set_num_lines(self, num_lines: int):
        """Set the height of the ScrollingFrame element in terms of the typical line height of
        other UI elements.

        Args:
            num_lines (int): Number of lines that should be shown in a ScrollingFrame.
        """
        self.frame.height = ui.Length(LABEL_HEIGHT * num_lines + self._bottom_line_buffer_size)

    def _create_frame(
        self, num_lines: Optional[int], enabled: bool, visible: bool, build_fn: Callable
    ) -> ui.ScrollingFrame:
        if num_lines is not None:
            height = ui.Length(LABEL_HEIGHT * num_lines + self._bottom_line_buffer_size)
        else:
            height = ui.Fraction(1)
        frame = ui.ScrollingFrame(
            height=height,
            visible=visible,
            enabled=enabled,
            build_fn=build_fn,
            style=get_style(),
            style_type_name_override="ScrollingFrame",
            horizontal_scrollbar_policy=ui.ScrollBarPolicy.SCROLLBAR_AS_NEEDED,
            vertical_scrollbar_policy=ui.ScrollBarPolicy.SCROLLBAR_ALWAYS_ON,
        )

        return frame


##########################################################################################
#                           Simple Editable UI Field Wrappers
##########################################################################################


class IntField(UIWidgetWrapper):
    """
    Creates a IntField UI element.

    Args:
        label (str): Short descriptive text to the left of the IntField.
        tooltip (str, optional): Text to appear when the mouse hovers over the IntField. Defaults to "".
        default_value (int, optional): Default value of the IntField. Defaults to 0.
        lower_limit (int, optional): Lower limit of float. Defaults to None.
        upper_limit (int, optional): Upper limit of float. Defaults to None.
        on_value_changed_fn (Callable, optional): Function to be called when the value of the int is changed.
            The function should take an int as an argument.  The return value will not be used. Defaults to None.
    """

    def __init__(
        self,
        label: str,
        tooltip: str = "",
        default_value: int = 0,
        lower_limit: int = None,
        upper_limit: int = None,
        on_value_changed_fn: Callable = None,
    ):
        self._lower_limit = int(lower_limit) if lower_limit is not None else None
        self._upper_limit = int(upper_limit) if upper_limit is not None else None

        self._default_value = int(default_value)

        self._on_value_changed_fn = on_value_changed_fn

        int_field_frame = self._create_ui_widget(label, tooltip, self._default_value)

        super().__init__(int_field_frame)

    @property
    def label(self) -> ui.Label:
        """
        Returns:
            omni.ui.Label: UI Label element that contains the descriptive text
        """
        return self._label

    @property
    def int_field(self) -> ui.IntField:
        """
        Returns:
            omni.ui.IntField: UI IntField elements
        """
        return self._int_field

    def get_value(self) -> int:
        """Get the current value of the int field

        Returns:
            int: Current value of the int field
        """
        return self.int_field.model.get_value_as_int()

    def get_upper_limit(self) -> int:
        """Get the upper limit on the IntField.

        Returns:
            int: Upper Limit on IntField
        """
        if self._upper_limit is None:
            return sys.maxsize
        return self._upper_limit

    def get_lower_limit(self) -> int:
        """Get the lower limit on the IntField.

        Returns:
            int: Lower Limit on IntField
        """
        if self._lower_limit is None:
            return sys.maxsize * -1
        return self._lower_limit

    def set_value(self, val: int):
        """Set the value in the IntField

        Args:
            val (int): Value to fill IntField
        """
        self.int_field.model.set_value(int(val))

    def set_upper_limit(self, upper_limit: int):
        """Set upper limit of IntField.
        If current value is higher than upper_limit, the current value will be clipped to upper_limit

        Args:
            upper_limit (int): Upper limit of IntField
        """
        upper_limit = int(upper_limit)
        self._upper_limit = upper_limit
        if self.get_value() > upper_limit:
            self.set_value(upper_limit)

    def set_lower_limit(self, lower_limit: int):
        """Set lower limit of IntField.
        If current value is lower than lower_limit, the current value will be clipped to lower_limit

        Args:
            lower_limit (int): lower limit of IntField
        """
        lower_limit = int(lower_limit)
        self._lower_limit = lower_limit
        if self.get_value() < lower_limit:
            self.set_value(lower_limit)

    def set_on_value_changed_fn(self, on_value_changed_fn: Callable):
        """Set function that is called when the value of the IntField is modified

        Args:
            on_value_changed_fn (Callable): Function that is called when the value of the IntField is modified.
                Function should take a int as the argument. The return value will not be used.
        """
        self._on_value_changed_fn = on_value_changed_fn

    def _on_value_changed_fn_wrapper(self, model):
        # Enforces upper and lower limits on value change
        model.set_max(self.get_upper_limit())
        model.set_min(self.get_lower_limit())

        val = model.get_value_as_int()
        if val < self.get_lower_limit():
            model.set_value(self.get_lower_limit())
            return
        if val > self.get_upper_limit():
            model.set_value(self.get_upper_limit())
            return

        if self._on_value_changed_fn is not None:
            self._on_value_changed_fn(val)

    def _create_ui_widget(self, label, tooltip, default_value):
        containing_frame = Frame().frame
        with containing_frame:
            with ui.HStack():
                self._label = ui.Label(
                    label, width=LABEL_WIDTH, alignment=ui.Alignment.LEFT_CENTER, tooltip=format_tt(tooltip)
                )
                self._int_field = ui.IntDrag(
                    name="Field",
                    height=LABEL_HEIGHT,
                    alignment=ui.Alignment.LEFT_CENTER,
                    min=sys.maxsize * -1,
                    max=sys.maxsize,
                )
                self.int_field.model.set_value(default_value)
                add_line_rect_flourish(False)
            self.int_field.model.add_value_changed_fn(self._on_value_changed_fn_wrapper)
        return containing_frame


class FloatField(UIWidgetWrapper):
    """
    Creates a FloatField UI element.

    Args:
        label (str): Short descriptive text to the left of the FloatField.
        tooltip (str, optional): Text to appear when the mouse hovers over the FloatField. Defaults to "".
        default_value (float, optional): Default value of the Float Field. Defaults to 0.0.
        step (float, optional): Smallest increment that the user can change the float by when dragging mouse. Defaults to 0.01.
        format (str, optional): Formatting string for float. Defaults to "%.2f".
        lower_limit (float, optional): Lower limit of float. Defaults to None.
        upper_limit (float, optional): Upper limit of float. Defaults to None.
        on_value_changed_fn (Callable, optional): Function to be called when the value of the float is changed.
            The function should take a float as an argument.  The return value will not be used. Defaults to None.
    """

    def __init__(
        self,
        label: str,
        tooltip: str = "",
        default_value: float = 0.0,
        step: float = 0.01,
        format: str = "%.2f",
        lower_limit: float = None,
        upper_limit: float = None,
        on_value_changed_fn: Callable = None,
    ):
        self._lower_limit = float(lower_limit) if lower_limit is not None else None
        self._upper_limit = float(upper_limit) if upper_limit is not None else None

        self._default_value = float(default_value)

        self._on_value_changed_fn = on_value_changed_fn

        float_field_frame = self._create_ui_widget(label, tooltip, self._default_value, step, format)

        super().__init__(float_field_frame)

    @property
    def label(self) -> ui.Label:
        """
        Returns:
            omni.ui.Label: UI Label element that contains the descriptive text
        """
        return self._label

    @property
    def float_field(self) -> ui.FloatField:
        """
        Returns:
            omni.ui.FloatField: UI FloatField element
        """
        return self._float_field

    def get_value(self) -> float:
        """Return the current value of the FloatField

        Returns:
            float: Current value of the FloatField
        """
        return self.float_field.model.get_value_as_float()

    def get_upper_limit(self) -> float:
        """Get the upper limit on the FloatField

        Returns:
            float: Upper limit on FloatField
        """
        if self._upper_limit is None:
            return inf
        return self._upper_limit

    def get_lower_limit(self) -> float:
        """Get the lower limit on the FloatField

        Returns:
            float: Lower limit on FloatField
        """
        if self._lower_limit is None:
            return -inf
        return self._lower_limit

    def set_value(self, val: float):
        """Set the value in the FloatField

        Args:
            val (float): Value to fill FloatField
        """
        self.float_field.model.set_value(float(val))

    def set_upper_limit(self, upper_limit: float):
        """Set upper limit of FloatField.
        If current value is higher than upper_limit, the current value will be clipped to upper_limit

        Args:
            upper_limit (float): Upper limit of FloatField
        """
        upper_limit = float(upper_limit)
        self._upper_limit = upper_limit
        if self.get_value() > upper_limit:
            self.set_value(upper_limit)

    def set_lower_limit(self, lower_limit: float):
        """Set lower limit of FloatField.
        If current value is lower than lower_limit, the current value will be clipped to lower_limit

        Args:
            lower_limit (float): lower limit of FloatField
        """
        lower_limit = float(lower_limit)
        self._lower_limit = lower_limit
        if self.get_value() < lower_limit:
            self.set_value(lower_limit)

    def set_on_value_changed_fn(self, on_value_changed_fn: Callable):
        """Set function that is called when the value of the FloatField is modified

        Args:
            on_value_changed_fn (Callable): Function that is called when the value of the FloatField is modified.
                Function should take a float as the argument. The return value will not be used.
        """
        self._on_value_changed_fn = on_value_changed_fn

    def _on_value_changed_fn_wrapper(self, model):
        # Enforces upper and lower limits on value change
        model.set_max(self.get_upper_limit())
        model.set_min(self.get_lower_limit())
        val = model.get_value_as_float()
        if self.get_upper_limit() < val:
            val = self._upper_limit
            model.set_value(float(val + 1))
            return
        elif self.get_lower_limit() > val:
            val = self._lower_limit
            model.set_value(float(val - 1))
            return

        if self._on_value_changed_fn is not None:
            self._on_value_changed_fn(val)

    def _create_ui_widget(self, label, tooltip, default_value, step, format):
        containing_frame = Frame().frame
        with containing_frame:
            with ui.HStack():
                self._label = ui.Label(
                    label, width=LABEL_WIDTH, alignment=ui.Alignment.LEFT_CENTER, tooltip=format_tt(tooltip)
                )
                self._float_field = ui.FloatDrag(
                    name="FloatField",
                    width=ui.Fraction(1),
                    height=0,
                    alignment=ui.Alignment.LEFT_CENTER,
                    min=-inf,
                    max=inf,
                    step=step,
                    format=format,
                )
                self.float_field.model.set_value(default_value)
                add_line_rect_flourish(False)

            self.float_field.model.add_value_changed_fn(self._on_value_changed_fn_wrapper)
        return containing_frame


class StringField(UIWidgetWrapper):
    """Create StringField UI Element.

    Starting at use_folder_picker, the arguments to the StringField all pertain to the folder_picker.
    If the folder_picker is not used, these arguments may all be ignored.


    Args:
        label (str, optional): Label to the left of the UI element. Defaults to "".
        tooltip (str, optional): Tooltip to display over the UI elements. Defaults to "".
        default_val (str, optional): Text to initialize in Stringfield. Defaults to " ".
        read_only (bool, optional): Prevents editing. Defaults to False.
        multiline_okay (bool, optional): If True, allow newline character in input strings. Defaults to False.
        on_value_changed_fn (Callable, optional) Function called when value of StringField is changed.
            The function should take a string as an argument.  The return value will not be used. Defaults to None.
        use_folder_picker (bool, optional): Add a folder picker button to the right. Defaults to False.
        item_filter_fn (Callable, optional): Filter function to pass to the FilePicker.  This function should take a string
            as an argument and return a boolean.  When the user opens the file picker, every file in the directory they are
            viewing will be passed to item_filter_fn, and when True is returned, the file will be shown.  When False is
            returned, the file will not be shown.  This can be used to ensure that the user may only select valid file types.
        bookmark_label (str, optional): Bookmark label to pass to the FilePicker.  This will create a bookmark when the
            file picker is used with the label specified here.
        bookmark_path (str, optional): Bookmark path to pass to the FilePicker.  This will create a bookmark when the file
            picker is used with the path specified here.
    """

    def __init__(
        self,
        label: str,
        tooltip: str = "",
        default_value: str = "",
        read_only=False,
        multiline_okay=False,
        on_value_changed_fn: Callable = None,
        use_folder_picker=False,
        item_filter_fn=None,
        bookmark_label=None,
        bookmark_path=None,
        folder_dialog_title="Select Output Folder",
        folder_button_title="Select Folder",
    ):
        self._default_value = default_value

        self._on_value_changed_fn = on_value_changed_fn

        self._item_filter_fn = item_filter_fn

        self._file_picker_frame = None
        self._file_picker_btn = None

        string_field_frame = self._create_ui_widget(
            label,
            default_value,
            tooltip,
            use_folder_picker,
            read_only,
            multiline_okay,
            bookmark_label,
            bookmark_path,
            folder_dialog_title,
            folder_button_title,
        )

        super().__init__(string_field_frame)

    @property
    def label(self) -> ui.Label:
        """
        Returns:
            omni.ui.Label: UI Label element that contains the descriptive text
        """
        return self._label

    @property
    def string_field(self) -> ui.StringField:
        """
        Returns:
            omni.ui.StringField: UI StringField element
        """
        return self._string_field

    @property
    def file_picker_frame(self) -> ui.Frame:
        """
        Returns:
            omni.ui.Frame: UI Frame containing FilePicker
        """
        return self._file_picker_frame

    @property
    def file_picker_btn(self) -> ui.Button:
        """
        Returns:
            omni.ui.Button: Button to activate file picker
        """
        return self._file_picker_btn

    def get_value(self) -> str:
        """Return the current value of the StringField

        Returns:
            str: Current value of the StringField
        """
        return self.string_field.model.get_value_as_string()

    def set_value(self, val: str):
        """Set the value of the StringField

        Args:
            val (str): Value to fill StringField
        """
        self.string_field.model.set_value(val)

    def set_on_value_changed_fn(self, on_value_changed_fn: Callable):
        """Set function that is called when the value of the StringField is modified

        Args:
            on_value_changed_fn (Callable): Function that is called when the value of the StringField is modified.
                Function should take a string as the argument. The return value will not be used.
        """
        self._on_value_changed_fn = on_value_changed_fn

    def set_item_filter_fn(self, item_filter_fn: Callable):
        """Set the filter function that will be used with the file picker

        Args:
            item_filter_fn (Callable): Filter function that will be called to filter the files shown in the
                picker.  This function should take a string file_path as the argument. The return value
                should be a bool, with True indicating the the file should be shown to the user in the file picker.
        """
        self._item_filter_fn = item_filter_fn

    def set_read_only(self, read_only: bool):
        """Set this StringField to be read only

        Args:
            read_only (bool): If True, StringField cannot be modified through the UI; it can still be
                modified programmatically with set_value()
        """
        self.string_field.read_only = read_only

    def set_multiline_okay(self, multiline_okay: bool):
        """Set the StringFiled to allow the newline character

        Args:
            multiline_okay (bool): If True, allow newline character in strings.
        """
        self.string_field.multiline = multiline_okay

    def _item_filter_fn_wrapper(self, file):
        if self._item_filter_fn is not None:
            return self._item_filter_fn(file.path)

    def _on_value_changed_fn_wrapper(self, model):
        val = model.get_value_as_string()
        if self._on_value_changed_fn is not None:
            self._on_value_changed_fn(val)

    def _create_ui_widget(
        self,
        label="",
        default_val=" ",
        tooltip="",
        use_folder_picker=False,
        read_only=False,
        multiline_okay=True,
        bookmark_label=None,
        bookmark_path=None,
        folder_dialog_title="Select Output Folder",
        folder_button_title="Select Folder",
    ):
        containing_frame = Frame().frame
        with containing_frame:
            with ui.HStack():
                self._label = ui.Label(
                    label, width=LABEL_WIDTH, alignment=ui.Alignment.LEFT_CENTER, tooltip=format_tt(tooltip)
                )
                self._string_field = ui.StringField(
                    name="StringField",
                    width=ui.Fraction(1),
                    height=0,
                    alignment=ui.Alignment.LEFT_CENTER,
                    read_only=read_only,
                    multiline=multiline_okay,
                )
                self.string_field.model.set_value(default_val)

                if use_folder_picker:

                    def update_field(filename, path):
                        if filename == "":
                            val = path
                        elif filename[0] != "/" and path[-1] != "/":
                            val = path + "/" + filename
                        elif filename[0] == "/" and path[-1] == "/":
                            val = path + filename[1:]
                        else:
                            val = path + filename
                        self.string_field.model.set_value(val)

                    self.add_folder_picker_icon(
                        update_field,
                        self._item_filter_fn_wrapper,
                        bookmark_label,
                        bookmark_path,
                        dialog_title=folder_dialog_title,
                        button_title=folder_button_title,
                    )
                else:
                    add_line_rect_flourish(False)

                self.string_field.model.add_value_changed_fn(self._on_value_changed_fn_wrapper)
        return containing_frame

    def add_folder_picker_icon(
        self,
        on_click_fn,
        item_filter_fn=None,
        bookmark_label=None,
        bookmark_path=None,
        dialog_title="Select Output Folder",
        button_title="Select Folder",
    ):
        def open_file_picker():
            def on_selected(filename, path):
                on_click_fn(filename, path)
                file_picker.hide()

            def on_canceled(a, b):
                file_picker.hide()

            file_picker = FilePickerDialog(
                dialog_title,
                allow_multi_selection=False,
                apply_button_label=button_title,
                click_apply_handler=lambda a, b: on_selected(a, b),
                click_cancel_handler=lambda a, b: on_canceled(a, b),
                item_filter_fn=item_filter_fn,
                enable_versioning_pane=True,
            )
            if bookmark_label and bookmark_path:
                file_picker.toggle_bookmark_from_path(bookmark_label, bookmark_path, True)

        self._file_picker_frame = ui.Frame(width=0, tooltip=button_title)
        with self.file_picker_frame:
            self._file_picker_btn = ui.Button(
                name="IconButton",
                width=24,
                height=24,
                clicked_fn=open_file_picker,
                style=get_style()["IconButton.Image::FolderPicker"],
                alignment=ui.Alignment.RIGHT_TOP,
            )


##########################################################################################
#                               UI Button Wrappers
##########################################################################################


class Button(UIWidgetWrapper):
    """Create a Button UI Element

    Args:
        label (str): Short descriptive text to the left of the Button
        text (str): Text on the Button
        tooltip (str, optional): Text to appear when the mouse hovers over the Button. Defaults to "".
        on_click_fn (Callable, optional): Callback function that will be called when the button is pressed.
            Function should take no arguments.  The return value will not be used.  Defaults to None.
    """

    def __init__(self, label: str, text: str, tooltip="", on_click_fn=None):
        self._on_click_fn = on_click_fn

        button_frame = self._create_ui_widget(label, text, tooltip)
        super().__init__(button_frame)

    @property
    def label(self) -> ui.Label:
        """
        Returns:
            omni.ui.Label: UI Label element that contains the descriptive text
        """
        return self._label

    @property
    def button(self) -> ui.Button:
        """
        Returns:
            omni.ui.Button: UI Button element
        """
        return self._button

    def set_on_click_fn(self, on_click_fn: Callable):
        """Set the callback function for when the Button is clicked.

        Args:
            on_click_fn (Callable): Callback function for when Button is clicked.
                The function should take a single bool argument.  The return value will not be used.
        """
        self._on_click_fn = on_click_fn

    def trigger_click(self):
        """Trigger identical behavior as if the user pressed the Button through the UI."""
        self._on_clicked_fn_wrapper()

    def _on_clicked_fn_wrapper(self):
        if self._on_click_fn is not None:
            self._on_click_fn()

    def _create_ui_widget(self, label: str, text: str, tooltip: str):
        containing_frame = Frame().frame
        with containing_frame:
            with ui.HStack():
                self._label = ui.Label(
                    label, width=LABEL_WIDTH, alignment=ui.Alignment.LEFT_CENTER, tooltip=format_tt(tooltip)
                )
                self._button = ui.Button(
                    text.upper(),
                    name="Button",
                    width=BUTTON_WIDTH,
                    clicked_fn=self._on_clicked_fn_wrapper,
                    style=get_style(),
                    alignment=ui.Alignment.LEFT_CENTER,
                )
                ui.Spacer(width=5)
                add_line_rect_flourish(True)

        return containing_frame


class StateButton(UIWidgetWrapper):
    """
    Creates a State Button UI element.
    A StateButton is a button that changes between two states A and B when clicked.
    In state A, the StateButton has a_text written on it, and
    in state B, the StateButton has b_text written on it.

    Args:
        label (str): Short descriptive text to the left of the StateButton
        a_text (str): Text on the StateButton in one of its two states
        b_text (str): Text on the StateButton in the other of its two states
        tooltip (str, optional): Text that appears when the mouse hovers over the button. Defaults to "".
        on_a_click_fn (Callable, optional): A function that should be called when the button is clicked while in
            state A. Function should have 0 arguments.  The return value will not be used.  Defaults to None.
        on_b_click_fn (Callable, optional): A function that should be called when the button is clicked while in
            state B. Function should have 0 arguments.  The return value will not be used.  Defaults to None.
        physics_callback_fn (Callable, optional): A function that will be called on every physics step while the
            button is in state B (a_text was pressed). The function should have one argument for physics step size (float).
            The return value will not be used. Defaults to None.
    """

    def __init__(
        self,
        label: str,
        a_text: str,
        b_text: str,
        tooltip="",
        on_a_click_fn: Callable = None,
        on_b_click_fn: Callable = None,
        physics_callback_fn: Callable = None,
    ):
        self.a_text = a_text.upper()
        self.b_text = b_text.upper()

        self._on_a_click_fn = on_a_click_fn
        self._on_b_click_fn = on_b_click_fn

        self._physics_callback_fn = physics_callback_fn
        self._physx_subscription = None
        self._physxIFace = _physx.acquire_physx_interface()

        state_btn_frame = self._creat_ui_widget(label, a_text, b_text, tooltip)

        super().__init__(state_btn_frame)

    @property
    def label(self) -> ui.Label:
        """
        Returns:
            omni.ui.Label: UI Label element that contains the descriptive text
        """
        return self._label

    @property
    def state_button(self) -> ui.Button:
        """
        Returns:
            omni.ui.Button: UI Button element
        """
        return self._state_button

    def set_physics_callback_fn(self, physics_callback_fn: Callable):
        """Set a physics callback function that will be called on every physics step while the StateButton is
        in state B.

        Args:
            physics_callback_fn (Callable): A function that will be called on every physics step while the
                button is in state B (a_text was pressed). The function should have one argument for physics step size (float).
                The return value will not be used.
        """
        self._physics_callback_fn = physics_callback_fn

    def set_on_a_click_fn(self, on_a_click_fn: Callable):
        """Set a function that is called when the button is clicked in state A.

        Args:
            on_a_click_fn (Callable): A function that is called when the button is clicked in state A.
                Function should take no arguments.  The return value will not be used.
        """
        self._on_a_click_fn = on_a_click_fn

    def set_on_b_click_fn(self, on_b_click_fn: Callable):
        """Set a function that is called when the button is clicked in state B.

        Args:
            on_b_click_fn (Callable): A function that is called when the button is clicked in state B.
                Function should take no arguments.  The return value will not be used.
        """
        self._on_b_click_fn = on_b_click_fn

    def is_in_a_state(self) -> bool:
        """Return True if the StateButton is in the a state.  False implies that it is in the b state.

        Returns:
            bool: True when the StateButton is in the b state.
        """
        if self.state_button.text == self.a_text:
            return True

    def trigger_click_if_a_state(self):
        """
        If in the A state, trigger button to execute the same behavior as if it were clicked by the
        user.  If the button is in the B state, nothing will happen.
        """
        if self.is_in_a_state():
            self.state_button.text = self.b_text
            self._on_clicked_fn_wrapper(True)

    def trigger_click_if_b_state(self):
        """
        If in the B state, trigger button to execute the same behavior as if it were clicked by the
        user.  If the button is in the A state, nothing will happen.  This is distinct from calling
        reset() because the user on_b_click_fn() will be triggered.
        """
        if not self.is_in_a_state():
            self.state_button.text = self.a_text
            self._on_clicked_fn_wrapper(False)

    def get_current_text(self) -> str:
        """Get the current text on the button."""
        return self.state_button.text

    def reset(self):
        """Reset StateButton to state A."""
        self.state_button.text = self.a_text
        self._remove_physics_callback()

    def cleanup(self):
        """Remove physics callback created by the StateButton if exists."""
        self._remove_physics_callback()

    def _create_physics_callback(self):
        self._physx_subscription = self._physxIFace.subscribe_physics_step_events(self._physics_callback_fn)

    def _remove_physics_callback(self):
        self._physx_subscription = None

    def _on_clicked_fn_wrapper(self, value):
        # Button pressed while saying a_text
        if value:
            if self._on_a_click_fn is not None:
                self._on_a_click_fn()
            if self._physics_callback_fn is not None:
                self._create_physics_callback()

        # Button pressed while saying b_text
        else:
            if self._on_b_click_fn is not None:
                self._on_b_click_fn()
            if self._physics_callback_fn is not None:
                self._remove_physics_callback()

    def _creat_ui_widget(self, label: str, a_text: str, b_text: str, tooltip: str):
        def toggle():
            if self.state_button.text == a_text.upper():
                self.state_button.text = b_text.upper()
                self._on_clicked_fn_wrapper(True)
            else:
                self.state_button.text = a_text.upper()
                self._on_clicked_fn_wrapper(False)

        containing_frame = Frame().frame
        with containing_frame:
            with ui.HStack():
                self._label = ui.Label(
                    label, width=LABEL_WIDTH, alignment=ui.Alignment.LEFT_CENTER, tooltip=format_tt(tooltip)
                )
                self._state_button = ui.Button(
                    a_text.upper(),
                    name="Button",
                    width=BUTTON_WIDTH,
                    clicked_fn=toggle,
                    style=get_style(),
                    alignment=ui.Alignment.LEFT_CENTER,
                )
                ui.Spacer(width=5)
                ui.Spacer(width=ui.Fraction(1))
                ui.Spacer(width=10)
                with ui.Frame(width=0):
                    with ui.VStack():
                        with ui.Placer(offset_x=0, offset_y=7):
                            ui.Rectangle(height=5, width=5, alignment=ui.Alignment.CENTER)
                ui.Spacer(width=5)
        return containing_frame


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
        containing_frame = Frame().frame
        with containing_frame:
            with ui.HStack():
                self._label = ui.Label(
                    label, width=LABEL_WIDTH, alignment=ui.Alignment.LEFT_CENTER, tooltip=format_tt(tooltip)
                )
                model = ui.SimpleBoolModel()
                model.set_value(default_value)
                self._checkbox = ui.CheckBox(model=model, tooltip=tooltip)
                model.add_value_changed_fn(self._on_click_fn_wrapper)

                add_line_rect_flourish()
        return containing_frame


##########################################################################################
#                          UI Selection Widget Wrappers
##########################################################################################


class DropDown(UIWidgetWrapper):
    """
    Create a DropDown UI element.
    A DropDown menu can be populated by the user, with a callback function specified
    for when an item is selected.

    Args:
        label (str): Short descriptive text to the left of the DropDown
        tooltip (str, optional): Text to appear when the mouse hovers over the DropDown. Defaults to "".
        populate_fn (Callable, optional): A user-defined function that returns a list[str] of items
            that should populate the drop-down menu.  This Function should have 0 arguments. Defaults to None.
        on_selection_fn (Callable, optional): A user-defined callback function for when an element is selected
            from the DropDown.  The function should take in a string argument of the selection.
            The return value will not be used.  Defaults to None.
        keep_old_selections (bool, optional): When the DropDown is repopulated with the user-defined populate_fn,
            the default behavior is to reset the selection in the DropDown to be at index 0.  If the user
            sets keep_old_selections=True, when the DropDown is repopulated and the old selection is still one of
            the options, the new selection will match the old selection.  Defaults to False.
    """

    def __init__(
        self,
        label: str,
        tooltip: str = "",
        populate_fn: Callable = None,
        on_selection_fn: Callable = None,
        keep_old_selections: bool = False,
    ):
        self._populate_fn = populate_fn
        self._on_selection_fn = on_selection_fn
        self._keep_old_selection = keep_old_selections
        self._items = []

        combobox_frame = self._create_ui_widget(label, tooltip)
        super().__init__(combobox_frame)

    @property
    def label(self) -> ui.Label:
        """
        Returns:
            omni.ui.Label: UI Label element that contains the descriptive text
        """
        return self._label

    @property
    def combobox(self) -> ui.ComboBox:
        """
        Returns:
            omni.ui.ComboBox: UI ComboBox element.
        """
        return self._combobox

    def repopulate(self):
        """A function that the user can call to make the DropDown menu repopulate.
        This will call the populate_fn set by the user.
        """
        if self._populate_fn is None:
            carb.log_warn("Unable to repopulate drop-down meny without a populate_fn being specified")
            return
        else:
            new_items = self._populate_fn()

            old_selection = self.get_selection()
            self.set_items(new_items)
            new_selection = self.get_selection()

            if self._on_selection_fn is not None and new_selection != old_selection:
                # Call the user on_selection_fn if the selection has changed as a result of repopulate()
                self._on_selection_fn(new_selection)

    def set_populate_fn(self, populate_fn: Callable, repopulate: bool = True):
        """Set the populate_fn for this DropDown

        Args:
            populate_fn (Callable): Function used to specify the options that fill the DropDown.
                Function should take no arguments and return a list[str].
            repopulate (bool, optional): If true, repopulate the DropDown using the new populate_fn. Defaults to True.
        """
        self._populate_fn = populate_fn
        if repopulate:
            self.repopulate()

    def get_items(self) -> List[str]:
        """Get the items in the DropDown

        Returns:
            List[str]: A list of the options in the DropDown
        """
        return self._items

    def get_selection_index(self) -> int:
        """Get index of selection in DropDown menu

        Returns:
            int: Index of selection in DropDown menu
        """
        return self.combobox.model.get_item_value_model().as_int

    def get_selection(self) -> str:
        """Get current selection in DropDown

        Returns:
            str: Current selection in DropDown
        """
        if len(self._items) == 0:
            return None
        return self._items[self.get_selection_index()]

    def set_items(self, items: List[str], select_index: int = None):
        """Set the items in the DropDown explicitly.

        Args:
            items (List[str]): New set of items in the DropDown
            select_index (int, optional): Index of item to select.  If left as None, behavior is determined by the
                keep_old_selections flag.  Defaults to None.
        """
        if self._keep_old_selection and select_index is None:
            selection = self.get_selection()
            if selection is not None and selection in items:
                select_index = items.index(selection)

        self._items = items
        self.combobox.model = DynamicComboBoxModel(items)

        if select_index is not None and select_index < len(items):
            self.combobox.model.get_item_value_model().set_value(select_index)

        self.combobox.model.add_item_changed_fn(self._item_changed_fn_wrapper)

    def set_selection(self, selection: str):
        """Set the selected item in the DropDown.
        If the specifified selection is not in the DropDown, nothing will happen.

        Args:
            selection (str): Item to select in the DropDown
        """
        if selection in self.get_items():
            select_index = self.get_items().index(selection)
            self.set_selection_by_index(select_index)
        else:
            carb.log_warn(f"Item {selection} is not present in DropDown, and cannot be set as the selected item.")

    def set_selection_by_index(self, select_index: int):
        """Set the selected item in the DropDown by index.
        If the provided index is out of bounds, nothing will happen.

        Args:
            select_index (int): Index of item to select from DropDown
        """
        if select_index < len(self.get_items()):
            self.combobox.model.get_item_value_model().set_value(select_index)
        else:
            carb.log_warn(
                f"Index {select_index} is out of bounds. The DropDown currently has {len(self.get_items())} items in it."
            )

    def set_on_selection_fn(self, on_selection_fn: Callable):
        """Set the function that gets called when a new item is selected from the DropDown

        Args:
            on_selection_fn (Callable): A function that is called when a new item is selected from the DropDown.
                he function should take in a string argument of the selection.  Its return value is not used.
        """
        self._on_selection_fn = on_selection_fn

    def set_keep_old_selection(self, val: bool):
        """Set keep_old_selection flag to determine behavior when repopulating the DropDown

        Args:
            val (bool): When the DropDown is repopulated with the user-defined populate_fn,
                the default behavior is to reset the selection in the DropDown to be at index 0.  If the user
                sets keep_old_selections=True, when the DropDown is repopulated and the old selection is still one of
                the options, the new selection will match the old selection, and the on_selection_fn() will not be called.
        """
        self._keep_old_selection = val

    def set_populate_fn_to_find_all_usd_objects_of_type(self, object_type: str, repopulate=True):
        """
        Set the populate_fn to find all objects of a specified type on the USD stage.  This is
        included as a convenience function to fulfill one common use-case for a DropDown menu.
        This overrides the populate_fn set by the user.

        Args:
            object_type (str): A string name of the type of USD object matching the output of
                omni.isaac.core.utils.prims.get_prim_object_type(prim_path)
            repopulate (bool, optional): Repopulate the DropDown immediately. Defaults to True.
        """
        self.set_populate_fn(lambda: self._find_all_usd_objects_of_type(object_type), repopulate=repopulate)

    def trigger_on_selection_fn_with_current_selection(self):
        """Call the user on_selection_fn() with whatever item is currently selected in the DropDown."""
        if self._on_selection_fn is not None:
            self._on_selection_fn(self.get_selection())

    def _item_changed_fn_wrapper(self, model, val):
        if self._on_selection_fn is not None:
            selected_item = self._items[model.get_item_value_model().as_int]
            self._on_selection_fn(selected_item)

    def _create_ui_widget(self, label, tooltip):
        items = []
        combobox_model = DynamicComboBoxModel(items)
        containing_frame = Frame().frame
        with containing_frame:
            with ui.HStack():
                self._label = ui.Label(label, width=LABEL_WIDTH, alignment=ui.Alignment.LEFT_CENTER, tooltip=tooltip)
                self._combobox = ui.ComboBox(combobox_model)
                add_line_rect_flourish(False)

        return containing_frame

    def _find_all_usd_objects_of_type(self, obj_type: str):
        items = []
        stage = get_context().get_stage()
        if stage:
            for prim in Usd.PrimRange(stage.GetPrimAtPath("/")):
                path = str(prim.GetPath())
                # Get prim type get_prim_object_type
                type = get_prim_object_type(path)
                if type == obj_type:
                    items.append(path)

        return items


class ColorPicker(UIWidgetWrapper):
    """Create a ColorPicker UI element to allow user-selection of an RGBA color

    Args:
        label (str): Short descriptive text to the left of the ColorPicker
        default_value (List[float], optional): RGBA color values between 0 and 1. Defaults to [1.0, 1.0, 1.0, 1.0].
        tooltip (str, optional): Text to appear when the mouse hovers over the ColorPicker. Defaults to "".
        on_color_picked_fn (Callable, optional): Function that will be called if the user picks a new color.
            Function should expect a List[float] as an argument with four RGBA color values between 0 and 1.
            The return value will not be used.
    """

    def __init__(
        self,
        label: str,
        default_value: List[float] = [1.0, 1.0, 1.0, 1.0],
        tooltip: str = "",
        on_color_picked_fn: Callable = None,
    ):
        self._on_color_picked_fn = on_color_picked_fn
        default_value = list(default_value)

        color_picker_frame = self._create_ui_widget(label, default_value, tooltip)
        super().__init__(color_picker_frame)

    @property
    def label(self) -> ui.Label:
        """
        Returns:
            omni.ui.Label: UI Label element that contains the descriptive text
        """
        return self._label

    @property
    def color_picker(self) -> ui.ColorWidget:
        """
        Returns:
            omni.ui.ColorWidget: UI ColorWidget element
        """
        return self._color_picker

    def get_color(self) -> List[float]:
        """Get the RGBA value of the selected color

        Returns:
            List[float]: RGBA color value with four values between 0 and 1
        """
        color = []
        for item in self.color_picker.model.get_item_children():
            val = self.color_picker.model.get_item_value_model(item).get_value_as_float()
            color.append(val)
        return color

    def set_color(self, color: List[float]):
        """Set the RGBA color value of the selected color

        Args:
            color (List[float]): RGBA color value with four values between 0 and 1
        """
        color = list(color)
        for i, item in enumerate(self.color_picker.model.get_item_children()):
            val = self.color_picker.model.get_item_value_model(item).set_value(color[i])
            color.append(val)

        self._on_color_picked_fn_wrapper()

    def set_on_color_picked_fn(self, on_color_picked_fn: Callable):
        """Set the function that should be called if the user picks a new color

        Args:
            on_color_picked_fn (Callable): Function that will be called if the user picks a new color.
                Function should expect a List[float] as an argument with four RGBA color values between 0 and 1.
                The return value will not be used.
        """
        self._on_color_picked_fn = on_color_picked_fn

    def _on_color_picked_fn_wrapper(self, *worthless_args):
        if self._on_color_picked_fn is not None:
            self._on_color_picked_fn(self.get_color())

    def _create_ui_widget(self, label: str, default_value: List[float], tooltip: str):
        containing_frame = Frame().frame
        with containing_frame:
            with ui.HStack():
                self._label = ui.Label(
                    label, width=LABEL_WIDTH, alignment=ui.Alignment.LEFT_CENTER, tooltip=format_tt(tooltip)
                )
                self._color_picker = ui.ColorWidget(*default_value, width=BUTTON_WIDTH)
                self.color_picker.model.add_end_edit_fn(self._on_color_picked_fn_wrapper)
                ui.Spacer(width=5)
                add_line_rect_flourish()
        return containing_frame


##########################################################################################
#                      UI Information Communication Wrappers
##########################################################################################


class TextBlock(UIWidgetWrapper):
    """Create a text block that is only modifiable through code. The user may not set
    the value of the text in the UI.

    Args:
        label (str): Short description of the contents of the TextBlock
        text (str, optional): Text to put in the TextBlock. Defaults to "".
        tooltip (str, optional): Text to appear when the mouse hovers over the TextBlock. Defaults to "".
        num_lines (int, optional): Number of lines that should be visible in the TextBlock at one time. Defaults to 5.
        include_copy_button (bool, optional): Include a copy_to_clipboard button. Defaults to True.
    """

    def __init__(self, label: str, text: str = "", tooltip: str = "", num_lines=5, include_copy_button: bool = True):
        self._copy_btn = None

        text_block_frame = self._create_ui_widget(num_lines, label, text, tooltip, include_copy_button)
        super().__init__(text_block_frame)

    @property
    def label(self) -> ui.Label:
        """
        Returns:
            omni.ui.Label: UI Label element that contains the descriptive text
        """
        return self._label

    @property
    def scrolling_frame(self) -> ui.ScrollingFrame:
        """
        Returns:
            omni.ui.ScrollingFrame: Scrolling Frame that contains the TextBlock text
        """
        return self._scrolling_frame

    @property
    def copy_btn(self) -> ui.Button:
        """
        Returns:
            omni.ui.Button: Copy Button.  If the TextBlock was built without a copy button, this will return None.
        """
        return self._copy_btn

    @property
    def text_block(self) -> ui.Label:
        """
        Returns:
            omni.ui.Label: UI element that contains the text in the text block
        """
        return self._text_block

    def get_text(self) -> str:
        """
        Returns:
            str: Text in the text block
        """
        return self.text_block.text

    def set_text(self, text: str):
        """
        Args:
            text (str): Set the text in the text block.
        """
        self.text_block.text = text

    def set_num_lines(self, num_lines: int):
        """Set the number of lines that should be visible in the TextBlock at one time.

        Args:
            num_lines (int): Number of lines that should be visible in the TextBlock at one time.
        """
        self.scrolling_frame.set_num_lines(num_lines)

    def _create_ui_widget(self, num_lines, label, text, tooltip, include_copy_btn):
        containing_frame = Frame().frame
        with containing_frame:
            with ui.VStack(style=get_style(), spacing=5):
                with ui.HStack():
                    self._label = ui.Label(
                        label, width=LABEL_WIDTH / 2, alignment=ui.Alignment.LEFT_TOP, tooltip=format_tt(tooltip)
                    )
                    self._scrolling_frame = ScrollingFrame(num_lines=num_lines)
                    with self._scrolling_frame:
                        self._text_block = ui.Label(
                            text,
                            style_type_name_override="Label::label",
                            word_wrap=True,
                            alignment=ui.Alignment.LEFT_TOP,
                        )
                    if include_copy_btn:
                        with ui.Frame(width=0, tooltip="Copy To Clipboard"):
                            self._copy_btn = ui.Button(
                                name="IconButton",
                                width=20,
                                height=20,
                                clicked_fn=lambda: on_copy_to_clipboard(to_copy=self._text_block.text),
                                style=get_style()["IconButton.Image::CopyToClipboard"],
                                alignment=ui.Alignment.RIGHT_TOP,
                            )
        return containing_frame


class XYPlot(UIWidgetWrapper):
    def __init__(
        self,
        label: str,
        tooltip: str = "",
        x_data: Union[List[List], List] = [],
        y_data: Union[List[List], List] = [],
        x_min: float = None,
        x_max: float = None,
        y_min: float = None,
        y_max: float = None,
        x_label: str = "X",
        y_label: str = "Y",
        plot_height: int = 10,
        show_legend: bool = False,
        legends: List[str] = None,
        plot_colors: List[List[int]] = None,
    ):
        """Create an XY plot UI Widget with axis scaling, legends, and support for multiple plots.
        Overlapping data is most accurately plotted when centered in the frame with reasonable axis scaling.
        Pressing down the mouse gives the x and y values of each function at an x coordinate.

        Args:
            label (str): Short descriptve text to the left of the plot
            tooltip (str, optional): Tooltip to appear when hovering the mouse over the plot label. Defaults to "".
            x_data (Union[List[List],List], optional): A possibly ragged list of lists where each ith inner list is the x coordinates for plot i.
                For a single plot, the data may be provided as a list of floats.  x_data must have exactly the same shape as y_data.  Defaults to [].
            y_data (Union[List[List],List], optional): A possibly ragged list of lists where each ith inner list is the y coordinates for plot i.
                For a single plot, the data may be provided as a list of floats.  y_data must have exactly the same shape as x_data.  Defaults to [].
            x_min (float, optional): Minimum value of x shown on the plot. If not specified, the minimum value found in x_data will be uesd. Defaults to None.
            x_max (float, optional): Maximum value of x shown on the plot.  If not specified, the maximum value found in x_data will be used. Defaults to None.
            y_min (float, optional): Minimum value of y shown on the plot. If not specified, the minimum value found in y_data will be uesd. Defaults to None.
            y_max (float, optional): Maximum value of y shown on the plot.  If not specified, the maximum value found in y_data will be used. Defaults to None.
            x_label (str, optional): Label of X axis. Defaults to "X".
            y_label (str, optional): Label of Y axis. Defaults to "Y".
            plot_height (int, optional): Height of the plot, proportional to the height of a line of text. Defaults to 10.
            show_legend (bool, optional): Show a legend on the plot. Defaults to False.
            legends (List[str], optional): Legend for the plotted data.  If not specified, names 'F_i(x)' will be automatically generated. Defaults to None.
            plot_colors (List[List], optional): Colors of the plotted data.  The ith entry in plot_colors is considered to be a list of [r,g,b] values in [0,255] for the ith plot color.
                If not specified, colors will be automatically generated.  Defaults to None.
        """
        self._has_built = False

        self._x_min = float(x_min) if x_min is not None else None
        self._x_max = float(x_max) if x_max is not None else None

        self._y_min = float(y_min) if y_min is not None else None
        self._y_max = float(y_max) if y_max is not None else None

        self._x_axis_float_fields = []
        self._y_axis_float_fields = []

        # These will be immediately resolved in set_data
        self._no_data = None
        self._is_plot_visible = None

        # Check assertions around data shape, reshape data; set class variables: self._no_data, self._is_plot_visible
        self.set_data(x_data, y_data)

        self._show_legend = bool(show_legend)
        self._legends = legends

        self._label = label
        self._tooltip = tooltip
        self._x_label = x_label
        self._y_label = y_label
        self._plot_num_lines = int(plot_height)

        self._data_colors = None
        if plot_colors is not None:
            self.set_plot_colors(plot_colors)

        self._num_points_per_plot = 3000

        plot_frame = self._create_ui_widget()
        super().__init__(plot_frame)

    def get_x_data(self) -> List[List[float]]:
        """x_data in the plot

        Returns:
            List[List[float]]: A possibly ragged list of lists where each ith inner list is the x coordinates for plot i.
        """
        return self._x_data

    def get_y_data(self) -> List[List[float]]:
        """y_data in the plot

        Returns:
            List[List[float]]: A possibly ragged list of lists where each ith inner list is the y coordinates for plot i.
        """
        return self._y_data

    def get_x_min(self) -> float:
        """Get the minimum value of x shown in the plot.

        Returns:
            float: Minimum value of x shown in the plot.
        """
        if self._x_min is not None:
            return float(self._x_min)
        elif self._x_data is not None:
            return self._get_ragged_data_min(self._x_data)
        else:
            return None

    def get_y_min(self) -> float:
        """Get the minimum value of y shown in the plot.

        Returns:
            float: Minimum value of y shown in the plot.
        """
        if self._y_min is not None:
            return float(self._y_min)
        elif self._y_data is not None:
            return self._get_ragged_data_min(self._y_data)
        else:
            return None

    def get_x_max(self) -> float:
        """Get the maximum value of x shown in the plot.

        Returns:
            float: Maximum value of x shown in the plot.
        """
        if self._x_max is not None:
            return float(self._x_max)
        elif self._x_data is not None:
            return self._get_ragged_data_max(self._x_data)
        else:
            return None

    def get_y_max(self) -> float:
        """Get the maximum value of y shown in the plot.

        Returns:
            float: Maximum value of y shown in the plot.
        """
        if self._y_max is not None:
            return float(self._y_max)
        elif self._y_data is not None:
            return self._get_ragged_data_max(self._y_data)
        else:
            return None

    def get_legends(self) -> List[str]:
        """Get the legends for the plotted data.

        Returns:
            List[str]: Legends for the plotted data
        """
        legends = []
        if self._legends is not None:
            for legend in self._legends:
                legends.append(legend)

        i = len(legends)
        while i < len(self.get_x_data()):
            legends.append(f"F_{i}(x)")
            i += 1

        if len(legends) > len(self._x_data):
            return legends[: len(self._x_data)]

        return legends

    def get_plot_height(self) -> int:
        """Get the height of the plot, proportional to the height of a line of text

        Returns:
            int: Height of the plot
        """
        return self._plot_num_lines

    def get_plot_colors(self) -> List[List[int]]:
        """Get the colors of the data in the plot

        Returns:
            List[List[int]]: List of lists where each inner list has length 3 corresponding to r,g,b values.
        """
        if self._data_colors is None:
            return [self._convert_hex_to_rgb(data_color) for data_color in self._get_distinct_colors(len(self._x_data))]
        else:
            return [self._convert_hex_to_rgb(data_color) for data_color in self._data_colors]

    def set_plot_color_by_index(self, index: int, r: int, g: int, b: int):
        """Set the color of a specific plot.

        Args:
            index (int): Index of the plot corresponding to the rows of x_data and y_data.
            r (int): Value for red in [0,255]
            g (int): Value for green in [0,255]
            b (int): Value for blue in [0,255]
        """
        if index >= len(self._x_data):
            carb.log_error("Index out of bounds for color on plot")
            return
        self._data_colors = self._get_data_colors(len(self._x_data))

        self._data_colors[index] = self._convert_rgb_to_hex(r, g, b)

        if self._has_built:
            self._plot_frames[index].rebuild()
            self._legend_frame.rebuild()

    def set_plot_colors(self, plot_colors: List[List[int]]):
        """Set the colors for every plot

        Args:
            plot_colors (List[List[int]]): A list of lists where each index corresponds to the rows of x_data and y_data.  Each inner list must
                contain [r,g,b] color values in [0,255]
        """
        if self._data_colors is None:
            self._data_colors = [self._convert_rgb_to_hex(r, g, b) for (r, g, b) in plot_colors]
        else:
            for i in range(len(plot_colors)):
                if i <= len(self._data_colors):
                    self._data_colors[i] = self._convert_rgb_to_hex(*plot_colors[i])
                else:
                    self._data_colors.append(self._convert_rgb_to_hex(*plot_colors[i]))

        if self._has_built:
            for plot in self._plot_frames:
                plot.rebuild()
            self._legend_frame.rebuild()

    def set_x_min(self, x_min: float):
        """Set the minimum value of x shown on the plot.
        If this value is not less than x_max, x_max will be updated to x_min + 1.

        Args:
            x_min (float): Minimum value of x shown on the plot.
        """
        x_min = float(x_min)
        self._x_min = x_min

        if self._has_built:
            self._x_min_float_drag.model.set_value(x_min)

    def set_y_min(self, y_min: float):
        """Set the minimum value of y shown on the plot.
        If this value is not less than y_max, y_max will be updated to y_min + 1.

        Args:
            y_min (float): Minimum value of y shown on the plot.
        """
        y_min = float(y_min)
        self._y_min = y_min

        if self._has_built:
            self._y_min_float_drag.model.set_value(y_min)

    def set_x_max(self, x_max: float):
        """Set maximum value of x shown on the plot.
        If this value is not greater than x_min, x_min will be updated to x_max - 1.

        Args:
            x_max (float): Maximum value of x shown on the plot.
        """
        x_max = float(x_max)
        self._x_max = x_max

        if self._has_built:
            self._x_max_float_drag.model.set_value(x_max)

    def set_y_max(self, y_max: float):
        """Set maximum value of y shown on the plot.
        If this value is not greater than y_min, y_min will be updated to y_max - 1.

        Args:
            y_max (float): Maximum value of y shown on the plot.
        """
        y_max = float(y_max)
        self._y_max = y_max

        if self._has_built:
            self._y_max_float_drag.model.set_value(y_max)

    def set_legend_by_index(self, idx: int, legend: str):
        """Set the legend for a specific plot whose index corresponds to the rows of x_data and y_data

        Args:
            idx (int): Index of legend to set.
            legend (str): Legend
        """
        if idx >= len(self._x_data) or idx < -len(self._x_data):
            carb.log_error("Legend index out of bounds")
        legends = self.get_legends()
        legends[idx] = legend

        self._legends = legends
        if self._has_built:
            self._legend_frame.invalidate_raster()
            self._legend_frame.rebuild()

    def set_legends(self, legends: List[str]):
        """Set legends for each plot.

        Args:
            legends (List[str]): List of legends for each plot.
        """
        if len(legends) != len(self._x_data):
            carb.log_error("Number of legends must match the number of plots")
        self._legends = legends
        if self._has_built:
            self._legend_frame.invalidate_raster()
            self._legend_frame.rebuild()

    def set_plot_height(self, plot_height: int):
        """Set the height of the plot.

        Args:
            plot_height (int): Height of the plot, proportional to the height of a line of text.
        """
        plot_height = int(plot_height)
        self._plot_num_lines = plot_height
        if self._has_built:
            self.container_frame.rebuild()

    def set_show_legend(self, show_legend: bool):
        """Hide or show the legend for this Widget

        Args:
            show_legend (bool): If True, show a legend for the Widget.
        """
        show_legend = bool(show_legend)
        self._show_legend = show_legend
        if self._has_built:
            self._show_legend_cb.model.set_value(show_legend)

    def set_data(self, x_data: Union[List[List], List], y_data: Union[List[List], List]):
        """Set the data to plot

        Args:
            x_data (Union[List[List],List]): A possibly ragged list of lists where each ith inner list is the x coordinates for plot i.
                For a single plot, the data may be provided as a list of floats.  x_data must have exactly the same shape as y_data.
            y_data (Union[List[List],List]): A possibly ragged list of lists where each ith inner list is the y coordinates for plot i.
                For a single plot, the data may be provided as a list of floats.  y_data must have exactly the same shape as x_data.
        """
        if len(x_data) != len(y_data):
            carb.log_error(f"x_data and y_data arguments must have the same shape.")

        self._no_data = True

        if len(x_data) == 0:
            self._x_data = [[]]
            self._y_data = [[]]
            self._is_plot_visible = []
            return

        if not isinstance(x_data[0], Iterable) and not isinstance(y_data[0], Iterable):
            if len(x_data) > 1:
                self._no_data = False
            x_data = [x_data]
            y_data = [y_data]

        for i in range(len(x_data)):
            if len(x_data[i]) != len(y_data[i]):
                carb.log_error(f"x_data and y_data arguments must have the same shape.  Mismatch found at index {i}")
            if len(x_data[i]) > 1:
                self._no_data = False

        self._x_data = x_data
        self._y_data = y_data

        self._is_plot_visible = [True] * len(self._x_data)

        if self._has_built:
            self.container_frame.rebuild()

    def set_plot_visible_by_index(self, index: int, visible: bool):
        """Hide or show a specific plot

        Args:
            index (int): Index of plot to show
            visible (bool):If True, show the plot, otherwise hide it.
        """
        if index >= len(self._x_data):
            carb.log_error("Index out of bounds for plot.")
            return

        self._is_plot_visible[index] = visible

        if self._has_built:
            if len(self._show_plot_cbs) == 0:
                self._plot_frames[index].visible = visible
            else:
                self._show_plot_cbs[index].model.set_value(visible)

    def _get_data_colors(self, num_colors) -> List[int]:
        # Get hex colors to use in the plot

        if self._data_colors is None:
            return self._get_distinct_colors(num_colors)
        elif len(self._data_colors) < num_colors:
            return self._data_colors + self._get_distinct_colors(num_colors - len(self._data_colors))
        else:
            return self._data_colors

    def _get_ragged_data_min(self, data) -> float:
        # Get the minimum value in a set of ragged data

        data_min = None

        for row in data:
            if len(row) > 0:
                m = np.min(row)
                if data_min is None or m < data_min:
                    data_min = m

        return float(data_min)

    def _get_ragged_data_max(self, data) -> float:
        # Get the maximum value in a set of ragged data

        data_max = None

        for row in data:
            if len(row) > 0:
                m = np.max(row)
                if data_max is None or m > data_max:
                    data_max = m

        return float(data_max)

    def _get_interpolated_data(self, x_min=None, x_max=None):
        """Get all data necessary for plotting

        Returns:
            x_fracs (np.array): (N x 2) corresponding to the fraction of x values that are covered by the min and max
                x values for each plot
            y_vals (np.array): (N x ?) a list of y values corresponding to each plot.  The shape may be ragged
            x_min (float): minimum value of x to be shown in the plot
            x_max (float): maximum value of x to be shown in the plot
        """

        if self._no_data:
            # There is no data at all
            return [[0, 1]], [[]], 0, 1

        if x_min is None:
            x_min = self._get_ragged_data_min(self._x_data)
        if x_max is None:
            x_max = self._get_ragged_data_max(self._x_data)

        if x_max <= x_min:
            return [[0, 1]] * len(self._y_data), [[]] * len(self._y_data), 0, 1

        spacing = (x_max - x_min) / self._num_points_per_plot
        x_val_range = np.arange(x_min, x_max + 0.5 * spacing, spacing)

        y_vals = []
        x_fracs = []

        for i in range(len(self._y_data)):
            y_vals.append([])
            for j in range(len(self._y_data[i]) - 1):
                y_low = self._y_data[i][j]
                y_high = self._y_data[i][j + 1]
                inds = np.where((x_val_range >= self._x_data[i][j]) & (x_val_range < self._x_data[i][j + 1]))[0]
                if len(inds) == 0:
                    continue
                x_frac = (x_val_range[inds] - self._x_data[i][j]) / (self._x_data[i][j + 1] - self._x_data[i][j])
                interp_fun = lambda x: y_low + x_frac * (y_high - y_low)
                y_vals[-1].extend(interp_fun(x_val_range[inds]))

            if len(self._x_data[i]) <= 1:
                x_fracs.append([0.0, 1.0])
                continue

            inds = np.where((x_val_range >= self._x_data[i][0]) & (x_val_range <= self._x_data[i][-1]))[0]
            if len(inds) > 0:
                x_fracs.append(
                    [
                        (x_val_range[inds[0]] - x_min) / (x_max - x_min),
                        (x_val_range[inds[-1]] - x_min) / (x_max - x_min),
                    ]
                )
            elif self._x_data[i][0] > x_val_range[-1]:
                x_fracs.append([1.0, 1.0])
            else:
                x_fracs.append([0.0, 0.0])

        return x_fracs, y_vals, x_min, x_max

    def _convert_rgb_to_hex(self, r, g, b) -> int:
        # convert an rgb color to a hex value
        return 0xFF * 16**6 + b * 16**4 + g * 16**2 + r

    def _convert_hex_to_rgb(self, hex_color) -> Tuple[int]:
        def residue(number):
            mask = 0xFFFFFF00
            number = number - (number & mask)
            return number

        r = residue(hex_color)
        g = residue(int(hex_color / 16**2))
        b = residue(int(hex_color / 16**4))
        return r, g, b

    def _get_distinct_colors(self, num_colors) -> List[int]:
        """
        This function returns a list of distinct colors for plotting.

        Args:
            num_colors (int): the number of colors to generate

        Returns:
            List[int]: a list of distinct colors in hexadecimal format 0xFFBBGGRR
        """
        import colorsys

        colors = []

        hues = [0, 0.25, 0.5, 0.9]
        sorted_hues = [0, 0.25, 0.5, 0.9]
        while len(hues) < num_colors:
            ind = np.argmax(np.diff(sorted_hues))
            hues.append((sorted_hues[ind] + sorted_hues[ind + 1]) / 2)
            sorted_hues.append(hues[-1])
            sorted_hues.sort()

        for i in range(num_colors):
            hue = hues[i]
            saturation = 0.8
            value = 1
            rgb = colorsys.hsv_to_rgb(hue, saturation, value)
            r, g, b = [int(255 * x) for x in rgb]
            color_hex = self._convert_rgb_to_hex(r, g, b)
            colors.append(color_hex)

        return colors

    def _get_fn_y_value(self, idx, x_value, decimals) -> float:
        # Get value of each plot evaluated at x_value.  This may require linear interpolation
        x_data = self._x_data[idx]
        y_data = self._y_data[idx]

        if x_value == x_data[-1]:
            return np.round(y_data[-1], decimals=decimals)

        for i in range(len(x_data) - 1):
            if x_value >= x_data[i] and x_value < x_data[i + 1]:
                frac = (x_value - x_data[i]) / (x_data[i + 1] - x_data[i])
                return np.round(y_data[i] + frac * (y_data[i + 1] - y_data[i]), decimals=decimals)

        return ""

    def _mouse_moved_on_plot(self, x, y, *args):
        # Show a tooltip with x,y and function values
        if self._no_data == True:
            # There is no data in the plots, so do nothing
            return

        tt_frame = self._tooltip_frame
        width = tt_frame.computed_width - 14
        height = tt_frame.computed_height - 8
        x_pos = tt_frame.screen_position_x + 8
        y_pos = tt_frame.screen_position_y + 5

        tt_frame.tooltip_offset_x = 0
        tt_frame.tooltip_offset_y = 0

        x_min = self.get_x_min()
        x_max = self.get_x_max()
        y_max = self.get_y_max()
        y_min = self.get_y_min()

        if x_max == x_min or y_max == y_min:
            # Can only happen when the user is intentionally trying to break things
            return

        num_decimals_x = int(max(0, np.log10(5 / (x_max - x_min)) + 2))
        num_decimals_y = int(max(0, np.log10(5 / (y_max - y_min)) + 2))

        x_value = np.round((x_max - x_min) * (x - x_pos) / width + x_min, decimals=num_decimals_x)
        y_value = np.round(y_max - (y_max - y_min) * (y - y_pos) / height, decimals=num_decimals_y)
        tt_string = f"x = {x_value},"
        tt_string += f"y = {y_value}\n\n"
        legends = self.get_legends()
        for i in range(len(self._interpolated_y_data)):
            if not self._plot_frames[i].visible:
                continue
            tt_string += legends[i] + " : "
            tt_string += f"{self._get_fn_y_value(i,x_value,num_decimals_y)}\n"
        tt_frame.set_tooltip(tt_string)

    def _on_mouse_released(self, *args):
        # Hide the tooltip off screen
        self._tooltip_frame.tooltip = " "
        self._tooltip_frame.tooltip_offset_x = -2000
        self._tooltip_frame.tooltip_offset_y = -2000

    def _create_ui_widget(self):
        containing_frame = ui.Frame(build_fn=self._build_widget)

        return containing_frame

    def _build_widget(self):
        self._plot_frames = []
        self._plots = []

        self._color_widgets = []
        self._show_plot_cbs = []

        LINE_HEIGHT = 23

        label = self._label
        tooltip = self._tooltip
        x_label = self._x_label
        y_label = self._y_label
        plot_num_lines = self._plot_num_lines

        plot_height = plot_num_lines * LINE_HEIGHT

        x_fracs, y_data, x_min, x_max = self._get_interpolated_data(self._x_min, self._x_max)
        self._x_min = x_min
        self._x_max = x_max
        self._interpolated_y_data = y_data
        self._x_fracs = x_fracs

        self._data_colors = self._get_data_colors(len(y_data))

        def on_show_legend(model):
            if model.get_value_as_bool():
                self._show_legend = True
                self._legend_frame.visible = True
                self._legend_frame.rebuild()
            else:
                self._show_legend = False
                self._legend_frame.visible = False

        def on_color_widget_set_plot_color(idx):
            plot_frame = self._plot_frames[idx]
            color_widget = self._color_widgets[idx]

            rgb_color = []
            for item in color_widget.model.get_item_children():
                val = color_widget.model.get_item_value_model(item).get_value_as_float()
                rgb_color.append(int(255 * val))
            hex_color = self._convert_rgb_to_hex(*rgb_color[:3])
            self._data_colors[idx] = hex_color

            plot_frame.rebuild()

        def build_legend_frame():
            def toggle_plot_visibility(show, i):
                self._is_plot_visible[i] = show
                if i < len(self._plot_frames):
                    self._plot_frames[i].visible = show

            legends = self.get_legends()
            self._color_widgets = []
            self._show_plot_cbs = []
            with ui.HStack(spacing=3):
                ui.Spacer()
                for i in range(len(self._x_data)):
                    label_frame_width = LABEL_HEIGHT * len(legends[i]) / 3 + 8
                    ui.Label(legends[i], width=label_frame_width, alignment=ui.Alignment.RIGHT)

                    color_widget = ui.ColorWidget(width=10)
                    color_widget.model.add_end_edit_fn(lambda *args, idx=i: on_color_widget_set_plot_color(idx))
                    self._color_widgets.append(color_widget)

                    color = list(self._convert_hex_to_rgb(self._data_colors[i]))
                    color.append(255)
                    color = np.array(color) / 255.0
                    for j, item in enumerate(color_widget.model.get_item_children()):
                        val = color_widget.model.get_item_value_model(item).set_value(color[j])

                    model = ui.SimpleBoolModel()
                    model.set_value(self._is_plot_visible[i])
                    show_plot_cb = ui.CheckBox(model=model)
                    self._show_plot_cbs.append(show_plot_cb)
                    model.add_value_changed_fn(
                        lambda model, idx=i: toggle_plot_visibility(model.get_value_as_bool(), idx)
                    )

        def set_x_axis_values(low, high):
            assert high >= low

            if high == low:
                high = low + 0.0001

                # Handle float overflow
                if high == low:
                    return

            num_fields = len(self._x_axis_float_fields)
            spacing = (high - low) / num_fields
            num_decimals = int(max(0, np.log10(5 / (high - low)) + 2))

            self._x_max_float_drag.step = (high - low) / 20
            self._x_min_float_drag.step = (high - low) / 20
            for i, float_field in enumerate(self._x_axis_float_fields):
                float_field.model.set_value(np.round(low + spacing / 2 + spacing * i, decimals=num_decimals))

        def set_y_axis_values(low, high):
            assert high >= low

            if high == low:
                high = low + 0.0001

                # Handle float overflow
                if high == low:
                    return

            num_fields = len(self._y_axis_float_fields)
            spacing = (high - low) / num_fields
            num_decimals = int(max(0, np.log10(5 / (high - low)) + 2))
            self._y_min_float_drag.step = (high - low) / 20
            self._y_max_float_drag.step = (high - low) / 20
            for i, float_field in enumerate(self._y_axis_float_fields):
                float_field.model.set_value(np.round(high - spacing / 2 - spacing * i, decimals=num_decimals))

        def build_x_axis_frame():
            self._x_axis_float_fields = []

            def update_x_min(model):
                self._x_min = model.as_float
                if self._x_min >= self._x_max:
                    self._x_max = self._x_min + 1.0
                    self._x_max_float_drag.model.set_value(self._x_max)
                self._x_fracs, self._interpolated_y_data, _, _ = self._get_interpolated_data(
                    x_min=self._x_min, x_max=self._x_max
                )

                for i, plot_frame in enumerate(self._plot_frames):
                    plot_frame.set_build_fn(
                        build_fn=lambda x_fracs=self._x_fracs[i], y_data=self._interpolated_y_data[
                            i
                        ], color_idx=i: build_plot(x_fracs, y_data, color_idx)
                    )

                set_x_axis_values(self._x_min, self._x_max)

            def update_x_max(model):
                self._x_max = model.as_float
                if self._x_max <= self._x_min:
                    self._x_min = self._x_max - 1.0
                    self._x_min_float_drag.model.set_value(self._x_min)
                self._x_fracs, self._interpolated_y_data, _, _ = self._get_interpolated_data(
                    x_min=self._x_min, x_max=self._x_max
                )

                for i, plot_frame in enumerate(self._plot_frames):
                    plot_frame.set_build_fn(
                        build_fn=lambda x_fracs=self._x_fracs[i], y_data=self._interpolated_y_data[
                            i
                        ], color_idx=i: build_plot(x_fracs, y_data, color_idx)
                    )

                set_x_axis_values(self._x_min, self._x_max)

            with ui.VStack(spacing=0):
                with ui.ZStack():
                    ui.FloatDrag(
                        enabled=False
                    )  # This fills in the whole x axis label area with the right color.  The number will get covered up.

                    with ui.HStack():
                        ui.Spacer(
                            width=6
                        )  # There is blank space between the plotted line and the sides of the plot's colored rectangle
                        for i in range(plot_num_lines):
                            float_field = ui.FloatDrag(enabled=False, alignment=ui.Alignment.CENTER)
                            self._x_axis_float_fields.append(float_field)
                        ui.Spacer(width=6)

                ui.Spacer(height=3)

                with ui.HStack():
                    # Add fields for controlling min and max x values on plot
                    self._x_min_float_drag = ui.FloatDrag(
                        name="Field", alignment=ui.Alignment.LEFT_TOP, tooltip="X Min"
                    )
                    x_min_model = self._x_min_float_drag.model
                    x_min_model.set_value(self._x_min)

                    ui.Label(x_label, alignment=ui.Alignment.CENTER_TOP, height=LABEL_HEIGHT)

                    self._x_max_float_drag = ui.FloatDrag(
                        name="Field", alignment=ui.Alignment.LEFT_BOTTOM, tooltip="X Max"
                    )
                    x_max_model = self._x_max_float_drag.model
                    x_max_model.set_value(self._x_max)

                    x_min_model.add_value_changed_fn(update_x_min)
                    x_max_model.add_value_changed_fn(update_x_max)

            set_x_axis_values(x_min, x_max)

        def build_y_axis_frame():
            # Add fields for controlling min and max y values on plot
            def update_y_min(model):
                self._y_min = model.as_float

                if self._y_min >= self._y_max:
                    self._y_max = self._y_min + 1.0
                    self._y_max_float_drag.model.set_value(self._y_max)

                for plot in self._plots:
                    plot.scale_min = model.as_float

                set_y_axis_values(self._y_min, self._y_max)

            def update_y_max(model):
                self._y_max = model.as_float
                if self._y_max <= self._y_min:
                    self._y_min = self._y_max - 1.0
                    self._y_min_float_drag.model.set_value(self._y_min)

                for plot in self._plots:
                    plot.scale_max = model.as_float

                set_y_axis_values(self._y_min, self._y_max)

            y_max = self.get_y_max()
            y_min = self.get_y_min()

            if y_min is None:
                y_min = 0.0
            self._y_min = y_min

            if y_max is None:
                y_max = y_min + 1.0
            self._y_max = y_max

            with ui.VStack():
                with ui.HStack(spacing=2):
                    # Make it wide enough to contain the text on one line with a minimum size of 60 pixels
                    label_frame_width = max(LABEL_HEIGHT * len(y_label) / 3 + 3, 60)

                    with ui.Frame(width=label_frame_width, height=plot_height):

                        # Make a frame with y axis max, min and label that has width label_frame_width
                        with ui.VStack():
                            self._y_max_float_drag = ui.FloatDrag(
                                name="Field",
                                width=label_frame_width,
                                height=LABEL_HEIGHT,
                                alignment=ui.Alignment.CENTER,
                                tooltip="Y Max",
                            )
                            y_max_model = self._y_max_float_drag.model

                            y_max_model.set_value(y_max)

                            with ui.HStack():
                                ui.Spacer()
                                ui.Label(y_label, word_wrap=True, alignment=ui.Alignment.LEFT_CENTER)
                                ui.Spacer()

                            self._y_min_float_drag = ui.FloatDrag(
                                name="Field",
                                width=label_frame_width,
                                height=LABEL_HEIGHT,
                                alignment=ui.Alignment.CENTER,
                                tooltip="Y Min",
                            )
                            y_min_model = self._y_min_float_drag.model
                            y_min_model.set_value(y_min)

                            y_min_model.add_value_changed_fn(update_y_min)
                            y_max_model.add_value_changed_fn(update_y_max)

                    # Make axis markers
                    self._y_axis_float_fields = []
                    with ui.Frame(height=plot_height):
                        with ui.ZStack():
                            ui.FloatDrag(
                                enabled=False
                            )  # This fills in the whole y axis label area with the right color.  The number will get covered up.

                            with ui.VStack(spacing=0):
                                ui.Spacer(
                                    height=3
                                )  # There is blank space between the plotted line and the edges of the plot's colored rectangle
                                for i in range(plot_num_lines):
                                    float_field = ui.FloatDrag(enabled=False, alignment=ui.Alignment.CENTER)
                                    self._y_axis_float_fields.append(float_field)
                                ui.Spacer(height=3)

                # Fill "Show Legend" Frame
                with ui.HStack():
                    ui.Label(" Show\nLegend")
                    model = ui.SimpleBoolModel()
                    model.set_value(self._show_legend)
                    ui.Spacer(width=3)

                    with ui.VStack():
                        ui.Spacer()
                        self._show_legend_cb = ui.CheckBox(model=model, alignment=ui.Alignment.CENTER)
                        ui.Spacer()

                    ui.Spacer()
                    model.add_value_changed_fn(on_show_legend)

            set_y_axis_values(y_min, y_max)

        def build_plot(x_fracs, y_data, color_idx):
            """Build the frame for a plot

            Args:
                x_fracs (np.array (2,)): Fraction of available space that the plot should cover
                y_data (np.array): data with which to fill the plot
            """

            color = self._data_colors[color_idx]
            visible = self._is_plot_visible[color_idx]
            with ui.HStack():
                # ui.Frame is the only omni.ui object that seems to consistently obey the given spacing rules
                # So each plot is on a Frame between two other invisible frames to get the placement right
                w = self._base_plot.computed_width
                if w == 0:
                    return

                # Plots have 6 pixel margins on them, so the fraction of the figure occupied needs to be modified
                x_low = max(0.0, x_fracs[0] - 6 / w)
                x_high = min(1.0, x_fracs[1] + 6 / w)
                y_min = self.get_y_min()
                y_max = self.get_y_max()
                if y_min is None:
                    y_min = 0.0
                if y_max is None:
                    y_max = 1.0

                f = ui.Frame(width=ui.Fraction(x_low))
                with ui.Frame(width=ui.Fraction(x_high - x_low)):
                    plot = ui.Plot(
                        ui.Type.LINE,
                        y_min,
                        y_max,
                        *y_data,
                        height=plot_height,
                        style={"color": color, "background_color": 0x0},
                    )
                    self._plots.append(plot)
                ui.Frame(width=ui.Fraction(1 - x_high))

        with ui.VStack(spacing=3):
            with ui.HStack():
                # Write the leftmost label for what this plot is
                ui.Label(label, width=LABEL_WIDTH, alignment=ui.Alignment.LEFT_TOP, tooltip=format_tt(tooltip))

                with ui.HStack(spacing=0):
                    # Make the y axis label
                    y_axis_frame = ui.Frame(build_fn=build_y_axis_frame, width=ui.Fraction(0.25))

                    # VStack the plot on top of the X axis label
                    with ui.VStack(spacing=0):

                        # ZStacking everything that goes on the actual data plot
                        with ui.ZStack():
                            ui.Rectangle(height=plot_height)
                            self._base_plot = ui.Plot(ui.Type.LINE, 0.0, 1.0, 0.0, style={"background_color": 0x0})
                            for i in range(len(y_data)):
                                plot_frame = Frame(
                                    build_fn=lambda x_fracs=x_fracs[i], y_data=y_data[i], color_idx=i: build_plot(
                                        x_fracs, y_data, color_idx
                                    )
                                )
                                plot_frame.visible = (
                                    self._is_plot_visible[i] if i < len(self._is_plot_visible) else False
                                )
                                self._plot_frames.append(plot_frame)

                            # Create an invisible frame on top that will give a helpful tooltip
                            self._tooltip_frame = ui.Plot(
                                mouse_moved_fn=self._mouse_moved_on_plot,
                                style={"color": 0xFFFFFFFF, "background_color": 0x0},
                            )
                            self._tooltip_frame.set_mouse_pressed_fn(self._mouse_moved_on_plot)
                            self._tooltip_frame.set_mouse_released_fn(self._on_mouse_released)
                            self._tooltip_frame.set_computed_content_size_changed_fn(
                                lambda *args: [plot.rebuild() for plot in self._plot_frames]
                            )
                            # Make the tooltip invisible
                            self._on_mouse_released()

                        # Create the x axis label
                        x_axis_frame = Frame(build_fn=build_x_axis_frame).frame
                        x_axis_frame.rebuild()

                ui.Spacer(width=20)

            with ui.HStack():
                ui.Spacer(width=LABEL_WIDTH)
                self._legend_frame = ui.Frame(build_fn=build_legend_frame)
                self._legend_frame.rebuild()
                self._legend_frame.visible = self._show_legend
                ui.Spacer(width=20)

            add_separator()

        self._has_built = True
