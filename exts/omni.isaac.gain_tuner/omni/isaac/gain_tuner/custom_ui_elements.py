# Copyright (c) 2024, NVIDIA CORPORATION. All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto. Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.
#

from cmath import inf

import omni.ui as ui
from omni.isaac.ui.element_wrappers import FloatField, Frame
from omni.isaac.ui.ui_utils import LABEL_WIDTH, add_line_rect_flourish, format_tt


class LogFloatField(FloatField):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)

    def _convert_value_to_formatted_string(self, value):
        return "{:.2e}".format(10**value)

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

        self._fixed_field.model.set_value(self._convert_value_to_formatted_string(val))

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
                    width=ui.Fraction(0.78),
                    height=0,
                    alignment=ui.Alignment.LEFT_CENTER,
                    min=-inf,
                    max=inf,
                    step=step,
                    format=format,
                )
                self.float_field.model.set_value(default_value)

                ui.Spacer(width=ui.Fraction(0.02))

                self._fixed_field = ui.StringField(
                    name="StringField", width=ui.Fraction(0.2), height=0, alignment=ui.Alignment.RIGHT_CENTER
                )
                self._fixed_field.read_only = True
                self._fixed_field.model.set_value(
                    self._convert_value_to_formatted_string(self.float_field.model.as_float)
                )

                add_line_rect_flourish(False)

            self.float_field.model.add_value_changed_fn(self._on_value_changed_fn_wrapper)
        return containing_frame
