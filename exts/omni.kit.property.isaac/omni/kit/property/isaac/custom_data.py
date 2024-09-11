# Copyright (c) 2018-2023, NVIDIA CORPORATION. All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto. Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.
#

from typing import List

import carb
import numpy as np
import omni.ui as ui
from omni.kit.property.usd.usd_attribute_model import UsdAttributeModel
from omni.kit.property.usd.usd_property_widget import UsdPropertiesWidget, UsdPropertyUiEntry
from omni.kit.property.usd.usd_property_widget_builder import UsdPropertiesWidgetBuilder
from omni.kit.property.usd.widgets import ICON_PATH
from omni.kit.window.property.templates import (
    HORIZONTAL_SPACING,
    LABEL_HEIGHT,
    LABEL_WIDTH,
    SimplePropertyWidget,
    build_frame_header,
)
from pxr import Gf, Sdf, Tf, Usd


def iterate_custom_data(custom_data):
    for key, value in custom_data.items():
        if isinstance(value, dict):
            iterate_custom_data(value)
        else:
            custom_data[key] = np.array(custom_data[key]).tolist()


class CustomDataWidget(SimplePropertyWidget):
    def _get_prim(self, prim_path):
        if prim_path:
            stage = self._payload.get_stage()
            if stage:
                return stage.GetPrimAtPath(prim_path)
        return None

    def on_new_payload(self, payload):
        """
        See PropertyWidget.on_new_payload
        """

        if not super().on_new_payload(payload):
            return False

        if len(self._payload) != 1:
            return False
        prim_path = self._payload.get_paths()[0]
        self._prim = self._get_prim(prim_path)
        if not self._prim:
            return False

        return True

    def build_items(self):
        import json

        def dupe_checking_hook(pairs):
            result = dict()
            for key, val in pairs:
                if key in result:
                    raise KeyError("Duplicate key specified: %s" % key)
                result[key] = val
            return result

        decoder = json.JSONDecoder(object_pairs_hook=dupe_checking_hook)

        data = ui.StringField(height=250, multiline=True).model
        ui.Label("Status:")
        error = ui.StringField(multiline=False).model
        error.set_value("Valid, changes saved")

        def validate(t):
            try:
                decoder.decode(t.get_value_as_string())
            except ValueError as e:
                error.set_value(str(e))
                return
            except KeyError as e:
                error.set_value(str(e))
                return
            error.set_value("Valid, changes saved")
            self._prim.SetCustomData(decoder.decode(t.get_value_as_string()))

        custom_data = self._prim.GetCustomData()

        iterate_custom_data(custom_data)

        j = json.dumps(custom_data, sort_keys=False, indent=4)

        data.set_value(j)
        data.add_value_changed_fn(lambda m: validate(m))

        pass

    def build_property_item(self, stage, ui_prop: UsdPropertyUiEntry, prim_paths: List[Sdf.Path]):
        if ui_prop.prim_paths:
            prim_paths = ui_prop.prim_paths
