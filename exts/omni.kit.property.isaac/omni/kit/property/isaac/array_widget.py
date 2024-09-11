# Copyright (c) 2018-2023, NVIDIA CORPORATION. All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto. Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.
#

from typing import List

import omni
import omni.ui as ui
from omni.kit.property.usd.usd_attribute_model import UsdAttributeModel
from omni.kit.property.usd.usd_property_widget import UsdPropertiesWidget, UsdPropertyUiEntry
from omni.kit.property.usd.usd_property_widget_builder import UsdPropertiesWidgetBuilder
from omni.kit.property.usd.widgets import ICON_PATH
from omni.kit.window.property.templates import HORIZONTAL_SPACING, LABEL_HEIGHT, LABEL_WIDTH
from pxr import Gf, Sdf, Tf, Usd

REMOVE_BUTTON_STYLE = style = {"image_url": str(ICON_PATH.joinpath("remove.svg")), "margin": 0, "padding": 0}
ADD_BUTTON_STYLE = style = {"image_url": str(ICON_PATH.joinpath("plus.svg")), "margin": 1, "padding": 0}


class BaseMultiField:
    def __init__(self, model, index, count, frame, button_style, callback, field_type, default):
        self._model = model
        self._index = index
        self._frame = frame
        self._count = count

        item = (
            self._model.get_item(self._index)
            if index is not None
            else [default for i in range(count)]
            if count > 1
            else default
        )
        self._args = [item[i] for i in range(count)] if count != 1 else [item]
        self._field = field_type(*self._args, h_spacing=5).model
        ui.Spacer(width=2)
        ui.Button(style=button_style, width=16, clicked_fn=getattr(self, callback))
        if self._index is not None:
            self._field.add_item_changed_fn(
                lambda m, n, list_model=self._model, field=self._field, index=self._index: list_model.set_item(
                    index, self.get_tuple()
                )
            )

    def remove(self):
        self._model.remove_item(self._index)
        self._frame.rebuild()

    def append(self):
        self._model.add_item(self.get_tuple())
        self._frame.rebuild()

    def get_tuple(self):
        return (
            tuple([self.get_field_value(i) for i in range(self._count)])
            if self._count != 1
            else self.get_field_value(0)
        )


class CustomMultiIntField(BaseMultiField):
    def __init__(self, model, index, count, frame, button_style, callback):
        super().__init__(model, index, count, frame, button_style, callback, ui.MultiIntField, 0)

    def get_field_value(self, index):
        return self._field.get_item_value_model(self._field.get_item_children()[index]).get_value_as_int()


class CustomMultiFloatField(BaseMultiField):
    def __init__(self, model, index, count, frame, button_style, callback):
        super().__init__(model, index, count, frame, button_style, callback, ui.MultiFloatField, 0.0)

    def get_field_value(self, index):
        return self._field.get_item_value_model(self._field.get_item_children()[index]).get_value_as_float()


class ArrayBaseItem:
    def __init__(self, type_name, model, index=None, frame=None):
        self._value = None
        self._type_name = type_name
        self._model = model
        self._index = index
        self._frame = frame

    def create_list_item(self, button_style, callback):
        with ui.HStack(height=0):
            if self._type_name == Tf.Type.FindByName("VtArray<int>"):
                CustomMultiIntField(self._model, self._index, 1, self._frame, button_style, callback)
            elif self._type_name == Tf.Type.FindByName("VtArray<float>"):
                CustomMultiFloatField(self._model, self._index, 1, self._frame, button_style, callback)
            elif self._type_name == Tf.Type.FindByName("VtArray<GfVec2i>"):
                CustomMultiIntField(self._model, self._index, 2, self._frame, button_style, callback)
            elif self._type_name == Tf.Type.FindByName("VtArray<GfVec2f>"):
                CustomMultiFloatField(self._model, self._index, 2, self._frame, button_style, callback)
            elif self._type_name == Tf.Type.FindByName("VtArray<GfVec3i>"):
                CustomMultiIntField(self._model, self._index, 3, self._frame, button_style, callback)
            elif self._type_name == Tf.Type.FindByName("VtArray<GfVec3f>"):
                CustomMultiFloatField(self._model, self._index, 3, self._frame, button_style, callback)
            elif self._type_name == Tf.Type.FindByName("VtArray<GfVec4i>"):
                CustomMultiIntField(self._model, self._index, 4, self._frame, button_style, callback)
            elif self._type_name == Tf.Type.FindByName("VtArray<GfVec4f>"):
                CustomMultiFloatField(self._model, self._index, 4, self._frame, button_style, callback)


class ArrayWidgetBuilder:
    def __init__(self, stage, attr_name, type_name, model):
        self._model = model
        self._stage = stage
        self._attr_name = attr_name
        self._type_name = type_name
        label_kwargs = {"name": "label", "word_wrap": True, "height": LABEL_HEIGHT}
        ui.Label(attr_name, **label_kwargs)
        ui.Label(type_name.typeName, **label_kwargs)
        self._length_label = ui.Label(str(self._model.get_length()), **label_kwargs)
        ui.Button("Edit", clicked_fn=self._create_edit_window)

    def _create_edit_window(self):
        self._window = omni.ui.Window(
            f"Editing: {self._attr_name} {self._type_name.typeName}", width=600, height=400, visible=True
        )

        with self._window.frame:
            with ui.VStack():
                frame = ui.Frame()

                def rebuild():
                    with ui.ScrollingFrame():
                        with ui.VStack():
                            for i in range(self._model.get_length()):
                                mod = ArrayBaseItem(self._type_name, self._model, i, frame)
                                mod.create_list_item(REMOVE_BUTTON_STYLE, "remove")
                    self._length_label.text = str(self._model.get_length())

                frame.set_build_fn(rebuild)
                ui.Spacer(height=1)
                ui.Label("Add new item:", height=0)
                ui.Spacer(height=2)
                mod = ArrayBaseItem(self._type_name, self._model, None, frame)
                mod.create_list_item(ADD_BUTTON_STYLE, "append")
                ui.Spacer(height=10)

    pass


class UsdArrayAttributeModel(UsdAttributeModel):
    def get_length(self):
        self._update_value()
        return len(self._value)

    def get_value(self):
        return self._value

    def get_item(self, index):
        return self._value[index]

    def set_item(self, index, item):
        new_list = list(self._value)
        new_list[index] = item
        self.set_value(new_list)

    def add_item(self, item):
        new_list = list(self._value)
        new_list.append(item)
        self.set_value(new_list)

    def remove_item(self, index):
        new_list = list(self._value)
        del new_list[index]
        self.set_value(new_list)


class ArrayPropertiesWidget(UsdPropertiesWidget):
    def on_new_payload(self, payload):
        """
        See PropertyWidget.on_new_payload
        """

        if not super().on_new_payload(payload):
            return False

        if len(self._payload) == 0:
            return False

        for prim_path in self._payload:
            prim = self._get_prim(prim_path)
            if not prim:
                return False

        return True

    def _filter_props_to_build(self, props):
        # simple widget that handles array based properties
        return [
            prop
            for prop in props
            if isinstance(prop, Usd.Attribute)
            and any(
                prop.GetTypeName() == item
                for item in ["int[]", "float[]", "int2[]", "float2[]", "int3[]", "float3[]", "int4[]", "float4[]"]
            )
        ]

    def build_property_item(self, stage, ui_prop: UsdPropertyUiEntry, prim_paths: List[Sdf.Path]):
        if ui_prop.prim_paths:
            prim_paths = ui_prop.prim_paths

        if ui_prop.property_type == Usd.Attribute:
            type_name = UsdPropertiesWidgetBuilder._get_type_name(ui_prop.metadata)
            tf_type = type_name.type

            self._array_builder(stage, ui_prop.prop_name, tf_type, ui_prop.metadata, prim_paths)

    @classmethod
    def _array_builder(
        cls,
        stage,
        attr_name,
        type_name,
        metadata,
        prim_paths: List[Sdf.Path],
        additional_label_kwargs=None,
        additional_widget_kwargs=None,
    ):
        with ui.HStack(spacing=HORIZONTAL_SPACING):
            model = UsdArrayAttributeModel(
                stage, [path.AppendProperty(attr_name) for path in prim_paths], False, metadata
            )
            ArrayWidgetBuilder(stage, attr_name, type_name, model)

            return model
