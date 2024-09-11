# Copyright (c) 2022 NVIDIA CORPORATION.  All rights reserved.
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto.  Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.

import omni.graph.core as og


class OgnControlsSettings:
    @staticmethod
    def compute(db) -> bool:
        db.outputs.control_settings.insert((og.Type(og.BaseDataType.TOKEN), "Forward"))
        db.outputs.control_settings.insert((og.Type(og.BaseDataType.TOKEN), "Backward"))
        db.outputs.control_settings.insert((og.Type(og.BaseDataType.TOKEN), "Right"))
        db.outputs.control_settings.insert((og.Type(og.BaseDataType.TOKEN), "Left"))
        db.outputs.control_settings.insert((og.Type(og.BaseDataType.TOKEN), "Up"))
        db.outputs.control_settings.insert((og.Type(og.BaseDataType.TOKEN), "Down"))
        db.outputs.control_settings.insert((og.Type(og.BaseDataType.FLOAT), "MouseSensitivity"))
        db.outputs.control_settings.insert((og.Type(og.BaseDataType.FLOAT), "GamepadSensitivity"))

        db.outputs.control_settings.attribute_by_name("Forward").value = db.inputs.forward
        db.outputs.control_settings.attribute_by_name("Backward").value = db.inputs.backward
        db.outputs.control_settings.attribute_by_name("Right").value = db.inputs.right
        db.outputs.control_settings.attribute_by_name("Left").value = db.inputs.left
        db.outputs.control_settings.attribute_by_name("Up").value = db.inputs.up
        db.outputs.control_settings.attribute_by_name("Down").value = db.inputs.down
        db.outputs.control_settings.attribute_by_name("MouseSensitivity").value = db.inputs.mouseSensitivity
        db.outputs.control_settings.attribute_by_name("GamepadSensitivity").value = db.inputs.gamepadSensitivity

        return True
