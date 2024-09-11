# Copyright (c) 2020-2023, NVIDIA CORPORATION. All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto. Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.
#
import omni.usd

from .array_widget import ArrayPropertiesWidget
from .custom_data import CustomDataWidget
from .name_override import NameOverrideWidget


class IsaacPropertyWidgets(omni.ext.IExt):
    def __init__(self):
        super().__init__()
        self._registered = False

    def on_startup(self, ext_id):
        self._register_widget()

    def on_shutdown(self):
        self._unregister_widget()

    def _register_widget(self):
        import omni.kit.window.property as p

        w = p.get_window()
        w.register_widget("prim", "isaac_array", ArrayPropertiesWidget(title="Array Properties", collapsed=True), False)
        w.register_widget(
            "prim", "isaac_custom_data", CustomDataWidget(title="Prim Custom Data", collapsed=True), False
        )
        self._isaac_name_override = NameOverrideWidget(title="Name Override", collapsed=False)
        w.register_widget("prim", "isaac_name_override", self._isaac_name_override, False)

    def _unregister_widget(self):
        import omni.kit.window.property as p

        w = p.get_window()
        if w:
            w.unregister_widget("prim", "isaac_array")
            w.unregister_widget("prim", "isaac_custom_data")
            self._isaac_name_override.destroy()
            self._registered = False
