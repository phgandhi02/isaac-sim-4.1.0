from typing import List, Union

import carb.profiler
import omni.physxzerogravity.bindings._physXZeroGravity as pxzerog
import omni.timeline
from omni.kit.manipulator.prim.core.prim_transform_manipulator import PrimTransformManipulator
from omni.kit.manipulator.transform.settings_listener import OpSettingsListener
from pxr import Sdf, Usd

from .settings import ZeroGravitySettings
from .utils import is_placement_prim, refresh_manipulator_selector
from .zerog_transform_model import ZeroGravityTransformModel
from omni.kit.manipulator.transform.settings_constants import c

# The ZeroGravity transform custom manipulator class: this class handles enabling and
# disabling the custom ZG manipulator (whose model forwards translation/rotate events to
# the underlying PlacementMode C++ class where the core logic resides). E.g. the custom
# manipulator is disabled when scaling is on or it's switched on in the editor.


class ZeroGravityTransformManipulator(PrimTransformManipulator):
    def __init__(self, usd_context_name: str = "", viewport_api=None):
        super().__init__(usd_context_name, viewport_api, "omni.physxzerogravity", ZeroGravityTransformModel(usd_context_name))

        self._logging_enabled = self._settings.get_as_bool(pxzerog.SETTINGS_LOGGING_ENABLED)

        self._simulation_mode = omni.timeline.get_timeline_interface().is_playing()
        self.enabled = self._settings.get_as_bool(ZeroGravitySettings.SETTINGS_CUSTOM_MANIPULATOR_ENABLED)

        self._settings = carb.settings.get_settings()
        self._settings_manipulator_enabled_sub = omni.kit.app.SettingChangeSubscription(
            ZeroGravitySettings.SETTINGS_CUSTOM_MANIPULATOR_ENABLED, self._settings_manipulator_enabled_setting_changed
        )

        self._op_listener = OpSettingsListener()
        self._op_listener_sub = self._op_listener.subscribe_listener(self._on_op_listener_changed)

    def __del__(self):
        self.destroy()

    def destroy(self):
        self._settings_manipulator_enabled_sub = None
        self._op_listener_sub = None
        if self._op_listener:
            self._op_listener.destroy()
            self._op_listener = None
        super().destroy()

    def _on_op_listener_changed(self, type: OpSettingsListener.CallbackType, value: str):
        # Scaling is not supported in ZeroG - disable it
        if type == OpSettingsListener.CallbackType.OP_CHANGED:
            if value == c.TRANSFORM_OP_SCALE:
                if self._logging_enabled:
                    carb.log_info("ZeroGravityTransformManipulator disabling manipulator: "
                                  "scaling is not supported in ZG")
                self.enabled = False
            else:
                self.enabled = True
                if self._logging_enabled:
                    carb.log_info("ZeroGravityTransformManipulator re-enabling manipulator")
                # This also retriggers an 'enabled = True', necessary to re-show a valid gizmo after enabling ZG with scaling on and switching to a valid gizmo
                refresh_manipulator_selector()

    def _settings_manipulator_enabled_setting_changed(self, item, event_type):
        if event_type == carb.settings.ChangeEventType.CHANGED:
            self.enabled = self._settings.get_as_bool(ZeroGravitySettings.SETTINGS_CUSTOM_MANIPULATOR_ENABLED)
            if self._logging_enabled:
                carb.log_info("ZeroGravityTransformManipulator custom manipulator setting"
                              f" changed, enabling manipulator: {self.enabled}")

    @PrimTransformManipulator.enabled.setter
    def enabled(self, value: bool):  # override
        # Do _not_ enable this manipulator if scaling is activated
        if (self._op_settings_listener is not None
                and self._op_settings_listener.selected_op == c.TRANSFORM_OP_SCALE):
            if self._logging_enabled:
                carb.log_info("ZeroGravityTransformManipulator custom manipulator setting"
                              f" changed, enabling manipulator: {self.enabled}")
            value = False
        self._manipulator.enabled = value

    def on_selection_changed(self, stage: Usd.Stage, selection: Union[List[Sdf.Path], None], *args, **kwargs) -> bool:  # override
        # Only placement prims (see is_placement_prim()) are supported by this manipulator.
        # A different manipulator will be selected by omni.kit.manipulator.selector if False is returned.

        assert isinstance(self.model, ZeroGravityTransformModel)

        if selection is None:
            self.model.on_selection_changed([])
            return False

        # Scaling is not supported in ZG so we 'cheat' here: we return True which means
        # 'I will handle this with my manipulator' but then we disable ourselves. The result
        # is: no other manipulator will try to handle it, and our ZG manipulator will be hidden.
        if (self._op_settings_listener is not None
                and self._op_settings_listener.selected_op == c.TRANSFORM_OP_SCALE):
            if self._logging_enabled:
                carb.log_info("ZeroGravityTransformManipulator disabling manipulator since"
                              " scaling is unsupported")
            self.enabled = False
            return True

        self.model.on_selection_changed(selection)

        for path in selection:
            prim = stage.GetPrimAtPath(path)
            if is_placement_prim(prim):
                if self._logging_enabled:
                    carb.log_info("ZeroGravityTransformManipulator found placement prims: "
                                  "enabling manipulator")
                self.enabled = True
                return True

        return False
