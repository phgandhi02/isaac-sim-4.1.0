from typing import List, Sequence

import carb.profiler
import omni.physxzerogravity.bindings._physXZeroGravity as pxzerog
import omni.timeline
from omni.kit.manipulator.prim.core.model import OpFlag, PrimTransformModel
from omni.kit.manipulator.transform.types import Operation
from omni.ui import scene as sc
from pxr import Sdf, Tf, Usd
from usdrt import Gf

from .payload import ZeroGravityPayload
from .utils import is_rigid_body_placement_prim


class ZeroGravityTransformModel(PrimTransformModel):
    DELTA_EPSILON = 0.01  # Don't even consider events with movements under this threshold for all components

    def __init__(self, usd_context_name: str = ""):
        self._settings = carb.settings.get_settings()
        self._logging_enabled = self._settings.get_as_bool(pxzerog.SETTINGS_LOGGING_ENABLED)

        self._settings_subs = []
        self._settings_subs.append(omni.kit.app.SettingChangeSubscription(
            pxzerog.SETTINGS_LOGGING_ENABLED,
            self._enable_logging_setting_changed
        ))

        super().__init__(usd_context_name)

    def __del__(self):
        self.destroy()

    def destroy(self):
        self._settings_subs = []
        self._settings = None

    def _enable_logging_setting_changed(self, item, event_type):
        if event_type == carb.settings.ChangeEventType.CHANGED:
            self._logging_enabled = self._settings.get_as_bool(pxzerog.SETTINGS_LOGGING_ENABLED)

    def on_began(self, payload):
        if self._logging_enabled:
            carb.log_info("ZeroGravityTransformModel.on_began")

        assert ZeroGravityPayload.placement is not None
        ZeroGravityPayload.placement.set_active(True)

        super().on_began(payload)

    def on_changed(self, payload):
        if self._logging_enabled:
            carb.log_info("ZeroGravityTransformModel.on_changed")

        super().on_changed(payload)

    def on_ended(self, payload):
        if self._logging_enabled:
            carb.log_info("ZeroGravityTransformModel.on_ended")

        assert ZeroGravityPayload.placement is not None
        ZeroGravityPayload.placement.set_active(False)

        super().on_ended(payload)

    def on_canceled(self, payload):
        if self._logging_enabled:
            carb.log_info("ZeroGravityTransformModel.on_canceled")

        assert ZeroGravityPayload.placement is not None
        ZeroGravityPayload.placement.set_active(False)

        super().on_canceled(payload)

    def on_selection_changed(self, selection: List[Sdf.Path]):
        if self._logging_enabled:
            carb.log_info("ZeroGravityTransformModel._on_selection_changed")

        assert ZeroGravityPayload.placement is not None

        self._xformable_prim_paths.clear()
        self._xformable_prim_paths_set.clear()
        self._xformable_prim_paths_prefix_set.clear()
        self._consolidated_xformable_prim_paths.clear()
        self._pivot_prim = None

        # do not show gizmo for certain prims (see should_show_gizmo)
        for sdf_path in selection:
            if ZeroGravityPayload.placement.should_show_gizmo(sdf_path):
                self._xformable_prim_paths.append(sdf_path)

        if self._xformable_prim_paths:
            # Make a sorted list so parents always appears before child
            self._xformable_prim_paths_sorted = self._xformable_prim_paths.copy()
            self._xformable_prim_paths_sorted.sort()

            # Find the most recently selected valid xformable prim as the pivot prim where the transform gizmo is located at.
            self._pivot_prim = self._data_accessor_selector.get_prim_at_path(self._xformable_prim_paths[-1])

            # Get least common prims ancestors.
            # We do this so that if one selected prim is a descendant of other selected prim, the descendant prim won't be
            # transformed twice.
            self._consolidated_xformable_prim_paths = Sdf.Path.RemoveDescendentPaths(self._xformable_prim_paths)

        self._xformable_prim_paths_set.update(self._xformable_prim_paths)
        for path in self._xformable_prim_paths_set:
            self._xformable_prim_paths_prefix_set.update(path.GetPrefixes())

        if self._update_transform_from_prims():
            self._item_changed(self._transform_item)

        # Happens when host widget is already enabled and first selection in a new stage
        if self._enabled_hosting_widget_count > 0 and self._stage_listener is None:
            self._stage_listener = self._data_accessor_selector.setup_update_callback()

    def set_floats(self, item: sc.AbstractManipulatorItem, value: Sequence[float]):
        if self._logging_enabled:
            carb.log_info(f"ZeroGravityTransformModel.set_floats({item}, {value})")

        assert ZeroGravityPayload.placement is not None

        if isinstance(item, omni.kit.manipulator.transform.model.AbstractTransformManipulatorModel.OperationItem):

            is_zero_delta = True
            for v in value:
                if abs(v) > ZeroGravityTransformModel.DELTA_EPSILON:
                    is_zero_delta = False
                    break

            if self._logging_enabled and is_zero_delta is True:
                carb.log_info(f"{value} identified as zero delta (<={ZeroGravityTransformModel.DELTA_EPSILON}).")

            if not is_zero_delta:
                if item.operation == Operation.TRANSLATE_DELTA:
                    ZeroGravityPayload.placement.set_translate_delta_xyz(*value)
                elif item.operation == Operation.ROTATE_DELTA:
                    ZeroGravityPayload.placement.set_rotate_delta_xyzw(*value)
                elif item.operation == Operation.SCALE_DELTA:
                    ZeroGravityPayload.placement.set_scale_delta_xyz(*value)

        super().set_floats(item, value)

    def _transform_selected_prims(
            self,
            new_manipulator_transform: Gf.Matrix4d,
            old_manipulator_transform_no_scale: Gf.Matrix4d,
            old_manipulator_scale: Gf.Vec3d,
            dirty_ops: OpFlag,
    ):
        if self._logging_enabled:
            carb.log_info("ZeroGravityTransformModel._transform_selected_prims")

        assert ZeroGravityPayload.placement is not None

        paths = []

        for path in self._consolidated_xformable_prim_paths:
            prim = self._data_accessor_selector.get_prim_at_path(path)
            if is_rigid_body_placement_prim(prim):
                continue
            paths.append(path)

        prev_paths = self._consolidated_xformable_prim_paths
        self._consolidated_xformable_prim_paths = paths

        super()._transform_selected_prims(
            new_manipulator_transform,
            old_manipulator_transform_no_scale,
            old_manipulator_scale,
            dirty_ops
        )

        self._consolidated_xformable_prim_paths = prev_paths
