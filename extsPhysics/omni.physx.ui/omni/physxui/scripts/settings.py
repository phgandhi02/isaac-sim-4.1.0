import omni.usd
from omni import ui
from omni.physx import get_physx_interface, get_physx_cooking_interface
from omni.physxcooking import get_physx_cooking_service_private_interface
import carb
import carb.settings
import carb.profiler
from omni.kit.window.preferences.scripts.preferences_window import PreferenceBuilder
from omni.kit.widget.settings import SettingsWidgetBuilder
from omni.kit.widget.settings.settings_widget import SettingType, create_setting_widget, create_setting_widget_combo
import omni.physx.bindings._physx as physx_bindings
import omni.physxui.scripts.utils as utils
from omni.physxui.scripts.utils import BackwardCompatMode
from omni.physx.scripts import utils as physx_utils
from functools import partial
from omni.physx.scripts.pythonUtils import ScopeGuard
import asyncio

BUTTON_HEIGHT = 30


def get_no_stage_err_msg(path):
    return f"You are setting a per-stage physics settings {path} without an active stage. This setting will be reset when a new stage is loaded!"


def build_section(name, build_func):
    with ui.CollapsableFrame(name, height=0):
        with ui.HStack():
            ui.Spacer(width=20)
            with ui.VStack():
                build_func()

def ujitso_can_use_omni_hub():
    omni_hub_supported = carb.settings.get_settings().get_as_bool(physx_bindings.SETTING_OMNI_HUB_SUPPORTED)
    local_mesh_cache_enabled = carb.settings.get_settings().get_as_bool(physx_bindings.SETTING_USE_LOCAL_MESH_CACHE)
    ujitso_cooking_enabled = carb.settings.get_settings().get_as_bool(physx_bindings.SETTING_UJITSO_COLLISION_COOKING)

    can_use_omni_hub = local_mesh_cache_enabled and ujitso_cooking_enabled and omni_hub_supported

    reason_text = None
    if not omni_hub_supported:
        reason_text = "OmniHub Not Available"
    elif not local_mesh_cache_enabled:
        reason_text = "Local Cache Not Enabled"
    elif not ujitso_cooking_enabled:
        reason_text = "Ujitso Cooking Not Enabled"

    return can_use_omni_hub, reason_text

class PhysicsPreferences(PreferenceBuilder):
    def __init__(self):
        super().__init__("Physics")
        self._widgets = {}
        self._line_height = 20
        self._developer_mode = carb.settings.get_settings().get("physics/developmentMode")

    def on_shutdown(self):
        self._widgets = {}

    def _add_setting_combo_and_label(self, name, path, items, **kwargs):
        with ui.HStack(height=27):
            self.label(name)
            create_setting_widget_combo(path, items, **kwargs)

    # def _add_setting_checkbox_and_label(self, name, path, **kwargs):
    #     with ui.HStack(height=self._line_height):
    #         self._widgets.append(create_setting_widget(path, SettingType.BOOL), **kwargs)
    #         # overpowering ui.Line from create_setting_widget to stay at ~zero width
    #         ui.Label(name, width=ui.Fraction(100))

    def _add_setting(self, setting_type, name, path, range_from=0, range_to=0, speed=1, **kwargs):
        widget = self.create_setting_widget(name, path, setting_type, range_from=range_from, range_to=range_to, speed=speed, **kwargs)
        self._widgets[path] = widget

        if setting_type in [SettingType.INT, SettingType.FLOAT]:
            def value_changed(model):
                val = model.as_int if setting_type == SettingType.INT else model.as_float
                if val < range_from:
                    model.set_value(range_from)
                elif val > range_to:
                    model.set_value(range_to)

            widget.model.add_value_changed_fn(value_changed)

        return widget

    def _add_release_cooked_data(self):
        button = ui.Button("Release Physics Cooked Data", height=BUTTON_HEIGHT, width=300)

        def on_clicked():
            stage = omni.usd.get_context().get_stage()
            physx_utils.release_cooked_data(stage)

        button.set_clicked_fn(on_clicked)

    def _add_run_backwards_compatibility(self):
        button = ui.Button("Run Backwards Compatibility On Current Stage", height=BUTTON_HEIGHT, width=300)

        def on_click():
            carb.log_warn("Running backward compatibility.")
            res = get_physx_interface().check_backwards_compatibility()
            if res:
                check_log = get_physx_interface().get_backwards_compatibility_check_log()
                carb.log_warn(f"{check_log}")
                get_physx_interface().run_backwards_compatibility()
            else:
                carb.log_warn("No deprecated schema found.")

        button.set_clicked_fn(on_click)

    def _add_run_backwards_compatibility_on_folder(self):
        button = ui.Button("Run Backwards Compatibility On Folder", height=BUTTON_HEIGHT, width=300)

        def on_click():
            utils.run_backward_compat_on_folder()

        button.set_clicked_fn(on_click)

    def _build_simulator_ui(self):
        self._line_height = 23
        self._add_setting(SettingType.INT, "Num Simulation Threads", physx_bindings.SETTING_NUM_THREADS, 0, 256)
        self._add_setting(SettingType.INT, "Stop simulation after number of (PhysX) errors", physx_bindings.SETTING_MAX_NUMBER_OF_PHYSX_ERRORS, 1, 4096)
        self._add_setting(SettingType.BOOL, "Use PhysX CPU Dispatcher", physx_bindings.SETTING_PHYSX_DISPATCHER)
        self._add_setting(SettingType.BOOL, "Expose PhysX SDK Profiler Data", physx_bindings.SETTING_EXPOSE_PROFILER_DATA)
        self._add_setting(SettingType.BOOL, "Expose USD Prim Names toPhysX SDK Names", physx_bindings.SETTING_EXPOSE_PRIM_PATH_NAMES)
        self._add_setting(SettingType.BOOL, "Simulate empty scene", physx_bindings.SETTING_SIMULATE_EMPTY_SCENE)
        self._add_setting(SettingType.BOOL, "Save Cooked data to USD", physx_bindings.SETTING_SAVE_COOKED_DATA)
        self._add_release_cooked_data()

    def _add_release_local_mesh_cache(self):
        button = ui.Button("Release Local Mesh Cache", height=BUTTON_HEIGHT, width=300)

        def on_click():
            get_physx_cooking_interface().release_local_mesh_cache()
        button.set_clicked_fn(on_click)

    def _omni_hub_ui_refresh(self, *args):
        can_use_omni_hub, reason_text = ujitso_can_use_omni_hub()
        omni_hub_widget = self._widgets.get(physx_bindings.SETTING_UJITSO_REQUEST_OMNI_HUB)
        if omni_hub_widget is not None:
            omni_hub_widget.enabled = can_use_omni_hub
            omni_hub_widget_label = self._widgets.get(physx_bindings.SETTING_UJITSO_REQUEST_OMNI_HUB + ":label")
            if omni_hub_widget_label is not None:
                omni_hub_widget_label_label_text = "Enable Ujitso OmniHub Cache"
                if not can_use_omni_hub and reason_text is not None:
                    omni_hub_widget_label_label_text += " (DISABLED: {})".format(reason_text)
                omni_hub_widget_label.text = omni_hub_widget_label_label_text

        omni_hub_requested = carb.settings.get_settings().get_as_bool(physx_bindings.SETTING_UJITSO_REQUEST_OMNI_HUB)
        use_omni_hub = can_use_omni_hub and omni_hub_requested
        carb.settings.get_settings().set_bool(physx_bindings.SETTING_UJITSO_USE_OMNI_HUB, use_omni_hub)

    def _add_omni_hub_ui(self):
        with ui.HStack(height=24):
            self._widgets[physx_bindings.SETTING_UJITSO_REQUEST_OMNI_HUB + ":label"] = ui.Label("", word_wrap=True, name="title", width=ui.Percent(50))
            widget, _ = create_setting_widget(physx_bindings.SETTING_UJITSO_REQUEST_OMNI_HUB, SettingType.BOOL)
            self._widgets[physx_bindings.SETTING_UJITSO_REQUEST_OMNI_HUB] = widget
            widget.model.add_value_changed_fn(self._omni_hub_ui_refresh)
            self._omni_hub_ui_refresh()

    def _build_local_mesh_cache_ui(self):
        self._line_height = 23
        enable_local_mesh_cache_widget = self._add_setting(SettingType.BOOL, "Enable Local Mesh Cache (Takes Effect on Sim Start)", physx_bindings.SETTING_USE_LOCAL_MESH_CACHE)
        enable_local_mesh_cache_widget.model.add_value_changed_fn(self._omni_hub_ui_refresh)
        self._add_setting(SettingType.INT, "Local Mesh Cache Size MB (Takes Effect on Sim Start)", physx_bindings.SETTING_LOCAL_MESH_CACHE_SIZE_MB, 16, 4096)
        enable_ujitso_cooking_widget = self._add_setting(SettingType.BOOL, "Enable Ujitso Collision Cooking", physx_bindings.SETTING_UJITSO_COLLISION_COOKING)
        self.label("NOTE: when Ujitso cooking is enabled, local cache resize, local cache release, and remote cache enabling only take place on application restart.")
        self._add_setting(SettingType.INT, "Ujitso Cooking Max Process Count", physx_bindings.SETTING_UJITSO_COOKING_MAX_PROCESS_COUNT, 1, 128)
        enable_ujitso_cooking_widget.model.add_value_changed_fn(self._omni_hub_ui_refresh)
        self._add_omni_hub_ui()
        if get_physx_cooking_service_private_interface().is_OVC_node():
            self._add_setting(SettingType.BOOL, "Enable Ujitso Remote Mesh Cache", physx_bindings.SETTING_UJITSO_REMOTE_CACHE)
        self._add_release_local_mesh_cache()

    def _build_general_settings(self):
        self._add_setting(SettingType.BOOL, "Reset Simulation on Stop", physx_bindings.SETTING_RESET_ON_STOP)
        self._add_setting(SettingType.BOOL, "Use Active CUDA Context (Requires Restart)", physx_bindings.SETTING_USE_ACTIVE_CUDA_CONTEXT)
        self._add_setting_combo_and_label("Use Physics Scene Multi-GPU Mode", physx_bindings.SETTING_PHYSICS_SCENE_MULTIGPU_MODE, {"Disabled": 0, "Use All Devices": 1, "Use All Devices Except First": 2})
        self._add_setting(SettingType.INT, "Add Menu Selection Prim Limit", physx_bindings.SETTING_ADDMENU_SELECTION_LIMIT, 0, 4294967295)
        self._add_setting(SettingType.INT, "Add Menu Subtree Prim Limit", physx_bindings.SETTING_ADDMENU_SUBTREE_LIMIT, 0, 4294967295)
        self._add_setting_combo_and_label(
            "Default Physics Simulator",
            physx_bindings.SETTING_DEFAULT_SIMULATOR,
            {
                "None": "None",
                "PhysX": "PhysX",
            },
        )

        def clicked(*_):
            get_physx_interface().reset_settings_in_preferences()

        ui.Button("Reset Physics Preferences", height=BUTTON_HEIGHT, width=300, clicked_fn=clicked)

    def _build_backward_compat_setting(self):
        self._add_setting_combo_and_label(
            "Backward Compatibility Check on Scene Open",
            physx_bindings.SETTING_BACKWARD_COMPATIBILITY,
            {
                "Disabled": BackwardCompatMode.DISABLED,
                "Warn": BackwardCompatMode.WARN,
                "Ask to Upgrade": BackwardCompatMode.ASK,
                "Auto-Upgrade": BackwardCompatMode.AUTO,
            },
        )

        ui.Spacer(height=3)
        self._add_run_backwards_compatibility()
        ui.Spacer(height=3)
        self._add_run_backwards_compatibility_on_folder()

    def build(self):
        with ui.VStack(height=0):
            build_section("General", self._build_general_settings)
            ui.Spacer(height=5)
            build_section("Backward Compatibility", self._build_backward_compat_setting)
            ui.Spacer(height=5)
            build_section("Simulator", self._build_simulator_ui)
            ui.Spacer(height=5)
            build_section("Local Mesh Cache", self._build_local_mesh_cache_ui)


class PhysicsSettings(ui.Window):
    def __init__(self):
        super().__init__(PhysicsSettings.get_window_name(), dockPreference=ui.DockPreference.RIGHT_TOP)
        self.deferred_dock_in("Stage", ui.DockPolicy.DO_NOTHING)

        @carb.profiler.profile
        def on_stage_event(event):
            if event.type == int(omni.usd.StageEventType.OPENED):
                self._refresh_from_usd()

        self._stage_event_sub = omni.usd.get_context().get_stage_event_stream().create_subscription_to_pop(on_stage_event)
        self._write_guard = ScopeGuard()
        self._buttons = {}
        self._build()

    @staticmethod
    def get_window_name() -> str:
        return "Physics Stage Settings"

    def on_shutdown(self):
        self._buttons = {}
        self._stage_event_sub = None

    def _build(self):
        from omni.kit.widget.settings import get_style
        self.frame.set_style(get_style())

        with self.frame:
            with ui.ScrollingFrame(
                horizontal_scrollbar_policy=ui.ScrollBarPolicy.SCROLLBAR_ALWAYS_OFF,
                vertical_scrollbar_policy=ui.ScrollBarPolicy.SCROLLBAR_AS_NEEDED,
            ):
                with ui.VStack(spacing=5):
                    build_section("Update", self._build_update_ui)
                    ui.Spacer(height=5)
                    build_section("Simulator", self._build_simulator_ui)
                    ui.Spacer(height=5)
                    build_section("Collision", self._build_collision_ui)
                    ui.Spacer(height=5)
                    build_section("Mouse Interaction", self._build_mouse_interaction_ui)
                    ui.Spacer(height=5)
                    build_section("Zero Gravity", self._build_zero_gravity_ui)
                    ui.Spacer(height=5)
                    build_section("Debug Visualization", self._build_debug_vis_ui)

    def _is_default(self, path, curr_val):
        def_val = carb.settings.get_settings().get("/defaults" + path)
        return def_val == curr_val

    def _refresh_from_usd(self):
        stage = omni.usd.get_context().get_stage()
        physics_settings = stage.GetRootLayer().customLayerData.get("physicsSettings")

        with self._write_guard:
            get_physx_interface().reset_settings_in_stage()

        for btn in self._buttons.values():
            btn.visible = False

        if physics_settings:
            for path, value in physics_settings.items():
                carb.settings.get_settings().set(path, value)
                if path in self._buttons:
                    self._buttons[path].visible = True

    def _write_to_usd(self, path, _):
        if self._write_guard.is_guarded():
            return

        # FIXME: this can be called during validation through an async thread
        # don't touch USD if there's no event loop available, callbacks use it
        try:
            asyncio.get_running_loop()
        except RuntimeError:
            return

        stage = omni.usd.get_context().get_stage()
        if not stage:
            carb.log_error(get_no_stage_err_msg(path))
            return

        val = carb.settings.get_settings().get(path)

        # changing it to default, delete it from usd instead (if present)
        if self._is_default(path, val):
            self._delete_from_usd(path)
            return

        custom_layer_data = stage.GetRootLayer().customLayerData

        if "physicsSettings" not in custom_layer_data:
            custom_layer_data["physicsSettings"] = {}

        custom_layer_data["physicsSettings"][path] = val

        btn = self._buttons.get(path)
        if btn:
            btn.visible = True

        stage.GetRootLayer().customLayerData = custom_layer_data

    def _delete_from_usd(self, path):
        stage = omni.usd.get_context().get_stage()
        custom_layer_data = stage.GetRootLayer().customLayerData

        if "physicsSettings" in custom_layer_data:
            custom_layer_data["physicsSettings"].pop(path, None)

        btn = self._buttons.get(path)
        if btn:
            btn.visible = False

        stage.GetRootLayer().customLayerData = custom_layer_data

    def _build_update_ui(self):
        self._add_setting(SettingType.BOOL, "Update to USD", physx_bindings.SETTING_UPDATE_TO_USD)
        self._add_setting(SettingType.BOOL, "Update Velocities to USD", physx_bindings.SETTING_UPDATE_VELOCITIES_TO_USD)
        self._add_setting(SettingType.BOOL, "Output Velocities in Local Space", physx_bindings.SETTING_OUTPUT_VELOCITIES_LOCAL_SPACE)
        self._add_setting(SettingType.BOOL, "Update Particles to USD", physx_bindings.SETTING_UPDATE_PARTICLES_TO_USD)
        self._add_setting(SettingType.BOOL, "Update Residuals to USD", physx_bindings.SETTING_UPDATE_RESIDUALS_TO_USD)

    def _build_collision_ui(self):
        self._add_setting(SettingType.BOOL, "Cones As Custom Geometry", physx_bindings.SETTING_COLLISION_CONE_CUSTOM_GEOMETRY)
        self._add_setting(SettingType.BOOL, "Cylinders As Custom Geometry", physx_bindings.SETTING_COLLISION_CYLINDER_CUSTOM_GEOMETRY)

    def _build_simulator_ui(self):
        self._add_setting(SettingType.INT, "Min Simulation Frame Rate", physx_bindings.SETTING_MIN_FRAME_RATE, 1, 1000)
        self._add_setting(SettingType.FLOAT, "Joint Body Transform Check Tolerance", physx_bindings.SETTING_JOINT_BODY_TRANSFORM_CHECK_TOLERANCE, 0.0, 10000000.0)

    def _build_mouse_interaction_ui(self):
        self._add_setting(SettingType.BOOL, "Mouse Interaction Enabled", physx_bindings.SETTING_MOUSE_INTERACTION_ENABLED)
        self._add_setting(SettingType.BOOL, "Mouse Grab", physx_bindings.SETTING_MOUSE_GRAB)
        self._add_setting(SettingType.BOOL, "Mouse Grab Ignore Invisible", physx_bindings.SETTING_MOUSE_GRAB_IGNORE_INVISBLE)
        self._add_setting(SettingType.BOOL, "Mouse Grab With Force", physx_bindings.SETTING_MOUSE_GRAB_WITH_FORCE)
        self._add_setting(SettingType.FLOAT, "Mouse Grab Force Coeff", physx_bindings.SETTING_MOUSE_PICKING_FORCE, 0.0, 10.0)
        self._add_setting(SettingType.FLOAT, "Mouse Push Acceleration", physx_bindings.SETTING_MOUSE_PUSH, 0.0, 1000.0)

    def _build_zero_gravity_ui(self):
        self._add_setting(SettingType.FLOAT, "Zero Gravity Speed", physx_bindings.SETTING_ZERO_GRAVITY_SPEED, 1.0, 10.0)

    def _build_debug_vis_ui(self):
        self._add_setting(SettingType.FLOAT, "Simplify Debug Visualization at Distance", physx_bindings.SETTING_DEBUG_VIS_SIMPLIFY_AT_DISTANCE, 1.0, 10000.0)
        self._add_setting(SettingType.BOOL, "Query USDRT For Stage Traversal", physx_bindings.SETTING_DEBUG_VIS_QUERY_USDRT_FOR_TRAVERSAL)

    def _reset_setting(self, path):
        self._delete_from_usd(path)
        self._refresh_from_usd()

    def _add_setting(self, setting_type, name, path, range_from=0, range_to=0, speed=1, tooltip=""):
        with ui.HStack(skip_draw_when_clipped=True):
            SettingsWidgetBuilder._create_label(name, path, tooltip, additional_label_kwargs={"width": 220})
            widget, model = create_setting_widget(path, setting_type, range_from, range_to, speed)
            model.add_value_changed_fn(partial(self._write_to_usd, path))
            btn = SettingsWidgetBuilder._build_reset_button(path)
            btn.set_mouse_pressed_fn(lambda *_, path=path: self._reset_setting(path))
            btn.set_tooltip("Click to clear from USD")
            self._buttons[path] = btn
        ui.Spacer(height=3)


def load_physics_settings_from_stage(stage):
    physics_settings = stage.GetRootLayer().customLayerData.get("physicsSettings")
    get_physx_interface().reset_settings_in_stage()
    if physics_settings:
        for path, value in physics_settings.items():
            carb.settings.get_settings().set(path, value)
