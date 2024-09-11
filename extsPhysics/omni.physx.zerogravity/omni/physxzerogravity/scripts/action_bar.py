import asyncio
import os
from typing import Any, Dict

import carb
import carb.dictionary
import carb.events
import carb.settings
import omni.kit.commands
import omni.kit.viewport.utility
import omni.physx
import omni.physxzerogravity.bindings._physXZeroGravity as pxzerog
import omni.ui as ui
import omni.usd
from pxr import UsdPhysics

from .constants import ColliderTypes, ZeroGravityConstants
from .menu_helpers import MenuHelpers
from .settings import ZeroGravitySettings
from .settings_menu import DynamicCollSettingsMenu, StaticCollSettingsMenu, ZeroGravitySettingsMenu
from .styles import Styles
from .utils import is_placement_prim, refresh_manipulator_selector, ui_wait


class ButtonDef:
    # button identification (keys to dict)
    BUTTON_CLEAR = "clear"
    BUTTON_STATIC = "static"
    BUTTON_STATIC2 = "static2"
    BUTTON_DYNAMIC = "dynamic"
    BUTTON_DYNAMIC2 = "dynamic2"
    BUTTON_DROP_START = "drop_start"  # always use this as a key into self._buttons dict, never the 'STOP' version
    BUTTON_DROP_STOP = "drop_stop"
    BUTTON_SWEEP_START = "sweep_start"  # always use this as a key into self._buttons dict, never the 'STOP' version
    BUTTON_SWEEP_STOP = "sweep_stop"
    BUTTON_SETTINGS = "settings"

    @classmethod
    def define_buttons(cls, button_defs_dict: Dict[str, Any], icon_folder: str):
        """Define toolbar buttons"""
        def def_button(
            key: str,
            icon_name: str,
            tooltip: str,
            width: int = Styles.BUTTON_SIZE_H,
            height: int = Styles.BUTTON_SIZE_V,
            style: str = Styles.BUTTON_STYLE
        ):
            """helps to define a toolbar button, adds it to button_defs_dict"""
            button_defs_dict[key] = [os.path.join(icon_folder, icon_name), tooltip, width, height, style]

        def_button(cls.BUTTON_CLEAR, "marker_clear.svg", "Clear Markers")
        def_button(cls.BUTTON_STATIC, "marker_static_1_menu.svg", "Mark Static")
        def_button(cls.BUTTON_STATIC2, "marker_static_2_menu.svg", "Mark Static")
        def_button(cls.BUTTON_DYNAMIC, "marker_dynamic_1_menu.svg", "Mark Dynamic")
        def_button(cls.BUTTON_DYNAMIC2, "marker_dynamic_2_menu.svg", "Mark Dynamic")
        def_button(cls.BUTTON_DROP_START, "drop_start.svg", "Start Dropping Selected Assets (<END> key)")
        def_button(cls.BUTTON_DROP_STOP, "drop_stop.svg", "Stop Dropping Selected Assets (<END> key)")
        def_button(cls.BUTTON_SWEEP_START, "sweep_start.svg", "Start Sweep Mode")
        def_button(cls.BUTTON_SWEEP_STOP, "sweep_stop.svg", "Stop Sweep Mode")
        def_button(cls.BUTTON_SETTINGS, "settings_menu.svg", "Settings")


class ActionBar:
    def __init__(self, icon_folder, zerog_iface):
        self._toolbar_name = ZeroGravityConstants.EXTENSION_DISPLAY_NAME

        self._physx_authoring = zerog_iface
        self._action_bar_visibility_changed_fn = None

        self._buttons_def = dict()
        ButtonDef.define_buttons(self._buttons_def, icon_folder)
        self._buttons = dict()  # this dictionary holds button instances. Key corresponds to the _buttons_def key

        self._settings_subs = []  # subscribed settings subs.
        self._settings = carb.settings.get_settings()
        self._selection = omni.usd.get_context().get_selection()

        self._window = None  # the toolbar
        self._enabled = None
        self._settings_menu = ZeroGravitySettingsMenu()
        self._static_coll_settings_menu = StaticCollSettingsMenu()
        self._dynamic_coll_settings_menu = DynamicCollSettingsMenu()
        self._dropping = False
        self._sweep_mode = True  # Sweep mode is enabled by default

        self._settings_subs.append(omni.kit.app.SettingChangeSubscription(
            ZeroGravitySettings.SETTINGS_STATIC_COLLIDER_SIMPLIFICATION_TYPE, self._static_coll_simpl_type_setting_changed
        ))

        self._settings_subs.append(omni.kit.app.SettingChangeSubscription(
            ZeroGravitySettings.SETTINGS_DYNAMIC_COLLIDER_SIMPLIFICATION_TYPE, self._dynamic_coll_simpl_type_setting_changed
        ))

        self._stage_event_sub = omni.usd.get_context().get_stage_event_stream().create_subscription_to_pop(
            self._on_stage_event, name="omni.physxzerogravity.ActionBar._on_stage_event"
        )

        self._settings.set_int(ZeroGravitySettings.SETTINGS_PLACEMENT_MODE_MARK_DEFAULT, 0)
        self._settings.set_int(ZeroGravitySettings.SETTINGS_PLACEMENT_MODE_RESTORE_ALL_TRANSFORMS, 0)
        self._settings.set_int(ZeroGravitySettings.SETTINGS_PLACEMENT_MODE_MARK_CLEAR, 0)
        self._settings.set_int(ZeroGravitySettings.SETTINGS_PLACEMENT_MODE_MARK_CONVEX_DECOMPOSITION, 0)
        self._settings.set_int(ZeroGravitySettings.SETTINGS_PLACEMENT_MODE_MARK_MESH_SIMPLIFICATION, 0)
        self._settings.set_int(ZeroGravitySettings.SETTINGS_PLACEMENT_MODE_MARK_CONVEX_HULL, 0)
        self._settings.set_int(ZeroGravitySettings.SETTINGS_PLACEMENT_MODE_MARK_TRIANGLE_MESH, 0)

    def clean(self):
        """Cleanup. ActionBar is being freed."""
        self._buttons_def = None
        self._buttons = None

        if self._static_coll_settings_menu:
            self._static_coll_settings_menu.hide()
            self._static_coll_settings_menu = None
        if self._dynamic_coll_settings_menu:
            self._dynamic_coll_settings_menu.hide()
            self._dynamic_coll_settings_menu = None
        if self._settings_menu:
            self._settings_menu.hide()
            self._settings_menu = None

        self._action_bar_visibility_changed_fn = None

        if self._window is not None:
            self._window.undock()
            self._window.visible = False
            self._window.destroy()
            self._window = None

        self._settings_subs = []
        self._stage_event_sub = None
        self._selection = None
        self._settings = None
        self._physx_authoring = None

    def _has_selected_placement_prims(self):
        selected_paths = self._selection.get_selected_prim_paths()
        stage = omni.usd.get_context().get_stage()
        for path in selected_paths:
            prim = stage.GetPrimAtPath(path)
            if is_placement_prim(prim):
                return True
        return False

    def _create_toolbar(self, visibility=True):
        def create_button(button, **kwargs):
            return ui.Button(
                '',
                image_url=button[0],
                tooltip=button[1],
                image_width=button[2],
                image_height=button[3],
                width=button[2],
                height=button[3],
                style=button[4],
                **kwargs
            )

        def create_separator(axis, sep_style):
            if axis == ui.ToolBarAxis.X:
                ui.Line(height=Styles.BUTTON_SIZE_V + 4, width=2, alignment=ui.Alignment.H_CENTER, style=sep_style)
            else:
                ui.Line(width=Styles.BUTTON_SIZE_H + 4, height=2, alignment=ui.Alignment.V_CENTER, style=sep_style)

        def create_buttons(axis):
            """Creates toolbar buttons and saves their instances into sel"""
            stack_style = {"Button.Image:disabled": {"color": Styles.DISABLED_COLOR}}
            sep_style = {"color": Styles.DISABLED_COLOR}

            with self._window.frame:
                with ui.VStack():
                    bk = None  # tmp var holding button dict key
                    if axis == ui.ToolBarAxis.X:
                        stack = ui.HStack(style=stack_style)
                    else:
                        stack = ui.VStack(style=stack_style)
                    with stack:
                        bk = ButtonDef.BUTTON_CLEAR  # clear marker
                        self._buttons[bk] = create_button(
                            self._buttons_def[bk],
                            clicked_fn=lambda: omni.kit.commands.execute("ZeroGravityClearSelectedCommand")
                        )
                        bk = ButtonDef.BUTTON_STATIC  # static marker
                        self._buttons[bk] = create_button(
                            self._buttons_def[bk],
                            clicked_fn=lambda: omni.kit.commands.execute("ZeroGravitySetSelectedStaticCommand"),
                            mouse_pressed_fn=self._on_static_button_mouse_pressed,
                            mouse_released_fn=self._on_static_button_mouse_released
                        )
                        bk = ButtonDef.BUTTON_DYNAMIC  # dynamic marker
                        self._buttons[bk] = create_button(
                            self._buttons_def[bk],
                            clicked_fn=lambda: omni.kit.commands.execute("ZeroGravitySetSelectedDynamicCommand"),
                            mouse_pressed_fn=self._on_dynamic_button_mouse_pressed,
                            mouse_released_fn=self._on_dynamic_button_mouse_released
                        )
                        create_separator(axis, sep_style)  # separator
                        bk = ButtonDef.BUTTON_DROP_START if not self._dropping else ButtonDef.BUTTON_DROP_STOP  # drop objects
                        self._buttons[ButtonDef.BUTTON_DROP_START] = create_button(
                            self._buttons_def[bk],
                            clicked_fn=self._on_dropping_button_clicked,
                        )
                        bk = ButtonDef.BUTTON_SWEEP_START if not self._sweep_mode else ButtonDef.BUTTON_SWEEP_STOP  # sweep mode
                        self._buttons[ButtonDef.BUTTON_SWEEP_START] = create_button(
                            self._buttons_def[bk],
                            clicked_fn=self._on_sweep_mode_button_clicked,
                        )
                        bk = ButtonDef.BUTTON_SETTINGS  # settings menu
                        self._buttons[bk] = create_button(
                            self._buttons_def[bk],
                            mouse_pressed_fn=lambda x, y, button, modifiers: self._settings_menu.show()
                        )
            self._enabled = self._has_selected_placement_prims()
            self._update_action_bar_ui()

        async def dock(visible):
            await ui_wait()  # we have to wait here 10 frames otherwise it won't dock in Create (as it applies fixed layout)
            vp = omni.kit.viewport.utility.get_active_viewport_window()
            if vp and self._window:
                viewport_window = ui.Workspace.get_window(vp.title)
                if viewport_window:
                    tab_visible = viewport_window.dock_tab_bar_visible
                    self._window.dock_in(viewport_window, ui.DockPosition.LEFT)
                    viewport_window.dock_tab_bar_visible = tab_visible
            if self._window:
                self._window.visible = visible

        def on_visiblity_changed(visible):
            if self._action_bar_visibility_changed_fn:
                self._action_bar_visibility_changed_fn(visible)

        self._window = ui.ToolBar(
            self._toolbar_name,
            axis_changed_fn=create_buttons,  # trigger toolbar creation on axis change
            visibility_changed_fn=on_visiblity_changed
        )

        create_buttons(ui.ToolBarAxis.X)

        if not self._window.docked:
            asyncio.ensure_future(dock(visibility))

    # Returns true if this prim has any collision related APIs on it. Note: do NOT use PhysX specific schemas here,
    # only USD ones
    def _prim_has_collision(self, prim):
        if prim.HasAPI(UsdPhysics.RigidBodyAPI) or prim.HasAPI(UsdPhysics.CollisionAPI) or prim.HasAPI(UsdPhysics.MeshCollisionAPI):
            return True
        return False

    def _static_coll_simpl_type_setting_changed(self, item, event_type):
        if event_type == carb.settings.ChangeEventType.CHANGED:
            self._update_action_bar_ui()

    def _dynamic_coll_simpl_type_setting_changed(self, item, event_type):
        if event_type == carb.settings.ChangeEventType.CHANGED:
            self._update_action_bar_ui()

    def _update_action_bar_ui(self):
        def update_button(button_key, from_button_def_key, custom_tooltip=None):
            button = self._buttons[button_key]
            button_def = self._buttons_def[from_button_def_key]
            button.enabled = self._enabled
            if button.image_url != button_def[0]:
                button.image_url = button_def[0]
            if not custom_tooltip:
                if button.tooltip != button_def[1]:
                    button.tooltip = button_def[1]
            else:
                if button.tooltip != custom_tooltip:
                    button.tooltip = custom_tooltip

        b_def_key = None

        static_coll_approx = self._settings.get_as_int(ZeroGravitySettings.SETTINGS_STATIC_COLLIDER_SIMPLIFICATION_TYPE)
        static_coll_approx_str = MenuHelpers._static_coll_approximations[static_coll_approx]

        if static_coll_approx == ColliderTypes.STATIC_COLLIDER_SIMPLIFICATION_TYPE_TRIANGLE_MESH:
            b_def_key = ButtonDef.BUTTON_STATIC2
        elif static_coll_approx == ColliderTypes.STATIC_COLLIDER_SIMPLIFICATION_TYPE_MESH_SIMPLIFICATION:
            b_def_key = ButtonDef.BUTTON_STATIC

        tooltip = f'{self._buttons_def[b_def_key][1]} ({static_coll_approx_str})'
        update_button(ButtonDef.BUTTON_STATIC, b_def_key, tooltip)

        dynamic_coll_approx = self._settings.get_as_int(ZeroGravitySettings.SETTINGS_DYNAMIC_COLLIDER_SIMPLIFICATION_TYPE)
        dynamic_coll_approx_str = MenuHelpers._dynamic_coll_approximations[dynamic_coll_approx]

        if dynamic_coll_approx == ColliderTypes.DYNAMIC_COLLIDER_SIMPLIFICATION_TYPE_CONVEX_HULL:
            b_def_key = ButtonDef.BUTTON_DYNAMIC
        elif dynamic_coll_approx == ColliderTypes.DYNAMIC_COLLIDER_SIMPLIFICATION_TYPE_CONVEX_DECOMPOSITION:
            b_def_key = ButtonDef.BUTTON_DYNAMIC2

        tooltip = f'{self._buttons_def[b_def_key][1]} ({dynamic_coll_approx_str})'
        update_button(ButtonDef.BUTTON_DYNAMIC, b_def_key, tooltip)

        self._buttons[ButtonDef.BUTTON_CLEAR].enabled = self._enabled

    def show(self):
        if self._window is None:
            self._create_toolbar()
        self._window.visible = True

    def hide(self):
        if self._window is not None:
            self._window.undock()
            self._window.visible = False
            self._window.destroy()
            self._window = None
            self.set_dropping(False)

    def _on_stage_event(self, evt):
        if evt.type == int(omni.usd.StageEventType.SELECTION_CHANGED):
            if self._window is not None:
                self._enabled = self._has_selected_placement_prims()
                self._update_action_bar_ui()

    def _on_static_button_mouse_pressed(self, x, y, button, modifiers):
        if button == 0 or button == 1:
            self._static_coll_settings_menu.show_popup(x, y, button == 0)

    def _on_static_button_mouse_released(self, x, y, button, modifiers):
        if button == 0:
            self._static_coll_settings_menu.cancel_popup_task()

    def _on_dynamic_button_mouse_pressed(self, x, y, button, modifiers):
        if button == 0 or button == 1:
            self._dynamic_coll_settings_menu.show_popup(x, y, button == 0)

    def _on_dynamic_button_mouse_released(self, x, y, button, modifiers):
        if button == 0:
            self._dynamic_coll_settings_menu.cancel_popup_task()

    def on_clear_button_clicked(self):
        count = self._settings.get(ZeroGravitySettings.SETTINGS_PLACEMENT_MODE_MARK_CLEAR) + 1
        self._settings.set_int(ZeroGravitySettings.SETTINGS_PLACEMENT_MODE_MARK_CLEAR, count)

    def _on_dropping_button_clicked(self):
        self._dropping = not self._dropping
        omni.kit.commands.execute("ZeroGravitySetDroppingCommand", dropping=self._dropping)

    def _on_sweep_mode_button_clicked(self):
        self._sweep_mode = not self._sweep_mode
        omni.kit.commands.execute("ZeroGravitySetSweepModeCommand", sweep_mode=self._sweep_mode)

    def set_swept_items_dynamic(self, use_dynamic_markers_for_swept_items):
        self._settings.set_bool(pxzerog.SETTINGS_PLACEMENT_MODE_SET_SWEPT_ITEMS_DYNAMIC, use_dynamic_markers_for_swept_items)
        self._physx_authoring.set_swept_items_dynamic(use_dynamic_markers_for_swept_items)

    def sweep_area_visualize(self, visualize_aabb):
        self._settings.set_bool(pxzerog.SETTINGS_PLACEMENT_MODE_SWEEP_AREA_VISUALIZE, visualize_aabb)
        self._physx_authoring.sweep_area_visualize(visualize_aabb)

    def zerog_simready_dnd_integration_enable(self, activate_on_drop):
        self._settings.set_bool(pxzerog.SETTINGS_ZEROG_SIMREADY_DND_ENABLED, activate_on_drop)

    def on_static_button_clicked(self):
        coll_approx = self._settings.get_as_int(ZeroGravitySettings.SETTINGS_STATIC_COLLIDER_SIMPLIFICATION_TYPE)
        static_detail = ZeroGravitySettings.SETTINGS_PLACEMENT_MODE_MARK_TRIANGLE_MESH
        if coll_approx == ColliderTypes.STATIC_COLLIDER_SIMPLIFICATION_TYPE_TRIANGLE_MESH:
            static_detail = ZeroGravitySettings.SETTINGS_PLACEMENT_MODE_MARK_TRIANGLE_MESH
        elif coll_approx == ColliderTypes.STATIC_COLLIDER_SIMPLIFICATION_TYPE_MESH_SIMPLIFICATION:
            static_detail = ZeroGravitySettings.SETTINGS_PLACEMENT_MODE_MARK_MESH_SIMPLIFICATION
        count = self._settings.get(static_detail) + 1
        self._settings.set_int(static_detail, count)

    def on_dynamic_button_clicked(self):
        coll_approx = self._settings.get_as_int(ZeroGravitySettings.SETTINGS_DYNAMIC_COLLIDER_SIMPLIFICATION_TYPE)
        dynamic_detail = ZeroGravitySettings.SETTINGS_PLACEMENT_MODE_MARK_CONVEX_HULL
        if coll_approx == ColliderTypes.DYNAMIC_COLLIDER_SIMPLIFICATION_TYPE_CONVEX_HULL:
            dynamic_detail = ZeroGravitySettings.SETTINGS_PLACEMENT_MODE_MARK_CONVEX_HULL
        elif coll_approx == ColliderTypes.DYNAMIC_COLLIDER_SIMPLIFICATION_TYPE_CONVEX_DECOMPOSITION:
            dynamic_detail = ZeroGravitySettings.SETTINGS_PLACEMENT_MODE_MARK_CONVEX_DECOMPOSITION
        count = self._settings.get(dynamic_detail) + 1
        self._settings.set_int(dynamic_detail, count)

    def on_clear_all_markers(self):
        self._physx_authoring.clear_all_markers()
        refresh_manipulator_selector()

    def on_restore_all_transforms(self):
        count = self._settings.get(ZeroGravitySettings.SETTINGS_PLACEMENT_MODE_RESTORE_ALL_TRANSFORMS) + 1
        self._settings.set_int(ZeroGravitySettings.SETTINGS_PLACEMENT_MODE_RESTORE_ALL_TRANSFORMS, count)

    def on_wait_for_simulation_step_completed(self):
        self._physx_authoring.wait_for_simulation_step_completed()

    def set_dropping(self, dropping):
        self._dropping = dropping
        if not self._settings.get(pxzerog.SETTINGS_PLACEMENT_MODE_HIDE_ACTION_BAR):
            b_def = self._buttons_def[ButtonDef.BUTTON_DROP_STOP if self._dropping else ButtonDef.BUTTON_DROP_START]
            if self._buttons:
                b_inst = self._buttons[ButtonDef.BUTTON_DROP_START]
                if b_inst:
                    b_inst.image_url = b_def[0]
                    b_inst.tooltip = b_def[1]
        self._physx_authoring.set_dropping_on_selected(self._dropping)

    def set_sweep_mode(self, sweep_mode):
        self._sweep_mode = sweep_mode
        if not self._settings.get(pxzerog.SETTINGS_PLACEMENT_MODE_HIDE_ACTION_BAR):
            b_def = self._buttons_def[ButtonDef.BUTTON_SWEEP_STOP if self._sweep_mode else ButtonDef.BUTTON_SWEEP_START]
            if self._buttons:
                b_inst = self._buttons[ButtonDef.BUTTON_SWEEP_START]
                if b_inst:
                    b_inst.image_url = b_def[0]
                    b_inst.tooltip = b_def[1]
        self._physx_authoring.set_sweep_mode(self._sweep_mode)
