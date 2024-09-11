import carb
import carb.settings
import carb.input
import omni.ext
import omni.kit.extensions
import omni.kit.app
import omni.kit.commands
import omni.kit.window.toolbar
import omni.kit.menu.utils
import omni.ui as ui
from omni.kit.widget.toolbar.builtin_tools.models.setting_model import BoolSettingModel
from omni.kit.widget.toolbar.hotkey import Hotkey
from omni.kit.widget.toolbar.widget_group import WidgetGroup
from .constants import ZeroGravityConstants
from .settings_menu import ZeroGravitySettingsPopupMenu
import omni.physxzerogravity.bindings._physXZeroGravity as pxzerog


# The single button toolbar addition (usually on the left) to enable zero gravity extension
class ToolButtonGroup(WidgetGroup):
    TOOLNAME = "zerogravity"
    ACTION_NAME = ZeroGravityConstants.EXTENSION_DISPLAY_NAME
    TOOLTIP_ENABLED = f"{ACTION_NAME} (X)"
    TOOLTIP_DISABLED_DUE_TO_INSPECTOR = f"{ACTION_NAME} disabled when Physics Inspector is active"

    GREY_STYLE = {f"Button.Image::{TOOLNAME}": {"color": 0xff888888}}
    WHITE_STYLE = {f"Button.Image::{TOOLNAME}": {"color": 0xffffffff}}

    def __init__(self, icon_path):
        """ Creates a kit toolbar tool button group using a boolean settings model, i.e. a 'switch on or off' mode """
        self._enabled = True
        self._enabled_button = None
        self._icon_path = icon_path
        self._settings = carb.settings.get_settings()
        self._model = BoolSettingModel(pxzerog.SETTINGS_PLACEMENT_MODE_ENABLED, False)

        # setup hotkey X for toggling placement mode
        self._hotkey = Hotkey(
            f"{self.TOOLNAME}_enabled::hotkey",
            carb.input.KeyboardInput.X,
            lambda: self._model.set_value(not self._model.get_value_as_bool()),
            lambda: True if self._enabled else False
        )

        # Setup context menu for options
        self._settings_menu_popup = ZeroGravitySettingsPopupMenu()

        self._settings_subs = []
        self._settings_subs.append(omni.kit.app.SettingChangeSubscription(
            pxzerog.SETTING_SUPPORTUI_PHYSICS_INSPECTOR_ENABLED, self._on_supportui_physics_inspector_enabled_setting_changed
        ))

    def _on_supportui_physics_inspector_enabled_setting_changed(self, item, event_type):
        """
            Event handler for supportUI physics inspector enabled setting changed. Inspector and ZG are exclusive.
            The inspector can only be enabled when ZG is off, so we do the same: we enable the toolbar button only when
            the inspector is off.
        """
        if self._enabled_button is None:
            return  # Toolbar button hasn't been created yet, skip this notification
        if event_type == carb.settings.ChangeEventType.CHANGED:
            inspector_enabled = self._settings.get_as_bool(pxzerog.SETTING_SUPPORTUI_PHYSICS_INSPECTOR_ENABLED)
            if inspector_enabled:
                self._enabled_button.set_tooltip(self.TOOLTIP_DISABLED_DUE_TO_INSPECTOR)
            else:
                self._enabled_button.set_tooltip(self.TOOLTIP_ENABLED)
            self.set_enabled(not inspector_enabled)

    def get_style(self):
        style = {f"Button.Image::{self.TOOLNAME}": {"image_url": f"{self._icon_path}"}}
        return style

    def set_enabled(self, enabled):
        self._enabled = enabled
        self._enabled_button.enabled = enabled
        if not enabled:
            self._enabled_button.set_style(self.GREY_STYLE)
        else:
            self._enabled_button.set_style(self.WHITE_STYLE)

    def create(self, default_size):
        self._enabled_button = ui.ToolButton(
            name=self.TOOLNAME,
            tooltip=self.TOOLTIP_ENABLED,
            model=self._model,
            width=default_size,
            height=default_size,
            mouse_pressed_fn=lambda x, y, b, _: self._on_mouse_pressed(x, y, b),
            mouse_released_fn=lambda x, y, b, _: self._on_mouse_released(b),
            checked=self._model.get_value_as_bool(),
            clicked_fn=self._on_activation_button_clicked,
        )
        return {self.TOOLNAME: self._enabled_button}

    def clean(self):
        """ Cleanup of the tool button"""
        self._model.clean()
        self._model = None
        self._hotkey.clean()
        self._hotkey = None
        self._settings_subs = []
        self._enabled_button = None

        if self._settings_menu_popup:
            self._settings_menu_popup.hide()
            self._settings_menu_popup = None

    def _on_activation_button_clicked(self):
        # This is actually redundant because of the model listeners but populates the command history
        omni.kit.commands.execute("ZeroGravitySetEnabledCommand", enabled=self._model.get_value_as_bool())

    def _on_mouse_pressed(self, x, y, button):
        if button == 0 or button == 1:
            self._settings_menu_popup.show_popup(x, y, button == 0)

    def _on_mouse_released(self, button):
        if button == 0:
            self._settings_menu_popup.cancel_popup_task()
