import carb.settings
import omni.kit.app
import omni.usd
import omni.ui as ui
from omni.kit.viewport.utility import get_active_viewport_window

PROGRESS_BAR_ENABLED = "/physx/zerogravity/progressBarEnabled"
PROGRESS_BAR_LABEL = "/physx/zerogravity/progressBarLabel"
PROGRESS_BAR_VALUE = "/physx/zerogravity/progressBarValue"


# A progressbar viewport overlay to keep track of zerogravity sweep mode progress
class PhysXZeroGravityProgressView:
    def __init__(self):
        self._viewport_overlay_frame = None
        self._viewport_window = None
        self._settings = carb.settings.get_settings()
        self._settings.set_default_bool(PROGRESS_BAR_ENABLED, False)
        self._settings.set_default_string(PROGRESS_BAR_LABEL, "Sweep Area")
        self._settings.set_default_float(PROGRESS_BAR_VALUE, 0)
        self._progress_bar_enabled = self._settings.get_as_bool(PROGRESS_BAR_ENABLED)
        self._progress_bar_label = self._settings.get_as_string(PROGRESS_BAR_LABEL)
        self._progress_bar_value = self._settings.get_as_float(PROGRESS_BAR_VALUE)
        self._progres_bar_percent = self._progress_bar_value * 100
        self._settings_change_sub_enabled = omni.kit.app.SettingChangeSubscription(
            PROGRESS_BAR_ENABLED, self._on_progress_settings_changed
        )
        self._settings_change_sub_label = omni.kit.app.SettingChangeSubscription(
            PROGRESS_BAR_LABEL, self._on_progress_settings_changed
        )
        self._settings_change_sub_value = omni.kit.app.SettingChangeSubscription(
            PROGRESS_BAR_VALUE, self._on_progress_settings_changed
        )

    def on_shutdown(self):
        self._settings_change_sub_enabled = None
        self._settings_change_sub_label = None
        self._settings_change_sub_value = None
        self._cleanup_overlay()

    def _on_progress_settings_changed(self, item, event_type):
        """ Event handler for progress settings changed """
        if event_type == carb.settings.ChangeEventType.CHANGED:
            self._progress_bar_enabled = self._settings.get_as_bool(PROGRESS_BAR_ENABLED)
            if self._progress_bar_enabled:
                self._progress_bar_label = self._settings.get_as_string(PROGRESS_BAR_LABEL)
                self._progress_bar_value = self._settings.get_as_float(PROGRESS_BAR_VALUE)
                self._progres_bar_percent = self._progress_bar_value * 100
                if self._viewport_overlay_frame is None:
                    self._viewport_window = get_active_viewport_window()
                    if self._viewport_window:
                        self._viewport_overlay_frame = self._viewport_window.get_frame("omni.physx.zerogravity.progress_bar")
                        self._viewport_overlay_frame.visible = True
                if self._viewport_overlay_frame:
                    self._build_ui()
            else:
                self._cleanup_overlay()

    def _cleanup_overlay(self):
        if self._viewport_overlay_frame:
            self._viewport_overlay_frame.visible = False
            self._viewport_overlay_frame = None
        self._viewport_window = None

    def _build_progress_bar(self):
        with ui.HStack():
            ui.Spacer()
            with ui.ZStack(width=0):
                ui.Rectangle(style={"background_color": 0xff25282a, 'border_radius': 5})
                with ui.HStack():
                    ui.Spacer(width=5)
                    ui.Label(self._progress_bar_label, alignment=ui.Alignment.CENTER)
                    ui.Spacer(width=5)
                    ui.ProgressBar(style={"border_width": 1, "color": 0xffDD0000, "margin": 2}, width=100).model.set_value(self._progress_bar_value)

    def _build_ui(self):
        with self._viewport_overlay_frame:
            with ui.VStack():
                ui.Spacer()
                with ui.HStack(height=0):
                    ui.Spacer()
                    self._build_progress_bar()
                ui.Spacer(height=5)

    # Enables or disables the progressbar overlay
    def set_enabled(self, enabled):
        if self._progress_bar_enabled != enabled:
            self._settings.set_bool(PROGRESS_BAR_ENABLED, enabled)
            # _on_progress_settings_changed will also be triggered and update the overlay accordingly
