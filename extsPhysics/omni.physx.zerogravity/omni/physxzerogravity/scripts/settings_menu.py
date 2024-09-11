import carb.settings
import omni.kit.commands
import omni.physx.bindings._physx as pb
import omni.physxzerogravity.bindings._physXZeroGravity as pxzerog
import omni.ui as ui

from .menu_helpers import MenuHelpers, PopupMenu


class StaticCollSettingsMenu(PopupMenu):
    def _build_menu(self):
        MenuHelpers.create_static_coll_menu()


class DynamicCollSettingsMenu(PopupMenu):
    def _build_menu(self):
        MenuHelpers.create_dynamic_coll_menu()


class ZeroGravitySettingsPopupMenu(PopupMenu):
    def _build_menu(self):
        ZeroGravitySettingsMenu.build_menu()


class ZeroGravitySettingsMenu:
    def __init__(self):
        self._settings_context_menu = ui.Menu("")

    def __del__(self):
        self._settings_context_menu = None

    @staticmethod
    def build_menu():
        """ Builds the menu UI """
        settings = carb.settings.get_settings()
        ui.MenuItem(
            "Move - Lock Rotation",
            checkable=True,
            checked=settings.get_as_bool(pxzerog.SETTINGS_PLACEMENT_MODE_LOCK_ROTATION),
            checked_changed_fn=lambda c: settings.set_bool(pxzerog.SETTINGS_PLACEMENT_MODE_LOCK_ROTATION, c),
        ),
        ui.MenuItem(
            "Show Dynamic Markers",
            checkable=True,
            checked=settings.get_as_bool(pxzerog.SETTINGS_PLACEMENT_MODE_SHOW_DYNAMICS),
            checked_changed_fn=lambda c: settings.set_bool(pxzerog.SETTINGS_PLACEMENT_MODE_SHOW_DYNAMICS, c),
        ),
        ui.MenuItem(
            "Show Static Markers",
            checkable=True,
            checked=settings.get_as_bool(pxzerog.SETTINGS_PLACEMENT_MODE_SHOW_STATICS),
            checked_changed_fn=lambda c: settings.set_bool(pxzerog.SETTINGS_PLACEMENT_MODE_SHOW_STATICS, c),
        ),
        ui.MenuItem(
            "Mark Sweep Items as Dynamic",
            checkable=True,
            checked=settings.get_as_bool(pxzerog.SETTINGS_PLACEMENT_MODE_SET_SWEPT_ITEMS_DYNAMIC),
            checked_changed_fn=lambda c: omni.kit.commands.execute("ZeroGravityMarkSweepItemsDynamicCommand", use_dynamic_markers_for_swept_items=c),
        ),
        ui.MenuItem(
            "Visualize Sweep Area",
            checkable=True,
            checked=settings.get_as_bool(pxzerog.SETTINGS_PLACEMENT_MODE_SWEEP_AREA_VISUALIZE),
            checked_changed_fn=lambda c: omni.kit.commands.execute("ZeroGravitySweepAreaVisualizeCommand", visualize_aabb=c),
        )
        development_mode = carb.settings.get_settings().get(pb.SETTING_PHYSICS_DEVELOPMENT_MODE)
        if development_mode:
            ui.MenuItem(
                "Enable Debug Logging",
                checkable=True,
                checked=settings.get_as_bool(pxzerog.SETTINGS_LOGGING_ENABLED),
                checked_changed_fn=lambda c: settings.set_bool(pxzerog.SETTINGS_LOGGING_ENABLED, c),
            )
        ui.MenuItem(
            "Activate ZeroGravity when dropping Simready assets",
            checkable=True,
            checked=settings.get_as_bool(pxzerog.SETTINGS_ZEROG_SIMREADY_DND_ENABLED),
            checked_changed_fn=lambda c: omni.kit.commands.execute("ZeroGravityActivateSimreadyDnDIntegrationCommand", activate_on_drop=c),
        )
        ui.MenuItem("Clear All Markers", triggered_fn=lambda: omni.kit.commands.execute("ZeroGravityClearAllCommand"))
        ui.MenuItem("Restore All Transforms", triggered_fn=lambda: omni.kit.commands.execute("ZeroGravityRestoreAllTransformsCommand"))

    def show(self):
        """ Build & show the settings menu """
        self._settings_context_menu.clear()
        with self._settings_context_menu:
            ZeroGravitySettingsMenu.build_menu()
        self._settings_context_menu.show()

    def hide(self):
        self._settings_context_menu.hide()
