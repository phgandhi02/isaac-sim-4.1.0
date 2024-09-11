import os
from functools import partial
import carb.settings
import omni.ext
import omni.kit.app
import omni.kit.menu.utils
import omni.kit.notification_manager as nm
import omni.physxsupportui.bindings._physxSupportUi as pxsupportui
from pxr import PhysicsSchemaTools
from .. import get_physx_supportui_interface
from .. import get_physx_supportui_private_interface
from .action_bar import ActionBar
from .helpers import Helpers
from .rigid_body_transform_manipulator_registry import TransformManipulatorRegistry
from .rigid_body_selection_mode import RigidBodySelectionMode
from .utils import refresh_manipulator_selector
from .inspector import PhysXInspector
from omni.physxuicommon import windowmenuitem

from omni.physx.scripts.utils import safe_import_tests
safe_import_tests("omni.physxsupportui.scripts.tests")


class PhysxSupportUiExtension(omni.ext.IExt):
    def on_startup(self, ext_id):
        self._manipulator_registry = TransformManipulatorRegistry()

        self._settings = carb.settings.get_settings()
        self._settings_subs = []
        self._selection = omni.usd.get_context().get_selection()
        self._physx_supportui_interface = get_physx_supportui_interface()
        self._physx_supportui_private_interface = get_physx_supportui_private_interface()

        ext_path = omni.kit.app.get_app().get_extension_manager().get_extension_path(ext_id)
        self._icons_folder = f"{os.path.join(ext_path, 'resources/icons')}"

        self._settings_subs.append(omni.kit.app.SettingChangeSubscription(
            pxsupportui.SETTINGS_ACTION_BAR_ENABLED, self._action_bar_enabled_changed
        ))

        self._settings_subs.append(omni.kit.app.SettingChangeSubscription(
            pxsupportui.SETTINGS_RIGID_BODY_SELECTION_MODE_ENABLED, self._rb_selection_enabled_setting_changed
        ))

        self._settings_subs.append(omni.kit.app.SettingChangeSubscription(
            pxsupportui.SETTINGS_CUSTOM_MANIPULATOR_ENABLED, self._enable_custom_manipulation_changed
        ))

        self._rb_sel_mode = RigidBodySelectionMode()

        self._stage_event_sub = omni.usd.get_context().get_stage_event_stream().create_subscription_to_pop(
            self._on_stage_event, name="omni.physx.supportui._on_stage_event"
        )

        event_stream = self._physx_supportui_interface.get_event_stream()
        self._supportui_event_sub = event_stream.create_subscription_to_pop(self._on_supportui_event)

        self._action_bar = None
        create_action_bar = self._settings.get(pxsupportui.SETTINGS_ACTION_BAR_ENABLED)
        self._inspector = PhysXInspector(self._icons_folder)

        def main_menu_click():
            val = self._settings.get_as_bool(pxsupportui.SETTINGS_ACTION_BAR_ENABLED)
            self._settings.set_bool(pxsupportui.SETTINGS_ACTION_BAR_ENABLED, not val)

        def is_ticked():
            return self._action_bar.is_visible() if self._action_bar else False

        self._menu = windowmenuitem.MenuItem("Physics/Physics Authoring Toolbar", "Window", main_menu_click, is_ticked, create_action_bar)

        if create_action_bar:
            self.create_action_bar()

    def _action_bar_enabled_changed(self, item, event_type):
        if event_type == carb.settings.ChangeEventType.CHANGED:
            action_bar_enabled = self._settings.get_as_bool(pxsupportui.SETTINGS_ACTION_BAR_ENABLED)
            self.show_action_bar(action_bar_enabled)

    def show_action_bar(self, visible):
        if visible:
            if self._action_bar is None:
                self.create_action_bar()
            else:
                self._action_bar._window.visible = True  # just show already existing toolbar to the user
        else:
            self.remove_action_bar()
        self._menu.refresh()

    def create_action_bar(self):
        self._action_bar = ActionBar(self._icons_folder, self._physx_supportui_interface)
        if self._action_bar:
            self._action_bar.set_action_bar_visibility_changed_fn(self._on_action_bar_visibility_changed)

    def remove_action_bar(self):
        if self._action_bar is not None:
            self._action_bar.clean()
            self._action_bar = None

    def _on_action_bar_visibility_changed(self, visible):
        if self._menu:
            self._menu.refresh()

        window_enabled = self._settings.get_as_bool(pxsupportui.SETTINGS_ACTION_BAR_ENABLED)
        if (window_enabled and not visible) or (not window_enabled and visible):
            self._settings.set_bool(pxsupportui.SETTINGS_ACTION_BAR_ENABLED, visible)

    def _rb_selection_enabled_setting_changed(self, item, event_type):
        if event_type == carb.settings.ChangeEventType.CHANGED:
            self._rb_sel_mode.enabled = self._settings.get_as_bool(pxsupportui.SETTINGS_RIGID_BODY_SELECTION_MODE_ENABLED)
            refresh_manipulator_selector()

    def _on_stage_event(self, evt):
        if evt.type == int(omni.usd.StageEventType.SELECTION_CHANGED):
            self._rb_sel_mode.try_select_rigid_body_parent()

    def _enable_custom_manipulation_changed(self, item, event_type):
        if event_type == carb.settings.ChangeEventType.CHANGED:
            refresh_manipulator_selector()

    def _on_supportui_event(self, event):
        if event.type == int(pxsupportui.SupportUiEventType.COLLIDER_CREATED):
            collider_type = event.payload['colliderType']
            simplification_type = event.payload['simplificationType']
            num_remaining_coll_tasks = event.payload['numRemainingCollTasks']
            num_total_coll_tasks = event.payload['numTotalCollTasks']
            # post notification
            if (
                num_remaining_coll_tasks == 0
                and num_total_coll_tasks > 0
                and self._settings.get_as_bool(pxsupportui.SETTINGS_FLOATING_NOTIFICATIONS_ENABLED)
            ):
                # collider_type: False = static, True = dynamic
                if collider_type is False:
                    coll_type, simp_type = ("static", Helpers._static_coll_approximations[simplification_type])
                else:
                    coll_type, simp_type = ("dynamic", Helpers._dynamic_coll_approximations[simplification_type])

                coll_str, were_was = ("collider", "was")
                if num_total_coll_tasks > 1:
                    coll_str, were_was = ("colliders", "were")

                nm.post_notification(
                    f"{num_total_coll_tasks} {coll_type} {coll_str} ({simp_type} approximation)\n{were_was} created.",
                    duration=5
                )

        elif event.type == int(pxsupportui.SupportUiEventType.KINEMATICS_TOGGLED):
            rb_manipulator_enabled = self._settings.get_as_bool(pxsupportui.SETTINGS_CUSTOM_MANIPULATOR_ENABLED)
            if rb_manipulator_enabled:
                prim_path_encoded = event.payload['primPath']
                prim_path = PhysicsSchemaTools.decodeSdfPath(prim_path_encoded[0], prim_path_encoded[1])
                selected_paths = self._selection.get_selected_prim_paths()
                if prim_path in selected_paths:
                    refresh_manipulator_selector()

        elif event.type == int(pxsupportui.SupportUiEventType.AUTO_COLL_CANCELED):
            prim_path_encoded = event.payload['primPath']
            prim_path = PhysicsSchemaTools.decodeSdfPath(prim_path_encoded[0], prim_path_encoded[1])
            self._selection.set_selected_prim_paths([str(prim_path)], True)

            def force_create_static_colliders():
                ct = pxsupportui.SupportUiColliderType                
                stage = omni.usd.get_context().get_stage()
                pseudo_root_path = stage.GetPseudoRoot().GetPath()
                self._selection.set_selected_prim_paths([str(pseudo_root_path)], True)
                self._physx_supportui_interface.create_colliders(
                    PhysicsSchemaTools.sdfPathToInt(pseudo_root_path), ct.STATIC
                )

            force_button = nm.NotificationButtonInfo("Yes", on_complete=force_create_static_colliders)
            dismiss_button = nm.NotificationButtonInfo("No", on_complete=None)
            nm.post_notification(
                "Automatic collider generation:\n"
                "Asset already has physics.\n"
                f"E.g. {prim_path}\n\n"
                "Force additional collider generation?",
                button_infos=[force_button, dismiss_button],
                status=nm.NotificationStatus.WARNING,
                duration=10
            )

    def on_shutdown(self):
        # Deregister the function that shows the window from omni.ui
        omni.ui.Workspace.set_show_window_fn(ActionBar.get_toolbar_name(), None)

        if self._menu:
            self._menu.remove()
            self._menu = None

        self.remove_action_bar()
        if self._inspector:
            self._inspector.destroy()
        self._inspector = None

        if self._manipulator_registry is not None:
            self._manipulator_registry.destroy()
            self._manipulator_registry = None

        self._rb_sel_mode = None
        self._supportui_event_sub = None
        self._stage_event_sub = None
        self._settings_subs = []
        self._selection = None
        self._settings = None

        pxsupportui.release_physx_supportui_interface(self._physx_supportui_interface)
        pxsupportui.release_physx_supportui_private_interface(self._physx_supportui_private_interface)
        pxsupportui.release_physx_supportui_private_interface_scripting(self._physx_supportui_private_interface)
        pxsupportui.release_physx_supportui_interface_scripting(self._physx_supportui_interface)

        self._physx_supportui_interface = None
        self._physx_supportui_private_interface = None
