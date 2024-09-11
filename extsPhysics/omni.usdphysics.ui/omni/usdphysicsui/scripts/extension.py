import omni.ext
from omni.usdphysicsui import get_usdphysicsui_interface
from omni.usdphysicsui import get_usdphysicsuiprivate_interface
from omni.usdphysicsui.bindings._usdphysicsUI import release_usdphysics_ui_interface, release_usdphysics_ui_interface_scripting
from omni.usdphysicsui.bindings._usdphysicsUI import release_usdphysics_ui_private_interface, release_usdphysics_ui_private_interface_scripting
from omni.kit.viewport.utility import get_active_viewport_window
from .commands import *
from .immediateModeViewportOverlays import ImmediateModeViewportOverlays
import omni.ui as ui
try:
    from .physicsViewportMenuHelper import PhysicsViewportMenuHelper
except:
    pass

_extension_instance = None


def get_usdphysicsui_instance():
    global _extension_instance
    return _extension_instance

class UsdPhysicsUIExtension(omni.ext.IExt):

    def on_startup(self):
        self._usdphysicsui_iface = get_usdphysicsui_interface()
        self._usdphysicsuiprivate_iface = get_usdphysicsuiprivate_interface()
        self.__viewport_overlays = ImmediateModeViewportOverlays("omni.usdphysics.ui.frame", self.__immediate_mode_draw_overlays)
        self.__viewport_overlays.install_on_viewport(get_active_viewport_window())        
        self._physics_viewport_menu = None
        try:
            # We have an optional dependency on VP2 "omni.kit.viewport.menubar.display" that will
            # not get loaded during kit VP1 L1 tests, and in turn will throw an exception here 
            from .physicsViewportMenu import PhysicsViewportMenu
            self._physics_viewport_menu = PhysicsViewportMenu()
            self._physics_viewport_menu.register_with_viewport()
        except:
            pass
        global _extension_instance
        _extension_instance = self

    def unregister_physics_viewport_menu(self):
        if self._physics_viewport_menu is not None:
            try:
                self._physics_viewport_menu.unregister_from_viewport()
            except:
                pass
        self._physics_viewport_menu = None

    def on_shutdown(self):
        self.unregister_physics_viewport_menu()
        self.__viewport_overlays.destroy()
        self.__viewport_overlays = None
        release_usdphysics_ui_interface(self._usdphysicsui_iface)
        release_usdphysics_ui_interface_scripting(self._usdphysicsui_iface)
        release_usdphysics_ui_private_interface(self._usdphysicsuiprivate_iface)
        release_usdphysics_ui_private_interface_scripting(self._usdphysicsuiprivate_iface)
        self._usdphysicsui_iface = None
        global _extension_instance
        _extension_instance = None
    
    def __immediate_mode_draw_overlays(self, scene_view, enable_pick, amended_projection):
        dpi_scale = ui.Workspace.get_dpi_scale()
        self._usdphysicsuiprivate_iface.private_draw_immediate_mode_viewport_overlays(
                        [scene_view.view[i] for i in range(16)],
                        [amended_projection[i] for i in range(16)],
                        scene_view.screen_position_x,
                        scene_view.screen_position_y,
                        scene_view.computed_content_width,
                        scene_view.computed_content_height,
                        dpi_scale,
                        enable_pick)