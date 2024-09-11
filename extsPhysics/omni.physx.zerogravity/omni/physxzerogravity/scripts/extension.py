import os
import carb
import carb.settings
import omni.ext
import omni.kit.extensions
import omni.kit.app
import omni.kit.commands
import omni.kit.widget.stage
import omni.kit.window.toolbar
import omni.kit.menu.utils
import omni.timeline
import omni.physx
import omni.usd
import omni.physxzerogravity
import omni.physxzerogravity.bindings._physXZeroGravity as pxzerog
from .toolbar_button import ToolButtonGroup
from .placement_mode import PlacementMode
from .payload import ZeroGravityPayload
from .commands import register_commands
from .utils import refresh_manipulator_selector
from .dndhelper import ZeroGravityDndHelper
from omni.physx.scripts.utils import safe_import_tests

safe_import_tests("omni.physxzerogravity.scripts", ["tests"])


class Extension(omni.ext.IExt):
    ICON_FOLDER = "resources/icons"
    ICON_SVG = "physics_placement_toolbar.svg"

    def __init__(self):
        self._sweepAreaProgressView = None
        # create the zero gravity drag and drop helper - this also activates it immediately
        self._dnd_helper = ZeroGravityDndHelper()
        super().__init__()

    def on_startup(self, ext_id):
        """ Extension startup for the physics zero gravity extension"""

        # create toolbar/button group
        ext_path = omni.kit.app.get_app().get_extension_manager().get_extension_path(ext_id)
        icon_folder = f"{os.path.join(ext_path, self.ICON_FOLDER)}"
        icon_path = f"{os.path.join(icon_folder, self.ICON_SVG)}"
        self._toolbar = omni.kit.window.toolbar.get_instance()
        self._toolbutton_group = ToolButtonGroup(icon_path)
        self._toolbar.add_widget(self._toolbutton_group, 100)

        register_commands()  # Register all ZG Kit commands

        self._physx_zero_gravity = omni.physxzerogravity.get_physx_zero_gravity_interface()

        ZeroGravityPayload.placement = PlacementMode(icon_folder)

        # subscribe to zerog event stream
        event_stream = self._physx_zero_gravity.get_event_stream()
        self._zerog_event_sub = event_stream.create_subscription_to_pop(self._on_zerog_event)

    def on_shutdown(self):
        """ Extension shutdown for the physics zero gravity extension"""
        if self._dnd_helper is not None:
            self._dnd_helper.destroy()
            self._dnd_helper = None

        if ZeroGravityPayload.cmds is not None:
            omni.kit.commands.unregister_module_commands(ZeroGravityPayload.cmds)
            ZeroGravityPayload.cmds = None

        if ZeroGravityPayload.placement is not None:
            ZeroGravityPayload.placement.clean()
            ZeroGravityPayload.placement = None

        self._toolbar.remove_widget(self._toolbutton_group)
        self._toolbutton_group.clean()
        self._toolbutton_group = None
        self._toolbar = None

        self._zerog_event_sub = None

        omni.physxzerogravity.free_physx_zero_gravity_interface()

    def _on_zerog_event(self, event):
        if event.type == int(pxzerog.ZeroGravityEventType.REFRESH_GIZMO):
            refresh_manipulator_selector()
