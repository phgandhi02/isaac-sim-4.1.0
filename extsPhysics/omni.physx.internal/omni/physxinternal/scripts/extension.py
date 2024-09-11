import omni.ext
import ctypes
from omni.physxinternal.bindings import _physxInternal

from .physicsTests import *

from .menu import PhysxInternalMenu
from .properties.propertyWidgetManager import PropertyWidgetManager
import omni.kit.property.physx as kitProperties

from .commands import *
from .physxInternalUtils import *
import omni.physxdemos as demo

from omni.physx.scripts import deformableUtils, utils
from omni.kit.commands import execute
from pxr import PhysxSchema, UsdGeom, UsdShade
import carb

DEMO_SCENES = "omni.physxinternal.scripts.internalScenes"


class PhysxInternalExtension(omni.ext.IExt):

    _internalAddItems = []

    def __init__(self):
        self._deformablePropertyWidgetManager = None
        super().__init__()

    def remove_api(self, api, prim, api_prefix: str = None, multiple_api_token: str = None):
        ret = execute("UnapplyAPISchema", api=api, prim=prim, api_prefix=api_prefix, multiple_api_token=multiple_api_token)
        if not ret:
            carb.log_error(f"Failed to remove {api.__name__} from prim {prim.GetPrimPath().pathString}")

    def on_startup(self):

        # This extension root folder:
        self._physxInternal = _physxInternal.acquire_physx_internal_interface()

        self._deformablePropertyWidgetManager = PropertyWidgetManager(self._physxInternal)
        self._deformablePropertyWidgetManager.set_up()

        self._menu = PhysxInternalMenu()
        self._menu.on_startup()
        demo.register(DEMO_SCENES)

        self._usd_context = omni.usd.get_context()
        self._selection = self._usd_context.get_selection()

        kitProperties.Manager.instance._invisible_widget.refresh_menu_items()  

    def on_shutdown(self):
        if self._deformablePropertyWidgetManager is not None:
            self._deformablePropertyWidgetManager.tear_down()
            self._deformablePropertyWidgetManager = None

        self._menu.on_shutdown()
        self._menu = None

        _physxInternal.release_physx_internal_interface(self._physxInternal)
        _physxInternal.release_physx_internal_interface_scripting(self._physxInternal) # OM-60917
        self._physxInternal = None
        demo.unregister(DEMO_SCENES)

        items_to_remove = []

        # unfortunately double loop to remove extra add items.
        for item in kitProperties.database.extra_add_items:
            for internalItem in self._internalAddItems:
                if item == internalItem:
                    items_to_remove.append(item)

        for item in items_to_remove:
            kitProperties.database.extra_add_items.remove(item)

        self._internalAddItems = []

        kitProperties.Manager.instance._invisible_widget.refresh_menu_items()
