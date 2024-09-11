from .bindings._physxSupportUi import acquire_physx_supportui_interface, IPhysxSupportUi
from .bindings._physxSupportUi import acquire_physx_supportui_private_interface, IPhysxSupportUiPrivate

def _get_interface(func, acq):
    if not hasattr(func, "iface"):
        func.iface = acq()
    return func.iface

def get_physx_supportui_interface() -> IPhysxSupportUi:
    return _get_interface(get_physx_supportui_interface, acquire_physx_supportui_interface)

def get_physx_supportui_private_interface() -> IPhysxSupportUiPrivate:
    return _get_interface(get_physx_supportui_private_interface, acquire_physx_supportui_private_interface)

from .scripts.extension import *
