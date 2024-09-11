from .bindings._physxCamera import acquire_physx_camera_interface, IPhysxCamera

def _get_interface(func, acq):
    if not hasattr(func, "iface"):
        func.iface = acq()
    return func.iface

def get_physx_camera_interface() -> IPhysxCamera:
    return _get_interface(get_physx_camera_interface, acquire_physx_camera_interface)

from .scripts.extension import *
