from .bindings._physxForceFields import acquire_physx_force_fields_interface, IPhysxForceFields


def _get_interface(func, acq):
    if not hasattr(func, "iface"):
        func.iface = acq()
    return func.iface

def get_physx_force_fields_interface() -> IPhysxForceFields:
    return _get_interface(get_physx_force_fields_interface, acquire_physx_force_fields_interface)

from .scripts.extension import *
