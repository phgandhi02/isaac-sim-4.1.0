from .bindings._physxVehicle import acquire_physx_vehicle_interface, IPhysxVehicle

def _get_interface(func, acq):
    if not hasattr(func, "iface"):
        func.iface = acq()
    return func.iface

def get_physx_vehicle_interface() -> IPhysxVehicle:
    return _get_interface(get_physx_vehicle_interface, acquire_physx_vehicle_interface)

from .scripts.extension import *
