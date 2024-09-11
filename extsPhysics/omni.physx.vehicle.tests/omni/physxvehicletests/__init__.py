from .bindings._physxVehicleTests import acquire_physx_vehicle_testing_interface, IPhysxVehicleTesting

def _get_interface(func, acq):
    if not hasattr(func, "iface"):
        func.iface = acq()
    return func.iface

def get_physx_vehicle_testing_interface() -> IPhysxVehicleTesting:
    return _get_interface(get_physx_vehicle_testing_interface, acquire_physx_vehicle_testing_interface)

from .scripts.extension import *
