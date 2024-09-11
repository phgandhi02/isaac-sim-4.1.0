from .bindings._physXZeroGravity import acquire_physx_zero_gravity_interface, release_physx_zero_gravity_interface, release_physx_zero_gravity_interface_scripting, PhysxZeroGravity

def get_physx_zero_gravity_interface() -> PhysxZeroGravity:
    if not hasattr(get_physx_zero_gravity_interface, "iface"):
        get_physx_zero_gravity_interface.iface = acquire_physx_zero_gravity_interface()
    return get_physx_zero_gravity_interface.iface

def free_physx_zero_gravity_interface():
    if hasattr(get_physx_zero_gravity_interface, "iface"):
        release_physx_zero_gravity_interface(get_physx_zero_gravity_interface.iface)
        release_physx_zero_gravity_interface_scripting(get_physx_zero_gravity_interface.iface) # OM-60917
        delattr(get_physx_zero_gravity_interface, "iface")

from .scripts.extension import *
from .scripts.placement_mode import SubscriptionId, subscribe_to_zerog_enabled, unsubscribe_from_zerog_enabled
