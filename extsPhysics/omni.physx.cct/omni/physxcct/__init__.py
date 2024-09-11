from .bindings._physxCct import acquire_physx_cct_interface, IPhysxCct, CctEvent

def get_physx_cct_interface() -> IPhysxCct:
    if not hasattr(get_physx_cct_interface, "iface"):
        get_physx_cct_interface.iface = acquire_physx_cct_interface()
    return get_physx_cct_interface.iface

from .scripts.extension import *
