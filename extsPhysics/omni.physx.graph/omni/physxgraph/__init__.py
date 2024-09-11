from .bindings._physxGraph import acquire_physx_graph_interface, IPhysxGraph

def _get_interface(func, acq):
    if not hasattr(func, "iface"):
        func.iface = acq()
    return func.iface

def get_physx_graph_interface() -> IPhysxGraph:
    return _get_interface(get_physx_graph_interface, acquire_physx_graph_interface)

from .scripts.extension import *
