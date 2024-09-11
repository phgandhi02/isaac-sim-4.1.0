import carb
import omni.ext
from omni.convexdecomposition.bindings import _convexdecomposition


class ConvexDecompositionExtension(omni.ext.IExt):
    def on_startup(self):
        self._convexdecomposition = _convexdecomposition.acquire_convexdecomposition_interface()

    def on_shutdown(self):
        _convexdecomposition.release_convexdecomposition_interface(self._convexdecomposition)
        _convexdecomposition.release_convexdecomposition_interface_scripting(self._convexdecomposition) # OM-60917
        self._convexdecomposition = None
