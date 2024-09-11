import weakref
from omni.kit.viewport.registry import RegisterScene
from .zerog_transform_manipulator import ZeroGravityTransformManipulator
from omni.kit.manipulator.prim.core.reference_prim_marker import ReferencePrimMarker


class ZeroGravityTransformManipulatorScene:
    def __init__(self, desc: dict):
        usd_context_name = desc.get("usd_context_name")
        self.__transform_manip = ZeroGravityTransformManipulator(
            usd_context_name=usd_context_name, viewport_api=desc.get("viewport_api")
        )
        self.__reference_prim_marker = ReferencePrimMarker(
            usd_context_name=usd_context_name, manipulator_model=weakref.proxy(self.__transform_manip.model)
        )

    def destroy(self):
        if self.__transform_manip:
            self.__transform_manip.destroy()
            self.__transform_manip = None

        if self.__reference_prim_marker:
            self.__reference_prim_marker.destroy()
            self.__reference_prim_marker = None

    @property
    def visible(self):
        return True

    @visible.setter
    def visible(self, value):
        pass

    @property
    def categories(self):
        return ("zero gravity manipulator")

    @property
    def name(self):
        return "Zero Gravity Transform"


class TransformManipulatorRegistry:
    def __init__(self):
        self._scene = RegisterScene(ZeroGravityTransformManipulatorScene, "omni.physxzerogravity")

    def __del__(self):
        self.destroy()

    def destroy(self):
        self._scene = None
