import omni.kit.window.property
from omni.kit.property.physx.widgets import PhysicsWidget, UiProp

from pxr import Sdf, PhysxSchema


class PropertyWidgetManager:
    scheme = "prim"

    def __init__(self, physxDeformableInterface):
        self._physxDeformableInterface = physxDeformableInterface

    def set_up(self):
        self._widgets = []

        self._register_widgets()

    def tear_down(self):
        self._unregister_widgets()

        self._widgets = None
        self._physxDeformableInterface = None

    def _register_widgets(self):
        propertyWindow = omni.kit.window.property.get_window()

        for widget in self._widgets:
            propertyWindow.register_widget(PropertyWidgetManager.scheme, widget.name, widget)

    def _unregister_widgets(self):
        propertyWindow = omni.kit.window.property.get_window()

        for widget in self._widgets:
            propertyWindow.unregister_widget(PropertyWidgetManager.scheme, widget.name)
