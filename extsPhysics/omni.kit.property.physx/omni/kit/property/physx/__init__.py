from collections import namedtuple
import omni.ext
import omni.kit.window.property as p
from .utils import get_title_from_schema, rebuild_property_window, get_widget_name
from .widgets import PhysicsWidget, InvisibleWidget, PhysicsMaterialBindingWidget, PhysicsCustomPropertiesWidget
from .widgets import JointVisualizationWidget, PhysicsDefaultMaterialBindingWidget, MainFrameWidget
from .overrides import custom_widgets, custom_builders
from .database import Component
from .builders import ModelWithWidgetBuilder
from . import database

"""
if you don't want to be directly dependent on this extension and still want to register
custom properties use the following snippet:
############################# example #############################
def custom_register():
    import omni.kit.property.physx
    omni.kit.property.physx.register_custom_property("physics:gravityDirection")
    omni.kit.property.physx.register_custom_property("physics:gravityMagnitude", "Read-only Magnitude", True)
    
manager = omni.kit.app.get_app().get_extension_manager()
self._ext_enable_sub = manager.subscribe_to_extension_enable(
    lambda _: custom_register(),
    None,
    ext_name="omni.kit.property.physx",
    hook_name="foo.bar listener")
"""

CustomPropertyData = namedtuple('CustomPropertyData', "display_name, read_only")

def register_custom_property(name, override_display_name=None, read_only=False):
    if Manager.instance is not None:
        Manager.instance.register_property(name, CustomPropertyData(override_display_name, read_only))


def register_schema(schema, order=None, component=None, custom_widget=None, prefix=None):
    if Manager.instance is None:
        return

    title = get_title_from_schema(schema)

    if order is not None:
        database.default_order_map[schema] = order

    if prefix is not None and schema not in database.ext_multi_api_prefixes:
        database.ext_multi_api_prefixes[schema] = prefix
        
    if component is not None:
        title = component.title
        database.components[schema.__name__] = component

    component = database.components.get(schema.__name__)
    widget_class = PhysicsWidget if custom_widget is None else custom_widget
    widget = widget_class(title, schema, custom_builders)
    Manager.instance._frame_widget.register_widget(get_widget_name(schema), widget)

    Manager.instance._invisible_widget.refresh_menu_items()
    rebuild_property_window()


def unregister_schema(schema):
    if Manager.instance is None:
        return

    database.default_order_map.pop(schema, None)
    database.components.pop(schema.__name__, None)
    database.ext_multi_api_prefixes.pop(schema, None)

    Manager.instance._frame_widget.unregister_widget(get_widget_name(schema))

    Manager.instance._invisible_widget.refresh_menu_items()
    rebuild_property_window()

def register_builder(property_name, builder_data):
    custom_builders[property_name] = builder_data


def unregister_builder(property_name):
    custom_builders.pop(property_name)


def register_widget(name, widget):
    if Manager.instance is None:
        return

    Manager.instance._frame_widget.register_widget(name, widget)


def unregister_widget(name):
    if Manager.instance is None:
        return

    Manager.instance._frame_widget.unregister_widget(name)


def refresh():
    if Manager.instance is None:
        return

    Manager.instance._frame_widget.request_rebuild()

class Extension(omni.ext.IExt):
    def on_startup(self, ext_id):
        self._manager = Manager()

    def on_shutdown(self):
        self._manager.on_shutdown()
        del self._manager


class Manager:
    scheme = "prim"
    instance = None

    def __init__(self):
        Manager.instance = self

        self._widgets = []
        self._custom_properties = dict()
        self._custom_properties_widget = PhysicsCustomPropertiesWidget()

        def gen_setup(schema):
            def get_widget(schema):
                return custom_widgets.get(schema, PhysicsWidget)

            component = database.components.get(schema.__name__)
            title = component.title if component else get_title_from_schema(schema)
            return (get_widget_name(schema), get_widget(schema), title, schema, custom_builders)

        self._widgets += [gen_setup(p) for p in database.base_prims] + [gen_setup(a) for a in database.base_apis]
        self._widgets += [gen_setup(a) for a in database.attachment_apis]
        self._widgets += [gen_setup(a) for a in database.axis_apis]
        self._widgets.append((JointVisualizationWidget.name, JointVisualizationWidget))
        self._widgets += [gen_setup(p) for p in database.joint_prims] + [gen_setup(a) for a in database.joint_apis]
        self._widgets.append((PhysicsMaterialBindingWidget.name, PhysicsMaterialBindingWidget))
        self._widgets.append((PhysicsDefaultMaterialBindingWidget.name, PhysicsDefaultMaterialBindingWidget))
        self._widgets += [gen_setup(a) for a in database.vehicle_apis]
        self._widgets += [gen_setup(a) for a in database.camera_apis]
        self._widgets += [gen_setup(a) for a in database.mimic_joint_apis]

        self._frame_widget = MainFrameWidget(self._widgets)
        self._invisible_widget = InvisibleWidget()

        self._register_widgets()

    def on_shutdown(self):
        self._unregister_widgets()

        self._widgets = []
        self._custom_properties_widget = None
        self._custom_properties = None
        self._invisible_widget = None
        self._frame_widget = None
        ModelWithWidgetBuilder.shutdown()
        Manager.instance = None

    def register_property(self, name, data):
        self._custom_properties[name] = data
        self._custom_properties_widget.set_properties(self._custom_properties)

    def _register_widgets(self):
        w = p.get_window()

        w.register_widget(Manager.scheme, MainFrameWidget.name, self._frame_widget)
        w.register_widget(Manager.scheme, PhysicsCustomPropertiesWidget.name, self._custom_properties_widget)
        w.register_widget(Manager.scheme, InvisibleWidget.name, self._invisible_widget)
        

    def _unregister_widgets(self):
        w = p.get_window()

        w.unregister_widget(Manager.scheme, MainFrameWidget.name)
        w.unregister_widget(Manager.scheme, PhysicsCustomPropertiesWidget.name)
        w.unregister_widget(Manager.scheme, InvisibleWidget.name)
