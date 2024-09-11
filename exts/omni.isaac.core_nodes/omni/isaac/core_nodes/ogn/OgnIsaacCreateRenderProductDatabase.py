"""Support for simplified access to data on nodes of type omni.isaac.core_nodes.IsaacCreateRenderProduct

 __   ___ .  .  ___  __       ___  ___  __      __   __   __   ___
/ _` |__  |\ | |__  |__)  /\   |  |__  |  \    /  ` /  \ |  \ |__
\__| |___ | \| |___ |  \ /--\  |  |___ |__/    \__, \__/ |__/ |___

 __   __     .  .  __  ___     .  .  __   __     ___
|  \ /  \    |\ | /  \  |      |\/| /  \ |  \ | |__  \ /
|__/ \__/    | \| \__/  |      |  | \__/ |__/ | |     |

Isaac Sim node that creates a render product for use with offscreen rendering
"""

import carb
import sys
import traceback
import usdrt

import omni.graph.core as og
import omni.graph.core._omni_graph_core as _og
import omni.graph.tools.ogn as ogn



class OgnIsaacCreateRenderProductDatabase(og.Database):
    """Helper class providing simplified access to data on nodes of type omni.isaac.core_nodes.IsaacCreateRenderProduct

    Class Members:
        node: Node being evaluated

    Attribute Value Properties:
        Inputs:
            inputs.cameraPrim
            inputs.enabled
            inputs.execIn
            inputs.height
            inputs.width
        Outputs:
            outputs.execOut
            outputs.renderProductPath
    """

    # Imprint the generator and target ABI versions in the file for JIT generation
    GENERATOR_VERSION = (1, 78, 0)
    TARGET_VERSION = (2, 170, 3)

    # This is an internal object that provides per-class storage of a per-node data dictionary
    PER_NODE_DATA = {}

    # This is an internal object that describes unchanging attributes in a generic way
    # The values in this list are in no particular order, as a per-attribute tuple
    #     Name, Type, ExtendedTypeIndex, UiName, Description, Metadata,
    #     Is_Required, DefaultValue, Is_Deprecated, DeprecationMsg
    # You should not need to access any of this data directly, use the defined database interfaces
    INTERFACE = og.Database._get_interface([
        ('inputs:cameraPrim', 'target', 0, None, 'Usd prim reference to the camera associated with this render product', {}, True, None, False, ''),
        ('inputs:enabled', 'bool', 0, None, 'Set to false to disable downstream execution and RP creation', {ogn.MetadataKeys.DEFAULT: 'true'}, True, True, False, ''),
        ('inputs:execIn', 'execution', 0, None, 'Input execution trigger', {}, True, None, False, ''),
        ('inputs:height', 'uint', 0, None, 'Height of the render product, in pixels', {ogn.MetadataKeys.DEFAULT: '720'}, True, 720, False, ''),
        ('inputs:width', 'uint', 0, None, 'Width of the render product, in pixels', {ogn.MetadataKeys.DEFAULT: '1280'}, True, 1280, False, ''),
        ('outputs:execOut', 'execution', 0, None, 'Output execution trigger', {}, True, None, False, ''),
        ('outputs:renderProductPath', 'token', 0, None, 'Render product path for the created hydra texture', {}, True, None, False, ''),
    ])

    @classmethod
    def _populate_role_data(cls):
        """Populate a role structure with the non-default roles on this node type"""
        role_data = super()._populate_role_data()
        role_data.inputs.cameraPrim = og.AttributeRole.TARGET
        role_data.inputs.execIn = og.AttributeRole.EXECUTION
        role_data.outputs.execOut = og.AttributeRole.EXECUTION
        return role_data

    class ValuesForInputs(og.DynamicAttributeAccess):
        LOCAL_PROPERTY_NAMES = {"enabled", "execIn", "height", "width", "_setting_locked", "_batchedReadAttributes", "_batchedReadValues"}
        """Helper class that creates natural hierarchical access to input attributes"""
        def __init__(self, node: og.Node, attributes, dynamic_attributes: og.DynamicAttributeInterface):
            """Initialize simplified access for the attribute data"""
            context = node.get_graph().get_default_graph_context()
            super().__init__(context, node, attributes, dynamic_attributes)
            self._batchedReadAttributes = [self._attributes.enabled, self._attributes.execIn, self._attributes.height, self._attributes.width]
            self._batchedReadValues = [True, None, 720, 1280]

        @property
        def cameraPrim(self):
            data_view = og.AttributeValueHelper(self._attributes.cameraPrim)
            return data_view.get()

        @cameraPrim.setter
        def cameraPrim(self, value):
            if self._setting_locked:
                raise og.ReadOnlyError(self._attributes.cameraPrim)
            data_view = og.AttributeValueHelper(self._attributes.cameraPrim)
            data_view.set(value)
            self.cameraPrim_size = data_view.get_array_size()

        @property
        def enabled(self):
            return self._batchedReadValues[0]

        @enabled.setter
        def enabled(self, value):
            self._batchedReadValues[0] = value

        @property
        def execIn(self):
            return self._batchedReadValues[1]

        @execIn.setter
        def execIn(self, value):
            self._batchedReadValues[1] = value

        @property
        def height(self):
            return self._batchedReadValues[2]

        @height.setter
        def height(self, value):
            self._batchedReadValues[2] = value

        @property
        def width(self):
            return self._batchedReadValues[3]

        @width.setter
        def width(self, value):
            self._batchedReadValues[3] = value

        def __getattr__(self, item: str):
            if item in self.LOCAL_PROPERTY_NAMES:
                return object.__getattribute__(self, item)
            else:
                return super().__getattr__(item)

        def __setattr__(self, item: str, new_value):
            if item in self.LOCAL_PROPERTY_NAMES:
                object.__setattr__(self, item, new_value)
            else:
                super().__setattr__(item, new_value)

        def _prefetch(self):
            readAttributes = self._batchedReadAttributes
            newValues = _og._prefetch_input_attributes_data(readAttributes)
            if len(readAttributes) == len(newValues):
                self._batchedReadValues = newValues

    class ValuesForOutputs(og.DynamicAttributeAccess):
        LOCAL_PROPERTY_NAMES = {"execOut", "renderProductPath", "_batchedWriteValues"}
        """Helper class that creates natural hierarchical access to output attributes"""
        def __init__(self, node: og.Node, attributes, dynamic_attributes: og.DynamicAttributeInterface):
            """Initialize simplified access for the attribute data"""
            context = node.get_graph().get_default_graph_context()
            super().__init__(context, node, attributes, dynamic_attributes)
            self._batchedWriteValues = { }

        @property
        def execOut(self):
            value = self._batchedWriteValues.get(self._attributes.execOut)
            if value:
                return value
            else:
                data_view = og.AttributeValueHelper(self._attributes.execOut)
                return data_view.get()

        @execOut.setter
        def execOut(self, value):
            self._batchedWriteValues[self._attributes.execOut] = value

        @property
        def renderProductPath(self):
            value = self._batchedWriteValues.get(self._attributes.renderProductPath)
            if value:
                return value
            else:
                data_view = og.AttributeValueHelper(self._attributes.renderProductPath)
                return data_view.get()

        @renderProductPath.setter
        def renderProductPath(self, value):
            self._batchedWriteValues[self._attributes.renderProductPath] = value

        def __getattr__(self, item: str):
            if item in self.LOCAL_PROPERTY_NAMES:
                return object.__getattribute__(self, item)
            else:
                return super().__getattr__(item)

        def __setattr__(self, item: str, new_value):
            if item in self.LOCAL_PROPERTY_NAMES:
                object.__setattr__(self, item, new_value)
            else:
                super().__setattr__(item, new_value)

        def _commit(self):
            _og._commit_output_attributes_data(self._batchedWriteValues)
            self._batchedWriteValues = { }

    class ValuesForState(og.DynamicAttributeAccess):
        """Helper class that creates natural hierarchical access to state attributes"""
        def __init__(self, node: og.Node, attributes, dynamic_attributes: og.DynamicAttributeInterface):
            """Initialize simplified access for the attribute data"""
            context = node.get_graph().get_default_graph_context()
            super().__init__(context, node, attributes, dynamic_attributes)

    def __init__(self, node):
        super().__init__(node)
        dynamic_attributes = self.dynamic_attribute_data(node, og.AttributePortType.ATTRIBUTE_PORT_TYPE_INPUT)
        self.inputs = OgnIsaacCreateRenderProductDatabase.ValuesForInputs(node, self.attributes.inputs, dynamic_attributes)
        dynamic_attributes = self.dynamic_attribute_data(node, og.AttributePortType.ATTRIBUTE_PORT_TYPE_OUTPUT)
        self.outputs = OgnIsaacCreateRenderProductDatabase.ValuesForOutputs(node, self.attributes.outputs, dynamic_attributes)
        dynamic_attributes = self.dynamic_attribute_data(node, og.AttributePortType.ATTRIBUTE_PORT_TYPE_STATE)
        self.state = OgnIsaacCreateRenderProductDatabase.ValuesForState(node, self.attributes.state, dynamic_attributes)

    class abi:
        """Class defining the ABI interface for the node type"""

        @staticmethod
        def get_node_type():
            get_node_type_function = getattr(OgnIsaacCreateRenderProductDatabase.NODE_TYPE_CLASS, 'get_node_type', None)
            if callable(get_node_type_function):  # pragma: no cover
                return get_node_type_function()
            return 'omni.isaac.core_nodes.IsaacCreateRenderProduct'

        @staticmethod
        def compute(context, node):
            def database_valid():
                return True
            try:
                per_node_data = OgnIsaacCreateRenderProductDatabase.PER_NODE_DATA[node.node_id()]
                db = per_node_data.get('_db')
                if db is None:
                    db = OgnIsaacCreateRenderProductDatabase(node)
                    per_node_data['_db'] = db
                if not database_valid():
                    per_node_data['_db'] = None
                    return False
            except:
                db = OgnIsaacCreateRenderProductDatabase(node)

            try:
                compute_function = getattr(OgnIsaacCreateRenderProductDatabase.NODE_TYPE_CLASS, 'compute', None)
                if callable(compute_function) and compute_function.__code__.co_argcount > 1:  # pragma: no cover
                    return compute_function(context, node)

                db.inputs._prefetch()
                db.inputs._setting_locked = True
                with og.in_compute():
                    return OgnIsaacCreateRenderProductDatabase.NODE_TYPE_CLASS.compute(db)
            except Exception as error:  # pragma: no cover
                stack_trace = "".join(traceback.format_tb(sys.exc_info()[2].tb_next))
                db.log_error(f'Assertion raised in compute - {error}\n{stack_trace}', add_context=False)
            finally:
                db.inputs._setting_locked = False
                db.outputs._commit()
            return False

        @staticmethod
        def initialize(context, node):
            OgnIsaacCreateRenderProductDatabase._initialize_per_node_data(node)
            initialize_function = getattr(OgnIsaacCreateRenderProductDatabase.NODE_TYPE_CLASS, 'initialize', None)
            if callable(initialize_function):  # pragma: no cover
                initialize_function(context, node)

            per_node_data = OgnIsaacCreateRenderProductDatabase.PER_NODE_DATA[node.node_id()]

            def on_connection_or_disconnection(*args):
                per_node_data['_db'] = None

            node.register_on_connected_callback(on_connection_or_disconnection)
            node.register_on_disconnected_callback(on_connection_or_disconnection)

        @staticmethod
        def release(node):
            release_function = getattr(OgnIsaacCreateRenderProductDatabase.NODE_TYPE_CLASS, 'release', None)
            if callable(release_function):  # pragma: no cover
                release_function(node)
            OgnIsaacCreateRenderProductDatabase._release_per_node_data(node)

        @staticmethod
        def init_instance(node, graph_instance_id):
            init_instance_function = getattr(OgnIsaacCreateRenderProductDatabase.NODE_TYPE_CLASS, 'init_instance', None)
            if callable(init_instance_function):  # pragma: no cover
                init_instance_function(node, graph_instance_id)

        @staticmethod
        def release_instance(node, graph_instance_id):
            release_instance_function = getattr(OgnIsaacCreateRenderProductDatabase.NODE_TYPE_CLASS, 'release_instance', None)
            if callable(release_instance_function):  # pragma: no cover
                release_instance_function(node, graph_instance_id)
            OgnIsaacCreateRenderProductDatabase._release_per_node_instance_data(node, graph_instance_id)

        @staticmethod
        def update_node_version(context, node, old_version, new_version):
            update_node_version_function = getattr(OgnIsaacCreateRenderProductDatabase.NODE_TYPE_CLASS, 'update_node_version', None)
            if callable(update_node_version_function):  # pragma: no cover
                return update_node_version_function(context, node, old_version, new_version)
            return False

        @staticmethod
        def initialize_type(node_type):
            initialize_type_function = getattr(OgnIsaacCreateRenderProductDatabase.NODE_TYPE_CLASS, 'initialize_type', None)
            needs_initializing = True
            if callable(initialize_type_function):  # pragma: no cover
                needs_initializing = initialize_type_function(node_type)
            if needs_initializing:
                node_type.set_metadata(ogn.MetadataKeys.EXTENSION, "omni.isaac.core_nodes")
                node_type.set_metadata(ogn.MetadataKeys.UI_NAME, "Isaac Create Render Product")
                node_type.set_metadata(ogn.MetadataKeys.CATEGORIES, "isaacCore")
                node_type.set_metadata(ogn.MetadataKeys.DESCRIPTION, "Isaac Sim node that creates a render product for use with offscreen rendering")
                node_type.set_metadata(ogn.MetadataKeys.LANGUAGE, "Python")
                icon_path = carb.tokens.get_tokens_interface().resolve("${omni.isaac.core_nodes}")
                icon_path = icon_path + '/' + "ogn/icons/omni.isaac.core_nodes.IsaacCreateRenderProduct.svg"
                node_type.set_metadata(ogn.MetadataKeys.ICON_PATH, icon_path)
                OgnIsaacCreateRenderProductDatabase.INTERFACE.add_to_node_type(node_type)

        @staticmethod
        def on_connection_type_resolve(node):
            on_connection_type_resolve_function = getattr(OgnIsaacCreateRenderProductDatabase.NODE_TYPE_CLASS, 'on_connection_type_resolve', None)
            if callable(on_connection_type_resolve_function):  # pragma: no cover
                on_connection_type_resolve_function(node)

    NODE_TYPE_CLASS = None

    @staticmethod
    def register(node_type_class):
        OgnIsaacCreateRenderProductDatabase.NODE_TYPE_CLASS = node_type_class
        og.register_node_type(OgnIsaacCreateRenderProductDatabase.abi, 2)

    @staticmethod
    def deregister():
        og.deregister_node_type("omni.isaac.core_nodes.IsaacCreateRenderProduct")
