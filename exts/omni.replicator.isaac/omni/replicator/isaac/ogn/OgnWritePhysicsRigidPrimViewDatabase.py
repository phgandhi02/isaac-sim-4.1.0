"""Support for simplified access to data on nodes of type omni.replicator.isaac.OgnWritePhysicsRigidPrimView

 __   ___ .  .  ___  __       ___  ___  __      __   __   __   ___
/ _` |__  |\ | |__  |__)  /\   |  |__  |  \    /  ` /  \ |  \ |__
\__| |___ | \| |___ |  \ /--\  |  |___ |__/    \__, \__/ |__/ |___

 __   __     .  .  __  ___     .  .  __   __     ___
|  \ /  \    |\ | /  \  |      |\/| /  \ |  \ | |__  \ /
|__/ \__/    | \| \__/  |      |  | \__/ |__/ | |     |

This node writes physics attributes to TensorAPI views
"""

import numpy
import sys
import traceback

import omni.graph.core as og
import omni.graph.core._omni_graph_core as _og
import omni.graph.tools.ogn as ogn



class OgnWritePhysicsRigidPrimViewDatabase(og.Database):
    """Helper class providing simplified access to data on nodes of type omni.replicator.isaac.OgnWritePhysicsRigidPrimView

    Class Members:
        node: Node being evaluated

    Attribute Value Properties:
        Inputs:
            inputs.attribute
            inputs.dist_param_1
            inputs.dist_param_2
            inputs.distribution
            inputs.execIn
            inputs.indices
            inputs.num_buckets
            inputs.on_reset
            inputs.operation
            inputs.prims
            inputs.values
        Outputs:
            outputs.execOut
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
        ('inputs:attribute', 'string', 0, None, 'Name of attribute that is to be written', {}, True, "", False, ''),
        ('inputs:dist_param_1', 'float[]', 0, None, 'Distribution parameter 1', {}, True, [], False, ''),
        ('inputs:dist_param_2', 'float[]', 0, None, 'Distribution parameter 2', {}, True, [], False, ''),
        ('inputs:distribution', 'string', 0, None, 'Type of distribution used to sample values', {}, True, "", False, ''),
        ('inputs:execIn', 'execution', 0, None, 'exec', {}, True, None, False, ''),
        ('inputs:indices', 'int[]', 0, None, 'Indices of the environments to assign the physics attribute', {}, True, [], False, ''),
        ('inputs:num_buckets', 'int', 0, None, 'Number of buckets to randomize from', {}, True, 0, False, ''),
        ('inputs:on_reset', 'bool', 0, None, 'indicates whether an on_reset context triggered the execution', {}, True, False, False, ''),
        ('inputs:operation', 'string', 0, None, 'Type of randomization operation to be applied', {}, True, "", False, ''),
        ('inputs:prims', 'string', 0, None, 'Name of registered view to randomize', {}, True, "", False, ''),
        ('inputs:values', 'float[]', 0, None, 'Values to be assigned to the physics attribute', {}, True, [], False, ''),
        ('outputs:execOut', 'execution', 0, None, 'exec', {}, True, None, False, ''),
    ])

    @classmethod
    def _populate_role_data(cls):
        """Populate a role structure with the non-default roles on this node type"""
        role_data = super()._populate_role_data()
        role_data.inputs.attribute = og.AttributeRole.TEXT
        role_data.inputs.distribution = og.AttributeRole.TEXT
        role_data.inputs.execIn = og.AttributeRole.EXECUTION
        role_data.inputs.operation = og.AttributeRole.TEXT
        role_data.inputs.prims = og.AttributeRole.TEXT
        role_data.outputs.execOut = og.AttributeRole.EXECUTION
        return role_data

    class ValuesForInputs(og.DynamicAttributeAccess):
        LOCAL_PROPERTY_NAMES = {"attribute", "distribution", "execIn", "num_buckets", "on_reset", "operation", "prims", "_setting_locked", "_batchedReadAttributes", "_batchedReadValues"}
        """Helper class that creates natural hierarchical access to input attributes"""
        def __init__(self, node: og.Node, attributes, dynamic_attributes: og.DynamicAttributeInterface):
            """Initialize simplified access for the attribute data"""
            context = node.get_graph().get_default_graph_context()
            super().__init__(context, node, attributes, dynamic_attributes)
            self._batchedReadAttributes = [self._attributes.attribute, self._attributes.distribution, self._attributes.execIn, self._attributes.num_buckets, self._attributes.on_reset, self._attributes.operation, self._attributes.prims]
            self._batchedReadValues = ["", "", None, 0, False, "", ""]

        @property
        def dist_param_1(self):
            data_view = og.AttributeValueHelper(self._attributes.dist_param_1)
            return data_view.get()

        @dist_param_1.setter
        def dist_param_1(self, value):
            if self._setting_locked:
                raise og.ReadOnlyError(self._attributes.dist_param_1)
            data_view = og.AttributeValueHelper(self._attributes.dist_param_1)
            data_view.set(value)
            self.dist_param_1_size = data_view.get_array_size()

        @property
        def dist_param_2(self):
            data_view = og.AttributeValueHelper(self._attributes.dist_param_2)
            return data_view.get()

        @dist_param_2.setter
        def dist_param_2(self, value):
            if self._setting_locked:
                raise og.ReadOnlyError(self._attributes.dist_param_2)
            data_view = og.AttributeValueHelper(self._attributes.dist_param_2)
            data_view.set(value)
            self.dist_param_2_size = data_view.get_array_size()

        @property
        def indices(self):
            data_view = og.AttributeValueHelper(self._attributes.indices)
            return data_view.get()

        @indices.setter
        def indices(self, value):
            if self._setting_locked:
                raise og.ReadOnlyError(self._attributes.indices)
            data_view = og.AttributeValueHelper(self._attributes.indices)
            data_view.set(value)
            self.indices_size = data_view.get_array_size()

        @property
        def values(self):
            data_view = og.AttributeValueHelper(self._attributes.values)
            return data_view.get()

        @values.setter
        def values(self, value):
            if self._setting_locked:
                raise og.ReadOnlyError(self._attributes.values)
            data_view = og.AttributeValueHelper(self._attributes.values)
            data_view.set(value)
            self.values_size = data_view.get_array_size()

        @property
        def attribute(self):
            return self._batchedReadValues[0]

        @attribute.setter
        def attribute(self, value):
            self._batchedReadValues[0] = value

        @property
        def distribution(self):
            return self._batchedReadValues[1]

        @distribution.setter
        def distribution(self, value):
            self._batchedReadValues[1] = value

        @property
        def execIn(self):
            return self._batchedReadValues[2]

        @execIn.setter
        def execIn(self, value):
            self._batchedReadValues[2] = value

        @property
        def num_buckets(self):
            return self._batchedReadValues[3]

        @num_buckets.setter
        def num_buckets(self, value):
            self._batchedReadValues[3] = value

        @property
        def on_reset(self):
            return self._batchedReadValues[4]

        @on_reset.setter
        def on_reset(self, value):
            self._batchedReadValues[4] = value

        @property
        def operation(self):
            return self._batchedReadValues[5]

        @operation.setter
        def operation(self, value):
            self._batchedReadValues[5] = value

        @property
        def prims(self):
            return self._batchedReadValues[6]

        @prims.setter
        def prims(self, value):
            self._batchedReadValues[6] = value

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
        LOCAL_PROPERTY_NAMES = {"execOut", "_batchedWriteValues"}
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
        self.inputs = OgnWritePhysicsRigidPrimViewDatabase.ValuesForInputs(node, self.attributes.inputs, dynamic_attributes)
        dynamic_attributes = self.dynamic_attribute_data(node, og.AttributePortType.ATTRIBUTE_PORT_TYPE_OUTPUT)
        self.outputs = OgnWritePhysicsRigidPrimViewDatabase.ValuesForOutputs(node, self.attributes.outputs, dynamic_attributes)
        dynamic_attributes = self.dynamic_attribute_data(node, og.AttributePortType.ATTRIBUTE_PORT_TYPE_STATE)
        self.state = OgnWritePhysicsRigidPrimViewDatabase.ValuesForState(node, self.attributes.state, dynamic_attributes)

    class abi:
        """Class defining the ABI interface for the node type"""

        @staticmethod
        def get_node_type():
            get_node_type_function = getattr(OgnWritePhysicsRigidPrimViewDatabase.NODE_TYPE_CLASS, 'get_node_type', None)
            if callable(get_node_type_function):  # pragma: no cover
                return get_node_type_function()
            return 'omni.replicator.isaac.OgnWritePhysicsRigidPrimView'

        @staticmethod
        def compute(context, node):
            def database_valid():
                return True
            try:
                per_node_data = OgnWritePhysicsRigidPrimViewDatabase.PER_NODE_DATA[node.node_id()]
                db = per_node_data.get('_db')
                if db is None:
                    db = OgnWritePhysicsRigidPrimViewDatabase(node)
                    per_node_data['_db'] = db
                if not database_valid():
                    per_node_data['_db'] = None
                    return False
            except:
                db = OgnWritePhysicsRigidPrimViewDatabase(node)

            try:
                compute_function = getattr(OgnWritePhysicsRigidPrimViewDatabase.NODE_TYPE_CLASS, 'compute', None)
                if callable(compute_function) and compute_function.__code__.co_argcount > 1:  # pragma: no cover
                    return compute_function(context, node)

                db.inputs._prefetch()
                db.inputs._setting_locked = True
                with og.in_compute():
                    return OgnWritePhysicsRigidPrimViewDatabase.NODE_TYPE_CLASS.compute(db)
            except Exception as error:  # pragma: no cover
                stack_trace = "".join(traceback.format_tb(sys.exc_info()[2].tb_next))
                db.log_error(f'Assertion raised in compute - {error}\n{stack_trace}', add_context=False)
            finally:
                db.inputs._setting_locked = False
                db.outputs._commit()
            return False

        @staticmethod
        def initialize(context, node):
            OgnWritePhysicsRigidPrimViewDatabase._initialize_per_node_data(node)
            initialize_function = getattr(OgnWritePhysicsRigidPrimViewDatabase.NODE_TYPE_CLASS, 'initialize', None)
            if callable(initialize_function):  # pragma: no cover
                initialize_function(context, node)

            per_node_data = OgnWritePhysicsRigidPrimViewDatabase.PER_NODE_DATA[node.node_id()]

            def on_connection_or_disconnection(*args):
                per_node_data['_db'] = None

            node.register_on_connected_callback(on_connection_or_disconnection)
            node.register_on_disconnected_callback(on_connection_or_disconnection)

        @staticmethod
        def release(node):
            release_function = getattr(OgnWritePhysicsRigidPrimViewDatabase.NODE_TYPE_CLASS, 'release', None)
            if callable(release_function):  # pragma: no cover
                release_function(node)
            OgnWritePhysicsRigidPrimViewDatabase._release_per_node_data(node)

        @staticmethod
        def init_instance(node, graph_instance_id):
            init_instance_function = getattr(OgnWritePhysicsRigidPrimViewDatabase.NODE_TYPE_CLASS, 'init_instance', None)
            if callable(init_instance_function):  # pragma: no cover
                init_instance_function(node, graph_instance_id)

        @staticmethod
        def release_instance(node, graph_instance_id):
            release_instance_function = getattr(OgnWritePhysicsRigidPrimViewDatabase.NODE_TYPE_CLASS, 'release_instance', None)
            if callable(release_instance_function):  # pragma: no cover
                release_instance_function(node, graph_instance_id)
            OgnWritePhysicsRigidPrimViewDatabase._release_per_node_instance_data(node, graph_instance_id)

        @staticmethod
        def update_node_version(context, node, old_version, new_version):
            update_node_version_function = getattr(OgnWritePhysicsRigidPrimViewDatabase.NODE_TYPE_CLASS, 'update_node_version', None)
            if callable(update_node_version_function):  # pragma: no cover
                return update_node_version_function(context, node, old_version, new_version)
            return False

        @staticmethod
        def initialize_type(node_type):
            initialize_type_function = getattr(OgnWritePhysicsRigidPrimViewDatabase.NODE_TYPE_CLASS, 'initialize_type', None)
            needs_initializing = True
            if callable(initialize_type_function):  # pragma: no cover
                needs_initializing = initialize_type_function(node_type)
            if needs_initializing:
                node_type.set_metadata(ogn.MetadataKeys.EXTENSION, "omni.replicator.isaac")
                node_type.set_metadata(ogn.MetadataKeys.UI_NAME, "Write Physics Attribute using Tensor API")
                node_type.set_metadata(ogn.MetadataKeys.CATEGORIES, "Replicator")
                node_type.set_metadata(ogn.MetadataKeys.CATEGORY_DESCRIPTIONS, "Replicator,Write Attribute")
                node_type.set_metadata(ogn.MetadataKeys.DESCRIPTION, "This node writes physics attributes to TensorAPI views")
                node_type.set_metadata(ogn.MetadataKeys.EXCLUSIONS, "tests")
                node_type.set_metadata(ogn.MetadataKeys.LANGUAGE, "Python")
                OgnWritePhysicsRigidPrimViewDatabase.INTERFACE.add_to_node_type(node_type)

        @staticmethod
        def on_connection_type_resolve(node):
            on_connection_type_resolve_function = getattr(OgnWritePhysicsRigidPrimViewDatabase.NODE_TYPE_CLASS, 'on_connection_type_resolve', None)
            if callable(on_connection_type_resolve_function):  # pragma: no cover
                on_connection_type_resolve_function(node)

    NODE_TYPE_CLASS = None

    @staticmethod
    def register(node_type_class):
        OgnWritePhysicsRigidPrimViewDatabase.NODE_TYPE_CLASS = node_type_class
        og.register_node_type(OgnWritePhysicsRigidPrimViewDatabase.abi, 1)

    @staticmethod
    def deregister():
        og.deregister_node_type("omni.replicator.isaac.OgnWritePhysicsRigidPrimView")
