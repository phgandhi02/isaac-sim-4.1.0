"""Support for simplified access to data on nodes of type omni.physx.cct.OgnSpawnCapsule

 __   ___ .  .  ___  __       ___  ___  __      __   __   __   ___
/ _` |__  |\ | |__  |__)  /\   |  |__  |  \    /  ` /  \ |  \ |__
\__| |___ | \| |___ |  \ /--\  |  |___ |__/    \__, \__/ |__/ |___

 __   __     .  .  __  ___     .  .  __   __     ___
|  \ /  \    |\ | /  \  |      |\/| /  \ |  \ | |__  \ /
|__/ \__/    | \| \__/  |      |  | \__/ |__/ | |     |

Spawn a Capsule prim with stage-defined up axis to be used with a Character Controller
"""

import numpy
import sys
import traceback

import omni.graph.core as og
import omni.graph.core._omni_graph_core as _og
import omni.graph.tools.ogn as ogn



class OgnSpawnCapsuleDatabase(og.Database):
    """Helper class providing simplified access to data on nodes of type omni.physx.cct.OgnSpawnCapsule

    Class Members:
        node: Node being evaluated

    Attribute Value Properties:
        Inputs:
            inputs.capsuleHeight
            inputs.capsulePos
            inputs.capsuleRadius
            inputs.spawn
        Outputs:
            outputs.done
            outputs.path
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
        ('inputs:capsuleHeight', 'float', 0, 'Height', 'Capsule Height', {ogn.MetadataKeys.DEFAULT: '100'}, True, 100, False, ''),
        ('inputs:capsulePos', 'float3', 0, 'Position', 'Capsule Position', {ogn.MetadataKeys.DEFAULT: '[0, 0, 0]'}, True, [0, 0, 0], False, ''),
        ('inputs:capsuleRadius', 'float', 0, 'Radius', 'Capsule Radius', {ogn.MetadataKeys.DEFAULT: '50'}, True, 50, False, ''),
        ('inputs:spawn', 'execution', 0, None, 'Spawn', {}, True, None, False, ''),
        ('outputs:done', 'execution', 0, 'Done', 'Activated after the capsule is spawned', {}, True, None, False, ''),
        ('outputs:path', 'path', 0, 'Path', 'Path', {}, True, None, False, ''),
    ])

    @classmethod
    def _populate_role_data(cls):
        """Populate a role structure with the non-default roles on this node type"""
        role_data = super()._populate_role_data()
        role_data.inputs.spawn = og.AttributeRole.EXECUTION
        role_data.outputs.done = og.AttributeRole.EXECUTION
        role_data.outputs.path = og.AttributeRole.PATH
        return role_data

    class ValuesForInputs(og.DynamicAttributeAccess):
        LOCAL_PROPERTY_NAMES = {"capsuleHeight", "capsulePos", "capsuleRadius", "spawn", "_setting_locked", "_batchedReadAttributes", "_batchedReadValues"}
        """Helper class that creates natural hierarchical access to input attributes"""
        def __init__(self, node: og.Node, attributes, dynamic_attributes: og.DynamicAttributeInterface):
            """Initialize simplified access for the attribute data"""
            context = node.get_graph().get_default_graph_context()
            super().__init__(context, node, attributes, dynamic_attributes)
            self._batchedReadAttributes = [self._attributes.capsuleHeight, self._attributes.capsulePos, self._attributes.capsuleRadius, self._attributes.spawn]
            self._batchedReadValues = [100, [0, 0, 0], 50, None]

        @property
        def capsuleHeight(self):
            return self._batchedReadValues[0]

        @capsuleHeight.setter
        def capsuleHeight(self, value):
            self._batchedReadValues[0] = value

        @property
        def capsulePos(self):
            return self._batchedReadValues[1]

        @capsulePos.setter
        def capsulePos(self, value):
            self._batchedReadValues[1] = value

        @property
        def capsuleRadius(self):
            return self._batchedReadValues[2]

        @capsuleRadius.setter
        def capsuleRadius(self, value):
            self._batchedReadValues[2] = value

        @property
        def spawn(self):
            return self._batchedReadValues[3]

        @spawn.setter
        def spawn(self, value):
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
        LOCAL_PROPERTY_NAMES = {"done", "path", "_batchedWriteValues"}
        """Helper class that creates natural hierarchical access to output attributes"""
        def __init__(self, node: og.Node, attributes, dynamic_attributes: og.DynamicAttributeInterface):
            """Initialize simplified access for the attribute data"""
            context = node.get_graph().get_default_graph_context()
            super().__init__(context, node, attributes, dynamic_attributes)
            self.path_size = None
            self._batchedWriteValues = { }

        @property
        def done(self):
            value = self._batchedWriteValues.get(self._attributes.done)
            if value:
                return value
            else:
                data_view = og.AttributeValueHelper(self._attributes.done)
                return data_view.get()

        @done.setter
        def done(self, value):
            self._batchedWriteValues[self._attributes.done] = value

        @property
        def path(self):
            value = self._batchedWriteValues.get(self._attributes.path)
            if value:
                return value
            else:
                data_view = og.AttributeValueHelper(self._attributes.path)
                return data_view.get()

        @path.setter
        def path(self, value):
            self._batchedWriteValues[self._attributes.path] = value

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
        self.inputs = OgnSpawnCapsuleDatabase.ValuesForInputs(node, self.attributes.inputs, dynamic_attributes)
        dynamic_attributes = self.dynamic_attribute_data(node, og.AttributePortType.ATTRIBUTE_PORT_TYPE_OUTPUT)
        self.outputs = OgnSpawnCapsuleDatabase.ValuesForOutputs(node, self.attributes.outputs, dynamic_attributes)
        dynamic_attributes = self.dynamic_attribute_data(node, og.AttributePortType.ATTRIBUTE_PORT_TYPE_STATE)
        self.state = OgnSpawnCapsuleDatabase.ValuesForState(node, self.attributes.state, dynamic_attributes)

    class abi:
        """Class defining the ABI interface for the node type"""

        @staticmethod
        def get_node_type():
            get_node_type_function = getattr(OgnSpawnCapsuleDatabase.NODE_TYPE_CLASS, 'get_node_type', None)
            if callable(get_node_type_function):  # pragma: no cover
                return get_node_type_function()
            return 'omni.physx.cct.OgnSpawnCapsule'

        @staticmethod
        def compute(context, node):
            def database_valid():
                return True
            try:
                per_node_data = OgnSpawnCapsuleDatabase.PER_NODE_DATA[node.node_id()]
                db = per_node_data.get('_db')
                if db is None:
                    db = OgnSpawnCapsuleDatabase(node)
                    per_node_data['_db'] = db
                if not database_valid():
                    per_node_data['_db'] = None
                    return False
            except:
                db = OgnSpawnCapsuleDatabase(node)

            try:
                compute_function = getattr(OgnSpawnCapsuleDatabase.NODE_TYPE_CLASS, 'compute', None)
                if callable(compute_function) and compute_function.__code__.co_argcount > 1:  # pragma: no cover
                    return compute_function(context, node)

                db.inputs._prefetch()
                db.inputs._setting_locked = True
                with og.in_compute():
                    return OgnSpawnCapsuleDatabase.NODE_TYPE_CLASS.compute(db)
            except Exception as error:  # pragma: no cover
                stack_trace = "".join(traceback.format_tb(sys.exc_info()[2].tb_next))
                db.log_error(f'Assertion raised in compute - {error}\n{stack_trace}', add_context=False)
            finally:
                db.inputs._setting_locked = False
                db.outputs._commit()
            return False

        @staticmethod
        def initialize(context, node):
            OgnSpawnCapsuleDatabase._initialize_per_node_data(node)
            initialize_function = getattr(OgnSpawnCapsuleDatabase.NODE_TYPE_CLASS, 'initialize', None)
            if callable(initialize_function):  # pragma: no cover
                initialize_function(context, node)

            per_node_data = OgnSpawnCapsuleDatabase.PER_NODE_DATA[node.node_id()]

            def on_connection_or_disconnection(*args):
                per_node_data['_db'] = None

            node.register_on_connected_callback(on_connection_or_disconnection)
            node.register_on_disconnected_callback(on_connection_or_disconnection)

        @staticmethod
        def release(node):
            release_function = getattr(OgnSpawnCapsuleDatabase.NODE_TYPE_CLASS, 'release', None)
            if callable(release_function):  # pragma: no cover
                release_function(node)
            OgnSpawnCapsuleDatabase._release_per_node_data(node)

        @staticmethod
        def init_instance(node, graph_instance_id):
            init_instance_function = getattr(OgnSpawnCapsuleDatabase.NODE_TYPE_CLASS, 'init_instance', None)
            if callable(init_instance_function):  # pragma: no cover
                init_instance_function(node, graph_instance_id)

        @staticmethod
        def release_instance(node, graph_instance_id):
            release_instance_function = getattr(OgnSpawnCapsuleDatabase.NODE_TYPE_CLASS, 'release_instance', None)
            if callable(release_instance_function):  # pragma: no cover
                release_instance_function(node, graph_instance_id)
            OgnSpawnCapsuleDatabase._release_per_node_instance_data(node, graph_instance_id)

        @staticmethod
        def update_node_version(context, node, old_version, new_version):
            update_node_version_function = getattr(OgnSpawnCapsuleDatabase.NODE_TYPE_CLASS, 'update_node_version', None)
            if callable(update_node_version_function):  # pragma: no cover
                return update_node_version_function(context, node, old_version, new_version)
            return False

        @staticmethod
        def initialize_type(node_type):
            initialize_type_function = getattr(OgnSpawnCapsuleDatabase.NODE_TYPE_CLASS, 'initialize_type', None)
            needs_initializing = True
            if callable(initialize_type_function):  # pragma: no cover
                needs_initializing = initialize_type_function(node_type)
            if needs_initializing:
                node_type.set_metadata(ogn.MetadataKeys.EXTENSION, "omni.physx.cct")
                node_type.set_metadata(ogn.MetadataKeys.UI_NAME, "Spawn Capsule")
                node_type.set_metadata(ogn.MetadataKeys.CATEGORIES, "Physx Character Controller")
                node_type.set_metadata(ogn.MetadataKeys.DESCRIPTION, "Spawn a Capsule prim with stage-defined up axis to be used with a Character Controller")
                node_type.set_metadata(ogn.MetadataKeys.LANGUAGE, "Python")
                __hints = node_type.get_scheduling_hints()
                if __hints is not None:
                    __hints.set_data_access(og.eAccessLocation.E_GLOBAL, og.eAccessType.E_WRITE)
                    __hints.set_data_access(og.eAccessLocation.E_USD, og.eAccessType.E_READ_WRITE)
                OgnSpawnCapsuleDatabase.INTERFACE.add_to_node_type(node_type)

        @staticmethod
        def on_connection_type_resolve(node):
            on_connection_type_resolve_function = getattr(OgnSpawnCapsuleDatabase.NODE_TYPE_CLASS, 'on_connection_type_resolve', None)
            if callable(on_connection_type_resolve_function):  # pragma: no cover
                on_connection_type_resolve_function(node)

    NODE_TYPE_CLASS = None

    @staticmethod
    def register(node_type_class):
        OgnSpawnCapsuleDatabase.NODE_TYPE_CLASS = node_type_class
        og.register_node_type(OgnSpawnCapsuleDatabase.abi, 1)

    @staticmethod
    def deregister():
        og.deregister_node_type("omni.physx.cct.OgnSpawnCapsule")
