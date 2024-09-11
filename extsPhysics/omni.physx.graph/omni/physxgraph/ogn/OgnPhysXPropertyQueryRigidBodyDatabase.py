"""Support for simplified access to data on nodes of type omni.physx.graph.PropertyQueryRigidBody

 __   ___ .  .  ___  __       ___  ___  __      __   __   __   ___
/ _` |__  |\ | |__  |__)  /\   |  |__  |  \    /  ` /  \ |  \ |__
\__| |___ | \| |___ |  \ /--\  |  |___ |__/    \__, \__/ |__/ |___

 __   __     .  .  __  ___     .  .  __   __     ___
|  \ /  \    |\ | /  \  |      |\/| /  \ |  \ | |__  \ /
|__/ \__/    | \| \__/  |      |  | \__/ |__/ | |     |

Returns the physics-related properties of an input rigid body.
"""

import numpy
import sys
import traceback
import usdrt

import omni.graph.core as og
import omni.graph.core._omni_graph_core as _og
import omni.graph.tools.ogn as ogn



class OgnPhysXPropertyQueryRigidBodyDatabase(og.Database):
    """Helper class providing simplified access to data on nodes of type omni.physx.graph.PropertyQueryRigidBody

    Class Members:
        node: Node being evaluated

    Attribute Value Properties:
        Inputs:
            inputs.prim
            inputs.timeout
            inputs.use_local_frame
        Outputs:
            outputs.AABBMax
            outputs.AABBMin
            outputs.centerOfMass
            outputs.diagonalInertia
            outputs.mass
            outputs.principalAxes
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
        ('inputs:prim', 'target', 0, 'Prim', 'Prim to query the properties of. This must be a rigid body.', {}, True, None, False, ''),
        ('inputs:timeout', 'int', 0, 'Timeout', 'Query timeout in milliseconds.', {ogn.MetadataKeys.LITERAL_ONLY: '1', ogn.MetadataKeys.DEFAULT: '10000'}, True, 10000, False, ''),
        ('inputs:use_local_frame', 'bool', 0, 'Output in local frame', "If set to true, output coordinates are provided in the rigid body's local frame of reference.", {ogn.MetadataKeys.LITERAL_ONLY: '1', ogn.MetadataKeys.DEFAULT: 'false'}, True, False, False, ''),
        ('outputs:AABBMax', 'float3', 0, 'Bounds max', 'A list of upper bounds coordinates of the colliders of the input rigid body.', {}, True, None, False, ''),
        ('outputs:AABBMin', 'float3', 0, 'Bounds min', 'A list of lower bounds coordinates of the colliders of the input rigid body.', {}, True, None, False, ''),
        ('outputs:centerOfMass', 'float3', 0, 'Center of mass', 'The center of mass of the input rigid body', {}, True, None, False, ''),
        ('outputs:diagonalInertia', 'float3', 0, 'Diagonal inertia', 'The diagonal inertia of the input rigid body (note: the return value is the inertia tensor relative to the principal axes.)', {}, True, None, False, ''),
        ('outputs:mass', 'float', 0, 'Mass', 'The total mass of the input rigid body.', {}, True, None, False, ''),
        ('outputs:principalAxes', 'float4', 0, 'Principal axes', "The principal axes for the inertia tensor of the input rigid body (note: the return value is always in the rigid body's local frame.)", {}, True, None, False, ''),
    ])

    @classmethod
    def _populate_role_data(cls):
        """Populate a role structure with the non-default roles on this node type"""
        role_data = super()._populate_role_data()
        role_data.inputs.prim = og.AttributeRole.TARGET
        return role_data

    class ValuesForInputs(og.DynamicAttributeAccess):
        LOCAL_PROPERTY_NAMES = {"timeout", "use_local_frame", "_setting_locked", "_batchedReadAttributes", "_batchedReadValues"}
        """Helper class that creates natural hierarchical access to input attributes"""
        def __init__(self, node: og.Node, attributes, dynamic_attributes: og.DynamicAttributeInterface):
            """Initialize simplified access for the attribute data"""
            context = node.get_graph().get_default_graph_context()
            super().__init__(context, node, attributes, dynamic_attributes)
            self._batchedReadAttributes = [self._attributes.timeout, self._attributes.use_local_frame]
            self._batchedReadValues = [10000, False]

        @property
        def prim(self):
            data_view = og.AttributeValueHelper(self._attributes.prim)
            return data_view.get()

        @prim.setter
        def prim(self, value):
            if self._setting_locked:
                raise og.ReadOnlyError(self._attributes.prim)
            data_view = og.AttributeValueHelper(self._attributes.prim)
            data_view.set(value)
            self.prim_size = data_view.get_array_size()

        @property
        def timeout(self):
            return self._batchedReadValues[0]

        @timeout.setter
        def timeout(self, value):
            self._batchedReadValues[0] = value

        @property
        def use_local_frame(self):
            return self._batchedReadValues[1]

        @use_local_frame.setter
        def use_local_frame(self, value):
            self._batchedReadValues[1] = value

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
        LOCAL_PROPERTY_NAMES = {"AABBMax", "AABBMin", "centerOfMass", "diagonalInertia", "mass", "principalAxes", "_batchedWriteValues"}
        """Helper class that creates natural hierarchical access to output attributes"""
        def __init__(self, node: og.Node, attributes, dynamic_attributes: og.DynamicAttributeInterface):
            """Initialize simplified access for the attribute data"""
            context = node.get_graph().get_default_graph_context()
            super().__init__(context, node, attributes, dynamic_attributes)
            self._batchedWriteValues = { }

        @property
        def AABBMax(self):
            value = self._batchedWriteValues.get(self._attributes.AABBMax)
            if value:
                return value
            else:
                data_view = og.AttributeValueHelper(self._attributes.AABBMax)
                return data_view.get()

        @AABBMax.setter
        def AABBMax(self, value):
            self._batchedWriteValues[self._attributes.AABBMax] = value

        @property
        def AABBMin(self):
            value = self._batchedWriteValues.get(self._attributes.AABBMin)
            if value:
                return value
            else:
                data_view = og.AttributeValueHelper(self._attributes.AABBMin)
                return data_view.get()

        @AABBMin.setter
        def AABBMin(self, value):
            self._batchedWriteValues[self._attributes.AABBMin] = value

        @property
        def centerOfMass(self):
            value = self._batchedWriteValues.get(self._attributes.centerOfMass)
            if value:
                return value
            else:
                data_view = og.AttributeValueHelper(self._attributes.centerOfMass)
                return data_view.get()

        @centerOfMass.setter
        def centerOfMass(self, value):
            self._batchedWriteValues[self._attributes.centerOfMass] = value

        @property
        def diagonalInertia(self):
            value = self._batchedWriteValues.get(self._attributes.diagonalInertia)
            if value:
                return value
            else:
                data_view = og.AttributeValueHelper(self._attributes.diagonalInertia)
                return data_view.get()

        @diagonalInertia.setter
        def diagonalInertia(self, value):
            self._batchedWriteValues[self._attributes.diagonalInertia] = value

        @property
        def mass(self):
            value = self._batchedWriteValues.get(self._attributes.mass)
            if value:
                return value
            else:
                data_view = og.AttributeValueHelper(self._attributes.mass)
                return data_view.get()

        @mass.setter
        def mass(self, value):
            self._batchedWriteValues[self._attributes.mass] = value

        @property
        def principalAxes(self):
            value = self._batchedWriteValues.get(self._attributes.principalAxes)
            if value:
                return value
            else:
                data_view = og.AttributeValueHelper(self._attributes.principalAxes)
                return data_view.get()

        @principalAxes.setter
        def principalAxes(self, value):
            self._batchedWriteValues[self._attributes.principalAxes] = value

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
        self.inputs = OgnPhysXPropertyQueryRigidBodyDatabase.ValuesForInputs(node, self.attributes.inputs, dynamic_attributes)
        dynamic_attributes = self.dynamic_attribute_data(node, og.AttributePortType.ATTRIBUTE_PORT_TYPE_OUTPUT)
        self.outputs = OgnPhysXPropertyQueryRigidBodyDatabase.ValuesForOutputs(node, self.attributes.outputs, dynamic_attributes)
        dynamic_attributes = self.dynamic_attribute_data(node, og.AttributePortType.ATTRIBUTE_PORT_TYPE_STATE)
        self.state = OgnPhysXPropertyQueryRigidBodyDatabase.ValuesForState(node, self.attributes.state, dynamic_attributes)

    class abi:
        """Class defining the ABI interface for the node type"""

        @staticmethod
        def get_node_type():
            get_node_type_function = getattr(OgnPhysXPropertyQueryRigidBodyDatabase.NODE_TYPE_CLASS, 'get_node_type', None)
            if callable(get_node_type_function):  # pragma: no cover
                return get_node_type_function()
            return 'omni.physx.graph.PropertyQueryRigidBody'

        @staticmethod
        def compute(context, node):
            def database_valid():
                return True
            try:
                per_node_data = OgnPhysXPropertyQueryRigidBodyDatabase.PER_NODE_DATA[node.node_id()]
                db = per_node_data.get('_db')
                if db is None:
                    db = OgnPhysXPropertyQueryRigidBodyDatabase(node)
                    per_node_data['_db'] = db
                if not database_valid():
                    per_node_data['_db'] = None
                    return False
            except:
                db = OgnPhysXPropertyQueryRigidBodyDatabase(node)

            try:
                compute_function = getattr(OgnPhysXPropertyQueryRigidBodyDatabase.NODE_TYPE_CLASS, 'compute', None)
                if callable(compute_function) and compute_function.__code__.co_argcount > 1:  # pragma: no cover
                    return compute_function(context, node)

                db.inputs._prefetch()
                db.inputs._setting_locked = True
                with og.in_compute():
                    return OgnPhysXPropertyQueryRigidBodyDatabase.NODE_TYPE_CLASS.compute(db)
            except Exception as error:  # pragma: no cover
                stack_trace = "".join(traceback.format_tb(sys.exc_info()[2].tb_next))
                db.log_error(f'Assertion raised in compute - {error}\n{stack_trace}', add_context=False)
            finally:
                db.inputs._setting_locked = False
                db.outputs._commit()
            return False

        @staticmethod
        def initialize(context, node):
            OgnPhysXPropertyQueryRigidBodyDatabase._initialize_per_node_data(node)
            initialize_function = getattr(OgnPhysXPropertyQueryRigidBodyDatabase.NODE_TYPE_CLASS, 'initialize', None)
            if callable(initialize_function):  # pragma: no cover
                initialize_function(context, node)

            per_node_data = OgnPhysXPropertyQueryRigidBodyDatabase.PER_NODE_DATA[node.node_id()]

            def on_connection_or_disconnection(*args):
                per_node_data['_db'] = None

            node.register_on_connected_callback(on_connection_or_disconnection)
            node.register_on_disconnected_callback(on_connection_or_disconnection)

        @staticmethod
        def release(node):
            release_function = getattr(OgnPhysXPropertyQueryRigidBodyDatabase.NODE_TYPE_CLASS, 'release', None)
            if callable(release_function):  # pragma: no cover
                release_function(node)
            OgnPhysXPropertyQueryRigidBodyDatabase._release_per_node_data(node)

        @staticmethod
        def init_instance(node, graph_instance_id):
            init_instance_function = getattr(OgnPhysXPropertyQueryRigidBodyDatabase.NODE_TYPE_CLASS, 'init_instance', None)
            if callable(init_instance_function):  # pragma: no cover
                init_instance_function(node, graph_instance_id)

        @staticmethod
        def release_instance(node, graph_instance_id):
            release_instance_function = getattr(OgnPhysXPropertyQueryRigidBodyDatabase.NODE_TYPE_CLASS, 'release_instance', None)
            if callable(release_instance_function):  # pragma: no cover
                release_instance_function(node, graph_instance_id)
            OgnPhysXPropertyQueryRigidBodyDatabase._release_per_node_instance_data(node, graph_instance_id)

        @staticmethod
        def update_node_version(context, node, old_version, new_version):
            update_node_version_function = getattr(OgnPhysXPropertyQueryRigidBodyDatabase.NODE_TYPE_CLASS, 'update_node_version', None)
            if callable(update_node_version_function):  # pragma: no cover
                return update_node_version_function(context, node, old_version, new_version)
            return False

        @staticmethod
        def initialize_type(node_type):
            initialize_type_function = getattr(OgnPhysXPropertyQueryRigidBodyDatabase.NODE_TYPE_CLASS, 'initialize_type', None)
            needs_initializing = True
            if callable(initialize_type_function):  # pragma: no cover
                needs_initializing = initialize_type_function(node_type)
            if needs_initializing:
                node_type.set_metadata(ogn.MetadataKeys.EXTENSION, "omni.physx.graph")
                node_type.set_metadata(ogn.MetadataKeys.TAGS, "physics,physx,simulation,rigid body")
                node_type.set_metadata(ogn.MetadataKeys.UI_NAME, "Get Rigid Body Properties")
                node_type.set_metadata(ogn.MetadataKeys.TOKENS, "{}")
                node_type.set_metadata(ogn.MetadataKeys.CATEGORIES, "PhysX Property Queries")
                node_type.set_metadata(ogn.MetadataKeys.DESCRIPTION, "Returns the physics-related properties of an input rigid body.")
                node_type.set_metadata(ogn.MetadataKeys.LANGUAGE, "Python")
                __hints = node_type.get_scheduling_hints()
                if __hints is not None:
                    __hints.set_data_access(og.eAccessLocation.E_GLOBAL, og.eAccessType.E_WRITE)
                    __hints.set_data_access(og.eAccessLocation.E_USD, og.eAccessType.E_READ_WRITE)
                OgnPhysXPropertyQueryRigidBodyDatabase.INTERFACE.add_to_node_type(node_type)

        @staticmethod
        def on_connection_type_resolve(node):
            on_connection_type_resolve_function = getattr(OgnPhysXPropertyQueryRigidBodyDatabase.NODE_TYPE_CLASS, 'on_connection_type_resolve', None)
            if callable(on_connection_type_resolve_function):  # pragma: no cover
                on_connection_type_resolve_function(node)

    NODE_TYPE_CLASS = None

    @staticmethod
    def register(node_type_class):
        OgnPhysXPropertyQueryRigidBodyDatabase.NODE_TYPE_CLASS = node_type_class
        og.register_node_type(OgnPhysXPropertyQueryRigidBodyDatabase.abi, 1)

    @staticmethod
    def deregister():
        og.deregister_node_type("omni.physx.graph.PropertyQueryRigidBody")
