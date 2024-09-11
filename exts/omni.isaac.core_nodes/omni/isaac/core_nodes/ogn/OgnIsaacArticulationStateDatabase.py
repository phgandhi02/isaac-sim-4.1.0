"""Support for simplified access to data on nodes of type omni.isaac.core_nodes.IsaacArticulationState

 __   ___ .  .  ___  __       ___  ___  __      __   __   __   ___
/ _` |__  |\ | |__  |__)  /\   |  |__  |  \    /  ` /  \ |  \ |__
\__| |___ | \| |___ |  \ /--\  |  |___ |__/    \__, \__/ |__/ |___

 __   __     .  .  __  ___     .  .  __   __     ___
|  \ /  \    |\ | /  \  |      |\/| /  \ |  \ | |__  \ /
|__/ \__/    | \| \__/  |      |  | \__/ |__/ | |     |

Articulated robot state. The node takes either joint names or joint indices, and outputs the joint positions and velocities,
as well as the measured joint efforts, forces, and torques.
"""

import numpy
import sys
import traceback
import usdrt

import omni.graph.core as og
import omni.graph.core._omni_graph_core as _og
import omni.graph.tools.ogn as ogn



class OgnIsaacArticulationStateDatabase(og.Database):
    """Helper class providing simplified access to data on nodes of type omni.isaac.core_nodes.IsaacArticulationState

    Class Members:
        node: Node being evaluated

    Attribute Value Properties:
        Inputs:
            inputs.execIn
            inputs.jointIndices
            inputs.jointNames
            inputs.robotPath
            inputs.targetPrim
        Outputs:
            outputs.jointNames
            outputs.jointPositions
            outputs.jointVelocities
            outputs.measuredJointEfforts
            outputs.measuredJointForces
            outputs.measuredJointTorques
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
        ('inputs:execIn', 'execution', 0, None, 'The input execution', {}, True, None, False, ''),
        ('inputs:jointIndices', 'int[]', 0, None, 'Queried joint indices, use either Joint Names or Joint Indices, if neither is given, default to all joints', {}, True, [], False, ''),
        ('inputs:jointNames', 'token[]', 0, None, 'Queried joint names, use either Joint Names or Joint Indices, if neither is given, default to all joints', {}, True, [], False, ''),
        ('inputs:robotPath', 'string', 0, None, 'Path to the robot articulation root. If this is populated, targetPrim is ignored.', {}, True, "", False, ''),
        ('inputs:targetPrim', 'target', 0, None, 'The target robot prim with robot articulation root. Ensure robotPath is empty for this to be considered.', {}, False, None, False, ''),
        ('outputs:jointNames', 'token[]', 0, None, 'Joint names', {}, True, None, False, ''),
        ('outputs:jointPositions', 'double[]', 0, None, 'Joint positions\nUnits:\n    linear (m)\n    angular (rad)', {}, True, None, False, ''),
        ('outputs:jointVelocities', 'double[]', 0, None, 'Joint velocities\nUnits:\n    linear (m/s)\n    angular (rad/s)', {}, True, None, False, ''),
        ('outputs:measuredJointEfforts', 'double[]', 0, None, 'Measured joint efforts\nForce Units:  \n    linear (kg*m/s^2) i.e. a force\n    angular (kg*m^2/s^2) i.e. a torque\nAcceleration Units:\n    linear (m/s^2), i.e. a linear acceleration\n    angular (rad/s^2) i.e. an angular acceleration', {}, True, None, False, ''),
        ('outputs:measuredJointForces', 'double3[]', 0, None, 'Measured joint reaction forces\nForce Units:  \n    linear (kg*m/s^2) i.e. a force\n    angular (kg*m^2/s^2) i.e. a torque\nAcceleration Units:\n    linear (m/s^2), i.e. a linear acceleration\n    angular (rad/s^2) i.e. an angular acceleration', {}, True, None, False, ''),
        ('outputs:measuredJointTorques', 'double3[]', 0, None, 'Measured joint reaction torques\nForce Units:  \n    linear (kg*m/s^2) i.e. a force\n    angular (kg*m^2/s^2) i.e. a torque\nAcceleration Units:\n    linear (m/s^2), i.e. a linear acceleration\n    angular (rad/s^2) i.e. an angular acceleration', {}, True, None, False, ''),
    ])

    @classmethod
    def _populate_role_data(cls):
        """Populate a role structure with the non-default roles on this node type"""
        role_data = super()._populate_role_data()
        role_data.inputs.execIn = og.AttributeRole.EXECUTION
        role_data.inputs.robotPath = og.AttributeRole.TEXT
        role_data.inputs.targetPrim = og.AttributeRole.TARGET
        return role_data

    class ValuesForInputs(og.DynamicAttributeAccess):
        LOCAL_PROPERTY_NAMES = {"execIn", "robotPath", "_setting_locked", "_batchedReadAttributes", "_batchedReadValues"}
        """Helper class that creates natural hierarchical access to input attributes"""
        def __init__(self, node: og.Node, attributes, dynamic_attributes: og.DynamicAttributeInterface):
            """Initialize simplified access for the attribute data"""
            context = node.get_graph().get_default_graph_context()
            super().__init__(context, node, attributes, dynamic_attributes)
            self._batchedReadAttributes = [self._attributes.execIn, self._attributes.robotPath]
            self._batchedReadValues = [None, ""]

        @property
        def jointIndices(self):
            data_view = og.AttributeValueHelper(self._attributes.jointIndices)
            return data_view.get()

        @jointIndices.setter
        def jointIndices(self, value):
            if self._setting_locked:
                raise og.ReadOnlyError(self._attributes.jointIndices)
            data_view = og.AttributeValueHelper(self._attributes.jointIndices)
            data_view.set(value)
            self.jointIndices_size = data_view.get_array_size()

        @property
        def jointNames(self):
            data_view = og.AttributeValueHelper(self._attributes.jointNames)
            return data_view.get()

        @jointNames.setter
        def jointNames(self, value):
            if self._setting_locked:
                raise og.ReadOnlyError(self._attributes.jointNames)
            data_view = og.AttributeValueHelper(self._attributes.jointNames)
            data_view.set(value)
            self.jointNames_size = data_view.get_array_size()

        @property
        def targetPrim(self):
            data_view = og.AttributeValueHelper(self._attributes.targetPrim)
            return data_view.get()

        @targetPrim.setter
        def targetPrim(self, value):
            if self._setting_locked:
                raise og.ReadOnlyError(self._attributes.targetPrim)
            data_view = og.AttributeValueHelper(self._attributes.targetPrim)
            data_view.set(value)
            self.targetPrim_size = data_view.get_array_size()

        @property
        def execIn(self):
            return self._batchedReadValues[0]

        @execIn.setter
        def execIn(self, value):
            self._batchedReadValues[0] = value

        @property
        def robotPath(self):
            return self._batchedReadValues[1]

        @robotPath.setter
        def robotPath(self, value):
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
        LOCAL_PROPERTY_NAMES = { }
        """Helper class that creates natural hierarchical access to output attributes"""
        def __init__(self, node: og.Node, attributes, dynamic_attributes: og.DynamicAttributeInterface):
            """Initialize simplified access for the attribute data"""
            context = node.get_graph().get_default_graph_context()
            super().__init__(context, node, attributes, dynamic_attributes)
            self.jointNames_size = None
            self.jointPositions_size = None
            self.jointVelocities_size = None
            self.measuredJointEfforts_size = None
            self.measuredJointForces_size = None
            self.measuredJointTorques_size = None
            self._batchedWriteValues = { }

        @property
        def jointNames(self):
            data_view = og.AttributeValueHelper(self._attributes.jointNames)
            return data_view.get(reserved_element_count=self.jointNames_size)

        @jointNames.setter
        def jointNames(self, value):
            data_view = og.AttributeValueHelper(self._attributes.jointNames)
            data_view.set(value)
            self.jointNames_size = data_view.get_array_size()

        @property
        def jointPositions(self):
            data_view = og.AttributeValueHelper(self._attributes.jointPositions)
            return data_view.get(reserved_element_count=self.jointPositions_size)

        @jointPositions.setter
        def jointPositions(self, value):
            data_view = og.AttributeValueHelper(self._attributes.jointPositions)
            data_view.set(value)
            self.jointPositions_size = data_view.get_array_size()

        @property
        def jointVelocities(self):
            data_view = og.AttributeValueHelper(self._attributes.jointVelocities)
            return data_view.get(reserved_element_count=self.jointVelocities_size)

        @jointVelocities.setter
        def jointVelocities(self, value):
            data_view = og.AttributeValueHelper(self._attributes.jointVelocities)
            data_view.set(value)
            self.jointVelocities_size = data_view.get_array_size()

        @property
        def measuredJointEfforts(self):
            data_view = og.AttributeValueHelper(self._attributes.measuredJointEfforts)
            return data_view.get(reserved_element_count=self.measuredJointEfforts_size)

        @measuredJointEfforts.setter
        def measuredJointEfforts(self, value):
            data_view = og.AttributeValueHelper(self._attributes.measuredJointEfforts)
            data_view.set(value)
            self.measuredJointEfforts_size = data_view.get_array_size()

        @property
        def measuredJointForces(self):
            data_view = og.AttributeValueHelper(self._attributes.measuredJointForces)
            return data_view.get(reserved_element_count=self.measuredJointForces_size)

        @measuredJointForces.setter
        def measuredJointForces(self, value):
            data_view = og.AttributeValueHelper(self._attributes.measuredJointForces)
            data_view.set(value)
            self.measuredJointForces_size = data_view.get_array_size()

        @property
        def measuredJointTorques(self):
            data_view = og.AttributeValueHelper(self._attributes.measuredJointTorques)
            return data_view.get(reserved_element_count=self.measuredJointTorques_size)

        @measuredJointTorques.setter
        def measuredJointTorques(self, value):
            data_view = og.AttributeValueHelper(self._attributes.measuredJointTorques)
            data_view.set(value)
            self.measuredJointTorques_size = data_view.get_array_size()

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
        self.inputs = OgnIsaacArticulationStateDatabase.ValuesForInputs(node, self.attributes.inputs, dynamic_attributes)
        dynamic_attributes = self.dynamic_attribute_data(node, og.AttributePortType.ATTRIBUTE_PORT_TYPE_OUTPUT)
        self.outputs = OgnIsaacArticulationStateDatabase.ValuesForOutputs(node, self.attributes.outputs, dynamic_attributes)
        dynamic_attributes = self.dynamic_attribute_data(node, og.AttributePortType.ATTRIBUTE_PORT_TYPE_STATE)
        self.state = OgnIsaacArticulationStateDatabase.ValuesForState(node, self.attributes.state, dynamic_attributes)

    class abi:
        """Class defining the ABI interface for the node type"""

        @staticmethod
        def get_node_type():
            get_node_type_function = getattr(OgnIsaacArticulationStateDatabase.NODE_TYPE_CLASS, 'get_node_type', None)
            if callable(get_node_type_function):  # pragma: no cover
                return get_node_type_function()
            return 'omni.isaac.core_nodes.IsaacArticulationState'

        @staticmethod
        def compute(context, node):
            def database_valid():
                return True
            try:
                per_node_data = OgnIsaacArticulationStateDatabase.PER_NODE_DATA[node.node_id()]
                db = per_node_data.get('_db')
                if db is None:
                    db = OgnIsaacArticulationStateDatabase(node)
                    per_node_data['_db'] = db
                if not database_valid():
                    per_node_data['_db'] = None
                    return False
            except:
                db = OgnIsaacArticulationStateDatabase(node)

            try:
                compute_function = getattr(OgnIsaacArticulationStateDatabase.NODE_TYPE_CLASS, 'compute', None)
                if callable(compute_function) and compute_function.__code__.co_argcount > 1:  # pragma: no cover
                    return compute_function(context, node)

                db.inputs._prefetch()
                db.inputs._setting_locked = True
                with og.in_compute():
                    return OgnIsaacArticulationStateDatabase.NODE_TYPE_CLASS.compute(db)
            except Exception as error:  # pragma: no cover
                stack_trace = "".join(traceback.format_tb(sys.exc_info()[2].tb_next))
                db.log_error(f'Assertion raised in compute - {error}\n{stack_trace}', add_context=False)
            finally:
                db.inputs._setting_locked = False
                db.outputs._commit()
            return False

        @staticmethod
        def initialize(context, node):
            OgnIsaacArticulationStateDatabase._initialize_per_node_data(node)
            initialize_function = getattr(OgnIsaacArticulationStateDatabase.NODE_TYPE_CLASS, 'initialize', None)
            if callable(initialize_function):  # pragma: no cover
                initialize_function(context, node)

            per_node_data = OgnIsaacArticulationStateDatabase.PER_NODE_DATA[node.node_id()]

            def on_connection_or_disconnection(*args):
                per_node_data['_db'] = None

            node.register_on_connected_callback(on_connection_or_disconnection)
            node.register_on_disconnected_callback(on_connection_or_disconnection)

        @staticmethod
        def release(node):
            release_function = getattr(OgnIsaacArticulationStateDatabase.NODE_TYPE_CLASS, 'release', None)
            if callable(release_function):  # pragma: no cover
                release_function(node)
            OgnIsaacArticulationStateDatabase._release_per_node_data(node)

        @staticmethod
        def init_instance(node, graph_instance_id):
            init_instance_function = getattr(OgnIsaacArticulationStateDatabase.NODE_TYPE_CLASS, 'init_instance', None)
            if callable(init_instance_function):  # pragma: no cover
                init_instance_function(node, graph_instance_id)

        @staticmethod
        def release_instance(node, graph_instance_id):
            release_instance_function = getattr(OgnIsaacArticulationStateDatabase.NODE_TYPE_CLASS, 'release_instance', None)
            if callable(release_instance_function):  # pragma: no cover
                release_instance_function(node, graph_instance_id)
            OgnIsaacArticulationStateDatabase._release_per_node_instance_data(node, graph_instance_id)

        @staticmethod
        def update_node_version(context, node, old_version, new_version):
            update_node_version_function = getattr(OgnIsaacArticulationStateDatabase.NODE_TYPE_CLASS, 'update_node_version', None)
            if callable(update_node_version_function):  # pragma: no cover
                return update_node_version_function(context, node, old_version, new_version)
            return False

        @staticmethod
        def initialize_type(node_type):
            initialize_type_function = getattr(OgnIsaacArticulationStateDatabase.NODE_TYPE_CLASS, 'initialize_type', None)
            needs_initializing = True
            if callable(initialize_type_function):  # pragma: no cover
                needs_initializing = initialize_type_function(node_type)
            if needs_initializing:
                node_type.set_metadata(ogn.MetadataKeys.EXTENSION, "omni.isaac.core_nodes")
                node_type.set_metadata(ogn.MetadataKeys.UI_NAME, "Articulation State")
                node_type.set_metadata(ogn.MetadataKeys.CATEGORIES, "isaacSim")
                node_type.set_metadata(ogn.MetadataKeys.CATEGORY_DESCRIPTIONS, "isaacSim,robot state inside Isaac Sim")
                node_type.set_metadata(ogn.MetadataKeys.DESCRIPTION, "Articulated robot state. The node takes either joint names or joint indices, and outputs the joint positions and velocities, as well as the measured joint efforts, forces, and torques.")
                node_type.set_metadata(ogn.MetadataKeys.LANGUAGE, "Python")
                OgnIsaacArticulationStateDatabase.INTERFACE.add_to_node_type(node_type)

        @staticmethod
        def on_connection_type_resolve(node):
            on_connection_type_resolve_function = getattr(OgnIsaacArticulationStateDatabase.NODE_TYPE_CLASS, 'on_connection_type_resolve', None)
            if callable(on_connection_type_resolve_function):  # pragma: no cover
                on_connection_type_resolve_function(node)

    NODE_TYPE_CLASS = None

    @staticmethod
    def register(node_type_class):
        OgnIsaacArticulationStateDatabase.NODE_TYPE_CLASS = node_type_class
        og.register_node_type(OgnIsaacArticulationStateDatabase.abi, 1)

    @staticmethod
    def deregister():
        og.deregister_node_type("omni.isaac.core_nodes.IsaacArticulationState")
