"""Support for simplified access to data on nodes of type omni.isaac.core_nodes.IsaacArticulationController

 __   ___ .  .  ___  __       ___  ___  __      __   __   __   ___
/ _` |__  |\ | |__  |__)  /\   |  |__  |  \    /  ` /  \ |  \ |__
\__| |___ | \| |___ |  \ /--\  |  |___ |__/    \__, \__/ |__/ |___

 __   __     .  .  __  ___     .  .  __   __     ___
|  \ /  \    |\ | /  \  |      |\/| /  \ |  \ | |__  \ /
|__/ \__/    | \| \__/  |      |  | \__/ |__/ | |     |

Controller for articulated robots. The controller takes either joint names or joint indices, and move them by the given position/velocity/effort
commands. Note angular units are expressed in radians while angles in USD are expressed in degrees and will be adjusted accordingly
by the articulation controller.
"""

import numpy
import sys
import traceback
import usdrt

import omni.graph.core as og
import omni.graph.core._omni_graph_core as _og
import omni.graph.tools.ogn as ogn



class OgnIsaacArticulationControllerDatabase(og.Database):
    """Helper class providing simplified access to data on nodes of type omni.isaac.core_nodes.IsaacArticulationController

    Class Members:
        node: Node being evaluated

    Attribute Value Properties:
        Inputs:
            inputs.effortCommand
            inputs.execIn
            inputs.jointIndices
            inputs.jointNames
            inputs.positionCommand
            inputs.robotPath
            inputs.targetPrim
            inputs.velocityCommand
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
        ('inputs:effortCommand', 'double[]', 0, None, 'effort commands\nForce Units:  \n    linear (kg*m/s^2) i.e. a force\n    angular (kg*m^2/s^2) i.e. a torque\nAcceleration Units:\n    linear (m/s^2), i.e. a linear acceleration\n    angular (rad/s^2) i.e. an angular acceleration', {}, True, [], False, ''),
        ('inputs:execIn', 'execution', 0, None, 'The input execution', {}, True, None, False, ''),
        ('inputs:jointIndices', 'int[]', 0, None, 'commanded joint indices, use either Joint Names or Joint Indices, if neither is given, default to all joints', {}, True, [], False, ''),
        ('inputs:jointNames', 'token[]', 0, None, 'commanded joint names, use either Joint Names or Joint Indices, if neither is given, default to all joints', {}, True, [], False, ''),
        ('inputs:positionCommand', 'double[]', 0, None, 'position commands\nUnits:\n    linear (m)\n    angular (rad)', {}, True, [], False, ''),
        ('inputs:robotPath', 'string', 0, None, 'path to the robot articulation root. If this is populated, targetPrim is ignored.', {}, True, "", False, ''),
        ('inputs:targetPrim', 'target', 0, None, 'The target robot prim with robot articulation root. Ensure robotPath is empty for this to be considered.', {}, False, None, False, ''),
        ('inputs:velocityCommand', 'double[]', 0, None, 'velocity commands\nUnits:\n    linear (m/s)\n    angular (rad/s)', {}, True, [], False, ''),
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
        def effortCommand(self):
            data_view = og.AttributeValueHelper(self._attributes.effortCommand)
            return data_view.get()

        @effortCommand.setter
        def effortCommand(self, value):
            if self._setting_locked:
                raise og.ReadOnlyError(self._attributes.effortCommand)
            data_view = og.AttributeValueHelper(self._attributes.effortCommand)
            data_view.set(value)
            self.effortCommand_size = data_view.get_array_size()

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
        def positionCommand(self):
            data_view = og.AttributeValueHelper(self._attributes.positionCommand)
            return data_view.get()

        @positionCommand.setter
        def positionCommand(self, value):
            if self._setting_locked:
                raise og.ReadOnlyError(self._attributes.positionCommand)
            data_view = og.AttributeValueHelper(self._attributes.positionCommand)
            data_view.set(value)
            self.positionCommand_size = data_view.get_array_size()

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
        def velocityCommand(self):
            data_view = og.AttributeValueHelper(self._attributes.velocityCommand)
            return data_view.get()

        @velocityCommand.setter
        def velocityCommand(self, value):
            if self._setting_locked:
                raise og.ReadOnlyError(self._attributes.velocityCommand)
            data_view = og.AttributeValueHelper(self._attributes.velocityCommand)
            data_view.set(value)
            self.velocityCommand_size = data_view.get_array_size()

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
            self._batchedWriteValues = { }

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
        self.inputs = OgnIsaacArticulationControllerDatabase.ValuesForInputs(node, self.attributes.inputs, dynamic_attributes)
        dynamic_attributes = self.dynamic_attribute_data(node, og.AttributePortType.ATTRIBUTE_PORT_TYPE_OUTPUT)
        self.outputs = OgnIsaacArticulationControllerDatabase.ValuesForOutputs(node, self.attributes.outputs, dynamic_attributes)
        dynamic_attributes = self.dynamic_attribute_data(node, og.AttributePortType.ATTRIBUTE_PORT_TYPE_STATE)
        self.state = OgnIsaacArticulationControllerDatabase.ValuesForState(node, self.attributes.state, dynamic_attributes)

    class abi:
        """Class defining the ABI interface for the node type"""

        @staticmethod
        def get_node_type():
            get_node_type_function = getattr(OgnIsaacArticulationControllerDatabase.NODE_TYPE_CLASS, 'get_node_type', None)
            if callable(get_node_type_function):  # pragma: no cover
                return get_node_type_function()
            return 'omni.isaac.core_nodes.IsaacArticulationController'

        @staticmethod
        def compute(context, node):
            def database_valid():
                return True
            try:
                per_node_data = OgnIsaacArticulationControllerDatabase.PER_NODE_DATA[node.node_id()]
                db = per_node_data.get('_db')
                if db is None:
                    db = OgnIsaacArticulationControllerDatabase(node)
                    per_node_data['_db'] = db
                if not database_valid():
                    per_node_data['_db'] = None
                    return False
            except:
                db = OgnIsaacArticulationControllerDatabase(node)

            try:
                compute_function = getattr(OgnIsaacArticulationControllerDatabase.NODE_TYPE_CLASS, 'compute', None)
                if callable(compute_function) and compute_function.__code__.co_argcount > 1:  # pragma: no cover
                    return compute_function(context, node)

                db.inputs._prefetch()
                db.inputs._setting_locked = True
                with og.in_compute():
                    return OgnIsaacArticulationControllerDatabase.NODE_TYPE_CLASS.compute(db)
            except Exception as error:  # pragma: no cover
                stack_trace = "".join(traceback.format_tb(sys.exc_info()[2].tb_next))
                db.log_error(f'Assertion raised in compute - {error}\n{stack_trace}', add_context=False)
            finally:
                db.inputs._setting_locked = False
                db.outputs._commit()
            return False

        @staticmethod
        def initialize(context, node):
            OgnIsaacArticulationControllerDatabase._initialize_per_node_data(node)
            initialize_function = getattr(OgnIsaacArticulationControllerDatabase.NODE_TYPE_CLASS, 'initialize', None)
            if callable(initialize_function):  # pragma: no cover
                initialize_function(context, node)

            per_node_data = OgnIsaacArticulationControllerDatabase.PER_NODE_DATA[node.node_id()]

            def on_connection_or_disconnection(*args):
                per_node_data['_db'] = None

            node.register_on_connected_callback(on_connection_or_disconnection)
            node.register_on_disconnected_callback(on_connection_or_disconnection)

        @staticmethod
        def release(node):
            release_function = getattr(OgnIsaacArticulationControllerDatabase.NODE_TYPE_CLASS, 'release', None)
            if callable(release_function):  # pragma: no cover
                release_function(node)
            OgnIsaacArticulationControllerDatabase._release_per_node_data(node)

        @staticmethod
        def init_instance(node, graph_instance_id):
            init_instance_function = getattr(OgnIsaacArticulationControllerDatabase.NODE_TYPE_CLASS, 'init_instance', None)
            if callable(init_instance_function):  # pragma: no cover
                init_instance_function(node, graph_instance_id)

        @staticmethod
        def release_instance(node, graph_instance_id):
            release_instance_function = getattr(OgnIsaacArticulationControllerDatabase.NODE_TYPE_CLASS, 'release_instance', None)
            if callable(release_instance_function):  # pragma: no cover
                release_instance_function(node, graph_instance_id)
            OgnIsaacArticulationControllerDatabase._release_per_node_instance_data(node, graph_instance_id)

        @staticmethod
        def update_node_version(context, node, old_version, new_version):
            update_node_version_function = getattr(OgnIsaacArticulationControllerDatabase.NODE_TYPE_CLASS, 'update_node_version', None)
            if callable(update_node_version_function):  # pragma: no cover
                return update_node_version_function(context, node, old_version, new_version)
            return False

        @staticmethod
        def initialize_type(node_type):
            initialize_type_function = getattr(OgnIsaacArticulationControllerDatabase.NODE_TYPE_CLASS, 'initialize_type', None)
            needs_initializing = True
            if callable(initialize_type_function):  # pragma: no cover
                needs_initializing = initialize_type_function(node_type)
            if needs_initializing:
                node_type.set_metadata(ogn.MetadataKeys.EXTENSION, "omni.isaac.core_nodes")
                node_type.set_metadata(ogn.MetadataKeys.UI_NAME, "Articulation Controller")
                node_type.set_metadata(ogn.MetadataKeys.CATEGORIES, "isaacSim")
                node_type.set_metadata(ogn.MetadataKeys.CATEGORY_DESCRIPTIONS, "isaacSim,robot controller inside Isaac Sim")
                node_type.set_metadata(ogn.MetadataKeys.DESCRIPTION, "Controller for articulated robots. The controller takes either joint names or joint indices, and move them by the given position/velocity/effort commands. Note angular units are expressed in radians while angles in USD are expressed in degrees and will be adjusted accordingly by the articulation controller.")
                node_type.set_metadata(ogn.MetadataKeys.LANGUAGE, "Python")
                OgnIsaacArticulationControllerDatabase.INTERFACE.add_to_node_type(node_type)

        @staticmethod
        def on_connection_type_resolve(node):
            on_connection_type_resolve_function = getattr(OgnIsaacArticulationControllerDatabase.NODE_TYPE_CLASS, 'on_connection_type_resolve', None)
            if callable(on_connection_type_resolve_function):  # pragma: no cover
                on_connection_type_resolve_function(node)

    NODE_TYPE_CLASS = None

    @staticmethod
    def register(node_type_class):
        OgnIsaacArticulationControllerDatabase.NODE_TYPE_CLASS = node_type_class
        og.register_node_type(OgnIsaacArticulationControllerDatabase.abi, 1)

    @staticmethod
    def deregister():
        og.deregister_node_type("omni.isaac.core_nodes.IsaacArticulationController")
