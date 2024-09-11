"""Support for simplified access to data on nodes of type omni.isaac.manipulators.IsaacGripperController

 __   ___ .  .  ___  __       ___  ___  __      __   __   __   ___
/ _` |__  |\ | |__  |__)  /\   |  |__  |  \    /  ` /  \ |  \ |__
\__| |___ | \| |___ |  \ /--\  |  |___ |__/    \__, \__/ |__/ |___

 __   __     .  .  __  ___     .  .  __   __     ___
|  \ /  \    |\ | /  \  |      |\/| /  \ |  \ | |__  \ /
|__/ \__/    | \| \__/  |      |  | \__/ |__/ | |     |

Isaac Sim Gripper Controller Node
"""

import numpy
import sys
import traceback
import usdrt

import omni.graph.core as og
import omni.graph.core._omni_graph_core as _og
import omni.graph.tools.ogn as ogn



class OgnIsaacGripperControllerDatabase(og.Database):
    """Helper class providing simplified access to data on nodes of type omni.isaac.manipulators.IsaacGripperController

    Class Members:
        node: Node being evaluated

    Attribute Value Properties:
        Inputs:
            inputs.articulationRootPrim
            inputs.close
            inputs.closePosition
            inputs.execIn
            inputs.gripperPrim
            inputs.gripperSpeed
            inputs.jointNames
            inputs.open
            inputs.openPosition
            inputs.stop
        Outputs:
            outputs.jointNames
            outputs.positionCommands
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
        ('inputs:articulationRootPrim', 'target', 0, None, 'Articulation root prim of the robot', {}, False, None, False, ''),
        ('inputs:close', 'execution', 0, None, 'close gripper call', {}, True, None, False, ''),
        ('inputs:closePosition', 'double[]', 0, None, 'closing position for the gripper joints, will use the joint limit if not provided', {}, False, None, False, ''),
        ('inputs:execIn', 'execution', 0, None, 'tick', {}, True, None, False, ''),
        ('inputs:gripperPrim', 'target', 0, None, "The gripper's root link prim", {}, False, None, False, ''),
        ('inputs:gripperSpeed', 'double[]', 0, None, 'gripper speed (distance per frame)', {}, True, [], False, ''),
        ('inputs:jointNames', 'token[]', 0, None, 'gripper joint names', {}, True, [], False, ''),
        ('inputs:open', 'execution', 0, None, 'open gripper call', {}, True, None, False, ''),
        ('inputs:openPosition', 'double[]', 0, None, 'maximum opening position for the gripper joints, will use the joint limit if not provided', {}, False, None, False, ''),
        ('inputs:stop', 'execution', 0, None, 'stop gripper call', {}, True, None, False, ''),
        ('outputs:jointNames', 'token[]', 0, None, 'gripper joint names', {}, True, None, False, ''),
        ('outputs:positionCommands', 'double[]', 0, None, 'joint commands to the articulation controller', {}, True, None, False, ''),
    ])

    @classmethod
    def _populate_role_data(cls):
        """Populate a role structure with the non-default roles on this node type"""
        role_data = super()._populate_role_data()
        role_data.inputs.articulationRootPrim = og.AttributeRole.TARGET
        role_data.inputs.close = og.AttributeRole.EXECUTION
        role_data.inputs.execIn = og.AttributeRole.EXECUTION
        role_data.inputs.gripperPrim = og.AttributeRole.TARGET
        role_data.inputs.open = og.AttributeRole.EXECUTION
        role_data.inputs.stop = og.AttributeRole.EXECUTION
        return role_data

    class ValuesForInputs(og.DynamicAttributeAccess):
        LOCAL_PROPERTY_NAMES = {"close", "execIn", "open", "stop", "_setting_locked", "_batchedReadAttributes", "_batchedReadValues"}
        """Helper class that creates natural hierarchical access to input attributes"""
        def __init__(self, node: og.Node, attributes, dynamic_attributes: og.DynamicAttributeInterface):
            """Initialize simplified access for the attribute data"""
            context = node.get_graph().get_default_graph_context()
            super().__init__(context, node, attributes, dynamic_attributes)
            self._batchedReadAttributes = [self._attributes.close, self._attributes.execIn, self._attributes.open, self._attributes.stop]
            self._batchedReadValues = [None, None, None, None]

        @property
        def articulationRootPrim(self):
            data_view = og.AttributeValueHelper(self._attributes.articulationRootPrim)
            return data_view.get()

        @articulationRootPrim.setter
        def articulationRootPrim(self, value):
            if self._setting_locked:
                raise og.ReadOnlyError(self._attributes.articulationRootPrim)
            data_view = og.AttributeValueHelper(self._attributes.articulationRootPrim)
            data_view.set(value)
            self.articulationRootPrim_size = data_view.get_array_size()

        @property
        def closePosition(self):
            data_view = og.AttributeValueHelper(self._attributes.closePosition)
            return data_view.get()

        @closePosition.setter
        def closePosition(self, value):
            if self._setting_locked:
                raise og.ReadOnlyError(self._attributes.closePosition)
            data_view = og.AttributeValueHelper(self._attributes.closePosition)
            data_view.set(value)
            self.closePosition_size = data_view.get_array_size()

        @property
        def gripperPrim(self):
            data_view = og.AttributeValueHelper(self._attributes.gripperPrim)
            return data_view.get()

        @gripperPrim.setter
        def gripperPrim(self, value):
            if self._setting_locked:
                raise og.ReadOnlyError(self._attributes.gripperPrim)
            data_view = og.AttributeValueHelper(self._attributes.gripperPrim)
            data_view.set(value)
            self.gripperPrim_size = data_view.get_array_size()

        @property
        def gripperSpeed(self):
            data_view = og.AttributeValueHelper(self._attributes.gripperSpeed)
            return data_view.get()

        @gripperSpeed.setter
        def gripperSpeed(self, value):
            if self._setting_locked:
                raise og.ReadOnlyError(self._attributes.gripperSpeed)
            data_view = og.AttributeValueHelper(self._attributes.gripperSpeed)
            data_view.set(value)
            self.gripperSpeed_size = data_view.get_array_size()

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
        def openPosition(self):
            data_view = og.AttributeValueHelper(self._attributes.openPosition)
            return data_view.get()

        @openPosition.setter
        def openPosition(self, value):
            if self._setting_locked:
                raise og.ReadOnlyError(self._attributes.openPosition)
            data_view = og.AttributeValueHelper(self._attributes.openPosition)
            data_view.set(value)
            self.openPosition_size = data_view.get_array_size()

        @property
        def close(self):
            return self._batchedReadValues[0]

        @close.setter
        def close(self, value):
            self._batchedReadValues[0] = value

        @property
        def execIn(self):
            return self._batchedReadValues[1]

        @execIn.setter
        def execIn(self, value):
            self._batchedReadValues[1] = value

        @property
        def open(self):
            return self._batchedReadValues[2]

        @open.setter
        def open(self, value):
            self._batchedReadValues[2] = value

        @property
        def stop(self):
            return self._batchedReadValues[3]

        @stop.setter
        def stop(self, value):
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
        LOCAL_PROPERTY_NAMES = { }
        """Helper class that creates natural hierarchical access to output attributes"""
        def __init__(self, node: og.Node, attributes, dynamic_attributes: og.DynamicAttributeInterface):
            """Initialize simplified access for the attribute data"""
            context = node.get_graph().get_default_graph_context()
            super().__init__(context, node, attributes, dynamic_attributes)
            self.jointNames_size = None
            self.positionCommands_size = None
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
        def positionCommands(self):
            data_view = og.AttributeValueHelper(self._attributes.positionCommands)
            return data_view.get(reserved_element_count=self.positionCommands_size)

        @positionCommands.setter
        def positionCommands(self, value):
            data_view = og.AttributeValueHelper(self._attributes.positionCommands)
            data_view.set(value)
            self.positionCommands_size = data_view.get_array_size()

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
        self.inputs = OgnIsaacGripperControllerDatabase.ValuesForInputs(node, self.attributes.inputs, dynamic_attributes)
        dynamic_attributes = self.dynamic_attribute_data(node, og.AttributePortType.ATTRIBUTE_PORT_TYPE_OUTPUT)
        self.outputs = OgnIsaacGripperControllerDatabase.ValuesForOutputs(node, self.attributes.outputs, dynamic_attributes)
        dynamic_attributes = self.dynamic_attribute_data(node, og.AttributePortType.ATTRIBUTE_PORT_TYPE_STATE)
        self.state = OgnIsaacGripperControllerDatabase.ValuesForState(node, self.attributes.state, dynamic_attributes)

    class abi:
        """Class defining the ABI interface for the node type"""

        @staticmethod
        def get_node_type():
            get_node_type_function = getattr(OgnIsaacGripperControllerDatabase.NODE_TYPE_CLASS, 'get_node_type', None)
            if callable(get_node_type_function):  # pragma: no cover
                return get_node_type_function()
            return 'omni.isaac.manipulators.IsaacGripperController'

        @staticmethod
        def compute(context, node):
            def database_valid():
                return True
            try:
                per_node_data = OgnIsaacGripperControllerDatabase.PER_NODE_DATA[node.node_id()]
                db = per_node_data.get('_db')
                if db is None:
                    db = OgnIsaacGripperControllerDatabase(node)
                    per_node_data['_db'] = db
                if not database_valid():
                    per_node_data['_db'] = None
                    return False
            except:
                db = OgnIsaacGripperControllerDatabase(node)

            try:
                compute_function = getattr(OgnIsaacGripperControllerDatabase.NODE_TYPE_CLASS, 'compute', None)
                if callable(compute_function) and compute_function.__code__.co_argcount > 1:  # pragma: no cover
                    return compute_function(context, node)

                db.inputs._prefetch()
                db.inputs._setting_locked = True
                with og.in_compute():
                    return OgnIsaacGripperControllerDatabase.NODE_TYPE_CLASS.compute(db)
            except Exception as error:  # pragma: no cover
                stack_trace = "".join(traceback.format_tb(sys.exc_info()[2].tb_next))
                db.log_error(f'Assertion raised in compute - {error}\n{stack_trace}', add_context=False)
            finally:
                db.inputs._setting_locked = False
                db.outputs._commit()
            return False

        @staticmethod
        def initialize(context, node):
            OgnIsaacGripperControllerDatabase._initialize_per_node_data(node)
            initialize_function = getattr(OgnIsaacGripperControllerDatabase.NODE_TYPE_CLASS, 'initialize', None)
            if callable(initialize_function):  # pragma: no cover
                initialize_function(context, node)

            per_node_data = OgnIsaacGripperControllerDatabase.PER_NODE_DATA[node.node_id()]

            def on_connection_or_disconnection(*args):
                per_node_data['_db'] = None

            node.register_on_connected_callback(on_connection_or_disconnection)
            node.register_on_disconnected_callback(on_connection_or_disconnection)

        @staticmethod
        def release(node):
            release_function = getattr(OgnIsaacGripperControllerDatabase.NODE_TYPE_CLASS, 'release', None)
            if callable(release_function):  # pragma: no cover
                release_function(node)
            OgnIsaacGripperControllerDatabase._release_per_node_data(node)

        @staticmethod
        def init_instance(node, graph_instance_id):
            init_instance_function = getattr(OgnIsaacGripperControllerDatabase.NODE_TYPE_CLASS, 'init_instance', None)
            if callable(init_instance_function):  # pragma: no cover
                init_instance_function(node, graph_instance_id)

        @staticmethod
        def release_instance(node, graph_instance_id):
            release_instance_function = getattr(OgnIsaacGripperControllerDatabase.NODE_TYPE_CLASS, 'release_instance', None)
            if callable(release_instance_function):  # pragma: no cover
                release_instance_function(node, graph_instance_id)
            OgnIsaacGripperControllerDatabase._release_per_node_instance_data(node, graph_instance_id)

        @staticmethod
        def update_node_version(context, node, old_version, new_version):
            update_node_version_function = getattr(OgnIsaacGripperControllerDatabase.NODE_TYPE_CLASS, 'update_node_version', None)
            if callable(update_node_version_function):  # pragma: no cover
                return update_node_version_function(context, node, old_version, new_version)
            return False

        @staticmethod
        def initialize_type(node_type):
            initialize_type_function = getattr(OgnIsaacGripperControllerDatabase.NODE_TYPE_CLASS, 'initialize_type', None)
            needs_initializing = True
            if callable(initialize_type_function):  # pragma: no cover
                needs_initializing = initialize_type_function(node_type)
            if needs_initializing:
                node_type.set_metadata(ogn.MetadataKeys.EXTENSION, "omni.isaac.manipulators")
                node_type.set_metadata(ogn.MetadataKeys.UI_NAME, "Isaac Gripper Controller Node")
                node_type.set_metadata(ogn.MetadataKeys.CATEGORIES, "isaacSim")
                node_type.set_metadata(ogn.MetadataKeys.CATEGORY_DESCRIPTIONS, "isaacSim,Controller for Grippers")
                node_type.set_metadata(ogn.MetadataKeys.DESCRIPTION, "Isaac Sim Gripper Controller Node")
                node_type.set_metadata(ogn.MetadataKeys.LANGUAGE, "Python")
                OgnIsaacGripperControllerDatabase.INTERFACE.add_to_node_type(node_type)

        @staticmethod
        def on_connection_type_resolve(node):
            on_connection_type_resolve_function = getattr(OgnIsaacGripperControllerDatabase.NODE_TYPE_CLASS, 'on_connection_type_resolve', None)
            if callable(on_connection_type_resolve_function):  # pragma: no cover
                on_connection_type_resolve_function(node)

    NODE_TYPE_CLASS = None

    @staticmethod
    def register(node_type_class):
        OgnIsaacGripperControllerDatabase.NODE_TYPE_CLASS = node_type_class
        og.register_node_type(OgnIsaacGripperControllerDatabase.abi, 1)

    @staticmethod
    def deregister():
        og.deregister_node_type("omni.isaac.manipulators.IsaacGripperController")
