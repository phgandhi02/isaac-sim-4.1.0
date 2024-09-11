"""Support for simplified access to data on nodes of type omni.isaac.wheeled_robots.HolonomicController

 __   ___ .  .  ___  __       ___  ___  __      __   __   __   ___
/ _` |__  |\ | |__  |__)  /\   |  |__  |  \    /  ` /  \ |  \ |__
\__| |___ | \| |___ |  \ /--\  |  |___ |__/    \__, \__/ |__/ |___

 __   __     .  .  __  ___     .  .  __   __     ___
|  \ /  \    |\ | /  \  |      |\/| /  \ |  \ | |__  \ /
|__/ \__/    | \| \__/  |      |  | \__/ |__/ | |     |

Holonomic Controller
"""

import numpy
import sys
import traceback

import omni.graph.core as og
import omni.graph.core._omni_graph_core as _og
import omni.graph.tools.ogn as ogn



class OgnHolonomicControllerDatabase(og.Database):
    """Helper class providing simplified access to data on nodes of type omni.isaac.wheeled_robots.HolonomicController

    Class Members:
        node: Node being evaluated

    Attribute Value Properties:
        Inputs:
            inputs.angularGain
            inputs.execIn
            inputs.inputVelocity
            inputs.linearGain
            inputs.maxAngularSpeed
            inputs.maxLinearSpeed
            inputs.maxWheelSpeed
            inputs.mecanumAngles
            inputs.upAxis
            inputs.wheelAxis
            inputs.wheelOrientations
            inputs.wheelPositions
            inputs.wheelRadius
        Outputs:
            outputs.jointVelocityCommand
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
        ('inputs:angularGain', 'double', 0, None, 'angular gain', {ogn.MetadataKeys.DEFAULT: '1'}, True, 1, False, ''),
        ('inputs:execIn', 'execution', 0, None, 'The input execution', {}, True, None, False, ''),
        ('inputs:inputVelocity', 'double3', 0, 'Velocity Commands for the vehicle', 'velocity in x and y and rotation', {}, True, [0.0, 0.0, 0.0], False, ''),
        ('inputs:linearGain', 'double', 0, None, 'linear gain', {ogn.MetadataKeys.DEFAULT: '1'}, True, 1, False, ''),
        ('inputs:maxAngularSpeed', 'double', 0, None, 'maximum angular rotation speed allowed for the vehicle', {ogn.MetadataKeys.DEFAULT: '100000'}, False, 100000, False, ''),
        ('inputs:maxLinearSpeed', 'double', 0, None, 'maximum speed allowed for the vehicle', {ogn.MetadataKeys.DEFAULT: '100000'}, False, 100000, False, ''),
        ('inputs:maxWheelSpeed', 'double', 0, None, 'maximum rotation speed allowed for the wheel joints', {ogn.MetadataKeys.DEFAULT: '100000'}, False, 100000, False, ''),
        ('inputs:mecanumAngles', 'double[]', 0, None, "angles of the mecanum wheels with respect to wheel's rotation axis", {}, True, [], False, ''),
        ('inputs:upAxis', 'double3', 0, None, 'the rotation axis of the vehicle', {ogn.MetadataKeys.DEFAULT: '[0.0, 0.0, 1.0]'}, True, [0.0, 0.0, 1.0], False, ''),
        ('inputs:wheelAxis', 'double3', 0, None, 'the rotation axis of the wheels', {ogn.MetadataKeys.DEFAULT: '[1.0, 0.0, 0.0]'}, True, [1.0, 0.0, 0.0], False, ''),
        ('inputs:wheelOrientations', 'double4[]', 0, None, "orientation of the wheel with respect to chassis' center of mass frame ", {}, True, [], False, ''),
        ('inputs:wheelPositions', 'double3[]', 0, None, "position of the wheel with respect to chassis' center of mass", {}, True, [], False, ''),
        ('inputs:wheelRadius', 'double[]', 0, None, 'an array of wheel radius', {}, True, [], False, ''),
        ('outputs:jointVelocityCommand', 'double[]', 0, None, 'velocity commands for the wheels joints', {}, True, None, False, ''),
    ])

    @classmethod
    def _populate_role_data(cls):
        """Populate a role structure with the non-default roles on this node type"""
        role_data = super()._populate_role_data()
        role_data.inputs.execIn = og.AttributeRole.EXECUTION
        return role_data

    class ValuesForInputs(og.DynamicAttributeAccess):
        LOCAL_PROPERTY_NAMES = {"angularGain", "execIn", "inputVelocity", "linearGain", "maxAngularSpeed", "maxLinearSpeed", "maxWheelSpeed", "upAxis", "wheelAxis", "_setting_locked", "_batchedReadAttributes", "_batchedReadValues"}
        """Helper class that creates natural hierarchical access to input attributes"""
        def __init__(self, node: og.Node, attributes, dynamic_attributes: og.DynamicAttributeInterface):
            """Initialize simplified access for the attribute data"""
            context = node.get_graph().get_default_graph_context()
            super().__init__(context, node, attributes, dynamic_attributes)
            self._batchedReadAttributes = [self._attributes.angularGain, self._attributes.execIn, self._attributes.inputVelocity, self._attributes.linearGain, self._attributes.maxAngularSpeed, self._attributes.maxLinearSpeed, self._attributes.maxWheelSpeed, self._attributes.upAxis, self._attributes.wheelAxis]
            self._batchedReadValues = [1, None, [0.0, 0.0, 0.0], 1, 100000, 100000, 100000, [0.0, 0.0, 1.0], [1.0, 0.0, 0.0]]

        @property
        def mecanumAngles(self):
            data_view = og.AttributeValueHelper(self._attributes.mecanumAngles)
            return data_view.get()

        @mecanumAngles.setter
        def mecanumAngles(self, value):
            if self._setting_locked:
                raise og.ReadOnlyError(self._attributes.mecanumAngles)
            data_view = og.AttributeValueHelper(self._attributes.mecanumAngles)
            data_view.set(value)
            self.mecanumAngles_size = data_view.get_array_size()

        @property
        def wheelOrientations(self):
            data_view = og.AttributeValueHelper(self._attributes.wheelOrientations)
            return data_view.get()

        @wheelOrientations.setter
        def wheelOrientations(self, value):
            if self._setting_locked:
                raise og.ReadOnlyError(self._attributes.wheelOrientations)
            data_view = og.AttributeValueHelper(self._attributes.wheelOrientations)
            data_view.set(value)
            self.wheelOrientations_size = data_view.get_array_size()

        @property
        def wheelPositions(self):
            data_view = og.AttributeValueHelper(self._attributes.wheelPositions)
            return data_view.get()

        @wheelPositions.setter
        def wheelPositions(self, value):
            if self._setting_locked:
                raise og.ReadOnlyError(self._attributes.wheelPositions)
            data_view = og.AttributeValueHelper(self._attributes.wheelPositions)
            data_view.set(value)
            self.wheelPositions_size = data_view.get_array_size()

        @property
        def wheelRadius(self):
            data_view = og.AttributeValueHelper(self._attributes.wheelRadius)
            return data_view.get()

        @wheelRadius.setter
        def wheelRadius(self, value):
            if self._setting_locked:
                raise og.ReadOnlyError(self._attributes.wheelRadius)
            data_view = og.AttributeValueHelper(self._attributes.wheelRadius)
            data_view.set(value)
            self.wheelRadius_size = data_view.get_array_size()

        @property
        def angularGain(self):
            return self._batchedReadValues[0]

        @angularGain.setter
        def angularGain(self, value):
            self._batchedReadValues[0] = value

        @property
        def execIn(self):
            return self._batchedReadValues[1]

        @execIn.setter
        def execIn(self, value):
            self._batchedReadValues[1] = value

        @property
        def inputVelocity(self):
            return self._batchedReadValues[2]

        @inputVelocity.setter
        def inputVelocity(self, value):
            self._batchedReadValues[2] = value

        @property
        def linearGain(self):
            return self._batchedReadValues[3]

        @linearGain.setter
        def linearGain(self, value):
            self._batchedReadValues[3] = value

        @property
        def maxAngularSpeed(self):
            return self._batchedReadValues[4]

        @maxAngularSpeed.setter
        def maxAngularSpeed(self, value):
            self._batchedReadValues[4] = value

        @property
        def maxLinearSpeed(self):
            return self._batchedReadValues[5]

        @maxLinearSpeed.setter
        def maxLinearSpeed(self, value):
            self._batchedReadValues[5] = value

        @property
        def maxWheelSpeed(self):
            return self._batchedReadValues[6]

        @maxWheelSpeed.setter
        def maxWheelSpeed(self, value):
            self._batchedReadValues[6] = value

        @property
        def upAxis(self):
            return self._batchedReadValues[7]

        @upAxis.setter
        def upAxis(self, value):
            self._batchedReadValues[7] = value

        @property
        def wheelAxis(self):
            return self._batchedReadValues[8]

        @wheelAxis.setter
        def wheelAxis(self, value):
            self._batchedReadValues[8] = value

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
            self.jointVelocityCommand_size = None
            self._batchedWriteValues = { }

        @property
        def jointVelocityCommand(self):
            data_view = og.AttributeValueHelper(self._attributes.jointVelocityCommand)
            return data_view.get(reserved_element_count=self.jointVelocityCommand_size)

        @jointVelocityCommand.setter
        def jointVelocityCommand(self, value):
            data_view = og.AttributeValueHelper(self._attributes.jointVelocityCommand)
            data_view.set(value)
            self.jointVelocityCommand_size = data_view.get_array_size()

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
        self.inputs = OgnHolonomicControllerDatabase.ValuesForInputs(node, self.attributes.inputs, dynamic_attributes)
        dynamic_attributes = self.dynamic_attribute_data(node, og.AttributePortType.ATTRIBUTE_PORT_TYPE_OUTPUT)
        self.outputs = OgnHolonomicControllerDatabase.ValuesForOutputs(node, self.attributes.outputs, dynamic_attributes)
        dynamic_attributes = self.dynamic_attribute_data(node, og.AttributePortType.ATTRIBUTE_PORT_TYPE_STATE)
        self.state = OgnHolonomicControllerDatabase.ValuesForState(node, self.attributes.state, dynamic_attributes)

    class abi:
        """Class defining the ABI interface for the node type"""

        @staticmethod
        def get_node_type():
            get_node_type_function = getattr(OgnHolonomicControllerDatabase.NODE_TYPE_CLASS, 'get_node_type', None)
            if callable(get_node_type_function):  # pragma: no cover
                return get_node_type_function()
            return 'omni.isaac.wheeled_robots.HolonomicController'

        @staticmethod
        def compute(context, node):
            def database_valid():
                return True
            try:
                per_node_data = OgnHolonomicControllerDatabase.PER_NODE_DATA[node.node_id()]
                db = per_node_data.get('_db')
                if db is None:
                    db = OgnHolonomicControllerDatabase(node)
                    per_node_data['_db'] = db
                if not database_valid():
                    per_node_data['_db'] = None
                    return False
            except:
                db = OgnHolonomicControllerDatabase(node)

            try:
                compute_function = getattr(OgnHolonomicControllerDatabase.NODE_TYPE_CLASS, 'compute', None)
                if callable(compute_function) and compute_function.__code__.co_argcount > 1:  # pragma: no cover
                    return compute_function(context, node)

                db.inputs._prefetch()
                db.inputs._setting_locked = True
                with og.in_compute():
                    return OgnHolonomicControllerDatabase.NODE_TYPE_CLASS.compute(db)
            except Exception as error:  # pragma: no cover
                stack_trace = "".join(traceback.format_tb(sys.exc_info()[2].tb_next))
                db.log_error(f'Assertion raised in compute - {error}\n{stack_trace}', add_context=False)
            finally:
                db.inputs._setting_locked = False
                db.outputs._commit()
            return False

        @staticmethod
        def initialize(context, node):
            OgnHolonomicControllerDatabase._initialize_per_node_data(node)
            initialize_function = getattr(OgnHolonomicControllerDatabase.NODE_TYPE_CLASS, 'initialize', None)
            if callable(initialize_function):  # pragma: no cover
                initialize_function(context, node)

            per_node_data = OgnHolonomicControllerDatabase.PER_NODE_DATA[node.node_id()]

            def on_connection_or_disconnection(*args):
                per_node_data['_db'] = None

            node.register_on_connected_callback(on_connection_or_disconnection)
            node.register_on_disconnected_callback(on_connection_or_disconnection)

        @staticmethod
        def release(node):
            release_function = getattr(OgnHolonomicControllerDatabase.NODE_TYPE_CLASS, 'release', None)
            if callable(release_function):  # pragma: no cover
                release_function(node)
            OgnHolonomicControllerDatabase._release_per_node_data(node)

        @staticmethod
        def init_instance(node, graph_instance_id):
            init_instance_function = getattr(OgnHolonomicControllerDatabase.NODE_TYPE_CLASS, 'init_instance', None)
            if callable(init_instance_function):  # pragma: no cover
                init_instance_function(node, graph_instance_id)

        @staticmethod
        def release_instance(node, graph_instance_id):
            release_instance_function = getattr(OgnHolonomicControllerDatabase.NODE_TYPE_CLASS, 'release_instance', None)
            if callable(release_instance_function):  # pragma: no cover
                release_instance_function(node, graph_instance_id)
            OgnHolonomicControllerDatabase._release_per_node_instance_data(node, graph_instance_id)

        @staticmethod
        def update_node_version(context, node, old_version, new_version):
            update_node_version_function = getattr(OgnHolonomicControllerDatabase.NODE_TYPE_CLASS, 'update_node_version', None)
            if callable(update_node_version_function):  # pragma: no cover
                return update_node_version_function(context, node, old_version, new_version)
            return False

        @staticmethod
        def initialize_type(node_type):
            initialize_type_function = getattr(OgnHolonomicControllerDatabase.NODE_TYPE_CLASS, 'initialize_type', None)
            needs_initializing = True
            if callable(initialize_type_function):  # pragma: no cover
                needs_initializing = initialize_type_function(node_type)
            if needs_initializing:
                node_type.set_metadata(ogn.MetadataKeys.EXTENSION, "omni.isaac.wheeled_robots")
                node_type.set_metadata(ogn.MetadataKeys.UI_NAME, "Holonomic Controller")
                node_type.set_metadata(ogn.MetadataKeys.CATEGORIES, "isaacSim")
                node_type.set_metadata(ogn.MetadataKeys.CATEGORY_DESCRIPTIONS, "isaacSim,robot controller inside Isaac Sim")
                node_type.set_metadata(ogn.MetadataKeys.DESCRIPTION, "Holonomic Controller")
                node_type.set_metadata(ogn.MetadataKeys.LANGUAGE, "Python")
                OgnHolonomicControllerDatabase.INTERFACE.add_to_node_type(node_type)

        @staticmethod
        def on_connection_type_resolve(node):
            on_connection_type_resolve_function = getattr(OgnHolonomicControllerDatabase.NODE_TYPE_CLASS, 'on_connection_type_resolve', None)
            if callable(on_connection_type_resolve_function):  # pragma: no cover
                on_connection_type_resolve_function(node)

    NODE_TYPE_CLASS = None

    @staticmethod
    def register(node_type_class):
        OgnHolonomicControllerDatabase.NODE_TYPE_CLASS = node_type_class
        og.register_node_type(OgnHolonomicControllerDatabase.abi, 2)

    @staticmethod
    def deregister():
        og.deregister_node_type("omni.isaac.wheeled_robots.HolonomicController")
