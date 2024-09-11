"""Support for simplified access to data on nodes of type omni.isaac.wheeled_robots.AckermannSteering

 __   ___ .  .  ___  __       ___  ___  __      __   __   __   ___
/ _` |__  |\ | |__  |__)  /\   |  |__  |  \    /  ` /  \ |  \ |__
\__| |___ | \| |___ |  \ /--\  |  |___ |__/    \__, \__/ |__/ |___

 __   __     .  .  __  ___     .  .  __   __     ___
|  \ /  \    |\ | /  \  |      |\/| /  \ |  \ | |__  \ /
|__/ \__/    | \| \__/  |      |  | \__/ |__/ | |     |

NOTE: DEPRECATED as of Isaac Sim 4.1.0 in favour of OgnAckermannController Ackermann Steering Geometry
"""

import numpy
import sys
import traceback

import omni.graph.core as og
import omni.graph.core._omni_graph_core as _og
import omni.graph.tools.ogn as ogn



class OgnAckermannDatabase(og.Database):
    """Helper class providing simplified access to data on nodes of type omni.isaac.wheeled_robots.AckermannSteering

    Class Members:
        node: Node being evaluated

    Attribute Value Properties:
        Inputs:
            inputs.DT
            inputs.acceleration
            inputs.currentLinearVelocity
            inputs.execIn
            inputs.invertSteeringAngle
            inputs.maxWheelRotation
            inputs.maxWheelVelocity
            inputs.speed
            inputs.steeringAngle
            inputs.trackWidth
            inputs.turningWheelRadius
            inputs.useAcceleration
            inputs.wheelBase
        Outputs:
            outputs.execOut
            outputs.leftWheelAngle
            outputs.rightWheelAngle
            outputs.wheelRotationVelocity
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
        ('inputs:DT', 'double', 0, None, 'Delta time for the simulation step', {}, True, 0.0, False, ''),
        ('inputs:acceleration', 'double', 0, None, 'Desired forward acceleration for the robot in m/s^2', {}, True, 0.0, False, ''),
        ('inputs:currentLinearVelocity', 'vector3d', 0, None, 'Current linear velocity of the robot in m/s', {}, True, [0.0, 0.0, 0.0], False, ''),
        ('inputs:execIn', 'execution', 0, None, 'The input execution', {}, True, None, False, ''),
        ('inputs:invertSteeringAngle', 'bool', 0, None, 'Flips the sign of the steering angle, Set to true for rear wheel steering', {}, True, False, False, ''),
        ('inputs:maxWheelRotation', 'double', 0, None, 'Maximum angle of rotation for the front wheels in radians', {}, True, 0.0, False, ''),
        ('inputs:maxWheelVelocity', 'double', 0, None, 'Maximum angular velocity of the robot wheel in rad/s', {}, True, 0.0, False, ''),
        ('inputs:speed', 'double', 0, None, 'Desired forward speed in m/s', {}, True, 0.0, False, ''),
        ('inputs:steeringAngle', 'double', 0, None, 'Desired virtual angle in radians. Corresponds to the yaw of a virtual wheel located at the center of the front axle. By default it is positive for turning left and negative for turning right for front wheel drive.', {ogn.MetadataKeys.DEFAULT: '0.0'}, True, 0.0, False, ''),
        ('inputs:trackWidth', 'double', 0, None, 'Distance between the left and right rear wheels of the robot in meters', {}, True, 0.0, False, ''),
        ('inputs:turningWheelRadius', 'double', 0, None, 'Radius of the front wheels of the robot in meters', {}, True, 0.0, False, ''),
        ('inputs:useAcceleration', 'bool', 0, None, 'Use acceleration as an input, Set to false to use speed as input instead', {ogn.MetadataKeys.DEFAULT: 'true'}, True, True, False, ''),
        ('inputs:wheelBase', 'double', 0, None, 'Distance between the front and rear axles of the robot in meters', {}, True, 0.0, False, ''),
        ('outputs:execOut', 'execution', 0, None, 'The output execution', {}, True, None, False, ''),
        ('outputs:leftWheelAngle', 'double', 0, None, 'Angle for the left turning wheel in radians', {}, True, None, False, ''),
        ('outputs:rightWheelAngle', 'double', 0, None, 'Angle for the right turning wheel in radians', {}, True, None, False, ''),
        ('outputs:wheelRotationVelocity', 'double', 0, None, 'Angular velocity for the turning wheels in rad/s', {}, True, None, False, ''),
    ])

    @classmethod
    def _populate_role_data(cls):
        """Populate a role structure with the non-default roles on this node type"""
        role_data = super()._populate_role_data()
        role_data.inputs.currentLinearVelocity = og.AttributeRole.VECTOR
        role_data.inputs.execIn = og.AttributeRole.EXECUTION
        role_data.outputs.execOut = og.AttributeRole.EXECUTION
        return role_data

    class ValuesForInputs(og.DynamicAttributeAccess):
        LOCAL_PROPERTY_NAMES = {"DT", "acceleration", "currentLinearVelocity", "execIn", "invertSteeringAngle", "maxWheelRotation", "maxWheelVelocity", "speed", "steeringAngle", "trackWidth", "turningWheelRadius", "useAcceleration", "wheelBase", "_setting_locked", "_batchedReadAttributes", "_batchedReadValues"}
        """Helper class that creates natural hierarchical access to input attributes"""
        def __init__(self, node: og.Node, attributes, dynamic_attributes: og.DynamicAttributeInterface):
            """Initialize simplified access for the attribute data"""
            context = node.get_graph().get_default_graph_context()
            super().__init__(context, node, attributes, dynamic_attributes)
            self._batchedReadAttributes = [self._attributes.DT, self._attributes.acceleration, self._attributes.currentLinearVelocity, self._attributes.execIn, self._attributes.invertSteeringAngle, self._attributes.maxWheelRotation, self._attributes.maxWheelVelocity, self._attributes.speed, self._attributes.steeringAngle, self._attributes.trackWidth, self._attributes.turningWheelRadius, self._attributes.useAcceleration, self._attributes.wheelBase]
            self._batchedReadValues = [0.0, 0.0, [0.0, 0.0, 0.0], None, False, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, True, 0.0]

        @property
        def DT(self):
            return self._batchedReadValues[0]

        @DT.setter
        def DT(self, value):
            self._batchedReadValues[0] = value

        @property
        def acceleration(self):
            return self._batchedReadValues[1]

        @acceleration.setter
        def acceleration(self, value):
            self._batchedReadValues[1] = value

        @property
        def currentLinearVelocity(self):
            return self._batchedReadValues[2]

        @currentLinearVelocity.setter
        def currentLinearVelocity(self, value):
            self._batchedReadValues[2] = value

        @property
        def execIn(self):
            return self._batchedReadValues[3]

        @execIn.setter
        def execIn(self, value):
            self._batchedReadValues[3] = value

        @property
        def invertSteeringAngle(self):
            return self._batchedReadValues[4]

        @invertSteeringAngle.setter
        def invertSteeringAngle(self, value):
            self._batchedReadValues[4] = value

        @property
        def maxWheelRotation(self):
            return self._batchedReadValues[5]

        @maxWheelRotation.setter
        def maxWheelRotation(self, value):
            self._batchedReadValues[5] = value

        @property
        def maxWheelVelocity(self):
            return self._batchedReadValues[6]

        @maxWheelVelocity.setter
        def maxWheelVelocity(self, value):
            self._batchedReadValues[6] = value

        @property
        def speed(self):
            return self._batchedReadValues[7]

        @speed.setter
        def speed(self, value):
            self._batchedReadValues[7] = value

        @property
        def steeringAngle(self):
            return self._batchedReadValues[8]

        @steeringAngle.setter
        def steeringAngle(self, value):
            self._batchedReadValues[8] = value

        @property
        def trackWidth(self):
            return self._batchedReadValues[9]

        @trackWidth.setter
        def trackWidth(self, value):
            self._batchedReadValues[9] = value

        @property
        def turningWheelRadius(self):
            return self._batchedReadValues[10]

        @turningWheelRadius.setter
        def turningWheelRadius(self, value):
            self._batchedReadValues[10] = value

        @property
        def useAcceleration(self):
            return self._batchedReadValues[11]

        @useAcceleration.setter
        def useAcceleration(self, value):
            self._batchedReadValues[11] = value

        @property
        def wheelBase(self):
            return self._batchedReadValues[12]

        @wheelBase.setter
        def wheelBase(self, value):
            self._batchedReadValues[12] = value

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
        LOCAL_PROPERTY_NAMES = {"execOut", "leftWheelAngle", "rightWheelAngle", "wheelRotationVelocity", "_batchedWriteValues"}
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
        def leftWheelAngle(self):
            value = self._batchedWriteValues.get(self._attributes.leftWheelAngle)
            if value:
                return value
            else:
                data_view = og.AttributeValueHelper(self._attributes.leftWheelAngle)
                return data_view.get()

        @leftWheelAngle.setter
        def leftWheelAngle(self, value):
            self._batchedWriteValues[self._attributes.leftWheelAngle] = value

        @property
        def rightWheelAngle(self):
            value = self._batchedWriteValues.get(self._attributes.rightWheelAngle)
            if value:
                return value
            else:
                data_view = og.AttributeValueHelper(self._attributes.rightWheelAngle)
                return data_view.get()

        @rightWheelAngle.setter
        def rightWheelAngle(self, value):
            self._batchedWriteValues[self._attributes.rightWheelAngle] = value

        @property
        def wheelRotationVelocity(self):
            value = self._batchedWriteValues.get(self._attributes.wheelRotationVelocity)
            if value:
                return value
            else:
                data_view = og.AttributeValueHelper(self._attributes.wheelRotationVelocity)
                return data_view.get()

        @wheelRotationVelocity.setter
        def wheelRotationVelocity(self, value):
            self._batchedWriteValues[self._attributes.wheelRotationVelocity] = value

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
        self.inputs = OgnAckermannDatabase.ValuesForInputs(node, self.attributes.inputs, dynamic_attributes)
        dynamic_attributes = self.dynamic_attribute_data(node, og.AttributePortType.ATTRIBUTE_PORT_TYPE_OUTPUT)
        self.outputs = OgnAckermannDatabase.ValuesForOutputs(node, self.attributes.outputs, dynamic_attributes)
        dynamic_attributes = self.dynamic_attribute_data(node, og.AttributePortType.ATTRIBUTE_PORT_TYPE_STATE)
        self.state = OgnAckermannDatabase.ValuesForState(node, self.attributes.state, dynamic_attributes)

    class abi:
        """Class defining the ABI interface for the node type"""

        @staticmethod
        def get_node_type():
            get_node_type_function = getattr(OgnAckermannDatabase.NODE_TYPE_CLASS, 'get_node_type', None)
            if callable(get_node_type_function):  # pragma: no cover
                return get_node_type_function()
            return 'omni.isaac.wheeled_robots.AckermannSteering'

        @staticmethod
        def compute(context, node):
            def database_valid():
                return True
            try:
                per_node_data = OgnAckermannDatabase.PER_NODE_DATA[node.node_id()]
                db = per_node_data.get('_db')
                if db is None:
                    db = OgnAckermannDatabase(node)
                    per_node_data['_db'] = db
                if not database_valid():
                    per_node_data['_db'] = None
                    return False
            except:
                db = OgnAckermannDatabase(node)

            try:
                compute_function = getattr(OgnAckermannDatabase.NODE_TYPE_CLASS, 'compute', None)
                if callable(compute_function) and compute_function.__code__.co_argcount > 1:  # pragma: no cover
                    return compute_function(context, node)

                db.inputs._prefetch()
                db.inputs._setting_locked = True
                with og.in_compute():
                    return OgnAckermannDatabase.NODE_TYPE_CLASS.compute(db)
            except Exception as error:  # pragma: no cover
                stack_trace = "".join(traceback.format_tb(sys.exc_info()[2].tb_next))
                db.log_error(f'Assertion raised in compute - {error}\n{stack_trace}', add_context=False)
            finally:
                db.inputs._setting_locked = False
                db.outputs._commit()
            return False

        @staticmethod
        def initialize(context, node):
            OgnAckermannDatabase._initialize_per_node_data(node)
            initialize_function = getattr(OgnAckermannDatabase.NODE_TYPE_CLASS, 'initialize', None)
            if callable(initialize_function):  # pragma: no cover
                initialize_function(context, node)

            per_node_data = OgnAckermannDatabase.PER_NODE_DATA[node.node_id()]

            def on_connection_or_disconnection(*args):
                per_node_data['_db'] = None

            node.register_on_connected_callback(on_connection_or_disconnection)
            node.register_on_disconnected_callback(on_connection_or_disconnection)

        @staticmethod
        def release(node):
            release_function = getattr(OgnAckermannDatabase.NODE_TYPE_CLASS, 'release', None)
            if callable(release_function):  # pragma: no cover
                release_function(node)
            OgnAckermannDatabase._release_per_node_data(node)

        @staticmethod
        def init_instance(node, graph_instance_id):
            init_instance_function = getattr(OgnAckermannDatabase.NODE_TYPE_CLASS, 'init_instance', None)
            if callable(init_instance_function):  # pragma: no cover
                init_instance_function(node, graph_instance_id)

        @staticmethod
        def release_instance(node, graph_instance_id):
            release_instance_function = getattr(OgnAckermannDatabase.NODE_TYPE_CLASS, 'release_instance', None)
            if callable(release_instance_function):  # pragma: no cover
                release_instance_function(node, graph_instance_id)
            OgnAckermannDatabase._release_per_node_instance_data(node, graph_instance_id)

        @staticmethod
        def update_node_version(context, node, old_version, new_version):
            update_node_version_function = getattr(OgnAckermannDatabase.NODE_TYPE_CLASS, 'update_node_version', None)
            if callable(update_node_version_function):  # pragma: no cover
                return update_node_version_function(context, node, old_version, new_version)
            return False

        @staticmethod
        def initialize_type(node_type):
            initialize_type_function = getattr(OgnAckermannDatabase.NODE_TYPE_CLASS, 'initialize_type', None)
            needs_initializing = True
            if callable(initialize_type_function):  # pragma: no cover
                needs_initializing = initialize_type_function(node_type)
            if needs_initializing:
                node_type.set_metadata(ogn.MetadataKeys.EXTENSION, "omni.isaac.wheeled_robots")
                node_type.set_metadata(ogn.MetadataKeys.UI_NAME, "Ackermann Steering")
                node_type.set_metadata(ogn.MetadataKeys.HIDDEN, "true")
                node_type.set_metadata(ogn.MetadataKeys.CATEGORIES, "isaacSim")
                node_type.set_metadata(ogn.MetadataKeys.CATEGORY_DESCRIPTIONS, "isaacSim,Ackermann steering for robots in Isaac Sim")
                node_type.set_metadata(ogn.MetadataKeys.DESCRIPTION, "NOTE: DEPRECATED as of Isaac Sim 4.1.0 in favour of OgnAckermannController Ackermann Steering Geometry")
                node_type.set_metadata(ogn.MetadataKeys.LANGUAGE, "Python")
                OgnAckermannDatabase.INTERFACE.add_to_node_type(node_type)

        @staticmethod
        def on_connection_type_resolve(node):
            on_connection_type_resolve_function = getattr(OgnAckermannDatabase.NODE_TYPE_CLASS, 'on_connection_type_resolve', None)
            if callable(on_connection_type_resolve_function):  # pragma: no cover
                on_connection_type_resolve_function(node)

    NODE_TYPE_CLASS = None

    @staticmethod
    def register(node_type_class):
        OgnAckermannDatabase.NODE_TYPE_CLASS = node_type_class
        og.register_node_type(OgnAckermannDatabase.abi, 1)

    @staticmethod
    def deregister():
        og.deregister_node_type("omni.isaac.wheeled_robots.AckermannSteering")
