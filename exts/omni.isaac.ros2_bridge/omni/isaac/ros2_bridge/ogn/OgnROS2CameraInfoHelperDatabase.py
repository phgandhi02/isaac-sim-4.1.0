"""Support for simplified access to data on nodes of type omni.isaac.ros2_bridge.ROS2CameraInfoHelper

 __   ___ .  .  ___  __       ___  ___  __      __   __   __   ___
/ _` |__  |\ | |__  |__)  /\   |  |__  |  \    /  ` /  \ |  \ |__
\__| |___ | \| |___ |  \ /--\  |  |___ |__/    \__, \__/ |__/ |___

 __   __     .  .  __  ___     .  .  __   __     ___
|  \ /  \    |\ | /  \  |      |\/| /  \ |  \ | |__  \ /
|__/ \__/    | \| \__/  |      |  | \__/ |__/ | |     |

This node automates the CameraInfo message pipeline for monocular and stereo cameras.
"""

import carb
import sys
import traceback

import omni.graph.core as og
import omni.graph.core._omni_graph_core as _og
import omni.graph.tools.ogn as ogn



class OgnROS2CameraInfoHelperDatabase(og.Database):
    """Helper class providing simplified access to data on nodes of type omni.isaac.ros2_bridge.ROS2CameraInfoHelper

    Class Members:
        node: Node being evaluated

    Attribute Value Properties:
        Inputs:
            inputs.context
            inputs.enabled
            inputs.execIn
            inputs.frameId
            inputs.frameIdRight
            inputs.frameSkipCount
            inputs.nodeNamespace
            inputs.qosProfile
            inputs.queueSize
            inputs.renderProductPath
            inputs.renderProductPathRight
            inputs.resetSimulationTimeOnStop
            inputs.topicName
            inputs.topicNameRight
            inputs.useSystemTime
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
        ('inputs:context', 'uint64', 0, None, 'ROS2 context handle, Default of zero will use the default global context', {ogn.MetadataKeys.DEFAULT: '0'}, True, 0, False, ''),
        ('inputs:enabled', 'bool', 0, None, 'True to enable the camera helper, False to disable', {ogn.MetadataKeys.DEFAULT: 'true'}, True, True, False, ''),
        ('inputs:execIn', 'execution', 0, None, 'Triggering this causes the sensor pipeline to be generated', {}, True, None, False, ''),
        ('inputs:frameId', 'string', 0, None, 'FrameId for ROS2 message from the monocular or left stereo camera.', {ogn.MetadataKeys.DEFAULT: '"sim_camera"'}, True, "sim_camera", False, ''),
        ('inputs:frameIdRight', 'string', 0, None, 'FrameId for ROS2 message from the right stereo camera.', {ogn.MetadataKeys.DEFAULT: '"sim_camera_right"'}, False, "sim_camera_right", False, ''),
        ('inputs:frameSkipCount', 'uint', 0, None, 'Specifies the number of simulation frames to skip between each message publish. (e.g. Set to 0 to publish each frame. Set 1 to publish every other frame)', {ogn.MetadataKeys.DEFAULT: '0'}, True, 0, False, ''),
        ('inputs:nodeNamespace', 'string', 0, None, 'Namespace of ROS2 Node, prepends any published/subscribed topic by the node namespace', {ogn.MetadataKeys.DEFAULT: '""'}, True, "", False, ''),
        ('inputs:qosProfile', 'string', 0, None, 'QoS profile config', {ogn.MetadataKeys.DEFAULT: '""'}, True, "", False, ''),
        ('inputs:queueSize', 'uint64', 0, None, "The number of messages to queue up before throwing some away, in case messages are collected faster than they can be sent. Only honored if 'history' QoS policy was set to 'keep last'. This setting can be overwritten by qosProfile input.", {ogn.MetadataKeys.DEFAULT: '10'}, True, 10, False, ''),
        ('inputs:renderProductPath', 'token', 0, None, 'Path of the render product used for capturing data from the monocular or left stereo camera', {}, True, "", False, ''),
        ('inputs:renderProductPathRight', 'token', 0, None, 'Path of the render product used for capturing data from the right stereo camera', {}, False, None, False, ''),
        ('inputs:resetSimulationTimeOnStop', 'bool', 0, 'Reset Simulation Time On Stop', 'If True the simulation time will reset when stop is pressed, False means time increases monotonically. This setting is ignored if useSystemTime is enabled.', {ogn.MetadataKeys.DEFAULT: 'false'}, True, False, False, ''),
        ('inputs:topicName', 'string', 0, None, 'Topic name for the monocular or left stereo camera data..', {ogn.MetadataKeys.DEFAULT: '"camera_info"'}, True, "camera_info", False, ''),
        ('inputs:topicNameRight', 'string', 0, None, 'Topic name for the right stereo camera data.', {ogn.MetadataKeys.DEFAULT: '"camera_info_right"'}, False, "camera_info_right", False, ''),
        ('inputs:useSystemTime', 'bool', 0, None, 'If True, system timestamp will be included in messages. If False, simulation timestamp will be included in messages', {ogn.MetadataKeys.DEFAULT: 'false'}, True, False, False, ''),
    ])

    @classmethod
    def _populate_role_data(cls):
        """Populate a role structure with the non-default roles on this node type"""
        role_data = super()._populate_role_data()
        role_data.inputs.execIn = og.AttributeRole.EXECUTION
        role_data.inputs.frameId = og.AttributeRole.TEXT
        role_data.inputs.frameIdRight = og.AttributeRole.TEXT
        role_data.inputs.nodeNamespace = og.AttributeRole.TEXT
        role_data.inputs.qosProfile = og.AttributeRole.TEXT
        role_data.inputs.topicName = og.AttributeRole.TEXT
        role_data.inputs.topicNameRight = og.AttributeRole.TEXT
        return role_data

    class ValuesForInputs(og.DynamicAttributeAccess):
        LOCAL_PROPERTY_NAMES = {"context", "enabled", "execIn", "frameId", "frameIdRight", "frameSkipCount", "nodeNamespace", "qosProfile", "queueSize", "renderProductPath", "renderProductPathRight", "resetSimulationTimeOnStop", "topicName", "topicNameRight", "useSystemTime", "_setting_locked", "_batchedReadAttributes", "_batchedReadValues"}
        """Helper class that creates natural hierarchical access to input attributes"""
        def __init__(self, node: og.Node, attributes, dynamic_attributes: og.DynamicAttributeInterface):
            """Initialize simplified access for the attribute data"""
            context = node.get_graph().get_default_graph_context()
            super().__init__(context, node, attributes, dynamic_attributes)
            self._batchedReadAttributes = [self._attributes.context, self._attributes.enabled, self._attributes.execIn, self._attributes.frameId, self._attributes.frameIdRight, self._attributes.frameSkipCount, self._attributes.nodeNamespace, self._attributes.qosProfile, self._attributes.queueSize, self._attributes.renderProductPath, self._attributes.renderProductPathRight, self._attributes.resetSimulationTimeOnStop, self._attributes.topicName, self._attributes.topicNameRight, self._attributes.useSystemTime]
            self._batchedReadValues = [0, True, None, "sim_camera", "sim_camera_right", 0, "", "", 10, "", None, False, "camera_info", "camera_info_right", False]

        @property
        def context(self):
            return self._batchedReadValues[0]

        @context.setter
        def context(self, value):
            self._batchedReadValues[0] = value

        @property
        def enabled(self):
            return self._batchedReadValues[1]

        @enabled.setter
        def enabled(self, value):
            self._batchedReadValues[1] = value

        @property
        def execIn(self):
            return self._batchedReadValues[2]

        @execIn.setter
        def execIn(self, value):
            self._batchedReadValues[2] = value

        @property
        def frameId(self):
            return self._batchedReadValues[3]

        @frameId.setter
        def frameId(self, value):
            self._batchedReadValues[3] = value

        @property
        def frameIdRight(self):
            return self._batchedReadValues[4]

        @frameIdRight.setter
        def frameIdRight(self, value):
            self._batchedReadValues[4] = value

        @property
        def frameSkipCount(self):
            return self._batchedReadValues[5]

        @frameSkipCount.setter
        def frameSkipCount(self, value):
            self._batchedReadValues[5] = value

        @property
        def nodeNamespace(self):
            return self._batchedReadValues[6]

        @nodeNamespace.setter
        def nodeNamespace(self, value):
            self._batchedReadValues[6] = value

        @property
        def qosProfile(self):
            return self._batchedReadValues[7]

        @qosProfile.setter
        def qosProfile(self, value):
            self._batchedReadValues[7] = value

        @property
        def queueSize(self):
            return self._batchedReadValues[8]

        @queueSize.setter
        def queueSize(self, value):
            self._batchedReadValues[8] = value

        @property
        def renderProductPath(self):
            return self._batchedReadValues[9]

        @renderProductPath.setter
        def renderProductPath(self, value):
            self._batchedReadValues[9] = value

        @property
        def renderProductPathRight(self):
            return self._batchedReadValues[10]

        @renderProductPathRight.setter
        def renderProductPathRight(self, value):
            self._batchedReadValues[10] = value

        @property
        def resetSimulationTimeOnStop(self):
            return self._batchedReadValues[11]

        @resetSimulationTimeOnStop.setter
        def resetSimulationTimeOnStop(self, value):
            self._batchedReadValues[11] = value

        @property
        def topicName(self):
            return self._batchedReadValues[12]

        @topicName.setter
        def topicName(self, value):
            self._batchedReadValues[12] = value

        @property
        def topicNameRight(self):
            return self._batchedReadValues[13]

        @topicNameRight.setter
        def topicNameRight(self, value):
            self._batchedReadValues[13] = value

        @property
        def useSystemTime(self):
            return self._batchedReadValues[14]

        @useSystemTime.setter
        def useSystemTime(self, value):
            self._batchedReadValues[14] = value

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
        self.inputs = OgnROS2CameraInfoHelperDatabase.ValuesForInputs(node, self.attributes.inputs, dynamic_attributes)
        dynamic_attributes = self.dynamic_attribute_data(node, og.AttributePortType.ATTRIBUTE_PORT_TYPE_OUTPUT)
        self.outputs = OgnROS2CameraInfoHelperDatabase.ValuesForOutputs(node, self.attributes.outputs, dynamic_attributes)
        dynamic_attributes = self.dynamic_attribute_data(node, og.AttributePortType.ATTRIBUTE_PORT_TYPE_STATE)
        self.state = OgnROS2CameraInfoHelperDatabase.ValuesForState(node, self.attributes.state, dynamic_attributes)

    class abi:
        """Class defining the ABI interface for the node type"""

        @staticmethod
        def get_node_type():
            get_node_type_function = getattr(OgnROS2CameraInfoHelperDatabase.NODE_TYPE_CLASS, 'get_node_type', None)
            if callable(get_node_type_function):  # pragma: no cover
                return get_node_type_function()
            return 'omni.isaac.ros2_bridge.ROS2CameraInfoHelper'

        @staticmethod
        def compute(context, node):
            def database_valid():
                return True
            try:
                per_node_data = OgnROS2CameraInfoHelperDatabase.PER_NODE_DATA[node.node_id()]
                db = per_node_data.get('_db')
                if db is None:
                    db = OgnROS2CameraInfoHelperDatabase(node)
                    per_node_data['_db'] = db
                if not database_valid():
                    per_node_data['_db'] = None
                    return False
            except:
                db = OgnROS2CameraInfoHelperDatabase(node)

            try:
                compute_function = getattr(OgnROS2CameraInfoHelperDatabase.NODE_TYPE_CLASS, 'compute', None)
                if callable(compute_function) and compute_function.__code__.co_argcount > 1:  # pragma: no cover
                    return compute_function(context, node)

                db.inputs._prefetch()
                db.inputs._setting_locked = True
                with og.in_compute():
                    return OgnROS2CameraInfoHelperDatabase.NODE_TYPE_CLASS.compute(db)
            except Exception as error:  # pragma: no cover
                stack_trace = "".join(traceback.format_tb(sys.exc_info()[2].tb_next))
                db.log_error(f'Assertion raised in compute - {error}\n{stack_trace}', add_context=False)
            finally:
                db.inputs._setting_locked = False
                db.outputs._commit()
            return False

        @staticmethod
        def initialize(context, node):
            OgnROS2CameraInfoHelperDatabase._initialize_per_node_data(node)
            initialize_function = getattr(OgnROS2CameraInfoHelperDatabase.NODE_TYPE_CLASS, 'initialize', None)
            if callable(initialize_function):  # pragma: no cover
                initialize_function(context, node)

            per_node_data = OgnROS2CameraInfoHelperDatabase.PER_NODE_DATA[node.node_id()]

            def on_connection_or_disconnection(*args):
                per_node_data['_db'] = None

            node.register_on_connected_callback(on_connection_or_disconnection)
            node.register_on_disconnected_callback(on_connection_or_disconnection)

        @staticmethod
        def release(node):
            release_function = getattr(OgnROS2CameraInfoHelperDatabase.NODE_TYPE_CLASS, 'release', None)
            if callable(release_function):  # pragma: no cover
                release_function(node)
            OgnROS2CameraInfoHelperDatabase._release_per_node_data(node)

        @staticmethod
        def init_instance(node, graph_instance_id):
            init_instance_function = getattr(OgnROS2CameraInfoHelperDatabase.NODE_TYPE_CLASS, 'init_instance', None)
            if callable(init_instance_function):  # pragma: no cover
                init_instance_function(node, graph_instance_id)

        @staticmethod
        def release_instance(node, graph_instance_id):
            release_instance_function = getattr(OgnROS2CameraInfoHelperDatabase.NODE_TYPE_CLASS, 'release_instance', None)
            if callable(release_instance_function):  # pragma: no cover
                release_instance_function(node, graph_instance_id)
            OgnROS2CameraInfoHelperDatabase._release_per_node_instance_data(node, graph_instance_id)

        @staticmethod
        def update_node_version(context, node, old_version, new_version):
            update_node_version_function = getattr(OgnROS2CameraInfoHelperDatabase.NODE_TYPE_CLASS, 'update_node_version', None)
            if callable(update_node_version_function):  # pragma: no cover
                return update_node_version_function(context, node, old_version, new_version)
            return False

        @staticmethod
        def initialize_type(node_type):
            initialize_type_function = getattr(OgnROS2CameraInfoHelperDatabase.NODE_TYPE_CLASS, 'initialize_type', None)
            needs_initializing = True
            if callable(initialize_type_function):  # pragma: no cover
                needs_initializing = initialize_type_function(node_type)
            if needs_initializing:
                node_type.set_metadata(ogn.MetadataKeys.EXTENSION, "omni.isaac.ros2_bridge")
                node_type.set_metadata(ogn.MetadataKeys.UI_NAME, "ROS2 Camera Info Helper")
                node_type.set_metadata(ogn.MetadataKeys.CATEGORIES, "isaacRos2")
                node_type.set_metadata(ogn.MetadataKeys.DESCRIPTION, "This node automates the CameraInfo message pipeline for monocular and stereo cameras.")
                node_type.set_metadata(ogn.MetadataKeys.LANGUAGE, "Python")
                icon_path = carb.tokens.get_tokens_interface().resolve("${omni.isaac.ros2_bridge}")
                icon_path = icon_path + '/' + "ogn/icons/omni.isaac.ros2_bridge.ROS2CameraInfoHelper.svg"
                node_type.set_metadata(ogn.MetadataKeys.ICON_PATH, icon_path)
                OgnROS2CameraInfoHelperDatabase.INTERFACE.add_to_node_type(node_type)

        @staticmethod
        def on_connection_type_resolve(node):
            on_connection_type_resolve_function = getattr(OgnROS2CameraInfoHelperDatabase.NODE_TYPE_CLASS, 'on_connection_type_resolve', None)
            if callable(on_connection_type_resolve_function):  # pragma: no cover
                on_connection_type_resolve_function(node)

    NODE_TYPE_CLASS = None

    @staticmethod
    def register(node_type_class):
        OgnROS2CameraInfoHelperDatabase.NODE_TYPE_CLASS = node_type_class
        og.register_node_type(OgnROS2CameraInfoHelperDatabase.abi, 1)

    @staticmethod
    def deregister():
        og.deregister_node_type("omni.isaac.ros2_bridge.ROS2CameraInfoHelper")
