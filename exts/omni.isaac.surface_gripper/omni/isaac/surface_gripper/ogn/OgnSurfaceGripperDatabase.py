"""Support for simplified access to data on nodes of type omni.isaac.surface_gripper.SurfaceGripper

 __   ___ .  .  ___  __       ___  ___  __      __   __   __   ___
/ _` |__  |\ | |__  |__)  /\   |  |__  |  \    /  ` /  \ |  \ |__
\__| |___ | \| |___ |  \ /--\  |  |___ |__/    \__, \__/ |__/ |___

 __   __     .  .  __  ___     .  .  __   __     ___
|  \ /  \    |\ | /  \  |      |\/| /  \ |  \ | |__  \ /
|__/ \__/    | \| \__/  |      |  | \__/ |__/ | |     |

Surface Gripper
"""

import sys
import traceback
import usdrt

import omni.graph.core as og
import omni.graph.core._omni_graph_core as _og
import omni.graph.tools.ogn as ogn



class OgnSurfaceGripperDatabase(og.Database):
    """Helper class providing simplified access to data on nodes of type omni.isaac.surface_gripper.SurfaceGripper

    Class Members:
        node: Node being evaluated

    Attribute Value Properties:
        Inputs:
            inputs.BendAngle
            inputs.Close
            inputs.Damping
            inputs.Delta
            inputs.DisableGravity
            inputs.ForceLimit
            inputs.GripPosition
            inputs.GripThreshold
            inputs.Open
            inputs.ParentRigidBody
            inputs.RetryClose
            inputs.Stiffness
            inputs.TorqueLimit
            inputs.enabled
            inputs.onStep
        Outputs:
            outputs.Closed
            outputs.GripBroken
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
        ('inputs:BendAngle', 'float', 0, None, 'maximum bend angle, degrees', {ogn.MetadataKeys.DEFAULT: '7.5'}, True, 7.5, False, ''),
        ('inputs:Close', 'execution', 0, None, 'call to close gripper', {}, True, None, False, ''),
        ('inputs:Damping', 'float', 0, None, 'Gripper damping', {ogn.MetadataKeys.DEFAULT: '1000.0'}, True, 1000.0, False, ''),
        ('inputs:Delta', 'float', 0, None, 'time since last step in seconds', {}, True, 0.0, False, ''),
        ('inputs:DisableGravity', 'bool', 0, None, "flag to disable gravity of picked object to compensate for object's mass on robotic controllers", {ogn.MetadataKeys.DEFAULT: 'true'}, True, True, False, ''),
        ('inputs:ForceLimit', 'float', 0, None, 'Gripper breaking force', {ogn.MetadataKeys.DEFAULT: '1000000.0'}, True, 1000000.0, False, ''),
        ('inputs:GripPosition', 'target', 0, None, 'The point at which objects will be gripped', {}, True, None, False, ''),
        ('inputs:GripThreshold', 'float', 0, None, 'How far from an object it allows the gripper to lock in. Object will be pulled in this distance when gripper is closed', {ogn.MetadataKeys.DEFAULT: '0.01'}, True, 0.01, False, ''),
        ('inputs:Open', 'execution', 0, None, 'call to Open gripper', {}, True, None, False, ''),
        ('inputs:ParentRigidBody', 'target', 0, None, 'The rigid body that is used as a surface Gripper', {}, True, None, False, ''),
        ('inputs:RetryClose', 'bool', 0, None, 'Flag to indicate if gripper should keep attempting to close until it grips some object', {ogn.MetadataKeys.DEFAULT: 'false'}, True, False, False, ''),
        ('inputs:Stiffness', 'float', 0, None, 'Gripper stiffness', {ogn.MetadataKeys.DEFAULT: '10000.0'}, True, 10000.0, False, ''),
        ('inputs:TorqueLimit', 'float', 0, None, 'Torque breaking limit', {ogn.MetadataKeys.DEFAULT: '1000000.0'}, True, 1000000.0, False, ''),
        ('inputs:enabled', 'bool', 0, None, 'node does not execute if disabled', {ogn.MetadataKeys.DEFAULT: 'true'}, True, True, False, ''),
        ('inputs:onStep', 'execution', 0, None, 'step to animate textures', {}, True, None, False, ''),
        ('outputs:Closed', 'bool', 0, None, 'Surface gripper is closed or not', {}, True, None, False, ''),
        ('outputs:GripBroken', 'execution', 0, None, 'triggered when surface gripper unexpectedly breaks open', {}, True, None, False, ''),
    ])

    @classmethod
    def _populate_role_data(cls):
        """Populate a role structure with the non-default roles on this node type"""
        role_data = super()._populate_role_data()
        role_data.inputs.Close = og.AttributeRole.EXECUTION
        role_data.inputs.GripPosition = og.AttributeRole.TARGET
        role_data.inputs.Open = og.AttributeRole.EXECUTION
        role_data.inputs.ParentRigidBody = og.AttributeRole.TARGET
        role_data.inputs.onStep = og.AttributeRole.EXECUTION
        role_data.outputs.GripBroken = og.AttributeRole.EXECUTION
        return role_data

    class ValuesForInputs(og.DynamicAttributeAccess):
        LOCAL_PROPERTY_NAMES = {"BendAngle", "Close", "Damping", "Delta", "DisableGravity", "ForceLimit", "GripThreshold", "Open", "RetryClose", "Stiffness", "TorqueLimit", "enabled", "onStep", "_setting_locked", "_batchedReadAttributes", "_batchedReadValues"}
        """Helper class that creates natural hierarchical access to input attributes"""
        def __init__(self, node: og.Node, attributes, dynamic_attributes: og.DynamicAttributeInterface):
            """Initialize simplified access for the attribute data"""
            context = node.get_graph().get_default_graph_context()
            super().__init__(context, node, attributes, dynamic_attributes)
            self._batchedReadAttributes = [self._attributes.BendAngle, self._attributes.Close, self._attributes.Damping, self._attributes.Delta, self._attributes.DisableGravity, self._attributes.ForceLimit, self._attributes.GripThreshold, self._attributes.Open, self._attributes.RetryClose, self._attributes.Stiffness, self._attributes.TorqueLimit, self._attributes.enabled, self._attributes.onStep]
            self._batchedReadValues = [7.5, None, 1000.0, 0.0, True, 1000000.0, 0.01, None, False, 10000.0, 1000000.0, True, None]

        @property
        def GripPosition(self):
            data_view = og.AttributeValueHelper(self._attributes.GripPosition)
            return data_view.get()

        @GripPosition.setter
        def GripPosition(self, value):
            if self._setting_locked:
                raise og.ReadOnlyError(self._attributes.GripPosition)
            data_view = og.AttributeValueHelper(self._attributes.GripPosition)
            data_view.set(value)
            self.GripPosition_size = data_view.get_array_size()

        @property
        def ParentRigidBody(self):
            data_view = og.AttributeValueHelper(self._attributes.ParentRigidBody)
            return data_view.get()

        @ParentRigidBody.setter
        def ParentRigidBody(self, value):
            if self._setting_locked:
                raise og.ReadOnlyError(self._attributes.ParentRigidBody)
            data_view = og.AttributeValueHelper(self._attributes.ParentRigidBody)
            data_view.set(value)
            self.ParentRigidBody_size = data_view.get_array_size()

        @property
        def BendAngle(self):
            return self._batchedReadValues[0]

        @BendAngle.setter
        def BendAngle(self, value):
            self._batchedReadValues[0] = value

        @property
        def Close(self):
            return self._batchedReadValues[1]

        @Close.setter
        def Close(self, value):
            self._batchedReadValues[1] = value

        @property
        def Damping(self):
            return self._batchedReadValues[2]

        @Damping.setter
        def Damping(self, value):
            self._batchedReadValues[2] = value

        @property
        def Delta(self):
            return self._batchedReadValues[3]

        @Delta.setter
        def Delta(self, value):
            self._batchedReadValues[3] = value

        @property
        def DisableGravity(self):
            return self._batchedReadValues[4]

        @DisableGravity.setter
        def DisableGravity(self, value):
            self._batchedReadValues[4] = value

        @property
        def ForceLimit(self):
            return self._batchedReadValues[5]

        @ForceLimit.setter
        def ForceLimit(self, value):
            self._batchedReadValues[5] = value

        @property
        def GripThreshold(self):
            return self._batchedReadValues[6]

        @GripThreshold.setter
        def GripThreshold(self, value):
            self._batchedReadValues[6] = value

        @property
        def Open(self):
            return self._batchedReadValues[7]

        @Open.setter
        def Open(self, value):
            self._batchedReadValues[7] = value

        @property
        def RetryClose(self):
            return self._batchedReadValues[8]

        @RetryClose.setter
        def RetryClose(self, value):
            self._batchedReadValues[8] = value

        @property
        def Stiffness(self):
            return self._batchedReadValues[9]

        @Stiffness.setter
        def Stiffness(self, value):
            self._batchedReadValues[9] = value

        @property
        def TorqueLimit(self):
            return self._batchedReadValues[10]

        @TorqueLimit.setter
        def TorqueLimit(self, value):
            self._batchedReadValues[10] = value

        @property
        def enabled(self):
            return self._batchedReadValues[11]

        @enabled.setter
        def enabled(self, value):
            self._batchedReadValues[11] = value

        @property
        def onStep(self):
            return self._batchedReadValues[12]

        @onStep.setter
        def onStep(self, value):
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
        LOCAL_PROPERTY_NAMES = {"Closed", "GripBroken", "_batchedWriteValues"}
        """Helper class that creates natural hierarchical access to output attributes"""
        def __init__(self, node: og.Node, attributes, dynamic_attributes: og.DynamicAttributeInterface):
            """Initialize simplified access for the attribute data"""
            context = node.get_graph().get_default_graph_context()
            super().__init__(context, node, attributes, dynamic_attributes)
            self._batchedWriteValues = { }

        @property
        def Closed(self):
            value = self._batchedWriteValues.get(self._attributes.Closed)
            if value:
                return value
            else:
                data_view = og.AttributeValueHelper(self._attributes.Closed)
                return data_view.get()

        @Closed.setter
        def Closed(self, value):
            self._batchedWriteValues[self._attributes.Closed] = value

        @property
        def GripBroken(self):
            value = self._batchedWriteValues.get(self._attributes.GripBroken)
            if value:
                return value
            else:
                data_view = og.AttributeValueHelper(self._attributes.GripBroken)
                return data_view.get()

        @GripBroken.setter
        def GripBroken(self, value):
            self._batchedWriteValues[self._attributes.GripBroken] = value

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
        self.inputs = OgnSurfaceGripperDatabase.ValuesForInputs(node, self.attributes.inputs, dynamic_attributes)
        dynamic_attributes = self.dynamic_attribute_data(node, og.AttributePortType.ATTRIBUTE_PORT_TYPE_OUTPUT)
        self.outputs = OgnSurfaceGripperDatabase.ValuesForOutputs(node, self.attributes.outputs, dynamic_attributes)
        dynamic_attributes = self.dynamic_attribute_data(node, og.AttributePortType.ATTRIBUTE_PORT_TYPE_STATE)
        self.state = OgnSurfaceGripperDatabase.ValuesForState(node, self.attributes.state, dynamic_attributes)

    class abi:
        """Class defining the ABI interface for the node type"""

        @staticmethod
        def get_node_type():
            get_node_type_function = getattr(OgnSurfaceGripperDatabase.NODE_TYPE_CLASS, 'get_node_type', None)
            if callable(get_node_type_function):  # pragma: no cover
                return get_node_type_function()
            return 'omni.isaac.surface_gripper.SurfaceGripper'

        @staticmethod
        def compute(context, node):
            def database_valid():
                return True
            try:
                per_node_data = OgnSurfaceGripperDatabase.PER_NODE_DATA[node.node_id()]
                db = per_node_data.get('_db')
                if db is None:
                    db = OgnSurfaceGripperDatabase(node)
                    per_node_data['_db'] = db
                if not database_valid():
                    per_node_data['_db'] = None
                    return False
            except:
                db = OgnSurfaceGripperDatabase(node)

            try:
                compute_function = getattr(OgnSurfaceGripperDatabase.NODE_TYPE_CLASS, 'compute', None)
                if callable(compute_function) and compute_function.__code__.co_argcount > 1:  # pragma: no cover
                    return compute_function(context, node)

                db.inputs._prefetch()
                db.inputs._setting_locked = True
                with og.in_compute():
                    return OgnSurfaceGripperDatabase.NODE_TYPE_CLASS.compute(db)
            except Exception as error:  # pragma: no cover
                stack_trace = "".join(traceback.format_tb(sys.exc_info()[2].tb_next))
                db.log_error(f'Assertion raised in compute - {error}\n{stack_trace}', add_context=False)
            finally:
                db.inputs._setting_locked = False
                db.outputs._commit()
            return False

        @staticmethod
        def initialize(context, node):
            OgnSurfaceGripperDatabase._initialize_per_node_data(node)
            initialize_function = getattr(OgnSurfaceGripperDatabase.NODE_TYPE_CLASS, 'initialize', None)
            if callable(initialize_function):  # pragma: no cover
                initialize_function(context, node)

            per_node_data = OgnSurfaceGripperDatabase.PER_NODE_DATA[node.node_id()]

            def on_connection_or_disconnection(*args):
                per_node_data['_db'] = None

            node.register_on_connected_callback(on_connection_or_disconnection)
            node.register_on_disconnected_callback(on_connection_or_disconnection)

        @staticmethod
        def release(node):
            release_function = getattr(OgnSurfaceGripperDatabase.NODE_TYPE_CLASS, 'release', None)
            if callable(release_function):  # pragma: no cover
                release_function(node)
            OgnSurfaceGripperDatabase._release_per_node_data(node)

        @staticmethod
        def init_instance(node, graph_instance_id):
            init_instance_function = getattr(OgnSurfaceGripperDatabase.NODE_TYPE_CLASS, 'init_instance', None)
            if callable(init_instance_function):  # pragma: no cover
                init_instance_function(node, graph_instance_id)

        @staticmethod
        def release_instance(node, graph_instance_id):
            release_instance_function = getattr(OgnSurfaceGripperDatabase.NODE_TYPE_CLASS, 'release_instance', None)
            if callable(release_instance_function):  # pragma: no cover
                release_instance_function(node, graph_instance_id)
            OgnSurfaceGripperDatabase._release_per_node_instance_data(node, graph_instance_id)

        @staticmethod
        def update_node_version(context, node, old_version, new_version):
            update_node_version_function = getattr(OgnSurfaceGripperDatabase.NODE_TYPE_CLASS, 'update_node_version', None)
            if callable(update_node_version_function):  # pragma: no cover
                return update_node_version_function(context, node, old_version, new_version)
            return False

        @staticmethod
        def initialize_type(node_type):
            initialize_type_function = getattr(OgnSurfaceGripperDatabase.NODE_TYPE_CLASS, 'initialize_type', None)
            needs_initializing = True
            if callable(initialize_type_function):  # pragma: no cover
                needs_initializing = initialize_type_function(node_type)
            if needs_initializing:
                node_type.set_metadata(ogn.MetadataKeys.EXTENSION, "omni.isaac.surface_gripper")
                node_type.set_metadata(ogn.MetadataKeys.UI_NAME, "Surface Gripper")
                node_type.set_metadata(ogn.MetadataKeys.CATEGORIES, "isaacSim")
                node_type.set_metadata(ogn.MetadataKeys.CATEGORY_DESCRIPTIONS, "isaacSim,Surface Gripper inside Isaac Sim")
                node_type.set_metadata(ogn.MetadataKeys.DESCRIPTION, "Surface Gripper")
                node_type.set_metadata(ogn.MetadataKeys.LANGUAGE, "Python")
                __hints = node_type.get_scheduling_hints()
                if __hints is not None:
                    __hints.set_data_access(og.eAccessLocation.E_USD, og.eAccessType.E_WRITE)
                    __hints.compute_rule = og.eComputeRule.E_ON_REQUEST
                OgnSurfaceGripperDatabase.INTERFACE.add_to_node_type(node_type)

        @staticmethod
        def on_connection_type_resolve(node):
            on_connection_type_resolve_function = getattr(OgnSurfaceGripperDatabase.NODE_TYPE_CLASS, 'on_connection_type_resolve', None)
            if callable(on_connection_type_resolve_function):  # pragma: no cover
                on_connection_type_resolve_function(node)

    NODE_TYPE_CLASS = None

    @staticmethod
    def register(node_type_class):
        OgnSurfaceGripperDatabase.NODE_TYPE_CLASS = node_type_class
        og.register_node_type(OgnSurfaceGripperDatabase.abi, 1)

    @staticmethod
    def deregister():
        og.deregister_node_type("omni.isaac.surface_gripper.SurfaceGripper")
