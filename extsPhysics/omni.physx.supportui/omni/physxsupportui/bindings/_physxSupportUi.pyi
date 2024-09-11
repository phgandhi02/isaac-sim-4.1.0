"""pybind11 carb.physx.supportui bindings"""
from __future__ import annotations
import omni.physxsupportui.bindings._physxSupportUi
import typing
import carb._carb
import carb.events._events
import omni.ui._ui
import omni.ui_scene._scene

__all__ = [
    "IPhysxSupportUi",
    "IPhysxSupportUiPrivate",
    "PhysXInspectorModel",
    "PhysXInspectorModelControlType",
    "PhysXInspectorModelDataShapeType",
    "PhysXInspectorModelInspectorType",
    "PhysXInspectorModelPropertyType",
    "PhysXInspectorModelState",
    "PhysXInspectorWidget",
    "PhysxInspectorOverlay",
    "SETTINGS_ACTION_BAR_ENABLED",
    "SETTINGS_ACTION_BAR_ENABLED_DEFAULT",
    "SETTINGS_ASYNC_COOKING_AT_STAGE_LOAD",
    "SETTINGS_ASYNC_COOKING_AT_STAGE_LOAD_DEFAULT",
    "SETTINGS_AUTOMATIC_COLLIDER_CREATION_ENABLED",
    "SETTINGS_AUTOMATIC_COLLIDER_CREATION_ENABLED_DEFAULT",
    "SETTINGS_AVOID_CHANGING_EXISTING_COLLIDERS",
    "SETTINGS_AVOID_CHANGING_EXISTING_COLLIDERS_DEFAULT",
    "SETTINGS_CUSTOM_MANIPULATOR_ENABLED",
    "SETTINGS_CUSTOM_MANIPULATOR_ENABLED_DEFAULT",
    "SETTINGS_DYNAMIC_COLLIDER_SIMPLIFICATION_TYPE",
    "SETTINGS_DYNAMIC_COLLIDER_SIMPLIFICATION_TYPE_DEFAULT",
    "SETTINGS_FLOATING_NOTIFICATIONS_ENABLED",
    "SETTINGS_FLOATING_NOTIFICATIONS_ENABLED_DEFAULT",
    "SETTINGS_LOGGING_ENABLED",
    "SETTINGS_LOGGING_ENABLED_DEFAULT",
    "SETTINGS_MANIP_MODE_ALLOW_RIGID_BODY_TRAVERSAL",
    "SETTINGS_MANIP_MODE_ALLOW_RIGID_BODY_TRAVERSAL_DEFAULT",
    "SETTINGS_MANIP_MODE_ALLOW_ROT_ON_OTHER_AXES_WHILE_ROTATING",
    "SETTINGS_MANIP_MODE_ALLOW_ROT_ON_OTHER_AXES_WHILE_ROTATING_DEFAULT",
    "SETTINGS_MANIP_MODE_ALLOW_ROT_WHILE_TRANSLATING",
    "SETTINGS_MANIP_MODE_ALLOW_ROT_WHILE_TRANSLATING_DEFAULT",
    "SETTINGS_MANIP_MODE_ALLOW_TRAN_ON_OTHER_AXES_WHILE_TRANSLATING",
    "SETTINGS_MANIP_MODE_ALLOW_TRAN_ON_OTHER_AXES_WHILE_TRANSLATING_DEFAULT",
    "SETTINGS_MANIP_MODE_ALLOW_TRAN_WHILE_ROTATING",
    "SETTINGS_MANIP_MODE_ALLOW_TRAN_WHILE_ROTATING_DEFAULT",
    "SETTINGS_PHYSICS_INSPECTOR_ENABLED",
    "SETTINGS_RIGID_BODY_SELECTION_MODE_ENABLED",
    "SETTINGS_RIGID_BODY_SELECTION_MODE_ENABLED_DEFAULT",
    "SETTINGS_STATIC_COLLIDER_SIMPLIFICATION_TYPE",
    "SETTINGS_STATIC_COLLIDER_SIMPLIFICATION_TYPE_DEFAULT",
    "SETTINGS_TOOLBAR_BUTTONS_WITH_TEXT",
    "SETTINGS_TOOLBAR_BUTTONS_WITH_TEXT_DEFAULT",
    "SupportUiColliderType",
    "SupportUiDynamicColliderSimplificationType",
    "SupportUiEventType",
    "SupportUiRigidBodyManipulator",
    "SupportUiStaticColliderSimplificationType",
    "acquire_physx_supportui_interface",
    "acquire_physx_supportui_private_interface",
    "acquire_rigid_body_manipulator",
    "release_physx_supportui_interface",
    "release_physx_supportui_interface_scripting",
    "release_physx_supportui_private_interface",
    "release_physx_supportui_private_interface_scripting",
    "release_rigid_body_manipulator"
]


class IPhysxSupportUi():
    def clear_colliders_processing_queue(self) -> None: 
        """
        Clears colliders processing queue which effectively stops coll. creation.
        """
    def create_colliders(self, arg0: int, arg1: SupportUiColliderType) -> bool: 
        """
        Attempts to create collider(s) on supplied prim and all its children.
        In case of static collider type, existing rigid bodies are removed.
        In case of dynamic collider type, rigid body is created if there
        was none.
        Rules for rigid body creation:
        1) Rigid body should not be created if there is other rigid body up
           in the hierarchy. Warning will be produced.
        2) The most common node, when rigid bodies are created, is a node
           with 'kind' set to 'component'.
        3) In case there is no component kind, the top most node ('primPath')
           gets rigid body created.

        NOTE: This method just queues prims for collider creation, which
              happens later, batched together with other coll. requests.

        Args:
           primPath: prim path (uint64_t -> const pxr::SdfPath&)
           colliderType: collider type specified by SupportUiColliderType enum
        """
    def get_event_stream(self) -> carb.events._events.IEventStream: 
        """
        Event stream sending various events defined in SupportUiEvent enum.

        Returns:
            Event stream sending the events.
        """
    def get_num_of_colliders_to_create(self) -> int: 
        """
        Returns:
           return number of queued colliders yet to create.
        """
    pass
class IPhysxSupportUiPrivate():
    def commit_authoring_state(self) -> None: ...
    def enable_inspector_authoring_mode(self) -> None: ...
    def enable_notice_handler(self, arg0: bool) -> None: 
        """
        Enables USD Notice handler
        """
    def get_inspector_event_stream(self) -> carb.events._events.IEventStream: 
        """
        Gets the event stream of the Inspector
        """
    def get_inspector_state(self) -> PhysXInspectorModelState: 
        """
        Gets the state of the Inspector (returns PhysXInspectorModelState).
        """
    def refresh_all_inspector_models_structure(self) -> None: 
        """
        Refresh all inspector models structure (rebuilds joints hierarchy)
        """
    def refresh_all_inspector_models_values(self) -> None: 
        """
        Refresh all inspector models values (does not rebuild joints hierarchy)
        """
    def reset_inspector_to_authoring_start(self) -> None: ...
    def step_inspector_simulation(self, arg0: float) -> None: 
        """
        Step inspector simulation
        """
    pass
class PhysXInspectorModel(omni.ui._ui.AbstractItemModel):
    def __init__(self, arg0: typing.List[str]) -> None: ...
    def get_control_type_model(self) -> omni.ui._ui.SimpleStringModel: ...
    def get_data_shape_model(self) -> omni.ui._ui.SimpleIntModel: ...
    def get_enable_gravity_model(self) -> omni.ui._ui.SimpleBoolModel: ...
    def get_enable_quasi_static_mode_model(self) -> omni.ui._ui.SimpleBoolModel: ...
    def get_fix_articulation_base_model(self) -> omni.ui._ui.SimpleBoolModel: ...
    def get_inspected_prim_string_model(self) -> omni.ui._ui.SimpleStringModel: ...
    def get_inspector_type(self) -> PhysXInspectorModelInspectorType: ...
    def get_items_matching_paths(self, paths: typing.List[str]) -> typing.List[omni.ui._ui.AbstractItem]: ...
    def get_joint_value_attribute_name(self, arg0: str) -> str: ...
    def get_property_type_model(self) -> omni.ui._ui.SimpleStringModel: ...
    def get_show_masses_and_inertia_model(self) -> omni.ui._ui.SimpleBoolModel: ...
    def refresh_model_structure(self) -> None: ...
    def refresh_model_values(self) -> None: ...
    def select_all_connected_body_joints(self) -> None: ...
    def select_all_connected_body_shapes(self) -> None: ...
    def select_all_connected_joint_shapes(self) -> None: ...
    def select_all_connected_links(self) -> None: ...
    def set_inspector_type(self, arg0: PhysXInspectorModelInspectorType) -> None: ...
    def set_joint_value(self, arg0: str, arg1: float) -> bool: ...
    def set_selected_paths(self, arg0: typing.List[str]) -> None: ...
    pass
class PhysXInspectorModelControlType():
    """
    Control Type

    Members:

      AUTOMATIC : PhysXInspectorModel::ControlType::eAutomatic

      JOINT_STATE : PhysXInspectorModel::ControlType::eJointState

      JOINT_DRIVE : PhysXInspectorModel::ControlType::eJointDrive

      JOINT_VELOCITY : PhysXInspectorModel::ControlType::eJointVelocity
    """
    def __eq__(self, other: object) -> bool: ...
    def __getstate__(self) -> int: ...
    def __hash__(self) -> int: ...
    def __index__(self) -> int: ...
    def __init__(self, value: int) -> None: ...
    def __int__(self) -> int: ...
    def __ne__(self, other: object) -> bool: ...
    def __repr__(self) -> str: ...
    def __setstate__(self, state: int) -> None: ...
    @property
    def name(self) -> str:
        """
        :type: str
        """
    @property
    def value(self) -> int:
        """
        :type: int
        """
    AUTOMATIC: omni.physxsupportui.bindings._physxSupportUi.PhysXInspectorModelControlType # value = <PhysXInspectorModelControlType.AUTOMATIC: 0>
    JOINT_DRIVE: omni.physxsupportui.bindings._physxSupportUi.PhysXInspectorModelControlType # value = <PhysXInspectorModelControlType.JOINT_DRIVE: 2>
    JOINT_STATE: omni.physxsupportui.bindings._physxSupportUi.PhysXInspectorModelControlType # value = <PhysXInspectorModelControlType.JOINT_STATE: 1>
    JOINT_VELOCITY: omni.physxsupportui.bindings._physxSupportUi.PhysXInspectorModelControlType # value = <PhysXInspectorModelControlType.JOINT_VELOCITY: 3>
    __members__: dict # value = {'AUTOMATIC': <PhysXInspectorModelControlType.AUTOMATIC: 0>, 'JOINT_STATE': <PhysXInspectorModelControlType.JOINT_STATE: 1>, 'JOINT_DRIVE': <PhysXInspectorModelControlType.JOINT_DRIVE: 2>, 'JOINT_VELOCITY': <PhysXInspectorModelControlType.JOINT_VELOCITY: 3>}
    pass
class PhysXInspectorModelDataShapeType():
    """
    Data Shape Type

    Members:

      FLAT : PhysXInspectorModel::DataShapeType::eFlat

      HIERARCHICAL : PhysXInspectorModel::DataShapeType::eHierarchical
    """
    def __eq__(self, other: object) -> bool: ...
    def __getstate__(self) -> int: ...
    def __hash__(self) -> int: ...
    def __index__(self) -> int: ...
    def __init__(self, value: int) -> None: ...
    def __int__(self) -> int: ...
    def __ne__(self, other: object) -> bool: ...
    def __repr__(self) -> str: ...
    def __setstate__(self, state: int) -> None: ...
    @property
    def name(self) -> str:
        """
        :type: str
        """
    @property
    def value(self) -> int:
        """
        :type: int
        """
    FLAT: omni.physxsupportui.bindings._physxSupportUi.PhysXInspectorModelDataShapeType # value = <PhysXInspectorModelDataShapeType.FLAT: 0>
    HIERARCHICAL: omni.physxsupportui.bindings._physxSupportUi.PhysXInspectorModelDataShapeType # value = <PhysXInspectorModelDataShapeType.HIERARCHICAL: 1>
    __members__: dict # value = {'FLAT': <PhysXInspectorModelDataShapeType.FLAT: 0>, 'HIERARCHICAL': <PhysXInspectorModelDataShapeType.HIERARCHICAL: 1>}
    pass
class PhysXInspectorModelInspectorType():
    """
    Inspection Type

    Members:

      INSPECTOR_TYPE_JOINTS_LIST : PhysXInspectorModel::InspectorType::eJointsList

      INSPECTOR_TYPE_JOINTS_BODIES_HIERARCHY : PhysXInspectorModel::InspectorType::eJointsBodiesHierarchy
    """
    def __eq__(self, other: object) -> bool: ...
    def __getstate__(self) -> int: ...
    def __hash__(self) -> int: ...
    def __index__(self) -> int: ...
    def __init__(self, value: int) -> None: ...
    def __int__(self) -> int: ...
    def __ne__(self, other: object) -> bool: ...
    def __repr__(self) -> str: ...
    def __setstate__(self, state: int) -> None: ...
    @property
    def name(self) -> str:
        """
        :type: str
        """
    @property
    def value(self) -> int:
        """
        :type: int
        """
    INSPECTOR_TYPE_JOINTS_BODIES_HIERARCHY: omni.physxsupportui.bindings._physxSupportUi.PhysXInspectorModelInspectorType # value = <PhysXInspectorModelInspectorType.INSPECTOR_TYPE_JOINTS_BODIES_HIERARCHY: 1>
    INSPECTOR_TYPE_JOINTS_LIST: omni.physxsupportui.bindings._physxSupportUi.PhysXInspectorModelInspectorType # value = <PhysXInspectorModelInspectorType.INSPECTOR_TYPE_JOINTS_LIST: 0>
    __members__: dict # value = {'INSPECTOR_TYPE_JOINTS_LIST': <PhysXInspectorModelInspectorType.INSPECTOR_TYPE_JOINTS_LIST: 0>, 'INSPECTOR_TYPE_JOINTS_BODIES_HIERARCHY': <PhysXInspectorModelInspectorType.INSPECTOR_TYPE_JOINTS_BODIES_HIERARCHY: 1>}
    pass
class PhysXInspectorModelPropertyType():
    """
    Property Type

    Members:

      SHOW_LIMITS : PhysXInspectorModel::PropertyType::eShowLimits

      SHOW_GAINS : PhysXInspectorModel::PropertyType::eShowGains
    """
    def __eq__(self, other: object) -> bool: ...
    def __getstate__(self) -> int: ...
    def __hash__(self) -> int: ...
    def __index__(self) -> int: ...
    def __init__(self, value: int) -> None: ...
    def __int__(self) -> int: ...
    def __ne__(self, other: object) -> bool: ...
    def __repr__(self) -> str: ...
    def __setstate__(self, state: int) -> None: ...
    @property
    def name(self) -> str:
        """
        :type: str
        """
    @property
    def value(self) -> int:
        """
        :type: int
        """
    SHOW_GAINS: omni.physxsupportui.bindings._physxSupportUi.PhysXInspectorModelPropertyType # value = <PhysXInspectorModelPropertyType.SHOW_GAINS: 1>
    SHOW_LIMITS: omni.physxsupportui.bindings._physxSupportUi.PhysXInspectorModelPropertyType # value = <PhysXInspectorModelPropertyType.SHOW_LIMITS: 0>
    __members__: dict # value = {'SHOW_LIMITS': <PhysXInspectorModelPropertyType.SHOW_LIMITS: 0>, 'SHOW_GAINS': <PhysXInspectorModelPropertyType.SHOW_GAINS: 1>}
    pass
class PhysXInspectorModelState():
    """
    Members:

      DISABLED

      AUTHORING

      RUNNING_SIMULATION
    """
    def __eq__(self, other: object) -> bool: ...
    def __getstate__(self) -> int: ...
    def __hash__(self) -> int: ...
    def __index__(self) -> int: ...
    def __init__(self, value: int) -> None: ...
    def __int__(self) -> int: ...
    def __ne__(self, other: object) -> bool: ...
    def __repr__(self) -> str: ...
    def __setstate__(self, state: int) -> None: ...
    @property
    def name(self) -> str:
        """
        :type: str
        """
    @property
    def value(self) -> int:
        """
        :type: int
        """
    AUTHORING: omni.physxsupportui.bindings._physxSupportUi.PhysXInspectorModelState # value = <PhysXInspectorModelState.AUTHORING: 1>
    DISABLED: omni.physxsupportui.bindings._physxSupportUi.PhysXInspectorModelState # value = <PhysXInspectorModelState.DISABLED: 0>
    RUNNING_SIMULATION: omni.physxsupportui.bindings._physxSupportUi.PhysXInspectorModelState # value = <PhysXInspectorModelState.RUNNING_SIMULATION: 2>
    __members__: dict # value = {'DISABLED': <PhysXInspectorModelState.DISABLED: 0>, 'AUTHORING': <PhysXInspectorModelState.AUTHORING: 1>, 'RUNNING_SIMULATION': <PhysXInspectorModelState.RUNNING_SIMULATION: 2>}
    pass
class PhysXInspectorWidget(omni.ui._ui.Widget):
    def __init__(self, **kwargs) -> None: ...
    @property
    def inspector_type(self) -> PhysXInspectorModelInspectorType:
        """
        :type: PhysXInspectorModelInspectorType
        """
    @inspector_type.setter
    def inspector_type(self, arg1: PhysXInspectorModelInspectorType) -> None:
        pass
    @property
    def model(self) -> PhysXInspectorModel:
        """
        :type: PhysXInspectorModel
        """
    @model.setter
    def model(self, arg1: PhysXInspectorModel) -> None:
        pass
    FLAG_WANT_CAPTURE_KEYBOARD = 1073741824
    pass
class PhysxInspectorOverlay(omni.ui_scene._scene.Manipulator, omni.ui_scene._scene.AbstractContainer, omni.ui_scene._scene.AbstractItem):
    def __init__(self, **kwargs) -> None: ...
    pass
class SupportUiColliderType():
    """
    SupportUi collider type used upon collider creation.

    Members:

      AUTODETECT : IPhysxSupportUi::ColliderType::eAutodetect

      STATIC : IPhysxSupportUi::ColliderType::eStatic

      DYNAMIC : IPhysxSupportUi::ColliderType::eDynamic
    """
    def __eq__(self, other: object) -> bool: ...
    def __getstate__(self) -> int: ...
    def __hash__(self) -> int: ...
    def __index__(self) -> int: ...
    def __init__(self, value: int) -> None: ...
    def __int__(self) -> int: ...
    def __ne__(self, other: object) -> bool: ...
    def __repr__(self) -> str: ...
    def __setstate__(self, state: int) -> None: ...
    @property
    def name(self) -> str:
        """
        :type: str
        """
    @property
    def value(self) -> int:
        """
        :type: int
        """
    AUTODETECT: omni.physxsupportui.bindings._physxSupportUi.SupportUiColliderType # value = <SupportUiColliderType.AUTODETECT: 0>
    DYNAMIC: omni.physxsupportui.bindings._physxSupportUi.SupportUiColliderType # value = <SupportUiColliderType.DYNAMIC: 2>
    STATIC: omni.physxsupportui.bindings._physxSupportUi.SupportUiColliderType # value = <SupportUiColliderType.STATIC: 1>
    __members__: dict # value = {'AUTODETECT': <SupportUiColliderType.AUTODETECT: 0>, 'STATIC': <SupportUiColliderType.STATIC: 1>, 'DYNAMIC': <SupportUiColliderType.DYNAMIC: 2>}
    pass
class SupportUiDynamicColliderSimplificationType():
    """
    SupportUi dynamic collider simplification type used upon collider creation.

    Members:

      CONVEX_HULL : IPhysxSupportUi::DynamicColliderSimplificationType::eConvexHull

      CONVEX_DECOMPOSITION : IPhysxSupportUi::DynamicColliderSimplificationType::eConvexDecomposition

      SDF : IPhysxSupportUi::DynamicColliderSimplificationType::eSDF
    """
    def __eq__(self, other: object) -> bool: ...
    def __getstate__(self) -> int: ...
    def __hash__(self) -> int: ...
    def __index__(self) -> int: ...
    def __init__(self, value: int) -> None: ...
    def __int__(self) -> int: ...
    def __ne__(self, other: object) -> bool: ...
    def __repr__(self) -> str: ...
    def __setstate__(self, state: int) -> None: ...
    @property
    def name(self) -> str:
        """
        :type: str
        """
    @property
    def value(self) -> int:
        """
        :type: int
        """
    CONVEX_DECOMPOSITION: omni.physxsupportui.bindings._physxSupportUi.SupportUiDynamicColliderSimplificationType # value = <SupportUiDynamicColliderSimplificationType.CONVEX_DECOMPOSITION: 1>
    CONVEX_HULL: omni.physxsupportui.bindings._physxSupportUi.SupportUiDynamicColliderSimplificationType # value = <SupportUiDynamicColliderSimplificationType.CONVEX_HULL: 0>
    SDF: omni.physxsupportui.bindings._physxSupportUi.SupportUiDynamicColliderSimplificationType # value = <SupportUiDynamicColliderSimplificationType.SDF: 2>
    __members__: dict # value = {'CONVEX_HULL': <SupportUiDynamicColliderSimplificationType.CONVEX_HULL: 0>, 'CONVEX_DECOMPOSITION': <SupportUiDynamicColliderSimplificationType.CONVEX_DECOMPOSITION: 1>, 'SDF': <SupportUiDynamicColliderSimplificationType.SDF: 2>}
    pass
class SupportUiEventType():
    """
    SupportUi events used by event stream.

    Members:

      COLLIDER_CREATED : When collider has been created; contains the following in dictionary:
                    'primPath': int2 - Usd path to the prim decoded into two ints. PhysicsSchemaTools.decodeSdfPath will return SdfPath.
                    'colliderType': bool - collider type. false = static, true = dynamic
                    'simplificationType': int - if collType is static then StaticColliderSimplificationType, otherwise DynamicColliderSimplificationType
                    'numRemainingCollTasks': int - number of remaining collision tasks
                    'numTotalCollTasks': int - total number of collision tasks
                

      RIGID_BODY_CREATED : When a rigid body API has been applied; contains the following in dictionary:
                    'primPath': int2 - Usd path to the prim decoded into two ints. PhysicsSchemaTools.decodeSdfPath will return SdfPath.
                

      RIGID_BODY_REMOVED : When a rigid body API has been removed; contains the following in dictionary:
                    'primPath': int2 - Usd path to the prim decoded into two ints. PhysicsSchemaTools.decodeSdfPath will return SdfPath.
                

      KINEMATICS_TOGGLED : When a kinematic rigid body flag was toggled; contains the following in dictionary:
                    'primPath': int2 - Usd path to the prim decoded into two ints. PhysicsSchemaTools.decodeSdfPath will return SdfPath.
                

      AUTO_COLL_CANCELED : When an applied physics/physx schema on prim is detected, automatic collider creation is canceled; contains the following in dictionary:
                    'primPath': int2 - Usd path to the prim containing the collider decoded into two ints. PhysicsSchemaTools.decodeSdfPath will return SdfPath.
                
    """
    def __eq__(self, other: object) -> bool: ...
    def __getstate__(self) -> int: ...
    def __hash__(self) -> int: ...
    def __index__(self) -> int: ...
    def __init__(self, value: int) -> None: ...
    def __int__(self) -> int: ...
    def __ne__(self, other: object) -> bool: ...
    def __repr__(self) -> str: ...
    def __setstate__(self, state: int) -> None: ...
    @property
    def name(self) -> str:
        """
        :type: str
        """
    @property
    def value(self) -> int:
        """
        :type: int
        """
    AUTO_COLL_CANCELED: omni.physxsupportui.bindings._physxSupportUi.SupportUiEventType # value = <SupportUiEventType.AUTO_COLL_CANCELED: 4>
    COLLIDER_CREATED: omni.physxsupportui.bindings._physxSupportUi.SupportUiEventType # value = <SupportUiEventType.COLLIDER_CREATED: 0>
    KINEMATICS_TOGGLED: omni.physxsupportui.bindings._physxSupportUi.SupportUiEventType # value = <SupportUiEventType.KINEMATICS_TOGGLED: 3>
    RIGID_BODY_CREATED: omni.physxsupportui.bindings._physxSupportUi.SupportUiEventType # value = <SupportUiEventType.RIGID_BODY_CREATED: 1>
    RIGID_BODY_REMOVED: omni.physxsupportui.bindings._physxSupportUi.SupportUiEventType # value = <SupportUiEventType.RIGID_BODY_REMOVED: 2>
    __members__: dict # value = {'COLLIDER_CREATED': <SupportUiEventType.COLLIDER_CREATED: 0>, 'RIGID_BODY_CREATED': <SupportUiEventType.RIGID_BODY_CREATED: 1>, 'RIGID_BODY_REMOVED': <SupportUiEventType.RIGID_BODY_REMOVED: 2>, 'KINEMATICS_TOGGLED': <SupportUiEventType.KINEMATICS_TOGGLED: 3>, 'AUTO_COLL_CANCELED': <SupportUiEventType.AUTO_COLL_CANCELED: 4>}
    pass
class SupportUiRigidBodyManipulator():
    def manipulation_began(self, path: int) -> None: 
        """
        Must be called when manipulation begins, before calling move() or rotate().

        Args:
            'path': int2 - Usd path to the manipulated prim.
        """
    def manipulation_ended(self, path: int) -> None: 
        """
        Must be called when manipulation ends, always in pairs with manipulation_began().

        Args:
            'path': int2 - Usd path to the manipulated prim.
        """
    def move(self, path: int, delta_translation: carb._carb.Float3, lock_rotation: bool, lock_translation: bool) -> bool: 
        """
        Moves an object using physics force.

        Args:
            'path': int2 - Usd path to the manipulated prim.
            'delta_translation': float3 - translation delta.
            'lock_rotation': bool - allow/disallow rotation while translating.
            'lock_translation': bool - allow/disallow translating on other axes while translating.
        """
    def rotate(self, path: int, pivot_world_pos: carb._carb.Float3, delta_rotation: carb._carb.Float4, lock_rotation: bool, lock_translation: bool) -> bool: 
        """
        Rotates an object using physics force.

        Args:
            'path': int2 - Usd path to the manipulated prim.
            'pivot_world_pos': float3 - world pivot position to rotate around.
            'delta_rotation': float4 - target rotation quaternion.
            'lock_rotation': bool - allow/disallow rotation on other axes while rotating.
            'lock_translation': bool - allow/disallow translating while rotating.
        """
    pass
class SupportUiStaticColliderSimplificationType():
    """
    SupportUi static collider simplification type used upon collider creation.

    Members:

      NONE : IPhysxSupportUi::StaticColliderSimplificationType::eNone

      MESH : IPhysxSupportUi::StaticColliderSimplificationType::eMeshSimplification
    """
    def __eq__(self, other: object) -> bool: ...
    def __getstate__(self) -> int: ...
    def __hash__(self) -> int: ...
    def __index__(self) -> int: ...
    def __init__(self, value: int) -> None: ...
    def __int__(self) -> int: ...
    def __ne__(self, other: object) -> bool: ...
    def __repr__(self) -> str: ...
    def __setstate__(self, state: int) -> None: ...
    @property
    def name(self) -> str:
        """
        :type: str
        """
    @property
    def value(self) -> int:
        """
        :type: int
        """
    MESH: omni.physxsupportui.bindings._physxSupportUi.SupportUiStaticColliderSimplificationType # value = <SupportUiStaticColliderSimplificationType.MESH: 1>
    NONE: omni.physxsupportui.bindings._physxSupportUi.SupportUiStaticColliderSimplificationType # value = <SupportUiStaticColliderSimplificationType.NONE: 0>
    __members__: dict # value = {'NONE': <SupportUiStaticColliderSimplificationType.NONE: 0>, 'MESH': <SupportUiStaticColliderSimplificationType.MESH: 1>}
    pass
def acquire_physx_supportui_interface(plugin_name: str = None, library_path: str = None) -> IPhysxSupportUi:
    pass
def acquire_physx_supportui_private_interface(plugin_name: str = None, library_path: str = None) -> IPhysxSupportUiPrivate:
    pass
def acquire_rigid_body_manipulator(plugin_name: str = None, library_path: str = None) -> SupportUiRigidBodyManipulator:
    pass
def release_physx_supportui_interface(arg0: IPhysxSupportUi) -> None:
    pass
def release_physx_supportui_interface_scripting(arg0: IPhysxSupportUi) -> None:
    pass
def release_physx_supportui_private_interface(arg0: IPhysxSupportUiPrivate) -> None:
    pass
def release_physx_supportui_private_interface_scripting(arg0: IPhysxSupportUiPrivate) -> None:
    pass
def release_rigid_body_manipulator(arg0: SupportUiRigidBodyManipulator) -> None:
    pass
SETTINGS_ACTION_BAR_ENABLED = '/persistent/physics/supportUiActionBarEnabled'
SETTINGS_ACTION_BAR_ENABLED_DEFAULT = '/defaults/persistent/physics/supportUiActionBarEnabled'
SETTINGS_ASYNC_COOKING_AT_STAGE_LOAD = '/persistent/physics/supportUiAsyncCookingAtStageLoad'
SETTINGS_ASYNC_COOKING_AT_STAGE_LOAD_DEFAULT = '/defaults/persistent/physics/supportUiAsyncCookingAtStageLoad'
SETTINGS_AUTOMATIC_COLLIDER_CREATION_ENABLED = '/persistent/physics/supportUiAutomaticCollisionCreationEnabled'
SETTINGS_AUTOMATIC_COLLIDER_CREATION_ENABLED_DEFAULT = '/defaults/persistent/physics/supportUiAutomaticCollisionCreationEnabled'
SETTINGS_AVOID_CHANGING_EXISTING_COLLIDERS = '/persistent/physics/supportUiAvoidChangingExistingColliders'
SETTINGS_AVOID_CHANGING_EXISTING_COLLIDERS_DEFAULT = '/defaults/persistent/physics/supportUiAvoidChangingExistingColliders'
SETTINGS_CUSTOM_MANIPULATOR_ENABLED = '/persistent/physics/supportUiCustomManipulatorEnabled'
SETTINGS_CUSTOM_MANIPULATOR_ENABLED_DEFAULT = '/defaults/persistent/physics/supportUiCustomManipulatorEnabled'
SETTINGS_DYNAMIC_COLLIDER_SIMPLIFICATION_TYPE = '/persistent/physics/supportUiDynamicColliderSimplificationType'
SETTINGS_DYNAMIC_COLLIDER_SIMPLIFICATION_TYPE_DEFAULT = '/defaults/persistent/physics/supportUiDynamicColliderSimplificationType'
SETTINGS_FLOATING_NOTIFICATIONS_ENABLED = '/persistent/physics/supportUiFloatingNotificationsEnabled'
SETTINGS_FLOATING_NOTIFICATIONS_ENABLED_DEFAULT = '/defaults/persistent/physics/supportUiFloatingNotificationsEnabled'
SETTINGS_LOGGING_ENABLED = '/persistent/physics/supportUiLoggingEnabled'
SETTINGS_LOGGING_ENABLED_DEFAULT = '/defaults/persistent/physics/supportUiLoggingEnabled'
SETTINGS_MANIP_MODE_ALLOW_RIGID_BODY_TRAVERSAL = '/persistent/physics/supportUiManipModeAllowRigidBodyTraversal'
SETTINGS_MANIP_MODE_ALLOW_RIGID_BODY_TRAVERSAL_DEFAULT = '/defaults/persistent/physics/supportUiManipModeAllowRigidBodyTraversal'
SETTINGS_MANIP_MODE_ALLOW_ROT_ON_OTHER_AXES_WHILE_ROTATING = '/persistent/physics/supportUiManipModeAllowRotOnOtherAxesWhileRotating'
SETTINGS_MANIP_MODE_ALLOW_ROT_ON_OTHER_AXES_WHILE_ROTATING_DEFAULT = '/defaults/persistent/physics/supportUiManipModeAllowRotOnOtherAxesWhileRotating'
SETTINGS_MANIP_MODE_ALLOW_ROT_WHILE_TRANSLATING = '/persistent/physics/supportUiManipModeAllowRotWhileTranslating'
SETTINGS_MANIP_MODE_ALLOW_ROT_WHILE_TRANSLATING_DEFAULT = '/defaults/persistent/physics/supportUiManipModeAllowRotWhileTranslating'
SETTINGS_MANIP_MODE_ALLOW_TRAN_ON_OTHER_AXES_WHILE_TRANSLATING = '/persistent/physics/supportUiManipModeAllowTranOnOtherAxesWhileTranslating'
SETTINGS_MANIP_MODE_ALLOW_TRAN_ON_OTHER_AXES_WHILE_TRANSLATING_DEFAULT = '/defaults/persistent/physics/supportUiManipModeAllowTranOnOtherAxesWhileTranslating'
SETTINGS_MANIP_MODE_ALLOW_TRAN_WHILE_ROTATING = '/persistent/physics/supportUiManipModeAllowTranWhileRotating'
SETTINGS_MANIP_MODE_ALLOW_TRAN_WHILE_ROTATING_DEFAULT = '/defaults/persistent/physics/supportUiManipModeAllowTranWhileRotating'
SETTINGS_PHYSICS_INSPECTOR_ENABLED = '/physics/supportUiPhysicsInspector/enabled'
SETTINGS_RIGID_BODY_SELECTION_MODE_ENABLED = '/physics/supportUiRigidBodySelectionModeEnabled'
SETTINGS_RIGID_BODY_SELECTION_MODE_ENABLED_DEFAULT = '/defaults/physics/supportUiRigidBodySelectionModeEnabled'
SETTINGS_STATIC_COLLIDER_SIMPLIFICATION_TYPE = '/persistent/physics/supportUiStaticColliderSimplificationType'
SETTINGS_STATIC_COLLIDER_SIMPLIFICATION_TYPE_DEFAULT = '/defaults/persistent/physics/supportUiStaticColliderSimplificationType'
SETTINGS_TOOLBAR_BUTTONS_WITH_TEXT = '/persistent/physics/supportUiToolbarButtonsWithText'
SETTINGS_TOOLBAR_BUTTONS_WITH_TEXT_DEFAULT = '/defaults/persistent/physics/supportUiToolbarButtonsWithText'
