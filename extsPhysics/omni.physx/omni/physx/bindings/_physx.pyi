"""
        This module contains python bindings to the C++ omni::physx interface.
        Omni::physx contains several interfaces::
            PhysX -- Main interface used for physics simulation.
            PhysXVisualization -- Interface for debug visualization control.
            PhysXUnitTests -- Interface for unit tests.
    """
from __future__ import annotations
import omni.physx.bindings._physx
import typing
import carb._carb
import carb.events._events

__all__ = [
    "ContactData",
    "ContactDataVector",
    "ContactEventHeader",
    "ContactEventHeaderVector",
    "ContactEventType",
    "ErrorEvent",
    "FrictionAnchor",
    "FrictionAnchorsDataVector",
    "IPhysxAttachment",
    "IPhysxBenchmarks",
    "IPhysxPropertyQuery",
    "IPhysxReplicator",
    "IPhysxSimulation",
    "IPhysxStageUpdate",
    "IPhysxStatistics",
    "METADATA_ATTRIBUTE_NAME_LOCALSPACEVELOCITIES",
    "OverlapHit",
    "ParticleVisualizationPositionType",
    "ParticleVisualizationRadiusType",
    "PhysX",
    "PhysXCooking",
    "PhysXSceneQuery",
    "PhysXUnitTests",
    "PhysXVisualization",
    "PhysicsInteractionEvent",
    "PhysicsSceneStats",
    "PhysxCollisionRepresentationResult",
    "PhysxCollisionRepresentationTask",
    "PhysxConvexMeshData",
    "PhysxConvexMeshPolygon",
    "PhysxPropertyQueryColliderResponse",
    "PhysxPropertyQueryMode",
    "PhysxPropertyQueryResult",
    "PhysxPropertyQueryRigidBodyResponse",
    "PhysxPropertyQueryRigidBodyResponseType",
    "RaycastHit",
    "SETTING_ADDMENU_SELECTION_LIMIT",
    "SETTING_ADDMENU_SELECTION_LIMIT_DEFAULT",
    "SETTING_ADDMENU_SUBTREE_LIMIT",
    "SETTING_ADDMENU_SUBTREE_LIMIT_DEFAULT",
    "SETTING_AUTOCREATE_PHYSICS_SCENE",
    "SETTING_AUTOCREATE_PHYSICS_SCENE_DEFAULT",
    "SETTING_AUTO_POPUP_SIMULATION_OUTPUT_WINDOW",
    "SETTING_AUTO_POPUP_SIMULATION_OUTPUT_WINDOW_DEFAULT",
    "SETTING_BACKWARD_COMPATIBILITY",
    "SETTING_BACKWARD_COMPATIBILITY_DEFAULT",
    "SETTING_COLLISION_CONE_CUSTOM_GEOMETRY",
    "SETTING_COLLISION_CONE_CUSTOM_GEOMETRY_DEFAULT",
    "SETTING_COLLISION_CYLINDER_CUSTOM_GEOMETRY",
    "SETTING_COLLISION_CYLINDER_CUSTOM_GEOMETRY_DEFAULT",
    "SETTING_CUDA_DEVICE",
    "SETTING_CUDA_DEVICE_DEFAULT",
    "SETTING_DEBUG_VIS_QUERY_USDRT_FOR_TRAVERSAL",
    "SETTING_DEBUG_VIS_QUERY_USDRT_FOR_TRAVERSAL_DEFAULT",
    "SETTING_DEBUG_VIS_SIMPLIFY_AT_DISTANCE",
    "SETTING_DEBUG_VIS_SIMPLIFY_AT_DISTANCE_DEFAULT",
    "SETTING_DEFAULT_SIMULATOR",
    "SETTING_DEFAULT_SIMULATOR_DEFAULT",
    "SETTING_DEMO_ASSETS_PATH",
    "SETTING_DEMO_ASSETS_PATH_DEFAULT",
    "SETTING_DISABLE_CONTACT_PROCESSING",
    "SETTING_DISABLE_CONTACT_PROCESSING_DEFAULT",
    "SETTING_DISABLE_SLEEPING",
    "SETTING_DISABLE_SLEEPING_DEFAULT",
    "SETTING_DISPLAY_ATTACHMENTS",
    "SETTING_DISPLAY_ATTACHMENTS_DEFAULT",
    "SETTING_DISPLAY_ATTACHMENTS_HIDE_ACTOR_0",
    "SETTING_DISPLAY_ATTACHMENTS_HIDE_ACTOR_0_DEFAULT",
    "SETTING_DISPLAY_ATTACHMENTS_HIDE_ACTOR_1",
    "SETTING_DISPLAY_ATTACHMENTS_HIDE_ACTOR_1_DEFAULT",
    "SETTING_DISPLAY_COLLIDERS",
    "SETTING_DISPLAY_COLLIDERS_DEFAULT",
    "SETTING_DISPLAY_COLLIDER_NORMALS",
    "SETTING_DISPLAY_COLLIDER_NORMALS_DEFAULT",
    "SETTING_DISPLAY_DEFORMABLE_BODIES",
    "SETTING_DISPLAY_DEFORMABLE_BODIES_DEFAULT",
    "SETTING_DISPLAY_DEFORMABLE_BODY_TYPE",
    "SETTING_DISPLAY_DEFORMABLE_BODY_TYPE_DEFAULT",
    "SETTING_DISPLAY_DEFORMABLE_SURFACES",
    "SETTING_DISPLAY_DEFORMABLE_SURFACES_DEFAULT",
    "SETTING_DISPLAY_JOINTS",
    "SETTING_DISPLAY_JOINTS_DEFAULT",
    "SETTING_DISPLAY_MASS_PROPERTIES",
    "SETTING_DISPLAY_MASS_PROPERTIES_DEFAULT",
    "SETTING_DISPLAY_PARTICLES",
    "SETTING_DISPLAY_PARTICLES_DEFAULT",
    "SETTING_DISPLAY_PARTICLES_POSITION_TYPE",
    "SETTING_DISPLAY_PARTICLES_POSITION_TYPE_DEFAULT",
    "SETTING_DISPLAY_PARTICLES_RADIUS_TYPE",
    "SETTING_DISPLAY_PARTICLES_RADIUS_TYPE_DEFAULT",
    "SETTING_DISPLAY_PARTICLES_SHOW_CLOTH_MESH",
    "SETTING_DISPLAY_PARTICLES_SHOW_CLOTH_MESH_DEFAULT",
    "SETTING_DISPLAY_PARTICLES_SHOW_CLOTH_MESH_LINES",
    "SETTING_DISPLAY_PARTICLES_SHOW_CLOTH_MESH_LINES_DEFAULT",
    "SETTING_DISPLAY_PARTICLES_SHOW_CLOTH_PARTICLES",
    "SETTING_DISPLAY_PARTICLES_SHOW_CLOTH_PARTICLES_DEFAULT",
    "SETTING_DISPLAY_PARTICLES_SHOW_DIFFUSE",
    "SETTING_DISPLAY_PARTICLES_SHOW_DIFFUSE_DEFAULT",
    "SETTING_DISPLAY_PARTICLES_SHOW_FLUID_SURFACE",
    "SETTING_DISPLAY_PARTICLES_SHOW_FLUID_SURFACE_DEFAULT",
    "SETTING_DISPLAY_PARTICLES_SHOW_PARTICLE_SET_PARTICLES",
    "SETTING_DISPLAY_PARTICLES_SHOW_PARTICLE_SET_PARTICLES_DEFAULT",
    "SETTING_DISPLAY_SIMULATION_DATA_VISUALIZER",
    "SETTING_DISPLAY_SIMULATION_DATA_VISUALIZER_DEFAULT",
    "SETTING_DISPLAY_SIMULATION_OUTPUT",
    "SETTING_DISPLAY_SIMULATION_OUTPUT_DEFAULT",
    "SETTING_DISPLAY_TENDONS",
    "SETTING_DISPLAY_TENDONS_DEFAULT",
    "SETTING_ENABLE_SYNCHRONOUS_KERNEL_LAUNCHES",
    "SETTING_ENABLE_SYNCHRONOUS_KERNEL_LAUNCHES_DEFAULT",
    "SETTING_EXPOSE_PRIM_PATH_NAMES",
    "SETTING_EXPOSE_PRIM_PATH_NAMES_DEFAULT",
    "SETTING_EXPOSE_PROFILER_DATA",
    "SETTING_EXPOSE_PROFILER_DATA_DEFAULT",
    "SETTING_FORCE_PARSE_ONLY_SINGLE_SCENE",
    "SETTING_FORCE_PARSE_ONLY_SINGLE_SCENE_DEFAULT",
    "SETTING_JOINT_BODY_TRANSFORM_CHECK_TOLERANCE",
    "SETTING_JOINT_BODY_TRANSFORM_CHECK_TOLERANCE_DEFAULT",
    "SETTING_LOCAL_MESH_CACHE_SIZE_MB",
    "SETTING_LOCAL_MESH_CACHE_SIZE_MB_DEFAULT",
    "SETTING_LOG_ROBOTICS",
    "SETTING_LOG_ROBOTICS_DEFAULT",
    "SETTING_LOG_SCENEMULTIGPU",
    "SETTING_LOG_SCENEMULTIGPU_DEFAULT",
    "SETTING_MASS_DISTRIBUTION_MANIPULATOR",
    "SETTING_MASS_DISTRIBUTION_MANIPULATOR_DEFAULT",
    "SETTING_MAX_NUMBER_OF_PHYSX_ERRORS",
    "SETTING_MAX_NUMBER_OF_PHYSX_ERRORS_DEFAULT",
    "SETTING_MIN_FRAME_RATE",
    "SETTING_MIN_FRAME_RATE_DEFAULT",
    "SETTING_MOUSE_GRAB",
    "SETTING_MOUSE_GRAB_DEFAULT",
    "SETTING_MOUSE_GRAB_IGNORE_INVISBLE",
    "SETTING_MOUSE_GRAB_IGNORE_INVISBLE_DEFAULT",
    "SETTING_MOUSE_GRAB_WITH_FORCE",
    "SETTING_MOUSE_GRAB_WITH_FORCE_DEFAULT",
    "SETTING_MOUSE_INTERACTION_ENABLED",
    "SETTING_MOUSE_INTERACTION_ENABLED_DEFAULT",
    "SETTING_MOUSE_PICKING_FORCE",
    "SETTING_MOUSE_PICKING_FORCE_DEFAULT",
    "SETTING_MOUSE_PUSH",
    "SETTING_MOUSE_PUSH_DEFAULT",
    "SETTING_NUM_EVENT_PUMPS_FOR_TEST_STAGE_SETUP",
    "SETTING_NUM_EVENT_PUMPS_FOR_TEST_STAGE_SETUP_DEFAULT",
    "SETTING_NUM_THREADS",
    "SETTING_NUM_THREADS_DEFAULT",
    "SETTING_OMNIPVD_ENABLED",
    "SETTING_OMNIPVD_ENABLED_DEFAULT",
    "SETTING_OMNIPVD_IS_OVD_STAGE",
    "SETTING_OMNIPVD_IS_OVD_STAGE_DEFAULT",
    "SETTING_OMNIPVD_IS_RECORDING",
    "SETTING_OMNIPVD_IS_RECORDING_DEFAULT",
    "SETTING_OMNIPVD_OVD_RECORDING_DIRECTORY",
    "SETTING_OMNIPVD_OVD_RECORDING_DIRECTORY_DEFAULT",
    "SETTING_OMNI_HUB_SUPPORTED",
    "SETTING_OMNI_HUB_SUPPORTED_DEFAULT",
    "SETTING_OUTPUT_VELOCITIES_LOCAL_SPACE",
    "SETTING_OUTPUT_VELOCITIES_LOCAL_SPACE_DEFAULT",
    "SETTING_OVERRIDE_GPU",
    "SETTING_OVERRIDE_GPU_DEFAULT",
    "SETTING_PHYSICS_DEVELOPMENT_MODE",
    "SETTING_PHYSICS_DEVELOPMENT_MODE_DEFAULT",
    "SETTING_PHYSICS_SCENE_MULTIGPU_MODE",
    "SETTING_PHYSICS_SCENE_MULTIGPU_MODE_DEFAULT",
    "SETTING_PHYSX_DISPATCHER",
    "SETTING_PHYSX_DISPATCHER_DEFAULT",
    "SETTING_PVD_DEBUG",
    "SETTING_PVD_DEBUG_DEFAULT",
    "SETTING_PVD_ENABLED",
    "SETTING_PVD_ENABLED_DEFAULT",
    "SETTING_PVD_IP_ADDRESS",
    "SETTING_PVD_IP_ADDRESS_DEFAULT",
    "SETTING_PVD_MEMORY",
    "SETTING_PVD_MEMORY_DEFAULT",
    "SETTING_PVD_OUTPUT_DIRECTORY",
    "SETTING_PVD_OUTPUT_DIRECTORY_DEFAULT",
    "SETTING_PVD_PROFILE",
    "SETTING_PVD_PROFILE_DEFAULT",
    "SETTING_PVD_STREAM_TO_FILE",
    "SETTING_PVD_STREAM_TO_FILE_DEFAULT",
    "SETTING_RESET_ON_STOP",
    "SETTING_RESET_ON_STOP_DEFAULT",
    "SETTING_SAVE_COOKED_DATA",
    "SETTING_SAVE_COOKED_DATA_DEFAULT",
    "SETTING_SHOW_COLLISION_GROUPS_WINDOW",
    "SETTING_SHOW_COLLISION_GROUPS_WINDOW_DEFAULT",
    "SETTING_SIMULATE_EMPTY_SCENE",
    "SETTING_SIMULATE_EMPTY_SCENE_DEFAULT",
    "SETTING_SUPPRESS_READBACK",
    "SETTING_SUPPRESS_READBACK_DEFAULT",
    "SETTING_TESTS_ASSETS_PATH",
    "SETTING_TESTS_ASSETS_PATH_DEFAULT",
    "SETTING_TEST_RUNNER_FILTER",
    "SETTING_TEST_RUNNER_FILTER_DEFAULT",
    "SETTING_TEST_RUNNER_REPEATS",
    "SETTING_TEST_RUNNER_REPEATS_DEFAULT",
    "SETTING_TEST_RUNNER_SELECTION",
    "SETTING_TEST_RUNNER_SELECTION_DEFAULT",
    "SETTING_TEST_RUNNER_STATUS",
    "SETTING_TEST_RUNNER_STATUS_DEFAULT",
    "SETTING_UJITSO_COLLISION_COOKING",
    "SETTING_UJITSO_COLLISION_COOKING_DEFAULT",
    "SETTING_UJITSO_COOKING_DEV_KEY",
    "SETTING_UJITSO_COOKING_DEV_KEY_DEFAULT",
    "SETTING_UJITSO_COOKING_MAX_PROCESS_COUNT",
    "SETTING_UJITSO_COOKING_MAX_PROCESS_COUNT_DEFAULT",
    "SETTING_UJITSO_REMOTE_CACHE",
    "SETTING_UJITSO_REMOTE_CACHE_DEFAULT",
    "SETTING_UJITSO_REQUEST_OMNI_HUB",
    "SETTING_UJITSO_REQUEST_OMNI_HUB_DEFAULT",
    "SETTING_UJITSO_USE_OMNI_HUB",
    "SETTING_UJITSO_USE_OMNI_HUB_DEFAULT",
    "SETTING_UPDATE_PARTICLES_TO_USD",
    "SETTING_UPDATE_PARTICLES_TO_USD_DEFAULT",
    "SETTING_UPDATE_RESIDUALS_TO_USD",
    "SETTING_UPDATE_RESIDUALS_TO_USD_DEFAULT",
    "SETTING_UPDATE_TO_USD",
    "SETTING_UPDATE_TO_USD_DEFAULT",
    "SETTING_UPDATE_VELOCITIES_TO_USD",
    "SETTING_UPDATE_VELOCITIES_TO_USD_DEFAULT",
    "SETTING_USE_ACTIVE_CUDA_CONTEXT",
    "SETTING_USE_ACTIVE_CUDA_CONTEXT_DEFAULT",
    "SETTING_USE_LOCAL_MESH_CACHE",
    "SETTING_USE_LOCAL_MESH_CACHE_DEFAULT",
    "SETTING_VISUALIZATION_COLLISION_MESH",
    "SETTING_VISUALIZATION_COLLISION_MESH_DEFAULT",
    "SETTING_VISUALIZATION_GAP",
    "SETTING_VISUALIZATION_GAP_DEFAULT",
    "SETTING_ZERO_GRAVITY_SPEED",
    "SETTING_ZERO_GRAVITY_SPEED_DEFAULT",
    "SceneQueryHitLocation",
    "SceneQueryHitObject",
    "SimulationEvent",
    "SweepHit",
    "TriggerEventData",
    "TriggerEventType",
    "VEHICLE_AUTOMATIC_TRANSMISSION_GEAR_VALUE",
    "VEHICLE_DRIVE_STATE_ACCELERATOR",
    "VEHICLE_DRIVE_STATE_AUTOBOX_TIME_SINCE_LAST_SHIFT",
    "VEHICLE_DRIVE_STATE_AUTOMATIC_TRANSMISSION",
    "VEHICLE_DRIVE_STATE_BRAKE0",
    "VEHICLE_DRIVE_STATE_BRAKE1",
    "VEHICLE_DRIVE_STATE_CLUTCH",
    "VEHICLE_DRIVE_STATE_CURRENT_GEAR",
    "VEHICLE_DRIVE_STATE_ENGINE_ROTATION_SPEED",
    "VEHICLE_DRIVE_STATE_GEAR_SWITCH_TIME",
    "VEHICLE_DRIVE_STATE_STEER",
    "VEHICLE_DRIVE_STATE_TARGET_GEAR",
    "VEHICLE_WHEEL_STATE_GROUND_ACTOR",
    "VEHICLE_WHEEL_STATE_GROUND_HIT_POSITION",
    "VEHICLE_WHEEL_STATE_GROUND_MATERIAL",
    "VEHICLE_WHEEL_STATE_GROUND_PLANE",
    "VEHICLE_WHEEL_STATE_GROUND_SHAPE",
    "VEHICLE_WHEEL_STATE_IS_ON_GROUND",
    "VEHICLE_WHEEL_STATE_LOCAL_POSE_POSITION",
    "VEHICLE_WHEEL_STATE_LOCAL_POSE_QUATERNION",
    "VEHICLE_WHEEL_STATE_ROTATION_ANGLE",
    "VEHICLE_WHEEL_STATE_ROTATION_SPEED",
    "VEHICLE_WHEEL_STATE_STEER_ANGLE",
    "VEHICLE_WHEEL_STATE_SUSPENSION_FORCE",
    "VEHICLE_WHEEL_STATE_SUSPENSION_JOUNCE",
    "VEHICLE_WHEEL_STATE_TIRE_FORCE",
    "VEHICLE_WHEEL_STATE_TIRE_FRICTION",
    "VEHICLE_WHEEL_STATE_TIRE_LATERAL_DIRECTION",
    "VEHICLE_WHEEL_STATE_TIRE_LATERAL_SLIP",
    "VEHICLE_WHEEL_STATE_TIRE_LONGITUDINAL_DIRECTION",
    "VEHICLE_WHEEL_STATE_TIRE_LONGITUDINAL_SLIP",
    "VisualizerMode",
    "acquire_physx_attachment_interface",
    "acquire_physx_benchmarks_interface",
    "acquire_physx_cooking_interface",
    "acquire_physx_interface",
    "acquire_physx_property_query_interface",
    "acquire_physx_replicator_interface",
    "acquire_physx_scene_query_interface",
    "acquire_physx_simulation_interface",
    "acquire_physx_stage_update_interface",
    "acquire_physx_statistics_interface",
    "acquire_physx_visualization_interface",
    "acquire_physxunittests_interface",
    "ancestorHasAPI",
    "descendantHasAPI",
    "hasconflictingapis_ArticulationRoot",
    "hasconflictingapis_CollisionAPI",
    "hasconflictingapis_PhysxDeformableBodyAPI",
    "hasconflictingapis_PhysxDeformableSurfaceAPI",
    "hasconflictingapis_PhysxHairAPI",
    "hasconflictingapis_PhysxParticleClothAPI",
    "hasconflictingapis_PhysxParticleSamplingAPI",
    "hasconflictingapis_Precompute",
    "hasconflictingapis_RigidBodyAPI",
    "isOverConflictingApisSubtreeLimit",
    "release_physx_attachment_interface",
    "release_physx_benchmarks_interface",
    "release_physx_cooking_interface",
    "release_physx_interface",
    "release_physx_interface_scripting",
    "release_physx_property_query_interface",
    "release_physx_replicator_interface",
    "release_physx_replicator_interface_scripting",
    "release_physx_scene_query_interface",
    "release_physx_simulation_interface",
    "release_physx_stage_update_interface",
    "release_physx_stage_update_interface_scripting",
    "release_physx_statistics_interface",
    "release_physx_statistics_interface_scripting",
    "release_physx_visualization_interface",
    "release_physxunittests_interface"
]


class ContactData():
    """
    Contact data.
    """
    @property
    def face_index0(self) -> int:
        """
        Face index 0 - non zero only for meshes

        :type: int
        """
    @property
    def face_index1(self) -> int:
        """
        Face index 1 - non zero only for meshes.

        :type: int
        """
    @property
    def impulse(self) -> carb._carb.Float3:
        """
        Contact impulse

        :type: carb._carb.Float3
        """
    @property
    def material0(self) -> int:
        """
        Material0 - uint64 use PhysicsSchemaTools::intToSdfPath to convert to SdfPath.

        :type: int
        """
    @property
    def material1(self) -> int:
        """
        Material1 - uint64 use PhysicsSchemaTools::intToSdfPath to convert to SdfPath.

        :type: int
        """
    @property
    def normal(self) -> carb._carb.Float3:
        """
        Contact normal.

        :type: carb._carb.Float3
        """
    @property
    def position(self) -> carb._carb.Float3:
        """
        Contact position.

        :type: carb._carb.Float3
        """
    @property
    def separation(self) -> float:
        """
        Contact separation value.

        :type: float
        """
    pass
class ContactDataVector():
    def __bool__(self) -> bool: 
        """
        Check whether the list is nonempty
        """
    @typing.overload
    def __delitem__(self, arg0: int) -> None: 
        """
        Delete the list elements at index ``i``

        Delete list elements using a slice object
        """
    @typing.overload
    def __delitem__(self, arg0: slice) -> None: ...
    @typing.overload
    def __getitem__(self, s: slice) -> ContactDataVector: 
        """
        Retrieve list elements using a slice object
        """
    @typing.overload
    def __getitem__(self, arg0: int) -> ContactData: ...
    @typing.overload
    def __init__(self) -> None: 
        """
        Copy constructor
        """
    @typing.overload
    def __init__(self, arg0: ContactDataVector) -> None: ...
    @typing.overload
    def __init__(self, arg0: typing.Iterable) -> None: ...
    def __iter__(self) -> typing.Iterator: ...
    def __len__(self) -> int: ...
    @typing.overload
    def __setitem__(self, arg0: int, arg1: ContactData) -> None: 
        """
        Assign list elements using a slice object
        """
    @typing.overload
    def __setitem__(self, arg0: slice, arg1: ContactDataVector) -> None: ...
    def append(self, x: ContactData) -> None: 
        """
        Add an item to the end of the list
        """
    def clear(self) -> None: 
        """
        Clear the contents
        """
    @typing.overload
    def extend(self, L: ContactDataVector) -> None: 
        """
        Extend the list by appending all the items in the given list

        Extend the list by appending all the items in the given list
        """
    @typing.overload
    def extend(self, L: typing.Iterable) -> None: ...
    def insert(self, i: int, x: ContactData) -> None: 
        """
        Insert an item at a given position.
        """
    @typing.overload
    def pop(self) -> ContactData: 
        """
        Remove and return the last item

        Remove and return the item at index ``i``
        """
    @typing.overload
    def pop(self, i: int) -> ContactData: ...
    pass
class ContactEventHeader():
    """
    Contact event header.
    """
    @property
    def actor0(self) -> int:
        """
        Actor0 - uint64 use PhysicsSchemaTools::intToSdfPath to convert to SdfPath.

        :type: int
        """
    @property
    def actor1(self) -> int:
        """
        Actor1 - uint64 use PhysicsSchemaTools::intToSdfPath to convert to SdfPath.

        :type: int
        """
    @property
    def collider0(self) -> int:
        """
        Collider0 - uint64 use PhysicsSchemaTools::intToSdfPath to convert to SdfPath.

        :type: int
        """
    @property
    def collider1(self) -> int:
        """
        Collider1 - uint64 use PhysicsSchemaTools::intToSdfPath to convert to SdfPath.

        :type: int
        """
    @property
    def contact_data_offset(self) -> int:
        """
        Contact data offset.

        :type: int
        """
    @property
    def friction_anchors_offset(self) -> int:
        """
        Friction anchors data offset.

        :type: int
        """
    @property
    def num_contact_data(self) -> int:
        """
        Number of contact data.

        :type: int
        """
    @property
    def num_friction_anchors_data(self) -> int:
        """
        Number of contact data.

        :type: int
        """
    @property
    def proto_index0(self) -> int:
        """
        Protoindex0 from a point instancer (0xFFFFFFFF means collider is not part of an instancer).

        :type: int
        """
    @property
    def proto_index1(self) -> int:
        """
        Protoindex1 from a point instancer (0xFFFFFFFF means collider is not part of an instancer).

        :type: int
        """
    @property
    def stage_id(self) -> int:
        """
        Stage id.

        :type: int
        """
    @property
    def type(self) -> ContactEventType:
        """
        Contact event type.

        :type: ContactEventType
        """
    pass
class ContactEventHeaderVector():
    def __bool__(self) -> bool: 
        """
        Check whether the list is nonempty
        """
    @typing.overload
    def __delitem__(self, arg0: int) -> None: 
        """
        Delete the list elements at index ``i``

        Delete list elements using a slice object
        """
    @typing.overload
    def __delitem__(self, arg0: slice) -> None: ...
    @typing.overload
    def __getitem__(self, s: slice) -> ContactEventHeaderVector: 
        """
        Retrieve list elements using a slice object
        """
    @typing.overload
    def __getitem__(self, arg0: int) -> ContactEventHeader: ...
    @typing.overload
    def __init__(self) -> None: 
        """
        Copy constructor
        """
    @typing.overload
    def __init__(self, arg0: ContactEventHeaderVector) -> None: ...
    @typing.overload
    def __init__(self, arg0: typing.Iterable) -> None: ...
    def __iter__(self) -> typing.Iterator: ...
    def __len__(self) -> int: ...
    @typing.overload
    def __setitem__(self, arg0: int, arg1: ContactEventHeader) -> None: 
        """
        Assign list elements using a slice object
        """
    @typing.overload
    def __setitem__(self, arg0: slice, arg1: ContactEventHeaderVector) -> None: ...
    def append(self, x: ContactEventHeader) -> None: 
        """
        Add an item to the end of the list
        """
    def clear(self) -> None: 
        """
        Clear the contents
        """
    @typing.overload
    def extend(self, L: ContactEventHeaderVector) -> None: 
        """
        Extend the list by appending all the items in the given list

        Extend the list by appending all the items in the given list
        """
    @typing.overload
    def extend(self, L: typing.Iterable) -> None: ...
    def insert(self, i: int, x: ContactEventHeader) -> None: 
        """
        Insert an item at a given position.
        """
    @typing.overload
    def pop(self) -> ContactEventHeader: 
        """
        Remove and return the last item

        Remove and return the item at index ``i``
        """
    @typing.overload
    def pop(self, i: int) -> ContactEventHeader: ...
    pass
class ContactEventType():
    """
            Contact event type.
            

    Members:

      CONTACT_FOUND : Contact found.

      CONTACT_LOST : Contact lost.

      CONTACT_PERSIST : Contact persist.
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
    CONTACT_FOUND: omni.physx.bindings._physx.ContactEventType # value = <ContactEventType.CONTACT_FOUND: 0>
    CONTACT_LOST: omni.physx.bindings._physx.ContactEventType # value = <ContactEventType.CONTACT_LOST: 1>
    CONTACT_PERSIST: omni.physx.bindings._physx.ContactEventType # value = <ContactEventType.CONTACT_PERSIST: 2>
    __members__: dict # value = {'CONTACT_FOUND': <ContactEventType.CONTACT_FOUND: 0>, 'CONTACT_LOST': <ContactEventType.CONTACT_LOST: 1>, 'CONTACT_PERSIST': <ContactEventType.CONTACT_PERSIST: 2>}
    pass
class ErrorEvent():
    """
            Error events used by physics error event stream.
            

    Members:

      USD_LOAD_ERROR : Usd load error, event has dictionary with key 'errorString':string

      PHYSX_ERROR : PhysX runtime error, event has dictionary with key 'errorString':string

      PHYSX_TOO_MANY_ERRORS : PhysX exceeded maximum number of reported errors, event has dictionary with key 'errorString':string

      PHYSX_CUDA_ERROR : PhysX GPU Cuda error, event has dictionary with key 'errorString':string
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
    PHYSX_CUDA_ERROR: omni.physx.bindings._physx.ErrorEvent # value = <ErrorEvent.PHYSX_CUDA_ERROR: 2>
    PHYSX_ERROR: omni.physx.bindings._physx.ErrorEvent # value = <ErrorEvent.PHYSX_ERROR: 1>
    PHYSX_TOO_MANY_ERRORS: omni.physx.bindings._physx.ErrorEvent # value = <ErrorEvent.PHYSX_TOO_MANY_ERRORS: 3>
    USD_LOAD_ERROR: omni.physx.bindings._physx.ErrorEvent # value = <ErrorEvent.USD_LOAD_ERROR: 0>
    __members__: dict # value = {'USD_LOAD_ERROR': <ErrorEvent.USD_LOAD_ERROR: 0>, 'PHYSX_ERROR': <ErrorEvent.PHYSX_ERROR: 1>, 'PHYSX_TOO_MANY_ERRORS': <ErrorEvent.PHYSX_TOO_MANY_ERRORS: 3>, 'PHYSX_CUDA_ERROR': <ErrorEvent.PHYSX_CUDA_ERROR: 2>}
    pass
class FrictionAnchor():
    """
    Contact data.
    """
    @property
    def impulse(self) -> carb._carb.Float3:
        """
        Contact impulse

        :type: carb._carb.Float3
        """
    @property
    def position(self) -> carb._carb.Float3:
        """
        Contact position.

        :type: carb._carb.Float3
        """
    pass
class FrictionAnchorsDataVector():
    def __bool__(self) -> bool: 
        """
        Check whether the list is nonempty
        """
    @typing.overload
    def __delitem__(self, arg0: int) -> None: 
        """
        Delete the list elements at index ``i``

        Delete list elements using a slice object
        """
    @typing.overload
    def __delitem__(self, arg0: slice) -> None: ...
    @typing.overload
    def __getitem__(self, s: slice) -> FrictionAnchorsDataVector: 
        """
        Retrieve list elements using a slice object
        """
    @typing.overload
    def __getitem__(self, arg0: int) -> FrictionAnchor: ...
    @typing.overload
    def __init__(self) -> None: 
        """
        Copy constructor
        """
    @typing.overload
    def __init__(self, arg0: FrictionAnchorsDataVector) -> None: ...
    @typing.overload
    def __init__(self, arg0: typing.Iterable) -> None: ...
    def __iter__(self) -> typing.Iterator: ...
    def __len__(self) -> int: ...
    @typing.overload
    def __setitem__(self, arg0: int, arg1: FrictionAnchor) -> None: 
        """
        Assign list elements using a slice object
        """
    @typing.overload
    def __setitem__(self, arg0: slice, arg1: FrictionAnchorsDataVector) -> None: ...
    def append(self, x: FrictionAnchor) -> None: 
        """
        Add an item to the end of the list
        """
    def clear(self) -> None: 
        """
        Clear the contents
        """
    @typing.overload
    def extend(self, L: FrictionAnchorsDataVector) -> None: 
        """
        Extend the list by appending all the items in the given list

        Extend the list by appending all the items in the given list
        """
    @typing.overload
    def extend(self, L: typing.Iterable) -> None: ...
    def insert(self, i: int, x: FrictionAnchor) -> None: 
        """
        Insert an item at a given position.
        """
    @typing.overload
    def pop(self) -> FrictionAnchor: 
        """
        Remove and return the last item

        Remove and return the item at index ``i``
        """
    @typing.overload
    def pop(self, i: int) -> FrictionAnchor: ...
    pass
class IPhysxAttachment():
    """
    This interface is the access point to the omni.physx attachment functionality.
    """
    def add_surface_sampler_points(self, surface_sampler: int, points: typing.List[carb._carb.Float3]) -> None: 
        """
        Add samples to surface sampler.

        Args:
            surface_sampler:    Handle to surface sampler
            points:             Points to add

        This function adds samples to the surface sampler that are then considered for further sampling.
        """
    def compute_attachment_points(self, attachment_path: str) -> bool: 
        """
        Compute attachment points

        Args:
            attachment_path: path to PhysxSchema.PhysxPhysicsAttachment primitive with PhysxSchema.PhysxAutoAttachmentAPI
        """
    def create_point_finder(self, points: typing.List[carb._carb.Float3]) -> int: 
        """
        Create point finder.

        Args:
            points:         Points of the mesh

        Returns:
            Handle to point finder
        """
    def create_surface_sampler(self, collider_path: str, sampling_distance: float) -> int: 
        """
        Create surface Poisson sampler.

        Args:
            collider_path:       Path to prim with UsdPhysics.CollisionAPI
            sampling_distance:   Distance used for sampling points

        Returns:
            Handle to surface sampler
        """
    def create_tet_finder(self, points: typing.List[carb._carb.Float3], indices: typing.List[int]) -> int: 
        """
        Create tet finder.

        Args:
            points:          Points of the tetrahedral mesh
            indices:         Indices of the tetrahedral mesh

        Returns:
            Handle to tet finder
        """
    def create_tri_mesh_sampler(self, surface_sampler: int) -> None: 
        """
        Get closest points to the input points on the prim

        Args:
            points: input points
            path:  prim path

        Returns:
            Return a dictionary with closest information::

                'closest_points': float3 - Closest points to the input points on the prim. Only valid when returned distance is strictly positive.
                'dists': float - Square distances between the points and the geom object. 0.0 if the point is inside the object.
        """
    def get_closest_points(self, point: typing.List[carb._carb.Float3], path: str) -> dict: 
        """
        Get closest points to the input points on the prim

        Args:
            points: input points
            path:  prim path

        Returns:
            Return a dictionary with closest information::

                'closest_points': float3 - Closest points to the input points on the prim. Only valid when returned distance is strictly positive.
                'dists': float - Square distances between the points and the geom object. 0.0 if the point is inside the object.
        """
    def get_surface_sampler_points(self, surface_sampler: int) -> dict: 
        """
        Get the samples of surface sampler.

        Args:
            surface_sampler:    Handle to surface sampler

        Returns:
            Dict mapping 'points' to the resulting sampling points
        """
    def is_point_inside(self, surface_sampler: int, point: carb._carb.Float3) -> bool: 
        """
        Get closest points to the input points on the prim

        Args:
            points: input points
            path:  prim path

        Returns:
            Return a dictionary with closest information::

                'closest_points': float3 - Closest points to the input points on the prim. Only valid when returned distance is strictly positive.
                'dists': float - Square distances between the points and the geom object. 0.0 if the point is inside the object.
        """
    def overlap_tetmesh_capsule(self, tet_finder: int, pos: carb._carb.Float3, axis: carb._carb.Float3, radius: float, half_height: float) -> dict: 
        """
        Finds all tetrahedra overlapping with a capsule.

        Args:
            tet_finder:         Handle to tet finder
            pos:                Position of capsule
            axis:               Orientation of the capsule
            radius:             Radius of the capsule
            half_height         Half height of the capsule

        Returns:
            Dict mapping 'tet_ids' to the resulting indices of tetrahedra overlapping with the capsule.
        """
    def overlap_tetmesh_sphere(self, tet_finder: int, center: carb._carb.Float3, radius: float) -> dict: 
        """
        Finds all tetrahedra overlapping with a sphere.

        Args:
            tet_finder:         Handle to tet finder
            center:             Center of sphere
            radius:             Radius of sphere

        Returns:
            Dict mapping 'tet_ids' to the resulting indices of tetrahedra overlapping with the sphere.
        """
    def points_to_indices(self, point_finder: int, points: typing.List[carb._carb.Float3]) -> dict: 
        """
        Map points to indices.

        Args:
            point_finder:   Handle to point finder
            points:         Points to be mapped

        Returns:
            Dict mapping 'indices' to the resulting mesh coordinates. Indices might be -1 for points outside of the mesh.
        """
    def points_to_tetmesh_local(self, tet_finder: int, points: typing.List[carb._carb.Float3]) -> dict: 
        """
        Map points in euclidean coordiantes to tet mesh coordinates.

        Args:
            tet_finder:    Handle to tet finder
            points:        Points to be mapped

        Returns:
            Dict mapping 'tet_ids' and 'barycentric_coords' to the resulting local tet mesh coordinates. Tet indices might be -1 for points outside of the tetrahedral mesh.
        """
    def release_point_finder(self, point_finder: int) -> None: 
        """
        Release point finder.

        Args:
            point_finder:   Handle to point finder
        """
    def release_surface_sampler(self, surface_sampler: int) -> None: 
        """
        Release surface Poisson sampler.

        Args:
            surface_sampler:     Handle to surface sampler
        """
    def release_tet_finder(self, tet_finder: int) -> None: 
        """
        Release tet finder.

        Args:
            tet_finder:     Handle to tet finder
        """
    def remove_surface_sampler_points(self, surface_sampler: int, points: typing.List[carb._carb.Float3]) -> None: 
        """
        Remove samples from surface sampler.

        Args:
            surface_sampler:    Handle to surface sampler
            points:             Points to remove

        This function removes samples from the surface sampler.
        """
    def sample_surface(self, surface_sampler: int, sphere_center: carb._carb.Float3, sphere_radius: float, sampling_distance: float) -> dict: 
        """
        Create new surface samples within specified sphere.

        Args:
            surface_sampler:    Handle to surface sampler
            sphere_center:      Center of sphere used for restricting sampling domain
            sphere_radius:      Radius of sphere used for restricting sampling domain

        Returns:
            Dict mapping 'points' to the resulting sampling points

        New samples are created on the surface for which the samples has been created, and within the sphere specified.
        """
    def tetmesh_local_to_points(self, tet_finder: int, tet_ids: typing.List[int], barycentric_coords: typing.List[carb._carb.Float4]) -> dict: 
        """
        Map tet mesh coordinates to points in euclidean coordinates.

        Args:
            tet_finder:         Handle to tet finder
            tet_ids:            Tetrahedra indices
            barycentric_coords: Barycentric coordinates

        Returns:
            Dict mapping 'points' to the resulting points in euclidean coordinates.
        """
    pass
class IPhysxBenchmarks():
    """
    This interface is the access point to the omni.physx benchmarks.
    """
    def enable_profile(self, enable_profile: bool) -> None: 
        """
        Enabled profiling
        Args:
            enable_profile: Do custom physics profiling
        """
    def get_profile_stats(self) -> dict: 
        """
        Get profiling stats
        """
    pass
class IPhysxPropertyQuery():
    def query_prim(self, stage_id: int, prim_id: int, query_mode: PhysxPropertyQueryMode = PhysxPropertyQueryMode.QUERY_RIGID_BODY_WITH_COLLIDERS, timeout_ms: int = -1, finished_fn: typing.Callable[[], None] = None, rigid_body_fn: typing.Callable[[PhysxPropertyQueryRigidBodyResponse], None] = None, collider_fn: typing.Callable[[PhysxPropertyQueryColliderResponse], None] = None) -> None: 
        """
        Returns information for given prim

        Args:
            stage_id:       uint64_t                Stage id
            prim_id:        uint64_t                USD path (use PhysicsSchemaTools::sdfPathToInt)
            query_mode:     PhysxPropertyQueryMode  Type of query to be made
            timeout_ms:     int64                   Timeout (in milliseconds) for the request. (-1 means wait forever)
            finished_fn:  function                  Report function called when enumeration of all objects is finished
            rigid_body_fn:  function                Report function where rigid body information will be returned in PhysxPropertyQueryRigidBodyResponse object
            collider_fn:    function                Report function where collider information will be returned in PhysxPropertyQueryRigidBodyResponse object
        """
    pass
class IPhysxReplicator():
    def register_replicator(self, stage_id: int, replicator_attach_fn: typing.Callable[[int], list], replicator_attach_end_fn: typing.Callable[[int], None], hierarchy_rename_fn: typing.Callable[[str, int], str]) -> None: 
        """
        Register replicator for given stage.

        stage_id: current stageId

        replicator_attach_fn: std::function<py::list(uint64_t stageId)>;

        replicator_attach_end_fn: std::function<py::list(uint64_t stageId)>;

        hierarchy_rename_fn: std::function<const char* (const char* replicatePath, uint32_t index)>;
        """
    def replicate(self, stage_id: int, path: str, numReplications: int) -> bool: 
        """
        Replicate hierarchy.

        stage_id: stage id
        path: path to replicate
        numReplications: number of replications
        """
    def unregister_replicator(self, stage_id: int) -> None: 
        """
        Unregister replicator from stage.
        """
    pass
class IPhysxSimulation():
    """
    This interface is the access point to the omni.physx simulation control.
    """
    def apply_force_at_pos(self, stage_id: int, body_path: int, force: carb._carb.Float3, pos: carb._carb.Float3, mode: str = 'Force') -> None: 
        """
        Applies force at given body with given force position.

        Args:
            stage_id: USD stageId
            body_path: USD path of the body encoded in uint64_t use PhysicsSchemaTools::sdfPathToInt
            force: Force to apply to the body.
            pos: World position where the force is applied.
            mode: Supporting various modes - Force (default), Impulse, Velocity, Acceleration.
        """
    def apply_force_at_pos_instanced(self, stage_id: int, point_instancer_path: int, force: carb._carb.Float3, pos: carb._carb.Float3, mode: str = 'Force', proto_index: int = 4294967295) -> None: 
        """
        Applies force at given point instancer body with given force position.

        Args:
            stage_id: USD stageId
            point_instancer_path: USD path to a point instancer encoded in uint64_t use PhysicsSchemaTools::sdfPathToInt
            force: Force to apply to the target.
            pos: World position where the force is applied.
            mode: Supporting various modes - Force (default), Impulse, Velocity, Acceleration.
            protoIndex: If protoIndex is 0xffffffff (default), force will be applied to all instances,
                        otherwise it will only be applied to the instance at this index.
        """
    def apply_torque(self, stage_id: int, body_path: int, torque: carb._carb.Float3) -> None: 
        """
        Applies torque at given body at body center of mass.

        Args:
            stage_id: USD stageId
            body_path: USD path of the body encoded in uint64_t use PhysicsSchemaTools::sdfPathToInt
            torque: Torque to apply to the body.
        """
    def apply_torque_instanced(self, stage_id: int, body_path: int, torque: carb._carb.Float3, proto_index: int = 4294967295) -> None: 
        """
        Applies torque at given point instancer body at body center of mass.

        Args:
            stage_id: USD stageId
            point_instancer_path: USD path to a point instancer encoded in uint64_t use PhysicsSchemaTools::sdfPathToInt
            torque: Torque to apply to the body.
            protoIndex: If protoIndex is 0xffffffff (default), torque will be applied to all instances,
                        otherwise it will only be applied to the instance at this index.
        """
    def attach_stage(self, stage_id: int) -> bool: 
        """
        Attach USD stage. This will run the physics parser
        and will populate the PhysX SDK with the corresponding simulation objects.

        Note: previous stage will be detached.

        Args:
            stage_id: USD stageId (can be retrieved from a stagePtr - UsdUtils.StageCache.Get().GetId(stage).ToLongInt())

        Returns:
            Returns true if stage was successfully attached.
        """
    def check_results(self) -> bool: 
        """
        Check if simulation finished.

        Returns:
            Returns true if simulation has finished.
        """
    def check_results_scene(self, scene_path: int) -> bool: 
        """
        Check if a simulation scene is finished. Disabling a scene has no effect on this function.
        If scenePath is empty, it behaves like IPhysxSimulation::checkResults

        Returns True if the simulation scene is finished.

        scenePath       uint64_t        Scene USD path use PhysicsSchemaTools::sdfPathToInt
        """
    def detach_stage(self) -> None: 
        """
        Detach USD stage, this will remove all objects from the PhysX SDK
        """
    def fetch_results(self) -> None: 
        """
        Fetch simulation results.
        Writing out simulation results based on physics settings.

        Note This is a blocking call. The function will wait until the simulation is finished.
        """
    def fetch_results_scene(self, scene_path: int) -> None: 
        """
        Fetch simulation scene results and writes out simulation results based on physics settings for
        a specific scene. Disabling a scene has no effect on this function.
        If scenePath is empty, it behaves like IPhysxSimulation::fetchResults

        Note: this is a blocking call. The function will wait until the simulation scene is finished.

        scenePath       uint64_t        Scene USD path use PhysicsSchemaTools::sdfPathToInt
        """
    def flush_changes(self) -> None: 
        """
        Flush changes will force physics to process buffered changes

        Changes to physics gets buffered, in some cases flushing changes is required if order is required.

        Example - prim A gets added. Existing prim B has a relationship that gets switched to use A. Currently,
        the relationship change gets processed immediately and fails because prim A only gets added at the
        start of the next sim step.
        """
    def get_attached_stage(self) -> int: 
        """
        Returns the currently attached stage through the IPhysxSimulation interface

        Returns:
            Returns USD stage id.
        """
    def get_contact_report(self) -> tuple: 
        """
        Get contact report data for current simulation step directly.

        The contact buffer data are available for one simulation step

        Returns:
            Tuple with contact event vector and contact data vector::

            'contact_headers': vector of contact event headers
            'contact_data': vector of contact data
            'friction_anchors': vector of friction anchors data
        """
    def get_full_contact_report(self) -> tuple: 
        """
        Get contact report data for current simulation step directly, including friction anchors.

        The contact buffer data are available for one simulation step

        Returns:
            Tuple with contact event vector and contact data vector::

            'contact_headers': vector of contact event headers
            'contact_data': vector of contact data
            'friction_anchors': vector of friction anchors data
        """
    def is_change_tracking_paused(self) -> bool: 
        """
        Check if fabric change tracking for physics listener is paused or not

        Returns true if paused change tracking.
        """
    def is_sleeping(self, stage_id: int, body_path: int) -> bool: 
        """
        Is body sleeping.

        Args:
            stage_id: USD stageId
            body_path: USD path of the body encoded in uint64_t use PhysicsSchemaTools::sdfPathToInt

        Returns:
            True if body is asleeep
        """
    def is_sleeping_instanced(self, stage_id: int, body_path: int, proto_index: int) -> bool: 
        """
        Is point instancer bodypoint instancer body sleeping.

        Args:
            stage_id: USD stageId
            point_instancer_path: USD path to a point instancer encoded in uint64_t use PhysicsSchemaTools::sdfPathToInt
            protoIndex: Check against the instance at this index.

        Returns:
            True if body is asleeep
        """
    def pause_change_tracking(self, pause: bool) -> None: 
        """
        Pause fabric change tracking for physics listener.

        Args:
            pause bool Pause or unpause change tracking.
        """
    def put_to_sleep(self, stage_id: int, body_path: int) -> None: 
        """
        Put body to sleep.

        Args:
            stage_id: USD stageId
            body_path: USD path of the body encoded in uint64_t use PhysicsSchemaTools::sdfPathToInt
        """
    def put_to_sleep_instanced(self, stage_id: int, body_path: int, proto_index: int = 4294967295) -> None: 
        """
        Put point instancer body to sleep.

        Args:
            stage_id: USD stageId
            point_instancer_path: USD path to a point instancer encoded in uint64_t use PhysicsSchemaTools::sdfPathToInt
            protoIndex: If protoIndex is 0xffffffff (default), all instances will be put to sleep,
                        otherwise it will only be applied to the instance at this index.
        """
    def simulate(self, elapsed_time: float, current_time: float) -> None: 
        """
        Execute physics asynchronous simulation.

        Args:
            elapsed_time: Simulation time (seconds).
            current_time: Current time (seconds), might be used for time sampled transformations to apply.
        """
    def simulate_scene(self, scene_path: int, elapsed_time: float, current_time: float) -> None: 
        """
        Execute the physics simulation on a specific scene.

        The PhysX simulation in the scene will simulate the exact elapsedTime passed. No substepping will happen.
        It is the caller's responsibility to provide a reasonable elapsedTime.
        In general it is recommended to use fixed size time steps with a maximum of 1/60 of a second.
        If scenePath is empty, it behaves like IPhysxSimulation::simulate

        scenePath       uint64_t        Scene USD path use PhysicsSchemaTools::sdfPathToInt
        elapsedTime     float           Simulation time (seconds).
        currentTime     float           Current time (seconds), might be used for time sampled transformations to apply.
        """
    def subscribe_contact_report_events(self, fn: typing.Callable[[ContactEventHeaderVector, ContactDataVector], None]) -> carb._carb.Subscription: 
        """
        Subscribes to contact report callback function.

        The contact buffer data are available for one simulation step

        Args:
            fn: The callback to be called after simulation step to receive contact reports.

        Returns:
            The subscription holder.
        """
    def subscribe_full_contact_report_events(self, fn: typing.Callable[[ContactEventHeaderVector, ContactDataVector, FrictionAnchorsDataVector], None]) -> carb._carb.Subscription: 
        """
        Subscribes to contact report callback function including friction anchors.

        The contact buffer data are available for one simulation step

        Args:
            fn: The callback to be called after simulation step to receive contact reports.

        Returns:
            The subscription holder.
        """
    def subscribe_physics_trigger_report_events(self, trigger_report_fn: typing.Callable[[TriggerEventData], None], stage_id: int = 0, prim_id: int = 0) -> int: 
        """
        Register for trigger notifications

        Args:
            trigger_report_fn:  function            Report function where enter or leave trigger events will be notified
            stage_id:           uint64_t            [Optional] Stage id to filter triggers from
            prim_id:            uint64_t            [Optional] USD path to filter triggers from (use PhysicsSchemaTools::sdfPathToInt)

        Returns:
            subscription_id     : SubscriptionId    Subscription Id that can be unregistered with unsubscribe_physics_trigger_report_events
        """
    def unsubscribe_physics_trigger_report_events(self, subscriptionId: int) -> None: 
        """
        Unregister a subscription for trigger notifications

        Args:
            subscription_id:           SubscriptionId            Subscription Id
        """
    def wake_up(self, stage_id: int, body_path: int) -> None: 
        """
        Wake up body.

        Args:
            stage_id: USD stageId
            body_path: USD path of the body encoded in uint64_t use PhysicsSchemaTools::sdfPathToInt
        """
    def wake_up_instanced(self, stage_id: int, body_path: int, proto_index: int = 4294967295) -> None: 
        """
        Wake up point instancer body.

        Args:
            stage_id: USD stageId
            point_instancer_path: USD path to a point instancer encoded in uint64_t use PhysicsSchemaTools::sdfPathToInt
            protoIndex: If protoIndex is 0xffffffff (default), all instances will be awakened,
                        otherwise it will only be applied to the instance at this index.
        """
    pass
class IPhysxStageUpdate():
    def on_attach(self, arg0: int) -> None: 
        """
        Called when a stage gets attached, does not load physics. Does just set internally stage.

        stage_id: current stageId
        """
    def on_detach(self) -> None: 
        """
        Called when stage gets detached.
        """
    def on_fabric_attach(self, arg0: int) -> None: 
        """
        Called again when a stage gets attached, but at a later point, where fabric stage is already created.

        stage_id: current stageId
        """
    def on_pause(self) -> None: 
        """
        Called when timeline gets paused.
        """
    def on_reset(self) -> None: 
        """
        Called when timeline is stopped.
        """
    def on_resume(self, arg0: float) -> None: 
        """
        Called when timeline play is requested.

        current_time: Current time in seconds
        """
    def on_update(self, arg0: float, arg1: float, arg2: bool) -> None: 
        """
        Called when on stage update.

        current_time: Current time in seconds
        elapsed_sec: Elapsed time from previous update in seconds
        enable_update: Enable physics update, physics can be disabled, but we still need to update other subsystems
        """
    pass
class IPhysxStatistics():
    def get_physx_scene_statistics(self, arg0: int, arg1: int, arg2: PhysicsSceneStats) -> bool: 
        """
        Get physics scene PhysX simulation statistics.

        stage_id: current stageId
        path: physics scene path encoded into two integers. Use PhysicsSchemaTools.encodeSdfPath to get those.
        """
    pass
class OverlapHit(SceneQueryHitObject):
    """
    Sweep hit results structure.
    """
    pass
class ParticleVisualizationPositionType():
    """
    Particle position debug visualiztion option for smoothed fluid particles for use with
    SETTING_DISPLAY_PARTICLES_POSITION_TYPE.
    """
    SIM_POSITIONS = 0
    SMOOTHED_POSITIONS = 1
    pass
class ParticleVisualizationRadiusType():
    """
    Particle radius debug visualization option for use with SETTING_DISPLAY_PARTICLES_RADIUS_TYPE.
    """
    ANISOTROPY = 4
    CONTACT_OFFSET = 0
    PARTICLE_CONTACT_OFFSET = 2
    PARTICLE_REST_OFFSET = 3
    RENDER_GEOMETRY = 5
    REST_OFFSET = 1
    pass
class PhysX():
    """
    This interface is the main access point to omni.physx extension.
    It contains functions that can control the simulation, modify the simulation
    or work directly with physics objects.
    """
    def apply_force_at_pos(self, body_path: str, force: carb._carb.Float3, pos: carb._carb.Float3, mode: str = 'Force') -> None: 
        """
        Applies force at given body with given force position.

        Deprecated, please use apply_force_at_pos at physx_simulation_interface

        Args:
            body_path: USD path of the body.
            force: Force to apply to the body.
            pos: World position where the force is applied.
            mode: Supporting various modes - Force (default), Impulse, Velocity, Acceleration.
        """
    def cancel_collision_tasks(self) -> int: 
        """
        Returns the number of active collision cooking tasks which were canceled
        """
    def check_backwards_compatibility(self, stage_id: int = -1) -> bool: 
        """
        Checks for occurrences of any old schema format.
        """
    def compute_vehicle_velocity(self, path: str, direction: carb._carb.Float3) -> float: 
        """
        Get the linear velocity of a vehicle (vehicle prim needs to have vehicle API applied).

        Note:
            Should only be called after the simulation has been started.

        Args:
            path: Vehicle USD path.
            direction: carb.Float3: Unit length direction vector along which the linear
                       velocity should get computed. The vector is considered to be
                       relative to the center of mass frame of the vehicle. If None is
                       passed in, then the local forward direction of the vehicle will be used.

        Returns:
            The velocity along the provided direction vector.
        """
    def force_load_physics_from_usd(self) -> None: 
        """
        Forces load physics objects from USD into PhysX. By default physics is not loaded;
        this function forces a load of all physics from USD into PhysX.
        """
    def get_backwards_compatibility_check_log(self) -> str: 
        """
        Get backwards compatibility check log. Valid for last run of backward compatibility check.
        """
    def get_error_event_stream(self) -> carb.events._events.IEventStream: 
        """
        Error event stream sending various error events defined in ErrorEvent enum.

        Returns:
            Event stream sending the physics errors.
        """
    def get_num_collision_tasks(self) -> int: 
        """
        Returns the number of active collision cooking tasks
        """
    def get_overwrite_gpu_setting(self) -> int: 
        """
        Get CPU vs GPU simulation settings.

        Returns:
            Integer defining the behavior: -1 - use setting from schema, 0 - force CPU, 1 - force GPU
        """
    def get_rigidbody_transformation(self, path: str) -> dict: 
        """
        Gets rigid body current transformation in a global space.

        Args:
            path: The USD path to the rigid body.

        Returns:
            Return a dictionary with transformation info::

                'ret_val': bool - whether transformation was found
                'position': float3 - rigid body position
                'rotation': float4 - rigid body rotation (quat - x,y,z,w)
        """
    def get_simulation_event_stream_v2(self) -> carb.events._events.IEventStream: 
        """
        Simulation event stream sending various simulation events defined in SimulationEvent enum.

        Returns:
            Event stream sending the simulation events.
        """
    def get_vehicle_drive_state(self, path: str) -> dict: 
        """
        Get the drive state of a vehicle.

        Note:
            Should only be called after the simulation has been started.

        Args:
            path: Vehicle USD path.

        Returns:
            A dictionary with the following key, value pairs. An empty dictionary is returned for an invalid path,
            when the simulation is not running or when the vehicle does not have a Basic or Standard drive type.

            | VEHICLE_DRIVE_STATE_ACCELERATOR, d (in range [0, 1]),
            | VEHICLE_DRIVE_STATE_BRAKE0, d (in range [0, 1]),
            | VEHICLE_DRIVE_STATE_BRAKE1, d (in range [0, 1]),
            | VEHICLE_DRIVE_STATE_STEER, d (in range [-1, 1]),
            | VEHICLE_DRIVE_STATE_CLUTCH, d (in range [0, 1], only applicable to PhysxVehicleDriveStandard drive type),
            | VEHICLE_DRIVE_STATE_CURRENT_GEAR, i (only applicable to PhysxVehicleDriveStandard drive type),
            | VEHICLE_DRIVE_STATE_TARGET_GEAR, i (only applicable to PhysxVehicleDriveStandard drive type),
            | VEHICLE_DRIVE_STATE_GEAR_SWITCH_TIME, d (in seconds, negative value if no gear shift is in process. Only applicable to PhysxVehicleDriveStandard drive type),
            | VEHICLE_DRIVE_STATE_AUTOBOX_TIME_SINCE_LAST_SHIFT, d (in seconds, only applicable toPhysxVehicleDriveStandard drive type),
            | VEHICLE_DRIVE_STATE_ENGINE_ROTATION_SPEED, d (in radians per second, only applicable to PhysxVehicleDriveStandard drive type),
            | VEHICLE_DRIVE_STATE_AUTOMATIC_TRANSMISSION, i (only applicable to PhysxVehicleDriveStandard drive type)
        """
    def get_wheel_state(self, path: str) -> dict: 
        """
        Get the wheel state.

        Note:
            Should only be called after the simulation has been started.

        Args:
            path: Wheel attachment USD path.

        Returns:
            A dictionary with the following key, value pairs. An empty dictionary is returned for an invalid path.

            | VEHICLE_WHEEL_STATE_LOCAL_POSE_POSITION, (d, d, d),
            | VEHICLE_WHEEL_STATE_LOCAL_POSE_QUATERNION, (d, d, d, d)
            | VEHICLE_WHEEL_STATE_ROTATION_SPEED, d (in radians per second),
            | VEHICLE_WHEEL_STATE_ROTATION_ANGLE, d (in radians),
            | VEHICLE_WHEEL_STATE_STEER_ANGLE, d (in radians),
            | VEHICLE_WHEEL_STATE_GROUND_PLANE, (d, d, d, d) (first 3 entries are plane normal n, fourth entry is d of equation dot(n, v) + d = 0). Only valid if wheel is on ground, see VEHICLE_WHEEL_STATE_IS_ON_GROUND,
            | VEHICLE_WHEEL_STATE_GROUND_ACTOR, string (USD path of the actor prim the wheel is driving on). Only valid if wheel is on ground, see VEHICLE_WHEEL_STATE_IS_ON_GROUND,
            | VEHICLE_WHEEL_STATE_GROUND_SHAPE, string (USD path of the collider prim the wheel is driving on). Only valid if wheel is on ground, see VEHICLE_WHEEL_STATE_IS_ON_GROUND,
            | VEHICLE_WHEEL_STATE_GROUND_MATERIAL, string (USD path of the material prim the wheel is driving on). Only valid if wheel is on ground, see VEHICLE_WHEEL_STATE_IS_ON_GROUND,
            | VEHICLE_WHEEL_STATE_GROUND_HIT_POSITION, (d, d, d) (hit position on the ground in world space). Only valid if wheel is on ground, see VEHICLE_WHEEL_STATE_IS_ON_GROUND,
            | VEHICLE_WHEEL_STATE_SUSPENSION_JOUNCE, d,
            | VEHICLE_WHEEL_STATE_SUSPENSION_FORCE, (d, d, d),
            | VEHICLE_WHEEL_STATE_TIRE_FRICTION, d,
            | VEHICLE_WHEEL_STATE_TIRE_LONGITUDINAL_DIRECTION, (d, d, d),
            | VEHICLE_WHEEL_STATE_TIRE_LATERAL_DIRECTION, (d, d, d),
            | VEHICLE_WHEEL_STATE_TIRE_LONGITUDINAL_SLIP, d,
            | VEHICLE_WHEEL_STATE_TIRE_LATERAL_SLIP, d,
            | VEHICLE_WHEEL_STATE_TIRE_FORCE, (d, d, d),
            | VEHICLE_WHEEL_STATE_IS_ON_GROUND, i, whether the wheel did touch the ground or is in air (=0). If the vehicle is disabled or sleeping then the wheel will be treated as not touching ground too.
        """
    def is_asyncsimrender_enabled(self) -> bool: 
        """
        Returns true if asynchronous simulation and rendering is enabled for one of the scenes in the simulation.
        """
    def is_interactive_actor_raycast(self, arg0: carb._carb.Float3, arg1: carb._carb.Float3) -> bool: 
        """
        Raycast check to detect interactive physics actors

        Note:
            Only produces positive results during simulation

        Args:
            origin:    carb.Float3: World-space origin of raycast
            direction: carb.Float3: Unit-length direction vector of raycast

        Returns:
            True if interactive actor is hit; False otherwise, or if simulation is not running
        """
    def is_readback_suppressed(self) -> bool: 
        """
        Check if GPU readback is suppressed for currently running simulation.

        Returns:
            True if simulation is running with suppressed readback.  Always returns false when simulation is not running.
        """
    def is_running(self) -> bool: 
        """
        Check if simulation loop is running, this function returns true if play was pressed or if
        IPhysxSimulation was attached.
        """
    def overwrite_gpu_setting(self, gpuSetting: int) -> None: 
        """
        Override CPU vs GPU simulation settings.

        Args:
            gpuSetting:  Integer defining the behavior: -1 - use setting from schema, 0 - force CPU, 1 - force GPU
        """
    def overwrite_solver_type(self, solverSetting: int) -> None: 
        """
        Override PhysX solver settings.

        Args:
            solverSetting:  Integer defining the behavior: -1 - use setting from schema, 0 - force PGS, 1 - force TGS
        """
    def put_to_sleep(self, body_path: str) -> None: 
        """
        Put body to sleep.

        Deprecated, please use apply_force_at_pos at physx_simulation_interface

        Args:
            body_path: USD path of the body.
        """
    def reconnect_pvd(self) -> None: 
        """
        Reconnect to PVD (PhysX Visual Debugger)
        """
    def release_local_mesh_cache(self) -> None: 
        """
        Release Local Mesh Cache.
        """
    def release_physics_objects(self) -> None: 
        """
        Forces release of all physics objects from PhysX.
        """
    def reset_setting(self, arg0: str) -> None: 
        """
        Resets all physics settings to their default values.
        """
    def reset_settings(self) -> None: 
        """
        Resets all physics settings to their default values.
        """
    def reset_settings_in_preferences(self) -> None: 
        """
        Resets physics preferences to their default values.
        """
    def reset_settings_in_stage(self) -> None: 
        """
        Resets physics per-stage settings  to their default values.
        """
    def reset_simulation(self) -> None: 
        """
        Reset physics simulation and set back original transformations stored during start_simulation.
        This will also remove all data from PhysX.
        """
    def run_backwards_compatibility(self, stage_id: int = -1) -> None: 
        """
        Runs the backwards compatibility code, will try to fix older schema format to a new format.
        """
    def save_scene_to_repx(self, arg0: str) -> bool: 
        """
        Save scene to RepX, if scene is not loaded, load scene, save and release.
        """
    def set_simulation_layer(self, layer: str) -> None: 
        """
        Sets simulation layer. This layer is used when simulation output transformations are written to USD.

        Args:
            layer: The layer that we simulate to.
        """
    def set_thread_count(self, numThreads: int) -> None: 
        """
        Sets number of threads for simulation.

        Args:
            numThreads:  Number of threads that physics should use
        """
    def set_vehicle_to_rest_state(self, path: str) -> None: 
        """
        Set the internal dynamics state of a vehicle back to the rest state.

        Note:
            Should only be called after the simulation has been started.

        Args:
            path: Vehicle USD path.
        """
    def set_voxel_range(self, stage_id: int, path: str, sx: int, sy: int, sz: int, ex: int, ey: int, ez: int, type: int, subtype: int, update: int) -> bool: 
        """
        Set Voxelmap Voxels

        Args:
            stage_id: Stage containing source mesh primitive.
            input_path: path to input primitive
            sx: voxel range start X
            sy: voxel range start Y
            sz: voxel range start Z
            ex: voxel range end X
            ey: voxel range end Y
            ez: voxel range end Z
            type: voxel type
            sub_type: voxel subtype
            update: update flag, if zero, writing changes to USD is postponed, if non-zero, all accumulated changes are written to USD
        """
    def set_wheel_rotation_angle(self, path: str, rotationAngle: float) -> None: 
        """
        Set the rotation angle about the rolling axis of a wheel.

        Note:
            Should only be called after the simulation has been started.

        Args:
            path: Wheel attachment USD path.
            rotationAngle: Rotation angle of the wheel in radians.
        """
    def set_wheel_rotation_speed(self, path: str, rotationSpeed: float) -> None: 
        """
        Set the rotation speed about the rolling axis of a wheel.

        Note:
            Should only be called after the simulation has been started.

        Args:
            path: Wheel attachment USD path.
            rotationSpeed: Rotation speed of the wheel in radians per second.
        """
    def start_simulation(self) -> None: 
        """
        Start simulation, store initial USD data. Call this before manually stepping simulation.
        """
    def subscribe_object_changed_notifications(self, object_creation_fn: typing.Callable[[int, int, int], None] = None, object_destruction_fn: typing.Callable[[int, int, int], None] = None, all_objects_destruction_fn: typing.Callable[[], None] = None, stop_callback_when_sim_stopped: bool = True) -> int: 
        """
        Subscribes to Object Changed Notifications

        Args:
            object_creation_fn:         function(path, object_id, physx_type)   Notification when a physics object gets created during simulation.
            object_destruction_fn:      function(path, object_id, physx_type)   Notification when a physics object gets destroyed during simulation.
            all_objects_destruction_fn: function                                Notification when all physics objects get destroyed during simulation.
            stopCallbackWhenSimStopped: bool                                    Whether these callbacks should not be sent when the simulation stops

        Returns:
            The subscription holder (SubscriptionId) to use with unsubscribe_object_change_notifications
        """
    def subscribe_physics_on_step_events(self, fn: typing.Callable[[float], None], pre_step: bool, order: int) -> carb._carb.Subscription: 
        """
        Subscribes to physics pre-step or post-step events.

        Subscription cannot be changed in the onUpdate callback

        Args:
            fn:         The callback to be called right before or after every physics step.
            pre_step:   Whether fn has to be called right *before* the physics step. If this is false,
                        it will be called right *after* the physics step.
            order:      An integer value used to order the callbacks: 0 means "highest priority", 1 is "less priority" and so on.

        Returns:
            The subscription holder.
        """
    def subscribe_physics_step_events(self, fn: typing.Callable[[float], None]) -> carb._carb.Subscription: 
        """
        Subscribes to physics step events.

        Subscription cannot be changed in the onUpdate callback

        Args:
            fn: The callback to be called on every physics step.

        Returns:
            The subscription holder.
        """
    def unsubscribe_object_change_notifications(self, subscription_id: int) -> None: 
        """
        Unsubscribes object change notifications with the subscriptionID as returned by subscribe_object_changed_notifications
        """
    def update_interaction(self, arg0: carb._carb.Float3, arg1: carb._carb.Float3, arg2: PhysicsInteractionEvent) -> None: 
        """
        Updates actor interaction based on user input and raycast origin and direction

        Note:
            Only provides interaction during simulation

        Args:
            origin:          carb.Float3: World-space origin of interaction
            direction:       carb.Float3: Unit-length direction vector of interaction
            event:           PhysicsInteractionEvent triggered.
        """
    def update_simulation(self, elapsedStep: float, currentTime: float) -> None: 
        """
        Update physics simulation by one step.

        Args:
            elapsedStep:  Simulation step time (seconds), time elapsed between last step and this step.
            currentTime:  Current time (seconds), might be used for time sampled transformations to apply.
        """
    def update_simulation_scene(self, scene_path: int, elapsed_step: float, current_time: float) -> None: 
        """
        Update and step a specific scene in the physics simulation. The specific scene specified in scenePath
        is updated and stepped *even if marked as 'Disabled'*.
        If scenePath is empty, it behaves like IPhysx::updateSimulation

        Args:
            scenePath:       uint64_t       Scene USD path use PhysicsSchemaTools::sdfPathToInt
            elapsedStep:     float          Simulation time (seconds).
            currentTime:     float          Current time (seconds), might be used for time sampled transformations to apply.
        """
    def update_transformations(self, updateToFastCache: bool, updateToUsd: bool, updateVelocitiesToUsd: bool = False, outputVelocitiesLocalSpace: bool = False) -> None: 
        """
        Update transformations after simulation is done.

        Args:
            updateToFastCache:      Update transformation in fast cache.
            updateToUsd:            Update transformations in USD.
            updateVelocitiesToUsd:  Update velocities in USD.
        """
    def update_transformations_scene(self, scene_path: int, update_to_usd: bool, update_velocities_to_usd: bool) -> None: 
        """
        Update the transformations for a specific scene in the physics simulation. The specific scene specified in scenePath
        has its transformations updated *even if it is marked as 'Disabled'*.
        If scenePath is empty, it behaves like IPhysx::updateTransformations

        scenePath              uint64_t         Scene USD path use PhysicsSchemaTools::sdfPathToInt
        updateToUsd            bool             Update transforms to USD.
        updateVelocitiesToUsd  bool             Update velocities to USD.
        """
    def wake_up(self, body_path: str) -> None: 
        """
        Wake up body.

        Deprecated, please use apply_force_at_pos at physx_simulation_interface

        Args:
            body_path: USD path of the body.
        """
    pass
class PhysXCooking():
    """
    This interface is the access point to the omni.physx extension cooking API.
    """
    @staticmethod
    def add_prim_to_cooking_refresh_set(*args, **kwargs) -> typing.Any: 
        """
        Adds Prim to cooking refresh set.

        Args:
            primPath: path to Prim
        """
    def cancel_collision_representation_task(self, task: PhysxCollisionRepresentationTask, invoke_callback: bool = True) -> None: 
        """
        Cancels an async physics collision representation task made with request_convex_collision_representation

        Args:
            task:            PhysxCollisionRepresentationTask The task returned by request_convex_collision_representation to cancel
            invoke_callback: bool                             If the on_result callback should be invoked anyway
        """
    def cancel_collision_tasks(self) -> int: 
        """
        Returns the number of active collision cooking tasks which were canceled
        """
    def compute_conforming_tetrahedral_mesh(self, src_tri_points: typing.List[carb._carb.Float3], src_tri_indices: typing.List[int]) -> dict: 
        """
        Create a conforming tetrahedral mesh from a closed source triangle mesh.

        Args:
            src_tri_points      Vertices of the source triangle mesh
            src_tri_indices     Vertex indices of the source triangles

        Returns:
            Dict mapping 'points' and 'indices' to the resulting tetrahedral mesh points and indices

        The conforming tetrahedral mesh is defined as a tetrahedral mesh whose surface triangles align with the closed source triangle mesh and whose
        internal vertices lie on the inside of the closed source triangle mesh.
        """
    def compute_edge_length_limited_triangle_mesh(self, arg0: typing.List[carb._carb.Float3], arg1: typing.List[int], arg2: float) -> dict: 
        """
        Limits the maximum edge length in a triangle mesh
        """
    def compute_voxel_tetrahedral_mesh(self, src_tet_points: typing.List[carb._carb.Float3], src_tet_indices: typing.List[int], src_scale: carb._carb.Float3, voxel_resolution: int) -> dict: 
        """
        Create a voxel tetrahedral mesh from a source tetrahedral mesh.

        Args:
            src_tet_points:     Vertices of teh source tetrahedral mesh
            src_tet_indices:    Vertex indices of the source tetrahedral mesh
            src_scale:          Scale of source mesh used to determine resolution of the resulting voxel mesh
            voxel_resolution:   Number of voxels along longest dimension of axis aligned bounding box of source mesh

        Returns:
            Dict mapping 'points' and 'indices' to the resulting tetrahedral mesh points and indices

        The voxel tetrahedral mesh is made by voxelizing the source tetrahedra on a regular grid. The
        resulting voxel tetrahedral mesh embeds all tetrahedra of the source mesh.
        The provided voxel resolution may be lowered automatically in order to match a lower resolution detected in
        the source mesh. This may help to avoid softbody convergence issues with high-resolution tetrahedra
        embedding low resolution collision meshes.
        """
    def cook_deformable_body_mesh(self, deformable_body_path: str) -> bool: 
        """
        Cooks deformable body mesh

        Args:
            stage_id: Stage containing source mesh primitive.
            deformable_body_path: path to UsdGeomMesh with PhysxSchemaPhysxDeformableBodyAPI
        """
    def get_convex_mesh_data(self, meshPath: str, convexIndex: int) -> dict: 
        """
        Get convex mesh data for given prim path. (Does work only when simulation is running.)

        Returns:
            Return a dictionary with convex mesh::

                'num_vertices': int - number of vertices
                'num_polygons': int - number of polygons
                'vertices': list - vertices list
                'polygons': list - list of dictionaries, dictionary with polygon data::
                    'plane' : float4 - Plane format: (mPlane[0],mPlane[1],mPlane[2]).dot(x) + mPlane[3] = 0
                    'num_vertices' : int - number of vertices for the current polygon
                    'index_base' : int - base index to the indices array
                'indices': list - list of indices
        """
    def get_nb_convex_mesh_data(self, meshPath: str) -> int: 
        """
        Get number of convex mesh data for given prim path. (Does work only when simulation is running.)
        """
    def get_num_collision_tasks(self) -> int: 
        """
        Returns the number of active collision cooking tasks
        """
    @staticmethod
    def poisson_sample_mesh(*args, **kwargs) -> typing.Any: 
        """
        Samples particle positions from mesh

        Args:
            mesh_path: path to UsdGeomMesh with PhysxParticleSamplingAPI
        """
    def release_local_mesh_cache(self) -> None: 
        """
        Release Local Mesh Cache.
        """
    @staticmethod
    def request_convex_collision_representation(*args, **kwargs) -> typing.Any: 
        """
        Request physics collision representation of an USD mesh

        Args:
            stage_id:           uint64_t    Stage where prim exists
            collision_prim_id:  uint64_t    Prim Id of the prim with CollisionAPI
            run_asynchronously: bool        If the request should be run asynchronously
            on_result:          Callable[PhysxCollisionRepresentationResult, list[PhysxConvexMeshData])  
                    The callback with the wanted collision representation.
                    result: Value of type PhysxCollisionRepresentationResult
                    convexes: list of PhysxConvexMeshData

        Returns: A PhysxCollisionRepresentationTask that can be cancelled with cancel_collision_representation_task 
                 (if async request has been made)
        """
    def wait_for_cooking_to_finish(self) -> None: 
        """
        Waits for all cooking tasks to finish.
        """
    pass
class PhysXSceneQuery():
    """
    This interface is the access point to the omni.physx extension scene query API.
    """
    def overlap_box(self, halfExtent: carb._carb.Float3, pos: carb._carb.Float3, rot: carb._carb.Float4, reportFn: typing.Callable[[OverlapHit], bool], anyHit: bool = False) -> int: 
        """
        Overlap test of a box against objects in the physics scene.

        Args:
            halfExtent: Box half extent.
            pos: Origin of the box overlap (barycenter of the box).
            rot: Rotation of the box overlap (quat x, y, z, w)
            reportFn: Report function where SceneQueryHits will be reported, return True to continue traversal, False to stop traversal
            anyHit: Boolean defining whether overlap should report only bool hit (0 no hit, 1 hit found).

        Returns:
            Return number of hits founds
        """
    def overlap_box_any(self, halfExtent: carb._carb.Float3, pos: carb._carb.Float3, rot: carb._carb.Float4) -> bool: 
        """
        Overlap test of a box against objects in the physics scene, reports only boolean

        Args:
            extent: Box extent.
            pos: Origin of the box overlap.
            rot: Rotation of the box overlap (quat x, y, z, w)

        Returns:
            Returns True if overlap found
        """
    def overlap_mesh(self, meshPath0: int, meshPath1: int, reportFn: typing.Callable[[OverlapHit], bool], anyHit: bool = False) -> int: 
        """
        Overlap test of a UsdGeom.Mesh against objects in the physics scene. Overlap test will use convex mesh
        approximation of the input mesh. The first query will need to cook this approximation so if the results
        are not stored in a local cache, this query might be slow.

        Args:
            mesh: UsdGeom.Mesh path encoded into two integers. Use PhysicsSchemaTools.encodeSdfPath to get those.
            reportFn: Report function where SceneQueryHits will be reported, return True to continue traversal, False to stop traversal
            anyHit: Boolean defining whether overlap should report only bool hit (0 no hit, 1 hit found).

        Returns:
            Return number of hits founds
        """
    def overlap_mesh_any(self, meshPath0: int, meshPath1: int) -> bool: 
        """
        Overlap test of a UsdGeom.Mesh against objects in the physics scene. Overlap test will use convex mesh
        approximation of the input mesh. The first query will need to cook this approximation so if the results
        are not stored in a local cache, this query might be slow. Reports only boolean.

        Args:
            mesh: UsdGeom.Mesh path encoded into two integers. Use PhysicsSchemaTools.encodeSdfPath to get those.

        Returns:
            Returns True if overlap was found.
        """
    def overlap_shape(self, meshPath0: int, meshPath1: int, reportFn: typing.Callable[[OverlapHit], bool], anyHit: bool = False) -> int: 
        """
        Overlap test of a UsdGeom.GPrim against objects in the physics scene. Overlap test will use convex mesh
        approximation if the input is a mesh. The first query will need to cook this approximation so if the results
        are not stored in a local cache, this query might be slow.

        Args:
            gprim: UsdGeom.GPrim path encoded into two integers. Use PhysicsSchemaTools.encodeSdfPath to get those.
            reportFn: Report function where SceneQueryHits will be reported, return True to continue traversal, False to stop traversal
            anyHit: Boolean defining whether overlap should report only bool hit (0 no hit, 1 hit found).

        Returns:
            Return number of hits founds
        """
    def overlap_shape_any(self, meshPath0: int, meshPath1: int) -> bool: 
        """
        Overlap test of a UsdGeom.GPrim against objects in the physics scene. Overlap test will use convex mesh
        approximation if the input is a mesh. The first query will need to cook this approximation so if the results
        are not stored in a local cache, this query might be slow. Reports only boolean.

        Args:
            gprim: UsdGeom.GPrim path encoded into two integers. Use PhysicsSchemaTools.encodeSdfPath to get those.

        Returns:
            Returns True if overlap was found.
        """
    def overlap_sphere(self, radius: float, pos: carb._carb.Float3, reportFn: typing.Callable[[OverlapHit], bool], anyHit: bool = False) -> int: 
        """
        Overlap test of a sphere against objects in the physics scene.

        Args:
            radius: Sphere radius.
            pos: Origin of the sphere overlap.
            reportFn: Report function where SceneQueryHits will be reported, return True to continue traversal, False to stop traversal
            anyHit: Boolean defining whether overlap should report only bool hit (0 no hit, 1 hit found).

        Returns:
            Return number of hits founds
        """
    def overlap_sphere_any(self, radius: float, pos: carb._carb.Float3) -> bool: 
        """
        Overlap test of a sphere against objects in the physics scene, reports only boolean.

        Args:
            radius: Sphere radius.
            pos: Origin of the sphere overlap.

        Returns:
            Returns True if overlap found.
        """
    def raycast_all(self, origin: carb._carb.Float3, dir: carb._carb.Float3, distance: float, reportFn: typing.Callable[[RaycastHit], bool], bothSides: bool = False) -> bool: 
        """
        Raycast physics scene for all collisions.

        Args:
            origin: Origin of the raycast.
            dir: Unit direction of the raycast.
            distance: Raycast distance.
            reportFn: Report function where RaycastHits will be reported, return True to continue traversal, False to stop traversal
            bothSides: Boolean defining whether front and back side of a mesh triangle should be considered for hits.

        Returns:
            Returns true if hit
        """
    def raycast_any(self, origin: carb._carb.Float3, dir: carb._carb.Float3, distance: float, bothSides: bool = False) -> bool: 
        """
        Raycast physics scene for any collision, reporting only boolean.

        Args:
            origin: Origin of the raycast.
            dir: Unit direction of the raycast.
            distance: Raycast distance.
            bothSides: Boolean defining whether front and back side of a mesh triangle should be considered for hits.

        Returns:
            Return a boolean whether raycast hit.
        """
    def raycast_closest(self, origin: carb._carb.Float3, dir: carb._carb.Float3, distance: float, bothSides: bool = False) -> dict: 
        """
        Raycast physics scene for the closest collision.

        Args:
            origin: Origin of the raycast.
            dir: Unit direction of the raycast.
            distance: Raycast distance.
            bothSides: Boolean defining whether front and back side of a mesh triangle should be considered for hits.

        Returns:
            Return a dictionary with raycast hit::

                'position': float3 -- Hit position.
                'normal': float3 -- Hit normal.
                'distance': float -- Hit distance.
                'faceIndex': int -- Hit mesh face index.
                'collision': string -- Hit collision USD path.
                'rigidBody': string -- Hit rigid body USD path.
                'protoIndex': int -- Rigid body protoIndex - if not point instancer 0xFFFFFFFF
                'material': string -- Hit collider material USD path.
        """
    def sweep_box_all(self, halfExtent: carb._carb.Float3, pos: carb._carb.Float3, rot: carb._carb.Float4, dir: carb._carb.Float3, distance: float, reportFn: typing.Callable[[SweepHit], bool], bothSides: bool = False) -> bool: 
        """
        Box sweep physics scene for all collisions.

        Args:
            halfExtent: Box half extent.
            pos: Origin of the sweep (barycenter of the box).
            rot: Rotation of the box (quat x, y, z, w)
            dir: Unit direction of the sweep.
            distance: sweep distance.
            reportFn: Report function where SweepHits will be reported, return True to continue traversal, False to stop traversal
            bothSides: Boolean defining whether front and back side of a mesh triangle should be considered for hits.

        Returns:
            Returns true if hit
        """
    def sweep_box_any(self, halfExtent: carb._carb.Float3, pos: carb._carb.Float3, rot: carb._carb.Float4, dir: carb._carb.Float3, distance: float, bothSides: bool = False) -> bool: 
        """
        Box sweep physics scene for any collision, reporting only boolean.

        Args:
            halfExtent: Box half extent.
            pos: Origin of the sweep (barycenter of the box).
            rot: Rotation of the sweep box (quat x, y, z, w)
            dir: Unit direction of the sweep.
            distance: sweep distance.
            bothSides: Boolean defining whether front and back side of a mesh triangle should be considered for hits.

        Returns:
            Return a boolean if sweep hit.
        """
    def sweep_box_closest(self, halfExtent: carb._carb.Float3, pos: carb._carb.Float3, rot: carb._carb.Float4, dir: carb._carb.Float3, distance: float, bothSides: bool = False) -> dict: 
        """
        Box sweep physics scene for the closest collision.

        Args:
            halfExtent: Box half extent.
            pos: Origin of the sweep (barycenter of the box).
            rot: Rotation of the sweep box (quat x, y, z, w)
            dir: Unit direction of the sweep.
            distance: sweep distance.
            bothSides: Boolean defining whether front and back side of a mesh triangle should be considered for hits.

        Returns:
            Return a dictionary with raycast hit::

                'position': float3 - hit position
                'normal': float3 - hit normal
                'distance': float - hit distance
                'faceIndex': int - hit mesh face index
                'collision': string - hit collision USD path
                'rigidBody': string - hit rigid body USD path
                'protoIndex': int -- Rigid body protoIndex - if not point instancer 0xFFFFFFFF
                'material': string -- Hit collider material USD path.
        """
    def sweep_mesh_all(self, meshPath0: int, meshPath1: int, dir: carb._carb.Float3, distance: float, reportFn: typing.Callable[[SweepHit], bool], bothSides: bool = False) -> bool: 
        """
        Sweep test of a UsdGeom.Mesh against objects in the physics scene. Sweep test will use convex mesh
        approximation of the input mesh. The first query will need to cook this approximation so if the results
        are not stored in a local cache, this query might be slow.

        Args:
            mesh: UsdGeom.Mesh path encoded into two integers. Use PhysicsSchemaTools.encodeSdfPath to get those.
            dir: Unit direction of the sweep.
            distance: sweep distance.
            reportFn: Report function where SweepHits will be reported, return True to continue traversal, False to stop traversal
            bothSides: Boolean defining whether front and back side of a mesh triangle should be considered for hits.

        Returns:
            Returns true if hit
        """
    def sweep_mesh_any(self, meshPath0: int, meshPath1: int, dir: carb._carb.Float3, distance: float, bothSides: bool = False) -> bool: 
        """
        Sweep test of a UsdGeom.Mesh against objects in the physics scene for any collision, reporting only boolean.
        Sweep test will use convex mesh approximation of the input mesh. The first query will need to cook this
        approximation so if the results are not stored in a local cache, this query might be slow.

        Args:
            mesh: UsdGeom.Mesh path encoded into two integers. Use PhysicsSchemaTools.encodeSdfPath to get those.
            dir: Unit direction of the sweep.
            distance: sweep distance.
            bothSides: Boolean defining whether front and back side of a mesh triangle should be considered for hits.

        Returns:
            Return a boolean if sweep hit.
        """
    def sweep_mesh_closest(self, meshPath0: int, meshPath1: int, dir: carb._carb.Float3, distance: float, bothSides: bool = False) -> dict: 
        """
        Sweep test of a UsdGeom.Mesh against objects in the physics scene for the closest collision. Sweep test will use convex mesh
        approximation of the input mesh. The first query will need to cook this approximation so if the results
        are not stored in a local cache, this query might be slow.

        Args:
            mesh: UsdGeom.Mesh path encoded into two integers. Use PhysicsSchemaTools.encodeSdfPath to get those.
            dir: Unit direction of the sweep.
            distance: sweep distance.
            bothSides: Boolean defining whether front and back side of a mesh triangle should be considered for hits.

        Returns:
            Return a dictionary with sweep hit:

                'position': float3 - hit position
                'normal': float3 - hit normal
                'distance': float - hit distance
                'faceIndex': int - hit mesh face index
                'collision': string - hit collision USD path
                'rigidBody': string - hit rigid body USD path
                'protoIndex': int -- Rigid body protoIndex - if not point instancer 0xFFFFFFFF
                'material': string -- Hit collider material USD path.
        """
    def sweep_shape_all(self, meshPath0: int, meshPath1: int, dir: carb._carb.Float3, distance: float, reportFn: typing.Callable[[SweepHit], bool], bothSides: bool = False) -> bool: 
        """
        Sweep test of a UsdGeom.GPrim against objects in the physics scene. Sweep test will use convex mesh
        approximation if the input is a mesh. The first query will need to cook this approximation so if the results
        are not stored in a local cache, this query might be slow.

        Args:
            gprim: UsdGeom.GPrim path encoded into two integers. Use PhysicsSchemaTools.encodeSdfPath to get those.
            dir: Unit direction of the sweep.
            distance: sweep distance.
            reportFn: Report function where SweepHits will be reported, return True to continue traversal, False to stop traversal
            bothSides: Boolean defining whether front and back side of a mesh triangle should be considered for hits.

        Returns:
            Returns true if hit
        """
    def sweep_shape_any(self, meshPath0: int, meshPath1: int, dir: carb._carb.Float3, distance: float, bothSides: bool = False) -> bool: 
        """
        Sweep test of a UsdGeom.GPrim against objects in the physics scene for any collision, reporting only boolean.
        Sweep test will use convex mesh approximation if the input is a mesh. The first query will need to cook this
        approximation so if the results are not stored in a local cache, this query might be slow.

        Args:
            gprim: UsdGeom.GPrim path encoded into two integers. Use PhysicsSchemaTools.encodeSdfPath to get those.
            dir: Unit direction of the sweep.
            distance: sweep distance.
            bothSides: Boolean defining whether front and back side of a mesh triangle should be considered for hits.

        Returns:
            Return a boolean if sweep hit.
        """
    def sweep_shape_closest(self, meshPath0: int, meshPath1: int, dir: carb._carb.Float3, distance: float, bothSides: bool = False) -> dict: 
        """
        Sweep test of a UsdGeom.GPrim against objects in the physics scene. Sweep test will use convex mesh
        approximation if the input is a mesh. The first query will need to cook this approximation so if the results
        are not stored in a local cache, this query might be slow.

        Args:
            gprim: UsdGeom.GPrim path encoded into two integers. Use PhysicsSchemaTools.encodeSdfPath to get those.
            dir: Unit direction of the sweep.
            distance: sweep distance.
            bothSides: Boolean defining whether front and back side of a mesh triangle should be considered for hits.

        Returns:
            Return a dictionary with sweep hit:

                'position': float3 - hit position
                'normal': float3 - hit normal
                'distance': float - hit distance
                'faceIndex': int - hit mesh face index
                'collision': string - hit collision USD path
                'rigidBody': string - hit rigid body USD path
                'protoIndex': int -- Rigid body protoIndex - if not point instancer 0xFFFFFFFF
                'material': string -- Hit collider material USD path.
        """
    def sweep_sphere_all(self, radius: float, origin: carb._carb.Float3, dir: carb._carb.Float3, distance: float, reportFn: typing.Callable[[SweepHit], bool], bothSides: bool = False) -> bool: 
        """
        Sphere sweep physics scene for all collisions.

        Args:
            radius: Sphere radius
            origin: Origin of the sweep.
            dir: Unit direction of the sweep.
            distance: sweep distance.
            reportFn: Report function where SweepHits will be reported, return True to continue traversal, False to stop traversal
            bothSides: Boolean defining whether front and back side of a mesh triangle should be considered for hits.

        Returns:
            Returns true if hit
        """
    def sweep_sphere_any(self, radius: float, origin: carb._carb.Float3, dir: carb._carb.Float3, distance: float, bothSides: bool = False) -> bool: 
        """
        Sphere sweep physics scene for any collision, reporting only boolean.

        Args:
            radius: Sphere radius.
            origin: Origin of the sphere sweep.
            dir: Unit direction of the sphere sweep.
            distance: Sphere sweep distance.
            bothSides: Boolean defining whether front and back side of a mesh triangle should be considered for hits.

        Returns:
            Return a boolean if sphere sweep hit.
        """
    def sweep_sphere_closest(self, radius: float, origin: carb._carb.Float3, dir: carb._carb.Float3, distance: float, bothSides: bool = False) -> dict: 
        """
        Sphere sweep physics scene for the closest collision.

        Args:
            radius: Sphere radius.
            origin: Origin of the sphere cast.
            dir: Unit direction of the sphere cast.
            distance: Sphere cast distance.
            bothSides: Boolean defining whether front and back side of a mesh triangle should be considered for hits.

        Returns:
            Return a dictionary with sweep hit:

                'position': float3 - hit position
                'normal': float3 - hit normal
                'distance': float - hit distance
                'faceIndex': int - hit mesh face index
                'collision': string - hit collision USD path
                'rigidBody': string - hit rigid body USD path
                'protoIndex': int -- Rigid body protoIndex - if not point instancer 0xFFFFFFFF
                'material': string -- Hit collider material USD path.
        """
    pass
class PhysXUnitTests():
    """
    This interface is the access point to omni.physx test support functions.
    """
    def end_logger_check(self) -> bool: ...
    def get_mass_information(self, body_path: str) -> dict: 
        """
        Returns mass information for a given body.

        Args:
            body_path - USD path to body.

        Returns:
            Return a dictionary with mass information::

                'mass': float - Mass of the body.
                'inertia': float3 - Inertia tensor of body.
                'com': float3 - Center of mass of the body.
        """
    def get_materials_paths(self, collider_path: str) -> list: 
        """
        Returns material paths for materials found on given collider path.

        Args:
            collider_path - USD path to a collider.

        Returns:
            Return a list of material paths
        """
    def get_physics_stats(self) -> dict: 
        """
        Returns simulation statistics after a simulation step.

        Returns:
            Return a dictionary with statistics::

                'numDynamicRigids': int - Number of dynamic rigids in simulation.
                'numStaticRigids': int - Number of static rigids in simulation.
                'numKinematicBodies': int - Number of kinematic rigids in simulation.
                'numArticulations': int - Number of articulations in simulation.
                'numSphereShapes': int - Number of sphere shapes in simulation.
                'numBoxShapes': int - Number of box shapes in simulation.
                'numCapsuleShapes': int - Number of capsule shapes in simulation.
                'numCylinderShapes': int - Number of cylinder shapes in simulation.
                'numConvexShapes': int - Number of convex shapes in simulation.
                'numTriMeshShapes': int - Number of triangle mesh shapes in simulation.
                'numPlaneShapes': int - Number of plane shapes in simulation.
                'numConeShapes': int - Number of cone shapes in simulation.
        """
    def is_cuda_lib_present(self) -> bool: ...
    def start_logger_check(self, arg0: str, arg1: bool) -> None: ...
    def start_logger_check_for_multiple(self, arg0: typing.List[str], arg1: bool, arg2: bool) -> None: ...
    def update(self, elapsedStep: float, currentTime: float) -> None: 
        """
        Update physics simulation by one step.

        Args:
            elapsedStep:  Simulation step time (seconds), time elapsed between last step and this step.
            currentTime:  Current time (seconds), might be used for time sampled transformations to apply.
        """
    pass
class PhysXVisualization():
    """
    This interface is the access point to PhysX SDK debug visualization.
    """
    def enable_visualization(self, enable: bool) -> None: 
        """
        Enable/disable PhysX debug visualization.

        Args:
            enable - Bool if enable or disable.
        """
    def get_nb_lines(self) -> int: 
        """
        Get number of PhysX debug visualization lines. This serves mostly as a test function.
        """
    def set_visualization_parameter(self, debug_vis: str, enable: bool) -> None: 
        """
        Toggle individual debug visualization features.

        Args:
            debug_vis - Debug visualization feature string identifier, can be one of the following:
                {'WorldAxes', 'BodyAxes', 'BodyMassAxes', 'BodyLinearVel', 'BodyAngularVel', 'ContactPoint',
                'ContactNormal', 'ContactError', 'ContactImpulse', 'FrictionPoint', 'FrictionNormal',
                'FrictionImpulse', 'ActorAxes', 'CollisionAABBs', 'CollisionShapes', 'CollisionAxes',
                'CollisionCompounds', 'CollisionEdges', 'CollisionStaticPruner', 'CollisionDynamicPruner',
                'JointLocalFrames', 'JointLimits', 'CullBox', 'MBPRegions'}
            enable - Bool to enable/disable the feature.
        """
    def set_visualization_scale(self, scale: float) -> None: 
        """
        Set PhysX debug visualization scale.

        Args:
            scale - Float value for scaling debug visualization.
        """
    pass
class PhysicsInteractionEvent():
    """
            Physics interaction event
            

    Members:

      MOUSE_DRAG_BEGAN : Signals that a mouse drag has begun.

      MOUSE_DRAG_CHANGED : Signals that the mouse is being dragged.

      MOUSE_DRAG_ENDED : Signals that the mouse drag is being released.

      MOUSE_LEFT_CLICK : Signals that the mouse left button is clicked.

      MOUSE_LEFT_DOUBLE_CLICK : Signals that the mouse left button is being double-clicked.
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
    MOUSE_DRAG_BEGAN: omni.physx.bindings._physx.PhysicsInteractionEvent # value = <PhysicsInteractionEvent.MOUSE_DRAG_BEGAN: 0>
    MOUSE_DRAG_CHANGED: omni.physx.bindings._physx.PhysicsInteractionEvent # value = <PhysicsInteractionEvent.MOUSE_DRAG_CHANGED: 1>
    MOUSE_DRAG_ENDED: omni.physx.bindings._physx.PhysicsInteractionEvent # value = <PhysicsInteractionEvent.MOUSE_DRAG_ENDED: 2>
    MOUSE_LEFT_CLICK: omni.physx.bindings._physx.PhysicsInteractionEvent # value = <PhysicsInteractionEvent.MOUSE_LEFT_CLICK: 3>
    MOUSE_LEFT_DOUBLE_CLICK: omni.physx.bindings._physx.PhysicsInteractionEvent # value = <PhysicsInteractionEvent.MOUSE_LEFT_DOUBLE_CLICK: 4>
    __members__: dict # value = {'MOUSE_DRAG_BEGAN': <PhysicsInteractionEvent.MOUSE_DRAG_BEGAN: 0>, 'MOUSE_DRAG_CHANGED': <PhysicsInteractionEvent.MOUSE_DRAG_CHANGED: 1>, 'MOUSE_DRAG_ENDED': <PhysicsInteractionEvent.MOUSE_DRAG_ENDED: 2>, 'MOUSE_LEFT_CLICK': <PhysicsInteractionEvent.MOUSE_LEFT_CLICK: 3>, 'MOUSE_LEFT_DOUBLE_CLICK': <PhysicsInteractionEvent.MOUSE_LEFT_DOUBLE_CLICK: 4>}
    pass
class PhysicsSceneStats():
    """
    Physics scene statistics.
    """
    def __init__(self) -> None: ...
    def __repr__(self) -> str: ...
    @property
    def compressed_contact_size(self) -> int:
        """
        The size (in bytes) of the compressed contact stream in the current simulation step.

        :type: int
        """
    @property
    def gpu_mem_collision_stack_size(self) -> int:
        """
        Actual GPU device memory (bytes) needed for the collision stack of Gpu Collision Stack Size set with physxScene:gpuCollisionStackSize

        :type: int
        """
    @property
    def gpu_mem_deformable_surface_contacts(self) -> int:
        """
        Actual number of deformable surface contacts needed of Gpu Max Deformable Surface Contacts set with physxScene:gpuMaxDeformableSurfaceContacts

        :type: int
        """
    @property
    def gpu_mem_fem_cloths(self) -> int:
        """
        GPU device memory in bytes allocated for FEM-based cloth state accessible through API.

        :type: int
        """
    @property
    def gpu_mem_found_lost_aggregate_pairs(self) -> int:
        """
        Actual number of found/lost aggregate pairs needed of Gpu Found Lost Aggregate Pairs Capacity set with physxScene:gpuFoundLostAggregatePairsCapacity

        :type: int
        """
    @property
    def gpu_mem_found_lost_pairs(self) -> int:
        """
        Actual number of found/lost pairs needed of Gpu Found Lost Pairs Capacity set with physxScene:gpuFoundLostPairsCapacity

        :type: int
        """
    @property
    def gpu_mem_hair_systems(self) -> int:
        """
        GPU device memory in bytes allocated for hairsystem state accessible through API.

        :type: int
        """
    @property
    def gpu_mem_heap(self) -> int:
        """
        GPU device memory in bytes allocated for internal heap allocation based on initial Gpu Heap Capacity set with physxScene:gpuHeapCapacity.

        :type: int
        """
    @property
    def gpu_mem_heap_articulation(self) -> int:
        """
        GPU device heap memory used for articulations in bytes.

        :type: int
        """
    @property
    def gpu_mem_heap_broadphase(self) -> int:
        """
        GPU device heap memory used for broad phase in bytes.

        :type: int
        """
    @property
    def gpu_mem_heap_fem_cloths(self) -> int:
        """
        GPU device heap memory used for shared buffers in the FEM-based cloth pipeline in bytes.

        :type: int
        """
    @property
    def gpu_mem_heap_hair_systems(self) -> int:
        """
        GPU device heap memory used for shared buffers in the hairsystem pipeline in bytes.

        :type: int
        """
    @property
    def gpu_mem_heap_narrowphase(self) -> int:
        """
        GPU device heap memory used for narrow phase in bytes.

        :type: int
        """
    @property
    def gpu_mem_heap_other(self) -> int:
        """
        GPU device heap memory not covered by other stats in bytes.

        :type: int
        """
    @property
    def gpu_mem_heap_particles(self) -> int:
        """
        GPU device heap memory used for shared buffers in the particles pipeline in bytes.

        :type: int
        """
    @property
    def gpu_mem_heap_simulation(self) -> int:
        """
        GPU device heap memory used for simulation pipeline in bytes.

        :type: int
        """
    @property
    def gpu_mem_heap_simulation_articulation(self) -> int:
        """
        GPU device heap memory used for articulations in the simulation pipeline in bytes.

        :type: int
        """
    @property
    def gpu_mem_heap_simulation_fem_cloths(self) -> int:
        """
        GPU device heap memory used for FEM-cloth in the simulation pipeline in bytes.

        :type: int
        """
    @property
    def gpu_mem_heap_simulation_hair_systems(self) -> int:
        """
        GPU device heap memory used for hairsystem in the simulation pipeline in bytes.

        :type: int
        """
    @property
    def gpu_mem_heap_simulation_particles(self) -> int:
        """
        GPU device heap memory used for particles in the simulation pipeline in bytes.

        :type: int
        """
    @property
    def gpu_mem_heap_simulation_softbody(self) -> int:
        """
        GPU device heap memory used for soft bodies in the simulation pipeline in bytes.

        :type: int
        """
    @property
    def gpu_mem_heap_softbodies(self) -> int:
        """
        GPU device heap memory used for shared buffers in the FEM-based soft body pipeline in bytes.

        :type: int
        """
    @property
    def gpu_mem_heap_solver(self) -> int:
        """
        GPU device heap memory used for solver in bytes.

        :type: int
        """
    @property
    def gpu_mem_particle_contacts(self) -> int:
        """
        Actual number of particle contacts needed of Gpu Max Particle Contact Count set with physxScene:gpuMaxParticleContacts

        :type: int
        """
    @property
    def gpu_mem_particles(self) -> int:
        """
        GPU device memory in bytes allocated for particle state accessible through API.

        :type: int
        """
    @property
    def gpu_mem_rigid_contact_count(self) -> int:
        """
        Actual number of rigid contacts needed of Gpu Max Rigid Contact Count set with physxScene:gpuMaxRigidContactCount

        :type: int
        """
    @property
    def gpu_mem_rigid_patch_count(self) -> int:
        """
        Actual number of rigid contact patches needed of Gpu Max Rigid Patch Count set with physxScene:gpuMaxRigidPatchCount

        :type: int
        """
    @property
    def gpu_mem_softbodies(self) -> int:
        """
        GPU device memory in bytes allocated for FEM-based soft body state accessible through API.

        :type: int
        """
    @property
    def gpu_mem_softbody_contacts(self) -> int:
        """
        Actual number of soft body contacts needed of Gpu Max Softbody Contacts set with physxScene:gpuMaxSoftBodyContacts

        :type: int
        """
    @property
    def gpu_mem_temp_buffer_capacity(self) -> int:
        """
        Actual GPU device memory (bytes) used for Temp Buffer based on initial Gpu Temp Buffer Capacity set with physxScene:gpuTempBufferCapacity

        :type: int
        """
    @property
    def gpu_mem_total_aggregate_pairs(self) -> int:
        """
        Actual number of aggregate pairs needed of Gpu Total Aggregate Pairs Capacity set with physxScene:gpuTotalAggregatePairsCapacity

        :type: int
        """
    @property
    def nb_active_constraints(self) -> int:
        """
        The number of active constraints.

        :type: int
        """
    @property
    def nb_active_dynamic_rigids(self) -> int:
        """
        The number of active dynamic rigid bodies.

        :type: int
        """
    @property
    def nb_active_kinematic_rigids(self) -> int:
        """
        The number of active kinematic rigid bodies.

        :type: int
        """
    @property
    def nb_aggregates(self) -> int:
        """
        The number of aggregates.

        :type: int
        """
    @property
    def nb_articulations(self) -> int:
        """
        The number of articulations.

        :type: int
        """
    @property
    def nb_axis_solver_constaints(self) -> int:
        """
        The number of 1D axis constraints(joints+contact) present in the current simulation step.

        :type: int
        """
    @property
    def nb_box_shapes(self) -> int:
        """
        The number of box shapes.

        :type: int
        """
    @property
    def nb_capsule_shapes(self) -> int:
        """
        The number of capsule shapes.

        :type: int
        """
    @property
    def nb_cone_shapes(self) -> int:
        """
        The number of cone shapes.

        :type: int
        """
    @property
    def nb_convex_shapes(self) -> int:
        """
        The number of convex shapes.

        :type: int
        """
    @property
    def nb_cylinder_shapes(self) -> int:
        """
        The number of cylinder shapes.

        :type: int
        """
    @property
    def nb_discrete_contact_pairs_total(self) -> int:
        """
        Total number of (non CCD) pairs reaching narrow phase.

        :type: int
        """
    @property
    def nb_discrete_contact_pairs_with_cache_hits(self) -> int:
        """
        Total number of (non CCD) pairs for which contacts are successfully cached (<=nbDiscreteContactPairsTotal) note This includes pairs for which no contacts are generated, it still counts as a cache hit.

        :type: int
        """
    @property
    def nb_discrete_contact_pairs_with_contacts(self) -> int:
        """
        Total number of (non CCD) pairs for which at least 1 contact was generated (<=nbDiscreteContactPairsTotal).

        :type: int
        """
    @property
    def nb_dynamic_rigids(self) -> int:
        """
        The number of dynamic rigid bodies.

        :type: int
        """
    @property
    def nb_kinematic_rigids(self) -> int:
        """
        The number of kinematic rigid bodies.

        :type: int
        """
    @property
    def nb_lost_pairs(self) -> int:
        """
        Number of lost pairs from BP this frame.

        :type: int
        """
    @property
    def nb_lost_touches(self) -> int:
        """
        Number of lost touches from NP this frame.

        :type: int
        """
    @property
    def nb_new_pairs(self) -> int:
        """
        Number of new pairs found by BP this frame.

        :type: int
        """
    @property
    def nb_new_touches(self) -> int:
        """
        Number of new touches found by NP this frame.

        :type: int
        """
    @property
    def nb_partitions(self) -> int:
        """
        Number of partitions used by the solver this frame.

        :type: int
        """
    @property
    def nb_plane_shapes(self) -> int:
        """
        The number of plane shapes.

        :type: int
        """
    @property
    def nb_sphere_shapes(self) -> int:
        """
        The number of sphere shapes.

        :type: int
        """
    @property
    def nb_static_rigids(self) -> int:
        """
        The number of static rigid bodies.

        :type: int
        """
    @property
    def nb_trimesh_shapes(self) -> int:
        """
        The number of triangle mesh shapes.

        :type: int
        """
    @property
    def peak_constraint_memory(self) -> int:
        """
        The peak amount of memory (in bytes) that was allocated for constraints (this includes joints) in the current simulation step.

        :type: int
        """
    @property
    def required_contact_constraint_memory(self) -> int:
        """
        The total required size (in bytes) of the contact constraints in the current simulation step.

        :type: int
        """
    pass
class PhysxCollisionRepresentationResult():
    """
    Collision representation result

    Members:

      RESULT_VALID

      RESULT_ERROR_NOT_READY

      RESULT_ERROR_INVALID_PARSING

      RESULT_ERROR_COOKING_FAILED

      RESULT_ERROR_UNSUPPORTED_APPROXIMATION

      RESULT_ERROR_INVALID_RESULT
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
    RESULT_ERROR_COOKING_FAILED: omni.physx.bindings._physx.PhysxCollisionRepresentationResult # value = <PhysxCollisionRepresentationResult.RESULT_ERROR_COOKING_FAILED: 3>
    RESULT_ERROR_INVALID_PARSING: omni.physx.bindings._physx.PhysxCollisionRepresentationResult # value = <PhysxCollisionRepresentationResult.RESULT_ERROR_INVALID_PARSING: 2>
    RESULT_ERROR_INVALID_RESULT: omni.physx.bindings._physx.PhysxCollisionRepresentationResult # value = <PhysxCollisionRepresentationResult.RESULT_ERROR_INVALID_RESULT: 5>
    RESULT_ERROR_NOT_READY: omni.physx.bindings._physx.PhysxCollisionRepresentationResult # value = <PhysxCollisionRepresentationResult.RESULT_ERROR_NOT_READY: 1>
    RESULT_ERROR_UNSUPPORTED_APPROXIMATION: omni.physx.bindings._physx.PhysxCollisionRepresentationResult # value = <PhysxCollisionRepresentationResult.RESULT_ERROR_UNSUPPORTED_APPROXIMATION: 4>
    RESULT_VALID: omni.physx.bindings._physx.PhysxCollisionRepresentationResult # value = <PhysxCollisionRepresentationResult.RESULT_VALID: 0>
    __members__: dict # value = {'RESULT_VALID': <PhysxCollisionRepresentationResult.RESULT_VALID: 0>, 'RESULT_ERROR_NOT_READY': <PhysxCollisionRepresentationResult.RESULT_ERROR_NOT_READY: 1>, 'RESULT_ERROR_INVALID_PARSING': <PhysxCollisionRepresentationResult.RESULT_ERROR_INVALID_PARSING: 2>, 'RESULT_ERROR_COOKING_FAILED': <PhysxCollisionRepresentationResult.RESULT_ERROR_COOKING_FAILED: 3>, 'RESULT_ERROR_UNSUPPORTED_APPROXIMATION': <PhysxCollisionRepresentationResult.RESULT_ERROR_UNSUPPORTED_APPROXIMATION: 4>, 'RESULT_ERROR_INVALID_RESULT': <PhysxCollisionRepresentationResult.RESULT_ERROR_INVALID_RESULT: 5>}
    pass
class PhysxCollisionRepresentationTask():
    """
    Task returned by request_convex_collision_representation
    """
    @property
    def task(self) -> capsule:
        """
        Task handle

        :type: capsule
        """
    pass
class PhysxConvexMeshData():
    """
    A convex mesh made of vertices, indices and polygons
    """
    @property
    def indices(self) -> typing.List[int]:
        """
        :type: typing.List[int]
        """
    @property
    def polygons(self) -> typing.List[PhysxConvexMeshPolygon]:
        """
        :type: typing.List[PhysxConvexMeshPolygon]
        """
    @property
    def vertices(self) -> typing.List[carb._carb.Float3]:
        """
        :type: typing.List[carb._carb.Float3]
        """
    pass
class PhysxConvexMeshPolygon():
    """
    A polygon of a convex mesh
    """
    @property
    def index_base(self) -> int:
        """
        :type: int
        """
    @property
    def num_vertices(self) -> int:
        """
        :type: int
        """
    @property
    def plane(self) -> carb._carb.Float4:
        """
        :type: carb._carb.Float4
        """
    pass
class PhysxPropertyQueryColliderResponse():
    """
    Collider query response.
    """
    @property
    def aabb_local_max(self) -> carb._carb.Float3:
        """
        AABB Max Local Bound

        :type: carb._carb.Float3
        """
    @property
    def aabb_local_min(self) -> carb._carb.Float3:
        """
        AABB Min Local Bound

        :type: carb._carb.Float3
        """
    @property
    def path_id(self) -> int:
        """
        USD Path

        :type: int
        """
    @property
    def result(self) -> PhysxPropertyQueryResult:
        """
        Result

        :type: PhysxPropertyQueryResult
        """
    @property
    def stage_id(self) -> int:
        """
        USD Stage

        :type: int
        """
    @property
    def volume(self) -> float:
        """
        Volume of the collider

        :type: float
        """
    pass
class PhysxPropertyQueryMode():
    """
            Query mode.
            

    Members:

      QUERY_RIGID_BODY_WITH_COLLIDERS : Query rigid body and its colliders
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
    QUERY_RIGID_BODY_WITH_COLLIDERS: omni.physx.bindings._physx.PhysxPropertyQueryMode # value = <PhysxPropertyQueryMode.QUERY_RIGID_BODY_WITH_COLLIDERS: 0>
    __members__: dict # value = {'QUERY_RIGID_BODY_WITH_COLLIDERS': <PhysxPropertyQueryMode.QUERY_RIGID_BODY_WITH_COLLIDERS: 0>}
    pass
class PhysxPropertyQueryResult():
    """
            Query result enumeration.
            

    Members:

      VALID : Result is valid

      ERROR_UNKNOWN_QUERY_MODE : The requested query mode is unknown

      ERROR_INVALID_USD_PATH : Result invalid because of an invalid USD path

      ERROR_INVALID_USD_STAGE : Result invalid because of an invalid or expired USD stage

      ERROR_INVALID_USD_PRIM : Result invalid because of an invalid or deleted USD prim

      ERROR_PARSING : Result invalid because parsing USD failed

      ERROR_TIMEOUT : Result invalid because async operation exceeds timeout

      ERROR_RUNTIME : Result invalid because PhysX runtime is in invalid state
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
    ERROR_INVALID_USD_PATH: omni.physx.bindings._physx.PhysxPropertyQueryResult # value = <PhysxPropertyQueryResult.ERROR_INVALID_USD_PATH: 2>
    ERROR_INVALID_USD_PRIM: omni.physx.bindings._physx.PhysxPropertyQueryResult # value = <PhysxPropertyQueryResult.ERROR_INVALID_USD_PRIM: 4>
    ERROR_INVALID_USD_STAGE: omni.physx.bindings._physx.PhysxPropertyQueryResult # value = <PhysxPropertyQueryResult.ERROR_INVALID_USD_STAGE: 3>
    ERROR_PARSING: omni.physx.bindings._physx.PhysxPropertyQueryResult # value = <PhysxPropertyQueryResult.ERROR_PARSING: 5>
    ERROR_RUNTIME: omni.physx.bindings._physx.PhysxPropertyQueryResult # value = <PhysxPropertyQueryResult.ERROR_RUNTIME: 7>
    ERROR_TIMEOUT: omni.physx.bindings._physx.PhysxPropertyQueryResult # value = <PhysxPropertyQueryResult.ERROR_TIMEOUT: 6>
    ERROR_UNKNOWN_QUERY_MODE: omni.physx.bindings._physx.PhysxPropertyQueryResult # value = <PhysxPropertyQueryResult.ERROR_UNKNOWN_QUERY_MODE: 1>
    VALID: omni.physx.bindings._physx.PhysxPropertyQueryResult # value = <PhysxPropertyQueryResult.VALID: 0>
    __members__: dict # value = {'VALID': <PhysxPropertyQueryResult.VALID: 0>, 'ERROR_UNKNOWN_QUERY_MODE': <PhysxPropertyQueryResult.ERROR_UNKNOWN_QUERY_MODE: 1>, 'ERROR_INVALID_USD_PATH': <PhysxPropertyQueryResult.ERROR_INVALID_USD_PATH: 2>, 'ERROR_INVALID_USD_STAGE': <PhysxPropertyQueryResult.ERROR_INVALID_USD_STAGE: 3>, 'ERROR_INVALID_USD_PRIM': <PhysxPropertyQueryResult.ERROR_INVALID_USD_PRIM: 4>, 'ERROR_PARSING': <PhysxPropertyQueryResult.ERROR_PARSING: 5>, 'ERROR_TIMEOUT': <PhysxPropertyQueryResult.ERROR_TIMEOUT: 6>, 'ERROR_RUNTIME': <PhysxPropertyQueryResult.ERROR_RUNTIME: 7>}
    pass
class PhysxPropertyQueryRigidBodyResponse():
    """
    Rigid body query response.
    """
    @property
    def center_of_mass(self) -> carb._carb.Float3:
        """
        Center of Mass

        :type: carb._carb.Float3
        """
    @property
    def inertia(self) -> carb._carb.Float3:
        """
        Inertia

        :type: carb._carb.Float3
        """
    @property
    def mass(self) -> float:
        """
        Mass

        :type: float
        """
    @property
    def path_id(self) -> int:
        """
        USD Path

        :type: int
        """
    @property
    def principal_axes(self) -> carb._carb.Float4:
        """
        Principal Axes Quaternion

        :type: carb._carb.Float4
        """
    @property
    def result(self) -> PhysxPropertyQueryResult:
        """
        Result

        :type: PhysxPropertyQueryResult
        """
    @property
    def stage_id(self) -> int:
        """
        USD Stage

        :type: int
        """
    @property
    def type(self) -> PhysxPropertyQueryRigidBodyResponseType:
        """
        Type

        :type: PhysxPropertyQueryRigidBodyResponseType
        """
    pass
class PhysxPropertyQueryRigidBodyResponseType():
    """
            Query result.
            

    Members:

      RIGID_DYNAMIC : Body is a rigid dynamic
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
    RIGID_DYNAMIC: omni.physx.bindings._physx.PhysxPropertyQueryRigidBodyResponseType # value = <PhysxPropertyQueryRigidBodyResponseType.RIGID_DYNAMIC: 0>
    __members__: dict # value = {'RIGID_DYNAMIC': <PhysxPropertyQueryRigidBodyResponseType.RIGID_DYNAMIC: 0>}
    pass
class RaycastHit(SceneQueryHitLocation, SceneQueryHitObject):
    """
    Raycast hit results structure.
    """
    pass
class SceneQueryHitLocation(SceneQueryHitObject):
    """
    Scene query hit location results structure.
    """
    @property
    def distance(self) -> float:
        """
        Hit location distance.

        :type: float
        """
    @property
    def face_index(self) -> int:
        """
        Hit location face index.

        :type: int
        """
    @property
    def material(self) -> str:
        """
        Path string to the collider material that was hit.

        :type: str
        """
    @property
    def material_encoded(self) -> list:
        """
        Encoded SdfPath to the collider material that was hit. PhysicsSchemaTools.decodeSdfPath will return SdfPath.

        :type: list
        """
    @property
    def normal(self) -> carb._carb.Float3:
        """
        Hit location normal.

        :type: carb._carb.Float3
        """
    @property
    def position(self) -> carb._carb.Float3:
        """
        Hit location position.

        :type: carb._carb.Float3
        """
    pass
class SceneQueryHitObject():
    """
    Scene query hit results structure.
    """
    @property
    def collision(self) -> str:
        """
        Path string to the collision that was hit.

        :type: str
        """
    @property
    def collision_encoded(self) -> list:
        """
        Encoded SdfPath to the collision that was hit. PhysicsSchemaTools.decodeSdfPath will return SdfPath.

        :type: list
        """
    @property
    def protoIndex(self) -> int:
        """
        ProtoIndex, filled for pointInstancers otherwise 0xFFFFFFFF.

        :type: int
        """
    @property
    def rigid_body(self) -> str:
        """
        Path string to the rigid body that was hit.

        :type: str
        """
    @property
    def rigid_body_encoded(self) -> list:
        """
        Encoded SdfPath to the rigid body that was hit. PhysicsSchemaTools.decodeSdfPath will return SdfPath.

        :type: list
        """
    pass
class SimulationEvent():
    """
            Simulation events used by simulation event stream.
            

    Members:

      RESUMED : Simulation resumed, no additional data are send in the event

      PAUSED : Simulation paused, no additional data are send in the event

      STOPPED : Simulation stopped, no additional data are send in the event

      CONTACT_FOUND : Contact found event header: sends header information regarding which colliders started to collide; contains the following in a dictionary:

                .. code-block:: text

                    'actor0':int2 - Usd path to rigid body actor 0 decoded into two ints. PhysicsSchemaTools.decodeSdfPath will return SdfPath.
                    'actor1':int2 - Usd path to rigid body actor 1 decoded into two ints. PhysicsSchemaTools.decodeSdfPath will return SdfPath.
                    'collider0':int2 - Usd path to collider 0 decoded into two ints. PhysicsSchemaTools.decodeSdfPath will return SdfPath.
                    'collider1':int2 - Usd path to collider 1 decoded into two ints. PhysicsSchemaTools.decodeSdfPath will return SdfPath.
                    'numContactData':int - Num contact data sent after the header is sent.
                    'stageId':long1 - Current USD stage id, long array with one item.
                

      CONTACT_LOST : Contact lost event header: sends header information regarding which colliders lost contact; contains the following in a dictionary:

                .. code-block:: text

                    'actor0':int2 - Usd path to rigid body actor 0 decoded into two ints. PhysicsSchemaTools.decodeSdfPath will return SdfPath.
                    'actor1':int2 - Usd path to rigid body actor 1 decoded into two ints. PhysicsSchemaTools.decodeSdfPath will return SdfPath.
                    'collider0':int2 - Usd path to collider 0 decoded into two ints. PhysicsSchemaTools.decodeSdfPath will return SdfPath.
                    'collider1':int2 - Usd path to collider 1 decoded into two ints. PhysicsSchemaTools.decodeSdfPath will return SdfPath.
                    'numContactData':int - Num contact data sent after the header is sent.
                    'stageId':long1 - Current USD stage id, long array with one item.
                

      CONTACT_PERSISTS : Contact persists event header: sends header information regarding which colliders are still in contact; contains the following in a dictionary:

                .. code-block:: text

                    'actor0':int2 - Usd path to rigid body actor 0 decoded into two ints. PhysicsSchemaTools.decodeSdfPath will return SdfPath.
                    'actor1':int2 - Usd path to rigid body actor 1 decoded into two ints. PhysicsSchemaTools.decodeSdfPath will return SdfPath.
                    'collider0':int2 - Usd path to collider 0 decoded into two ints. PhysicsSchemaTools.decodeSdfPath will return SdfPath.
                    'collider1':int2 - Usd path to collider 1 decoded into two ints. PhysicsSchemaTools.decodeSdfPath will return SdfPath.
                    'numContactData':int - Num contact data sent after the header is sent.
                    'stageId':long1 - Current USD stage id, long array with one item.
                

      CONTACT_DATA : Contact data sent after each header contact information is sent; contains the following in a dictionary:

                .. code-block:: text

                    'position':float3 - Contact position
                    'normal':float3 - Contact normal
                    'impulse':float3 - Contact impulse
                    'separation':float - Separation value for collisions.
                    'faceIndex0':int - USD face index 0.
                    'faceIndex1':int - USD face index 0.
                    'material0':int2 - Usd path to material 0 decoded into two ints. PhysicsSchemaTools.decodeSdfPath will return SdfPath.
                    'material1':int2 - Usd path to material 0 decoded into two ints. PhysicsSchemaTools.decodeSdfPath will return SdfPath.
                

      JOINT_BREAK : Joint break event; contains the following in a dictionary:

                .. code-block:: text

                    'jointPath':int2 - Usd path to joint that did break decoded into two ints. PhysicsSchemaTools.decodeSdfPath will return SdfPath.
                

      POINT_GRABBED : Point grabbed; contains the following in a dictionary:

                .. code-block:: text

                    'grabbed_position':float3 - Current world position of grabbed point.
                    'grab_force_position':float3 - Current world position of the position being grabbed towards.
                

      POINT_RELEASED : Point released.
                

      POINT_PUSHED : Point pushed; contains the following in a dictionary:

                .. code-block:: text

                    'pushed_position':float3 - World position of point pushed.
                

      ATTACHED_TO_STAGE : When physx stage attachment (initialization) finished.
                

      DETACHED_FROM_STAGE : When physx stage detachment (deinitialization) finished.
                
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
    ATTACHED_TO_STAGE: omni.physx.bindings._physx.SimulationEvent # value = <SimulationEvent.ATTACHED_TO_STAGE: 10>
    CONTACT_DATA: omni.physx.bindings._physx.SimulationEvent # value = <SimulationEvent.CONTACT_DATA: 6>
    CONTACT_FOUND: omni.physx.bindings._physx.SimulationEvent # value = <SimulationEvent.CONTACT_FOUND: 3>
    CONTACT_LOST: omni.physx.bindings._physx.SimulationEvent # value = <SimulationEvent.CONTACT_LOST: 4>
    CONTACT_PERSISTS: omni.physx.bindings._physx.SimulationEvent # value = <SimulationEvent.CONTACT_PERSISTS: 5>
    DETACHED_FROM_STAGE: omni.physx.bindings._physx.SimulationEvent # value = <SimulationEvent.DETACHED_FROM_STAGE: 11>
    JOINT_BREAK: omni.physx.bindings._physx.SimulationEvent # value = <SimulationEvent.JOINT_BREAK: 7>
    PAUSED: omni.physx.bindings._physx.SimulationEvent # value = <SimulationEvent.PAUSED: 1>
    POINT_GRABBED: omni.physx.bindings._physx.SimulationEvent # value = <SimulationEvent.POINT_GRABBED: 8>
    POINT_PUSHED: omni.physx.bindings._physx.SimulationEvent # value = <SimulationEvent.POINT_PUSHED: 12>
    POINT_RELEASED: omni.physx.bindings._physx.SimulationEvent # value = <SimulationEvent.POINT_RELEASED: 9>
    RESUMED: omni.physx.bindings._physx.SimulationEvent # value = <SimulationEvent.RESUMED: 0>
    STOPPED: omni.physx.bindings._physx.SimulationEvent # value = <SimulationEvent.STOPPED: 2>
    __members__: dict # value = {'RESUMED': <SimulationEvent.RESUMED: 0>, 'PAUSED': <SimulationEvent.PAUSED: 1>, 'STOPPED': <SimulationEvent.STOPPED: 2>, 'CONTACT_FOUND': <SimulationEvent.CONTACT_FOUND: 3>, 'CONTACT_LOST': <SimulationEvent.CONTACT_LOST: 4>, 'CONTACT_PERSISTS': <SimulationEvent.CONTACT_PERSISTS: 5>, 'CONTACT_DATA': <SimulationEvent.CONTACT_DATA: 6>, 'JOINT_BREAK': <SimulationEvent.JOINT_BREAK: 7>, 'POINT_GRABBED': <SimulationEvent.POINT_GRABBED: 8>, 'POINT_RELEASED': <SimulationEvent.POINT_RELEASED: 9>, 'POINT_PUSHED': <SimulationEvent.POINT_PUSHED: 12>, 'ATTACHED_TO_STAGE': <SimulationEvent.ATTACHED_TO_STAGE: 10>, 'DETACHED_FROM_STAGE': <SimulationEvent.DETACHED_FROM_STAGE: 11>}
    pass
class SweepHit(SceneQueryHitLocation, SceneQueryHitObject):
    """
    Sweep hit results structure.
    """
    pass
class TriggerEventData():
    """
    Parameters for trigger event callback.
    """
    @property
    def event_type(self) -> TriggerEventType:
        """
        Event Type (enter or leave)

        :type: TriggerEventType
        """
    @property
    def other_body_prim_id(self) -> int:
        """
        USD Path of the body containgint the other collider prim entering trigger

        :type: int
        """
    @property
    def other_collider_prim_id(self) -> int:
        """
        USD Path of other prim entering trigger

        :type: int
        """
    @property
    def stage_id(self) -> int:
        """
        USD Stage

        :type: int
        """
    @property
    def subscription_id(self) -> int:
        """
        Id of the subscription returned by subscribe_physics_trigger_report_events

        :type: int
        """
    @property
    def trigger_body_prim_id(self) -> int:
        """
        USD Path of the body containg the collider prim representing the trigger

        :type: int
        """
    @property
    def trigger_collider_prim_id(self) -> int:
        """
        USD Path of prim representing the trigger

        :type: int
        """
    pass
class TriggerEventType():
    """
            Trigger Event type.
            

    Members:

      TRIGGER_ON_ENTER : The collider has entered trigger volume

      TRIGGER_ON_LEAVE : The collider has left trigger volume
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
    TRIGGER_ON_ENTER: omni.physx.bindings._physx.TriggerEventType # value = <TriggerEventType.TRIGGER_ON_ENTER: 0>
    TRIGGER_ON_LEAVE: omni.physx.bindings._physx.TriggerEventType # value = <TriggerEventType.TRIGGER_ON_LEAVE: 1>
    __members__: dict # value = {'TRIGGER_ON_ENTER': <TriggerEventType.TRIGGER_ON_ENTER: 0>, 'TRIGGER_ON_LEAVE': <TriggerEventType.TRIGGER_ON_LEAVE: 1>}
    pass
class VisualizerMode():
    """
    Visualization mode for collider, particles, deformables, etc. object types.
    """
    ALL = 2
    NONE = 0
    SELECTED = 1
    pass
def acquire_physx_attachment_interface(plugin_name: str = None, library_path: str = None) -> IPhysxAttachment:
    pass
def acquire_physx_benchmarks_interface(plugin_name: str = None, library_path: str = None) -> IPhysxBenchmarks:
    pass
def acquire_physx_cooking_interface(plugin_name: str = None, library_path: str = None) -> PhysXCooking:
    pass
def acquire_physx_interface(plugin_name: str = None, library_path: str = None) -> PhysX:
    pass
def acquire_physx_property_query_interface(plugin_name: str = None, library_path: str = None) -> IPhysxPropertyQuery:
    pass
def acquire_physx_replicator_interface(plugin_name: str = None, library_path: str = None) -> IPhysxReplicator:
    pass
def acquire_physx_scene_query_interface(plugin_name: str = None, library_path: str = None) -> PhysXSceneQuery:
    pass
def acquire_physx_simulation_interface(plugin_name: str = None, library_path: str = None) -> IPhysxSimulation:
    pass
def acquire_physx_stage_update_interface(plugin_name: str = None, library_path: str = None) -> IPhysxStageUpdate:
    pass
def acquire_physx_statistics_interface(plugin_name: str = None, library_path: str = None) -> IPhysxStatistics:
    pass
def acquire_physx_visualization_interface(plugin_name: str = None, library_path: str = None) -> PhysXVisualization:
    pass
def acquire_physxunittests_interface(plugin_name: str = None, library_path: str = None) -> PhysXUnitTests:
    pass
def ancestorHasAPI(arg0: TfType, arg1: UsdPrim) -> bool:
    pass
def descendantHasAPI(arg0: TfType, arg1: UsdPrim) -> bool:
    pass
def hasconflictingapis_ArticulationRoot(arg0: UsdPrim, arg1: bool) -> bool:
    pass
def hasconflictingapis_CollisionAPI(arg0: UsdPrim, arg1: bool) -> bool:
    pass
def hasconflictingapis_PhysxDeformableBodyAPI(arg0: UsdPrim, arg1: bool) -> bool:
    pass
def hasconflictingapis_PhysxDeformableSurfaceAPI(arg0: UsdPrim, arg1: bool) -> bool:
    pass
def hasconflictingapis_PhysxHairAPI(arg0: UsdPrim, arg1: bool) -> bool:
    pass
def hasconflictingapis_PhysxParticleClothAPI(arg0: UsdPrim, arg1: bool) -> bool:
    pass
def hasconflictingapis_PhysxParticleSamplingAPI(arg0: UsdPrim, arg1: bool) -> bool:
    pass
def hasconflictingapis_Precompute(arg0: UsdPrim) -> typing.Annotated[typing.List[bool], pybind11_stubgen.typing_ext.FixedSize(3)]:
    pass
def hasconflictingapis_RigidBodyAPI(arg0: UsdPrim, arg1: bool) -> bool:
    pass
def isOverConflictingApisSubtreeLimit(arg0: UsdPrim, arg1: int) -> bool:
    pass
def release_physx_attachment_interface(arg0: IPhysxAttachment) -> None:
    pass
def release_physx_benchmarks_interface(arg0: IPhysxBenchmarks) -> None:
    pass
def release_physx_cooking_interface(arg0: PhysXCooking) -> None:
    pass
def release_physx_interface(arg0: PhysX) -> None:
    pass
def release_physx_interface_scripting(arg0: PhysX) -> None:
    pass
def release_physx_property_query_interface(arg0: IPhysxPropertyQuery) -> None:
    pass
def release_physx_replicator_interface(arg0: IPhysxReplicator) -> None:
    pass
def release_physx_replicator_interface_scripting(arg0: IPhysxReplicator) -> None:
    pass
def release_physx_scene_query_interface(arg0: PhysXSceneQuery) -> None:
    pass
def release_physx_simulation_interface(arg0: IPhysxSimulation) -> None:
    pass
def release_physx_stage_update_interface(arg0: IPhysxStageUpdate) -> None:
    pass
def release_physx_stage_update_interface_scripting(arg0: IPhysxStageUpdate) -> None:
    pass
def release_physx_statistics_interface(arg0: IPhysxStatistics) -> None:
    pass
def release_physx_statistics_interface_scripting(arg0: IPhysxStatistics) -> None:
    pass
def release_physx_visualization_interface(arg0: PhysXVisualization) -> None:
    pass
def release_physxunittests_interface(arg0: PhysXUnitTests) -> None:
    pass
METADATA_ATTRIBUTE_NAME_LOCALSPACEVELOCITIES = 'physics:localSpaceVelocities'
SETTING_ADDMENU_SELECTION_LIMIT = '/physics/addMenuSelectionLimit'
SETTING_ADDMENU_SELECTION_LIMIT_DEFAULT = '/defaults/physics/addMenuSelectionLimit'
SETTING_ADDMENU_SUBTREE_LIMIT = '/physics/addMenuSubtreeLimit'
SETTING_ADDMENU_SUBTREE_LIMIT_DEFAULT = '/defaults/physics/addMenuSubtreeLimit'
SETTING_AUTOCREATE_PHYSICS_SCENE = '/persistent/physics/autocreatePhysicsScene'
SETTING_AUTOCREATE_PHYSICS_SCENE_DEFAULT = '/defaults/persistent/physics/autocreatePhysicsScene'
SETTING_AUTO_POPUP_SIMULATION_OUTPUT_WINDOW = '/physics/autoPopupSimulationOutputWindow'
SETTING_AUTO_POPUP_SIMULATION_OUTPUT_WINDOW_DEFAULT = '/defaults/physics/autoPopupSimulationOutputWindow'
SETTING_BACKWARD_COMPATIBILITY = '/persistent/physics/backwardCompatibilityCheckMode'
SETTING_BACKWARD_COMPATIBILITY_DEFAULT = '/defaults/persistent/physics/backwardCompatibilityCheckMode'
SETTING_COLLISION_CONE_CUSTOM_GEOMETRY = '/physics/collisionConeCustomGeometry'
SETTING_COLLISION_CONE_CUSTOM_GEOMETRY_DEFAULT = '/defaults/physics/collisionConeCustomGeometry'
SETTING_COLLISION_CYLINDER_CUSTOM_GEOMETRY = '/physics/collisionCylinderCustomGeometry'
SETTING_COLLISION_CYLINDER_CUSTOM_GEOMETRY_DEFAULT = '/defaults/physics/collisionCylinderCustomGeometry'
SETTING_CUDA_DEVICE = '/physics/cudaDevice'
SETTING_CUDA_DEVICE_DEFAULT = '/defaults/physics/cudaDevice'
SETTING_DEBUG_VIS_QUERY_USDRT_FOR_TRAVERSAL = '/persistent/physics/visualizationQueryUsdrtForTraversal'
SETTING_DEBUG_VIS_QUERY_USDRT_FOR_TRAVERSAL_DEFAULT = '/defaults/persistent/physics/visualizationQueryUsdrtForTraversal'
SETTING_DEBUG_VIS_SIMPLIFY_AT_DISTANCE = '/persistent/physics/visualizationSimplifyAtDistance'
SETTING_DEBUG_VIS_SIMPLIFY_AT_DISTANCE_DEFAULT = '/defaults/persistent/physics/visualizationSimplifyAtDistance'
SETTING_DEFAULT_SIMULATOR = '/physics/defaultSimulator'
SETTING_DEFAULT_SIMULATOR_DEFAULT = '/defaults/physics/defaultSimulator'
SETTING_DEMO_ASSETS_PATH = '/physics/demoAssetsPath'
SETTING_DEMO_ASSETS_PATH_DEFAULT = '/defaults/physics/demoAssetsPath'
SETTING_DISABLE_CONTACT_PROCESSING = '/physics/disableContactProcessing'
SETTING_DISABLE_CONTACT_PROCESSING_DEFAULT = '/defaults/physics/disableContactProcessing'
SETTING_DISABLE_SLEEPING = '/physics/disableSleeping'
SETTING_DISABLE_SLEEPING_DEFAULT = '/defaults/physics/disableSleeping'
SETTING_DISPLAY_ATTACHMENTS = '/persistent/physics/visualizationDisplayAttachments'
SETTING_DISPLAY_ATTACHMENTS_DEFAULT = '/defaults/persistent/physics/visualizationDisplayAttachments'
SETTING_DISPLAY_ATTACHMENTS_HIDE_ACTOR_0 = '/persistent/physics/visualizationDisplayAttachmentsHideActor0'
SETTING_DISPLAY_ATTACHMENTS_HIDE_ACTOR_0_DEFAULT = '/defaults/persistent/physics/visualizationDisplayAttachmentsHideActor0'
SETTING_DISPLAY_ATTACHMENTS_HIDE_ACTOR_1 = '/persistent/physics/visualizationDisplayAttachmentsHideActor1'
SETTING_DISPLAY_ATTACHMENTS_HIDE_ACTOR_1_DEFAULT = '/defaults/persistent/physics/visualizationDisplayAttachmentsHideActor1'
SETTING_DISPLAY_COLLIDERS = '/persistent/physics/visualizationDisplayColliders'
SETTING_DISPLAY_COLLIDERS_DEFAULT = '/defaults/persistent/physics/visualizationDisplayColliders'
SETTING_DISPLAY_COLLIDER_NORMALS = '/persistent/physics/visualizationDisplayColliderNormals'
SETTING_DISPLAY_COLLIDER_NORMALS_DEFAULT = '/defaults/persistent/physics/visualizationDisplayColliderNormals'
SETTING_DISPLAY_DEFORMABLE_BODIES = '/persistent/physics/visualizationDisplayDeformableBodies'
SETTING_DISPLAY_DEFORMABLE_BODIES_DEFAULT = '/defaults/persistent/physics/visualizationDisplayDeformableBodies'
SETTING_DISPLAY_DEFORMABLE_BODY_TYPE = '/persistent/physics/visualizationDisplayDeformableBodyType'
SETTING_DISPLAY_DEFORMABLE_BODY_TYPE_DEFAULT = '/defaults/persistent/physics/visualizationDisplayDeformableBodyType'
SETTING_DISPLAY_DEFORMABLE_SURFACES = '/physics/visualizationDisplayDeformableSurfaces'
SETTING_DISPLAY_DEFORMABLE_SURFACES_DEFAULT = '/defaults/physics/visualizationDisplayDeformableSurfaces'
SETTING_DISPLAY_JOINTS = '/persistent/physics/visualizationDisplayJoints'
SETTING_DISPLAY_JOINTS_DEFAULT = '/defaults/persistent/physics/visualizationDisplayJoints'
SETTING_DISPLAY_MASS_PROPERTIES = '/persistent/physics/visualizationDisplayMassProperties'
SETTING_DISPLAY_MASS_PROPERTIES_DEFAULT = '/defaults/persistent/physics/visualizationDisplayMassProperties'
SETTING_DISPLAY_PARTICLES = '/persistent/physics/visualizationDisplayParticles'
SETTING_DISPLAY_PARTICLES_DEFAULT = '/defaults/persistent/physics/visualizationDisplayParticles'
SETTING_DISPLAY_PARTICLES_POSITION_TYPE = '/persistent/physics/visualizationDisplayParticlesParticlePositions'
SETTING_DISPLAY_PARTICLES_POSITION_TYPE_DEFAULT = '/defaults/persistent/physics/visualizationDisplayParticlesParticlePositions'
SETTING_DISPLAY_PARTICLES_RADIUS_TYPE = '/persistent/physics/visualizationDisplayParticlesParticleRadius'
SETTING_DISPLAY_PARTICLES_RADIUS_TYPE_DEFAULT = '/defaults/persistent/physics/visualizationDisplayParticlesParticleRadius'
SETTING_DISPLAY_PARTICLES_SHOW_CLOTH_MESH = '/persistent/physics/visualizationDisplayParticlesShowDeformableMesh'
SETTING_DISPLAY_PARTICLES_SHOW_CLOTH_MESH_DEFAULT = '/defaults/persistent/physics/visualizationDisplayParticlesShowDeformableMesh'
SETTING_DISPLAY_PARTICLES_SHOW_CLOTH_MESH_LINES = '/persistent/physics/visualizationDisplayParticlesClothMeshLines'
SETTING_DISPLAY_PARTICLES_SHOW_CLOTH_MESH_LINES_DEFAULT = '/defaults/persistent/physics/visualizationDisplayParticlesClothMeshLines'
SETTING_DISPLAY_PARTICLES_SHOW_CLOTH_PARTICLES = '/persistent/physics/visualizationDisplayParticlesShowDeformableParticles'
SETTING_DISPLAY_PARTICLES_SHOW_CLOTH_PARTICLES_DEFAULT = '/defaults/persistent/physics/visualizationDisplayParticlesShowDeformableParticles'
SETTING_DISPLAY_PARTICLES_SHOW_DIFFUSE = '/persistent/physics/visualizationDisplayParticlesShowDiffuseParticles'
SETTING_DISPLAY_PARTICLES_SHOW_DIFFUSE_DEFAULT = '/defaults/persistent/physics/visualizationDisplayParticlesShowDiffuseParticles'
SETTING_DISPLAY_PARTICLES_SHOW_FLUID_SURFACE = '/persistent/physics/visualizationDisplayParticlesShowFluidSurface'
SETTING_DISPLAY_PARTICLES_SHOW_FLUID_SURFACE_DEFAULT = '/defaults/persistent/physics/visualizationDisplayParticlesShowFluidSurface'
SETTING_DISPLAY_PARTICLES_SHOW_PARTICLE_SET_PARTICLES = '/persistent/physics/visualizationDisplayParticlesShowParticleSetParticles'
SETTING_DISPLAY_PARTICLES_SHOW_PARTICLE_SET_PARTICLES_DEFAULT = '/defaults/persistent/physics/visualizationDisplayParticlesShowParticleSetParticles'
SETTING_DISPLAY_SIMULATION_DATA_VISUALIZER = '/persistent/physics/visualizationSimulationDataVisualizer'
SETTING_DISPLAY_SIMULATION_DATA_VISUALIZER_DEFAULT = '/defaults/persistent/physics/visualizationSimulationDataVisualizer'
SETTING_DISPLAY_SIMULATION_OUTPUT = '/persistent/physics/visualizationSimulationOutput'
SETTING_DISPLAY_SIMULATION_OUTPUT_DEFAULT = '/defaults/persistent/physics/visualizationSimulationOutput'
SETTING_DISPLAY_TENDONS = '/persistent/physics/visualizationDisplayTendons'
SETTING_DISPLAY_TENDONS_DEFAULT = '/defaults/persistent/physics/visualizationDisplayTendons'
SETTING_ENABLE_SYNCHRONOUS_KERNEL_LAUNCHES = '/physics/enableSynchronousKernelLaunches'
SETTING_ENABLE_SYNCHRONOUS_KERNEL_LAUNCHES_DEFAULT = '/defaults/physics/enableSynchronousKernelLaunches'
SETTING_EXPOSE_PRIM_PATH_NAMES = '/physics/exposePrimPathNames'
SETTING_EXPOSE_PRIM_PATH_NAMES_DEFAULT = '/defaults/physics/exposePrimPathNames'
SETTING_EXPOSE_PROFILER_DATA = '/physics/exposeProfilerData'
SETTING_EXPOSE_PROFILER_DATA_DEFAULT = '/defaults/physics/exposeProfilerData'
SETTING_FORCE_PARSE_ONLY_SINGLE_SCENE = '/physics/forceParseOnlySingleScene'
SETTING_FORCE_PARSE_ONLY_SINGLE_SCENE_DEFAULT = '/defaults/physics/forceParseOnlySingleScene'
SETTING_JOINT_BODY_TRANSFORM_CHECK_TOLERANCE = '/simulation/jointBodyTransformCheckTolerance'
SETTING_JOINT_BODY_TRANSFORM_CHECK_TOLERANCE_DEFAULT = '/defaults/simulation/jointBodyTransformCheckTolerance'
SETTING_LOCAL_MESH_CACHE_SIZE_MB = '/persistent/physics/localMeshCacheSizeMB'
SETTING_LOCAL_MESH_CACHE_SIZE_MB_DEFAULT = '/defaults/persistent/physics/localMeshCacheSizeMB'
SETTING_LOG_ROBOTICS = '/physics/logRobotics'
SETTING_LOG_ROBOTICS_DEFAULT = '/defaults/physics/logRobotics'
SETTING_LOG_SCENEMULTIGPU = '/physics/logSceneMultiGPU'
SETTING_LOG_SCENEMULTIGPU_DEFAULT = '/defaults/physics/logSceneMultiGPU'
SETTING_MASS_DISTRIBUTION_MANIPULATOR = '/persistent/physics/massDistributionManipulator'
SETTING_MASS_DISTRIBUTION_MANIPULATOR_DEFAULT = '/defaults/persistent/physics/massDistributionManipulator'
SETTING_MAX_NUMBER_OF_PHYSX_ERRORS = '/physics/maxNumberOfPhysXErrors'
SETTING_MAX_NUMBER_OF_PHYSX_ERRORS_DEFAULT = '/defaults/physics/maxNumberOfPhysXErrors'
SETTING_MIN_FRAME_RATE = '/persistent/simulation/minFrameRate'
SETTING_MIN_FRAME_RATE_DEFAULT = '/defaults/persistent/simulation/minFrameRate'
SETTING_MOUSE_GRAB = '/physics/mouseGrab'
SETTING_MOUSE_GRAB_DEFAULT = '/defaults/physics/mouseGrab'
SETTING_MOUSE_GRAB_IGNORE_INVISBLE = '/physics/mouseGrabIgnoreInvisible'
SETTING_MOUSE_GRAB_IGNORE_INVISBLE_DEFAULT = '/defaults/physics/mouseGrabIgnoreInvisible'
SETTING_MOUSE_GRAB_WITH_FORCE = '/physics/forceGrab'
SETTING_MOUSE_GRAB_WITH_FORCE_DEFAULT = '/defaults/physics/forceGrab'
SETTING_MOUSE_INTERACTION_ENABLED = '/physics/mouseInteractionEnabled'
SETTING_MOUSE_INTERACTION_ENABLED_DEFAULT = '/defaults/physics/mouseInteractionEnabled'
SETTING_MOUSE_PICKING_FORCE = '/physics/pickingForce'
SETTING_MOUSE_PICKING_FORCE_DEFAULT = '/defaults/physics/pickingForce'
SETTING_MOUSE_PUSH = '/physics/mousePush'
SETTING_MOUSE_PUSH_DEFAULT = '/defaults/physics/mousePush'
SETTING_NUM_EVENT_PUMPS_FOR_TEST_STAGE_SETUP = '/physics/numEventPumpsForTestStageSetup'
SETTING_NUM_EVENT_PUMPS_FOR_TEST_STAGE_SETUP_DEFAULT = '/defaults/physics/numEventPumpsForTestStageSetup'
SETTING_NUM_THREADS = '/persistent/physics/numThreads'
SETTING_NUM_THREADS_DEFAULT = '/defaults/persistent/physics/numThreads'
SETTING_OMNIPVD_ENABLED = '/physics/omniPvdOutputEnabled'
SETTING_OMNIPVD_ENABLED_DEFAULT = '/defaults/physics/omniPvdOutputEnabled'
SETTING_OMNIPVD_IS_OVD_STAGE = '/physics/omniPvdIsOVDStage'
SETTING_OMNIPVD_IS_OVD_STAGE_DEFAULT = '/defaults/physics/omniPvdIsOVDStage'
SETTING_OMNIPVD_IS_RECORDING = '/physics/omniPvdIsRecording'
SETTING_OMNIPVD_IS_RECORDING_DEFAULT = '/defaults/physics/omniPvdIsRecording'
SETTING_OMNIPVD_OVD_RECORDING_DIRECTORY = '/persistent/physics/omniPvdOvdRecordingDirectory'
SETTING_OMNIPVD_OVD_RECORDING_DIRECTORY_DEFAULT = '/defaults/persistent/physics/omniPvdOvdRecordingDirectory'
SETTING_OMNI_HUB_SUPPORTED = '/physics/cooking/omniHubSupported'
SETTING_OMNI_HUB_SUPPORTED_DEFAULT = '/defaults/physics/cooking/omniHubSupported'
SETTING_OUTPUT_VELOCITIES_LOCAL_SPACE = '/physics/outputVelocitiesLocalSpace'
SETTING_OUTPUT_VELOCITIES_LOCAL_SPACE_DEFAULT = '/defaults/physics/outputVelocitiesLocalSpace'
SETTING_OVERRIDE_GPU = '/persistent/physics/overrideGPUSettings'
SETTING_OVERRIDE_GPU_DEFAULT = '/defaults/persistent/physics/overrideGPUSettings'
SETTING_PHYSICS_DEVELOPMENT_MODE = '/physics/developmentMode'
SETTING_PHYSICS_DEVELOPMENT_MODE_DEFAULT = '/defaults/physics/developmentMode'
SETTING_PHYSICS_SCENE_MULTIGPU_MODE = '/physics/sceneMultiGPUMode'
SETTING_PHYSICS_SCENE_MULTIGPU_MODE_DEFAULT = '/defaults/physics/sceneMultiGPUMode'
SETTING_PHYSX_DISPATCHER = '/physics/physxDispatcher'
SETTING_PHYSX_DISPATCHER_DEFAULT = '/defaults/physics/physxDispatcher'
SETTING_PVD_DEBUG = '/persistent/physics/pvdDebug'
SETTING_PVD_DEBUG_DEFAULT = '/defaults/persistent/physics/pvdDebug'
SETTING_PVD_ENABLED = '/persistent/physics/pvdEnabled'
SETTING_PVD_ENABLED_DEFAULT = '/defaults/persistent/physics/pvdEnabled'
SETTING_PVD_IP_ADDRESS = '/persistent/physics/pvdIP'
SETTING_PVD_IP_ADDRESS_DEFAULT = '/defaults/persistent/physics/pvdIP'
SETTING_PVD_MEMORY = '/persistent/physics/pvdMemory'
SETTING_PVD_MEMORY_DEFAULT = '/defaults/persistent/physics/pvdMemory'
SETTING_PVD_OUTPUT_DIRECTORY = '/persistent/physics/pvdOutputDirectory'
SETTING_PVD_OUTPUT_DIRECTORY_DEFAULT = '/defaults/persistent/physics/pvdOutputDirectory'
SETTING_PVD_PROFILE = '/persistent/physics/pvdProfile'
SETTING_PVD_PROFILE_DEFAULT = '/defaults/persistent/physics/pvdProfile'
SETTING_PVD_STREAM_TO_FILE = '/persistent/physics/pvdStreamToFile'
SETTING_PVD_STREAM_TO_FILE_DEFAULT = '/defaults/persistent/physics/pvdStreamToFile'
SETTING_RESET_ON_STOP = '/physics/resetOnStop'
SETTING_RESET_ON_STOP_DEFAULT = '/defaults/physics/resetOnStop'
SETTING_SAVE_COOKED_DATA = '/physics/saveCookedData'
SETTING_SAVE_COOKED_DATA_DEFAULT = '/defaults/physics/saveCookedData'
SETTING_SHOW_COLLISION_GROUPS_WINDOW = '/physics/showCollisionGroupsWindow'
SETTING_SHOW_COLLISION_GROUPS_WINDOW_DEFAULT = '/defaults/physics/showCollisionGroupsWindow'
SETTING_SIMULATE_EMPTY_SCENE = '/physics/simulateEmptyScene'
SETTING_SIMULATE_EMPTY_SCENE_DEFAULT = '/defaults/physics/simulateEmptyScene'
SETTING_SUPPRESS_READBACK = '/physics/suppressReadback'
SETTING_SUPPRESS_READBACK_DEFAULT = '/defaults/physics/suppressReadback'
SETTING_TESTS_ASSETS_PATH = '/physics/testsAssetsPath'
SETTING_TESTS_ASSETS_PATH_DEFAULT = '/defaults/physics/testsAssetsPath'
SETTING_TEST_RUNNER_FILTER = '/persistent/physics/testRunnerFilter'
SETTING_TEST_RUNNER_FILTER_DEFAULT = '/defaults/persistent/physics/testRunnerFilter'
SETTING_TEST_RUNNER_REPEATS = '/physics/testRunnerRepeats'
SETTING_TEST_RUNNER_REPEATS_DEFAULT = '/defaults/physics/testRunnerRepeats'
SETTING_TEST_RUNNER_SELECTION = '/persistent/physics/testRunnerSelection'
SETTING_TEST_RUNNER_SELECTION_DEFAULT = '/defaults/persistent/physics/testRunnerSelection'
SETTING_TEST_RUNNER_STATUS = '/physics/testRunnerStatus'
SETTING_TEST_RUNNER_STATUS_DEFAULT = '/defaults/physics/testRunnerStatus'
SETTING_UJITSO_COLLISION_COOKING = '/physics/cooking/ujitsoCollisionCooking'
SETTING_UJITSO_COLLISION_COOKING_DEFAULT = '/defaults/physics/cooking/ujitsoCollisionCooking'
SETTING_UJITSO_COOKING_DEV_KEY = '/physics/cooking/ujitsoCookingDevKey'
SETTING_UJITSO_COOKING_DEV_KEY_DEFAULT = '/defaults/physics/cooking/ujitsoCookingDevKey'
SETTING_UJITSO_COOKING_MAX_PROCESS_COUNT = '/persistent/physics/cooking/ujitsoCookingMaxProcessCount'
SETTING_UJITSO_COOKING_MAX_PROCESS_COUNT_DEFAULT = '/defaults/persistent/physics/cooking/ujitsoCookingMaxProcessCount'
SETTING_UJITSO_REMOTE_CACHE = '/physics/cooking/ujitsoRemoteCache'
SETTING_UJITSO_REMOTE_CACHE_DEFAULT = '/defaults/physics/cooking/ujitsoRemoteCache'
SETTING_UJITSO_REQUEST_OMNI_HUB = '/physics/cooking/ujitsoRequestOmniHub'
SETTING_UJITSO_REQUEST_OMNI_HUB_DEFAULT = '/defaults/physics/cooking/ujitsoRequestOmniHub'
SETTING_UJITSO_USE_OMNI_HUB = '/physics/cooking/ujitsoUseOmniHub'
SETTING_UJITSO_USE_OMNI_HUB_DEFAULT = '/defaults/physics/cooking/ujitsoUseOmniHub'
SETTING_UPDATE_PARTICLES_TO_USD = '/physics/updateParticlesToUsd'
SETTING_UPDATE_PARTICLES_TO_USD_DEFAULT = '/defaults/physics/updateParticlesToUsd'
SETTING_UPDATE_RESIDUALS_TO_USD = '/physics/updateResidualsToUsd'
SETTING_UPDATE_RESIDUALS_TO_USD_DEFAULT = '/defaults/physics/updateResidualsToUsd'
SETTING_UPDATE_TO_USD = '/physics/updateToUsd'
SETTING_UPDATE_TO_USD_DEFAULT = '/defaults/physics/updateToUsd'
SETTING_UPDATE_VELOCITIES_TO_USD = '/physics/updateVelocitiesToUsd'
SETTING_UPDATE_VELOCITIES_TO_USD_DEFAULT = '/defaults/physics/updateVelocitiesToUsd'
SETTING_USE_ACTIVE_CUDA_CONTEXT = '/persistent/physics/useActiveCudaContext'
SETTING_USE_ACTIVE_CUDA_CONTEXT_DEFAULT = '/defaults/persistent/physics/useActiveCudaContext'
SETTING_USE_LOCAL_MESH_CACHE = '/persistent/physics/useLocalMeshCache'
SETTING_USE_LOCAL_MESH_CACHE_DEFAULT = '/defaults/persistent/physics/useLocalMeshCache'
SETTING_VISUALIZATION_COLLISION_MESH = '/persistent/physics/visualizationCollisionMesh'
SETTING_VISUALIZATION_COLLISION_MESH_DEFAULT = '/defaults/persistent/physics/visualizationCollisionMesh'
SETTING_VISUALIZATION_GAP = '/persistent/physics/visualizationGap'
SETTING_VISUALIZATION_GAP_DEFAULT = '/defaults/persistent/physics/visualizationGap'
SETTING_ZERO_GRAVITY_SPEED = '/physics/zeroGSpeed'
SETTING_ZERO_GRAVITY_SPEED_DEFAULT = '/defaults/physics/zeroGSpeed'
VEHICLE_AUTOMATIC_TRANSMISSION_GEAR_VALUE = 255
VEHICLE_DRIVE_STATE_ACCELERATOR = 0
VEHICLE_DRIVE_STATE_AUTOBOX_TIME_SINCE_LAST_SHIFT = 8
VEHICLE_DRIVE_STATE_AUTOMATIC_TRANSMISSION = 10
VEHICLE_DRIVE_STATE_BRAKE0 = 1
VEHICLE_DRIVE_STATE_BRAKE1 = 2
VEHICLE_DRIVE_STATE_CLUTCH = 4
VEHICLE_DRIVE_STATE_CURRENT_GEAR = 5
VEHICLE_DRIVE_STATE_ENGINE_ROTATION_SPEED = 9
VEHICLE_DRIVE_STATE_GEAR_SWITCH_TIME = 7
VEHICLE_DRIVE_STATE_STEER = 3
VEHICLE_DRIVE_STATE_TARGET_GEAR = 6
VEHICLE_WHEEL_STATE_GROUND_ACTOR = 6
VEHICLE_WHEEL_STATE_GROUND_HIT_POSITION = 9
VEHICLE_WHEEL_STATE_GROUND_MATERIAL = 8
VEHICLE_WHEEL_STATE_GROUND_PLANE = 5
VEHICLE_WHEEL_STATE_GROUND_SHAPE = 7
VEHICLE_WHEEL_STATE_IS_ON_GROUND = 18
VEHICLE_WHEEL_STATE_LOCAL_POSE_POSITION = 0
VEHICLE_WHEEL_STATE_LOCAL_POSE_QUATERNION = 1
VEHICLE_WHEEL_STATE_ROTATION_ANGLE = 3
VEHICLE_WHEEL_STATE_ROTATION_SPEED = 2
VEHICLE_WHEEL_STATE_STEER_ANGLE = 4
VEHICLE_WHEEL_STATE_SUSPENSION_FORCE = 11
VEHICLE_WHEEL_STATE_SUSPENSION_JOUNCE = 10
VEHICLE_WHEEL_STATE_TIRE_FORCE = 17
VEHICLE_WHEEL_STATE_TIRE_FRICTION = 12
VEHICLE_WHEEL_STATE_TIRE_LATERAL_DIRECTION = 14
VEHICLE_WHEEL_STATE_TIRE_LATERAL_SLIP = 16
VEHICLE_WHEEL_STATE_TIRE_LONGITUDINAL_DIRECTION = 13
VEHICLE_WHEEL_STATE_TIRE_LONGITUDINAL_SLIP = 15
