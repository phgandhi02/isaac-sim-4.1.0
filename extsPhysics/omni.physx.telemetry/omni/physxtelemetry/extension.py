import carb
import carb.settings
import usdrt

import omni.ext
import omni.kit
import omni.timeline

from .physx_telemetry import OmniPhysXTelemetry

SETTING_PHYSICS_TELEMETRY_ENABLED = "/persistent/physics/enableTelemetry"

try:
    from omni.physx.scripts.utils import safe_import_tests
    safe_import_tests("omni.physxtelemetry.tests")
except Exception as e:
    print(f'safe_import_tests("omni.physxtelemetry.tests") FAILED with exception: {e}')


class OmniPhysXTelemetryExtension(omni.ext.IExt):

    def __init__(self) -> None:
        super().__init__()
        self._zerog_enabled_sub = None
        self._omni_physx_zerog_ext_enabled_sub = None

    def on_startup(self, ext_id) -> None:
        # Read whatever persistent setting is set and store it in a class attribute
        _settings = carb.settings.get_settings()
        OmniPhysXTelemetry.telemetry_is_enabled = _settings.get_as_bool(SETTING_PHYSICS_TELEMETRY_ENABLED)

        if OmniPhysXTelemetry.telemetry_is_enabled:
            self._stage_event_sub = omni.usd.get_context().get_stage_event_stream().create_subscription_to_pop(
                self._on_stage_event, name="omni.physx.telemetry._on_stage_event"
            )

            # Track ZG extension activation so we can put a callback for ZG mode enablement
            manager = omni.kit.app.get_app().get_extension_manager()
            self._omni_physx_zerog_ext_enabled_sub = manager.subscribe_to_extension_enable(self._on_omni_physx_zerog_ext_enabled, None, ext_name="omni.physx.zerogravity", hook_name="on_omni_physx_zerogravity_enabled")

    def on_shutdown(self) -> None:
        self._stage_event_sub = None
        self._omni_physx_zerog_ext_enabled_sub = None
        if self._zerog_enabled_sub is not None:
            try:
                import omni.physxzerogravity

                if omni.physxzerogravity is not None:
                    omni.physxzerogravity.unsubscribe_from_zerog_enabled(self._zerog_enabled_sub)
            except ImportError:
                pass
            self._zerog_enabled_sub = None
        self._omni_physx_fabric_ext_enabled_sub = None

    def _on_zerog_enabled(self):
        OmniPhysXTelemetry.log_omni_physx_zerog_activated()

        # Also unregister this callback - we're no longer interested in it as long as ZG is not unloaded and reloaded
        # again (we're done for this session)
        try:
            import omni.physxzerogravity

            if omni.physxzerogravity is not None and self._zerog_enabled_sub is not None:
                omni.physxzerogravity.unsubscribe_from_zerog_enabled(self._zerog_enabled_sub)
        except ImportError:
            pass
        self._zerog_enabled_sub = None

    def _on_omni_physx_zerog_ext_enabled(self, ext_id: str):
        try:
            import omni.physxzerogravity

            if omni.physxzerogravity is not None:
                self._zerog_enabled_sub = omni.physxzerogravity.subscribe_to_zerog_enabled(self._on_zerog_enabled)
        except ImportError:
            carb.log_warn("Could not import omni.physx.zerogravity - telemetry will not log ZG enablement")

    def _on_stage_event(self, evt):  # Track stage open events so we can log which physics APIs are used
        if evt.type == int(omni.usd.StageEventType.OPENED):

            usdrtStage = usdrt.Usd.Stage.Attach(omni.usd.get_context().get_stage_id())
            if usdrtStage is None:
                carb.log_warn("Failed to get usdrt stage - telemetry will not log physics API usage")

            # A list of APIs that we want to log at stage load in telemetry to see which ones users are using
            apis_to_log = {
                # USD schemas
                "PhysicsArticulationRootAPI",
                "PhysicsCollisionAPI",
                "PhysicsCollisionGroup",
                "PhysicsDistanceJoint",
                "PhysicsDriveAPI",
                "PhysicsFilteredPairAPI",
                "PhysicsFixedJoint",
                "PhysicsJoint",
                "PhysicsLimitAPI",
                "PhysicsMassAPI",
                "PhysicsMaterialAPI",
                "PhysicsPrismaticJoint",
                "PhysicsRevoluteJoint",
                "PhysicsRigidBodyAPI",
                "PhysicsScene",
                "PhysicsSphericalJoint",
                # Physx schemas
                "PhysxAckermannSteeringAPI",
                "PhysxAnisotropyAPI",
                "PhysxArticulationAPI",
                "PhysxAutoAttachmentAPI",
                "PhysxAutoParticleClothAPI",
                "PhysxBrakesAPI",
                "PhysxCameraAPI",
                "PhysxCameraDroneAPI",
                "PhysxCameraFollowAPI",
                "PhysxCameraFollowLookAPI",
                "PhysxCameraFollowVelocityAPI",
                "PhysxCollisionAPI",
                "PhysxCookedDataAPI",
                "PhysxDeformableAPI",
                "PhysxDeformableBodyAPI",
                "PhysxDeformableBodyMaterialAPI",
                "PhysxDeformableSurfaceAPI",
                "PhysxDeformableSurfaceMaterialAPI",
                "PhysxDiffuseParticlesAPI",
                "PhysxDistanceJointAPI",
                "PhysxDriveBasicAPI",
                "PhysxDriveStandardAPI",
                "PhysxEngineAPI",
                "PhysxForceAPI",
                "PhysxGearJoint",
                "PhysxGearsAPI",
                "PhysxHairAPI",
                "PhysxHairMaterialAPI",
                "PhysxIsosurfaceAPI",
                "PhysxMaterialAPI",
                "PhysxMeshMergeCollisionAPI",
                "PhysxParticleAPI",
                "PhysxParticleAnisotropyAPI",
                "PhysxParticleClothAPI",
                "PhysxParticleIsosurfaceAPI",
                "PhysxParticleSamplingAPI",
                "PhysxParticleSetAPI",
                "PhysxPBDMaterialAPI",
                "PhysxPhysicsAttachment",
                "PhysxPhysicsInstancer",
                "PhysxPhysicsJointInstancer",
                "PhysxPhysicsRackAndPinionJoint",
                "PhysxPhysicsTireFrictionTable",
                "PhysxPhysicsTriggerAPI",
                "PhysxPhysicsTriggerStateAPI",
                "PhysxPhysicsTendonAxisAPI",
                "PhysxPhysicsTendonAxisRootAPI",
                "PhysxPhysicsTendonAttachmentAPI",
                "PhysxPhysicsTendonAttachmentRootAPI",
                "PhysxPhysicsTendonAttachmentLeafAPI",
                "PhysxPhysicsVehicleAPI",
                "PhysxPhysicsVehicleAutoGearBoxAPI",
                "PhysxPhysicsVehicleClutchAPI",
                "PhysxPhysicsVehicleControllerAPI",
                "PhysxPhysicsVehicleDriveBasicAPI",
                "PhysxPhysicsVehicleDriveStandardAPI",
                "PhysxPhysicsVehicleMultiWheelDifferentialAPI",
                "PhysxPhysicsVehicleTankControllerAPI",
                "PhysxPhysicsVehicleTankDifferentialAPI",
                "PhysxPhysicsVehicleWheelControllerAPI",
                "PhysxPhysicsVehicleNonlinearCommandResponseAPI",
                "PhysxRackAndPinionJoint",
                "PhysxRigidBodyAPI",
                "PhysxSceneAPI",
                "PhysxSceneQuasistaticAPI",
                "PhysxSphereFillCollisionAPI",
                "PhysxSteeringAPI",
                "PhysxSurfaceMaterialAPI",
                "PhysxTankControllerAPI",
                "PhysxTendonAttachmentAPI",
                "PhysxTendonAttachmentLeafAPI",
                "PhysxTendonAttachmentRootAPI",
                "PhysxTendonAxisAPI",
                "PhysxTireAPI",
                "PhysxTriggerAPI",
                "PhysxTriggerStateAPI",
                "PhysxVehicleAPI",
                "PhysxVehicleAckermannSteeringAPI",
                "PhysxVehicleAutoGearBoxAPI",
                "PhysxVehicleBrakesAPI",
                "PhysxVehicleClutchAPI",
                "PhysxVehicleControllerAPI",
                "PhysxVehicleDriveBasicAPI",
                "PhysxVehicleDriveStandardAPI",
                "PhysxVehicleEngineAPI",
                "PhysxVehicleGearsAPI",
                "PhysxVehicleMultiWheelDifferentialAPI",
                "PhysxVehicleNonlinearCommandResponseAPI",
                "PhysxVehicleSteeringAPI",
                "PhysxVehicleSuspensionAPI",
                "PhysxVehicleSuspensionComplianceAPI",
                "PhysxVehicleTankControllerAPI",
                "PhysxVehicleTankDifferentialAPI",
                "PhysxVehicleTireAPI",
                "PhysxVehicleTireFrictionTable",
                "PhysxVehicleWheelAPI",
                "PhysxVehicleWheelAttachmentAPI",
                "PhysxVehicleWheelControllerAPI",

                # additional attribute
                "UsdPhysics.MeshCollisionAPI:physics:approximation"
            }

            apis_found_on_prims = set()

            for api_name in apis_to_log:
                prim_paths = usdrtStage.GetPrimsWithAppliedAPIName(api_name)
                if len(prim_paths) > 0:
                    apis_found_on_prims.add(api_name)

            if len(apis_found_on_prims) > 0:
                apis_found_on_prims = ";".join(apis_found_on_prims)
                OmniPhysXTelemetry.log_omni_physx_stage_apis(apis_found_on_prims)
