# Changelog

Omni PhysX Bundle

## [106.0.20] - 2024-07-12

Version 2bda2f465f9c8a0ad1b7135e19fd8631379e6fd5

### Core

**General**

    - Fixed

        - Changes to joint maximum velocity and friction coefficient did not update for GPU simulation



## [106.0.19] - 2024-07-03

Version 7231900e93130ba1bb6c9d11684bd4829a872d4c

### Core

**General**

    - Fixed

        - IPhysxReplicator is now supporting articulation mimic joints too (PhysxMimicJointAPI)

**Python Bindings API**

    - Added

        - Added GPU buffer usage statistics to simulation statistics interface. Interface minor version increased to 1.1

## [106.0.18] - 2024-06-28

Version 5e1ad7159b18f7fc33b5d2edec8ab5572eae80a1 

### Core

**General**

    - Added

        - Add omni.physx.asset_validator extension with JointState API validation

    - Changed

        - Removed Kapla Tower Demo Scene, and moved Joint State Demo Scene to Articulations Section

    - Fixed

        - A crash in the GPU broadphase has been fixed.
        - Performance issue with collision mesh merging initialization
        - Fixed Invalid transforms appearing when adding a large articulation to a scene that already contained articulations
        - Convert surface angular velocity to radians during property update
        - Do not create temporary physics scene when write to USD is disabled
        - Physics inspector should avoid fixing articulations linked to other articulations
        - Fixed Particle Cloth attachment not working with jointed articulation link

**TensorAPI**

    - Added

        - The RigidBodyView class allows retrieving rigid-body accelerations with get_accelerations
        - The GPU ArticulationView class allows updating kinematics with the update_articulations_kinematic API
        - The GPU ArticulationView class makes use of the CPU API an allow setting the COM via set_coms
        - The RigidContactView allows for reading friction and contact forces
        - The RigidContactView, RigidBodyView and ArticulationView support list expression tocreateg the objects
        - Added a new apply_forces_and_torques_at_position API to apply external forces to articulation links
        - The SimulationView Python client can invalidate the simulation view via the invalidate API

    - Fixed

        - Fixed the application crash when stoping the timeline, closing an existing stage and creating a new one 
        - Fixed the bug with material properties in the RigidB?odyView

### UI

**General**

    - Changed

        - When adding a mimic joint, there will be no option to define the instance name anymore as the UI is targeted for prismatic or revolute joints (where the instance name is ignored). Furthermore, the Reference Joint Axis dropdown field in the property window of a mimic joint will be disabled if the referenced joint is a prismatic or revolute joint.

## [106.0.17] - 2024-06-03
Version bump.

## [106.0.16] - 2024-05-24

Version f22ca04a18ff3e557d4cf9e00c462644ce57e7bd

### Core

**General**

    - Fixed

        - Character controller keyboard handling fix

## [106.0.15] - 2024-05-21

Version 987aa39841d8e80592876cc2960af2cbda41a4c5

### Core

**General**

    - Fixed

        - Reduce FPS spikes due to periodically writing cooking related persistent settings to disk
        - Print a warning (only once) when cooking disk cache cannot be opened as it's locked by another kit process

## [106.0.14] - 2024-05-17

Version 50cae035afe037ad95c9b1282f15ccd2cd10c710 

### Core

**General**

    - Changed

        - Articulation transformation updates are now propagated to the root links, while the regular links updates are ignored

    - Fixed

        - Fix clash detection viewport errors on clashes between instanced prims
        - Fix Physics Inspector options menu items not updated properly

    - Deprecated

        - Deprecations of particle cloth, deformable body and deformable surface types and functions in the following files: 
IPhysx.h, IPhysxSettings.h, IPhysxUsdLoad.h, PhysxUsd.h, deformableUtils.py, particleUtils.py.
The interfaces and implementations will be replaced by a new deformable body system.

**USD Schemas**

    - Deprecated

        - The improvePatchFriction attribute of the PhysxMaterialAPI schema has been deprecated. At a future point, the simulation will always behave as if this attribute was set to true and no longer support the legacy behavior (triggered by setting the attribute to false). At that point this attribute will be removed. Until then, everyone is highly encouraged to always set this attribute to true to adapt to the corresponding friction behavior (note that true is currently the default value).
        - The friction type ""twoDirectional"" of the PhysxSceneAPI schema attribute ""frictionType"" has been deprecated. Please use the friction type ""patch"" instead.
        - Deprecated PhysxSchema.TetrahedralMesh, physxPBDMaterial:lift, physxPBDMaterial:drag, PhysxParticleClothAPI, PhysxAutoParticleClothAPI, PhysxDeformableBodyMaterialAPI, PhysxDeformableSurfaceMaterialAPI, PhysxDeformableAPI, PhysxDeformableBodyAPI, PhysxDeformableSurfaceAPI, PhysxPhysicsAttachment, PhysxAutoAttachmentAPI
will all be replaced by a new deformable schema in a future release.

## [106.0.13] - 2024-05-13

Version 1e7925bff0f06801875dead5b4fd4ada28bb6c91 

### Core

**General**

    - Added

        - IPhysxStatistics (get_physx_statistics_interface) interface added for PhysX simulation statistics access

### UI

**General**

    - Added

        - Physics Authoring Toolbar: new option: Allow Prim to Parent Search for Rigid Body - If activated and the selected prim lacks a rigid body, the manipulator can traverse up the hierarchy to find a parent with a rigid body, allowing for manipulation through physics forces.

## [106.0.12] - 2024-05-03

Version b2768ef22ffa3d6aa68589d0c2bbff89bb92768f

### UI

**General**

    - Fixed

        - Manually closing the Simulation Data Visualizer window now properly toggles off the setting

## [106.0.11] - 2024-04-23

Version bec05583ade3c285ae1dec8caf159a705f957e81

### Core

**General**

    - Fixed

        - Backwards compatibility check might not have detected outdated USD asset.
        - UsdPhysicsJoint body relationship checks if prim exists in a stage, otherwise joint is not created. 

**USD Schemas**

    - Changed

        - Default PhysxScene bounce threshold is set to 0 from 2 m/s

### Force Fields

**General**

    - Added

        - Allow Force Field forces and torques to be applied to articulation links.

### UI

**General**

    - Fixed

        - Collider Debug Visualization: Point Instancer: Fixed support for scaled instances.

## [106.0.10] - 2024-04-16

Version d1c87401ca780c1bd8807de15be06ff3dbefef2b 

### Core

**General**

    - Added

        - Added support for friction anchors to contact reports, these data are returned through new interfaces with full contact info
        - Added possibility to obtain residual values on the physics scene, articulation roots and joints to monitor solver convergence

    - Fixed

        - Transformation reset for referenced assets

**USD Schemas**

    - Added

        - The PhysxMimicJointAPI schema has been introduced to connect articulation joints using a gearing ratio and an offset.

    - Removed

        - Removed PhysxSchema PhysxArticulationForceSensorAPI

### UI

**General**

    - Added

        - Added new Simulation Data Visualizer window, accessible through the ""eye"" viewport menu.

    - Fixed

        - Fixed crash with joint was created through a python script and moved with gizmo

## [106.0.9] - 2024-04-02

### Core

**General**

    - Fixed

        - Cleaned up IStageUpdate deprecation warnings by replacing it with the newer interface calls

**USD Schemas**

    - Added

        - Extend the physxMaterial schema for compliant contacts by exposing the dampingCombineMode and compliantContactAccelerationSpring settings.

### UI

**General**

    - Added

        - Allow Physics Inspector to fix articulation base and commit or discard authoring simulation

    - Changed

        - Replace mainpulators with fabric enabled manipulators

## [106.0.8] - 2024-03-18

### Core

**General**

    - Fixed

        - omni.physx.stageupdate dependencies
        - Clash when articulations were deleted while using PhysX Inspector

## [106.0.7] - 2024-03-15

### UI

**General**

    - Added

        - Simulation object monitor

    - Changed

        - PhysX Inspector updated and improved

## [106.0.6] - 2024-02-29

### Core

**General**

    - Changed

      - PhysX Replicator optimized
      - PhysX Fabric output optimized

## [106.0.5] - 2024-02-26
 - Kit SDK update

## [106.0.4] - 2024-02-15
 - Kit SDK update

## [106.0.3] - 2024-01-25
 - Kit SDK update

## [106.0.2] - 2024-01-18

 - Kit SDK update

## [106.0.1] - 2024-01-16

 - Kit SDK update
 - Added extensions publishing

## [106.0.0] - 2024-01-10


### Core

**General**

    - Added

        - Added new PhysxScene attribute ""enableExternalForcesEveryIteration"" which can help with TGS solver convergence. Currently only implemented for PhysxParticleSystem gravity application.
        - Add support for articulation links for attachments
        - Use either /physics/sceneMultiGPUMode or Preferences/Physics to turn on a mode where every PhysicsScene is simulated on a separate GPU assigned in a rotating fashion.
        - Physics scene quasistatic mode added

    - Changed

        - USD Geom Subsets (face materials) are used in mesh hash computation for cooking. Invalidates hashes for cooked data generated with previous versions of omni.physx
        - Extended valid domain of physxPBDMaterial:cflCoefficient to [0,inf]
        - Update scene query OmniGraph nodes to use target attribute types (path tokens have been deprecated).

    - Fixed

        - Performance improvements for USD and Fabric transformation update
        - Backward compatibility now removes obsolete physxCollisionCustomGeometry attributes
        - Disable autotargeting session layer for physics properties on OmniJoint prims
        - CharacterController node's Done pin is executed after either activation or deactivation is finished
        - Fix property query tests
        - Reduce physics cooking blocking of main thread on very large meshes and stages
        - Physics cooking will reuse active tasks when matching mesh crc
        - Fix crash with async simulation stepping
        - Fix crash with particle cloth parsing
        - Update Trigger and Contact OmniGraph nodes inputs using the new target type instead of deprecated bundle type for USD Path inputs
        - Fix issue with PhysxPropertyQuery return wrong data for instanced prims and reading garbage data for convex decompositions used in Simready assets (affects Mass Visualization Gizmo)
        - ZeroGravity: crash fix - no longer delegating to ZG correct resync
        - ZeroGravity causing crash at startup
        - Fix crash when updating deformable simulation positions/velocties at runtime with inconsistent element counts
        - Fix crash when updating kinematic deformable mesh points with inconsistent element counts
        - Physics demos have wrong hitbox colors
        - Defensive programming: making sure ZG sweep thread is disabled immediately at ZG shutdown
        - Fix crashes when resampling a mesh with the particle sampler if the point count of the target particle prim has been changed externally.
        - More USD parsing checks for deformables and particles
        - Wrong type hints in physicsUtils.set_or_add_orient_op as reported by user in https://forums.developer.nvidia.com/t/wrong-type-hints-in-helper-function/267542
        - Fix a crash when PhysX errors are reported during shutdown
        - Fix crash in particle sampling when cooking is not initialized
        - Fix crash when cooking deformable tet meshes for kinematic deformables in the presence of near degenerate triangles in the original mesh.
        - Fix inflatable particle cloth crash issue when pressure is 0
        - CharacterController headless mode crash
        - Refactor physics attachment and add support for undo. The refactor also addresses the multiple crashes and bugs filed for attachments.
        - The maxForce attribute of the PhysicsDriveAPI schema was treated as an impulse instead of a force, if the joint was not part of an articulation system.
        - Crash in GPU island manager

**USD Schemas**

    - Added

        - PhysxSchema.PhysxMeshMergeCollisionAPI that is able to merge meshes into one collision based on collection definition.

**Python Bindings API**

    - Added

        - Add support for box, mesh and shape sweep scene queries.

### Vehicles

**General**

    - Changed

        - Some vehicle samples did automatically start playing after the sample was loaded. To be more consistent with other samples, this is not the case any longer and the play button has to be pressed to get these samples started.

## [105.2.1] - 2024-01-16
Version 72282ae72edbbdff91db25532d75c56338506609 

 - Kit SDK update
 - Added extensions publishing

## [105.2.0] - 2024-01-15

Version 8aeff4cd0c378eb8f436119a3536c72553268019 

### Core

**General**

    - Added

        - Added new PhysxScene attribute ""enableExternalForcesEveryIteration"" which can help with TGS solver convergence. Currently only implemented for PhysxParticleSystem gravity application.
        - Add support for articulation links for attachments

    - Changed

        - USD Geom Subsets (face materials) are used in mesh hash computation for cooking. Invalidates hashes for cooked data generated with previous versions of omni.physx
        - Extended valid domain of physxPBDMaterial:cflCoefficient to [0,inf]
        - Update scene query OmniGraph nodes to use target attribute types (path tokens have been deprecated).

    - Fixed

        - Performance improvements for USD and Fabric transformation update
        - Backward compatibility now removes obsolete physxCollisionCustomGeometry attributes
        - Disable autotargeting session layer for physics properties on OmniJoint prims
        - CharacterController node's Done pin is executed after either activation or deactivation is finished
        - Fix property query tests
        - Reduce physics cooking blocking of main thread on very large meshes and stages
        - Physics cooking will reuse active tasks when matching mesh crc
        - Fix crash with async simulation stepping
        - Fix crash with particle cloth parsing
        - Update Trigger and Contact OmniGraph nodes inputs using the new target type instead of deprecated bundle type for USD Path inputs
        - Fix issue with PhysxPropertyQuery return wrong data for instanced prims and reading garbage data for convex decompositions used in Simready assets (affects Mass Visualization Gizmo)
        - ZeroGravity: crash fix - no longer delegating to ZG correct resync
        - ZeroGravity causing crash at startup
        - Fix crash when updating deformable simulation positions/velocties at runtime with inconsistent element counts
        - Fix crash when updating kinematic deformable mesh points with inconsistent element counts
        - Physics demos have wrong hitbox colors
        - Defensive programming: making sure ZG sweep thread is disabled immediately at ZG shutdown
        - Fix crashes when resampling a mesh with the particle sampler if the point count of the target particle prim has been changed externally.
        - More USD parsing checks for deformables and particles
        - Wrong type hints in physicsUtils.set_or_add_orient_op as reported by user in https://forums.developer.nvidia.com/t/wrong-type-hints-in-helper-function/267542
        - Fix a crash when PhysX errors are reported during shutdown
        - Fix crash in particle sampling when cooking is not initialized
        - Fix crash when cooking deformable tet meshes for kinematic deformables in the presence of near degenerate triangles in the original mesh.
        - Fix inflatable particle cloth crash issue when pressure is 0
        - CharacterController headless mode crash
        - Refactor physics attachment and add support for undo. The refactor also addresses the multiple crashes and bugs filed for attachments.
        - The maxForce attribute of the PhysicsDriveAPI schema was treated as an impulse instead of a force, if the joint was not part of an articulation system.

**USD Schemas**

    - Added

        - PhysxSchema.PhysxMeshMergeCollisionAPI that is able to merge meshes into one collision based on collection definition.

**Python Bindings API**

    - Added

        - Add support for box, mesh and shape sweep scene queries.

### Vehicles

**General**

    - Changed

        - Some vehicle samples did automatically start playing after the sample was loaded. To be more consistent with other samples, this is not the case any longer and the play button has to be pressed to get these samples started.


## [105.1.11] - 2023-01-12

Version a836d8ed45fae75202dbb77aecfa7cb3cfa7e3df 

### Core

**General**

    - Changed

        - The physxScene:gpuTempBufferCapacity parameter on the physics scene is now interpreted as a user-provided initial size, and will resize automatically if more memory is needed.

    - Fixed

        - Possible crash when triggers were deleted
        - Possible crash when creating fixed tendons
        - Crash when a PhysX SDK scene failed to be created
        - Articulation arggregate number of shapes computed correctly
        - Fix Inspector incorrect rounding of prismatic joint values ignoring meters to units
        - A bug that led to a memory corruption in the GPU solver when using joints connected to both rigid bodies and articulations has been fixed.
        - Issues with collision detection on articulations have been fixed that could have led to missed or phantom collisions.
        - Spherical articulation joint limit behavior is fixed for links with nonidentity principal axis transform
        - A bug causing slower convergence and potentially unstable collision responses between rigids and articulations.
        - Articulation joint velocity limits are respected when articulation joint drives are configured to push past the limit.
        - Particle - Convex Shape collisions failing with spread out particles.
        - D6 Joint angular, Revolute Joint and Spherical Joint Limits close to 180deg behave correctly for TGS solver and GPU dynamics.
        - Sphere-Triangle Mesh collisions for GPU Dynamics have been improved
        - Cylinder and cone simulated with custom/analytic geometry did not respect the restOffset parameter
        - Small improvements to SDF collisions, especially when objects with wildly different size collide
        - Fix crashes when resampling a mesh with the particle sampler if the point count of the target particle prim has been changed externally.
        - Fix crash in particle sampling when cooking is not initialized
        - More USD parsing checks for deformables and particles
        - Fix crash when updating deformable simulation positions/velocties at runtime with inconsistent element counts
        - Fix crash when updating kinematic deformable mesh points with inconsistent element counts
        - Fix crash when cooking deformable tet meshes for kinematic deformables in the presence of near degenerate triangles in the original mesh.

**Fabric**

    - Fixed

        - Fix crash when releasing fabric buffers due to stale cuda context

## [105.1.10] - 2023-10-19

Version ce837e9cd730d260af3968409c69fbdd98242273 

### Core

**General**

   - Added

      - License for omni.usd.schema.physx

## [105.1.9] - 2023-09-23

Version abcecd407bfa5cd0f2c2f5a749faf0eaaba4aaf9

### Core

**General**

    - Fixed

        - Physics demos have wrong hitbox colors
        - Defensive programming: making sure ZG sweep thread is disabled immediately at ZG shutdown
        - PhysX replicator fix

## [105.1.8] - 2023-09-20

Version a2ccefd7ca8f418b1e781077c763868b0e3c292c

### Core

**General**

    - Fixed

        - Filtered pairs removal crash.
        - Issue with point instancer orientation update.

### Vehicles

**General**

    - Fixed

        - Crash when setting rigidBodyEnabled to false on a body that is part of a vehicle. This is an illegal operation and will now be ignored and an error message will get sent.
        - Undefined behavior when setting kinematicEnabled to true on a body that is part of an enabled vehicle. This is an illegal operation and will now result in the vehicle being disabled and an error message will get sent.

### UI

**General**

    - Fixed

        - Joint visualization respects parent visibility toggle.

## [105.1.7] - 2023-09-11

Version d27e1470097c3307bdcc05c491e0f01492aa2e5a

### Core

**General**

    - Fixed

        - Update Trigger and Contact OmniGraph nodes inputs using the new target type instead of deprecated bundle type for USD Path inputs
        - Get App window and mapping set nullptr checks
        - Fix prismatic joint alignment utility
        - Fix issue with PhysxPropertyQuery return wrong data for instanced prims and reading garbage data for convex decompositions used in Simready assets (affects Mass Visualization Gizmo)


## [105.1.6] - 2023-09-07

Version 5cef3dbabc05dff698edbd759aa7fe1dd65d3b76

### Core

**General**

    - Fixed

        - Fixed crash with particle creation
        - Crash with material delete for convex decomposition primitives
        - Mass computation for rigid bodies without specified mass
        - Fix property query tests
        - Reduce physics cooking blocking of main thread on very large meshes and stages
        - Physics cooking will reuse active tasks when matching mesh crc
        - Fix crash with async simulation stepping
        - Cleanup previous instance on Character controller OG node double activation
        - CharacterController OG node's Done pin is executed after either activation or deactivation is finished

### UI

**General**

    - Fixed

        - Zero Gravity Resync for instanceable change
        - Instanced meshes cooking caching is correctly triggered

## [105.1.5] - 2023-08-21

Version 77b5fe073d192801971f242edfbaf5415a2071ff 

### Core

**General**

    - Fixed

        - Demos nucleus path.

## [105.1.4] - 2023-08-14

Version 7d5ef01ed1f233dcb424a605d91c083591f93632

### Core

**General**

    - Fixed

        - A bug that led to incorrect, missing and phantom collisions in convex-trianglemesh collisions has been fixed.
        
## [105.1.3] - 2023-08-07

Version 72d4fd2f31646d1260cbe39b38784b2d09d07110

### Core

**General**

    - Fixed

        - Clear custom joints billboard instances when simulation play starts
        - Incorrectly reported articulation object on fixed joint path
        - A bug that led to incorrect and nondeterministic behaviour for trianglemesh-trianglemesh collisions on GPU has been fixed.
        - Fix potential crash on OmniGraph Immediate nodes with empty meshes or meshes failing to cook. Improve performance on very large loads for Compute Mesh Intersecting Faces node.


## [105.1.2] - 2023-08-01

Version 971d2c252956dfd1d6aaa33b3c85289baace7a60

### Core

**General**

    - Changed

        - Improved stage load time by switching stage traversal to USDRT
        - Improved synchonization between simulation and mouse physics interaction for enhanced precision and responsiveness.

    - Fixed

        - Safety checks for multi material, SDF approximation correctly uses just one material
        - Fixes a crash when moving a prim with PhysxParticleSamplingAPI to a new path while simulating.
        - A bug that led to incorrect and nondeterministic behaviour for convex-trianglemesh, convex-heightfield, sphere-trianglemesh, capsule-trianglemesh, sphere-heightfield and capsule-heightfield collisions on GPU has been fixed.

## [105.1.1] - 2023-07-27

Version 89299bc58e60b0f7cf130575a8143259606fc391 

### Core

**General**

    - Changed

        - Default contactOffset computation for convex decomposition shapes

    - Fixed

        - Only half the expected friction force was applied in certain articulation scenarios when using physxScene:solverType=""TGS"", physxScene:frictionType=""patch"", physxMaterial:improvePatchFriction=true and running on CPU.        
        - Joint align for misalligned joints when right clicking on joint poses

**Zero Gravity**

    - Fixed

      - Zero Gravity crash with simulation node unregister

## [105.1.0] - 2023-07-19

Version 0e189763accd3b7f1cd1e25c617f58cdc7e507a6

### Core

**General**

    - Added

        - Demos can be filtered through tags.
        - Added support for SDF collision approximation to Physics Authoring Toolbar 
        - Integrate SRE (SimReady assets Explorer) drag and drop with ZeroGravity opt-in integration for easy and physically correct placement of assets.
        - Toolbar have new option to not force collider type change when changing static<>dynamic
        - Support for attachment mask shapes. Optionally defines a volume in which attachment points are generated, using UsdGeomSphere, UsdGeomCube and UsdGeomCapsule. Also includes better support for world space attachments which can now be specified using mask shapes.
        - Support for hair attachments, including ability to specify how many hair root vertices should be minimally attached per hair strand.

    - Changed

        - Merge OmniGraph Physics Trigger Enter and Leave nodes in a single one with multiple execution pins
        - The TGS solver on CPU and GPU now computes the number of position and velocity iteration according to the requested numbers by the actors in each island, matching the behavior of the PGS solver. Previously TGS velocity iterations in excess of 4 were silently converted to position iterations. To preserve the old behavior any actor requesting more than 4 velocity iterations should convert excess velocity iteration counts to position iteration counts, e.g., formerly 10 position and 10 velocity iterations should become 16 position and 4 velocity iterations.

    - Fixed

        - added 'sdf' as a valid approximation param for omni.physx.utils.setCollider
        - Entering zeroG creates physics scene that is visible in the stage window
        - Create ground plane with zeroG on issues PhysX SDK errors
        - Starting application and enabling zeroG does not enter sweep mode
        - The torsionalPatchRadius parameter of PhysxCollisionAPI was not working as expected with GPU simulation.
        - Fixed crash when adding particle cloth component to meshes with non-referenced vertices 
        - Error when Physics Authoring toolbar feature ""Remove Physics"" was used and prims had certain vehicle API schemas applied.

**USD Schemas**

    - Deprecated

        - The friction type ""oneDirectional"" has been deprecated (see PhysxSceneAPI, attribute frictionType).

**Python Bindings API**

    - Added

        - Added point instancer variants of several simulation interface functions.

**C++ API**

    - Added

        - Added point instancer variants of several simulation interface functions.

### Vehicles

**General**

    - Added

        - Vehicle creation wizard: when selecting a wheel attachment prim, there will be a check to avoid assigning the same prim to multiple wheels.

    - Changed

        - The vehicle creation wizard does now only allow Xform prims to be chosen as vehicle prim.

    - Fixed

        - The vehicle creation wizard did not take vehicle prim scale into account for created wheel geometry collision and render prims.
        - The vehicle creation wizard was not computing correct center of mass or wheel attachment positions if there was an offset between vehicle prim and bounding box center of scanned chassis prims (and if the wheel attachment prims were not defined by the user).

**USD Schemas**

    - Added

        - The option to limit the suspension expansion velocity has been added (see attribute limitSuspensionExpansionVelocity of PhysxVehicleAPI).
        - The response to brake, steer and accelerator commands can now be described through nonlinear graphs (see the new API schema PhysxVehicleNonlinearCommandResponseAPI for details).

    - Changed

        - PhysxVehicleTankDifferentialAPI: it is now legal to define tank tracks with 1 or 0 wheels (entries in attribute numberOfWheelsPerTrack)

**Python Bindings API**

    - Removed

        - The following deprecated APIs have been removed: IPhysxVehicle.set_wheel_rotation_speed, IPhysxVehicle.set_wheel_rotation_angle, IPhysxVehicle.get_wheel_query_results, IPhysxVehicle.get_wheel_data, IPhysxVehicle.get_drive_data, IPhysxVehicle.compute_wheel_simulation_transforms. The following methods can be used instead: IPhysx.set_wheel_rotation_speed, IPhysx.set_wheel_rotation_angle, IPhysx.get_wheel_state, IPhysx.get_vehicle_drive_state of the omni.physx extension and IPhysxVehicle.compute_suspension_frame_transforms of the omni.physx.vehicle extension.

**C++ API**

    - Removed

        - The following deprecated APIs have been removed: IPhysxVehicle.setWheelRotationSpeed, IPhysxVehicle.setWheelRotationAngle, struct WheelQueryResults, struct WheelDynData, struct DriveDynData, IPhysxVehicle.getWheelQueryResults, IPhysxVehicle.getWheelDynData, IPhysxVehicle.getDriveDynData, IPhysx.getVehicleDriveData, IPhysxVehicle.computeWheelSimulationTransforms. The following methods can be used instead: IPhysx.setWheelRotationSpeed, IPhysx.setWheelRotationAngle, IPhysx.getWheelState, IPhysx.getVehicleDriveState of the omni.physx extension and IPhysxVehicle.computeSuspensionFrameTransforms of the omni.physx.vehicle extension.

### UI

**General**

    - Added

        - Collision visualizers are now being replaced by dimmed AABBs at user configurable distance (the distance is saved separately within each stage).
        - Option to enable synchronous CUDA kernel launches in the Physics Debug panel. Synchronous CUDA kernel launches are needed to correctly report CUDA errors.

    - Fixed

        - Fix error spew when query mesh/shapes overlap with room for Overlap Mesh/Shape demos

## [105.0.15] - 2023-06-23

Version c7f35c2c98e7901e2d7fc3f9010b1c3c61ed5546

### Core

**General**

    - Fixed

        - Issue with start up error when omni.usd.readable_usd_file_exts() is removed at startup from file importer

## [105.0.14] - 2023-06-22

Version 02615a1bd27084a2e10a08924d295a9bc965500e 

### Core

**General**

    - Fixed

        - Usd context is checked for validity when refreshing stage
        - Fixed crash occuring when particles interact with deformable bodies
        - Potential material removal crash fix

    - Removed

        - Fluid Isosurface Glass Box Demo - same feature is showcased by Particle Postprocessing

## [105.0.13] - 2023-06-09

Version e968182c8d78e39cbfe0c879f47d5609f6daa04d 

### Core

**General**

    - Fixed

        - omni.physx.ui has its dependency on the hotkey.core extension changed to optional
        - Crash occuring when having anisotropy or smoothing on particle system with no fluid particle sets
        - Crash occuring when going from pause to stop with anisotropy or smoothing on particle system 
        - Extremely-high-mass (~1e7 mass units) articulation links would not collide with other actors
        - An issue affecting PGS CPU simulation determinacy 
        - A crash affecting scenes with deformable-rigid attachments
        - Character controller camera stability when at a larger distance from zero

### Vehicles

**General**

    - Fixed

        - Crash when a wheel attachment prim of a vehicle with a tank differential was deleted while the simulation was running

## [105.0.12] - 2023-06-01

Version f8f0d65f97e88a28c8f435053e24236fa994563f

### Core

**General**

    - Added

        - Exposed mouse and gamepad sensitivity on the ControlsSettings character controller graph node.

    - Fixed

        - Clear buffered updates for recomputing localPos on joint body paths change if new localPos is received
        - The voxel mesh for soft bodies will now be more symmetric to avoid lateral ghost forces. This applies to soft bodies that get a new deformable API applied. Existing APIs in older files keep their mesh.
        - Hotkeys conflicting with active character controller inputs are disabled during simulation.
        - Character controller camera computes its transformation correctly when in a hierarchy with a parent transformation.

## [105.0.11] - 2023-05-30

Version a38dffbd5b9074328d881da819adbaf6406b77ae 

### Core

**General**

    - Fixed

        - Joint error icons refresh
        - Crash when body enabled was toggled in runtime, while a forceAPI has been active
        - Remove cooking button removes cooking API and checks for orphaned cooked data too

### Zero G

**General**

    - Fixed 
   
      - Collision filtering issues

### Vehicles

**General**

    - Fixed

        - Vehicle Creation Wizard triggering UI errors on second page
        - Broken camera setup in vehicle Ackermann steering sample


## [105.0.10] - 2023-05-17

Version 1ae469fb7fd324c1106fcee2bed3df73df053fa2

### Core

**General**

    - Fixed

        - CPU/GPU difference for articulation joint drives in high-iteration-count simulation steps.
        - Fixed joints involving an articulation did not properly lock the rotational degrees of freedom

## [105.0.9] - 2023-05-10

Version baf73d7694921f82d3d885fc6279e1d5b9a8ffea

### Core

**General**

    - Fixed

        - Simulation minFramerate accepts 0 as unlimited substepping
        - ShapeDebug draw crash

### UI

**General**

    - Fixed

        - Zero G improvements

## [105.0.8] - 2023-05-03

Version 2a6aed5f0b832d51ccbaeb609db1a047fc321f6d

### Core

**General**

    - Fixed

        - Added 'sdf' as a valid approximation param for omni.physx.utils.setCollider        
### UI

**General**

    - Fixed

        - Zero G improvements

## [105.0.7] - 2023-04-25

Version ccd31c93bdd2d7bd72a50210db2aa2521869d56c 

### Core

**General**

    - Fixed

        - SDK: Fixes convex hull cooking such that coplanar faces are more consistently merged when the GPU polygon limit is hit
        - SDK: Fixes crash when all collision geometry is removed from an articulation
        - A crash that appeared when deleting a particle set/cloth after the parent particle system has been fixed.
        - Crash when cudaContextManager failed to be initialized

## [105.0.6] - 2023-04-14

Version 20a8bc3c8fc04e57724fd96418bfd2c2ccd48ff0

### Core

**General**

    - Added

        - Support for other simulation engines, omni.physx does not anymore always create bodies if the simulationOwner is not known

    - Fixed

        - Fixed kind change notification, fixed articulation collider resync notification
        - Fixed a crash with when deleting all the colliders from an articulation.

## [105.0.5] - 2023-04-04

Version b16f7d6bbbdd50657f1b2a08dc3982cc270c1aea

### Core

**General**

    - Fixed

        - Support for MetricsAssembler
## [105.0.4] - 2023-03-29

Version 52d6c8f7efc5f820d4c52da30827e1b736390d12

### Core

**General**

    - Added

        - CUDA device is now picked as the first device from the assigned displayGroup, if available and not overriden by the cudaDevice setting.

    - Fixed

        - Kinematic bodies update correctly matches animation 
        - Prismatic joint body check also checks for position based on the joint's axis.
        - Point instancer will group shapes under a static body when no body is present in a prototype.
        - Reset camera if a Character controller in first person mode changed it (only when using Python utilities).
        - Multiple materials fixes
        - Rack and pinion and gear joint delete fixes
        - Multiple scenes contact report fix

## [105.0.3] - 2023-03-16

Version 4f9811339a014bdb46748f82a54bf79a3947cd07 

### Core

**General**

   - Fixed

      - FilteredPairs between two articulation roots

   - Changed

      - USD updated to 22.11
      - PhysxSchema.PhysicsPlane replaced by UsdGeom.Plane


## [105.0.2] - 2023-03-09

Version 4669bbbd8b08409081211f5d59ae203538a1c0c7 

### Core

**General**

    - Added

        - Character controller Action Graph nodes.
        - Add trigger nodes OmniGraph demos
        - Add immediate nodes OmniGraph demos
        - Added Contact Event OmniGraph nodes.
        - Add immediate OmniGraph node ""Compute Mesh Intersecting Faces""
        - Add immediate OmniGraph node ""Generate Geometry Contacts""
        - Add museum demo and HSOV testbed
        - Limit to a number of prims in subtrees of selected prims for which the Add menu is fully refreshed to avoid hangs, currently defaults to 100000. This limit is settable in Preferences.

    - Changed

        - Procedural Cameras are now added using the Add > Cameras right click menu.
        - The Update All Cameras button was replaced with an Alwyas Update property in the procedural camera Properties.

    - Fixed

        - Cooking triangulization fix
        - Fix articulation root pose not being properly updated on USD transform change
        - Material density update for convex decomposition shapes
        - Update kinematics flag
        - Properly update joint local poses connected to articulations root links during simulation (accounting also for scaling)
        - Fix using "Compute Bounds Overlap" OmniGraph node without any other Immediate mode node

    - Deprecated

        - Deprecated articulation force sensor

### Vehicles

**General**

    - Added

        - Many parameters are now recorded by the PhysX Visual Debugger (OmniPVD)

    - Fixed

        - The vehicle creation wizard falsely stated that chassis and wheel mass were in kilograms. These labels have been removed now since the units of the mass values are actually based on the stage kilogramsPerUnit setting.
        - The vehicle creation wizard created render cylinder objects with wrong extents for the wheels


## [105.0.1] - 2023-02-27

Version 309e1f29ba9555b67cdb528f2dce051e0d23e8d3

### Core

**General**

    - Added

        - Create python bindings for callback system to detect object changes
        - Add Collider Trigger Enter / Leave OmniGraph nodes
        - Add Immedate OmniGraph nodes (Compute Geometry Bounds, Compute Geometry Intersection, Compute Geometry Penetration, Compute Bounds Overlaps)

    - Changed

        - Add omni.physx.graph extension to omni.physx.bundle to make it available by default
        - Parsing of transform hierarchy now queries transforms at earliest time instead of default

    - Fixed

        - Fixed collision-behavior issue when using GPU tensor API to update joint positions and root poses of articulations with links with no collision geometry.
        - omni.physx will not crash during startup when CUDA libraries are not present
        - Visibility of SDF Mesh and Sphere Fill approximation properties in the Collider widget.

### Vehicles

**General**

    - Added

        - Vehicle Creation Wizard: there is now support to have the wizard work directly on an existing prim hierarchy instead of creating all prims from scratch. All the necessary API schemas will then get applied to the user specified prims.

    - Changed

        - Vehicle Creation Wizard: UsdGeomScope instead of UsdGeomXform is now used for prims where transforms are irrelevant.

    - Fixed

        - The wheel positions potentially got flipped if the order of the PhysxVehicleWheelAttachmentAPI prims did not match the order of the corresponding wheel indices.

    - Removed

        - Vehicle Creation Wizard: the option to adjust the vehicle position has been removed. It can be easily done after the vehicle has been created.

### Force Fields

**General**

    - Added

        - Add a Surface Detection feature that applies forces on the surface along the surface normal. This induces rotation into the resulting motion.
        - Added an option to scale forces by the cross sectional area of the Rigid Body facing the applied force. This is on by default and may require retuning previous Force Field coefficients or turning this scaling off.

## [105.0.0] - 2023-02-06

Version 27128a7963e0a6a98d0fece2c3225e97794e5115

### Core

**General**

    - Added

        - Allows the user to approximate a triangle mesh as a collection of spheres
        - IPhysxCooking precook function to populate cooking data
        - Shift+double-click now triggers force push during simulation. Keep the button down for continued application.
        - Added new display option for rigid body mass properties in the view menu.
        - Add collision audio to physics demos.
        - Support for using animated meshes as kinematic deformable bodies
        - Limit to a number of selected prims for which the Add menu is fully refreshed to avoid hangs, currently defaults to 1000. If this limit is reached the Add menu will show all options and a warning on top. This limit is settable in Preferences.

    - Changed

        - Add menu will now list all options for large selections regardless of the type of prims selected or APIs applied to avoid hangs. The selection prim limit for filtering of the Add menu can be changed in Physics Preferences.
        - Most demos now use a room-scale scene and have been scaled or modified to fit it
        - Selecting multiple rigid bodies when the mass distribution manipulator is enabled now shows combined mass properties.
        - Added Deformable Visualization option to hide attached actors
        - Heavy rework of zerog and zerog sweep: improved zerogravity sweep algorithm, automatic precooking, rendering improvements, colliders reusing, UX improvements.

    - Fixed

        - ZeroGravity sweep mode improvements - solved point instancing issues, small movements now don't reset the sweeping area entirely and greatly enhanced user experience moving bvh construction/refit/hit scans/selection updates to pooled lower priority secondary threads. Plus a bunch of other bugfixes to make it more stable.
        - Disallow usage of force grab during inspector simulation, that can potentially cause crash.
        - Makes custom cone and cylinder more stable when colliding with much bigger objects (a cylinder rolling on a very long rail)
        - Attachment visualization works with animated kinematic rigid bodies
        - Fix crash on updating joint state for joint that has been removed during simulation
        - Improve performance on colliders solid meshes visualization (Physics Debug->Colllision Mesh Debug Visualization->Explode View Distance)

    - Removed

        - Removed deprecated simulation event stream

### Vehicles

**General**

    - Changed

        - The attributes suspensionFramePosition, wheelFramePosition, wheelCenterOfMassOffset, suspensionForceAppPointOffset, tireForceAppPointOffset (see PhysxVehicleWheelAttachmentAPI) and suspensionForceAppPoint, tireForceAppPoint (see PhysxVehicleSuspensionComplianceAPI) now take the total scale of the vehicle prim into account. Note that it is still recommended to stick to identity scale when using vehicles since not all attributes are scale aware.

    - Fixed

        - Crash on invalid USD setup, having no physics scene prim with the PhysxVehicleContextAPI API schema applied.

**USD Schemas**

    - Added

        - The custom metadata physxVehicle:referenceFrameIsCenterOfMass has been introduced to define in what space certain wheel attachment properties are interpreted in. If set to True, the reference frame will be the vehicle center of mass frame. If set to False, the reference frame will be the vehicle prim frame. The affected properties are: suspensionTravelDirection, suspensionFramePosition, suspensionFrameOrientation, suspensionForceAppPointOffset, wheelCenterOfMassOffset and tireForceAppPointOffset (see PhysxVehicleWheelAttachmentAPI). This custom metadata can be set on the prim that has PhysxVehicleAPI applied. Note that using the center of mass frame as reference frame (=True) is deprecated and will not be supported for much longer.

    - Deprecated

        - The PhysxVehicleWheelAttachmentAPI properties suspensionTravelDirection, suspensionFramePosition, suspensionFrameOrientation, suspensionForceAppPointOffset, wheelCenterOfMassOffset and tireForceAppPointOffset were interpreted as being relative to the vehicle center of mass coordinate frame. In the future, these properties will be interpreted as being relative to the vehicle prim coordinate frame. For backwards compatibility reasons, the old interpretation is still used as the default but it is highly recommended to switch to the new interpretation as soon as possible. See the section about the new custom metadata physxVehicle:referenceFrameIsCenterOfMass which can be used to enable the new interpretation.

**Python Bindings API**

    - Changed

        - The IPhysx.get_wheel_state function will use a different coordinate frame for the provided wheel transforms depending on the value of the custom metadata physxVehicle:referenceFrameIsCenterOfMass. See the API documentation of this function for details.

    - Removed

        - The deprecated IPhysxVehicle.set_to_rest_state method has been removed. Please use IPhysx.set_vehicle_to_rest_state of the omni.physx extension instead.

**C++ API**

    - Changed

        - The IPhysx.getWheelState and IPhysx.getWheelTransformations functions will use a different coordinate frame for the provided wheel transforms depending on the value of the custom metadata physxVehicle:referenceFrameIsCenterOfMass. See the API documentation of these functions for details.

    - Removed

        - The deprecated IPhysxVehicle.setToRestState method has been removed. Please use IPhysx.setVehicleToRestState of the omni.physx extension instead.

### Force Fields

**General**

    - Added

        - A new Conical Force Field can push objects from a central point. Fall off and angular limits can be set to control the magnitude of the force. A new OmniGraph demo is provided to illustrate how it works.

    - Fixed

        - The OmniGraph Force Field demo works again. Press Control-C to grab barrels and release to throw them.

**USD Schemas**

    - Added

        - Added a new schema for a cone shaped Force Field.




## [104.2.4-5.1] - 2023-02-03

Version df874d67738df98835b6b5cf36ecec49a1207446    

### Core

   - Fixed

      - Bug involving GPU simulation where newly created actors could miss collisions against existing actors.
      - Debug visualization for point instancer with indices out of prototypes.

## [104.2.3-5.1] - 2023-01-30

Version 0baa322e1d6882284a50903e3ceba85f38df8ab3   

### Core

   - Fixed

      - Softbody attachments visualization crash.
      - Joint visualization crash fix with invalid descriptors.
      - Apply force error to disabled bodies.
      - Filtering pairs crash.
      
   - Added

      - Fabric change tracking pause exposed to python bindings.

## [104.2.2-5.1] - 2023-01-24

Version 71c501e407fb45cc3469c30af271c804801b7f26  
### Core

   - Fixed

      - Crashes where CudaContextManager failed to be created.
      - Demo text draw.
      - Contact report call when no scene exists.
      - Contact report crash when no points are reported.
      - Trigger reporting if either trigger or the other collider is deleted.
      - Race condition in omni physx cooking.
      - Fixed issue on CentOS7 with incorrect math library.
      - Crash in create D6 joint if the input parameters are illegal. 
      - Sending correctly create/destroy object notification if a body changes from dynamic to static.

## [104.2.1-5.1] - 2023-01-17

Version bfc1e042528ccf11058577e0438464b4d5be6db0 
### Core

   - Fixed

      - Material property updates when using GPU simulation with omni.physics.tensors.
      - Create of an external joint.
      - Unregister custom log channel for supportui.
      - Articulation Force Sensor delete issues.

   - Changed
      - Toggling whether to use convex mesh or custom geometry for cube and cylinder geometry is now a general setting
      found under Physics Settings. The default is using custom geometry.

   - Added

      - Missing is_sleeping/wake_up/put_to_sleep functionality for articulations.

### Vehicles

**General**

   - Fixed

      - Some auto-running samples could cause the application to freeze up when the running sequence ended.

## [104.2.0-5.1] - 2023-01-05

Version 2ac488fd0c9bbb4505ffff5bba0fae32986952d9 
### Core

   - Fixed

      - Custom cone and custom cylinder behavior.
      - Bounce threshold minimal value, cant be zero.
      - Crash in error reporting when CUDA initialization failed.
      - Contact report issue when zero gravity was disabled.
      - Crash when articulation root got deleted while rest of the hierarchy was still present.
      - JointState update if the joint was removed during simulation.
      - Crash when shape was enabled but not yet inserted into a scene.
   
   - Changed

      - Improved demos scope usage.

## [104.1.6-5.1] - 2022-12-09

Version c8bc89fe7a83d2121fb0d171e8cedd980c351d2d  
### Core

   - Fixed

      - Hierarchy delete with rigid bodies and joints.
      - Demo categories.
      - Shift click mouse gesture dragging.

## [104.1.5-5.1] - 2022-12-05

Version c6c19ec941a4d2a69f24014744e13f108a02bffa  
### Core

   - Fixed

      - Missed collisions with convex hull colliders.
      - Cooking data save for not writable prims.
      - ForceAPI applied to a static or kinematic body.
      - Material creation with invalid parameters.
      - Replicator materials.
      - Crash when multiple deformable bodies were deleted.
### Vehicles

**General**

   - Fixed

      - Suspension compliance sample.
## [104.1.4-5.1] - 2022-12-01

Version 05c0c29247ae0f2c83e5d73139f3e527f249cfd5 

### Core

   - Fixed

      - Mass distribution and physics inspector combined usage crash.
      - Physics demo closing script fix.
      - Contact reporting crash when zero gravity extension was unloaded.
      - Rope demo fix.
      - Articulation parsing crash fix.
      - Picking crash when a link was deleted.
      - Supportui crash when new stage was loaded.

   - Added

      - Force grab override.

### Vehicles

**General**

- Fixed

   - Crash on invalid USD setup (attribute rigidBodyEnabled being set to false on PhysicsRigidBodyAPI API schema or attribute kinematicEnabled being set to true together with vehicleEnabled being set to true).


## [104.1.3-5.1] - 2022-11-25

Version 15b4f133baae15b16bde5fa7df5fb130fe8867e6 

### Core

   - Fixed

      - Particle cloth rendering with flatcache and tensor API.
      - Crash when flatache extension was closed during simulation.
      - Get scene pointer crash fix when physics toolbar was used.
      - PhysX SDK binary compatibility.
      - Deformable debug visualization.

   - Changed

      - Demos category reorganization.


### Vehicles

**General**

- Fixed

   - Crash on invalid USD setup, having no physics scene prim with the PhysxVehicleContextAPI API schema applied.


## [104.1.2-5.1] - 2022-11-21

Version 013b7d61a0d0bb7c7c75cd21ae998c920469dfc5  

### Core

   - Added

      - IPhysxReplicator interface for physics replication.
      - physics/disableContactProcessing setting to skip onContact processing in omni.physx.
      - physics/saveCookedData setting to skip saving cooked data into USD.

   - Changed

      - Zero gravity sweep mode improvements.
      - Physics toolbar improvements for auto collision mode on scenes with existing physics.

   - Fixed

      - Flatcache reset on stop functionality.


## [104.1.1-5.1] - 2022-11-16

Version e864c1d0c486956db1d4b4448fe52beb923d390b 

### Core

   - Added

      - PhysxSDFMeshCollisionAPI for SDF properties. "sdf" is an additional valid token for the physics:approximation property. Setting it in the UI applies the new API.
      - Support angular surface velocity on kinematic bodies.

   - Changed

      - ResetOnStop persistent setting was changed to non-persistent.
      - SDF properties from PhysxTriangleMeshCollisionAPI were moved into the new PhysxSDFMeshCollisionAPI.
      - Improved physics toolbar work with scene graph instancing.
      - Mass manipulation improvements.

   - Fixed

      - Floating articulation root joint determinism.
      - RigidBody manipulator with kinematic bodies.

## [104.1.0-5.1] - 2022-11-09

Version 575f53e707ad3b2cfdfa1af2c735dc3f3b6952fb

### Core

   - Added      
      - Shape volume into IPhysxPropertyQuery.
      - Physics Authoring toolbar supports instanced assets.

   - Changed

      - Dynamic rigid bodies that do have animated xformOp are converted automatically to kinematic bodies. Warning is issued.    
      - Mass distribution manipulator enhancements.  
      - Simulation info window cleanup.

   - Fixed

      - SDF collisions contact report.
      - Physics Authoring toolbar disappears when layout window is used to reset windows.
      - Range traversal fix for prims with only overs.

### Vehicles

**General**

- Added

   - Extra wheel authoring helpers to configure steer/brake/differential multipliers on a wheel attachment prim.


   - Fixed

      - IPhysXPropertyQuery: Report aabbLocalMin / aabbLocalMax with correct local scaling instead of global scaling

## [104.0.4-5.1] - 2022-10-27

Version 01507652991331a3f20d04bbcf5bc81df3f9f51e

### Core

   - Changed

      - Simulation output window warns if scene graph instancing is used without omni hydra support enabled.

   - Fixed

      - IPhysXPropertyQuery: return valid bounding box for capsule shape
      - Correct updates to mass properties at runtime
      - Spherical joint position/-target user input check correctness

### Graph

   - Changed

      - Scene query graph demo now checks for omni graph flags that are required for the demo to work properly.


## [104.0.3-5.1] - 2022-10-21

Version 31957463

### Core

   - Fixed

      - Docking of Physics Authoring Toolbar to the main Viewport does not show Viewport tab (unless it was shown before).
      - Physics Authoring Toolbar: Collider creation: if existing collider on a prim matches the collider simplification target type, it is skipped and no notification message is shown.
      - Physics Authoring Toolbar: When auto. collision creation mode is enabled, any prim which has any physics or physx schema applied, is skipped. When collision creation is requested manually by clicking appropriate button on the toolbar, only allowed existing physics components on selected prim are CookedDataAPI, CollisionApi and RigidBodyAPI, otherwise coll. creation is skipped.
      - Physics Inspector button removed from Physics Authoring Toolbar (can be found under Settings submenu of the toolbar itself)	  
      - Crash if cuda context manager was not created correctly and GPUExtensions tried to initialize.
      - Mass visualization improvements, you can enable the feature via new button that was added to the Physics Authoring Toolbar.


## [104.0.2-5.1] - 2022-10-14

Version 31932187

### Core

   - Fixed

      - Reset rigid body for referenced prims when xformOp order was not overridden.
      - ForceField extension loading/unloading.
      - Fixed articulation crash when deleted.
      - Pressing play does not reset the property window scroll position.
      - OmniPVD OVD recording bug fixed

   - Added

      - Enable/Disable Solve Contact for a rigid body.

### Vehicles

**General**

- Fixed

   - Tank snippet sending wrong warning messages about using deprecated attributes when running more than once.
   - Friction vs. slip graph not showing up in the property window and not updating limits correctly when changing the values interactively.
   - Assert when collision geometry childed to a wheel attachment was using non unit scale.
   - Error message showing up when opening filter window for adding tire friction table target.

## [104.0.1-5.1] - 2022-10-04

Version 

### Core

**General**

   - Fixed

      - Gravity magnitude changes when starting at 0.
      - SupportUI physics inspector treeview issues and joint authoring for joints without limits.
      - IPhysxFlatcache interface release.

   - Removed

      - Deprecated event stream V1 usage.

   - Added

      - Disable for omni.physx.ui USD notice handling.


## [104.0.0-5.1] - 2022-10-01

Version 31867423

### Core

**General**

- Fixed

   - Transform gizmo is disabled when pressing Shift to prevent it colliding with the mouse push or drag functionality.
   - Angular velocity now properly adjusts the "Velocities in Local Space" setting.
   - Some advanced properties for spherical and distance joints now appear correctly in the UI.
   - Collision filtering for triggers.
   - Invisible prims are no longer grabbed in simulation mode.
   - Parsing of Improve Patch Friction attribute: The fallback (i.e. if unauthored) is now correctly parsed to enabled.
   - Crashes when articulations miss contacts due to inadequate contact offsets.
   - Debug visualization for asynchronous simulation is now working correctly.
   - Triggers work correctly with convex decomposition colliders.
   - Typo in particle debug visualization setting /persistent/physics/visualizationDisplayParticlesShowFluidSurface.
   - Made OmniPVD sampling more robust.
   - Joint helpers respect correctly visibility attribute.
   - Point instancers parsing with timesampled values.
   - Mass inertia when set back to 0,0,0 autocomputation gets correctly triggered.
   - Fixed incorrect assignment of PDB material

- Changed

   - Local cache path moved out from Documents folder to local OV cache folder. Example: C:\Users\${username}\Documents\Kit\apps\${appname}\cookedmeshcache -> C:\Users\${username}\AppData\Local\ov\cache\Kit\${kitversion}\${hexnumber}\cookedmeshcache
   - Renamed particle set debug visualization setting from /persistent/physics/visualizationDisplayParticlesShowFluidParticles
   to the more accurate visualizationDisplayParticlesShowParticleSetParticles
   - All joint prims will now show a base joint property widget with the common properties and an optional derived joint widget with the additional properties to enable changing base joint properties for different joint types in multiselection.
   - DiffuseParticlesAPI is applied on particle sets instead of particle systems.
   - Default RigidBody maxDepenetrationVelocity changed from 5 to 3.
   - Deprecated wakeUp, putToSleep and applyForceAtPos function on IPhysx interface and move them to IPhysxSimulation interface, they do now require stageId to be passed too.

- Added

   - Multiple scenes support
   - Save to file stream option for legacy PVD
   - Writes a warning to log if per-joint collisions are enabled for joints that are parts of an articulation.
   - Writes a warning to log if having the friction attribute set to non-zero on joints that are not parts of an articulation.
   - UI visualization of held point and force direction when dragging.
   - TriggerStateAPI that can be additionally applied to a trigger, the TriggerstateAPI contains list of triggered colliders.
   - Advanced attributes for compliant contacts in rigid-body material and corresponding demo that showcases the attribute.
   - Advanced Rigid-body attribute Contact Slop Coefficient and corresponding demo that showcases the attribute.
   - Open file directory and open file in default editor buttons added to the Demo Source window.
   - Demo Source window now has syntax highlighting.
   - Python bindings for debug visualization options: VisualizerMode, ParticleVisualizationRadiusType,
   ParticleVisualizationPositionType
   - Made it possible to toggle whether to ignore invisible objects when grabbing during simulation. This is enabled by default.
   - PhysxForceAPI - api that allows force definition for a rigid body. Added physics snippet Force to showcase the usage.
   - Exposed armature for articulated joints through PhysxJointAPI.
   - Exposed constraint-force-mixing scale term for articulated bodies through PhysxRigidBodyAPI.
   - Multiple particle samplers can now sample into the same points prim.
   - Runtime addition and removal of particle objects.
   - IPhysxCustomJoint interface that allows injection of custom PhysX SDK joints.
   - IPhysxCustomGeometry interface that allows injection of custom geometries into PhysX SDK.
   - IPhysxPropertyQuery to retrieve rigid body mass information.
   - New settings for empty scene simulation (default false), without physics objects simulation does not run.
   - New settings that disables sleeping (default false), useful for debugging or benchmarking, can be enabled in Debug window.
   - Apply torque on IPhysxSimulation interface.
   - Is sleeping query for a rigid body on IPhysxSimulation interface.
   - Mass distribution visualization and editing viewport overlay (accessible through the "eye" icon menu.)
   - Handle user changes to mass/density for PDB particles and deformabe body during simulation
   - Support PDB material changes during simulation
   - Dynamic rigid bodies can use triangle mesh colliders with SDFs to get a fast and precise collision response.
   - Added support for sparse SDFs on the triangle mesh collider.

### Vehicles

**General**

- Changed

   - The vehicle simulation code has been refactored and changes in vehicle dynamics behavior are expected, especially with respect to the suspension force and tire slip.
   - Many USD attributes of various vehicle components have been deprecated and will not be shown in the corresponding property widgets any longer. They can still be found in the Raw USD Properties view though.
   - Previously, it was possible to configure the suspension to create an asymmetric and physically questionable spring behavior. This was the case when having the maxDroop attribute value largely diverge from the recommended value of: ((sprungMass x gravity) / springStrength). Now it is not possible to create such an asymmetry any longer but as a consequence, the rest state of the suspension can change significantly and adjustments to the maxDroop value will be required (for example, if a larger value than the recommended one was used, the vehicle will end up sitting higher above ground than previously).
   - Vehicle rigid bodies should set the disableGravity attribute of the PhysxRigidBodyAPI API schema to true or else a warning will get triggered.
   - The computation of tire slip does not any longer try to artificially keep the slip low for cases where the longitudinal speed is low but wheel rotation speed is high. As a consequence, vehicles might accelerate more aggressively than previously in which case the tire longitudinal stiffness (longitudinalStiffness attribute of the PhysxVehicleTireAPI API schema) can be reduced to get closer to the legacy behavior. Furthermore, there is also a higher probability to encounter instabilities if a large tire longitudinal stiffness is chosen and at the same time a large simulation timestep is used. In such a case two options can be considered: either reducing the stiffness or increasing the number of vehicle simulation substeps (lowForwardSpeedSubStepCount and highForwardSpeedSubStepCount vehicle attributes). It is also possible to decrease the overall simulation timestep size (increasing timeStepsPerSecond of the physics scene), this, however, affects the whole simulation, thus increasing the vehicle simulation substep count is preferable as a first thing to try.
   - Using sweeps to detect collisions of the wheels with the ground does not require the wheels to have a collider any longer. The wheel radius and width will define the collision geometry that is used internally for the sweep.
   - The lowForwardSpeedSubStepCount and highForwardSpeedSubStepCount vehicle attributes are limited to a max mumber of 255.
   - The Vehicle Creation Wizard will set the targetGear attribute of the PhysxVehicleControllerAPI API schema to 1 for basic drive vehicles and to 255 (automatic gear change mode) for standard drive vehicles now (previously, 0 was used for those types).
   - The vehicle autobox will no longer kick in unless the target gear is set to automatic gear change mode (often referred to as "DRIVE" in automatic transmissions. Use the special value 255 for this purpose). If the transmission is in reverse or neutral gear, switching to automatic mode will have the same effect as switching to first gear.
   - Vehicles with basic drive will not accelerate any longer if the target gear is set to 0. Use a positive value for target gear to have a forward torque being applied to the wheels.
   - Part of the vehicle update logic will now run in parallel on multiple threads if available and if there are enough vehicles to simulate.
   - The local pose of the wheel collision shape prim with respect to its parent wheel prim will be accounted for in the computation of the local pose given to the PxShape instance that represents the wheel's collision shape. 

- Fixed

   - The effect of camber was not implemented properly. If the effect should be ignored, it is now important to set the camberStiffness attribute to 0.

- Added

   - Many new features and parameters have been added. See the USD Schemas section for more details.
   - Wheel based tank vehicles are now available (see USD API schemas PhysxVehicleTankDifferentialAPI and PhysxVehicleTankControllerAPI). The Vehicle Creation Wizard has an additional option "Tank Mode" to create such a vehicle type.
   - The Vehicle Creation Wizard has now an option to make use of Ackermann correction for a pair of steered wheels.

**USD Schemas**

- Changed

   - PhysxVehicleTireAPI API schema
      - The longitudinalStiffnessPerUnitGravity attribute will now be set to 500 if not specified.
      - The longitudinalStiffnessPerUnitGravity attribute has been deprecated and will be removed in the future. Please use the new attribute longitudinalStiffness instead. When migrating code/assets, multiply the old value by the gravitational acceleration.
      - The camberStiffnessPerUnitGravity attribute will now be set to 0 if not specified.
      - The camberStiffnessPerUnitGravity attribute has been deprecated and will be removed in the future. Please use the new attribute camberStiffness instead. When migrating, multiply the old value by the gravitational acceleration.
      - The latStiffX and latStiffY attributes have been deprecated and will be removed in the future. Please use the new lateralStiffnessGraph attribute instead. When migrating code/assets, latStiffX can be used as the first entry of lateralStiffnessGraph. The second entry can be approximated using (latStiffY x sprung mass x gravitational acceleration).
   - PhysxVehicleAutoGearBoxAPI API schema
      - The upRatios attribute must not contain an entry for neutral gear any longer (there will be no automatic shift from neutral to first gear anymore).
   - PhysxVehicleSuspensionAPI API schema
      - The maxDroop attribute can now be set to a negative number to indicate that the value should be computed automatically.
      - The maxDroop and maxCompression attributes have been deprecated and will be removed in the future. Please use the new travelDistance attribute instead (set it to maxDroop + maxCompression when migrating your code/assets).
      - The camberAtRest, camberAtMaxCompression and camberAtMaxDroop attributes have been deprecated and will be removed in the future. Please use the new PhysxVehicleSuspensionComplianceAPI API schema instead to specify a non-zero camber angle.
   - PhysxVehicleDriveBasicAPI API schema
      - The peakTorque attribute will now be set to 1000 if not specified. Furthermore, 0 is now a valid value too.
   - PhysxVehicleWheelAttachmentAPI API schema
      - The driven attribute will be removed in the future. Please use the new PhysxVehicleMultiWheelDifferentialAPI API schema instead to describe the wheels that are driven by the engine.
      - The wheelCenterOfMassOffset attribute is deprecated and will be removed in the future. Please use the new attributes suspensionFramePosition and suspensionFrameOrientation instead. For more details see the entry in the "Added" section about these new attributes.
      - The suspensionForceAppPointOffset and tireForceAppPointOffset attributes have been deprecated and will be removed in the future. Please use the new PhysxVehicleSuspensionComplianceAPI API schema instead to specify the force application points or else leave it up to omni.physx to choose the points.
   - PhysxVehicleWheelAPI API schema
      - The maxBrakeTorque and maxHandBrakeTorque attributes have been deprecated and will be removed in the future. Please use the new PhysxVehicleBrakesAPI API schema instead to specify the wheels that encounter a brake torque and to define the maximum torque that can be applied. See entry about PhysxVehicleBrakesAPI for more details.
      - The maxSteerAngle attribute has been deprecated and will be removed in the future. Please use the new PhysxVehicleSteeringAPI API schema instead to specify the wheels that are steerable and to define the maximum steer angle that can be applied. See entry about PhysxVehicleSteeringAPI for more details.
      - The toeAngle attribute has been deprecated and will be removed in the future. Please use the new PhysxVehicleSuspensionComplianceAPI API schema instead to specify a non-zero toe angle.
   - PhysxVehicleControllerAPI API schema
      - The brake and handbrake attributes have been deprecated and will be removed in the future. Please use the attributes brake0/brake1 instead. See entry about PhysxVehicleBrakesAPI for more details.
      - The steerLeft and steerRight attributes have been deprecated and will be removed in the future. Please use the attribute steer instead (see corresponding entry under the Added section).
   - PhysxVehicleContextAPI API schema
      - The upAxis and forwardAxis attributes have been deprecated and will be removed in the future. Please use the new attributes verticalAxis and longitudinalAxis instead. Note that these new attributes expect token values instead of vectors to make it very explicit that only the main axis directions are supported.
   - PhysxVehicleAPI API schema
      - The minLongitudinalSlipDenominator attribute has been deprecated and will be removed in the future. Please use the new attribute minPassiveLongitudinalSlipDenominator instead.
   - PhysxVehicleEngineAPI API schema
      - The attributes peakTorque, maxRotationSpeed, dampingRateFullThrottle, dampingRateZeroThrottleClutchEngaged and dampingRateZeroThrottleClutchDisengaged now also accept 0 as a valid value.

- Added

   - PhysxVehicleEngineAPI API schema
      - Has a new attribute idleRotationSpeed to define the minimum rotation speed of the engine.
   - PhysxVehicleTireAPI API schema
      - Has a new attribute restLoad that allows to explicitly specify the rest load of the tire. The attribute has a default value of zero which means it gets computed automatically using (sprungMass x gravity) as an approximation.
      - Has a new attribute lateralStiffnessGraph which replaces the deprecated attributes latStiffX and latStiffY.
      - Has a new attribute longitudinalStiffness which replaces the deprecated attribute longitudinalStiffnessPerUnitGravity.
      - Has a new attribute camberStiffness which replaces the deprecated attribute camberStiffnessPerUnitGravity.
   - PhysxVehicleAPI API schema
      - Has a new set of attributes to give some control over the sticky tire mode. See attributes longitudinalStickyTireThresholdSpeed, longitudinalStickyTireThresholdTime, longitudinalStickyTireDamping, lateralStickyTireThresholdSpeed, lateralStickyTireThresholdTime, lateralStickyTireDamping.
      - Has new attributes minPassiveLongitudinalSlipDenominator, minActiveLongitudinalSlipDenominator and minLateralSlipDenominator to define minimum values for the denominator when computing tire slip. The attribute minPassiveLongitudinalSlipDenominator replaces the deprecated attribute minLongitudinalSlipDenominator.
   - PhysxVehicleWheelAttachmentAPI API schema
      - Has new attributes suspensionFramePosition and suspensionFrameOrientation to define the transform of the suspension at max compression relative to the vehicle's center of mass frame. These attributes replace the deprecated wheelCenterOfMassOffset attribute. Note that as mentioned, these new attributes define the suspension transform at max compression while wheelCenterOfMassOffset used to define the local space wheel position at rest. For migrating old assets/code, the suspensionFramePosition can be set to: wheelCenterOfMassOffset - (suspensionTravelDirection x physxVehicleSuspension:maxCompression).
      - Has new attributes wheelFramePosition and wheelFrameOrientation to define an additional transform of the wheel relative to the suspension frame. This allows to, for example, have the steering axis not go through the wheel center or to introduce a fixed toe angle.
      - Has a new attribute "index" which will be used to reference wheel attachments as well as to order the wheel attachments within a vehicle.
   - PhysxVehicleMultiWheelDifferentialAPI API schema (new)
      - This new API schema has been introduced to define which wheels are driven by the engine. It also allows to specify how torque should be distributed among the driven wheels. This API schema is meant to replaces the "driven" attribute of the PhysxVehicleWheelAttachmentAPI API schema and will take precedence if used.
   - PhysxVehicleTankDifferentialAPI and PhysxVehicleTankControllerAPI API schemas (new)
      - These new API schemas have been introduced to set up a vehicle as a wheel based tank.
   - PhysxVehicleBrakesAPI API schema (new)
      - This new API schema has been introduced to specify the wheels that will encounter a brake torque when the brake control fires. The maximum brake torque as well as wheel specific multipliers can be defined too. To allow for up to two sets of brake configurations, PhysxVehicleBrakesAPI has been defined as a multiple apply API schema. The instance names "brakes0" and "brakes1" are used to distinguish between the two configurations. This is a generalization of the previous brake/handbrake split that was rather rigid. As a consequence, the brake controls that go with the two configurations are called brake0 and brake1 and can be found in the PhysxVehicleControllerAPI API schema.
   - PhysxVehicleControllerAPI API schema
      - Has two new attributes brake0/brake1 that replace the deprecated brake/handbrake attributes. See previous entry for more info.
      - Has a new attribute steer that replaces the deprecated steerLeft/steerRight attributes. The equivalent of a steerLeft/steerRight value pair of 1/0 is steer=1 and the equivalent of 0/1 is steer=-1.
   - PhysxVehicleSteeringAPI API schema (new)
      - This API schema has been introduced to specify the wheels that are affected by the steer control (see PhysxVehicleControllerAPI). The maximum steer angle as well as wheel specific multipliers can be defined too.
   - PhysxVehicleAckermannSteeringAPI API schema (new)
      - This API schema can be used to steer a pair of wheels with Ackermann correction applied.
   - PhysxVehicleSuspensionAPI API schema
      - Has a new attribute travelDistance to define the distance the suspension can travel from max compression to max droop. This attribute replaces the deprecatd maxDroop, maxCompression attributes.
   - PhysxVehicleSuspensionComplianceAPI API schema (new)
      - This API schema has been introduced to specify toe angle, camber angle and force application points. These properties can either be set to fixed values or denote fixed sized graphs where the values depend on the suspension jounce.
   - PhysxVehicleTireFrictionTable IsA schema
      - Has a new attribute defaultFrictionValue to specify what friction value to use if the material of the ground surface is not listed in physxVehicleTireFrictionTable:groundMaterials.
   - PhysxVehicleContextAPI API schema
      - Has two new attributes verticalAxis and longitudinalAxis that replace the deprecated upAxis and forwardAxis attributes.
   - PhysxCharacterControllerAPI API schema
      - All CharacterControllerAPI properties were moved to PhysxCharacterControllerAPI.
   - PhysxTriangleMeshCollisionAPI API schema
      - Added new attributes to set up and control sparse SDFs. The new attributes are sdfResolution, sdfSubgridResolution, sdfBitsPerSubgridPixel, sdfNarrowBandThickness and sdfMargin.

- Removed
   - PhysxVehicleControllerAPI API schema
      - The automatic attribute has been removed. To enable automatic gear change mode, the targetGear attribute has to be set to the special value 255 instead. Note that a vehicle still needs PhysxVehicleAutoGearBoxAPI configured to enable automatic mode.
   - PhysxVehicleContextAPI API schema
      - The sweepRadiusScale and sweepWidthScale attributes are not supported any longer and have been removed.
   - CharacterControllerAPI API schema
      - CharacterControllerAPI was removed and its properties moved to PhysxCharacterControllerAPI.
   - PhysxTriangleMeshCollisionAPI API schema
      - Custom attribute physxsdfcollision:resolution got removed and replaced by physxTriangleMeshCollision:sdfSubgridResolution

**Python Bindings API**

- Changed

   - The IPhysxVehicle.get_drive_data method has been deprecated and will be removed in the future. Please use IPhysx.get_vehicle_drive_state of the omni.physx extension instead.
   - The IPhysxVehicle.set_wheel_rotation_speed and IPhysxVehicle.set_wheel_rotation_angle methods have been deprecated and will be removed in the future. Please use IPhysx.set_wheel_rotation_speed and IPhysx.set_wheel_rotation_angle of the omni.physx extension instead.
   - The IPhysxVehicle.get_wheel_query_results and IPhysxVehicle.get_wheel_data methods have been deprecated and will be removed in the future. Please use IPhysx.get_wheel_state of the omni.physx extension instead. Please read the corresponding bullet point in the C++ API section for some more details.
   - The IPhysxVehicle.compute_sprung_masses method now returns an empty list if the computation failed.
   - The IPhysxVehicle.compute_wheel_simulation_transforms method has been deprecated and will be removed in the future. Please use IPhysxVehicle.compute_suspension_frame_transforms instead. More details can be found in the corresponding bullet point in the C++ API section.

- Added

   - The variable VEHICLE_AUTOMATIC_TRANSMISSION_GEAR_VALUE (import from omni.physx.bindings._physx) can be used to avoid hardcoding the special value for automatic gear change mode.
   - The IPhysxVehicle.compute_suspension_frame_transforms method has been added as a replacement for compute_wheel_simulation_transforms. More details can be found in the corresponding bullet point in the C++ API section.

**C++ API**

- Changed

   - The IPhysxVehicle.getDriveDynData and IPhysx.getVehicleDriveData methods have been deprecated and will be removed in the future. Please use IPhysx.getVehicleDriveState of the omni.physx extension instead.
   - The IPhysxVehicle.setWheelRotationSpeed and IPhysxVehicle.setWheelRotationAngle methods have been deprecated and will be removed in the future. Please use IPhysx.setWheelRotationSpeed and IPhysx.setWheelRotationAngle of the omni.physx extension instead.
   - The IPhysxVehicle.getWheelQueryResults and IPhysxVehicle.getWheelDynData methods have been deprecated and will be removed in the future. Please use IPhysx.getWheelState of the omni.physx extension instead. Some more detailed changes are listed below:
      - The returned suspension jounce value is now in range [0, maxCompression + maxDroop] (used to be [-maxDroop, maxCompression])
      - The returned suspension force is now a vector
   - The IPhysxVehicle.computeSprungMasses method now returns a boolean to denote success or failure.
   - If IPhysx.getWheelTransformations is used with addVehicleTransform=false, then the transforms will now be relative to the vehicle rigid body center of mass frame and not the vehicle frame.
   - The IPhysxVehicle.computeWheelSimulationTransforms method has been deprecated and will be removed in the future (because it is operating on deprecated USD attributes). Please use IPhysxVehicle.computeSuspensionFrameTransforms instead. Note that this replacement method will set the suspension frame transform which is basically the transform of the suspension at full compression (and not wheel rest position like in the old method). Furthermore, the suspension force and tire force application points will not be set by the replacement method any longer (those are part of the new PhysxVehicleSuspensionComplianceAPI API schema now).

- Added

   - The constant expression VehicleControllerDesc::automaticGearValue in the PhysxUsd header file can be used to avoid hardcoding the special value for automatic gear change mode.
   - The IPhysxVehicle.computeSuspensionFrameTransforms method has been added as a replacement for computeWheelSimulationTransforms. It computes and sets the newly introduced suspensionFramePosition and suspensionFrameOrientation USD attributes of the PhysxVehicleWheelAttachmentAPI API schema. The suspension frame transform is the transform of the suspension at full compression (relative to the vehicle's center of mass frame). The method will treat the current Xform transform of the wheel attachment prim as the transform at full suspension compression and derive the suspension frame transform from that. More information can be found in the API description of the method.

- Removed

   - The IPhysx.getWheelQueryResult method has been removed. Please use IPhysx.getWheelState as a replacement.


## [1.4.19-5.1] - 2022-09-02

Version: 31722217

### Core

**General**

   - Fixed

      - Transformation reset for referenced rigid bodies.
      - Physics material assignment issues.
      - Prismatic joint visualization clipping issue.

## [1.4.18.2-5.1] - 2022-08-16

Version: 31691169

### Core

**General**

   - Update to include optimizations for pairs filtering.

## [1.4.18.1-5.1] - 2022-08-08

Version: 31655032

### Core

**General**

- Fixed

   - Crash in simulation for stages without a PhysicsScene and with CollisionGroups.
   - Exclusive shape error when PointInstancer is used.
   - Cloth wireframe fix for visualization in debug viz.
   
- Added

   - Lego buggy and teddy on ice demo.


## [1.4.17.1-5.1] - 2022-07-28

Version: 31624704

### Core

**General**

- Fixed

   - Transformation xformOp stack reset with a suffix xformOp.
   - Transformation xformOp stack and velocities attributes are cleared on stop if simulated in a layer without an opinion.
   - Viewport2 and viewport legacy physics compatibility fixes.

## [1.4.16.1-5.1] - 2022-07-25

Version: 31602073

### Core

**General**

- Added

   - Support for viewport2.
   - Visualization for physics picking while simulation is running.


## [1.4.15.1-5.1] - 2022-07-20

Version: 31588921

### Core

**General**

- Added

   - Exposed armature for articulated joints through PhysxJointAPI.
   - Exposed constraint-force-mixing scale term for articulated bodies through PhysxRigidBodyAPI.
   - Support for setting particle cloth points and velocities during simulation (to implement reset)
   - TensorAPI updates

## [1.4.14-5.1] - 2022-06-20

Version 31475809

### Core

**General**

- Fixed

   - Joint localPose parsing fix.
   - Point instancer debug visualization.
   - Particle post processing enable/disable fix.
   - Mass computation for rigid bodies with triggers.
   

- Changed

   - Zero gravity button is disabled when simulation is running.

## [1.4.13-5.1] - 2022-06-01

Version 31412806

### Core

**General**

- Fixed

  - Joint icons will refresh properly when omni.physx.ui is enabled after a stage was already loaded.
  - Joint error icons selection fix.
  - Joint localPose computation if bodies had scale.

- Changed

  - When creating joints with createJoint(s) util with two selected prims the prim that was selected first is assigned to body 0 and the one selected second to body 1 instead of vice versa.

## [1.4.12-5.1] - 2022-05-27

### Core

**General**

- Fixed

  - Materials assigned to instanced colliders
  - Fixed performance issues with collision groups on linux

- Added

   - Setting physics/cudaDevice to define preferred cudaDevice for PhysX SDK


## [1.4.11-5.1] - 2022-05-20

Version 31370036

### Core

**General**

- Fixed

  - Crashes when articulations miss contacts due to inadequate contact offsets

## [1.4.10-5.1] - 2022-05-17

Version 31350991

### Core

**General**

- Fixed

  - Crash when setting transform scale to zero on attached deformable body
  - Attachment debug visualization out of sync with updated attachments
  - Tensor API error logging code

## [1.4.9-5.1] - 2022-05-11

Version 31325511

### Core

**General**

- Fixed

  - Crash when adding particles to an empty particle system while sim is running
  - Memory leak in isosurface generation
  - Bug causing inaccurate particle-solid collisions
  - Crash in poisson sampling
  - Crash when using contact reports with SDF tri-mesh collision preview feature
  - Articulation force sensors force reporting uninitialized memory forces on first stage update after timeline play when no sim steps were run
  - Spherical joint hard vs soft limit.
  - Static collider transformation for instanced prims.

- Added

  - Spherical and Prismatic joints with disjointed body transforms will simulate and produce a warning.
  - NaN validation check for USD transformation writes.


## [1.4.8-5.1] - 2022-05-04

Version 31296124

### Core

**General**

   - Fixed

      - Removed assumptions in the code that particles will always be rendered using USD spheres.
      - Rigid static and kinematic bodies transforms are not reset on simulation stop/reset.
      - Fixes in particle debug visualization.
      - Fix mass computation for scaled particle cloth
      - Fix crash when adding deformable body component to mesh with time sampled points. Added warings for collider, particle cloth, particle sampler components.

   - Added

      - OverlapShape/OverlapShapeAny added, takes arbitrary UsdGeomGPrim and uses that for overlap queries.

## [1.4.7-5.1] - 2022-04-27

version 31270438

### Core

**General**

   - Fixed

      - CollisionAPI applied to a prim in a hierarchy creates correctly static bodies.
      - Fix maximal-coordinate joints limits being clamped inside [-90, 90] range

   - Changed

      - Parsing optimizations and runtime update loop optimizations.

   - Added

      - Use PhysX unwrapped joints on revolute joints without limits to avoid infinite spinning with drive targets outside [-360, 360]
      - JointStateAPI - support for saving and restoring joint states (articulated joints are supported only atm). Changes to
      joint state position and velocities is possible only during simulation.


## [1.4.6-5.1] - 2022-04-19

Version 31229710

### Core

**General**

- Fixed

   - Attribute arrays dimension mismatch crash for user manually instanced actors.
   - Particle cloth pulled to orgin bug

- Changed

   - Detected closed articulation loops will print all looped joints and will not create the loop joint by default.
   - Show warning when the particle cloth contains vertices with too many adjacent triangles to fully support lift and drag
   - Revolute and Fixed joints with disjointed body transforms will simulate and produce a warning.

## [1.4.5-5.1] - 2022-04-08

Version 31184222

### Core

**General**

   - Fixed

      - Tensor API fixes.
      - Joint visualization for invisible joints.
      - Particle cloth cooking crash fix.

   - Changed

      - Improved behavior of particle fluids near solid boundaries.
      - Reduced default collision filtering radius for deformable body and particle cloth attachments.
      - Increased welding tolerance for particle cloth cooking to support default mesh primitives.

   - Added

      - Revolute and Fixed joints with disjointed body transforms will not simulate. Error marker will appear over joint helpers of joint with such body transforms.
      - Added interface and Python bindings for physics interaction API.
      - "Particle Postprocessing" and "Particle Sampler" tutorial snippets.

## [1.4.4-5.1] - 2022-04-04

Version 31161611

### Core

**General**

   - Added

      - Particles debug visualization: Debug-window option to display render-geometry radius in addition to offsets.
      - Per Rigid Body local velocity / global velocity switch.

   - Changed

      - Deformables brush: Disabled by default.
      - Attachments: Only support two targets, attaching to world not supported
      - Active actors available only if required.

   - Fixed

      - Gizmo UID free fix.
      - CPU Dispatcher can be now correctly created with zero threads.
      - Joint helpers visibility fix after start/stop has been pressed.
      - Local space velocities fixed.
      - PhysXSDK fixes for custom geometries.
      - Transform standardization of physics objects on simulation start correctly handles presence of a scale attribute in case there is no scale op in the XformOps stack.
      - Particle Sampler:
         - No more crash when removing particles relationship
         - Improved feature behavior when editing particles relationship
         - No more resampling when a stage is saved or loaded from file
      - Fixes for particle system surface extraction and anisotropy with multiple particle systems.
      - Asynchronous deformable body cooking is now triggered when changing the scale of a mesh with deformable body component.
      - Fix crash when removing a particle system referenced by a primitive with particle cloth component.
      - Support on a physics scene to disable scene queries.

## [1.4.3-5.1] - 2022-03-25

### Core

**General**

   - Changed

      - Debug visualization does not anymore toggle async rendering flag.

   - Fixed

      - Flatcache extension proper shutdown.
      - PhysXSDK fixes for gyroscopic forces.

## [1.4.2-5.1] - 2022-03-14

Internal Version 31077407

### Core

**General**

   - Added

      - New items in Demos/Particles and Tutorials: Cloth Deck Chair, Fluid Isosurface Glass Box, Paint Ball Emitter demos; and Particle Cloth and Particle Inflatable tutorials
      - Demos extension defines demo asset server base URL in extension toml, see physics.demoAssetsPath

   - Fixed

      - Kinematic body target velocity fix, the inertia is not set to infinity during contact
      - Corrected visualization for Spherical Joint Axis on Y and Z
      - Fixes for particle debug visualization
      - Fix visibility issues for non-fluid particle sets when surface extraction is on

   - Changed

      - PhysXDevice update
      - Particle System: change default particleContactOffset to 5cm instead of autocomputation
      - Particle Cloth: Mesh welding is enabled on by default. Mesh welding ensures that duplicate vertices and triangle indices are removed to enable the correct simulation of a piece of cloth. This option can be turned off if the mesh is deliberately designed to have duplicate vertices or triangle indices. For example, simulating a broken or torn particle cloth.
      - Fixed Tendon: Dynamics were updated and a new attribute was added that can be used to tune the tendon response per axis


## [1.4.1-5.1] - 2022-03-07

Internal Version 31050168

### Core

**General**

- Changed

   - Removed PhysxParticleSystem.MaxParticles attribute.
   - Renamed PhysxSchema.PhysxParticlePoissonSamplingAPI to PhysxSchema.PhysxParticleSamplingAPI.
   - Improved PhysxParticleSystem property UI.

- Fixed

   - Xform reset for RigidBody, CCT, DeformableBody and ParticleCloth.
   - Duplication of UsdGeom.Mesh with PhysxSchema.PhysxParticleSamplingAPI, and other particle sampling fixes.
   - Issues and crashes with Particle Isosurface, Smoothing and Anisotropy components.
   - Mass/density parsing for DeformableBodyAPI.
   - Attachment of ParticleCloth (with UsdGeom.Mesh that requires welding for simulation).

### Vehicles

**General**

- Fixed

   - A regression in the Vehicle Creation Wizard caused the second page of the wizard to be inaccessible.

## [1.4.0-5.1] - 2022-02-21

Internal Version 31013585

### Core

**General**

- Added

   - Support for flat cache change tracking mechanism.
   - IPhysxSimulation:flushChanges function to flush pending physics updates.
   - Physics Debug Window now has buttons to remove UsdPhysics and PhysxSchema from selected prims.
   - Exposed isReadbackSuppressed and /physics/suppressReadback setting for direct GPU access functionality.
   - Exposed PhysX SDK profile data into carb profiler, can be disabled through a physics setting.
   - Exposed raycast_any, sphere_sweep_any and overlap_any functions on IPhysxSceneQuery interface.
   - Exposed raycast/sphere cast collider material information
   - Button to Physics debug window - Save scene to PhysX SDK RepX format.
   - Kinematic vs Kinematic contact reporting, enabled when Report Kinematic Kinematic Pairs attribute on a PhysicsScene is set to True (default False)
   - Kinematic vs Static contact reporting, enabled when Report Kinematic Static Pairs attribute on a PhysicsScene is set to True (default False)
   - Exposed transformationUpdateFn for IPhysxSimulation callback structure. The function is called right after fetchResults.
   - Exposed maxBiasCoefficient used in the constraint solver on physics scene.
   - Exposed Mass Information section into Physics Debug Window.
   - Joint selection will now outline the rigid bodies it connects to (dynamic body is green, static is red).
   - Exposed resume flatcache change tracking through IPhysxSimulation callback.
   - All physics attributes hooked up for runtime changes are now registered automatically to flatcache change tracking mechanism.
   - Exposed material0 and material1 in contact data event stream.
   - Added PhysxSchemaPhysxArticulationAPI::ArticulationEnabled attribute that does enable/disable articulation definition.
   - Added an option to the Visibility (Eye Icon) > Physics > Show By Type > Colliders > Normals that toggles the surface normals.
   - Added Gear and Rack and Pinion joints to the Create menu.
   - Added Physics default material binding UI for PhysicsScene prims.
   - Added stage_id to contact notification events.
   - Added batched contact reporting through a callback system (IPhysxSimulation::subscribePhysicsContactReportEvents) or through immediate API (IPhysxSimulation::getContactReport) in both cases python binding exist.
   - Added protoIndex support for raycasts/sweeps/overlaps.
   - PhysicsScene min/maxIterationCount that limits individual actors (rigidbody, softbody, particles) iteration counts.
   - Added support for point instancer scale indices.
   - Added support for UsdGeom hole indices.
   - Added PhysX CPU dispatcher setting, by default carbonite CPU dispatcher is used, its possible to override this behavior through a physics preferences setting.
   - Added new attachment type which can be used to attach deformable bodies to colliders, rigid bodies or other deformable bodies or the world frame.
      - New Menu option Create > Physics > Attachment (based on the selection of one or two suitable primitives)
      - New visualization option (Eye Icon) > Physics > Show By Type > Attachments
      - New property window to control the auto generation of attachment points and filters based on two geometrically overlapping primitives
      - New deformable brush extension for erasing, painting attachment points and collision filters
   - Added new position based dynamics particle system type - supporting fluid, granular, cloth and inflatable simulations
      - New Menu option Create > Physics > Particle System (particle system creation is automatic on addition of corresponsing components)
      - New Option in Create > Physics > Physics Material: PBD Particle Material
      - New visualization option (Eye Icon) > Physics > Show By Type > Particles
      - New Particle Cloth component that can be added to mesh prims.
      - New Particle Set component that can be added to points or point instancer prims.
      - New Particle Sampler component that can be added to mesh prims. (Autogeneration of points or point instancer).
      - New Anisotropy, Smoothing and Isosurface components that can be added to a Particle System prim.
      - New Diffuse Particles component that can be added to a Particle System prim.
   - Added IPhysxSceneQuery::reportCollisionShapes functions to query created collision shapes in PhysX.
   - Added support for surface velocity for kinematic bodies. If a kinematic body does have linear velocity set, it is
   converted into surface velocity. This is useful for example for conveyor belt.
   - Added custom logging channel omni.physx.logging.robotic. Turned off by default and togglable through /physics/logRobotics setting.
   - Added support for joint drives on D6 joints that are part of articulations.
   - Added Attach/Detach StageUpdateNode into Physics debug window, this allows omni.physx simulation attach/detach.

-  Changed

   - IPhysxSimulation:simulate: the elapsedTime parameter gets passed to the PhysX simulation directly, no substepping happens anymore. It's the caller's responsibility to provide reasonable elapsedTime.
   - Gear joint GearRatio can be now a negative value and can be changed in runtime.
   - Rack and Pinion joint Ratio can be now a negative value and can be changed in runtime.
   - Backward compatibility will check for deprecated schema also after the processing is done and advise if there's still some deprecated schema found.
   - Convex hull vertex limit maximum changed from 60 to 64.
   - IPhysx:getObjectId will now return a valid ID even if there is no directly accessible internal physics representation available (since these IDs might still be used to indirectly access internal data and state through other APIs).
   - Character controller extension was heavily rewritten on both the python and C++ side.Please see docs and the Character Controller/Arena demo for details on the new usage and utils.
   - Grouped physics-related widgets in the Property window under a common "Physics" frame.
   - IPhysx::getSimulationEventStream is deprecated and getSimulationEventStreamV2 should be used instead. For contact reports please use IPhysxSimulation::subscribePhysicsContactReportEvents
   - For omni.physx release configuration PhysX SDK Checked configuration is used instead of Profile
   - For elapsed time less or equal to zero, physics update step is skipped completely.
   - D6 joint definition in articulation hierarchy will have by default all rotation DOF free.
   - The following Physics settings originally residing in the Preferences window were changed from persistent to per-stage and are now saved to USD and changeable through the Physics Settings window: whole Update group, whole Mouse interaction group and Zero Gravity speed, Min Simulation Frame Rate and Enable Anisotropy for Fluid Rendering settings. Convex decomposition and mesh simplification settings were deprecated alltogether.
   - If a parent xform of a PhysicsJoint changes scale, linear limits (PrismaticJoint limits for example) are scaled too.
   - When a joint is created from single selection the prim will be set as body1. A drive target added to it will then define a world to body transformation instead of vice versa when body0 was used. Changed for all joint types to keep consistency.
   - Joints, presets and rigid body and collider component Add menu options are now always shown, but disabled whenever the prim has alread applied any conflicting APIs (instead of not being shown at all).
   - Options in the Add menu are now shown/enabled whenever that menu option is usable for at least one of the prims in the current selection and not only when usable for all the prims in the selection.
   - SimulationOwner property for Rigid Bodies and Colliders is hidden until multiscene support is implemented.
   - Joint helper 3d models replaced with billboards.
   - Default rigid body maximum depenetration velocity has been changed to 5 m/s.

- Fixed

   - USD physics resync operation on rigid bodies will restore joints connections.
   - All joint widgets will now properly include properties from PhysxJointAPI.
   - PhysxPhysicsRackAndPinionJoint ratio attribute is now in deg/distance (from incorrect rad/distance)
   - UI widgets limits for physxConvexHullCollision:hullVertexLimit and physxConvexHullCollision:minThickness.
   - Detecting closed circuits in articulations.
   - Properties that are authored on a prim, present in UsdPhysics or PhysxSchema schemas, but without these being applied to that prim, will produce a warning. Fixed crashes in property widgets.
   - Raycast/sweep sphere cast report correct USD face index
   - Materials are taken correctly if applied within a hierarchy
   - Rigid body with Colliders Preset or Colliders Preset added to a selection will now undo correctly in one undo step.
   - Contact report for meshes with convex decomposition approximation does now correctly send
   only one report per collider.
   - Gear joint and Rack and Pinion joint are now compatible with articulations.
   - Removed limits for prismatic joint's lower and upper limit property widgets.
   - Distance limit API applied to a D6 joint will now get correctly applied.
   - Fixed point instancer transformations when point instancer prims did have additional transformation.
   - Revolute joint Y axis transformation.
   - Revolute joint velocity drive sign.
   - Joint authoring rotate gizmo fix.
   - Random crash during property changes in runtime.
   - Debug Stop resets time to start time instead of zero when timeline is stopped.
   - Rigid body with Colliders Preset and Colliders Preset use only one command now and can be safely used on large selections.
   - Reset transformation non determinism issue.
   - Articulation drives and limits respect the USD joints body order.
   - Articulation spherical joint is 3DOF instead of 2DOF to be consistent with max. coordinate spherical joint.
   - D6 joint limits can be udpated after simulation is started.
   - Point instancer prototype parsing, if the prototype was outside of the point instancer scope.
   - Point instancer prototype parsing, if the prototype was scene instanced and nested.
   - Kinematic rigid bodies compute correctly their mass properties.
   - Rigid bodies transformation reset now resets correctly if structural changes were made during simulation.
   - Scaling of rotational joint drive damping and stiffness (Revolute and D6) not part of articulations.
   - Spherical joint limit order when joint axis is not X, and updating joint limits for spherical joints in articulations from USD after simulation start.
   - Sign of maximal-coordinate revolute and D6 rotational velocity drive targets to be consistent with articulation directions
   - Body order assignment when creating a joint on two prims fixed back to first prim to Body 0 rel and second to Body 1.
   - Refreshing Create main menu on selection change to properly refresh the Joint submenu.

**C++ API**

- Added

   - The IPhysicsObjectChangeCallback structure has been added to get notifications about internal object creation and destruction.
   - New IPhysxAttachment interface with attachment functionality needed by attachment visualization and brush, for example.
   - New IPhysxParticles interface for particle authoring, currently exposing functionality for isosurface, anisotropy and smoothed positions pre-view.

### Vehicles

**General**

- Fixed

   - Having a vehicle camera in a layer with a stronger opinion than the authoring layer could cause a crash.
   - The Vehicle Creation Wizard did not compute the chassis mass correctly for very small vehicles if the stage meters per unit value was not set to 1.
   - Removing the PhysicsRigidBodyAPI on a vehicle prim while the simulation is running could cause a crash.
   - The Vehicle Creation Wizard did not properly scale camber stiffness for small sized vehicles.
   - The gamepad triggers had to potentially be pressed twice to go into auto-reverse.

- Changed

    - All cameras were removed from the Vehicle extension and moved into a new Camera extension.

**Python Bindings API**

- Changed

   - The IPhysxVehicle.set_to_rest_state method has been deprecated and will be removed in the future. Please use IPhysx.set_vehicle_to_rest_state of the omni.physx extension instead.
   - The get_update_all_cameras, set_update_all_cameras, add_follow_camera, add_drone_camera and process_pending_usd_changes methods were removed. Use the new Camera extension and the get_camera_interface to create and setup cameras that track vehicles.
   - The method IPhysx.compute_vehicle_velocity has been added as a helper to compute the vehicle speed along a given local direction.

**C++ API**

- Added

   - The method IPhysx.getWheelTransformations has been added to read wheel transformations after the simulation has started.
   - The method IPhysx.computeVehicleVelocity has been added as a helper to compute the vehicle speed along a given local direction.

- Changed

   - The IPhysxVehicle.setToRestState method has been deprecated and will be removed in the future. Please use IPhysx.setVehicleToRestState of the omni.physx extension instead.
   - The getUpdateAllCameras, setUpdateAllCameras, addFollowCamera, addDroneCamera and processPendingUsdChanges methods were removed. Please use the methods in IPhysxCamera.h to create and setup cameras that also track vehicles.

### Force Field

**General**

- Added

   - New Force Field extensions that applies forces to Rigid Body objects that make them move in interesting ways. Available force fields include, Planar, Linear, Spherical, Spin, Wind, Drag and Noise. Additional force fields can be added upon request. Refer to the app_create/prod_extensions/ext_force-fields.html documentation for additional details and implementation instructions.

**Python Bindings API**

- Added

    - Use acquire_physx_force_fields_interface() to acquire the interface and the CreateForceFieldCommand, AddPrimsToForceFieldCommand and RemovePrimsFromForceFieldCommand to add force fields to prims and Rigid Bodies they affect. Use the add_force_field, remove_force_field, set_force_field_xxx, and enable_force_field methods to add, remove and set up the force fields. The prefered method is to create prims and use the GUI. The attach_stage and detach_stage methods are used to set up the force fields without pressing Play in Kit.

**C++ API**

- Added

    - The IPhysxForceFields.h header lists all of the interface methods.

### Camera

**General**

- Added

   - New camera extension created that provides three cameras to track Rigid Body objects and animated Xforms. A Look Follow Camera follows objects that always stay upright, like vehicles, and points along the look vector direction. A Velocity Follow Camera follows behind any object that can tumble aligns with the velocity vector. A Drone Camera tracks objects from above.

**Python Bindings API**

- Added

    - Use the PhysXAddFollowCameraCommand, PhysXAddVelocityCameraCommand and PhysXAddDroneCameraCommand to add cameras that respond the Undo and Redo commands. Use the get_camera_interface to get the camera interface, get_update_all_cameras, set_update_all_cameras, add_follow_camera and add_drone_camera methods to add cameras in Python along with the attach_stage, detach_stage, update_controllers and process_pending_usd_changes methods to run the camera without pressing Play in Kit.

**C++ API**

- Added

    - Use the getUpdateAllCameras, setUpdateAllCameras, addFollowCamera, addVelocityCamera and addDroneCamera methods to add cameras in Python along with the attachStage, detachStage, updateControllers and processUSDNotifyUpdates methods to run the camera without pressing Play in Kit.

### Character Controller

**General**

- Fixed
   - Character capsule pose does reset correctly when stop is pressed.


## [1.3.15-5.1] - 2021-12-??

### Core

**General**

- Fixed

   - Omni.physx.ui initialization if viewport window failed to be initialized.
   - Pseudoroot resync USD notification results in a full physics reparse, fixes save as functionality while simulation is running.
   - CCT requirement check prim validation check added.
   - Physics debug view synchronization of visualization flags fixed.


## [1.3.14-5.1] - 2021-11-05

### Core

**General**

- Fixed

   - Two way coupling between softbodies and articulations.
   - Removed slowdown of stage update by Fixed-Tendon Debug Visualizer on stages with many prims
   - Possible asynchronous cooking threading issue.


## [1.3.13-5.1] - 2021-10-15

### Core

**General**

- Fixed

   - RigidBody Max linear velocity default value was changed to infinity.
   - Kinematic rigid bodies without shapes dont trigger mass related warnings.
   - ArticulationRootAPI defined on a static rigid body will get ignored and a warning is send to the user.
   - Removed incorrect layer check for transformation sanitation.
   - Added Deformable Hand and Deformable Poisson Ratio demo into omni.physx.preview extension.
   - Fixed mass computation for convex decomposition approximation.


## [1.3.12-5.1] - 2021-10-05

### Core

**General**

- Fixed

   - Fixed PhysX SDK debug visualization for articulations.
   - Setting joint body rel will recompute joint poses.
   - Fixed collider being applied to bodies hidden from the stage window.
   - Rigid body transformation setup on prims in a layer without edit spec will issue warning.
   - Exposed new triangle mesh collision parameter - Weld tolerance used for PhysX Cooking welding.



## [1.3.11-5.1] - 2021-09-22

### Core

**General**

- Fixed

   - PhysX SDK issue related to aggregates broadphase.
   - PhysX SDK issue related to articulations sleeping.
   - Added validity check for newly added prims before they are processed in physics update.


## [1.3.10-5.1] - 2021-09-01

### Core

**General**

- Changed

   - PhysX GPU Cuda context manager crash will stop the simulation. Next simulation will attempt to recreate the Cuda context manager or fall back to CPU.
   - Fixed joint authoring visualization when joint gizmos were disabled.
   - Articulation D6 joint is not anymore internally decomposed into several revolute joints,
   one spherical joint is used instead. Translate DOF is not supported in the D6 joint.
   - Physics flatcache update fixed a crash with a scene reload.
   - Fixed crash in Softbody debug visualization when stage got released.

## [1.3.9-5.1] - 2021-08-22

### Core

**General**

- Fixed

   - Fixed crash bug when deformable bodies come in contact with kinematic rigid bodies.
   - Improved deformable body collision behavior at small scales.
   - Crash if point instancer index was out of prototype bounds.
   - Improved PhysX SDK determinism.
   - Clamping values in physics preferences after ctrl+clicking a slider and entering a value outside of valid range.

- Changed

   - Convex hull mesh thickness now does not produce a large hull if the original mesh was very tiny.


## [1.3.8-5.1] - 2021-08-10

### Core

**General**

- Added

   - Backward compatibility on a folder GUI improvements.

- Fixed

   - Backward compatibility on a folder now supports paths on a nucleus server and will not crash on read-only files.
   - Collision debug visualization now skips camera changes.
   - Contact report lost is send when objects get deleted.
   - Joints created with instanceable objects dont issue python errors.

## [1.3.7-5.1] - 2021-08-05

### Core

**General**

- Fixed

   - Added missing IPhysXCooking interface dependency for omni.physx.ui extension.
   - Any prim can be now chosen as a Filtered pairs target.
   - Physics numeric property widgets will not call listeners on each digit change.
   - Added additional prim validation for range traversals.
   - CCT's MoveTarget attribute is now reset to its initial value when simulation stops.
   - Fixed articulation parsing for disabled joints.
   - Property widgets min/max and step values are now scaled with stage's metersPerUnit value where appropriate.
   - Default state UI will now reset to metersPerUnit scaled value instead of the original one and show correctly when the value is changed from default.

- Changed

   - Triangle mesh collisions are not supported for dynamic rigid bodies by PhysX simulation. Current error was replaced by a warning and collision approximation will fall back to a convexHull approximation.
   - Physics debug visualization does temporary disable async rendering.

- Added

   - Exposed convex mesh cooking functions to python. Its now possible to get resulting convex cooking data in runtime to python. Added sample for this functionality.
   - Improved collision between deformable bodies, self-collision, and collision versus articulations.

- Changed

   - Replaced mesh simplification approximation for colliders with better algorithm, and simplified parameters.

### Vehicles

**General**


## [1.3.6-5.1] - 2021-07-16

### Core

**General**

- Added

   - Process backward compatibility on a folder. Access from Preferences/Physics.

### Vehicles

**General**

- Fixed

   - Deleting a prim with PhysxVehicleWheelAttachmentAPI applied while in play mode could cause a crash.
   - Vehicles referenced into a stage during play did not work properly if the referenced prim was an ascendant of the vehicle prim (device input to control the vehicle was ignored, for example).

- Changed

   - For vehicles with standard drive, setting the transmission to automatic will be ignored if no auto gear box has been defined. In addition, an error message will get sent.
   - The property window will hide certain relationships if the corresponding API schemas have been applied instead.


## [1.3.5-5.1] - 2021-07-08

### Core

**General**

- Added

   - Improved Deformable Body collision mesh simplification. Better mesh simplification algorithm and parameterization, support for remeshing and faster tetrahedral mesh generation.

### Vehicles

**General**

- Fixed

   - The Vehicle Creation Wizard kept the forward axis option locked even if an undo removed the PhysxVehicleContextAPI from the physics scene prim.

- Changed

   - Negative max steer angles are supported now. This allows to create vehicles with a very small turning radius, for example.

### Character Controller

**General**

## [1.3.4-5.1] - 2021-07-02

### Core

**General**

- Added

   - Adding Deformable Body component during play
   - Backward compatibility check now reports a list of deprecated attributes and types. This is shown either in the backward compatibility dialog or in console, depending on settings.
   - IPhysxCooking interface - moved cooking related function from IPhysx and marked them deprecated on IPhysx.
   - Exposed CreateConvexMesh function on IPhysxCooking.

- Changed

   - Joint visualization is disabled on play and reappear when stopped is pressed (did appear on pause before).
   - Removed "Command" from command execution strings.

- Fixed

   - Gear joint and rack and pinion joint create order. If the joint was created before the required revolute/prismatic joint, application would crash.
   - Issue with single body articulation.
   - Property window refresh when APIApply commands were processed was not working sometimes (e.g. when some components were added or removed).
   - Crash when material prim did not existed.

### Vehicles

**General**

- Fixed

   - The Vehicle Creation Wizard did not position the wheels correctly if the scan feature was used for the chassis dimensions but not for the wheel dimensions.
   - Small scale vehicles generated via the Vehicle Creation Wizard did fall asleep too easily.

- Changed

   - The Vehicle Creation Wizard now adjusts the contact offset values for collision geometry based on the vehicle size.


## [1.3.3-5.1] - 2021-06-22

### Core

**General**

- Changed

   - Using custom PhysxSchemaTokens->physxCollisionCustomGeometry for custom geometry definition.
   - Default restOffset is -inf - value is computed automatically, its 0.0f for rigid bodies and 2.0cm for deformables (For small deformables, tweaking the rest offset might be required).
   - Default solver iteration count for rigid bodies was increased from 4 to 16.
   - Improved bounding sphere for symmetric objects.

- Fixed

   - Issue with materials and convexDecomposition colliders.
   - RigidBody enabled attribute can be changed in runtime and joints will correctly update.

- Added

   - Deformable CCD (enabled by default when a mesh is set as a deformable)
   - Adding Deformable Body component during play

### Vehicles

**General**

- Fixed

   - The auto-reverse logic could fail due to the vehicle rigid body falling asleep and it looked like the vehicle got stuck.

- Changed

   - The Vehicle Creation Wizard command does not open message dialog windows anymore to provide warning or error feedback. Instead, a list of message strings is returned.
   - The Vehicle Creation Wizard has changed thresholds such that it will warn more aggressively about setups that will likely need more simulation steps per second to be stable.

**General**

## [1.3.2-5.1] - 2021-06-15

### Core

**General**

   - Added

      - Overlap mesh query with a python demo.
      - Omni.physx.preview extension to showcase PhysX SDK technology that is not yet fully integrated.
      - Deformable Body collision mesh simiplification. Can be controlled through Deformable Body property widget.

   - Changed

      - Tendons authoring improvements.
      - Physics material does search for purpose physics token, then it falls back to non purpose binding.
      - Deformable Body collision and simulation mesh visualization improvements.

   - Fixed

      - Debug visualization for triangle meshes with various scales was incorrect.
      - Debug visualization for triangle mesh normals with a negative scale was incorrect.
      - Point instancer reset transformations when velocities arrays were not provided.
      - Articulation definition now takes a joint to world correctly as a fixed based articulation.

   - Removed

      - Create Deformable Body dialog. Use component add button instead.
      - Create Deformable Material, was replaced by create physics material.

**Python API**
   - Changed
      - omni.physx.utils
         - setCollider will correct approximation shape from none (trimesh) to convexHull when the prim is a part of a rigid body.

### Vehicles

**General**

- Added

   - Revolutions per minute can now be specified in the Vehicle Creation Wizard for standard drive vehicles. This gives a bit more control over top speed of the created vehicle.

- Fixed

   - The wheel mass field in the Vehicle Creation Wizard discarded changes unless some other fields were changed first.

- Changed

   - The Vehicle Creation Wizard has been modified to avoid some instabilities if small vehicles are created. As a consequence, the generated parameters might vary a lot compared to the previous version.
   - The tire prims generated by the Vehicle Creation Wizard are now placed under the vehicle root prim instead of under the shared data root prim.

**USD Schemas**

- Changed

   - The required APIs that need to be applied to prims such that they can be used as vehicle components have changed.

      - vehicle prim: instead of PhysxRigidBodyAPI, the existence of UsdPhysics RigidBodyAPI is verified.
      - vehicle wheel collision prim: instead of PhysxCollisionAPI, the existence of UsdPhysics CollisionAPI is verified.
      - tire friction table material prims: instead of PhysxMaterialAPI, the existence of UsdPhysics MaterialAPI is verified.

**Python Bindings API**

- Changed

   - The helper method compute_wheel_simulation_transforms() (computeWheelSimulationTransforms() in C++) sets the tire force application point offset close to the wheel center now.

### Character Controller

**General**

   - Fixed

      - Character controller capsule orientation fix

## [1.3.1-5.1] - 2021-06-04

### Core

**General**
   - Changed

      - Raycast/Sweep all demo does create three pyramids by default, the number of pyramids and the size was added as a parameter
      - Character controller API takes move target as a parameter. Improved Character controller snippet.
      - PhysxScene solver type TGS is now default.
      - PhysxCookedDataAPI was changed to be multiple applied API. This changed together with the PhysxMeshCollisionAPI changes
      ensure that multiple mesh approximations can be authored and precomputed resulting in fast collision switch in runtime.

   - Added

      - Added RigidBody Enabled Gyroscopic Force flag, it is enabled by default. (flag is in advanced section).
      - Softbodies
      - Articulation tendons
      - Joint break event into simulation event stream, added a snippet Joint Break to showcase feature.
      - Default material can be applied to PhysicsScene, Restitution Demo was modified to showcase this usage.

   - Removed

      - PhysxMeshCollisionAPI was removed. It was replaced by PhysxConvexHullCollisionAPI, PhysxConvexDecompositionCollisionAPI and
      PhysxTriangleMeshSimplificationCollisionAPI. Based on the UsdPhysicsMeshCollisionAPI:approximation attribute the matching schema
      is used. Base on the approximation only those relevant attributes show in the property window. The irrelevant attributes are
      in the inactive group.


### Vehicles

**General**

- Fixed

   - The vehicle creation wizard transformed the horsepower field to values of engine peak torque that were too high.

### Character Controller

**General**

- Added

   - Character controller collision event steam.

## [1.3.0-5.1] - 2021-05-28

### Core

**General**
   - Added

      - Reset Physics Settings button to reset all physics settings to their default values.
      - Joint localPose can be changed in runtime, time varied attribute changes are supported.
      - Added support for articulation force sensors.
      - Omni.physx.flatcache extension for fast rigid body transformation updates.

   - Fixed

      - ConvexDecomposition collision removal in runtime does now correctly trigger mass recomputation.

   - Remove

      - Fast cache option was removed. It was replaced by omni.physx.flatcache extension.

**Python API**
   - Added
      - omni.physx.utils
         - addDeformableBodyMaterial, addDeformableSurfaceMaterial.

      - omni.physxcommands
         - AddDeformableBodyMaterialCommand, AddDeformableSurfaceMaterialCommand.

   - Changed
      - omni.physx.utils
         - addMaterial renamed to addRigidBodyMaterial, material params are now optional.

### Vehicles

**General**

- Fixed

   - Optional USD relationships were falsely treated as an invalid setup, if they were authored but set to an empty list.
   - On simulation stop, attribute values of PhysxVehicleControllerAPI and PhysxVehicleWheelControllerAPI prims were not reset to the pre-simulation-start values.

**USD Schemas**

- Added

   - The attribute sprungMass has been added to the PhysxVehicleSuspensionAPI API schema and allows the user to set custom values if desired (by default, the values will be computed automatically based on vehicle mass and wheel center-of-mass offsets).



## [1.2.0-5.1] - 2021-05-05

### Core

**General**

- Added

   - RigidBody CCD demo.
   - RigidBody Kinematics demo.
   - Error message if a distance joint should be a part of an articulation.
   - Rigid Body and Collider to Sub-tree in the Property window's Add menu.
   - RigidBody retain accelerations flag exposed. Rigid body carry over forces/accelerations between frames, rather than clearing them.
   - Option to explicitly create a D6 joint.
   - Support for transformation changes through flat cache.
   - TimeStepsPerSeconds simulation parameter per scene. The final number of steps is based on this per scene value and simulation min frame rate persistent setting.
   - Setting to automatically create a temporary default PhysicsScene when simulation is started and none is available.
   - Support for RigidBody point instancer position/orientation/velocity/angularVelocity changes in runtime.
   - Multiple materials per triangle mesh support through UsdGeomSubset - demo added.

- Fixed

   - StartAsleep was not working correctly.
   - ContactOffset, restOffset runtime changes were not correctly updated.
   - Joint authoring X trans axis was not visible.
   - Joints/Articulation are now created with consistent order.
   - RackAndPinionJoint and GearJoint are now properly created.
   - ArticulationJoint changes in runtime.
   - Soft vs hard joint limits.
   - Static rigid body with a triangle mesh.
   - Point instanced rigid bodies initial velocity reset.
   - Static body is considered for joint creation.
   - Articulation parsing optimizations.
   - Physics objects are correctly removed when a prim is deleted in runtime.
   - Contact reports for Point Instancers.
   - Joint helpers movement when parent xform moved.
   - Debug visualization now respects collisionEnabled attribute.

- Changed

   - Physics demos are now in a separate extension.
   - Physics commands are now in a separate extension.
   - Physics schema is now in a separate extension.
   - Hidden "Asynchronous Simulate/Render" setting UI if not in Physics Development Mode.
   - Applying/adding colliders (through components, presets or scripting) now also work with instanced prims.
   - Issue warning and not create rigid body on an instance proxy.
   - Optimized NoticeHandler in favor of Flat Cache integration.
   - Physics Settings (except PVD) moved to the main Preferences window. PVD settings moved to the Physics Debug window.
   - Joint helpers are displayed only for 100 closest joints.
   - Instead of running a full upgrade process a backward compatibility check on scene open is now enabled by default and a prompt will be presented to run the upgrade on demand. This check can be disabled, only warn or run the upgrade without prompting in Preferences.
   - /persistent/physics/backwardCompatibilityCheck setting has been renamed to backwardCompatibilityCheckMode to indicate the aformentioned change in functionality.

**Python API**

- Added

   - omni.physx
      - reset_settings() will reset all physics settings to their default values.

- Changed

   - omni.physx.utils
      - setCollider, setColliderSubtree and removeCollider now also work with instanced prims.

**C++ API**

- Added

   - IPhysx.h
      - resetSettings() will reset all physics settings to their default values.

### Vehicles

**General**

- Added

   - Some debug visualization for vehicle suspension has been added. See the new option in the Physics Debug window.
   - Sample for how to use vehicles to set up a semi-trailer truck.
   - The PhysxVehicleContextAPI schema can be applied to a UsdPhysicsScene prim through the Add button in the property window.
   - Tire friction tables (PhysxVehicleTireFrictionTable) can now be added through the Create menu.
   - The Vehicle Creation Wizard scales physics settings so smaller vehicles are stable down to centimeter scales.

- Fixed
   - Deletion of vehicles after simulation start can now happen through a parent prim (so far, explicit deletion of the vehicle prims was required).

- Changed

   - The API schemas of the vehicle USD representation have undergone some major changes. Please read the USD Schemas section.
   - Due to the changes mentioned above, the vehicle creation wizard outputs are more compact representation by default now. To get the previous output, where all vehicle components have their own prim, an option has been added to the first page (Create Shareable Components).
   - It is now allowed to add tire friction tables (PhysxVehicleTireFrictionTable) after the simulation has started.
   - The Vehicle Creation Wizard reads the forward and up vehicle axes from the VehicleContextAPI prim, if one exists. Otherwise, the up axis is read from the stage and the forward axis can be specified in the wizard.

**USD Schemas**

- Changed

   - Many IsA schemas have been converted to API schemas. This allows for a more compact (fewer prims) vehicle setup for cases where there is no intent to share vehicle components among vehicle instances. Please have a look at the vehicle setup samples for examples. The following IsA schemas have been converted to API schemas:

      - PhysxVehicleGlobalSettings => PhysxVehicleContextAPI. In addition, this API has to be applied to a PhysicsScene prim.
      - The following changes add the option to apply the APIs directly to the prim that has PhysxVehicleWheelAttachmentAPI applied. The relationships to wheel, tire and suspension prim can then be omitted.

         - PhysxVehicleWheel => PhysxVehicleWheelAPI
         - PhysxVehicleTire => PhysxVehicleTireAPI
         - PhysxVehicleSuspension => PhysxVehicleSuspensionAPI

      - The following changes add the option to apply the APIs directly to the prim that has PhysxVehicleDriveStandardAPI applied. The relationships to engine, gears, auto-gear-box and clutch prim can then be omitted.

         - PhysxVehicleEngine => PhysxVehicleEngineAPI
         - PhysxVehicleGears => PhysxVehicleGearsAPI
         - PhysxVehicleAutoGearBox => PhysxVehicleAutoGearBoxAPI
         - PhysxVehicleClutch => PhysxVehicleClutchAPI

      - The following changes add the option to apply the APIs directly to the prim that has PhysxVehicleAPI applied. The relationship to the drive prim can then be omitted.

         - PhysxVehicleDriveBasic => PhysxVehicleDriveBasicAPI
         - PhysxVehicleDriveStandard => PhysxVehicleDriveStandardAPI

    - It is now legal to not define a tire friction table relationship in PhysxVehicleTireAPI. In such a case, any available table will be used (and if none exists, a default will be provided). It is still highly recommended to define a tire friction table and set the relationship but this change allows to reference a vehicle asset after the simulation has started and postpone the tire friction table setup.

**Python Bindings API**

- Added

   - The methods set_vehicle_visualization() and get_vehicle_visualization() can be used to enable debug visualization for vehicles (note: this is part of the omni.physx.ui extension).
   - To support add/remove of vehicles while explicitly stepping the simulation, the process_pending_usd_changes() method has been introduced.
   - The helper method compute_sprung_masses() has been introduced to compute sprung masses for a given wheel arrangement.

**C++ API**

- Added

   - The methods setVehicleVisualization() and getVehicleVisualization() can be used to enable debug visualization for vehicles (note: this is part of the omni.physx.ui extension).
   - The helper method computeSprungMasses() has been introduced to compute sprung masses for a given wheel arrangement.


## [1.1.0-5.1] - 2021-03-17

### General
- Fixed

   - Fixed RigidBody CCD


## [1.1.0-5.1] - 2021-02-05

### General

- Added

   - IPhysxSceneQuery::raycastAll function to support multiple hits per raycast, hits are reported through a hit callback
   RaycastAll demo was added to the physics samples. Python bindings for this function are exposed.
   - IPhysxSceneQuery::sweepSphereAll function to support multiple hits per sphere sweep, hits are reported through a hit callback
   SweepSphereAll demo was added to the physics samples. Python bindings for this function are exposed.
   - ISimulationCallback and SimulationFlag were added to the IPhysxSimulation interface. It is possible to register a callback and get notified about physics transformations. (C++ code only)
   - Point instancer support for shapes only, while rigid body API is on the instancer itself or above in hierarchy
   - Support for mesh winding orientation, please check debug visualization for triangle normal orientation
   - PhysX PVD connection settings exposed in Physics setting menu
   - Disable button in the character controller (CCT) window to disable CCT on a Capsule without the need to remove it
   - Pair Collision Filter removal added to stage and viewport context menus under Physics/Remove Preset

- Fixed

   - Removing convexDecomposition approximation, shapes are properly released from PhysX SDK.
   - Async simulation when provided timestep was smaller than 1.0/60.0
   - Point instancer with static rigid bodies
   - Articulation parsing fixes - better determination of articulation root, fixes when bodies 0,1 were in different order
   - Joint visibility fixed
   - Primitives under joints can be now selected
   - Triggers with script buffer type dont try to preload a script
   - Simulation events are not send from async thread, resolves issues when stepping manually from python
   - Issues with transformation matrix and scale during physics simulation initialization
   - KaplaArena sample does not collapse anymore
   - Removing shapes from compounds in runtime properly
   - Removing a character controller (CCT) enabled Capsule will properly remove it from the CCT manager and disable CCT camera


- Changed

   - IPhysx::subscribeToPhysicsStepEvents and IPhysx::unsubscribeToPhysicsStepEvents were renamed to subscribePhysicsStepEvents/unsubscribePhysicsStepEvents to match python names
   - IPhysx::subscribeToPhysicsSimulationEvents and IPhysx::unsubscribeToPhysicsSimulationEvents were renamed to subscribePhysicsSimulationEvents/unsubscribePhysicsSimulationEvents
   - omni.physx.core renamed to omni.physx.bundle
   - Centralized base geometry creation utilities into the omni.physx.scripts.physicsUtils module
   - Physics/Create main menu items moved to stage and viewport context menus under Create/Physics and to the Create/Physics main menu
   - Physics/Set and Physics/Remove main menu items moved to stage and viewport context menus under Physics/Apply Preset and Physics/Remove Preset

### Vehicle changes

#### General

- Added

   - Vehicles can be added and removed while the simulation is running using a script. There are certain restrictions about how this is done. Please refer to the documentation for details.
   - Support for sweep queries has been added to detect collision with the ground surface.

- Changed

   - The auto-reverse feature will not get applied anymore if accelerator and brake are controlled through USD directly. The brake value will always be treated as brake and the accelerator value always as accelerator. If a vehicle is controlled through device input (keyboard, gamepad etc.), auto-reverse should behave the same way as before.

#### USD Schemas

- Added

   - To support sweep queries, the attributes sweepRadiusScale, sweepWidthScale and suspensionLineQueryType have been added (see classes PhysxVehicleGlobalSettings and PhysxVehicleAPI).

- Removed

   - The attributes physxVehicleController:shiftUp and physxVehicleController:shiftDown have been removed. Please use physxVehicleController:targetGear instead (see class PhysxVehicleControllerAPI).

#### Python Bindings API

- Changed

   - The get_drive_data API call now follows the Vehicle USD schema definition for steerLeft and steerRight as well as gear numbering (reverse = -1, neutral = 0 etc.).


## [1.0.0-5.1] - 2020-10-28

### General

#### Changed:

   - PhysX SDK 5.1 integrated
   - Update UsdPhysics and PhysxSchema to use default values
   - Physics menu Add was renamed to Create
   - Physics settings updated to new omni.ui
   - Physics properties window replaced by omni.kit.property.physx
   - Update USD velocities is enabled by default
   - Interfaces renamed:
      - carb::physics::usd::Physics.h --> omni::physx::PhysxUsd.h
      - carb::physics::IPhysicsAuthoring.h --> omni::physx::IPhysicsAuthoring.h
      - carb::physics::IPhysicsSimulation.h --> omni::physx::IPhysicsSimulation.h
      - carb::physics::IPhysicsUsdLoad.h --> omni::physx::IPhysxUsdLoad.h
      - carb::physics::physx.h --> omni::physx::IPhysx.h
      - carb::physics::physxInternal.h --> omni::physx::IPhysxCooking.h
      - carb::physics::physxSceneQuery.h --> omni::physx::IPhysxSceneQuery.h
      - carb::physics::physxSceneQueryHit.h --> omni::physx::IPhysxSceneQuery.h
      - carb::physics::physxUI.h --> omni::physx::IPhysxUI.h
   - Python bindings moved to a subfolder bindings

#### Added:

   - Addeed IPhysicsSimulation
   - Physics cooked meshes local cache
   - Asynchronous mesh cooking
   - Cached getters for omni.physx interfaces
   - Physics demos can be now opened with a double click
   - Exposed rigid body transformation through an API, exposed to Python
   - Added overlap_box and overlap_sphere functions, exposed to Python though new PhysXSceneQuery interface, demos added.
   - Pvd IP settings and profile/debug/memory instrumentation checkbox

#### Fixed:

   - Regression in USD notice handlers when an API was applied in runtime
   - Fixed traversal to support scene graph instancing

#### Removed:

   - Raycast_closest and sweep_closest from physx interface, moved into PhysXSceneQuery interface
   - PSCL demos were removed, separate extension with these demos was created, they will be published to extension repository

### Python Bindings API

#### Added:

   - New PhysXSceneQuery interface. Overlap_sphere, overlap_box together with existing raycast_closest and sweep_closest (the query functions on PhysX interface were marked as deprecated).

### Vehicle changes

#### General

- Added

   - The recommended way to fetch the python vehicle interface is now to use omni.physxvehicle.get_physx_vehicle_interface().

- Changed

   - Fixed a bug which reset vehicle Inputs Enabled every time the simulation started.
   - Fixed a bug that made the vehicle properties window unresponsive in certain cases while in play mode.
   - Renamed interface carb::physics::physxVehicle.h --> omni::physx::IPhysxVehicle.h
   - The vehicle specific property window has been removed. The functionality is now part of the common property window.
   - Python bindings moved to a subfolder bindings

#### C++ API

- Changed:

   - The vehicle object ID parameter has been removed from getWheelIndex() as it is no longer needed.

#### USD Schemas

- Added:

   - The attribute vehicleEnabled has been added to the PhysxVehicleAPI. If false, the vehicle simulation update loop will not run for a vehicle.

- Removed

   - The sprungMass parameter of PhysxVehicleSuspension has been removed. The values will be automatically computed based on the mass parameter of the MassAPI schema as well as the wheelCenterOfMassOffset parameters of the PhysxVehicleWheelAttachmentAPI schema.


#### Vehicle Creation Wizard

- Added:

   - Added a Scan button to each page of the Vehicle Wizard that fits a bounding box around the selected prims
     in order to measure the Length, Width and Height of the mesh or shape. This information is used on the Basic
     page to find the dimensions of the chassis and compute the forward direction and vehicle mass. On the Axles
     page, the dimensions are used to position each of the tires and measure their width and radius. This feature
     makes it much faster to set up a vehicle.
   - Added a forward axis drop down to make it easier to orient the vehicle.
   - Added position inputs for the chassis so it can be positioned around the vehicle mesh. The Scan button will
     set the position to the center of the bounding box.
   - Added a third page of documentation to provide some of the "Next Steps" to be completed to finish setting up
     a vehicle.

- Changed:

   - All of the vehicle components are now created as children of the defaultPrim.
   - Fixed the calculation of the tire and suspension force positions and position the center of mass as a function
     of the weight distribution settings.
   - Moved the default locations of the tires inboard from the edge of the vehicle.
   - Tires can now be positioned, and tire radii and widths can be set for each tire individually instead of
     on a per axle basis.
   - Initialized the tire radii and width as a function of the chassis length so tires are not too large or small.

#### Python Bindings API

- Added:

   - Two new methods, get_update_all_cameras and set_update_all_cameras(bool updateAllCameras) were added.
     When updateAllCameras is true, all vehicle cameras are updated continuously, which allows seamless cuts
     from camera to camera. However, this can lower performance for stages with many cameras. To increase
     performance, set updateAllCameras to false. In this state, only the active vehicle camera is updated,
     but switching to a new camera will cause the new camera to reinitialize.
   - attach_stage(), detach_stage() and update_controllers() have been added to the vehicle interface and
     can be used to update the vehicle controllers explicitly (instead of being triggered by the stage
     update loop)

- Changed:

   - The compute_wheel_simulation_transforms API call now positions the tire forces at the bottom of the
     tire, where the rubber meets the road, instead of at the center of the tire. Since the
     Vehicle Authoring transforms autocompute tool calls this API function, it also takes advantage of this
     change.

- Removed:

   - create_controllers() and update_controllers() have been removed from the vehicle testing interface. The
     new attach_stage(), update_controllers() etc. on the vehicle interface can be used as a replacement.

### Omni PhysX Vehicle pre 1.0.0

#### General

- Added:

   - Support for PhysX vehicles
   - Default input device support to drive a vehicle
   - Custom camera types
   - Special property windows
   - Vehicle creation wizard
   - Samples

#### Python Bindings API

- Changed:

   - add_follow_camera() and add_drone_camera() now require a second argument specifying the USD path for the
     camera to create. Furthermore, True/False is returned to indicate success/failure of the operation.
