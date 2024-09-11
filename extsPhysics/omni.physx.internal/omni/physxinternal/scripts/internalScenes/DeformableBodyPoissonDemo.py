import math
import omni
import os
from omni.physx.scripts import deformableUtils, physicsUtils, utils
import omni.physxdemos as demo
import omni.physx.bindings._physx as physx_settings_bindings
import numpy as np

from pxr import UsdGeom, UsdLux, Gf, PhysxSchema, Usd, UsdPhysics


class DeformableBodyPoissonDemo(demo.AsyncDemoBase):
    title = "Deformable-body Poisson Ratio"
    category = demo.Categories.INTERNAL_SAMPLES
    short_description = "Stretching and twisting a deformable body"
    description = (
        "Demo of the Poisson ratio deformable-body material property that causes the body to thin when it is stretched."
    )

    kit_settings = {
        "persistent/app/viewport/displayOptions": demo.get_viewport_minimal_display_options_int(),
        physx_settings_bindings.SETTING_MIN_FRAME_RATE: 60,
        physx_settings_bindings.SETTING_UPDATE_VELOCITIES_TO_USD: False,
        physx_settings_bindings.SETTING_UPDATE_TO_USD: True,
    }

    def __init__(self):
        super().__init__(enable_tensor_api=True, enable_fabric=True)

    def on_physics_step(self, dt):
        self._time += dt
        phase = -math.pi / 2.0
        amp = 125
        twistAmp = 0.33 * math.pi
        freqHz = 0.2
        currentTwistAngle = self._time * freqHz * 2.0 * math.pi
        currentAngle = currentTwistAngle + phase
        doTwist = (currentTwistAngle % (4.0 * math.pi)) > (2.0 * math.pi)
        currentTwist = 0.0
        if doTwist:
            amp = 100
            currentTwist = twistAmp * (math.sin(currentAngle) - math.sin(phase))
        currentPosition = self._movingRBinitialX + amp * (math.sin(currentAngle) - math.sin(phase))
        movingTransform = np.zeros((1, 7), dtype=np.float32)
        self._currentPos = Gf.Vec3f((currentPosition, 0.0, self._bodyHeight))
        movingTransform[0][:3] = np.array((currentPosition, 0.0, self._bodyHeight), dtype=np.float32)
        rot = Gf.Rotation(Gf.Vec3d(1.0, 0.0, 0.0), currentTwist * 180.0 / math.pi)
        q = Gf.Quatf(rot.GetQuat())
        movingTransform[0][3:6] = np.array(q.GetImaginary(), dtype=np.float32)
        movingTransform[0][6] = q.GetReal()
        self._movingRB.set_kinematic_targets(movingTransform, self._movingRBind)

    def on_tensor_start(self, tensorApi):
        sim = tensorApi.create_simulation_view("numpy")
        self._movingRB = sim.create_rigid_body_view("/World/MovingBody")
        assert self._movingRB.count == 1
        self._movingRBind = np.arange(self._movingRB.count, dtype=np.int32)

        # prevent garbage collector from deleting the sim 
        self.sim = sim

    def on_shutdown(self):
        self._movingRB = None
        self.sim = None
        super().on_shutdown()

    def create(self, stage):
        # setup stage and scene:
        self._stage = stage
        UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
        unitsPerMeter = 100.0
        UsdGeom.SetStageMetersPerUnit(stage, 1.0 / unitsPerMeter)
        self._time = 0.0
        defaultPrimPath = stage.GetDefaultPrim().GetPath()

        # Physics scene
        scene = UsdPhysics.Scene.Define(stage, defaultPrimPath.AppendChild("physicsScene"))
        scene.CreateGravityDirectionAttr().Set(Gf.Vec3f(0.0, 0.0, -1.0))
        # disable gravity
        scene.CreateGravityMagnitudeAttr().Set(0.0)
        utils.set_physics_scene_asyncsimrender(scene.GetPrim())

        # stage geometries and visuals:
        self._bodyHeight = 150
        deformableXsize = 100
        deformableYZsize = 0.5 * deformableXsize
        rigidYZsize = deformableXsize
        rigidXsize = 0.3 * deformableXsize
        overlapDistance = rigidXsize * 0.25
        deformableColor = Gf.Vec3f(100.0 / 255.0, 100.0 / 255.0, 150.0 / 255.0)
        rigidColor = Gf.Vec3f(1.0, 165.0 / 255.0, 71.0 / 255.0)

        # light
        sphereLight = UsdLux.SphereLight.Define(stage, defaultPrimPath.AppendChild("SphereLight"))
        sphereLight.CreateRadiusAttr(150)
        sphereLight.CreateIntensityAttr(30000)
        sphereLight.AddTranslateOp().Set(Gf.Vec3f(-100.0, 0.0, 1150.0))

        # Create kinematic bodies to attach to:
        rbSize = Gf.Vec3f(rigidXsize, rigidYZsize, rigidYZsize)
        staticRbPos = Gf.Vec3f(0.0, 0.0, self._bodyHeight)
        self._movingRBinitialX = deformableXsize + 2.0 * (0.5 * rigidXsize - overlapDistance)
        self._movingRbInitPos = Gf.Vec3f(self._movingRBinitialX, 0.0, self._bodyHeight)
        fixedRBprim = physicsUtils.add_rigid_box(
            stage, defaultPrimPath.AppendChild("StaticBody"), size=rbSize, position=staticRbPos, color=rigidColor
        )
        rigidBodyAPI = UsdPhysics.RigidBodyAPI.Apply(fixedRBprim)
        rigidBodyAPI.CreateKinematicEnabledAttr(True)
        movingRBprim = physicsUtils.add_rigid_box(
            stage, defaultPrimPath.AppendPath("MovingBody"), size=rbSize, position=self._movingRbInitPos, color=rigidColor
        )
        rigidBodyAPI = UsdPhysics.RigidBodyAPI.Apply(movingRBprim)
        rigidBodyAPI.CreateKinematicEnabledAttr(True)
        self._movingRBxformable = UsdGeom.Xformable(movingRBprim)

        # create soft beam
        beamPath = defaultPrimPath.AppendChild("DeformableBeam")
        data_folder = os.path.normpath(os.path.join(os.path.dirname(os.path.abspath(__file__)), "../../../.."))
        box_high_path = os.path.normpath(data_folder + "/data/usd/assets/box_high.usda")
        stage.DefinePrim(beamPath).GetReferences().AddReference(box_high_path)
        skinMesh = UsdGeom.Mesh.Define(stage, beamPath)
        dbXpos = 0.5 * (deformableXsize + rigidXsize) - overlapDistance
        skinMesh.AddTranslateOp().Set(Gf.Vec3f(dbXpos, 0, self._bodyHeight))
        skinMesh.AddOrientOp().Set(Gf.Quatf(1.0))
        skinMesh.AddScaleOp().Set(Gf.Vec3f(deformableXsize, deformableYZsize, deformableYZsize))
        skinMesh.CreateDisplayColorAttr([deformableColor])

        omni.kit.commands.execute(
            "AddDeformableBodyComponent", skin_mesh_path=beamPath, voxel_resolution=20, collision_simplification=True
        )

        deformable_material_path = defaultPrimPath.AppendChild("deformableBodyMaterial")
        deformableUtils.add_deformable_body_material(
            stage,
            deformable_material_path,
            poissons_ratio=0.499,
            # the poisson-ratio effect is only visible for low Young's in the current SDK (5.1 trunk @ 30351535)
            youngs_modulus=20e2,
        )
        physicsUtils.add_physics_material_to_prim(stage, skinMesh.GetPrim(), deformable_material_path)

        # create attachments:
        attachment = PhysxSchema.PhysxPhysicsAttachment.Define(stage, beamPath.AppendChild("attachmentToStatic"))
        attachment.GetActor0Rel().SetTargets([beamPath])
        attachment.GetActor1Rel().SetTargets([fixedRBprim.GetPath()])
        PhysxSchema.PhysxAutoAttachmentAPI.Apply(attachment.GetPrim())

        attachment = PhysxSchema.PhysxPhysicsAttachment.Define(stage, beamPath.AppendChild("attachmentToMoving"))
        attachment.GetActor0Rel().SetTargets([beamPath])
        attachment.GetActor1Rel().SetTargets([movingRBprim.GetPath()])
        PhysxSchema.PhysxAutoAttachmentAPI.Apply(attachment.GetPrim())

        scenePath = scene.GetPrim().GetPath()
        deformableGroupPath = scenePath.AppendChild("collisionGroupDeformableBody")
        rigidBodyGroupPath = scenePath.AppendChild("collisionGroupRigids")
        deformableGroup = UsdPhysics.CollisionGroup.Define(stage, deformableGroupPath)
        rigidBodyGroup = UsdPhysics.CollisionGroup.Define(stage, rigidBodyGroupPath)

        filteredRel = deformableGroup.CreateFilteredGroupsRel()
        filteredRel.AddTarget(rigidBodyGroupPath)

        filteredRel = rigidBodyGroup.CreateFilteredGroupsRel()
        filteredRel.AddTarget(deformableGroupPath)

        collectionAPI = Usd.CollectionAPI.Apply(rigidBodyGroup.GetPrim(), "colliders")
        includesRel = collectionAPI.CreateIncludesRel()
        includesRel.AddTarget(fixedRBprim.GetPath())
        includesRel.AddTarget(movingRBprim.GetPath())

        collectionAPI = Usd.CollectionAPI.Apply(deformableGroup.GetPrim(), "colliders")
        collectionAPI.CreateIncludesRel().AddTarget(beamPath)

        # Plane
        physicsUtils.add_ground_plane(stage, "/groundPlane", "Z", 750.0, Gf.Vec3f(0.0), Gf.Vec3f(0.5))

        # set cam:
        x_offset = 175
        customLayerData = {
            "cameraSettings": {
                "Perspective": {
                    "position": Gf.Vec3d(-50 + x_offset, -400, 290),
                    "radius": 600,
                    "target": Gf.Vec3d(x_offset, 0, self._bodyHeight),
                },
                "boundCamera": "/OmniverseKit_Persp",
            }
        }
        stage.GetRootLayer().customLayerData = customLayerData

    def on_simulation_event(self, e):
        if e.type == int(physx_settings_bindings.SimulationEvent.STOPPED):
            self._time = 0.0
            physicsUtils.set_or_add_translate_op(self._movingRBxformable, translate=self._movingRbInitPos)
