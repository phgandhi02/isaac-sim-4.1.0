import math
import os
import random

from pxr import Gf, Sdf, Usd
from pxr import UsdGeom, UsdUtils, UsdPhysics, UsdLux
from pxr import PhysxSchema, PhysicsSchemaTools, ForceFieldSchema

import omni.physx.scripts.physicsUtils as physicsUtils
import omni.kit

import omni.physxdemos as demo


class LevitateForceFieldDemo(demo.Base):
    title = "Levitate"
    category = demo.Categories.FORCE_FIELDS
    short_description = "Levitate objects off the ground"
    description = "Shake objects on the ground as if they are struggling to levitate. Then begin the levitation."

    def create(self, stage):
        numberOfBoxes = 20
        boxSpacing = 2
        boxPathName = "/box"
        groundPathName = "/ground"
        scenePathName = "/scene"
        levitatePathName = "/levitate"

        # Physics scene
        up = Gf.Vec3f(0.0)
        up[1] = 1.0
        gravityDirection = -up
        gravityMagnitude = 1000.0
        forceRange = Gf.Vec2f(-1.0)
        center = Gf.Vec3f(0.0)
        center[1] = 400.0

        scene = UsdPhysics.Scene.Define(stage, scenePathName)
        scene.CreateGravityDirectionAttr(gravityDirection)
        scene.CreateGravityMagnitudeAttr(gravityMagnitude)

        # light
        sphereLight = UsdLux.SphereLight.Define(stage, "/World/SphereLight")
        sphereLight.CreateRadiusAttr().Set(1.5)
        sphereLight.CreateIntensityAttr().Set(60000000)
        sphereLight.AddTranslateOp(precision=UsdGeom.XformOp.PrecisionFloat).Set(Gf.Vec3f(6.5, 750.0, 0))

        # Plane
        physicsUtils.add_ground_plane(stage, groundPathName, "Y", 750.0, Gf.Vec3f(0.0), Gf.Vec3f(0.5))

        # Create the wind force field
        levitatePrim = stage.DefinePrim(levitatePathName)

        levitatePrim.CreateAttribute("xformOp:translate", Sdf.ValueTypeNames.Float3, False).Set(Gf.Vec3f(0.0, 0.0, 0.0))
        levitatePrim.CreateAttribute("xformOp:orient", Sdf.ValueTypeNames.Quatf, False).Set(Gf.Quatf(0.707, 0.0, 0.707, 0.0))
        levitatePrim.CreateAttribute("xformOp:scale", Sdf.ValueTypeNames.Float3, False).Set(Gf.Vec3f(1.0, 1.0, 1.0))
        #levitatePrim.CreateAttribute("xformOpOrder", Sdf.ValueTypeNames.String, False).Set(["xformOp:translate", "xformOp:orient", "xformOp:scale"])

        dragPrimApi = ForceFieldSchema.PhysxForceFieldDragAPI.Apply(levitatePrim, "Drag")
        dragPrimApi.CreateMinimumSpeedAttr(10.0)
        dragPrimApi.CreateLinearAttr(0.0)
        dragPrimApi.CreateSquareAttr(2.0)

        dragBaseApi = ForceFieldSchema.PhysxForceFieldAPI(levitatePrim, "Drag")
        dragBaseApi.CreateEnabledAttr(False)
        dragBaseApi.CreatePositionAttr(Gf.Vec3f(0.0, 0.0, 0.0))
        dragBaseApi.CreateRangeAttr(Gf.Vec2f(-1.0, -1.0))

        noisePrimApi = ForceFieldSchema.PhysxForceFieldNoiseAPI.Apply(levitatePrim, "Shake")
        noisePrimApi.CreateDragAttr(3000.0)
        noisePrimApi.CreateAmplitudeAttr(Gf.Vec3f(300.0, 0.0, 300.0))
        noisePrimApi.CreateFrequencyAttr(Gf.Vec3f(5.0))

        noiseBaseApi = ForceFieldSchema.PhysxForceFieldAPI(levitatePrim, "Shake")
        noiseBaseApi.CreateEnabledAttr(False)
        noiseBaseApi.CreatePositionAttr(Gf.Vec3f(0.0, 0.0, 0.0))
        noiseBaseApi.CreateRangeAttr(Gf.Vec2f(-1.0, -1.0))

        levitatePrimApi = ForceFieldSchema.PhysxForceFieldPlanarAPI.Apply(levitatePrim, "Levitate")
        levitatePrimApi.CreateNormalAttr(Gf.Vec3f(0.0, 1.0, 0.0))
        levitatePrimApi.CreateConstantAttr(0.0)
        levitatePrimApi.CreateLinearAttr(0.0)
        levitatePrimApi.CreateInverseSquareAttr(2.0e9)

        levitateBaseApi = ForceFieldSchema.PhysxForceFieldAPI(levitatePrim, "Levitate")
        levitateBaseApi.CreateEnabledAttr(False)
        levitateBaseApi.CreatePositionAttr(Gf.Vec3f(0.0, 0.0, 0.0))
        levitateBaseApi.CreateRangeAttr(Gf.Vec2f(-1.0, -1.0))

        # Add the collection 
        collectionAPI = Usd.CollectionAPI.Apply(levitatePrim, ForceFieldSchema.Tokens.forceFieldBodies)
        collectionAPI.CreateIncludesRel().AddTarget(stage.GetDefaultPrim().GetPath())

        # Boxes
        boxSize = Gf.Vec3f(100.0)
        boxPosition = Gf.Vec3f(0.0)
        m = (int)(math.sqrt(numberOfBoxes))

        for i in range(m):
            for j in range(m):
                boxPath = boxPathName + str(i) + str(j)
                boxPosition[0] = (i + 0.5 - (0.5 * m)) * boxSpacing * boxSize[0]
                boxPosition[1] = 0.5 * boxSize[1]
                boxPosition[2] = (j + 0.5 - (0.5 * m)) * boxSpacing * boxSize[2]
                boxPrim = physicsUtils.add_rigid_box(stage, boxPath, position=boxPosition, size=boxSize)

        # Animate the force fields on and off
        global time
        time = 0.0

        def force_fields_step(deltaTime):
            global time
            time = time + deltaTime 

            if time > 8.0:
                noiseBaseApi.GetEnabledAttr().Set(False)
                levitateBaseApi.GetEnabledAttr().Set(False)
                dragBaseApi.GetEnabledAttr().Set(False) 
            elif time > 4.0:
                noiseBaseApi.GetEnabledAttr().Set(False)
                levitateBaseApi.GetEnabledAttr().Set(True)
                dragBaseApi.GetEnabledAttr().Set(True) 
            else:
                noiseBaseApi.GetEnabledAttr().Set(True)
                levitateBaseApi.GetEnabledAttr().Set(False)
                dragBaseApi.GetEnabledAttr().Set(False) 

        def timeline_event(event):
            # on play press
            if event.type == int(omni.timeline.TimelineEventType.PLAY):
                global time
                time = 0.0

            # on stop press
            if event.type == int(omni.timeline.TimelineEventType.STOP):
                pass
            
        physxInterface = omni.physx.get_physx_interface()
        self._subscriptionId = physxInterface.subscribe_physics_step_events(force_fields_step)
 
        timelineInterface = omni.timeline.get_timeline_interface()
        stream = timelineInterface.get_timeline_event_stream()
        self._timeline_subscription = stream.create_subscription_to_pop(timeline_event)
