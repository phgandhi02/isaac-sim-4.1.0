import math
import os
import random

from pxr import Gf, Sdf, Usd
from pxr import UsdGeom, UsdUtils, UsdPhysics, UsdLux
from pxr import PhysxSchema, PhysicsSchemaTools, ForceFieldSchema

import omni.physx.scripts.physicsUtils as physicsUtils
import omni.kit

import omni.physxdemos as demo


class ExplodeForceFieldDemo(demo.Base):
    title = "Explode"
    category = demo.Categories.FORCE_FIELDS
    short_description = "Suck objects together before they are blown apart."
    description = "Use two spherical force fields to first attract objects to a central point and then explode them apart."

    def create(self, stage):
        numberOfBoxes = 20
        boxSpacing = 2
        boxPathName = "/box"
        groundPathName = "/ground"
        scenePathName = "/World/scene"
        explodePathName = "/World/explode"

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

        # Plane
        physicsUtils.add_ground_plane(stage, groundPathName, "Y", 750.0, Gf.Vec3f(0.0), Gf.Vec3f(0.5))

        # light
        sphereLight = UsdLux.SphereLight.Define(stage, "/World/SphereLight")
        sphereLight.CreateRadiusAttr().Set(1.5)
        sphereLight.CreateIntensityAttr().Set(60000000)
        sphereLight.AddTranslateOp(precision=UsdGeom.XformOp.PrecisionFloat).Set(Gf.Vec3f(6.5, 750.0, 0))

        # Create the force field prim
        xformPrim = UsdGeom.Xform.Define(stage, explodePathName)
        xformPrim.AddTranslateOp().Set(Gf.Vec3f(0.0, 300.0, 0.0))
        explodePrim = xformPrim.GetPrim()

        suckPrimApi = ForceFieldSchema.PhysxForceFieldSphericalAPI.Apply(explodePrim, "Suck")
        suckPrimApi.CreateConstantAttr(-1e6)
        suckPrimApi.CreateLinearAttr(0.0)
        suckPrimApi.CreateInverseSquareAttr(0.0)

        suckBaseApi = ForceFieldSchema.PhysxForceFieldAPI(explodePrim, "Suck")
        suckBaseApi.CreateEnabledAttr(False)
        suckBaseApi.CreatePositionAttr(Gf.Vec3f(0.0, 0.0, 0.0))
        suckBaseApi.CreateRangeAttr(Gf.Vec2f(-1.0, -1.0))

        explodePrimApi = ForceFieldSchema.PhysxForceFieldSphericalAPI.Apply(explodePrim, "Explode")
        explodePrimApi.CreateConstantAttr(8e6)
        explodePrimApi.CreateLinearAttr(0.0)
        explodePrimApi.CreateInverseSquareAttr(0.0)

        explodeBaseApi = ForceFieldSchema.PhysxForceFieldAPI(explodePrim, "Explode")
        explodeBaseApi.CreateEnabledAttr(False)
        explodeBaseApi.CreatePositionAttr(Gf.Vec3f(0.0, 0.0, 0.0))
        explodeBaseApi.CreateRangeAttr(Gf.Vec2f(-1.0, -1.0))

        dragPrimApi = ForceFieldSchema.PhysxForceFieldDragAPI.Apply(explodePrim, "Drag")
        dragPrimApi.CreateMinimumSpeedAttr(10.0)
        dragPrimApi.CreateLinearAttr(100.0)
        dragPrimApi.CreateSquareAttr(0.0)

        dragBaseApi = ForceFieldSchema.PhysxForceFieldAPI(explodePrim, "Drag")
        dragBaseApi.CreateEnabledAttr(False)
        dragBaseApi.CreatePositionAttr(Gf.Vec3f(0.0, 0.0, 0.0))
        dragBaseApi.CreateRangeAttr(Gf.Vec2f(-1.0, -1.0))

        # Add the collection 
        collectionAPI = Usd.CollectionAPI.Apply(explodePrim, ForceFieldSchema.Tokens.forceFieldBodies)
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

            if time > 4.1:
                suckBaseApi.GetEnabledAttr().Set(False)
                explodeBaseApi.GetEnabledAttr().Set(False)
                dragBaseApi.GetEnabledAttr().Set(False) 
            elif time > 4.0:
                suckBaseApi.GetEnabledAttr().Set(False)
                explodeBaseApi.GetEnabledAttr().Set(True)
                dragBaseApi.GetEnabledAttr().Set(False) 
            elif time > 2.0:
                suckBaseApi.GetEnabledAttr().Set(True)
                explodeBaseApi.GetEnabledAttr().Set(False)
                dragBaseApi.GetEnabledAttr().Set(True) 
            else:
                suckBaseApi.GetEnabledAttr().Set(True)
                explodeBaseApi.GetEnabledAttr().Set(False)
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
