import math
import os
import random

from pxr import Gf, Sdf, Usd
from pxr import UsdGeom, UsdUtils, UsdPhysics, UsdLux
from pxr import PhysxSchema, PhysicsSchemaTools, ForceFieldSchema

import omni.physx.scripts.physicsUtils as physicsUtils
import omni.kit

import omni.physxdemos as demo


class VortexForceFieldDemo(demo.Base):
    title = "Vortex"
    category = demo.Categories.FORCE_FIELDS
    short_description = "Tornado demo using force fields"
    description = "How to combine linear, spin and drag force fields to get objects spinning around like a tornado."

    def create(self, stage):
        numberOfBoxes = 200
        boxSpacing = 2
        boxPathName = "/box"
        groundPathName = "/ground"
        scenePathName = "/scene"
        vortexPathName = "/vortex"

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
        sphereLight.CreateIntensityAttr().Set(200000000)
        sphereLight.AddTranslateOp(precision=UsdGeom.XformOp.PrecisionFloat).Set(Gf.Vec3f(500.0, 750.0, 1000.0))

        # Plane
        physicsUtils.add_ground_plane(stage, groundPathName, "Y", 750.0, Gf.Vec3f(0.0), Gf.Vec3f(0.5))

        # Create the ForceFields using prims instead of directly communicating with the extension.
        vortexPrim = stage.DefinePrim(vortexPathName)

        vortexPrim.CreateAttribute("xformOp:translate", Sdf.ValueTypeNames.Float3, False).Set(Gf.Vec3f(0.0, 0.0, 0.0))
        vortexPrim.CreateAttribute("xformOp:orient", Sdf.ValueTypeNames.Quatf, False).Set(Gf.Quatf(1.0, 0.0, 0.0, 0.0))
        vortexPrim.CreateAttribute("xformOp:scale", Sdf.ValueTypeNames.Float3, False).Set(Gf.Vec3f(1.0, 1.0, 1.0))
        #vortexPrim.CreateAttribute("xformOpOrder", Sdf.ValueTypeNames.String, False).Set(["xformOp:translate", "xformOp:orient", "xformOp:scale"])

        dragPrimApi = ForceFieldSchema.PhysxForceFieldDragAPI.Apply(vortexPrim, "Drag")
        dragPrimApi.CreateMinimumSpeedAttr(10.0)
        dragPrimApi.CreateLinearAttr(0.0)
        dragPrimApi.CreateSquareAttr(0.02)

        dragBaseApi = ForceFieldSchema.PhysxForceFieldAPI(vortexPrim, "Drag")
        dragBaseApi.CreateEnabledAttr(True)
        dragBaseApi.CreatePositionAttr(Gf.Vec3f(0.0, 0.0, 0.0))
        dragBaseApi.CreateRangeAttr(Gf.Vec2f(-1.0, -1.0))

        linearPrimApi = ForceFieldSchema.PhysxForceFieldLinearAPI.Apply(vortexPrim, "Linear")
        linearPrimApi.CreateDirectionAttr(Gf.Vec3f(0.0, 1.0, 0.0))
        linearPrimApi.CreateConstantAttr(0.0)
        linearPrimApi.CreateLinearAttr(-5.0e4)
        linearPrimApi.CreateInverseSquareAttr(0.0)

        linearBaseApi = ForceFieldSchema.PhysxForceFieldAPI(vortexPrim, "Linear")
        linearBaseApi.CreateEnabledAttr(True)
        linearBaseApi.CreatePositionAttr(Gf.Vec3f(0.0, 0.0, 0.0))
        linearBaseApi.CreateRangeAttr(Gf.Vec2f(-1.0, -1.0))

        spinPrimApi = ForceFieldSchema.PhysxForceFieldSpinAPI.Apply(vortexPrim, "Spin")
        spinPrimApi.CreateConstantAttr(6e5)
        spinPrimApi.CreateLinearAttr(0.0)
        spinPrimApi.CreateInverseSquareAttr(0.0)
        spinPrimApi.CreateSpinAxisAttr(Gf.Vec3f(0.0, 1.0, 0.0))

        spinBaseApi = ForceFieldSchema.PhysxForceFieldAPI(vortexPrim, "Spin")
        spinBaseApi.CreateEnabledAttr(True)
        spinBaseApi.CreatePositionAttr(Gf.Vec3f(0.0, 0.0, 0.0))
        spinBaseApi.CreateRangeAttr(Gf.Vec2f(-1.0, -1.0))

        # Add the collection 
        collectionAPI = Usd.CollectionAPI.Apply(vortexPrim, ForceFieldSchema.Tokens.forceFieldBodies)
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
