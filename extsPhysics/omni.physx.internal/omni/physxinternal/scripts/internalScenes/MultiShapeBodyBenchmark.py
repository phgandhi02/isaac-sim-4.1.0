import math
from omni.physx.scripts import physicsUtils
from pxr import Usd, UsdLux, UsdGeom, Sdf, Gf, Tf, UsdPhysics
import omni.physxdemos as demo


class MultiShapeBodyBenchmark(demo.Base):
    title = "Multishape body benchmark"
    category = demo.Categories.INTERNAL
    short_description = "Benchmark demo showing a lot of rigid bodies falling on a ground plane"
    description = "Benchmark demo showing a lot of rigid bodies falling on a ground plane. Press play (space) to run the simulation."

    def create(self, stage):
        # set up axis to z
        UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
        UsdGeom.SetStageMetersPerUnit(stage, 0.01)

        defaultPrimPath = str(stage.GetDefaultPrim().GetPath())

        # light
        sphereLight = UsdLux.SphereLight.Define(stage, defaultPrimPath + "/SphereLight")
        sphereLight.CreateRadiusAttr(150)
        sphereLight.CreateIntensityAttr(30000)
        sphereLight.AddTranslateOp().Set(Gf.Vec3f(650.0, 0.0, 1150.0))

        # Physics scene
        scene = UsdPhysics.Scene.Define(stage, defaultPrimPath + "/physicsScene")
        scene.CreateGravityDirectionAttr().Set(Gf.Vec3f(0.0, 0.0, -1.0))
        scene.CreateGravityMagnitudeAttr().Set(981.0)


        numX = 30
        numY = 30
        for x in range(numX):
            for y in range(numY):
                self.addCompoundShape(
                    stage, defaultPrimPath + "/compoundRigid" + str(x) + "_" + str(y), Gf.Vec3f(100.0 * x, 0.0, 500 + 100.0 * y)
                )

        # Plane
        physicsUtils.add_ground_plane(stage, "/groundPlane", "Z", 750.0, Gf.Vec3f(0.0), Gf.Vec3f(0.5))

    def addCompoundShape(self, stage, rigidCompoundPath, rigidCompoundPos):
        size = 25.0

        # Top level actor, contains rigid body and its shapes
        rigidXform = UsdGeom.Xform.Define(stage, rigidCompoundPath)
        rigidPrim = stage.GetPrimAtPath(rigidCompoundPath)

        # Rigid body transform
        rigidXform.AddTranslateOp().Set(rigidCompoundPos)
        rigidXform.AddOrientOp().Set(Gf.Quatf(1.0, 0.0, 0.0, 0.0))

        physicsAPI = UsdPhysics.RigidBodyAPI.Apply(rigidPrim)
        (rigidPrim)
        UsdPhysics.MassAPI.Apply(rigidPrim)

        # Collision shapes
        collisionShapePath0 = rigidCompoundPath + "/physicsBoxShape0"

        shapePos = Gf.Vec3f(0.0, 0.0, -25.0)
        shapeQuat = Gf.Quatf(1.0, 0.0, 0.0, 0.0)
        shapeColor = Gf.Vec3f(71.0 / 255.0, 165.0 / 255.0, 1.0)

        cubeGeom = UsdGeom.Cube.Define(stage, collisionShapePath0)
        cubePrim = stage.GetPrimAtPath(collisionShapePath0)
        cubeGeom.CreateSizeAttr(size)
        half_extent = size / 2
        cubeGeom.CreateExtentAttr([(-half_extent, -half_extent, -half_extent), (half_extent, half_extent, half_extent)])
        cubeGeom.AddTranslateOp().Set(shapePos)
        cubeGeom.AddOrientOp().Set(shapeQuat)
        cubeGeom.CreateDisplayColorAttr().Set([shapeColor])

        UsdPhysics.CollisionAPI.Apply(cubePrim)

        collisionShapePath1 = rigidCompoundPath + "/physicsBoxShape1"

        shapePos = Gf.Vec3f(0.0, 50.0, -25.0)
        shapeQuat = Gf.Quatf(1.0, 0.0, 0.0, 0.0)
        shapeColor = Gf.Vec3f(71.0 / 255.0, 25.0 / 255.0, 1.0)

        cubeGeom = UsdGeom.Cube.Define(stage, collisionShapePath1)
        cubePrim = stage.GetPrimAtPath(collisionShapePath1)
        cubeGeom.CreateSizeAttr(size)
        cubeGeom.CreateExtentAttr([(-half_extent, -half_extent, -half_extent), (half_extent, half_extent, half_extent)])
        cubeGeom.AddTranslateOp().Set(shapePos)
        cubeGeom.AddOrientOp().Set(shapeQuat)
        cubeGeom.CreateDisplayColorAttr().Set([shapeColor])

        UsdPhysics.CollisionAPI.Apply(cubePrim)
