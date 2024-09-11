from omni.physx.scripts import physicsUtils
from pxr import UsdLux, UsdGeom, Gf, UsdPhysics, Sdf, PhysxSchema
import omni.physxdemos as demo


class CustomCylinderDemo(demo.Base):
    title = "Custom Cylinder"
    category = demo.Categories.CONTACTS
    short_description = "Basic demo showing the difference between a ConvexMesh-based cylinder and a CustomGeometry-based cylinder"
    description = "Basic demo showing the difference between a mesh cylinder (blue), which uses a ConvexMesh collider, and a shape cylinder (magenta), which uses CustomGeometry by default. You can toggle whether to use ConvexMesh or CustomGeometry for cylinder shapes in the scene Physics Settings.\n\nPress play (space) to run the simulation. Shift left click allows physics objects dragging during simulation."

    def create(self, stage):
        defaultPrimPath, scene = demo.setup_physics_scene(self, stage)

        room = demo.get_demo_room(self, stage, zoom = 0.25, contactRestOffset = Gf.Vec2f(0.5, 0.25))

        scaleFactor = 0.125

        # Cylinder 0 (ConvexMesh)
        cylinder0ActorPath = defaultPrimPath + "/cylinder0Actor"

        radius = 50.0 * scaleFactor
        height = 100.0 * scaleFactor
        position = Gf.Vec3f(250.0, -250.0, 100.0) * scaleFactor
        orientation = Gf.Quatf(0.0, 0.38268, 0.0, 0.92388)
        linVelocity = Gf.Vec3f(0.0, 0.0, 0.0)
        angularVelocity = Gf.Vec3f(0.0, 0.0, 1000.0)

        cylinder0Geom = physicsUtils.create_mesh_cylinder(stage, cylinder0ActorPath, height, radius)
        cylinder0Prim = stage.GetPrimAtPath(cylinder0ActorPath)
        cylinder0Geom.AddTranslateOp().Set(position)
        cylinder0Geom.AddOrientOp().Set(orientation)
        cylinder0Geom.AddScaleOp().Set(Gf.Vec3f(1.0, 1.0, 1.0))
        cylinder0Geom.CreateDisplayColorAttr().Set([demo.get_primary_color(0)])

        UsdPhysics.CollisionAPI.Apply(cylinder0Prim)
        mesh_api = UsdPhysics.MeshCollisionAPI.Apply(cylinder0Prim)
        mesh_api.CreateApproximationAttr(UsdPhysics.Tokens.convexHull)

        physicsAPI = UsdPhysics.RigidBodyAPI.Apply(cylinder0Prim)
        physicsAPI.CreateVelocityAttr().Set(linVelocity)
        physicsAPI.CreateAngularVelocityAttr().Set(angularVelocity)
        UsdPhysics.MassAPI.Apply(cylinder0Prim)

        # Cylinder 1 (CustomGeometry)
        cylinder1ActorPath = defaultPrimPath + "/cylinder1Actor"

        radius = 50.0 * scaleFactor
        height = 100.0 * scaleFactor
        position = Gf.Vec3f(-250.0, 250.0, 100.0) * scaleFactor
        orientation = Gf.Quatf(0.0, 0.38268, 0.0, 0.92388)
        linVelocity = Gf.Vec3f(0.0, 0.0, 0.0)
        angularVelocity = Gf.Vec3f(0.0, 0.0, 1000.0)

        cylinder1Geom = UsdGeom.Cylinder.Define(stage, cylinder1ActorPath)
        cylinder1Prim = stage.GetPrimAtPath(cylinder1ActorPath)
        cylinder1Geom.CreateHeightAttr(height)
        cylinder1Geom.CreateRadiusAttr(radius)
        cylinder1Geom.CreateExtentAttr([(-radius, -radius, -height/2), (radius, radius, height/2)])
        cylinder1Geom.CreateAxisAttr(UsdGeom.GetStageUpAxis(stage))
        cylinder1Geom.AddTranslateOp().Set(position)
        cylinder1Geom.AddOrientOp().Set(orientation)
        cylinder1Geom.AddScaleOp().Set(Gf.Vec3f(1.0, 1.0, 1.0))
        cylinder1Geom.CreateDisplayColorAttr().Set([demo.get_primary_color(1)])
        # make the cylinder high-quality render
        cylinder1Prim.CreateAttribute("refinementEnableOverride", Sdf.ValueTypeNames.Bool, True).Set(True)
        cylinder1Prim.CreateAttribute("refinementLevel", Sdf.ValueTypeNames.Int, True).Set(2)

        UsdPhysics.CollisionAPI.Apply(cylinder1Prim)
        physicsAPI = UsdPhysics.RigidBodyAPI.Apply(cylinder1Prim)
        physicsAPI.CreateVelocityAttr().Set(linVelocity)
        physicsAPI.CreateAngularVelocityAttr().Set(angularVelocity)
        UsdPhysics.MassAPI.Apply(cylinder1Prim)
