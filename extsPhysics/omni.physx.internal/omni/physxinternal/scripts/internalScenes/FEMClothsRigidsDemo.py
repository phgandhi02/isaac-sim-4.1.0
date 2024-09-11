import math
import omni
from omni.physx.scripts import physicsUtils, deformableUtils
import omni.physxdemos as demo
import omni.usd

from pxr import Usd, UsdGeom, UsdLux, Sdf, Gf, Tf, Vt, UsdPhysics, PhysxSchema


class FEMClothsRigidsDemo(demo.Base):
    title = "FEM Cloths and Rigids"
    category = demo.Categories.INTERNAL
    short_description = "FEM Cloths-rigids interaction"
    description = "This snippet shows a FEM cloth interacting with rigid bodies."

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

        numCloths = 2
        for i in range(numCloths):

            # Create grid mesh
            clothMeshPath = omni.usd.get_stage_next_free_path(stage, "/cloth" + str(i), True)
            clothMesh = UsdGeom.Mesh.Define(stage, clothMeshPath)
            tri_points, tri_indices = deformableUtils.create_triangle_mesh_square(32, 32)
            clothMesh.GetPointsAttr().Set(tri_points)
            clothMesh.GetFaceVertexIndicesAttr().Set(tri_indices)
            clothMesh.GetFaceVertexCountsAttr().Set([3] * (len(tri_indices) // 3))
            color = Vt.Vec3fArray([Gf.Vec3f(71.0 / 255.0, 165.0 / 255.0, 0.3 * (i + 1))])
            clothMesh.CreateDisplayColorAttr(color)
            clothMesh.AddTranslateOp().Set(Gf.Vec3f(0.0, 450.0 * i, 800.0 * (i + 1)))
            clothMesh.AddRotateXOp().Set(30.0 * i)
            clothMesh.AddScaleOp().Set(Gf.Vec3f(400.0, 400.0, 400.0) * (i + 1))

            deformableUtils.add_physx_deformable_surface(
                stage,
                prim_path=clothMeshPath,
                simulation_indices=tri_indices,
                bending_stiffness_scale=10,
                solver_position_iteration_count=20,
                vertex_velocity_damping=0.0,
                self_collision_filter_distance=0.05
            )

        height = 50.0
        numBoxes = 2
        for i in range(numBoxes):
            # Box
            boxActorPath = "/boxActor" + str(i)

            size = Gf.Vec3f(200.0)
            position = Gf.Vec3f(0.0, 0.0, height + (i + 1) * 250)
            orientation = Gf.Quatf(1.0, 0.0, 0.0, 0.0)
            color = Gf.Vec3f(200.0 / 255.0, 165.0 / 255.0, 1.0)
            linVelocity = Gf.Vec3f(20.0, 10.0, 20.0)
            angularVelocity = Gf.Vec3f(1.0, 0.0, 0.0)

            physicsUtils.add_rigid_box(
                stage, boxActorPath, size, position, orientation, color, 10.0, linVelocity, angularVelocity
            )

        # Plane
        physicsUtils.add_ground_plane(stage, "/groundPlane", "Z", 750.0, Gf.Vec3f(0.0), Gf.Vec3f(0.5))
