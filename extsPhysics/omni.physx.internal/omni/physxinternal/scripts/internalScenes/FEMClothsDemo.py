import math
import omni
from omni.physx.scripts import physicsUtils
import omni.physxdemos as demo
import omni.usd

from omni.physx.scripts import deformableUtils

from pxr import Usd, UsdGeom, UsdLux, Sdf, Gf, Tf, Vt, UsdPhysics


class FEMClothsDemo(demo.Base):
    title = "FEM Cloths"
    category = demo.Categories.INTERNAL
    short_description = "FEM Cloths scene setup"
    description = "This snippet sets up a FEM cloths scene with self collision, where each cloth can interact with the others."

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

        numCloths = 3
        for i in range(numCloths):

            # Create grid mesh
            clothMeshPath = omni.usd.get_stage_next_free_path(stage, "/cloth" + str(i), True)
            clothMesh = UsdGeom.Mesh.Define(stage, clothMeshPath)
            tri_points, tri_indices = deformableUtils.create_triangle_mesh_square(32, 32)
            clothMesh.GetPointsAttr().Set(tri_points)
            clothMesh.GetFaceVertexIndicesAttr().Set(tri_indices)
            clothMesh.GetFaceVertexCountsAttr().Set([3]*(len(tri_indices)//3))
            color = Vt.Vec3fArray([Gf.Vec3f(71.0 / 255.0, 165.0 / 255.0, 0.3 * (i + 1))])
            clothMesh.CreateDisplayColorAttr(color)
            clothMesh.AddTranslateOp().Set(Gf.Vec3f(0.0, 300.0 * (i-1) + 500.0, 1000.0))
            clothMesh.AddRotateXOp().Set(30.0 * i)
            clothMesh.AddScaleOp().Set(Gf.Vec3f(400.0, 400.0, 400.0))

            deformableUtils.add_physx_deformable_surface(
                stage,
                prim_path=clothMeshPath,
                simulation_indices=tri_indices,
                bending_stiffness_scale=10.0,
                solver_position_iteration_count=20,
                vertex_velocity_damping=0.0
            )
   
        # Plane
        physicsUtils.add_ground_plane(stage, "/groundPlane", "Z", 750.0, Gf.Vec3f(0.0), Gf.Vec3f(0.5))
