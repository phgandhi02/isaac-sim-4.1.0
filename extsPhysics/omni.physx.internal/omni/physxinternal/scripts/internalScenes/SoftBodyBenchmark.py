import omni

from omni.physx.scripts import deformableUtils, physicsUtils, deformableMeshUtils
import omni.physxdemos as demo
import omni.usd

from .. import commands as physxInternalCommands

from omni.physx import get_physx_interface, get_physx_cooking_interface
from omni.physxinternal.bindings import _physxInternal


from pxr import UsdGeom, UsdLux, Gf, Vt, UsdUtils, UsdPhysics


class SoftBodyDemo(demo.Base):
    title = "Soft body Benchmark"
    category = demo.Categories.INTERNAL_SAMPLES
    short_description = "FEM soft body benchmark scene"
    description = "This sample sets up a FEM soft body benchmark scene"      

    def create(self, stage):
        self._stage = stage
        timeline = omni.timeline.get_timeline_interface()
        stream = timeline.get_timeline_event_stream()

        physx_internal_iface = _physxInternal.acquire_physx_internal_interface()
        physx_cooking_iface = get_physx_cooking_interface()
        stage_id = UsdUtils.StageCache.Get().GetId(stage).ToLongInt()

        # set up axis to z
        UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
        UsdGeom.SetStageMetersPerUnit(stage, 0.01)

        defaultPrimPath = str(stage.GetDefaultPrim().GetPath())

        # light
        sphereLight = UsdLux.SphereLight.Define(stage, defaultPrimPath + "/SphereLight")
        sphereLight.CreateRadiusAttr(150)
        sphereLight.CreateIntensityAttr(30000)
        sphereLight.AddTranslateOp().Set(Gf.Vec3f(650.0, 0.0, 3150.0))

        # Physics scene
        scene = UsdPhysics.Scene.Define(stage, defaultPrimPath + "/physicsScene")
        scene.CreateGravityDirectionAttr().Set(Gf.Vec3f(0.0, 0.0, -1.0))
        scene.CreateGravityMagnitudeAttr().Set(981.0)

        tri_points, tri_indices = deformableMeshUtils.createTriangleMeshCube(8)

        deformable_material_path = omni.usd.get_stage_next_free_path(stage, "/deformableBodyMaterial", True)
        deformableUtils.add_deformable_body_material(stage, deformable_material_path,
                                                  youngs_modulus=20000.0,
                                                  poissons_ratio=0.45,
                                                  damping_scale=0.0,
                                                  dynamic_friction=0.5)

        nb_x = 16
        nb_y = 16
        nb_z = 1
        meshScale = Gf.Vec3f(400.0, 400.0, 400.0)
        index = 0
        for i in range(nb_x):
            for j in range(nb_y):
                for k in range(nb_z):
                    skinMeshPath = omni.usd.get_stage_next_free_path(self._stage, "/cube_skin_mesh"+str(index), True)
                    skinMesh = UsdGeom.Mesh.Define(stage, skinMeshPath)
                    skinMesh.GetPointsAttr().Set(tri_points)
                    skinMesh.GetFaceVertexIndicesAttr().Set(tri_indices)
                    skinMesh.GetFaceVertexCountsAttr().Set([3] * (len(tri_indices) // 3))
                    skinMesh.GetSubdivisionSchemeAttr().Set("none")
                    skinMesh.AddTranslateOp().Set(Gf.Vec3f(i*500, j*500, k*500+500.0))
                    skinMesh.AddScaleOp().Set(meshScale)
                    skinMeshColor = Vt.Vec3fArray([Gf.Vec3f(1.0, 165.0 / 255.0, 71.0 / 255.0)])
                    skinMesh.CreateDisplayColorAttr(skinMeshColor)

                    # Simple Softbody
                    omni.kit.commands.execute(
                        "AddDeformableBodyComponent",
                        skin_mesh_path=skinMeshPath,
                        solver_position_iteration_count=20
                    )

                    physicsUtils.add_physics_material_to_prim(stage, skinMesh.GetPrim(), deformable_material_path)

        # Plane
        physicsUtils.add_ground_plane(stage, "/groundPlane", "Z", 750.0, Gf.Vec3f(0.0), Gf.Vec3f(0.5))
