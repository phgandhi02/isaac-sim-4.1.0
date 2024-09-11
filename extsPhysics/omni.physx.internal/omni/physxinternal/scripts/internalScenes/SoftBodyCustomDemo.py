import omni
from omni.physx.scripts import deformableUtils, physicsUtils, deformableMeshUtils
import omni.physxdemos as demo
import omni.usd

from pxr import UsdGeom, UsdLux, Sdf, Gf, UsdPhysics, PhysxSchema


class SoftBodyCustomDemo(demo.Base):
    title = "Soft body custom"
    category = demo.Categories.INTERNAL
    short_description = "FEM soft body custom scene setup"
    description = "This snippet sets up a custom FEM soft body scene"     

    params = {
        "shared_tet_mesh": demo.CheckboxParam(False),
        "skin_mesh": demo.CheckboxParam(True),
    }

    def create(self, stage, shared_tet_mesh, skin_mesh):
        self._stage = stage

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

        meshTranslation = Gf.Vec3f(0.0, 0.0, 600.0)
        meshScale = Gf.Vec3f(300.0, 300.0, 300.0)

        skinMeshPath = Sdf.Path()
        collMeshPath = Sdf.Path()
        simMeshPath = Sdf.Path()

        tet_points, tet_indices = deformableMeshUtils.createTetraVoxelSphere(3)

        if shared_tet_mesh:
            collMeshPath = simMeshPath = omni.usd.get_stage_next_free_path(self._stage, "/coll_sim_tetmesh", True)
            # Collision/Simulation Tetra Mesh
            tetMesh = PhysxSchema.TetrahedralMesh.Define(self._stage, collMeshPath)
            tetMesh.GetPointsAttr().Set(tet_points)
            tetMesh.GetIndicesAttr().Set(tet_indices)
            tetMesh.AddTranslateOp().Set(meshTranslation)
            tetMesh.AddScaleOp().Set(meshScale)
        else:
            collMeshPath = omni.usd.get_stage_next_free_path(self._stage, "/coll_tetmesh", True)
            simMeshPath = omni.usd.get_stage_next_free_path(self._stage, "/sim_tetmesh", True)
            # Collision Tetra Mesh
            tetMesh = PhysxSchema.TetrahedralMesh.Define(self._stage, collMeshPath)
            tetMesh.GetPointsAttr().Set(tet_points)
            tetMesh.GetIndicesAttr().Set(tet_indices)
            tetMesh.AddTranslateOp().Set(meshTranslation)
            tetMesh.AddScaleOp().Set(meshScale)
            # Simulation Tetra Mesh
            tetMesh = PhysxSchema.TetrahedralMesh.Define(self._stage, simMeshPath)
            tetMesh.GetPointsAttr().Set(tet_points)
            tetMesh.GetIndicesAttr().Set(tet_indices)
            tetMesh.AddTranslateOp().Set(meshTranslation)
            tetMesh.AddScaleOp().Set(meshScale)

        skinMeshPath = omni.usd.get_stage_next_free_path(self._stage, "/softbody", True)
        if skin_mesh:
            skinMesh = UsdGeom.Mesh.Define(self._stage, skinMeshPath)
            tri_points, tri_indices = deformableUtils.extractTriangleSurfaceFromTetra(tet_points, tet_indices)
            skinMesh.GetPointsAttr().Set(tri_points)
            skinMesh.GetFaceVertexIndicesAttr().Set(tri_indices)
            skinMesh.GetFaceVertexCountsAttr().Set([3] * (len(tri_indices) // 3))
            skinMesh.GetSubdivisionSchemeAttr().Set("none")
            skinMesh.AddTranslateOp().Set(meshTranslation)
            skinMesh.AddScaleOp().Set(meshScale)
        else:
            deformableUtils.create_skin_mesh_from_tetrahedral_mesh(skinMeshPath, self._stage, collMeshPath)

        omni.kit.commands.execute(
            "AddDeformableBodyComponent",
            skin_mesh_path=skinMeshPath,
            collision_mesh_path=collMeshPath,
            simulation_mesh_path=simMeshPath,
            solver_position_iteration_count=20
        )

        deformable_material_path = omni.usd.get_stage_next_free_path(stage, "/deformableBodyMaterial", True)
        deformableUtils.add_deformable_body_material(stage, deformable_material_path,
                                                  damping_scale=0.0,
                                                  poissons_ratio=0.45,
                                                  youngs_modulus=20000.0,
                                                  dynamic_friction=0.5)
        physicsUtils.add_physics_material_to_prim(stage, stage.GetPrimAtPath(skinMeshPath), deformable_material_path)

        stage.RemovePrim(collMeshPath)
        if not shared_tet_mesh:
            stage.RemovePrim(simMeshPath)

        # Plane
        physicsUtils.add_ground_plane(stage, "/groundPlane", "Z", 750.0, Gf.Vec3f(0.0), Gf.Vec3f(0.5))
