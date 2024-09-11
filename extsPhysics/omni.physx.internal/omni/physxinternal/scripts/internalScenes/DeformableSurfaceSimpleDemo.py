from omni.physx.scripts import physicsUtils, deformableUtils
import omni.physxdemos as demo
import omni.usd
from pxr import Usd, UsdGeom, UsdLux, Sdf, Gf, Tf, Vt, UsdPhysics, PhysxSchema


class DeformableSurfaceSimpleDemo(demo.Base):
    title = "Simple Deformable Surface Demo"
    category = demo.Categories.INTERNAL
    short_description = "Derformable surface falling on sphere"
    description = ("Simple demo of a cloth falling onto a sphere then onto the ground.")

    def create(self, stage):
        defaultPrimPath, scene = demo.setup_physics_scene(self, stage)
        room = demo.get_demo_room(self, stage, hasTable=False)

        # Create grid mesh
        clothMeshPath = omni.usd.get_stage_next_free_path(stage, "/cloth", True)
        clothMesh = UsdGeom.Mesh.Define(stage, clothMeshPath)
        tri_points, tri_indices = deformableUtils.create_triangle_mesh_square(128, 128)
        clothMesh.GetPointsAttr().Set(tri_points)
        clothMesh.GetFaceVertexIndicesAttr().Set(tri_indices)
        clothMesh.GetFaceVertexCountsAttr().Set([3] * (len(tri_indices) // 3))
        clothMesh.AddTranslateOp().Set(Gf.Vec3f(30, 0, 150))
        clothMesh.AddScaleOp().Set(Gf.Vec3f(150, 150, 1))

        deformableUtils.add_physx_deformable_surface(
            stage,
            prim_path=clothMeshPath,
            simulation_indices=tri_indices,
            bending_stiffness_scale=0.001,
            solver_position_iteration_count=20,
            vertex_velocity_damping=0.0,
            self_collision=True,
            self_collision_filter_distance=1
        )
        deformable_api = PhysxSchema.PhysxCollisionAPI.Get(stage, clothMeshPath)
        deformable_api.GetRestOffsetAttr().Set(0.5)
        deformable_api.GetContactOffsetAttr().Set(0.8)

        sphere_path = omni.usd.get_stage_next_free_path(stage, "/sphere", True)
        sphere = physicsUtils.add_sphere(stage, sphere_path, 20, Gf.Vec3f(0, 0, 100))
        UsdPhysics.CollisionAPI.Apply(sphere)
