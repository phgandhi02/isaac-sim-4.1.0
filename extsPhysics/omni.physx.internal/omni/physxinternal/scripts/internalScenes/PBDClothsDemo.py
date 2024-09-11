import omni
from omni.physx.scripts import physicsUtils, particleUtils, deformableUtils
import omni.physxdemos as demo
import omni.usd

from pxr import UsdGeom, UsdLux, Sdf, Gf, Tf, Vt, UsdPhysics


class ClothsDemo(demo.Base):
    title = "PBD Cloths"
    category = demo.Categories.INTERNAL
    short_description = "PBD Cloths scene setup"
    description = "This snippet sets up a particle system based cloth scene with self collision, where each cloth can interact with the others."

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

        radius = 10

        # Particle System
        particleSystemPath = Sdf.Path("/particleSystem0")

        restOffset = radius
        contactOffset = restOffset + 0.1

        particleUtils.add_physx_particle_system(
            stage=stage,
            particle_system_path=particleSystemPath,
            contact_offset=contactOffset,
            rest_offset=restOffset,
            particle_contact_offset=contactOffset,
            solid_rest_offset=restOffset,
            fluid_rest_offset=0.0,
            solver_position_iterations=16,
            simulation_owner=scene.GetPath(),
        )

        pbd_particle_material_path = omni.usd.get_stage_next_free_path(stage, "/pbdParticleMaterial", True)
        particleUtils.add_pbd_particle_material(stage, pbd_particle_material_path, friction=0.6, drag=0.1, lift=0.3)
        physicsUtils.add_physics_material_to_prim(
            stage, stage.GetPrimAtPath(particleSystemPath), pbd_particle_material_path
        )

        numCloths = 3
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
            clothMesh.AddTranslateOp().Set(Gf.Vec3f(0.0, 300.0 * (i - 1) + 500.0, 1000.0))
            clothMesh.AddRotateXOp().Set(30.0 * i)
            clothMesh.AddScaleOp().Set(Gf.Vec3f(400.0, 400.0, 400.0))

            particleUtils.add_physx_particle_cloth(
                stage=stage,
                path=clothMeshPath,
                dynamic_mesh_path=None,
                particle_system_path=particleSystemPath,
                spring_stretch_stiffness=10000.0,
                spring_bend_stiffness=200.0,
                spring_shear_stiffness=100.0,
                spring_damping=0.2,
                self_collision=True,
                self_collision_filter=True,
                particle_group=0,
            )

            clothPrim = stage.GetPrimAtPath(clothMeshPath)
            particleMass = 0.02
            massApi = UsdPhysics.MassAPI.Apply(clothPrim)
            massApi.GetMassAttr().Set(particleMass * len(tri_points))

        # Plane
        physicsUtils.add_ground_plane(stage, "/groundPlane", "Z", 750.0, Gf.Vec3f(0.0), Gf.Vec3f(0.5))
