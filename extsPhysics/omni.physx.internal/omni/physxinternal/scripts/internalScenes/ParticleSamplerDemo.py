from omni.physx.scripts import physicsUtils, particleUtils
from pxr import UsdLux, UsdGeom, Sdf, Gf, Vt, UsdPhysics, PhysxSchema
import omni.physxdemos as demo
import omni.usd
import os

class ParticleSamplerDemo(demo.Base):
    title = "Particle Sampler"
    category = demo.Categories.INTERNAL
    short_description = "Particle position sampling from a mesh geometry."
    description = "This snippet samples initial particles positions from a mesh geometry."

    def create(self, stage):
        # set up axis to z
        UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
        UsdGeom.SetStageMetersPerUnit(stage, 1)

        # light
        sphereLight = UsdLux.SphereLight.Define(stage, Sdf.Path("/SphereLight"))
        sphereLight.CreateRadiusAttr(150)
        sphereLight.CreateIntensityAttr(30000)
        sphereLight.AddTranslateOp().Set(Gf.Vec3f(650.0, 0.0, 1150.0))

        # Physics scene
        scenePath = Sdf.Path("/physicsScene")
        scene = UsdPhysics.Scene.Define(stage, scenePath)
        scene.CreateGravityDirectionAttr().Set(Gf.Vec3f(0.0, 0.0, -1.0))
        scene.CreateGravityMagnitudeAttr().Set(9.81)

        # Ground Plane
        physicsUtils.add_ground_plane(stage, "/groundPlane", "Z", 25.0, Gf.Vec3f(0.0), Gf.Vec3f(0.5))

        # Particle System
        particleSystemPath = Sdf.Path("/particleSystem0")

        particleSpacing = 0.2
        restOffset = particleSpacing * 0.9
        solidRestOffset = restOffset
        fluidRestOffset = restOffset * 0.6
        particleContactOffset = max(solidRestOffset + 0.001, fluidRestOffset / 0.6)
        contactOffset = restOffset + 0.001
        particleUtils.add_physx_particle_system(
            stage=stage,
            particle_system_path=particleSystemPath,
            contact_offset=contactOffset,
            rest_offset=restOffset,
            particle_contact_offset=particleContactOffset,
            solid_rest_offset=solidRestOffset,
            fluid_rest_offset=fluidRestOffset,
            solver_position_iterations=4,
            simulation_owner=scenePath,
        )

        particleUtils.add_physx_particle_smoothing(
            stage=stage,
            path=particleSystemPath,
            strength=0.8,
        )

        # Create a pbd particle material and set it on the particle system
        pbd_particle_material_path = omni.usd.get_stage_next_free_path(stage, "/pbdParticleMaterial", True)
        particleUtils.add_pbd_particle_material(stage, pbd_particle_material_path, cohesion=0.5, friction=0.5)
        physicsUtils.add_physics_material_to_prim(stage, stage.GetPrimAtPath(particleSystemPath), pbd_particle_material_path)

        # prepare the source mesh
        mesh_path = omni.usd.get_stage_next_free_path(stage, "/mesh", True)
        data_folder = os.path.normpath(os.path.join(os.path.dirname(os.path.abspath(__file__)), "../../../.."))
        chibi_path = os.path.normpath(
            data_folder
            + "/data/usd/assets/bunny.obj.usda"
        )

        mesh = UsdGeom.Mesh.Define(stage, mesh_path)
        mesh_prim = mesh.GetPrim()
        mesh_prim.GetReferences().AddReference(chibi_path)

        mesh.AddTranslateOp().Set(Gf.Vec3f(0.0, 0.0, 10.0))
        mesh.AddScaleOp().Set(Gf.Vec3f(1.5, 1.5, 1.5))

        # apply the poisson sampling api
        samplingApi = PhysxSchema.PhysxParticleSamplingAPI.Apply(mesh_prim)

        # use the particleContactOffset for the sampling distance - results in ~25000 particles for this tutorial
        samplingApi.CreateSamplingDistanceAttr().Set(particleContactOffset)
        samplingApi.CreateVolumeAttr().Set(True)
        samplingApi.CreateMaxSamplesAttr().Set(50000)

        mesh.CreateVisibilityAttr("invisible")

        # create particle set
        particlePath = omni.usd.get_stage_next_free_path(stage, str(stage.GetDefaultPrim().GetPath().AppendElementString("particles")), False)
        points = UsdGeom.Points.Define(stage, particlePath)
        points.CreateDisplayColorAttr().Set(Vt.Vec3fArray([Gf.Vec3f(71.0 / 255.0, 125.0 / 255.0, 1.0)]))
        particleUtils.configure_particle_set(points.GetPrim(), particleSystemPath, True, True, 0)

        # reference the particle set in the sampling api
        samplingApi.CreateParticlesRel().AddTarget(particlePath)
