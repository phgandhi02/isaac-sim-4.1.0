import os
import carb
import math
from omni.physx.scripts import physicsUtils, particleUtils
from pxr import Usd, UsdLux, UsdGeom, Sdf, Gf, Tf, Vt, UsdPhysics
import omni.physxdemos as demo


class MixedDemo(demo.Base):
    title = "Mixed"
    category = demo.Categories.INTERNAL_SAMPLES
    short_description = "Two way coupling of particles, cloth and inflatables"
    description = (
        "This snippet sets up a particle system scene where particles, cloth and inflatables interact with each other"
    )

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

        data_folder = os.path.normpath(os.path.join(os.path.dirname(os.path.abspath(__file__)), "../../../.."))
        chibi_path = os.path.normpath(data_folder + "/data/usd/assets/chibi.obj.usda")
        bunny_path = os.path.normpath(data_folder + "/data/usd/assets/bunny.obj.usda")

        numParticles = 100000
        numInflatables = 3

        radius = 0.2

        worldPos = Gf.Vec3f(0.0, 0.0, 0.0)
        # worldPos = Gf.Vec3f(-452.0, -970.0, 84.0)

        # Particle System
        particleSystemPath = Sdf.Path("/particleSystem0")

        restOffset = radius
        contactOffset = radius + 0.005
        particleContactOffset = contactOffset
        solidRestOffset = restOffset
        fluidRestOffset = contactOffset * 0.6
        particleUtils.add_physx_particle_system(
            stage=stage,
            particle_system_path=particleSystemPath,
            contact_offset=contactOffset,
            rest_offset=restOffset,
            particle_contact_offset=particleContactOffset,
            solid_rest_offset=solidRestOffset,
            fluid_rest_offset=fluidRestOffset,
            solver_position_iterations=16,
            simulation_owner=scenePath,
        )

        pbd_particle_material_path = Sdf.Path("/pbdParticleMaterial")
        particleUtils.add_pbd_particle_material(stage, pbd_particle_material_path)
        physicsUtils.add_physics_material_to_prim(
            stage, stage.GetPrimAtPath(particleSystemPath), pbd_particle_material_path
        )

        numCloths = 2
        for i in range(numCloths):
            # Simple Cloth
            clothPathStr = "/cloth" + str(i)
            clothPath = Sdf.Path(clothPathStr)

            particleMass = 0.02
            stretchStiffness = 1000.0
            bendStiffness = 100.0
            shearStiffness = 100.0
            damping = 0.2

            dynamicMeshPath = chibi_path

            particleUtils.add_physx_particle_cloth(
                stage=stage,
                path=clothPath,
                dynamic_mesh_path=dynamicMeshPath,
                particle_system_path=particleSystemPath,
                spring_stretch_stiffness=stretchStiffness,
                spring_bend_stiffness=bendStiffness,
                spring_shear_stiffness=shearStiffness,
                spring_damping=damping,
                self_collision=True,
                self_collision_filter=True,
                particle_group=i,
            )

            clothPrim = stage.GetPrimAtPath(clothPath)
            mesh = UsdGeom.Mesh(clothPrim)

            mass = particleMass * len(mesh.GetPointsAttr().Get())
            massApi = UsdPhysics.MassAPI.Apply(clothPrim)
            massApi.GetMassAttr().Set(mass)

            # Set color
            color = Vt.Vec3fArray([Gf.Vec3f(0.3 * (i + 1), 71.0 / 255.0, 165.0 / 255.0)])
            mesh.CreateDisplayColorAttr(color)

            # Local to World
            translation = Gf.Vec3f(0.0, 20.0 * i - 20.0, 30.0) + worldPos
            mesh.AddTranslateOp().Set(translation)
            scale = Gf.Vec3f(3.0, 3.0, 3.0)
            mesh.AddScaleOp().Set(scale)
            rotateX = 45.0 * i
            mesh.AddRotateXOp().Set(rotateX)

        for i in range(numInflatables):
            # Simple Cloth
            clothPathStr = "/inflatables" + str(i)
            clothPath = Sdf.Path(clothPathStr)

            particleMass = 0.02
            stretchStiffness = 5000.0
            bendStiffness = 100.0
            shearStiffness = 50.0
            damping = 0.2

            dynamicMeshPath = bunny_path

            particleUtils.add_physx_particle_cloth(
                stage=stage,
                path=clothPath,
                dynamic_mesh_path=dynamicMeshPath,
                particle_system_path=particleSystemPath,
                spring_stretch_stiffness=stretchStiffness,
                spring_bend_stiffness=bendStiffness,
                spring_shear_stiffness=shearStiffness,
                spring_damping=damping,
                self_collision=False,
                self_collision_filter=False,
                particle_group=numCloths + i,
                pressure=0.8,
            )

            clothPrim = stage.GetPrimAtPath(clothPath)
            mesh = UsdGeom.Mesh(clothPrim)

            mass = particleMass * len(mesh.GetPointsAttr().Get())
            massApi = UsdPhysics.MassAPI.Apply(clothPrim)
            massApi.GetMassAttr().Set(mass)

            # Set color
            color = Vt.Vec3fArray([Gf.Vec3f(71.0 / 255.0, 165.0 / 255.0, 0.3 * (i + 1))])
            mesh.CreateDisplayColorAttr(color)

            # Local to World
            translation = Gf.Vec3f(0.0, -25.0 * i, 20.0) + worldPos
            mesh.AddTranslateOp().Set(translation)
            scale = Gf.Vec3f(1.0, 1.0, 1.0)
            mesh.AddScaleOp().Set(scale)
            rotateX = 45.0 * i
            mesh.AddRotateXOp().Set(rotateX)

        numParticleInstances = 1
        for i in range(numParticleInstances):
            # Simple Particle
            particleInstanceStr = "/particlesInstance" + str(i)
            particleInstancePath = Sdf.Path(particleInstanceStr)

            lower = Gf.Vec3f(15.0 * i, 0.0, 50.0) + worldPos
            particleSpacing = 2.0 * radius
            positions, velocities = particleUtils.create_particles_grid(lower, particleSpacing, 10, 10, 10)

            particleUtils.add_physx_particleset_pointinstancer(
                stage,
                particleInstancePath,
                Vt.Vec3fArray(positions),
                Vt.Vec3fArray(velocities),
                particleSystemPath,
                self_collision=True,
                fluid=True,
                particle_group=numCloths + numInflatables + i,
                particle_mass=1.0,
                density=0.0,
            )

            # Set color
            color_rgb = [0.0, 0.0, 0.0]
            color_rgb[i] = 1.0
            color = Vt.Vec3fArray([Gf.Vec3f(color_rgb[0], color_rgb[1], color_rgb[2])])
            colorPathStr = particleInstanceStr + "/particlePrototype0"
            gprim = UsdGeom.Sphere.Define(stage, Sdf.Path(colorPathStr))
            gprim.CreateDisplayColorAttr(color)

            scale = Gf.Vec3f(radius, radius, radius)
            # gprim.AddScaleOp().Set(scale)
            gprim.GetRadiusAttr().Set(radius)

        # Plane
        physicsUtils.add_ground_plane(stage, "/groundPlane", "Z", 25.0, Gf.Vec3f(0.0), Gf.Vec3f(0.5))
