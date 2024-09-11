import math
from omni.physx.scripts import physicsUtils, particleUtils
from pxr import Usd, UsdLux, UsdGeom, Sdf, Gf, Tf, Vt, UsdPhysics
import omni.physxdemos as demo
import omni.usd

class FluidAnisotropyDemo(demo.Base):
    title = "Fluid with Anisotropy"
    category = demo.Categories.INTERNAL
    short_description = "PBD Fluid-rigids interaction with anisotropy visualization"
    description = "This snippet shows a PBD fluid interacting with rigid bodies. Fluid Anisotropy is visualized."       

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

        numParticles = 100000
        numInflatables = 0
        numCloths = 0

        radius = 0.2

        worldPos = Gf.Vec3f(0.0, 0.0, 0.0)
        # worldPos = Gf.Vec3f(-452.0, -970.0, 84.0)

        # Particle System
        particleSystemPath = Sdf.Path("/particleSystem0")

        particleSpacing = 0.2
        restOffset = particleSpacing * 0.9
        solidRestOffset = restOffset
        fluidRestOffset = restOffset * 0.6
        particleContactOffset = max(solidRestOffset + 0.001, fluidRestOffset / 0.6)
        contactOffset = restOffset * 2 + 0.001
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

        particleUtils.add_physx_particle_anisotropy(
            stage=stage,
            path=particleSystemPath,
            scale=1.0,
        )

        pbd_particle_material_path = omni.usd.get_stage_next_free_path(stage, "/pbdParticleMaterial", True)
        particleUtils.add_pbd_particle_material(stage, pbd_particle_material_path, cohesion=0.002)
        physicsUtils.add_physics_material_to_prim(stage, stage.GetPrimAtPath(particleSystemPath), pbd_particle_material_path)

        numParticleInstances = 1
        for i in range(numParticleInstances):
            # Simple Particle
            particleInstanceStr = "/particlesInstance" + str(i)
            particleInstancePath = Sdf.Path(particleInstanceStr)

            lower = Gf.Vec3f(15.0 * i, 0.0, 1.0) + worldPos
            particleSpacing = 2.0 * radius * 0.6
            positions, velocities = particleUtils.create_particles_grid(lower, particleSpacing, 20, 20, 10)

            particleUtils.add_physx_particleset_pointinstancer(
                stage, particleInstancePath, Vt.Vec3fArray(positions), Vt.Vec3fArray(velocities), particleSystemPath,
                self_collision=True, fluid=True, particle_group=numCloths + numInflatables + i, particle_mass=0.0, density=0.02
            )

            # Set color
            color_rgb = [0.0, 0.0, 0.0]
            color_rgb[i] = 1.0
            color = Vt.Vec3fArray([Gf.Vec3f(color_rgb[0], color_rgb[1], color_rgb[2])])
            colorPathStr = particleInstanceStr + "/particlePrototype0"
            gprim = UsdGeom.Sphere.Define(stage, Sdf.Path(colorPathStr))
            gprim.CreateDisplayColorAttr(color)
            gprim.CreateRadiusAttr().Set(particleSpacing * 0.5)

            # enable anisotropy and make sure scales and orientations are authored
            pprim = stage.GetPrimAtPath(particleInstanceStr)
            pprim.GetAttribute("scales").Set([(0.5, 0.5, 0.5) for i in range(len(positions))])
            pprim.GetAttribute("orientations").Set(
                [Gf.Quath(0.8660254, 0.0, 0.0, 0.5) for i in range(len(positions))]
            )

        numBoxes = 10
        for i in range(numBoxes):
            # Box
            boxActorPath = "/boxActor" + str(i + numBoxes)

            size = Gf.Vec3f(0.8)
            position = Gf.Vec3f(0.0, 0.0, 10.0 + (i * 2.5))
            orientation = Gf.Quatf(1.0, 0.0, 0.0, 0.0)
            color = Gf.Vec3f(200.0 / 255.0, 165.0 / 165.0 / 255.0, 1.0)
            linVelocity = Gf.Vec3f(2.0, 1.0, 2.0)
            angularVelocity = Gf.Vec3f(1.0, 0.0, 0.0)

            physicsUtils.add_rigid_box(stage, boxActorPath, size, position, orientation, color, 1.0, linVelocity, angularVelocity)

        # Plane
        physicsUtils.add_ground_plane(stage, "/groundPlane", "Z", 25.0, Gf.Vec3f(0.0), Gf.Vec3f(0.5))

        # Wall
        boxActorPath = "/topWallActor"
        size = Gf.Vec3f(10.0, 2.0, 10.0)
        position = Gf.Vec3f(0.0, -10.0, 10.0)
        orientation = Gf.Quatf(1.0, 0.0, 0.0, 0.0)
        color = Gf.Vec3f(0.5)
        linVelocity = Gf.Vec3f(0.0)
        angularVelocity = Gf.Vec3f(0.0, 0.0, 0.0)
        physicsUtils.add_rigid_box(stage, boxActorPath, size, position, orientation, color, 1.0, linVelocity, angularVelocity)

        boxActorPath = "/bottomWallActor"
        position = Gf.Vec3f(0.0, 10.0, 10.0)
        physicsUtils.add_rigid_box(stage, boxActorPath, size, position, orientation, color, 1.0, linVelocity, angularVelocity)
