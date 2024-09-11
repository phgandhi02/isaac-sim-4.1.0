from omni.physx.scripts import physicsUtils, particleUtils
from pxr import UsdLux, UsdGeom, Sdf, Gf, Vt, UsdPhysics
import omni.physxdemos as demo
import omni.usd


class FluidPointsDemo(demo.Base):
    title = "Fluid and Rigids"
    category = demo.Categories.INTERNAL
    short_description = "PBD Fluid-rigids interaction"
    description = "This snippet shows a PBD fluid interacting with rigid bodies."

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

        radius = 0.2

        worldPos = Gf.Vec3f(0.0, 0.0, 0.0)

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
        particleUtils.add_pbd_particle_material(stage, pbd_particle_material_path, cohesion=0.002)
        physicsUtils.add_physics_material_to_prim(stage, stage.GetPrimAtPath(particleSystemPath), pbd_particle_material_path)

        particlePointsStr = "/particles"
        particlePointsPath = Sdf.Path(particlePointsStr)

        positions_list = []
        velocities_list = []
        widths_list = []

        lower = Gf.Vec3f(0.0, 0.0, 4.0) + worldPos
        particleSpacing = 2.0 * radius * 0.6

        x = lower[0]
        y = lower[1]
        z = lower[2]

        for i in range(30):
            for j in range(30):
                for k in range(30):
                    positions_list.append(Gf.Vec3f(x, y, z))
                    velocities_list.append(Gf.Vec3f(0.0, 0.0, 0.0))
                    widths_list.append(2 * radius * 0.5)
                    z = z + particleSpacing
                z = lower[2]
                y = y + particleSpacing
            y = lower[1]
            x = x + particleSpacing

        particles = particleUtils.add_physx_particleset_points(
            stage,
            particlePointsPath,
            positions_list,
            velocities_list,
            widths_list,
            particleSystemPath,
            True,
            True,
            0,
            1.0,
            0.02
        )

        particles.GetDisplayColorAttr().Set(Vt.Vec3fArray([Gf.Vec3f(1.0, 0.0, 0.0)]))

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

            physicsUtils.add_rigid_box(
                stage, boxActorPath, size, position, orientation, color, 1.0, linVelocity, angularVelocity
            )

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
        physicsUtils.add_rigid_box(
            stage, boxActorPath, size, position, orientation, color, 1.0, linVelocity, angularVelocity
        )

        boxActorPath = "/bottomWallActor"
        position = Gf.Vec3f(0.0, 10.0, 10.0)
        physicsUtils.add_rigid_box(
            stage, boxActorPath, size, position, orientation, color, 1.0, linVelocity, angularVelocity
        )
