import math
from omni.physx.scripts import physicsUtils, particleUtils
from pxr import Usd, UsdGeom, Sdf, Gf, Tf, Vt, UsdPhysics, PhysxSchema
import omni.physxdemos as demo


class ParticlesMultipleDemo(demo.Base):
    title = "Particles - multiple"
    category = demo.Categories.INTERNAL
    short_description = "Multiple particle systems scene setup"
    description = "This snippet sets up a multiple particle systems"      

    def create(self, stage):
        # set up axis to z
        UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
        UsdGeom.SetStageMetersPerUnit(stage, 1)

        # Physics scene
        scenePath = Sdf.Path("/physicsScene")
        scene = UsdPhysics.Scene.Define(stage, scenePath)
        scene.CreateGravityDirectionAttr().Set(Gf.Vec3f(0.0, 0.0, -1.0))
        scene.CreateGravityMagnitudeAttr().Set(9.81)

        for x in range(3):
            # Particle System
            particleSystemStr = "/particleSystem" + str(x)
            particleSystemPath = Sdf.Path(particleSystemStr)
            restOffset = 1.0
            contactOffset = restOffset + 0.005
            particleUtils.add_physx_particle_system(
                stage=stage,
                particle_system_path=particleSystemPath,
                contact_offset=contactOffset,
                rest_offset=restOffset,
                particle_contact_offset=contactOffset,
                solid_rest_offset=restOffset,
                fluid_rest_offset=0.0,
                solver_position_iterations=5,
                simulation_owner=scenePath
            )

            for y in range(x + 1):
                # Simple Particle
                particleInstanceStr = "/particlesInstance" + str(x) + "_" + str(y)
                particleInstancePath = Sdf.Path(particleInstanceStr)
                z_offset = 10.0 + (5.0 * x)
                y_offset = 0.0 + (5.0 * y)
                positions = Vt.Vec3fArray([Gf.Vec3f(-5.0, y_offset, z_offset), Gf.Vec3f(5.0, y_offset, z_offset)])
                velocities = Vt.Vec3fArray([Gf.Vec3f(0.0), Gf.Vec3f(0.0, 0.0, 10.0)])
                particleUtils.add_physx_particleset_pointinstancer(
                    stage, particleInstancePath, positions, velocities, particleSystemPath,
                    self_collision=True, fluid=False, particle_group=0, particle_mass=1.0, density=0.0
                )

                # Set color
                color_rgb = [0.0, 0.0, 0.0]
                color_rgb[x] = 1.0 / (y + 1)
                color = Vt.Vec3fArray([Gf.Vec3f(color_rgb[0], color_rgb[1], color_rgb[2])])
                colorPathStr = particleInstanceStr + "/particlePrototype0"
                gprim = UsdGeom.Sphere.Define(stage, Sdf.Path(colorPathStr))
                gprim.CreateDisplayColorAttr(color)

        # Plane
        physicsUtils.add_ground_plane(stage, "/groundPlane", "Z", 25.0, Gf.Vec3f(0.0), Gf.Vec3f(0.5))
