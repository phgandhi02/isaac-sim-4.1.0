import math
import omni
from omni.physx.scripts import deformableUtils, physicsUtils, particleUtils, deformableMeshUtils
from pxr import Usd, UsdLux, UsdGeom, Sdf, Gf, Tf, Vt, UsdPhysics, PhysxSchema
import omni.physxdemos as demo
import omni.usd


class CollisionGroupsDemo(demo.Base):
    title = "Collision groups"
    category = demo.Categories.INTERNAL
    short_description = "Collision groups filtering"
    description = "This snippet shows collision groups filtering, objects with different colors should not collide with each other."

    def create(self, stage):

        # set up axis to z
        UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
        UsdGeom.SetStageMetersPerUnit(stage, 0.01)

        # light
        sphereLight = UsdLux.SphereLight.Define(stage, Sdf.Path("/SphereLight"))
        sphereLight.CreateRadiusAttr(150)
        sphereLight.CreateIntensityAttr(30000)
        sphereLight.AddTranslateOp().Set(Gf.Vec3f(650.0, 0.0, 1150.0))

        # Physics scene
        scenePath = Sdf.Path("/physicsScene")
        scene = UsdPhysics.Scene.Define(stage, scenePath)
        scene.CreateGravityDirectionAttr().Set(Gf.Vec3f(0.0, 0.0, -1.0))
        scene.CreateGravityMagnitudeAttr().Set(981.0)

        # add collision groups
        rigidGroup = "/physicsScene/collisionGroupRigids"
        softbodyGroup = "/physicsScene/collisionGroupSoftbodies"
        particleGroup = "/physicsScene/collisionGroupParticles"
        collisionGroupRigids = UsdPhysics.CollisionGroup.Define(stage, rigidGroup)
        collisionGroupSoftbodies = UsdPhysics.CollisionGroup.Define(stage, softbodyGroup)
        collisionGroupParticles = UsdPhysics.CollisionGroup.Define(stage, particleGroup)

        # setup groups so that they dont collide with each other
        filteredRel = collisionGroupRigids.CreateFilteredGroupsRel()
        filteredRel.AddTarget(particleGroup)
        filteredRel.AddTarget(softbodyGroup)

        filteredRel = collisionGroupSoftbodies.CreateFilteredGroupsRel()
        filteredRel.AddTarget(rigidGroup)
        filteredRel.AddTarget(particleGroup)

        filteredRel = collisionGroupParticles.CreateFilteredGroupsRel()
        filteredRel.AddTarget(rigidGroup)
        filteredRel.AddTarget(softbodyGroup)

        stackPos = [Gf.Vec3f(50.0, -50.0, 0.0), Gf.Vec3f(-50.0, 50.0, 0.0)]

        # groupRigids
        rigidActorsPath = "/rigidActors"
        UsdGeom.Xform.Define(stage, "/World" + rigidActorsPath)

        size = Gf.Vec3f(50.0)
        color = Gf.Vec3f(71.0 / 255.0, 85.0 / 255.0, 1.0)

        numRigidsPerStack = 2
        for s in range(2):
            for i in range(numRigidsPerStack):
                position = stackPos[s] + Gf.Vec3f(0.0, 0.0, 50.0 + i * 100.0)
                rigidPath = rigidActorsPath + "/rigidActor" + str(s * numRigidsPerStack + i)

                physicsUtils.add_rigid_box(
                    stage,
                    rigidPath,
                    size,
                    position,
                    Gf.Quatf(1.0, 0.0, 0.0, 0.0),
                    color,
                    1000.0,
                    Gf.Vec3f(0.0),
                    Gf.Vec3f(0.0),
                )

                collisionAPI = UsdPhysics.CollisionAPI.Get(stage, "/World" + rigidPath)
                physicsUtils.add_collision_to_collision_group(stage, "/World" + rigidPath, rigidGroup)

        # softbodies/groupSoftbodies
        points, indices = deformableMeshUtils.createTetraVoxelBox(4)
        tetraMeshPath = omni.usd.get_stage_next_free_path(stage, "/tetra_mesh", True)
        tetraMesh = PhysxSchema.TetrahedralMesh.Define(stage, tetraMeshPath)
        tetraMesh.GetPointsAttr().Set(points)
        tetraMesh.GetIndicesAttr().Set(indices)
        tetraMeshColor = Vt.Vec3fArray([Gf.Vec3f(1.0, 165.0 / 255.0, 71.0 / 255.0)])
        tetraMesh.CreateDisplayColorAttr(tetraMeshColor)
        tetraMesh.AddTranslateOp().Set(stackPos[0] + Gf.Vec3f(0.0, 0.0, 300.0))
        tetraMesh.AddOrientOp().Set(Gf.Quatf(1.0))
        tetraMesh.AddScaleOp().Set(Gf.Vec3f(50.0, 50.0, 50.0))

        deformableBodyPath = omni.usd.get_stage_next_free_path(stage, "/deformableBody", True)
        deformableUtils.create_skin_mesh_from_tetrahedral_mesh(deformableBodyPath, stage, tetraMeshPath)

        omni.kit.commands.execute(
            "AddDeformableBodyComponent",
            skin_mesh_path=deformableBodyPath,
            collision_mesh_path=tetraMeshPath,
            simulation_mesh_path=tetraMeshPath,
            solver_position_iteration_count=20,
        )

        deformable_material_path = omni.usd.get_stage_next_free_path(stage, "/deformableBodyMaterial", True)
        deformableUtils.add_deformable_body_material(
            stage,
            deformable_material_path,
            dynamic_friction=0.5,
            damping_scale=0.0,
            poissons_ratio=0.45,
            youngs_modulus=20000.0,
        )
        physicsUtils.add_physics_material_to_prim(
            stage, stage.GetPrimAtPath(deformableBodyPath), deformable_material_path
        )

        # we don't need the tetrahedral mesh anymore
        stage.RemovePrim(tetraMeshPath)

        physicsUtils.add_collision_to_collision_group(stage, deformableBodyPath, softbodyGroup)

        # particles/groupParticles

        radius = 10.0
        particleSystemPath = Sdf.Path("/particleSystem")
        restOffset = radius
        contactOffset = restOffset + 0.1
        particleUtils.add_physx_particle_system(
            stage=stage,
            particle_system_path=particleSystemPath,
            contact_offset=contactOffset,
            rest_offset=restOffset,
            particle_contact_offset=contactOffset,
            solid_rest_offset=restOffset,
            solver_position_iterations=5,
            simulation_owner=scenePath,
        )

        physicsUtils.add_collision_to_collision_group(stage, particleSystemPath, particleGroup)

        # add particle instance
        particlePath = Sdf.Path("/particlesInstance")

        positions = []
        velocities = []

        dimx = dimy = dimz = 2
        for x in range(0, dimx):
            for y in range(0, dimy):
                for z in range(0, dimz):
                    positions.append(stackPos[1] + Gf.Vec3f(x * 2 * radius, y * 2 * radius, 300.0 + z * 2 * radius))
                    velocities.append(Gf.Vec3f(0.0))

        pos_in = Vt.Vec3fArray(positions)
        vel_in = Vt.Vec3fArray(velocities)
        particleUtils.add_physx_particleset_pointinstancer(
            stage,
            particlePath,
            pos_in,
            vel_in,
            particleSystemPath,
            self_collision=True,
            fluid=False,
            particle_group=0,
            particle_mass=1.0,
            density=0.0,
        )

        # set color
        color0 = Vt.Vec3fArray([Gf.Vec3f(71.0 / 255.0, 165.0 / 255.0, 1.0)])
        gprim0 = UsdGeom.Sphere.Define(stage, Sdf.Path("/particlesInstance/particlePrototype0"))
        gprim0.CreateDisplayColorAttr(color0)
        gprim0.CreateRadiusAttr(radius)

        # Plane
        physicsUtils.add_ground_plane(stage, "/groundPlane", "Z", 750.0, Gf.Vec3f(0.0), Gf.Vec3f(0.5))

        return "Demo showing collision groups filtering, the two stacks boxes/spheres/capsules/cones should not collide between each other. Press play (space) to run the simulation."
