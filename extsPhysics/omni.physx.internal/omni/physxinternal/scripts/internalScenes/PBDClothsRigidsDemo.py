from omni.physx.scripts import utils
from omni.physx.scripts import physicsUtils, particleUtils
import omni.physxdemos as demo
import omni.physx.bindings._physx as physx_settings_bindings
from pxr import UsdGeom, UsdLux, Sdf, Gf, Vt, UsdPhysics, PhysxSchema

class ClothsRigidsDemo(demo.Base):
    title = "Cloth and rigids"
    category = demo.Categories.INTERNAL
    short_description = "Cloth interacting with rigid bodies"
    description = "This snippet shows a particle system based cloth interacting with rigid bodies."

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
        utils.set_physics_scene_asyncsimrender(scene.GetPrim())

        dimX = 128
        dimY = 128
        radius = 7.0

        # Particle System
        particleSystemPath = Sdf.Path("/particleSystem0")

        restOffset = radius
        contactOffset = radius + 0.1
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

        # Simple Cloth
        clothPath = Sdf.Path("/cloth0")

        positions_list = []
        normals_list = []
        restPositions_list = []
        velocities_list = []
        triangleIndices_list = []
        springIndices_list = []
        springStiffnesses_list = []
        springDampings_list = []
        springRestLengths_list = []

        lower = Gf.Vec3f(-radius * dimX / 2, -radius * dimY / 2, 0.0)
        stretchStiffness = 20.0
        bendStiffness = 10.0
        shearStiffness = 5.0
        damping = 0.01
        mass = 1.0

        useAttachment = True
        attachmentPositions_list = []

        particleUtils.create_spring_grid(
            lower,
            dimX,
            dimY,
            radius,
            positions_list,
            normals_list,
            restPositions_list,
            velocities_list,
            triangleIndices_list,
            springIndices_list,
            springStiffnesses_list,
            springDampings_list,
            springRestLengths_list,
            stretchStiffness,
            damping,
            bendStiffness,
            damping,
            shearStiffness,
            damping,
            useAttachment,
            attachmentPositions_list
        )

        positions = Vt.Vec3fArray(positions_list)
        normals = Vt.Vec3fArray(normals_list)
        restPositions = Vt.Vec3fArray(restPositions_list)
        velocities = Vt.Vec3fArray(velocities_list)
        triangleIndices = Vt.IntArray(triangleIndices_list)
        springIndices = Vt.Vec2iArray(springIndices_list)
        springStiffnesses = Vt.FloatArray(springStiffnesses_list)
        springDampings = Vt.FloatArray(springDampings_list)
        springRestLengths = Vt.FloatArray(springRestLengths_list)

        particleUtils.add_physx_particle_cloth_with_constraints(
            stage,
            clothPath,
            particleSystemPath,
            positions,
            normals,
            restPositions,
            velocities,
            triangleIndices,
            springIndices,
            springStiffnesses,
            springDampings,
            springRestLengths,
            self_collision=True,
            self_collision_filter=True,
            particle_group=0
        )

        clothPrim = stage.GetPrimAtPath(clothPath)
        massApi = UsdPhysics.MassAPI.Apply(clothPrim)
        massApi.GetMassAttr().Set(mass)

        # Set color
        color0 = Vt.Vec3fArray([Gf.Vec3f(71.0 / 255.0, 165.0 / 255.0, 1.0)])
        gprim0 = UsdGeom.Mesh.Define(stage, Sdf.Path("/cloth0"))
        gprim0.CreateDisplayColorAttr(color0)

        # Local to World
        height = 800.0
        translation = Gf.Vec3f(0.0, 0.0, height)
        gprim0.AddTranslateOp().Set(translation)

        if useAttachment is True:
            attachment_path = Sdf.Path("/cloth0_attachment")
            attachment = PhysxSchema.PhysxPhysicsAttachment.Define(stage, attachment_path)
            attachment.GetActor0Rel().SetTargets([clothPath])
            attachmentPositions = Vt.Vec3fArray(attachmentPositions_list)
            attachment.CreatePoints0Attr().Set(attachmentPositions)
            globalPositions_list = []
            for x in attachmentPositions_list:
                globalPositions_list.append(x + translation);
            globalPositions = Vt.Vec3fArray(globalPositions_list)
            attachment.CreatePoints1Attr().Set(globalPositions)

        numBoxes = 50
        for i in range(numBoxes):
            # Box
            boxActorPath = "/boxActor" + str(i)

            size = Gf.Vec3f(30.0)
            position = Gf.Vec3f(-300.0, 0.0, height + (i + 1) * 45.0)
            orientation = Gf.Quatf(1.0, 0.0, 0.0, 0.0)
            color = Gf.Vec3f(200.0 / 255.0, 165.0 / 255.0, 1.0)
            linVelocity = Gf.Vec3f(20.0, 10.0, 20.0)
            angularVelocity = Gf.Vec3f(1.0, 0.0, 0.0)
            density = 0.2 * (0.01 * 0.01 * 0.01)

            physicsUtils.add_rigid_box(
                stage,
                boxActorPath,
                size,
                position,
                orientation,
                color,
                density,
                linVelocity,
                angularVelocity,
            )

        # Plane
        physicsUtils.add_ground_plane(stage, "/groundPlane", "Z", 750.0, Gf.Vec3f(0.0), Gf.Vec3f(0.5))
