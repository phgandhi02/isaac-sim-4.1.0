import math
import os

from omni.physx.scripts import physicsUtils, particleUtils, deformableUtils
import omni.physxdemos as demo
import omni.usd


from pxr import Usd, UsdGeom, UsdLux, Sdf, Gf, Tf, Vt, UsdPhysics, PhysxSchema


class Mixed2Demo(demo.Base):
    title = "Mixed 2"
    category = demo.Categories.INTERNAL_SAMPLES
    short_description = "Two way coupling of rigids, cloth and inflatables"
    description = "This snippet sets up a scene where rigid bodies, cloth and inflatables interact with each other"

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

        data_folder = os.path.normpath(os.path.join(os.path.dirname(os.path.abspath(__file__)), "../../../.."))
        box_high_path = os.path.normpath(data_folder + "/data/usd/assets/box_high.usda")

        numParticles = 250000
        radius = 5.0

        clothDimX = 96
        clothDimY = 96

        numCloths = 1
        numInflatables = 1

        stackSpacingCloths = 270.0
        stackSpacingInflatables = 300.0

        clothesWorldPos = Gf.Vec3f(0.0, 0.0, 1000.0)
        inflatablesWorldPos = Gf.Vec3f(0.0, 0.0, 300.0)
        boxesWorldPos = Gf.Vec3f(0.0, 0.0, 600.0)

        # Particle System
        particleSystemPath = Sdf.Path("/particleSystem0")

        restOffset = radius
        contactOffset = radius + 1.0
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
            solver_position_iterations=8,
            simulation_owner=scene.GetPath(),
        )

        pbd_particle_material_path = Sdf.Path("/pbdParticleMaterial")
        particleUtils.add_pbd_particle_material(stage, pbd_particle_material_path)
        physicsUtils.add_physics_material_to_prim(
            stage, stage.GetPrimAtPath(particleSystemPath), pbd_particle_material_path
        )

        mass = 4.0
        stretchStiffness = 100.0
        bendStiffness = 20.0
        shearStiffness = 5.0
        damping = 0.1

        for i in range(numInflatables):
            # Simple Cloth
            clothPathStr = "/inflatables" + str(i)
            clothPath = Sdf.Path(clothPathStr)

            dynamicMeshPath = box_high_path

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
                pressure=1.2,
            )

            clothPrim = stage.GetPrimAtPath(clothPath)
            massApi = UsdPhysics.MassAPI.Apply(clothPrim)
            massApi.GetMassAttr().Set(mass)

            # Set color
            color = Vt.Vec3fArray([Gf.Vec3f(71.0 / 255.0, 165.0 / 255.0, 0.3 * (i + 1))])
            gprim = UsdGeom.Mesh.Define(stage, Sdf.Path(clothPathStr))
            gprim.CreateDisplayColorAttr(color)

            # Local to World
            translation = inflatablesWorldPos + Gf.Vec3f(0.0, 0.0, i * stackSpacingInflatables)
            gprim.AddTranslateOp().Set(translation)
            scale = Gf.Vec3f(200.0, 200.0, 200.0)
            gprim.AddScaleOp().Set(scale)
            rotateX = 0.0
            gprim.AddRotateXOp().Set(rotateX)

        for i in range(numCloths):

            # Create grid mesh
            clothMeshPath = omni.usd.get_stage_next_free_path(stage, "/cloth" + str(i), True)
            clothMesh = UsdGeom.Mesh.Define(stage, clothMeshPath)
            tri_points, tri_indices = deformableUtils.create_triangle_mesh_square(clothDimX, clothDimY)
            clothMesh.GetPointsAttr().Set(tri_points)
            clothMesh.GetFaceVertexIndicesAttr().Set(tri_indices)
            clothMesh.GetFaceVertexCountsAttr().Set([3] * (len(tri_indices) // 3))
            color_rgb = [0.0, 0.0, 0.0]
            color_rgb[i % 3] = 1.0
            color = Vt.Vec3fArray([Gf.Vec3f(color_rgb[0], color_rgb[1], color_rgb[2])])
            clothMesh.CreateDisplayColorAttr(color)
            clothMesh.AddTranslateOp().Set(Gf.Vec3f(0.0, 0.0, 500.0))
            clothMesh.AddScaleOp().Set(Gf.Vec3f(800.0, 800.0, 800.0))

            particleUtils.add_physx_particle_cloth(
                stage=stage,
                path=clothMeshPath,
                dynamic_mesh_path=None,
                particle_system_path=particleSystemPath,
                spring_stretch_stiffness=stretchStiffness,
                spring_bend_stiffness=bendStiffness,
                spring_shear_stiffness=shearStiffness,
                spring_damping=damping,
                self_collision=True,
                self_collision_filter=True,
                particle_group=numInflatables + i,
            )

            clothPrim = stage.GetPrimAtPath(clothMeshPath)
            massApi = UsdPhysics.MassAPI.Apply(clothPrim)
            massApi.GetMassAttr().Set(mass)

        for i in range(2):
            for j in range(2):
                # Box
                boxActorPath = "/boxActor" + str(j) + str(i)

                size = Gf.Vec3f(60.0)
                position = boxesWorldPos + Gf.Vec3f((i - 0.5) * 600.0, (j - 0.5) * 600.0, 0.0)
                orientation = Gf.Quatf(1.0, 0.0, 0.0, 0.0)
                color = Gf.Vec3f(200.0 / 255.0, 165.0 / 255.0, 1.0)
                linVelocity = Gf.Vec3f(0.0, 0.0, 0.0)  # Gf.Vec3f(2.0, 1.0, 2.0)
                angularVelocity = Gf.Vec3f(0.0, 0.0, 0.0)  # Gf.Vec3f(1.0, 0.0, 0.0)

                physicsUtils.add_rigid_box(
                    stage, boxActorPath, size, position, orientation, color, 20.0, linVelocity, angularVelocity
                )

        # Plane
        physicsUtils.add_ground_plane(stage, "/groundPlane", "Z", 750.0, Gf.Vec3f(0.0), Gf.Vec3f(0.5))
