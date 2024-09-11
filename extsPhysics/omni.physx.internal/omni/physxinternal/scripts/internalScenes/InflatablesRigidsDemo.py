import os
from omni.physx.scripts import physicsUtils, particleUtils
import omni.physxdemos as demo
from pxr import UsdGeom, UsdLux, Sdf, Gf, Vt, UsdPhysics


class InflatablesFlexDemo(demo.Base):
    title = "Inflatables and Rigids"
    category = demo.Categories.INTERNAL
    short_description = "PBD Inflatables-rigid interaction"
    description = "This snippet shows a particle system based inflatables interacting with rigid bodies."

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

        numInflatablesX = 3
        numInflatablesY = 3

        radius = 15.0

        data_folder = os.path.normpath(os.path.join(os.path.dirname(os.path.abspath(__file__)), "../../../.."))
        box_high_path = os.path.normpath(data_folder + "/data/usd/assets/box_high.usda")
        sphere_high_path = os.path.normpath(data_folder + "/data/usd/assets/sphere_high.usda")

        # Particle System
        particleSystemPath = Sdf.Path("/particleSystem0")

        restOffset = radius
        contactOffset = radius + 1.0
        particleUtils.add_physx_particle_system(
            stage=stage,
            particle_system_path=particleSystemPath,
            contact_offset=contactOffset,
            rest_offset=restOffset,
            particle_contact_offset=contactOffset,
            solid_rest_offset=restOffset,
            fluid_rest_offset=0.0,
            solver_position_iterations=8,
            simulation_owner=scene.GetPath(),
        )

        for i in range(numInflatablesX):
            for j in range(numInflatablesY):
                # Simple Cloth
                index = i * numInflatablesY + j

                clothPathStr = "/inflatables" + str(index)
                clothPath = Sdf.Path(clothPathStr)

                initialVelocity = Gf.Vec3f(0.0, 0.0, 0.0)
                particleMass = 0.5
                stretchStiffness = 20000.0
                bendStiffness = 100.0
                shearStiffness = 100.0
                damping = 0.5

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
                    self_collision=False,
                    self_collision_filter=False,
                    particle_group=index,
                    pressure=1.5
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
                translation = Gf.Vec3f((i - 1) * 300.0, (j - 1) * 300.0, 300.0)
                mesh.AddTranslateOp().Set(translation)
                scale = Gf.Vec3f(200.0, 200.0, 200.0)
                mesh.AddScaleOp().Set(scale)
                rotateX = 45.0 * i
                mesh.AddRotateXOp().Set(rotateX)

        for i in range(numInflatablesX):
            for j in range(numInflatablesY):
                # Simple Cloth
                index = numInflatablesX * numInflatablesY + i * numInflatablesY + j

                clothPathStr = "/inflatables" + str(index)
                clothPath = Sdf.Path(clothPathStr)

                particleMass = 0.5
                stretchStiffness = 20000.0
                bendStiffness = 100.0
                shearStiffness = 100.0
                damping = 0.5

                dynamicMeshPath = sphere_high_path

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
                    particle_group=index,
                    pressure=1.2
                )

                clothPrim = stage.GetPrimAtPath(clothPath)
                mesh = UsdGeom.Mesh(clothPrim)

                mass = particleMass * len(mesh.GetPointsAttr().Get())
                massApi = UsdPhysics.MassAPI.Apply(clothPrim)
                massApi.GetMassAttr().Set(mass)

                # Set color
                color = Vt.Vec3fArray([Gf.Vec3f(71.0 / 255.0, 0.3 * (i + 1), 165.0 / 255.0)])
                mesh = UsdGeom.Mesh.Define(stage, Sdf.Path(clothPathStr))
                mesh.CreateDisplayColorAttr(color)

                # Local to World
                translation = Gf.Vec3f((i - 1) * 300.0, (j - 1) * 300.0, 600.0)
                mesh.AddTranslateOp().Set(translation)
                scale = Gf.Vec3f(100.0, 100.0, 100.0)
                mesh.AddScaleOp().Set(scale)
                rotateX = 45.0 * i
                mesh.AddRotateXOp().Set(rotateX)

        numBoxes = 10
        for i in range(numBoxes):
            # Box
            boxActorPath = "/boxActor" + str(i)

            size = Gf.Vec3f(50.0)
            position = Gf.Vec3f(-500.0, 0.0, 1000.0 + (i * 250.0))
            orientation = Gf.Quatf(1.0, 0.0, 0.0, 0.0)
            color = Gf.Vec3f(200.0 / 255.0, 165.0 / 255.0, 1.0)
            linVelocity = Gf.Vec3f(200.0, 100.0, 200.0)
            angularVelocity = Gf.Vec3f(1.0, 0.0, 0.0)

            physicsUtils.add_rigid_box(
                stage, boxActorPath, size, position, orientation, color, 0.002, linVelocity, angularVelocity
            )

        for i in range(numBoxes):
            # Box
            boxActorPath = "/boxActor" + str(i + numBoxes)

            size = Gf.Vec3f(80.0)
            position = Gf.Vec3f(0.0, 0.0, 1000.0 + (i * 250.0))
            orientation = Gf.Quatf(1.0, 0.0, 0.0, 0.0)
            color = Gf.Vec3f(200.0 / 255.0, 165.0 / 165.0 / 255.0, 1.0)
            linVelocity = Gf.Vec3f(200.0, 100.0, 200.0)
            angularVelocity = Gf.Vec3f(1.0, 0.0, 0.0)

            physicsUtils.add_rigid_box(
                stage, boxActorPath, size, position, orientation, color, 0.002, linVelocity, angularVelocity
            )

        # Plane
        physicsUtils.add_ground_plane(stage, "/groundPlane", "Z", 750.0, Gf.Vec3f(0.0), Gf.Vec3f(0.5))
