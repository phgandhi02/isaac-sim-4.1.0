import math
import os

from omni.physx.scripts import physicsUtils, particleUtils
import omni.physxdemos as demo

from pxr import Usd, UsdGeom, UsdLux, Sdf, Gf, Tf, Vt, UsdPhysics, PhysxSchema


class InflatablesDemo(demo.Base):
    title = "Inflatables"
    category = demo.Categories.INTERNAL
    short_description = "PBD Inflatables scene setup"
    description = "This snippet sets up a particle system based inflatables scene"      

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
        bunny_path = os.path.normpath(
            data_folder
            + "/data/usd/assets/bunny.obj.usda"
        )
        numColumnsInflatables = 2
        numRowsInflatables = 3

        radius = 10.0

        # Particle System
        particleSystemPath = Sdf.Path("/particleSystem0")

        restOffset = radius
        contactOffset = radius + 1.0
        particleUtils.add_physx_particle_system(
            stage,
            particle_system_path=particleSystemPath,
            contact_offset=contactOffset,
            rest_offset=restOffset,
            particle_contact_offset=contactOffset,
            solid_rest_offset=restOffset,
            fluid_rest_offset=0.0,
            solver_position_iterations=5,
            simulation_owner=scene.GetPath()
        )

        pbd_particle_material_path = Sdf.Path("/pbdParticleMaterial")
        particleUtils.add_pbd_particle_material(stage, pbd_particle_material_path, friction=0.4, damping=0.1)
        physicsUtils.add_physics_material_to_prim(stage, stage.GetPrimAtPath(particleSystemPath), pbd_particle_material_path)

        count = 0

        for i in range(numColumnsInflatables):
            for j in range(numRowsInflatables):
                # Simple Cloth
                clothPathStr = "/inflatables" + str(count)
                count = count + 1
                clothPath = Sdf.Path(clothPathStr)

                mass = 5.0
                stretchStiffness = 10000.0
                bendStiffness = 100.0
                shearStiffness = 100.0
                damping = 0.2

                dynamicMeshPath = bunny_path
                pressure = 4.2 - 0.1 * (count)

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
                    particle_group=count,
                    pressure=pressure
                )

                clothPrim = stage.GetPrimAtPath(clothPath)
                massApi = UsdPhysics.MassAPI.Apply(clothPrim)
                massApi.GetMassAttr().Set(mass)

                # Set color
                color = Vt.Vec3fArray([Gf.Vec3f(71.0 / 255.0, 165.0 / 255.0, 0.3 * (i + 1))])
                gprim = UsdGeom.Mesh.Define(stage, Sdf.Path(clothPathStr))
                gprim.CreateDisplayColorAttr(color)

                # Local to World
                translation = Gf.Vec3f(0.0, -500.0 * i, 600.0 + 300.0 * j)
                gprim.AddTranslateOp().Set(translation)
                scale = Gf.Vec3f(50.0, 50.0, 50.0)
                gprim.AddScaleOp().Set(scale)
                rotateX = 45.0 * i
                gprim.AddRotateXOp().Set(rotateX)

        # Plane
        physicsUtils.add_ground_plane(stage, "/groundPlane", "Z", 750.0, Gf.Vec3f(0.0), Gf.Vec3f(0.5))
