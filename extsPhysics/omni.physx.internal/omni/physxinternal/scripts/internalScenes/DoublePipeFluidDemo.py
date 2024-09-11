from pxr import Usd, UsdLux, UsdGeom, Sdf, Gf, Tf, Vt, UsdPhysics, PhysxSchema
from omni.physx.scripts import physicsUtils, particleUtils, utils
import omni.physxdemos as demo
import omni.physx.bindings._physx as physx_settings_bindings
import omni.usd
import carb
import math
import random

class DoublePipeFluidDemo(demo.Base):
    title = "Double-Pipe Fluid Flow"
    category = demo.Categories.INTERNAL_SAMPLES
    short_description = "Fluid flowing through two spiral-shaped tubes."
    description = "Position-based-dynamics fluid flowing through two spiral-shaped tubes rendered using a marching-cube-extracted isosurface."

    kit_settings = {
        "persistent/app/viewport/displayOptions": demo.get_viewport_minimal_display_options_int(),
        physx_settings_bindings.SETTING_MIN_FRAME_RATE: 30,
        physx_settings_bindings.SETTING_UPDATE_VELOCITIES_TO_USD: False,
        physx_settings_bindings.SETTING_UPDATE_PARTICLES_TO_USD: False,
        physx_settings_bindings.SETTING_UPDATE_TO_USD: True,
        "rtx/post/aa/op": 1,
        "rtx/post/dlss/execMode": 1,
        "rtx/translucency/maxRefractionBounces": 6
    }

    def set_initial_camera(self, stage, position, target):
        customLayerData = {
            "cameraSettings": {
                "Perspective": {"position": position, "radius": 500, "target": target},
                "boundCamera": "/OmniverseKit_Persp",
            }
        }
        stage.GetRootLayer().customLayerData = customLayerData

    def CreateParticleCylinder(self, center, radius, height, particleSpacing, positons, velocities, protoIndices):
        
        dimXZ = math.ceil(2 * radius / particleSpacing)
        dimY =  math.ceil(height / particleSpacing)

        for i in range(dimXZ):
            for j in range(dimY):
                for k in range(dimXZ):
                    x = i * particleSpacing - radius
                    y = j * particleSpacing - height*0.5
                    z = k * particleSpacing - radius
                    
                    d2 = x*x+z*z
                    if d2 < radius*radius:
                        positons.append(Gf.Vec3f(center[0] + x, center[1] + y, center[2] + z))
                        velocities.append(Gf.Vec3f(0.0, 0.0, 0.0))
                        protoIndices.append(0)

    def create(self, stage):
        cam_position = Gf.Vec3d(-80.86733545825442, 237.07501608209122, 196.97956114457173)
        cam_target = Gf.Vec3d(37.015096682555175, 100.97798820743212, 37.76753503009586)
        self.set_initial_camera(stage, cam_position, cam_target)

        scene_asset_path = "omniverse://ov-content/Projects/DemoContent/DoNotDistribute/Physics_Preview_Demos/fluid_pipes/PipeFluidFlowRaw.usd"
        stage.GetDefaultPrim().GetReferences().AddReference(scene_asset_path, "/stage")
        defaultPrimPath = stage.GetDefaultPrim().GetPath()
        scenePath = defaultPrimPath.AppendChild("PhysicsScene")
        scenePrim = stage.GetPrimAtPath(scenePath)
        utils.set_physics_scene_asyncsimrender(scenePrim)

         # Particle System
        particleSystemPath = defaultPrimPath.AppendChild("particleSystem0")

        # particle params
        particleSpacing = 0.9
        restOffset = 0.6
        solidRestOffset = 1.0
        fluidRestOffset = 0.45
        particleContactOffset = 1.15
        particle_system = particleUtils.add_physx_particle_system(
            stage=stage,
            particle_system_path=particleSystemPath,
            simulation_owner=scenePath,
            contact_offset=1.15,
            rest_offset=restOffset,
            particle_contact_offset=particleContactOffset,
            solid_rest_offset=solidRestOffset,
            fluid_rest_offset=fluidRestOffset,
            solver_position_iterations=4,
            max_neighborhood=196
        )

        pbd_particle_material_path = defaultPrimPath.AppendPath("Looks/Paint")

        # Create a pbd particle material and set it on the particle system
        
        print(stage.GetPrimAtPath(pbd_particle_material_path))
        materialAPI = PhysxSchema.PhysxPBDMaterialAPI.Apply(stage.GetPrimAtPath(pbd_particle_material_path))
        materialAPI.CreateFrictionAttr().Set(0.1)
        materialAPI.CreateDampingAttr().Set(0.0)
        materialAPI.CreateViscosityAttr().Set(100)
        materialAPI.CreateVorticityConfinementAttr().Set(0.0)
        materialAPI.CreateSurfaceTensionAttr().Set(0)
        materialAPI.CreateCohesionAttr().Set(1)
        materialAPI.CreateLiftAttr().Set(0.0)
        materialAPI.CreateDragAttr().Set(0.0)
        materialAPI.CreateCflCoefficientAttr().Set(1.0)

        omni.kit.commands.execute("BindMaterial", prim_path=particleSystemPath, material_path=pbd_particle_material_path)
        physicsUtils.add_physics_material_to_prim(stage, particle_system.GetPrim(), pbd_particle_material_path)

        particle_system.CreateMaxVelocityAttr().Set(100)

        # apply some particle position smoothing
        smoothingAPI =  PhysxSchema.PhysxParticleSmoothingAPI.Apply(particle_system.GetPrim())
        smoothingAPI.CreateParticleSmoothingEnabledAttr().Set(True)
        smoothingAPI.CreateStrengthAttr().Set(0.5)

        # apply isosurface params
        isosurfaceAPI = PhysxSchema.PhysxParticleIsosurfaceAPI.Apply(particle_system.GetPrim())
        isosurfaceAPI.CreateIsosurfaceEnabledAttr().Set(True)
        isosurfaceAPI.CreateMaxVerticesAttr().Set(1024 * 1024)
        isosurfaceAPI.CreateMaxTrianglesAttr().Set(2 * 1024 * 1024)
        isosurfaceAPI.CreateMaxSubgridsAttr().Set(1024 * 8)
        isosurfaceAPI.CreateGridSpacingAttr().Set(fluidRestOffset*1.5)

        isosurfaceAPI.CreateSurfaceDistanceAttr().Set(fluidRestOffset*1.8)
        isosurfaceAPI.CreateGridFilteringPassesAttr().Set("S")
        isosurfaceAPI.CreateGridSmoothingRadiusAttr().Set(fluidRestOffset*1.0)

        isosurfaceAPI.CreateNumMeshSmoothingPassesAttr().Set(2)

        primVarsApi = UsdGeom.PrimvarsAPI(particle_system);
        primVarsApi.CreatePrimvar("doNotCastShadows", Sdf.ValueTypeNames.Bool).Set(False);
        
        particlesPath = defaultPrimPath.AppendChild("particles")

        positions = []
        velocities = []
        protoIndices = []
        
        center = Gf.Vec3f(69.13, 177.41754, -1.25)
        self.CreateParticleCylinder(center, 38*0.5, 25, particleSpacing, positions, velocities, protoIndices)
        center = Gf.Vec3f(-0.242, 177.41754, -1.25)
        self.CreateParticleCylinder(center, 38*0.5, 25, particleSpacing, positions, velocities, protoIndices)
        
        uniform_range = particleSpacing * 0.7

        for i in range(len(positions)):
            pos = positions[i]
            pos[0] += random.uniform(0, uniform_range) - uniform_range * 0.5
            pos[1] += random.uniform(0, uniform_range) - uniform_range * 0.5
            pos[2] += random.uniform(0, uniform_range) - uniform_range * 0.5
            positions[i] = pos

        widths = [particleSpacing] * len(positions)
        particleUtils.add_physx_particleset_points(
            stage,
            particlesPath,
            Vt.Vec3fArray(positions),
            Vt.Vec3fArray(velocities),
            widths,
            particleSystemPath,
            self_collision=True,
            fluid=True,
            particle_group=0,
            particle_mass=0.001,
            density=0.0
        )

