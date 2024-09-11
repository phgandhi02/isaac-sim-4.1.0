from omni.physx.scripts import physicsUtils, particleUtils, utils
from pxr import Usd, UsdLux, UsdGeom, Sdf, Gf, Vt, UsdPhysics, PhysxSchema
import omni.physxdemos as demo
import omni.timeline
import carb
import os
import numpy as np
import omni.kit.commands

import omni.physx.bindings._physx as physx_settings_bindings


def set_initial_camera(stage, position, target):
    customLayerData = {
        "cameraSettings": {
            "Perspective": {"position": position, "radius": 500, "target": target},
            "boundCamera": "/OmniverseKit_Persp",
        }
    }
    stage.GetRootLayer().customLayerData = customLayerData


class FluidIsosurfaceDiffuseParticlesGlassBoxDemo(demo.AsyncDemoBase):
    title = "Fluid Isosurface Diffuse Particles Glass Box"
    category = demo.Categories.INTERNAL_SAMPLES
    short_description = "Fluid in a glass box rendered as fluid isosurface including diffuse particles"
    description = (
        "This snippet sets up a glass box scene rendered with fluid Isosurface method including diffuse particles."
    )

    kit_settings = {
        "persistent/app/viewport/displayOptions": demo.get_viewport_minimal_display_options_int(),
        physx_settings_bindings.SETTING_MIN_FRAME_RATE: 30,
        physx_settings_bindings.SETTING_UPDATE_VELOCITIES_TO_USD: False,
        physx_settings_bindings.SETTING_UPDATE_PARTICLES_TO_USD: False,
        physx_settings_bindings.SETTING_UPDATE_TO_USD: True,
        "rtx-defaults/pathtracing/lightcache/cached/enabled": False,
        "rtx-defaults/pathtracing/cached/enabled": False,
        "rtx-defaults/pathtracing/fireflyFilter/maxIntensityPerSample": 10000,
        "rtx-defaults/pathtracing/fireflyFilter/maxIntensityPerSampleDiffuse": 50000,
        "rtx-defaults/pathtracing/optixDenoiser/blendFactor": 0.09,
        "rtx-defaults/pathtracing/aa/op": 2,
        "rtx-defaults/pathtracing/maxBounces": 32,
        "rtx-defaults/pathtracing/maxSpecularAndTransmissionBounces": 16,
        "rtx/translucency/maxRefractionBounces": 12,
    }

    def __init__(self):
        super().__init__(enable_tensor_api=True, enable_fabric=True)

    def create(self, stage):
        self._setup_flow()
        numParticlesZ = 30

        # set up axis to z
        UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
        UsdGeom.SetStageMetersPerUnit(stage, 1)

        self._setup_callbacks()
        self.stage = stage
        self.time = 0
        self.it = 0

        set_initial_camera(self.stage, Gf.Vec3d(-30.0, 4.0, 20.0), Gf.Vec3d(10.0, 13.0, 6.0))

        data_folder = os.path.normpath(os.path.join(os.path.dirname(os.path.abspath(__file__)), "../../../.."))
        box_path = os.path.normpath(data_folder + "/data/usd/assets/fluid_box2.usda")
        stage.GetDefaultPrim().GetReferences().AddReference(box_path, "/World")
        self._default_prim_path = stage.GetDefaultPrim().GetPath()

        # light
        sphereLightPath = self._default_prim_path.AppendChild("SphereLight05")
        sphereLight = UsdLux.SphereLight.Define(stage, sphereLightPath)
        sphereLight.CreateRadiusAttr(150)
        sphereLight.CreateIntensityAttr(30000)
        sphereLight.AddTranslateOp().Set(Gf.Vec3f(650.0, 0.0, 1150.0))

        # Physics scene
        scenePath = self._default_prim_path.AppendChild("physicsScene")
        scene = UsdPhysics.Scene.Define(stage, scenePath)
        scene.CreateGravityDirectionAttr().Set(Gf.Vec3f(0.0, 0.0, -1.0))
        scene.CreateGravityMagnitudeAttr().Set(10.0)
        utils.set_physics_scene_asyncsimrender(scene.GetPrim())
        physxAPI = PhysxSchema.PhysxSceneAPI.Apply(scene.GetPrim())
        physxAPI = PhysxSchema.PhysxSceneAPI.Apply(scene.GetPrim())
        physxAPI.CreateSolverTypeAttr("TGS")

        radius = 0.2

        # Enable Flow rendering, plus shadows, reflections, and translucency
        flowRenderSettings = {
            "rtx:flow:pathTracingEnabled": True,
            "rtx:flow:pathTracingShadowsEnabled": True,
            "rtx:flow:enabled": True,
            "rtx:flow:rayTracedReflectionsEnabled": True,
            "rtx:flow:rayTracedTranslucencyEnabled": True,
            "rtx:flow:rayTracedShadowsEnabled": True,
            "rtx:flow:compositeEnabled": True,
            "rtx:flow:useFlowLibraryComposite": True,
            "rtx:flow:useFlowLibrarySelfShadow": True,
        }

        metadata = stage.GetMetadata("customLayerData")
        if "renderSettings" in metadata:
            renderSettings = metadata["renderSettings"]
        else:
            renderSettings = {}
        metadata["renderSettings"] = {**renderSettings, **flowRenderSettings}
        stage.SetMetadata("customLayerData", metadata)

        # Particle System
        particleSystemPath = Sdf.Path("/World/particleSystem0")

        # particle params
        particleSpacing = 0.2
        restOffset = particleSpacing * 0.9
        solidRestOffset = restOffset
        fluidRestOffset = restOffset * 0.6
        particleContactOffset = max(solidRestOffset + 0.001, fluidRestOffset / 0.6)
        contactOffset = restOffset*1.5 + 0.01
        particleSystem = particleUtils.add_physx_particle_system(
            stage=stage,
            particle_system_path=particleSystemPath,
            contact_offset=contactOffset,
            rest_offset=restOffset*1.5,
            particle_contact_offset=particleContactOffset,
            solid_rest_offset=solidRestOffset*1.5,
            fluid_rest_offset=fluidRestOffset,
            solver_position_iterations=4,
            simulation_owner=scenePath,
        )

        particleSystem.CreateMaxVelocityAttr().Set(40.0)

        particleSystemPrim = particleSystem.GetPrim()

        # Render material
        pbd_particle_material_path = Sdf.Path("/World/Looks/OmniSurface_DeepWater")
        omni.kit.commands.execute("BindMaterial", prim_path=particleSystemPath, material_path=pbd_particle_material_path)

        # Create a pbd particle material and set it on the particle system
        particleUtils.add_pbd_particle_material(stage, pbd_particle_material_path, cohesion=0.01, viscosity=0.0091, surface_tension=0.0074, friction=0.1)
        physicsUtils.add_physics_material_to_prim(stage, particleSystemPrim, pbd_particle_material_path)

        # apply particle smooting
        smoothingAPI = PhysxSchema.PhysxParticleSmoothingAPI.Apply(particleSystemPrim)
        smoothingAPI.CreateParticleSmoothingEnabledAttr().Set(True)
        smoothingAPI.CreateStrengthAttr().Set(0.8)

        # apply isosurface params
        isosurfaceAPI = PhysxSchema.PhysxParticleIsosurfaceAPI.Apply(particleSystemPrim)
        isosurfaceAPI.CreateIsosurfaceEnabledAttr().Set(True)
        isosurfaceAPI.CreateMaxVerticesAttr().Set(1024 * 1024)
        isosurfaceAPI.CreateMaxTrianglesAttr().Set(2 * 1024 * 1024)
        isosurfaceAPI.CreateMaxSubgridsAttr().Set(1024 * 8)
        isosurfaceAPI.CreateGridSpacingAttr().Set(fluidRestOffset*1.5)

        isosurfaceAPI.CreateSurfaceDistanceAttr().Set(fluidRestOffset*1.6)
        isosurfaceAPI.CreateGridFilteringPassesAttr().Set("")
        isosurfaceAPI.CreateGridSmoothingRadiusAttr().Set(fluidRestOffset*2.0)

        isosurfaceAPI.CreateNumMeshSmoothingPassesAttr().Set(2)
        isosurfaceAPI.CreateNumMeshNormalSmoothingPassesAttr().Set(2)

        primVarsApi = UsdGeom.PrimvarsAPI(particleSystemPrim);
        primVarsApi.CreatePrimvar("doNotCastShadows", Sdf.ValueTypeNames.Bool).Set(True);

        stage.SetInterpolationType(Usd.InterpolationTypeHeld)

        # Simple Particle
        particleInstancePath = Sdf.Path("/World/particlesInstance0")

        lower = Gf.Vec3f(12.0, 1.5, 2.0)
        positions, velocities = particleUtils.create_particles_grid(lower, particleSpacing+0.01, 50, 100, numParticlesZ)

        uniform_range = radius * 0.2
        _rng_seed = 42
        _rng = np.random.default_rng(_rng_seed)
        for i in range(len(positions)):
            positions[i][0] += _rng.uniform(-uniform_range, uniform_range)
            positions[i][1] += _rng.uniform(-uniform_range, uniform_range)
            positions[i][2] += _rng.uniform(-uniform_range, uniform_range)

        particles = particleUtils.add_physx_particleset_points(
            stage,
            particleInstancePath,
            Vt.Vec3fArray(positions),
            Vt.Vec3fArray(velocities),
            [0.25] * len(positions),
            particleSystemPath,
            self_collision=True,
            fluid=True,
            particle_group=0,
            particle_mass=0.001,
            density=0.0,
        )

        # Diffuse particle settings
        maxDiffuseParticleMultiplier = float(250000/len(positions))
        diffuseParticlesAPI = PhysxSchema.PhysxDiffuseParticlesAPI.Apply(particles.GetPrim())
        diffuseParticlesAPI.CreateDiffuseParticlesEnabledAttr().Set(True)
        diffuseParticlesAPI.CreateMaxDiffuseParticleMultiplierAttr().Set(maxDiffuseParticleMultiplier)
        diffuseParticlesAPI.CreateThresholdAttr().Set(220.0)
        diffuseParticlesAPI.CreateLifetimeAttr().Set(3.0)
        diffuseParticlesAPI.CreateAirDragAttr().Set(3.0)
        diffuseParticlesAPI.CreateBubbleDragAttr().Set(0.9)
        diffuseParticlesAPI.CreateBuoyancyAttr().Set(0.9)

        diffuseParticlesAPI.CreateKineticEnergyWeightAttr().Set(3.0)
        diffuseParticlesAPI.CreatePressureWeightAttr().Set(1.0)
        diffuseParticlesAPI.CreateDivergenceWeightAttr().Set(1.0)
        diffuseParticlesAPI.CreateCollisionDecayAttr().Set(0.5)

        # Diffuse particle rendering
        flowSimulate = self.stage.GetPrimAtPath("/World/flowSimulate")
        # densityCellSize influences the size of the diffuse paricle on the screen
        flowSimulate.CreateAttribute("densityCellSize", Sdf.ValueTypeNames.Float, True).Set(0.07)

        flowRayMarch = self.stage.GetPrimAtPath("/World/flowRender/rayMarch")
        # attenuation influences the transparency of the diffuse paricle on the screen
        flowRayMarch.CreateAttribute("attenuation", Sdf.ValueTypeNames.Float, True).Set(0.4)
        # colorScale influences the brightness of the diffuse paricle on the screen
        flowRayMarch.CreateAttribute("colorScale", Sdf.ValueTypeNames.Float, True).Set(2.0)

        self.movingWallY = -1.0535
        self.stepY = 0.0
        self.stepSize = 0.0
        self.wall_init_translation = Gf.Vec3f(10.58994, -1.0535, 4.78121)

    def on_physics_step(self, dt):
        yMin = -1.0535
        yMax = 10
        # print(f"Time is {self.time} and dt = {dt}")
        if self.movingWallY <= yMin:
            self.stepY = self.stepSize
        elif self.movingWallY >= yMax:
            self.stepY = -self.stepSize

        self.movingWallY += self.stepY * dt * 60.0

        #movingWall = UsdGeom.Xformable(self.stage.GetPrimAtPath("/World/Xform/Cube_02"))
        #ops = movingWall.GetOrderedXformOps()
        #translateOp = ops[0]
        #translateOp.Set(Gf.Vec3f(10.58994, self.movingWallY, 4.78121))

        
        movingTransform = np.zeros((1, 7), dtype=np.float32)
        movingTransform[0][0] = self.wall_init_translation[0]
        movingTransform[0][1] = self.wall_init_translation[1] + self.movingWallY
        movingTransform[0][2] = self.wall_init_translation[2]
        movingTransform[0][6] = 1.0

        self.wallTensorView.set_kinematic_targets(movingTransform, self.wallTensorInd)

        # wait some more time in the beginning
        if self.it == 0 and self.time < 5:
            self.time += dt
            return

        self.stepSize = 0.05

    def on_tensor_start(self, tensorApi):
        sim = tensorApi.create_simulation_view("numpy")
        self.wallTensorView = sim.create_rigid_body_view("/World/Xform/Cube_02")
        assert self.wallTensorView.count == 1
        self.wallTensorInd = np.arange(self.wallTensorView.count, dtype=np.int32)
        # prevent garbage collector from deleting the sim 
        self.sim = sim

    def on_timeline_event(self, e):
        if e.type == int(omni.timeline.TimelineEventType.STOP):
            self.time = 0
            self.it = 0
            self.movingWallY = -1.0535
            self.stepY = 0.0
            self.stepSize = 0.0

    def _setup_flow(self):
        manager = omni.kit.app.get_app().get_extension_manager()
        self._flow_was_enabled = manager.is_extension_enabled("omni.flowusd")
        if not self._flow_was_enabled:
            manager.set_extension_enabled_immediate("omni.flowusd", True)

    def on_shutdown(self):
        if not self._flow_was_enabled:
            manager = omni.kit.app.get_app().get_extension_manager()
            manager.set_extension_enabled_immediate("omni.flowusd", False)
        self.wallTensorView = None
        self.sim = None
        super().on_shutdown()
