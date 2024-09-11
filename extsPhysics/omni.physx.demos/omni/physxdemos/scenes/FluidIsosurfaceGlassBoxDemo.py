from omni.physx.scripts import physicsUtils, particleUtils
from pxr import Usd, UsdGeom, Sdf, Gf, Vt, UsdPhysics, PhysxSchema
import omni.physx.bindings._physx as physx_settings_bindings
import omni.physxdemos as demo
import omni.timeline
import numpy as np
import omni.kit.commands


class FluidIsosurfaceGlassBoxDemo(demo.AsyncDemoBase):
    title = "Fluid Isosurface"
    category = demo.Categories.PARTICLES
    short_description = "PBD fluid in an invisible box rendered using a marching-cube-extracted isosurface"
    description = (
        "The position-based-dynamics (PBD) fluid is turned into a wave-pool by a moving wall of an invisible box."
    )

    kit_settings = {
        "persistent/app/viewport/displayOptions": demo.get_viewport_minimal_display_options_int(),
        physx_settings_bindings.SETTING_UPDATE_TO_USD: True,
        physx_settings_bindings.SETTING_MIN_FRAME_RATE: 60,
        physx_settings_bindings.SETTING_UPDATE_VELOCITIES_TO_USD: False,
        physx_settings_bindings.SETTING_UPDATE_PARTICLES_TO_USD: False,
        "rtx/pathtracing/lightcache/cached/enabled": False,
        "rtx/pathtracing/cached/enabled": False,
        "rtx/pathtracing/fireflyFilter/maxIntensityPerSample": 10000,
        "rtx/pathtracing/fireflyFilter/maxIntensityPerSampleDiffuse": 50000,
        "rtx/pathtracing/optixDenoiser/blendFactor": 0.09,
        "rtx/pathtracing/aa/op": 2,
        "rtx/pathtracing/maxBounces": 32,
        "rtx/pathtracing/maxSpecularAndTransmissionBounces": 16,
        "rtx/post/dlss/execMode": 1,
        "rtx/translucency/maxRefractionBounces": 12,
    }

    def __init__(self):
        super().__init__(enable_tensor_api=True, enable_fabric=False, fabric_compatible=False)
        self._time = 0
        self._rng_seed = 42
        self._rng = np.random.default_rng(self._rng_seed)
        self._wall_speed = 2.2
        self._yMax = 10
        self._motion_dir = 1
        self._wall_init_translation = Gf.Vec3f(10.58994, -1.0535, 7.8294)
        self._movingWallDeltaY = 0.0
        self._movingWall = None
        self._wallTensorView = None
        self._is_running = False
        self._wallMoveStartTime = 10.0
        self._isActive = True

    def create(self, stage):
        self._defaultPrimPath, scene = demo.setup_physics_scene(self, stage, gravityMod=0.1, primPathAsString=False)
        scenePath = self._defaultPrimPath.AppendChild("physicsScene")
        physxScene = PhysxSchema.PhysxSceneAPI.Apply(scene.GetPrim())
        physxScene.GetEnableExternalForcesEveryIterationAttr().Set(True)

        numParticlesZ = 35

        self._stage = stage

        box_path = demo.get_demo_asset_path("FluidIsosurfaceGlassBox/fluid_box.usd")

        stage.DefinePrim(self._defaultPrimPath.AppendPath("Box")).GetReferences().AddReference(box_path)
        self._movingWall = UsdGeom.Xformable(self._stage.GetPrimAtPath(self._defaultPrimPath.AppendPath("Box/Xform/Cube_02")))
        # set to kinematic RB:
        rigidBodyAPI = UsdPhysics.RigidBodyAPI.Apply(self._movingWall.GetPrim())
        rigidBodyAPI.CreateKinematicEnabledAttr(True)

        # solver iterations
        self._solverPositionIterations = 4
        physxAPI = PhysxSchema.PhysxSceneAPI.Apply(scene.GetPrim())
        physxAPI.CreateSolverTypeAttr("TGS")

        # Particle System
        particleSystemPath = self._defaultPrimPath.AppendChild("particleSystem0")

        # particle params
        particleSpacing = 0.2
        restOffset = particleSpacing * 0.9
        fluidRestOffset = restOffset * 0.6
        particleContactOffset = restOffset + 0.001
        particle_system = particleUtils.add_physx_particle_system(
            stage=stage,
            particle_system_path=particleSystemPath,
            simulation_owner=scenePath,
            contact_offset=restOffset * 1.5 + 0.01,
            rest_offset=restOffset * 1.5,
            particle_contact_offset=particleContactOffset,
            solid_rest_offset=0.0,
            fluid_rest_offset=fluidRestOffset,
            solver_position_iterations=self._solverPositionIterations,
        )

        mtl_created = []
        omni.kit.commands.execute(
            "CreateAndBindMdlMaterialFromLibrary",
            mdl_name="OmniSurfacePresets.mdl",
            mtl_name="OmniSurface_DeepWater",
            mtl_created_list=mtl_created,
            select_new_prim=False,
        )
        pbd_particle_material_path = mtl_created[0]
        omni.kit.commands.execute(
            "BindMaterial", prim_path=particleSystemPath, material_path=pbd_particle_material_path
        )

        # Create a pbd particle material and set it on the particle system
        particleUtils.add_pbd_particle_material(
            stage,
            pbd_particle_material_path,
            cohesion=0.01,
            viscosity=0.0091,
            surface_tension=0.0074,
            friction=0.1,
        )
        physicsUtils.add_physics_material_to_prim(stage, particle_system.GetPrim(), pbd_particle_material_path)

        particle_system.CreateMaxVelocityAttr().Set(200)

        # add particle anisotropy
        anisotropyAPI = PhysxSchema.PhysxParticleAnisotropyAPI.Apply(particle_system.GetPrim())
        anisotropyAPI.CreateParticleAnisotropyEnabledAttr().Set(True)
        aniso_scale = 5.0
        anisotropyAPI.CreateScaleAttr().Set(aniso_scale)
        anisotropyAPI.CreateMinAttr().Set(1.0)
        anisotropyAPI.CreateMaxAttr().Set(2.0)

        # add particle smoothing
        smoothingAPI = PhysxSchema.PhysxParticleSmoothingAPI.Apply(particle_system.GetPrim())
        smoothingAPI.CreateParticleSmoothingEnabledAttr().Set(True)
        smoothingAPI.CreateStrengthAttr().Set(0.5)

        # apply isosurface params
        isosurfaceAPI = PhysxSchema.PhysxParticleIsosurfaceAPI.Apply(particle_system.GetPrim())
        isosurfaceAPI.CreateIsosurfaceEnabledAttr().Set(True)
        isosurfaceAPI.CreateMaxVerticesAttr().Set(1024 * 1024)
        isosurfaceAPI.CreateMaxTrianglesAttr().Set(2 * 1024 * 1024)
        isosurfaceAPI.CreateMaxSubgridsAttr().Set(1024 * 4)
        isosurfaceAPI.CreateGridSpacingAttr().Set(fluidRestOffset * 1.5)
        isosurfaceAPI.CreateSurfaceDistanceAttr().Set(fluidRestOffset * 1.6)
        isosurfaceAPI.CreateGridFilteringPassesAttr().Set("")
        isosurfaceAPI.CreateGridSmoothingRadiusAttr().Set(fluidRestOffset * 2)

        isosurfaceAPI.CreateNumMeshSmoothingPassesAttr().Set(1)

        primVarsApi = UsdGeom.PrimvarsAPI(particle_system)
        primVarsApi.CreatePrimvar("doNotCastShadows", Sdf.ValueTypeNames.Bool).Set(True)

        stage.SetInterpolationType(Usd.InterpolationTypeHeld)

        particlesPath = self._defaultPrimPath.AppendChild("particles")

        lower = Gf.Vec3f(-0.75, 10.5, 0.1)
        positions, velocities = particleUtils.create_particles_grid(
            lower, particleSpacing + 0.01, 110, 65, numParticlesZ
        )

        uniform_range = particleSpacing * 0.2

        for i in range(len(positions)):
            positions[i][0] += self._rng.uniform(-uniform_range, uniform_range)
            positions[i][1] += self._rng.uniform(-uniform_range, uniform_range)
            positions[i][2] += self._rng.uniform(-uniform_range, uniform_range)

        widths = [particleSpacing] * len(positions)
        particleUtils.add_physx_particleset_points(
            stage=stage,
            path=particlesPath,
            positions_list=Vt.Vec3fArray(positions),
            velocities_list=Vt.Vec3fArray(velocities),
            widths_list=widths,
            particle_system_path=particleSystemPath,
            self_collision=True,
            fluid=True,
            particle_group=0,
            particle_mass=0.001,
            density=0.0,
        )

        rootPath = str(self._defaultPrimPath)

        _ = demo.get_demo_room(self, stage, zoom=0.1, pathsToHide=[
            rootPath + "/Box/SphereLight",
            rootPath + "/Box/SphereLight_01",
            rootPath + "/Box/SphereLight_02",
            rootPath + "/Box/SphereLight_03",
            rootPath + "/Box/SphereLight_04",
            rootPath + "/Box/DomeLight",
            rootPath + "/Box/groundPlane/CollisionMesh",
            rootPath + "/Box/groundPlane/CollisionPlane"
        ])

        paths = [
            rootPath + "/Box/Xform/Cube",
            rootPath + "/Box/Xform/Cube_01",
            rootPath + "/Box/Xform/Cube_02",
            rootPath + "/Box/Xform/Cube_03"
        ]
        for path in paths:
            self.set_shadow(stage, path)

    def set_shadow(self, stage, path):
        prim = stage.GetPrimAtPath(path)
        prim.CreateAttribute("primvars:doNotCastShadows", Sdf.ValueTypeNames.Bool).Set(True)

    def on_timeline_event(self, e):
        if not self._isActive:
            return
        if e.type == int(omni.timeline.TimelineEventType.STOP):
            self._is_running = False
            self._time = 0
            self._rng = np.random.default_rng(self._rng_seed)  # reset rng to get repeatable box spawning
            self._movingWallDeltaY = 0.0
        elif e.type == int(omni.timeline.TimelineEventType.PLAY):
            self._is_running = True

    def on_physics_step(self, dt):
        if not self._isActive:
            return
        self._time += dt
        if self._time < self._wallMoveStartTime:
            return

        if self._movingWallDeltaY <= 0.0:
            self._motion_dir = 1
        elif self._movingWallDeltaY >= self._yMax:
            self._motion_dir = -1

        self._movingWallDeltaY += self._wall_speed * self._motion_dir * dt

        movingTransform = np.zeros((1, 7), dtype=np.float32)
        movingTransform[0][0] = self._wall_init_translation[0]
        movingTransform[0][1] = self._wall_init_translation[1] + self._movingWallDeltaY
        movingTransform[0][2] = self._wall_init_translation[2]
        movingTransform[0][6] = 1.0

        if self._wallTensorView:
            self._wallTensorView.set_kinematic_targets(movingTransform, self._wallTensorInd)

    def on_simulation_event(self, e):
        if not self._isActive:
            return
        if e.type == int(physx_settings_bindings.SimulationEvent.STOPPED):
            physicsUtils.set_or_add_translate_op(self._movingWall, translate=self._wall_init_translation)

    def on_tensor_start(self, tensorApi):
        if not self._isActive:
            return
        sim = tensorApi.create_simulation_view("numpy")
        self._wallTensorView = sim.create_rigid_body_view(str(self._defaultPrimPath) + "/Box/Xform/Cube_02")
        assert self._wallTensorView.count == 1
        self._wallTensorInd = np.arange(self._wallTensorView.count, dtype=np.int32)
        # prevent garbage collector from deleting the sim
        self.sim = sim

    def on_shutdown(self):
        self._isActive = False
        self._wallTensorView = None
        self._movingWall = None
        self.sim = None
        super().on_shutdown()
