import math
from omni.physx.scripts import physicsUtils, particleUtils, utils
from pxr import Usd, UsdLux, UsdGeom, Sdf, Gf, Vt, UsdPhysics, PhysxSchema
import omni.physxdemos as demo
import omni.usd
import omni.physx.bindings._physx as physx_settings_bindings
import os


def rotate_around_axis(x, y, z, angle):
    s = math.sin(0.5 * angle)
    return Gf.Quatf(math.cos(0.5 * angle), s * x, s * y, s * z)


def get_quat_from_extrinsic_xyz_rotation(angleXrad: float = 0.0, angleYrad: float = 0.0, angleZrad: float = 0.0):
    # angles are in radians
    rotX = rotate_around_axis(1, 0, 0, angleXrad)
    rotY = rotate_around_axis(0, 1, 0, angleYrad)
    rotZ = rotate_around_axis(0, 0, 1, angleZrad)
    return rotZ * rotY * rotX


def set_initial_camera(stage, position, target):
    customLayerData = {
        "cameraSettings": {
            "Perspective": {"position": position, "radius": 500, "target": target},
            "boundCamera": "/OmniverseKit_Persp",
        }
    }
    stage.GetRootLayer().customLayerData = customLayerData


class FluidIsosurfaceCupDemo(demo.Base):
    title = "Fluid Isosurface Mug"
    category = demo.Categories.INTERNAL_SAMPLES
    short_description = "A mug rendered as fluid isosurface"
    description = "This snippet sets up a mug scene rendered with fluid Isosurface method."

    kit_settings = {
        "persistent/app/viewport/displayOptions": demo.get_viewport_minimal_display_options_int(),
        physx_settings_bindings.SETTING_UPDATE_TO_USD: True,
        physx_settings_bindings.SETTING_MIN_FRAME_RATE: 120,
        physx_settings_bindings.SETTING_UPDATE_VELOCITIES_TO_USD: False,
        physx_settings_bindings.SETTING_UPDATE_PARTICLES_TO_USD: False,
        "rtx-defaults/post/aa/op": 3,
        "rtx-defaults/translucency/maxRefractionBounces": 12,
        "rtx-defaults/post/dlss/execMode": 1,
        physx_settings_bindings.SETTING_MOUSE_PICKING_FORCE: 4.0,
    }

    def create(self, stage):
        UsdGeom.SetStageMetersPerUnit(stage, 0.01)

        # set up axis to z
        UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
        UsdGeom.SetStageMetersPerUnit(stage, 1)

        set_initial_camera(stage, Gf.Vec3d(21.37050879832708, 21.370508798326888, 24.471014229699623), Gf.Vec3d(-2.5634165423481576, -2.5634165423485555, 5.782573221352223))

        # mug
        default_prim_path = stage.GetDefaultPrim().GetPath()
        dataFolder = os.path.normpath(os.path.join(os.path.dirname(os.path.abspath(__file__)), "../../../.."))
        mugPath = os.path.normpath(dataFolder + "/data/usd/assets/Mug.usda")
        stage.DefinePrim(default_prim_path.AppendPath("Mug")).GetReferences().AddReference(mugPath)
        mug = stage.GetPrimAtPath(default_prim_path.AppendPath("Mug/geom"))

        prim = mug.GetPrim()
        utils.setRigidBody(mug.GetPrim(), approximationShape="convexDecomposition", kinematic=False)
        # assert prim.CreateAttribute("physxMeshCollision:minThickness", Sdf.ValueTypeNames.Float).Set(0.001)
        # self._setup_physics_material(mug.GetPath())
        assert prim.CreateAttribute("physxCollision:contactOffset", Sdf.ValueTypeNames.Float).Set(0.5)
        assert prim.CreateAttribute("physxMeshCollision:maxConvexHulls", Sdf.ValueTypeNames.Float).Set(32)
        assert prim.CreateAttribute("mass", Sdf.ValueTypeNames.Float).Set(0.1)
        mug = UsdGeom.Xformable(stage.GetPrimAtPath(default_prim_path.AppendPath("Mug")))
        mug.AddTranslateOp().Set(Gf.Vec3f(0, 0, 0))
        mug.AddOrientOp().Set(get_quat_from_extrinsic_xyz_rotation(angleXrad=0.5 * math.pi))

        # light
        sphereLight = UsdLux.SphereLight.Define(stage, Sdf.Path("/SphereLight"))
        sphereLight.CreateRadiusAttr(150)
        sphereLight.CreateIntensityAttr(30000)
        sphereLight.AddTranslateOp().Set(Gf.Vec3f(650.0, 0.0, 1150.0))

        domeLight = UsdLux.DomeLight.Define(stage, default_prim_path.AppendChild("DomeLight"))
        domeLight.CreateIntensityAttr(500)

        # Physics scene
        scenePath = Sdf.Path("/physicsScene")
        scene = UsdPhysics.Scene.Define(stage, scenePath)
        scene.CreateGravityDirectionAttr().Set(Gf.Vec3f(0.0, 0.0, -1.0))
        scene.CreateGravityMagnitudeAttr().Set(400.0)
        utils.set_physics_scene_asyncsimrender(scene.GetPrim())
        physxAPI = PhysxSchema.PhysxSceneAPI.Apply(scene.GetPrim())
        physxAPI = PhysxSchema.PhysxSceneAPI.Apply(scene.GetPrim())
        physxAPI.CreateSolverTypeAttr("TGS")
        physxAPI.CreateTimeStepsPerSecondAttr().Set(120.0)

        # Particle System
        particleSystemPath = Sdf.Path("/particleSystem0")

        fluidRestOffset = 0.22
        solidRestOffset = 0
        particleContactOffset = fluidRestOffset * 2.0

        restOffset = fluidRestOffset * 0.7
        contactOffset = restOffset + 0.1

        _ = particleUtils.add_physx_particle_system(
            stage=stage,
            particle_system_path=particleSystemPath,
            contact_offset=contactOffset,
            rest_offset=restOffset,
            particle_contact_offset=particleContactOffset,
            solid_rest_offset=solidRestOffset,
            fluid_rest_offset=fluidRestOffset,
            solver_position_iterations=16,
            max_neighborhood=96,
            max_velocity=restOffset * 500.0,
            simulation_owner=scenePath
        )

        particleSystem = stage.GetPrimAtPath(particleSystemPath)

        # Render material
        mtl_created = []
        omni.kit.commands.execute(
            "CreateAndBindMdlMaterialFromLibrary",
            mdl_name="OmniSurfacePresets.mdl",
            mtl_name="OmniSurface_Honey",
            mtl_created_list=mtl_created,
            select_new_prim=False,
        )
        mtl_path = mtl_created[0]
        omni.kit.commands.execute("BindMaterial", prim_path=particleSystemPath, material_path=mtl_path)

        # Create a pbd particle material and set it on the particle system
        particleUtils.add_pbd_particle_material(stage, mtl_path, cohesion=0.3, friction=0.15, viscosity=20.0, surface_tension=0.074, cfl_coefficient=1.0)
        physicsUtils.add_physics_material_to_prim(stage, particleSystem, mtl_path)

        # add particle anisotropy
        anisotropyAPI = PhysxSchema.PhysxParticleAnisotropyAPI.Apply(particleSystem)
        anisotropyAPI.CreateParticleAnisotropyEnabledAttr().Set(True)
        aniso_scale = 2.5
        anisotropyAPI.CreateScaleAttr().Set(aniso_scale)
        anisotropyAPI.CreateMinAttr().Set(0.3 * aniso_scale)
        anisotropyAPI.CreateMaxAttr().Set(1.5 * aniso_scale)

        # add particle smoothing
        smoothingAPI = PhysxSchema.PhysxParticleSmoothingAPI.Apply(particleSystem)
        smoothingAPI.CreateParticleSmoothingEnabledAttr().Set(True)
        smoothingAPI.CreateStrengthAttr().Set(0.5)

        # apply isosurface params
        isosurfaceAPI = PhysxSchema.PhysxParticleIsosurfaceAPI.Apply(particleSystem)
        isosurfaceAPI.CreateIsosurfaceEnabledAttr().Set(True)
        isosurfaceAPI.CreateMaxVerticesAttr().Set(1024 * 1024)
        isosurfaceAPI.CreateMaxTrianglesAttr().Set(2 * 1024 * 1024)
        isosurfaceAPI.CreateMaxSubgridsAttr().Set(1024 * 4)
        isosurfaceAPI.CreateGridSpacingAttr().Set(fluidRestOffset * 0.9)
        isosurfaceAPI.CreateSurfaceDistanceAttr().Set(fluidRestOffset * 0.95)
        isosurfaceAPI.CreateGridFilteringPassesAttr().Set("GS")
        isosurfaceAPI.CreateGridSmoothingRadiusAttr().Set(fluidRestOffset * 1.0)
        isosurfaceAPI.CreateNumMeshSmoothingPassesAttr().Set(4)
        isosurfaceAPI.CreateNumMeshNormalSmoothingPassesAttr().Set(4)

        primVarsApi = UsdGeom.PrimvarsAPI(particleSystem)
        primVarsApi.CreatePrimvar("doNotCastShadows", Sdf.ValueTypeNames.Bool).Set(True)

        stage.SetInterpolationType(Usd.InterpolationTypeHeld)

        # Simple Particle
        particleInstanceStr = "/particles"
        particleInstancePath = Sdf.Path(particleInstanceStr)

        numX = 9
        numY = 9
        # numZ = 25

        particleSpacing = 2.0 * fluidRestOffset
        lower = Gf.Vec3f(-numX * 0.5 * particleSpacing, -numY * 0.5 * particleSpacing, 1.0)

        positions, velocities = particleUtils.create_particles_grid(lower, particleSpacing, 10, 10, 30)

        particleUtils.add_physx_particleset_pointinstancer(
            stage,
            particleInstancePath,
            Vt.Vec3fArray(positions),
            Vt.Vec3fArray(velocities),
            particleSystemPath,
            self_collision=True,
            fluid=True,
            particle_group=0,
            particle_mass=0.001,
            density=0.0
        )

        # Set color
        color_rgb = [0.0, 0.0, 0.0]
        color_rgb[0] = 1.0
        color = Vt.Vec3fArray([Gf.Vec3f(color_rgb[0], color_rgb[1], color_rgb[2])])
        colorPathStr = particleInstanceStr + "/particlePrototype0"
        gprim = UsdGeom.Sphere.Define(stage, Sdf.Path(colorPathStr))
        gprim.CreateDisplayColorAttr(color)
        gprim.GetRadiusAttr().Set(fluidRestOffset)

        # Plane
        physicsUtils.add_ground_plane(stage, "/groundPlane", "Z", 100.0, Gf.Vec3f(0.0), Gf.Vec3f(0.5))

    def on_shutdown(self):
        self._timeline_subscription = None
