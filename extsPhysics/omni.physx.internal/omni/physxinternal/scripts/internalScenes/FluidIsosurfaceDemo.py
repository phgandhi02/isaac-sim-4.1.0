import omni.kit.commands
from omni.physx.scripts import physicsUtils, particleUtils, utils
from pxr import Usd, UsdLux, UsdGeom, Sdf, Gf, Vt, UsdPhysics, PhysxSchema
import omni.physxdemos as demo
import omni.usd
import omni.physx.bindings._physx as physx_settings_bindings


def set_initial_camera(stage, position, target):
    customLayerData = {
        "cameraSettings": {
            "Perspective": {"position": position, "radius": 500, "target": target},
            "boundCamera": "/OmniverseKit_Persp",
        }
    }
    stage.GetRootLayer().customLayerData = customLayerData


class FluidIsosurfaceDemo(demo.Base):
    title = "Fluid Isosurface"
    category = demo.Categories.INTERNAL_SAMPLES
    short_description = "Fluid isosurface with Flow's rendering settings"
    description = "This snippet demonstrates the method to render the fluid isosurface with Flow's rendering settings."

    kit_settings = {
        "persistent/app/viewport/displayOptions": demo.get_viewport_minimal_display_options_int(),
        physx_settings_bindings.SETTING_UPDATE_TO_USD: True,
        physx_settings_bindings.SETTING_MIN_FRAME_RATE: 30,
        physx_settings_bindings.SETTING_UPDATE_VELOCITIES_TO_USD: False,
        physx_settings_bindings.SETTING_UPDATE_PARTICLES_TO_USD: False,
        "rtx-defaults/post/aa/op": 3,
        "rtx-defaults/translucency/maxRefractionBounces": 6,
        "rtx-defaults/post/dlss/execMode": 1,
        physx_settings_bindings.SETTING_MOUSE_PICKING_FORCE: 4.0,
    }

    def create(self, stage):
        UsdGeom.SetStageMetersPerUnit(stage, 0.01)

        # set up axis to z
        UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
        UsdGeom.SetStageMetersPerUnit(stage, 1)

        set_initial_camera(stage, Gf.Vec3d(26.197044789165616, -6.884089058483182, 21.84811751302347), Gf.Vec3d(-3.4062655621816162, 2.687579594100079, 0.2657886171608297))

        # light
        sphereLight = UsdLux.SphereLight.Define(stage, Sdf.Path("/SphereLight"))
        sphereLight.CreateRadiusAttr(150)
        sphereLight.CreateIntensityAttr(30000)
        sphereLight.AddTranslateOp().Set(Gf.Vec3f(650.0, 0.0, 1150.0))

        domeLight = UsdLux.DomeLight.Define(stage, Sdf.Path("/DomeLight"))
        domeLight.CreateIntensityAttr(500)

        # Physics scene
        scenePath = Sdf.Path("/physicsScene")
        scene = UsdPhysics.Scene.Define(stage, scenePath)
        scene.CreateGravityDirectionAttr().Set(Gf.Vec3f(0.0, 0.0, -1.0))
        scene.CreateGravityMagnitudeAttr().Set(1000)
        utils.set_physics_scene_asyncsimrender(scene.GetPrim())

        radius = 0.2

        worldPos = Gf.Vec3f(0.0, 0.0, 0.0)

        # Particle System
        particleSystemPath = Sdf.Path("/particleSystem0")

        restOffset = radius / 4
        contactOffset = restOffset + 0.001
        particleContactOffset = radius + 0.001
        solidRestOffset = 0
        fluidRestOffset = particleContactOffset * 0.6
        particle_system = particleUtils.add_physx_particle_system(
            stage=stage,
            particle_system_path=particleSystemPath,
            contact_offset=contactOffset,
            rest_offset=restOffset,
            particle_contact_offset=particleContactOffset,
            solid_rest_offset=solidRestOffset,
            fluid_rest_offset=fluidRestOffset,
            solver_position_iterations=16,
            simulation_owner=scenePath,
        )

        particleSystem = stage.GetPrimAtPath(particleSystemPath)
        particle_system.CreateMaxVelocityAttr().Set(restOffset * 1000.0)

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
        particleUtils.add_pbd_particle_material(stage, mtl_path, cohesion=10, friction=0.2, viscosity=1.0)
        physicsUtils.add_physics_material_to_prim(stage, particleSystem, mtl_path)

        # apply isosurface params
        isosurfaceAPI = PhysxSchema.PhysxParticleIsosurfaceAPI.Apply(particleSystem)
        isosurfaceAPI.CreateIsosurfaceEnabledAttr().Set(True)
        isosurfaceAPI.CreateMaxVerticesAttr().Set(1024 * 1024)
        isosurfaceAPI.CreateMaxTrianglesAttr().Set(2 * 1024 * 1024)
        isosurfaceAPI.CreateMaxSubgridsAttr().Set(1024 * 4)
        isosurfaceAPI.CreateGridSpacingAttr().Set(fluidRestOffset * 1.2)
        isosurfaceAPI.CreateSurfaceDistanceAttr().Set(fluidRestOffset * 1.6)
        isosurfaceAPI.CreateGridFilteringPassesAttr().Set("S")
        isosurfaceAPI.CreateGridSmoothingRadiusAttr().Set(fluidRestOffset * 2.0)
        isosurfaceAPI.CreateNumMeshSmoothingPassesAttr().Set(1)
        isosurfaceAPI.CreateNumMeshNormalSmoothingPassesAttr().Set(1)

        primVarsApi = UsdGeom.PrimvarsAPI(particleSystem)
        primVarsApi.CreatePrimvar("doNotCastShadows", Sdf.ValueTypeNames.Bool).Set(True)

        stage.SetInterpolationType(Usd.InterpolationTypeHeld)

        # Simple Particle
        particleInstanceStr = "/particles"
        particleInstancePath = Sdf.Path(particleInstanceStr)

        lower = Gf.Vec3f(0.0, 0.0, 1.0) + worldPos
        particleSpacing = 2.0 * fluidRestOffset
        positions, velocities = particleUtils.create_particles_grid(lower, particleSpacing, 20, 20, 10)

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
        gprim.GetRadiusAttr().Set(particleSpacing)

        numBoxes = 10
        for i in range(numBoxes):
            # Box
            boxActorPath = "/boxActor" + str(i + numBoxes)

            size = Gf.Vec3f(0.8)
            position = Gf.Vec3f(0.0, 0.0, 10.0 + (i * 2.5))
            orientation = Gf.Quatf(1.0, 0.0, 0.0, 0.0)
            color = Gf.Vec3f(200.0 / 255.0, 165.0 / 165.0 / 255.0, 1.0)
            linVelocity = Gf.Vec3f(2.0, 1.0, 2.0)
            angularVelocity = Gf.Vec3f(1.0, 0.0, 0.0)

            physicsUtils.add_rigid_box(
                stage, boxActorPath, size, position, orientation, color, 1.0, linVelocity, angularVelocity
            )

        # Plane
        physicsUtils.add_ground_plane(stage, "/groundPlane", "Z", 100.0, Gf.Vec3f(0.0), Gf.Vec3f(0.5))

        # Wall
        boxActorPath = "/topWallActor"
        size = Gf.Vec3f(10.0, 2.0, 10.0)
        position = Gf.Vec3f(0.0, -10.0, 10.0)
        orientation = Gf.Quatf(1.0, 0.0, 0.0, 0.0)
        color = Gf.Vec3f(0.5)
        linVelocity = Gf.Vec3f(0.0)
        angularVelocity = Gf.Vec3f(0.0, 0.0, 0.0)
        physicsUtils.add_rigid_box(
            stage, boxActorPath, size, position, orientation, color, 1.0, linVelocity, angularVelocity
        )

        boxActorPath = "/bottomWallActor"
        position = Gf.Vec3f(0.0, 10.0, 10.0)
        physicsUtils.add_rigid_box(
            stage, boxActorPath, size, position, orientation, color, 1.0, linVelocity, angularVelocity
        )

    def on_shutdown(self):
        self._timeline_subscription = None
