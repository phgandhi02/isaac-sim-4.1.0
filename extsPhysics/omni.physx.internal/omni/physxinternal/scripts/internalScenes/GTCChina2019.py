import carb
import carb.input
import carb.settings
import omni.kit.app
import omni.kit.ui
import omni.appwindow
import omni.client
import omni.usd
import asyncio
import random
from pxr import UsdGeom, Sdf, Vt, Gf, UsdPhysics
from omni.physx.scripts import particleUtils
import omni.ui as ui
import omni.physxdemos as demo

eParticlePhaseGroupMask = int("0x000fffff", 0)
eParticlePhaseFlagsMask = int("0x00f00000", 0)
eParticlePhaseSelfCollide = 1 << 20
eParticlePhaseSelfCollideFilter = 1 << 21
eParticlePhaseFluid = 1 << 22
WINDOW_NAME = "GTC China 2019 PhysX Demo"


class GtcChinaDemo(demo.Base):
    title = "GTC China 2019 PhysX Sample"
    category = demo.Categories.INTERNAL_SAMPLES
    short_description = "Two way couping of PBD Particles, cloth, rigid bodies and FEM soft bodies"
    description = "This snippet shows a scene where particles interacting with rigid bodies, cloth and FEM soft bodies."

    kit_settings = {
        "/app/renderer/generateTBNFrameOnGPU": True
    }

    def on_startup(self):
        super().on_startup()
        print("startup")
        self._appwindow = omni.appwindow.get_default_app_window()
        self._usd_context = omni.usd.get_context()
        self._input = carb.input.acquire_input_interface()
        self._keyboard = self._appwindow.get_keyboard()
        self._sub_keyboard = self._input.subscribe_to_keyboard_events(self._keyboard, self._sub_keyboard_event)

        self._build_ui()

    def _build_ui(self):
        self._window = omni.kit.ui.Window(WINDOW_NAME, width=1280, height=720, add_to_menu=False)
        self.loadMapButton = omni.kit.ui.Button(f"Load Map")
        self.initEnvButton = omni.kit.ui.Button(f"Initialize Environment")
        self.addParticleButton = omni.kit.ui.Button(f"Add Particles")
        self.addRBButton = omni.kit.ui.Button(f"Add Rigidbody")
        self._window.layout.add_child(self.loadMapButton)
        self._window.layout.add_child(self.initEnvButton)
        self._window.layout.add_child(self.addParticleButton)
        self._window.layout.add_child(self.addRBButton)
        self.loadMapButton.set_clicked_fn(self._loadMap)
        self.initEnvButton.set_clicked_fn(self._on_init_env_button_clicked)
        self.addParticleButton.set_clicked_fn(self._on_add_particle_button_clicked)
        self.addRBButton.set_clicked_fn(self._on_add_rb_button_clicked)

        async def dock():
            await omni.kit.app.get_app().next_update_async()
            parent_handle = ui.Workspace.get_window("Content")
            my_handle = ui.Workspace.get_window(WINDOW_NAME)
            if parent_handle and my_handle:
                my_handle.dock_in(parent_handle, ui.DockPosition.SAME)

        asyncio.ensure_future(dock())

        self._window.show()

    def _sub_keyboard_event(self, event, *args, **kwargs):
        if event.keyboard == self._keyboard:
            if event.type == carb.input.KeyboardEventType.KEY_PRESS:
                if event.input == carb.input.KeyboardInput.L:
                    asyncio.ensure_future(self.task())
            return True

    # Load Map routines
    async def task(self):
        await omni.usd.get_context().open_stage_async(
            "omniverse://omniverse@ov-content/Projects/GTC_China_2019_PhysX/gtc_china_physx_demo_new_schema.usd"
        )

        stage = self._usd_context.get_stage()
        scene_prim = stage.GetPrimAtPath("/Stage/physicsScene")
        set_physics_scene_asyncsimrender(scene_prim)

    def _loadMap(self, widget):
        asyncio.ensure_future(self.task())

    def _on_init_env_button_clicked(self, widget):
        self._initialize_env()

    def _initialize_env(self):
        # # Set the order of the PhysX and Mesh-Mesh Skinning Node
        # stage_update = omni.kit.editor.get_stage_update_interface()
        # nodes = stage_update.get_stage_update_nodes()
        # physx_order = 0
        # for node in nodes:
        # if node["name"] == "PhysX":
        # physx_order = node["order"]
        # for node in nodes:
        # if node["name"] == "Mesh-Mesh Constraint":
        # stage_update.set_stage_update_node_order(node["index"], physx_order + 1)
        pass

    def _on_add_particle_button_clicked(self, widget):
        self._particle_param_popup = omni.kit.ui.Popup(
            "Create Particle Bomb Parameters", modal=True, width=400, height=150
        )
        self._particle_bomb_radius_field = omni.kit.ui.DragDouble(
            "Particle Bomb Radius", value=50.0, min=1.0, max=100.0, drag_speed=1.0
        )
        self._particle_radius_field = omni.kit.ui.DragDouble(
            "Particle Radius", value=3.0, min=0.1, max=10, drag_speed=0.1
        )
        self._max_num_particle_field = omni.kit.ui.DragInt(
            "Particle Number", value=10000, min=1000, max=50000, drag_speed=1000
        )
        self._particle_param_popup.layout.add_child(self._particle_bomb_radius_field)
        self._particle_param_popup.layout.add_child(self._particle_radius_field)
        self._particle_param_popup.layout.add_child(self._max_num_particle_field)
        self._ok_button = omni.kit.ui.Button(f"OK")
        self._cancel_button = omni.kit.ui.Button(f"Cancel")
        row = omni.kit.ui.RowLayout()
        row.add_child(self._ok_button)
        row.add_child(self._cancel_button)
        self._particle_param_popup.layout.add_child(row)

        def _on_ok_clicked(widget):
            self._create_particle_bomb(
                self._particle_bomb_radius_field.value,
                self._particle_radius_field.value,
                self._max_num_particle_field.value,
            )
            self._particle_param_popup = None

        self._ok_button.set_clicked_fn(_on_ok_clicked)

        def _on_cancel_clicked(widget):
            self._particle_param_popup = None

        self._cancel_button.set_clicked_fn(_on_cancel_clicked)

    def create_particle_phase(self, group, flags):
        return (group & eParticlePhaseGroupMask) | (flags & eParticlePhaseFlagsMask)

    def create_particle_sphere(
        self, center, particleSpacing, radius, max_num_particles, positions, velocities, protoIndices, num_proto
    ):
        particle_number = int(2 * radius / particleSpacing)
        num_particles = 0
        particle_radius = particleSpacing / 2.1
        for i in range(particle_number):
            for j in range(particle_number):
                for k in range(particle_number):
                    x_off = i * particleSpacing - radius
                    y_off = j * particleSpacing - radius
                    z_off = k * particleSpacing - radius
                    if x_off * x_off + y_off * y_off + z_off * z_off <= radius * radius:
                        noise = random.uniform(-0.1, 0.1)
                        x = center[0] - radius + i * particleSpacing + noise * particle_radius
                        y = center[1] - radius + j * particleSpacing
                        z = center[2] - radius + k * particleSpacing + noise * particle_radius
                        if num_particles < max_num_particles:
                            positions.append(Gf.Vec3f(x, y, z))
                            velocities.append(Gf.Vec3f(0.0, 0.0, 0.0))
                            protoIndices.append(num_particles % num_proto)
                            num_particles = num_particles + 1

    def _create_particle_bomb(self, bomb_radius, particle_radius, max_num_particles):
        if particle_radius == 0 or max_num_particles == 0:
            return
        worldPos = Gf.Vec3f(0.0, 0.0, 0.0)
        stage = self._usd_context.get_stage()

        pos_iter = 5
        vel_iter = 5
        if bomb_radius == -1.0:
            # Use default value
            worldPos = Gf.Vec3f(43.679, 205.425, -179.636)
            bomb_radius = 60.0
            particle_radius = 3.0
            max_num_particles = 15000
            pos_iter = 10

        # Particle System
        particleSystemPath = Sdf.Path("/particleSystem0")
        # contactOffset is 0.001, solverPositionIteration is 16, solverVelocityIteration is 16
        contactOffset = 0.1
        # meters_per_unit = UsdGeom.GetStageMetersPerUnit(stage)
        meters_per_unit = 1
        particleUtils.add_physx_particle_system(
            stage=stage,
            particle_system_path=particleSystemPath,
            contact_offset=(contactOffset + particle_radius) / meters_per_unit,
            rest_offset=particle_radius / meters_per_unit,
            particle_contact_offset=(contactOffset + particle_radius) / meters_per_unit,
            solid_rest_offset=particle_radius / meters_per_unit,
            fluid_rest_offset=0.6 * particle_radius,
            solver_position_iterations=pos_iter,
            simulation_owner=Sdf.Path("/Stage/physicsScene"),
        )

        # Simple Particle
        particleInstanceStr = "/particlesInstance0"
        particleInstancePath = Sdf.Path(particleInstanceStr)

        positions_list = []
        velocities_list = []
        protoIndices_list = []

        particleSpacing = 2.1 * particle_radius  # introduce 10% spacing room for random noise
        emitter_sphere_radius = bomb_radius
        num_protos = 3
        self.create_particle_sphere(
            worldPos,
            particleSpacing,
            emitter_sphere_radius,
            max_num_particles,
            positions_list,
            velocities_list,
            protoIndices_list,
            num_protos,
        )

        protoIndices = Vt.IntArray(protoIndices_list)
        positions = Vt.Vec3fArray(positions_list)
        velocities = Vt.Vec3fArray(velocities_list)
        particleUtils.add_physx_particleset_pointinstancer(
            stage,
            particleInstancePath,
            positions,
            velocities,
            particleSystemPath,
            self_collision=True,
            fluid=False,
            particle_group=0,
            particle_mass=1.0,
            density=0.0,
            num_prototypes=num_protos,
            prototype_indices=protoIndices,
        )
        carb.log_info("%i number of PhysX particles are create!" % len(protoIndices_list))

        # Set color & scale for particlePrototype0 particlePrototype1 & particlePrototype2
        color = None
        for i in range(num_protos):
            if i == 0:
                color = Vt.Vec3fArray([Gf.Vec3f(58 / 255.0, 23 / 255.0, 114 / 255.0)])
            elif i == 1:
                color = Vt.Vec3fArray([Gf.Vec3f(83 / 255.0, 152 / 255.0, 190 / 255.0)])
            else:
                color = Vt.Vec3fArray([Gf.Vec3f(242 / 255.0, 205 / 255.0, 93 / 255.0)])
            colorPathStr = particleInstanceStr + "/particlePrototype" + str(i)
            gprim = UsdGeom.Sphere.Define(stage, Sdf.Path(colorPathStr))
            gprim.CreateDisplayColorAttr(color)

            gprim.GetRadiusAttr().Set(particle_radius)

        self._particle_radius = 0
        self._num_particles = 0

    def _on_add_rb_button_clicked(self, widget):
        self._rb_param_popup = omni.kit.ui.Popup("Create RB Sphere Bomb Parameters", modal=True, width=400, height=150)
        self._rb_bomb_radius_field = omni.kit.ui.DragDouble(
            "RB Bomb Radius", value=50, min=1.0, max=100.0, drag_speed=1.0
        )
        self._rb_radius_field = omni.kit.ui.DragDouble("Sphere Radius", value=2, min=0.1, max=10, drag_speed=0.1)
        self._max_num_rb_field = omni.kit.ui.DragInt("Sphere Number", value=10000, min=1000, max=50000, drag_speed=1000)
        self._rb_param_popup.layout.add_child(self._rb_bomb_radius_field)
        self._rb_param_popup.layout.add_child(self._rb_radius_field)
        self._rb_param_popup.layout.add_child(self._max_num_rb_field)
        self._rb_ok_button = omni.kit.ui.Button(f"OK")
        self._rb_cancel_button = omni.kit.ui.Button(f"Cancel")
        row = omni.kit.ui.RowLayout()
        row.add_child(self._rb_ok_button)
        row.add_child(self._rb_cancel_button)
        self._rb_param_popup.layout.add_child(row)

        def _on_rb_ok_clicked(widget):
            stage = self._usd_context.get_stage()
            defaultPrim = stage.GetDefaultPrim()
            if not defaultPrim:
                self._rb_param_popup = None
            self._create_rb_sphere_bomb(
                defaultPrim.GetPath().pathString + "/rbPointInstancer",
                self._rb_bomb_radius_field.value,
                self._rb_radius_field.value,
                self._max_num_rb_field.value,
            )
            self._rb_param_popup = None

        self._rb_ok_button.set_clicked_fn(_on_rb_ok_clicked)

        def _on_rb_cancel_clicked(widget):
            self._rb_param_popup = None

        self._rb_cancel_button.set_clicked_fn(_on_rb_cancel_clicked)

    def _create_rb_sphere_bomb(self, point_instancer_path, bomb_radius, sphere_radius, max_num_sphere):
        if sphere_radius == 0 or max_num_sphere == 0:
            return
        worldPos = Gf.Vec3f(0.0, bomb_radius, 0.0)
        stage = self._usd_context.get_stage()

        # Add 3 rigid body instances
        protoPaths_list = []
        color = None
        for i in range(3):
            if i == 0:
                color = Vt.Vec3fArray([Gf.Vec3f(58 / 255.0, 23 / 255.0, 114 / 255.0)])
            elif i == 1:
                color = Vt.Vec3fArray([Gf.Vec3f(83 / 255.0, 152 / 255.0, 190 / 255.0)])
            else:
                color = Vt.Vec3fArray([Gf.Vec3f(242 / 255.0, 205 / 255.0, 93 / 255.0)])
            proto_path = point_instancer_path + "/rbPrototype" + str(i)
            protoPaths_list.append(proto_path)
            self._add_rigidbody_sphere_for_instancing(stage, proto_path, sphere_radius, color, 1.0)

        # Prepare PointInstancer data
        positions_list = []
        orientations_list = []
        linearVelocities_list = []
        angularVelocities_list = []
        protoIndices_list = []
        particleSpacing = 2.0 * sphere_radius
        emitter_sphere_radius = bomb_radius
        self._create_rigidbody_sphere(
            worldPos,
            particleSpacing,
            emitter_sphere_radius,
            max_num_sphere,
            positions_list,
            orientations_list,
            linearVelocities_list,
            angularVelocities_list,
            protoIndices_list,
            3,
        )

        # Create a PointInstancer
        pointInstancer = UsdGeom.PointInstancer.Define(stage, Sdf.Path(point_instancer_path))

        # Fill the prototype list attribute
        protoTypeRel = pointInstancer.GetPrototypesRel()
        if protoTypeRel:
            for protoPath in protoPaths_list:
                protoTypeRel.AddTarget(protoPath)

        # Fill the point instancer specific attributes
        pointInstancer.GetProtoIndicesAttr().Set(protoIndices_list)
        pointInstancer.GetPositionsAttr().Set(positions_list)
        pointInstancer.GetOrientationsAttr().Set(orientations_list)
        pointInstancer.GetVelocitiesAttr().Set(linearVelocities_list)
        pointInstancer.GetAngularVelocitiesAttr().Set(angularVelocities_list)

    def _add_rigidbody_sphere_for_instancing(self, stage, rb_path, sphere_radius, color, density):
        # Create a Sphere Prim
        spherePrim = UsdGeom.Sphere.Define(stage, Sdf.Path(rb_path))
        spherePrim.CreateDisplayColorAttr(color)
        # Apply Physics API onto the prim
        physicsAPI = UsdPhysics.RigidBodyAPI.Apply(spherePrim.GetPrim())
        # Add Density attribute
        massAPI = UsdPhysics.MassAPI.Apply(spherePrim.GetPrim())
        massAPI.CreateDensityAttr().Set(density)
        # set the scale
        scale = Gf.Vec3f(sphere_radius, sphere_radius, sphere_radius)
        spherePrim.AddScaleOp().Set(scale)
        # Apply CollisionAPI to the sphere prim
        UsdPhysics.CollisionAPI.Apply(spherePrim.GetPrim())

    def _create_rigidbody_sphere(
        self,
        center,
        sphereSpacing,
        radius,
        max_num_sphere,
        positions,
        orientations,
        linearVelocities,
        angularVelocities,
        protoIndices,
        num_proto,
    ):
        sphere_number = int(2 * radius / sphereSpacing)
        num_sphere = 0
        for i in range(sphere_number):
            for j in range(sphere_number):
                for k in range(sphere_number):
                    x_off = i * sphereSpacing - radius
                    y_off = j * sphereSpacing - radius
                    z_off = k * sphereSpacing - radius
                    if x_off * x_off + y_off * y_off + z_off * z_off <= radius * radius:
                        x = center[0] - radius + i * sphereSpacing
                        y = center[1] - radius + j * sphereSpacing
                        z = center[2] - radius + k * sphereSpacing
                        if num_sphere < max_num_sphere:
                            positions.append(Gf.Vec3f(x, y, z))
                            linearVelocities.append(Gf.Vec3f(0.0, 0.0, 0.0))
                            angularVelocities.append(Gf.Vec3f(0.0, 0.0, 0.0))
                            orientations.append(Gf.Quath(1.0, 0.0, 0.0, 0.0))
                            protoIndices.append(num_sphere % num_proto)
                            num_sphere = num_sphere + 1

    def on_shutdown(self):
        self._input.unsubscribe_to_keyboard_events(self._keyboard, self._sub_keyboard)

        self._window.hide()
        self._window = None
