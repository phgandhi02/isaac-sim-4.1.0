import os
import carb
import math
from pxr import UsdGeom, Gf, UsdPhysics, PhysxSchema, Sdf, UsdLux
import omni.kit.commands
import omni.physxdemos as demo
from omni.physx.scripts import physicsUtils
from omni.physx.bindings._physx import SETTING_MIN_FRAME_RATE


class SDFNutsDemo(demo.Base):
    title = "SDF Nuts Vibration"
    category = demo.Categories.INTERNAL_SAMPLES
    short_description = "Snippet demonstrating SDF Nuts Vibrating on a Table."
    description = (
        "Vibratory table moving nuts."
    )

    kit_settings = {
        SETTING_MIN_FRAME_RATE: 60,
        "persistent/app/viewport/displayOptions": demo.get_viewport_minimal_display_options_int(),
    }

    params = {"Enable_Motion": demo.CheckboxParam(True)}

    def create(self, stage, Enable_Motion):
        self._stage = stage
        self._default_prim_path = stage.GetDefaultPrim().GetPath()
        self._gravity_magnitude = 9.81  # [m/s^2]
        self._sim_steps_per_second = 1000
        self._sim_dt = 1.0 / self._sim_steps_per_second

        # table dims and motion
        self._table_color = Gf.Vec3f(165.0 / 255.0, 21.0 / 255.0, 21.0 / 255.0)
        self._table_position = Gf.Vec3f(0.0, 0.1, 0.0)  # [m]
        self._motion_amplitudes = Gf.Vec3f(-0.0003, 0.0002, 0.0000)  # [m]
        self._motion_phases = Gf.Vec3f(0.0, 0.0, 0.0)  # [m]
        self._motion_frequency = 60.0  # [Hz]

        # nuts and distribution
        self._num_nuts = 5
        self._nut_radius = 0.05
        self._nut_pos = Gf.Vec3f(self._table_position)
        self._nut_pos[0] += 0.05 + self._nut_radius
        self._nut_pos[1] += 0.02
        self._nut_contact_offset = 0.02
        self._nut_solver_pos_iterations = 16

        self._setup_stage()
        self._setup_table()
        self._setup_nuts()

        if Enable_Motion:
            self._time = 0.0  # motion variable tracking sim time
            self._setup_callbacks()

    def _setup_stage(self):
        # setup y-up, SI-unit stage
        UsdGeom.SetStageUpAxis(self._stage, UsdGeom.Tokens.y)
        UsdGeom.SetStageMetersPerUnit(self._stage, 1.0)
        physicsScenePath = self._default_prim_path.AppendChild("physicsScene")
        scene = UsdPhysics.Scene.Define(self._stage, physicsScenePath)
        scene.CreateGravityDirectionAttr().Set(Gf.Vec3f(0, -1, 0))
        scene.CreateGravityMagnitudeAttr().Set(self._gravity_magnitude)
        physxSceneAPI = PhysxSchema.PhysxSceneAPI.Apply(scene.GetPrim())
        physxSceneAPI.GetTimeStepsPerSecondAttr().Set(self._sim_steps_per_second)

        physicsUtils.add_ground_plane(
            self._stage,
            "ground",
            "Y",
            10.00,
            Gf.Vec3f(0),
            Gf.Vec3f(0.2, 0.25, 0.25),
        )

        # setup light:
        sphereLight = UsdLux.SphereLight.Define(self._stage, self._default_prim_path.AppendChild("SphereLight"))
        sphereLight.CreateEnableColorTemperatureAttr().Set(True)
        sphereLight.CreateColorTemperatureAttr().Set(5500)
        sphereLight.CreateRadiusAttr().Set(1)
        sphereLight.CreateIntensityAttr().Set(30000)
        sphereLight.AddTranslateOp().Set(Gf.Vec3f(2.25, 3.36, 4.49))
        # set cam:
        customLayerData = {
            "cameraSettings": {
                "Perspective": {
                    "position": Gf.Vec3d(0.01590454343306974, 0.754622681760281, 1.361496300347918),
                    "target": Gf.Vec3d(-0.19695870502812712, -1.7559401737217524, -3.707651891027873),
                },
                "boundCamera": "/OmniverseKit_Persp",
            }
        }
        self._stage.GetRootLayer().customLayerData = customLayerData

    def _setup_table(self):
        table_asset_path = carb.tokens.get_tokens_interface().resolve("omniverse://ov-content/Projects/DemoContent/DoNotDistribute/Physics_SDF_NutBolt/nut_vibration_table.usda")
        table_path = self._default_prim_path.AppendPath("Table")
        assert (
            self._stage.DefinePrim(table_path)
            .GetReferences()
            .AddReference(table_asset_path, "/World/Table")
        )
        table_prim = self._stage.GetPrimAtPath(table_path)
        self._table = UsdGeom.Xform(table_prim)
        ops = self._table.GetOrderedXformOps()
        translateOp = ops[0]
        assert translateOp.GetOpType() == UsdGeom.XformOp.TypeTranslate
        translateOp.Set(self._table_position)
        omni.kit.commands.execute(
            "SetRigidBodyCommand", path=table_path, approximationShape="convexHull", kinematic=True
        )

    def _setup_nuts(self):
        # paths:
        nut_asset_path = carb.tokens.get_tokens_interface().resolve("omniverse://ov-content/Projects/DemoContent/DoNotDistribute/Physics_SDF_NutBolt/M20_nut_SI_vibraTable_R256_physx_30786833.usd")

        for i in range(self._num_nuts):
            nut_path = self._default_prim_path.AppendPath(f"Nut_{i}")
            assert (
                self._stage.DefinePrim(nut_path)
                .GetReferences()
                .AddReference(nut_asset_path, "/World")
            )
            nut_prim = self._stage.GetPrimAtPath(nut_path)
            nut_xformable = UsdGeom.Xformable(nut_prim)
            nut_pos = Gf.Vec3f(self._nut_pos)
            nut_pos[0] += self._nut_radius * math.sin(math.pi * .33 * i)
            nut_pos[2] += self._nut_radius * math.cos(math.pi * .33 * i)
            nut_pos[1] += i * 0.0
            nut_xformable.AddTranslateOp().Set(nut_pos)
            quatv = 1.0 / math.sqrt(2)
            nut_xformable.AddOrientOp().Set(Gf.Quatf(quatv, quatv, 0, 0))
            # set solver iterations:
            physxRBapi = PhysxSchema.PhysxRigidBodyAPI.Apply(nut_prim)
            physxRBapi.CreateSolverPositionIterationCountAttr().Set(self._nut_solver_pos_iterations)
            nut_mesh_prim = self._stage.GetPrimAtPath(nut_path.AppendChild("nut_m20_tight"))
            physxCollisionapi = PhysxSchema.PhysxCollisionAPI.Apply(nut_mesh_prim)
            physxCollisionapi.CreateContactOffsetAttr().Set(self._nut_contact_offset)

    def on_shutdown(self):
        self._deregister_callbacks()

    def _setup_callbacks(self):
        # callbacks
        self._timeline = omni.timeline.get_timeline_interface()
        stream = self._timeline.get_timeline_event_stream()
        self._timeline_subscription = stream.create_subscription_to_pop(self._on_timeline_event)
        # subscribe to Physics updates:
        self._physics_update_subscription = omni.physx.get_physx_interface().subscribe_physics_step_events(
            self.on_physics_step
        )

    def _deregister_callbacks(self):
        self._timeline_subscription = None
        self._physics_update_subscription = None

    def _on_timeline_event(self, e):
        if e.type == int(omni.timeline.TimelineEventType.STOP):
            self._time = 0
            # cannot call update tendons here to properly reset the demo, because on new stage load while running,
            # Kit crashes due to a fabric issue otherwise.
        if e.type == int(omni.timeline.TimelineEventType.PLAY):
            # set clip range:
            self._stage.SetEditTarget(self._stage.GetSessionLayer())
            usdCamera = UsdGeom.Camera.Define(self._stage, "/OmniverseKit_Persp")
            usdCamera.CreateClippingRangeAttr(Gf.Vec2f(0.01, 1000))
            self._stage.SetEditTarget(self._stage.GetRootLayer())

    def on_physics_step(self, dt):
        assert abs(dt - self._sim_dt) < 1.0e-5
        self._time += dt
        self._update_vibration()

    def _update_vibration(self):
        offsetPos = Gf.Vec3f()
        for i in range(3):
            offsetPos[i] = self._motion_amplitudes[i] * math.sin(self._motion_phases[i] + 2.0 * math.pi * self._time * self._motion_frequency)
        ops = self._table.GetOrderedXformOps()
        translateOp = ops[0]
        assert translateOp.GetOpType() == UsdGeom.XformOp.TypeTranslate
        translateOp.Set(self._table_position + offsetPos)
