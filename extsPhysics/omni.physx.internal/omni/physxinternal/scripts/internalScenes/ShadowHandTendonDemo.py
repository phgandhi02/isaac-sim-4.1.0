import carb
from pxr import UsdGeom, Gf, UsdPhysics
import omni.physx
import omni.physxdemos as demo
from omni.physxinternal.bindings import _physxInternal
import math


class JointAngleActuator:
    def __init__(self, jointDriveAPI, initValue: float = 0.0, maxRateRad: float = 0.01):
        self.maxDeltaPerFrame = maxRateRad
        self.jointDriveAPI = jointDriveAPI
        self.resetValue = initValue
        self.move = False
        self.reset()

    def set_current_angle_in_drive_api(self):
        targetDeg = self.currentValue
        self.jointDriveAPI.GetTargetPositionAttr().Set(targetDeg)

    def reset(self):
        self.targetValue = self.resetValue
        self.currentValue = self.resetValue
        self.move = False
        self.set_current_angle_in_drive_api()

    def set_target(self, targetValue):
        self.targetValue = targetValue

    def update(self):
        if self.currentValue > 0:
            self.currentValue = 0  # 0 deg
        else:
            self.currentValue = 90  # 90 deg
        self.set_current_angle_in_drive_api()


class ShadowHandDemo(demo.Base):
    title = "Shadow Hand"
    category = demo.Categories.INTERNAL_SAMPLES
    short_description = "Shadow hand(s) rigged with fixed tendons"
    description = "Shadow hand, fixed tendons, joint drives. There is only one joint drive per finger, the joint coupling is achieved via fixed tendons"

    params = {
        "use_25_hands": demo.CheckboxParam(False)
    }

    def __init__(self):
        self._shadowhand_path = "omniverse://ov-content:3009/Projects/DemoContent/DoNotDistribute/Physics_ShadowHand/GymHand.usd"
        self._step = 0
        self._is_playing = False
        self._jointDrivers = []
        self._index = 0

        self._drive_max_force = 1e20
        self._revolute_drive_stiffness = 10
        self._revolute_drive_damping = 1

        self._simulationRate = 60.0  # [Hz]
        jointAnglesMaxDegreesPerSecond = 20.0  # [deg]
        self._jointAngleRateLimitRad = jointAnglesMaxDegreesPerSecond / self._simulationRate * math.pi / 180.0

        # start finger movement in different positions
        self._start_angles = [0.0, 0.0, 0.0, 0.0]  # 25.0 / radToDeg, 50.0 / radToDeg, 75.0 / radToDeg]

    def on_shutdown(self):
        self._deregister_callbacks()

    def on_physics_step(self, dt):
        self._step += 1
        if self._is_playing:
            if self._step % 30 == 0:
                for arr in self._jointDrivers:
                    arr[self._index].update()
                if self._step % 60 == 0:
                    self._index += 1
                    if self._index > 3:
                        self._index = 0

    def _on_timeline_event(self, e):
        # on play press
        if e.type == int(omni.timeline.TimelineEventType.PLAY):
            self._is_playing = True
            pass
        # on stop press
        if e.type == int(omni.timeline.TimelineEventType.STOP):
            # reset step counter
            self._step = 0
            self._is_playing = False
            self._index = 0
            # reset drivers
            for arr in self._jointDrivers:
                for driver in arr:
                    driver.reset()

    def create(self, stage, use_25_hands: bool = False):
        self._stage = stage
        default_prim_path = stage.GetDefaultPrim().GetPath()
        abspath = carb.tokens.get_tokens_interface().resolve(self._shadowhand_path)
        assert stage.DefinePrim(default_prim_path.AppendPath("physicsScene")).GetReferences().AddReference(abspath, "/physicsScene")
        assert stage.DefinePrim(default_prim_path.AppendPath("groundPlane")).GetReferences().AddReference(abspath, "/groundPlane")
        self._shadowHandPathString = default_prim_path.AppendPath("ShadowHand").pathString
        self._setup_callbacks()

        # setup internal interface and stage id:
        self._physxInternal = _physxInternal.acquire_physx_internal_interface()
        self._stage_id = omni.usd.get_context().get_stage_id()

        if use_25_hands:
            range_len = 5
        else:
            range_len = 1

        for x in range(range_len):
            for y in range(range_len):
                hand_name = "ShadowHand_" + str(x) + str(y)
                hand_geom = UsdGeom.Xform.Define(self._stage, default_prim_path.AppendPath(hand_name))
                pos = Gf.Vec3d(x * 80.0, y * 80.0, 40.0)
                hand_geom.AddTranslateOp().Set(pos)

                assert stage.DefinePrim(hand_geom.GetPath().AppendPath("ShadowHand")).GetReferences().AddReference(abspath, "/ShadowHand")

                # get joints to actuate
                jointPath = hand_geom.GetPath().AppendPath("ShadowHand").AppendChild("joints")
                middle_prim_paths = [jointPath.AppendChild("robot0_ffmiddle"), jointPath.AppendChild("robot0_mfmiddle"),
                                     jointPath.AppendChild("robot0_rfmiddle"), jointPath.AppendChild("robot0_lfmiddle")]

                # create joint drivers
                i = 0
                joint_drives = []
                for path in middle_prim_paths:
                    joint = UsdPhysics.RevoluteJoint(self._stage.GetPrimAtPath(path))
                    driveAPI = UsdPhysics.DriveAPI.Apply(joint.GetPrim(), "angular")
                    # driveAPI.CreateMaxForceAttr(self._drive_max_force)
                    driveAPI.CreateDampingAttr(self._revolute_drive_damping)
                    driveAPI.CreateStiffnessAttr(self._revolute_drive_stiffness)
                    targetAngle = self._start_angles[i]
                    joint_drives.append(JointAngleActuator(driveAPI, targetAngle, self._jointAngleRateLimitRad))
                    i += 1

                self._jointDrivers.append(joint_drives)

        # setup internal interface and stage id:
        self._physxInternal = _physxInternal.acquire_physx_internal_interface()
        self._stage_id = omni.usd.get_context().get_stage_id()

        # setup scene and stage:
        UsdGeom.SetStageMetersPerUnit(stage, 0.01)
        UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)

        # set cam:
        customLayerData = {
            "cameraSettings": {
                "Perspective": {"position": Gf.Vec3d(-50, -61, 60), "target": Gf.Vec3d(-23, -36, 55)},
                "boundCamera": "/OmniverseKit_Persp",
            }
        }
        stage.GetRootLayer().customLayerData = customLayerData

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