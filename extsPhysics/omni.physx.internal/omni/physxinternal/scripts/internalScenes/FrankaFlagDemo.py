from pxr import Gf, UsdGeom
import omni.physxdemos as demo
import carb
from omni.physx.scripts import utils, physicsUtils
import numpy as np

from omni.physxdemos.utils import franka_helpers
from omni.physxdemos.utils import numpy_utils

DEMO_ASSET_PATH = "omniverse://ov-content/Projects/DemoContent/DoNotDistribute/Physics_Preview_Demos/FrankaClothFlag/"

class FrankaFlagDemo(franka_helpers.FrankaDemoBase):
    title = "Franka Flag"
    category = demo.Categories.INTERNAL_SAMPLES
    short_description = "Franka robot arm picks up a flag"
    description = (
        "A franka robot arm described by an articulation picks up a flag which is implemented using particle cloth."
    )

    def __init__(self):
        super().__init__()

        self._camera_location_shift = Gf.Vec3f(1.23, -0.5, 0.33)  # shifts the camera from a location centered on all environments
        self._camera_target_shift = Gf.Vec3f(0.0, -0.2, 0.6)  # shifts the camera target away from the origin

        self.asset_paths = {
            "franka": DEMO_ASSET_PATH + "SubUSDs/Franka/franka_alt_fingers.usd",
        }
        self.demo_base_usd_url = DEMO_ASSET_PATH + "StagingClothFlag.usd"

        self._fsm_dt = 1.0 / 60.0

    def _setup_camera(self):
        super()._setup_camera()
        self._cam.CreateFocalLengthAttr().Set(16.0)

    def _setup_simulation(self):
        physxSceneAPI = super()._setup_simulation()

        # friction
        physxSceneAPI.CreateFrictionOffsetThresholdAttr().Set(0.01)
        physxSceneAPI.CreateFrictionCorrelationDistanceAttr().Set(0.0005)

        # GPU
        physxSceneAPI.CreateGpuTotalAggregatePairsCapacityAttr().Set(10 * 1024)
        physxSceneAPI.CreateGpuFoundLostPairsCapacityAttr().Set(10 * 1024)
        physxSceneAPI.CreateGpuCollisionStackSizeAttr().Set(64 * 1024 * 1024)
        physxSceneAPI.CreateGpuFoundLostAggregatePairsCapacityAttr().Set(10 * 1024)

    def _generate_fsm(self, env_idx):
        return FlagFSM(env_idx=env_idx)

    def _add_stool(self, env_path):
        env_xform = UsdGeom.Xform.Get(self._stage, env_path)

        stool_path = env_xform.GetPath().AppendChild("franka_stool")
        self._stage.DefinePrim(stool_path).GetReferences().AddReference(
            DEMO_ASSET_PATH + "SubUSDs/Franka_Stool/Franka_Stool.usd"
        )
        stool_xform = UsdGeom.Xform.Get(self._stage, stool_path)
        utils.setStaticCollider(stool_xform.GetPrim(), approximationShape="boundingCube")
        assert stool_xform
        physicsUtils.set_or_add_translate_op(stool_xform, translate=Gf.Vec3f(-0.027, 0.0, -0.001))

    def _add_cloth(self, env_path):
        env_xform = UsdGeom.Xform.Get(self._stage, env_path)
        cloth_path = env_xform.GetPath().AppendPath("cloth")
        assert (
            self._stage.DefinePrim(cloth_path)
            .GetReferences()
            .AddReference(DEMO_ASSET_PATH + "SubUSDs/cloth.usda")
        )
        self._cloth_prim = self._stage.GetPrimAtPath(cloth_path)
        assert self._cloth_prim
        physicsUtils.set_or_add_translate_op(
            UsdGeom.Xform.Get(self._stage, cloth_path), translate=Gf.Vec3f(-0.25, 0.0, 0.0)
        )

    def _add_custom(self, env_path):
        self._add_stool(env_path=env_path)
        self._add_cloth(env_path=env_path)

    def on_physics_step(self, dt):
        carb.profiler.begin(1, "Franka FSM Update")
        self._time += self._sim_dt
        self._fsm_time += self._sim_dt

        if self._fsm_time < self._fsm_dt:
            carb.profiler.end(1)
            return
        self._fsm_time = 0.0

        # get end effector transforms
        hand_transforms = self._hands.get_transforms()
        hand_poses = hand_transforms[:, :7]

        # FSM update
        for env in range(self._num_envs):
            self._fsms[env].update(self._fsm_dt, hand_poses[env, :])
            self._d_pose[env, :] = self._fsms[env].franka_state.pose_delta
            self._grip_sep[env] = self._fsms[env].franka_state.gripper_separation

        self._run_ik()

        carb.profiler.end(1)


class FlagFSM:
    def __init__(self, env_idx):

        self._env_idx = env_idx

        self.reset()

        self._hand_down_quat = np.array([1, 0, 0, 0], dtype=np.float32)

        self.franka_state = franka_helpers.FrankaState()
        self.franka_state.pos_error_threshold = 5.0e-3
        self.franka_state.rot_error_threshold = 2.5e-2
        self.franka_state.alignment_gain = -2.0

    def reset(self):
        self._state = "initial_pose"

    def update(self, fsm_dt, hand_pose):
        new_state = self._state

        if self._state == "advance":
            if self.franka_state.advance(hand_pose=hand_pose, dt=fsm_dt):
                new_state = self._state_after_advance
        elif self._state == "initial_pose":

            self.franka_state.gripper_separation = 0.08

            angle_rad_x = 0 * np.pi / 180
            target_quat = numpy_utils.quat_mul(numpy_utils.get_x_rot_quat(angle_rad_x), self._hand_down_quat)

            angle_rad_y = 0 * np.pi / 180
            target_quat = numpy_utils.quat_mul(numpy_utils.get_y_rot_quat(angle_rad_y), target_quat)

            angle_rad_z = -45 * np.pi / 180
            target_quat = numpy_utils.quat_mul(numpy_utils.get_z_rot_quat(angle_rad_z), target_quat)

            end_pos = np.array([0, 0.4, 0.8])

            new_state = "advance"
            self._state_after_advance = "touch_flag_pose"
            self.franka_state.current_time = 0.0
            self.franka_state.pose_motion = franka_helpers.PoseMotion(
                start_pos=hand_pose[:3],
                end_pos=end_pos,
                start_quat=hand_pose[3:],
                end_quat=target_quat,
                duration=2.0,
                use_alignment=True,
            )
        elif self._state == "touch_flag_pose":

            self.franka_state.gripper_separation = 0.08

            angle_rad_z = 90 * np.pi / 180
            target_quat = numpy_utils.quat_mul(numpy_utils.get_z_rot_quat(angle_rad_z), self._hand_down_quat)

            angle_rad_x = -45 * np.pi / 180
            target_quat = numpy_utils.quat_mul(numpy_utils.get_x_rot_quat(angle_rad_x), target_quat)

            angle_rad_y = 0 * np.pi / 180
            target_quat = numpy_utils.quat_mul(numpy_utils.get_y_rot_quat(angle_rad_y), target_quat)

            end_pos = np.array([0.35, -0.24, 0.56])

            new_state = "advance"
            self._state_after_advance = "grip_flag_pose"
            self.franka_state.current_time = 0.0
            self.franka_state.pose_motion = franka_helpers.PoseMotion(
                start_pos=hand_pose[:3],
                end_pos=end_pos,
                start_quat=hand_pose[3:],
                end_quat=target_quat,
                duration=4.0,
                use_alignment=True,
            )
        elif self._state == "grip_flag_pose":

            self.franka_state.gripper_separation = 0.08

            angle_rad_z = 90 * np.pi / 180
            target_quat = numpy_utils.quat_mul(numpy_utils.get_z_rot_quat(angle_rad_z), self._hand_down_quat)

            angle_rad_x = -45 * np.pi / 180
            target_quat = numpy_utils.quat_mul(numpy_utils.get_x_rot_quat(angle_rad_x), target_quat)

            angle_rad_y = 0 * np.pi / 180
            target_quat = numpy_utils.quat_mul(numpy_utils.get_y_rot_quat(angle_rad_y), target_quat)

            end_pos = np.array([0.42, -0.25, 0.55])

            new_state = "advance"
            self._state_after_advance = "hold_flag_up_pose"
            self.franka_state.current_time = 0.0
            self.franka_state.pose_motion = franka_helpers.PoseMotion(
                start_pos=hand_pose[:3],
                end_pos=end_pos,
                start_quat=hand_pose[3:],
                end_quat=target_quat,
                duration=1,
                use_alignment=True,
            )
        elif self._state == "hold_flag_up_pose":

            self.franka_state.gripper_separation = 0.000

            angle_rad_z = 90 * np.pi / 180
            target_quat = numpy_utils.quat_mul(numpy_utils.get_z_rot_quat(angle_rad_z), self._hand_down_quat)

            angle_rad_x = 0 * np.pi / 180
            target_quat = numpy_utils.quat_mul(numpy_utils.get_x_rot_quat(angle_rad_x), target_quat)

            angle_rad_y = 0 * np.pi / 180
            target_quat = numpy_utils.quat_mul(numpy_utils.get_y_rot_quat(angle_rad_y), target_quat)

            end_pos = np.array([0.35, -0.02, 0.88])

            new_state = "advance"
            self._state_after_advance = "freeze"
            self.franka_state.current_time = 0.0
            self.franka_state.pose_motion = franka_helpers.PoseMotion(
                start_pos=hand_pose[:3],
                end_pos=end_pos,
                start_quat=hand_pose[3:],
                end_quat=target_quat,
                duration=3.0,
                use_alignment=True,
                end_delay=0.5,
            )
        elif self._state == "freeze":
            pass

        if new_state != self._state:
            self._state = new_state
