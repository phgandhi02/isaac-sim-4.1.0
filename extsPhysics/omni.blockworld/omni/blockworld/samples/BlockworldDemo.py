from collections import defaultdict
import carb.input
import carb.tokens
import omni.appwindow
import omni.timeline
from pxr import Tf, UsdGeom, Usd, Gf, UsdPhysics, PhysxSchema, UsdLux
import omni.physxdemos as demo
from omni.physx import get_physx_interface
from omni.physxcct.scripts import utils
import math
import os
from functools import partial
from carb.input import KeyboardInput

DEFAULT_FILE = demo.get_demo_asset_path("Blockworld/Eiffel.usda")


class BlockworldDemo(demo.Base):
    title = "Block World"
    category = demo.Categories.CCT
    short_description = "Character controller in a block world"
    description = "Block world map, character controller. Press ASDW to move and hold right mouse button to rotate the camera. Press TG to change block type, YH to change block subtype, E to set a block and R to reset a block."
    tags = ["Character Controller"]

    params = [
        {"speed": demo.IntParam(10, 0, 100, 1), "gravity": demo.CheckboxParam(True)},
        {"filepath": demo.FileParam(DEFAULT_FILE)},
    ]

    def change_preview_block(self, type_offset, subtype_offset):
        # reset subtype on type change
        if type_offset != 0:
            self.subtype_ind = 0

        # rotate type and subtype indices and refresh block type and subtype
        self.type_ind = (self.type_ind + type_offset) % len(self.blocklib_keys)
        self.block_type = self.blocklib_keys[self.type_ind]
        subtypelib = self.blocklib[self.block_type]
        self.subtype_ind = (self.subtype_ind + subtype_offset) % len(subtypelib)
        self.block_subtype = subtypelib[self.subtype_ind]

        # show visual
        stage = omni.usd.get_context().get_stage()
        previewPrim = stage.GetPrimAtPath(self._preview_path)
        if previewPrim.IsValid():
            path = f"/World/VoxelMap/BlockLib/Blocks/Block_{str(self.block_type)}_{str(self.block_subtype)}"
            previewPrim.GetInherits().SetInherits([path])

    def set_voxel(self, x, y, z, type, subType):
        get_physx_interface().set_voxel_range(self._stage_id, self._voxelMapPathString, x, y, z, x, y, z, type, subType, 0)

    def set_voxel_range(self, sx, sy, sz, ex, ey, ez, type, subType):
        get_physx_interface().set_voxel_range(self._stage_id, self._voxelMapPathString, sx, sy, sz, ex, ey, ez, type, subType, 0)

    def update_voxels(self):
        get_physx_interface().set_voxel_range(self._stage_id, self._voxelMapPathString, 0, 0, 0, -1, -1, -1, 0, 0, 1)

    def create(self, stage, gravity, speed, filepath):
        UsdGeom.SetStageMetersPerUnit(stage, 1)

        default_prim_path = str(stage.GetDefaultPrim().GetPath())

        self._stage = stage
        self._stage_id = omni.usd.get_context().get_stage_id()
        self._voxelMapPathString = "/World/VoxelMap"

        # add base scene
        abspath = carb.tokens.get_tokens_interface().resolve(filepath)
        prim_path = "/World/VoxelMap"
        tmp_stage = Usd.Stage.Open(abspath)
        for prim in tmp_stage.Traverse():
            if prim.GetName() == "VoxelMap":
                prim_path = prim.GetPrimPath()
                break
        tmp_stage = None
        stage.DefinePrim(default_prim_path + "/VoxelMap").GetReferences().AddReference(abspath, prim_path)

        # register available blocks
        blocks_path = default_prim_path + "/VoxelMap/BlockLib/Blocks"
        blocks_prim = stage.GetPrimAtPath(blocks_path)
        looks_prim = stage.GetPrimAtPath(f"{blocks_path}/Looks")
        pit = iter(Usd.PrimRange(blocks_prim))
        id_start = len(f"{blocks_path}/Block_")
        self.blocklib = defaultdict(list)
        max_type = 0
        for p in pit:
            if p == looks_prim:
                pit.PruneChildren()
                continue

            if p.IsA(UsdGeom.Xform):
                ids = str(p.GetPath())[id_start:].split("_")
                id_type, id_sub = int(ids[0]), int(ids[1])
                max_type = max(max_type, id_type)
                self.blocklib[id_type].append(id_sub)
        self.blocklib_keys = list(self.blocklib.keys())
        self.type_ind = 0
        self.subtype_ind = 0

        # create capsule for cct
        path_cct = default_prim_path + "/CapsuleCCT"
        capsule_geom = UsdGeom.Capsule.Define(stage, path_cct)
        capsule_geom.CreateHeightAttr(1.85)
        capsule_geom.CreateRadiusAttr(0.9)
        capsule_geom.CreatePurposeAttr(UsdGeom.Tokens.guide)
        capsule_geom.CreateAxisAttr(UsdGeom.GetStageUpAxis(stage))
        capsule_geom.AddTranslateOp().Set(Gf.Vec3f(0, 16, 0))
        capsule_geom.AddOrientOp().Set(Gf.Quatf(1.0))
        capsule_geom.AddScaleOp().Set(Gf.Vec3f(1.0))

        # camera
        cameraPath = default_prim_path + "/CameraCCT"
        usdCamera = UsdGeom.Camera.Define(stage, cameraPath)
        usdCamera.CreateClippingRangeAttr(Gf.Vec2f(0.1, 1000))
        usdCamera.CreateHorizontalApertureAttr(60)

        # light
        lampLight = UsdLux.SphereLight.Define(stage, cameraPath + "/Lamp")
        lampLight.CreateRadiusAttr(0)
        lampLight.CreateIntensityAttr(10000)
        lampLight.AddTranslateOp().Set(Gf.Vec3f(0, 2, 0))

        # block preview
        preview = UsdGeom.Xform.Define(stage, cameraPath + "/Preview")
        self._preview_path = cameraPath + "/Preview/Block"
        stage.DefinePrim(self._preview_path)
        self.change_preview_block(0, 0)
        preview.AddTranslateOp().Set(Gf.Vec3f(0.25, -0.325, -1))
        preview.AddScaleOp().Set(Gf.Vec3f(0.3, 0.3, 0.3))

        # create sphere for target
        self._target_path = cameraPath + "/Target"
        targetGeom = UsdGeom.Sphere.Define(stage, self._target_path)
        targetGeom.CreateRadiusAttr(0.025)
        targetGeom.AddTranslateOp().Set(Gf.Vec3f(0, 0, -2.5))

        # Physics scene
        physicsScenePath = default_prim_path + "/PhysicsScene"
        scene = UsdPhysics.Scene.Define(stage, physicsScenePath)
        scene.CreateGravityDirectionAttr().Set(Gf.Vec3f(0.0, -1.0, 0.0))
        scene.CreateGravityMagnitudeAttr().Set(9.81)
        physxAPI = PhysxSchema.PhysxSceneAPI.Apply(scene.GetPrim())
        physxAPI.CreateSolverTypeAttr("TGS")

        # light
        sunLight = UsdLux.DistantLight.Define(stage, default_prim_path + "/Sun")
        sunLight.CreateIntensityAttr(1000)
        sunLight.AddTranslateOp().Set(Gf.Vec3f(-90.0, 130.0, 110.0))
        rotZYXangles = Gf.Vec3f(290, 345, 0)
        sunLight.AddRotateZYXOp(precision=UsdGeom.XformOp.PrecisionDouble).Set(rotZYXangles)

        # start cct control
        self._cct = utils.CharacterController(path_cct, cameraPath, gravity)
        self._cct.activate(stage)
        self._cct.setup_controls(speed, utils.ControlFlag.KBD_MOUSE)
        state = self._cct.control_state
        state.register_keyboard_action("BlockworldTypeUp", KeyboardInput.T, 0, partial(self._on_change_action, 1, 0))
        state.register_keyboard_action("BlockworldTypeDown", KeyboardInput.G, 0, partial(self._on_change_action, -1, 0))
        state.register_keyboard_action("BlockworldSubTypeUp", KeyboardInput.Y, 0, partial(self._on_change_action, 0, 1))
        state.register_keyboard_action("BlockworldSubTypeDown", KeyboardInput.H, 0, partial(self._on_change_action, 0, -1))
        state.register_keyboard_action("BlockworldSetBlock", KeyboardInput.E, 0, self._on_set_block_action)
        state.register_keyboard_action("BlockworldResetBlock", KeyboardInput.R, 0, self._on_reset_block_action)

        # reset camera pos and view
        customLayerData = {
            "cameraSettings": {
                "Perspective": {"position": Gf.Vec3d(0, 16, 0), "radius": 5, "target": Gf.Vec3d(1, 16, 0)},
            }
        }
        stage.GetRootLayer().customLayerData = customLayerData

    def on_shutdown(self):
        self._cct.shutdown()
        self._cct = None

    def _on_change_action(self, type_offset, subtype_offset, e):
        if e.flags & carb.input.BUTTON_FLAG_PRESSED:
            self.change_preview_block(type_offset, subtype_offset)

    def _on_set_block_action(self, e):
        print("set block action")
        if e.flags & carb.input.BUTTON_FLAG_PRESSED:
            stage = omni.usd.get_context().get_stage()
            targetPos = UsdGeom.Xformable.Get(stage, self._target_path).ComputeLocalToWorldTransform(Usd.TimeCode.Default()).ExtractTranslation()
            self.set_voxel(int(math.floor(targetPos[0])), int(math.floor(targetPos[1])), int(math.floor(targetPos[2])), self.block_type, self.block_subtype)
            self.update_voxels()

    def _on_reset_block_action(self, e):
        if e.flags & carb.input.BUTTON_FLAG_PRESSED:
            stage = omni.usd.get_context().get_stage()
            targetPos = UsdGeom.Xformable.Get(stage, self._target_path).ComputeLocalToWorldTransform(Usd.TimeCode.Default()).ExtractTranslation()
            self.set_voxel(int(math.floor(targetPos[0])), int(math.floor(targetPos[1])), int(math.floor(targetPos[2])), 0, 0)
            self.update_voxels()
