from pxr import Gf
from omni.physx.scripts import utils
import omni.physxdemos as demo
import omni.physx.bindings._physx as physx_settings_bindings
from pxr import UsdGeom


class ClothAtticDemo(demo.Base):
    title = "Cloth Curtains"
    category = demo.Categories.INTERNAL_SAMPLES
    short_description = "Cloth curtains blowing in the wind."
    description = "Position-based-dynamics cloth blowing in the wind."

    kit_settings = {
        "persistent/app/viewport/displayOptions": demo.get_viewport_minimal_display_options_int(),
        physx_settings_bindings.SETTING_MIN_FRAME_RATE: 30,
        physx_settings_bindings.SETTING_UPDATE_VELOCITIES_TO_USD: False,
        physx_settings_bindings.SETTING_UPDATE_TO_USD: True,
        "rtx/post/aa/op": 3,
        "rtx/post/dlss/execMode": 1,
        "rtx/translucency/maxRefractionBounces": 6,
    }

    def set_initial_camera(self, stage, position, target):
        customLayerData = {
            "cameraSettings": {
                "Perspective": {"position": position, "radius": 2.7207318526778557, "target": target},
                "boundCamera": "/OmniverseKit_Persp",
            }
        }
        stage.GetRootLayer().customLayerData = customLayerData

    def create(self, stage):
        UsdGeom.SetStageUpAxis(stage, "Z")

        cam_position = Gf.Vec3d(-476.27659372368436, -1367.426008178913, 345.087890558258)
        cam_target = Gf.Vec3d(-473.89655055437674, -1366.1842865168774, 344.6452805262441)
        self.set_initial_camera(stage, cam_position, cam_target)

        scene_asset_path = "omniverse://ov-content/Projects/DemoContent/DoNotDistribute/Physics_Preview_Demos/attic/Attic_NVIDIA-reduced-softbody-and-curtains.usd"

        stage.GetDefaultPrim().GetReferences().AddReference(scene_asset_path)
        defaultPrimPath = stage.GetDefaultPrim().GetPath()
        scenePrim = stage.GetPrimAtPath(defaultPrimPath.AppendChild("PhysicsScene"))
        utils.set_physics_scene_asyncsimrender(scenePrim)
