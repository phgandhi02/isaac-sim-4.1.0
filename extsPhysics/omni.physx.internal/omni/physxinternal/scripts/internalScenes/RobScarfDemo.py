from pxr import Gf, PhysxSchema
from omni.physx.scripts import utils
import omni.physxdemos as demo
import omni.physx.bindings._physx as physx_settings_bindings
from omni.kit.commands import execute
import omni.kit.property as kitProperties


class RobScarfDemo(demo.Base):
    title = "Rob's Scarf"
    category = demo.Categories.INTERNAL_SAMPLES
    short_description = "Humanoid with a scarf."
    description = "Humanoid wearing a scarf around the neck."

    kit_settings = {
        "persistent/app/viewport/displayOptions": (1 << 13) | 1,  # disable all except fps and gpu mem
        physx_settings_bindings.SETTING_MIN_FRAME_RATE: 30,
        physx_settings_bindings.SETTING_UPDATE_VELOCITIES_TO_USD: False,
        physx_settings_bindings.SETTING_UPDATE_TO_USD: True,
        "rtx/post/aa/op": 3,
        "rtx/post/dlss/execMode": 1,
        "rtx/translucency/maxRefractionBounces": 6
    }

    def set_initial_camera(self, stage):
        customLayerData = {
            "cameraSettings": {
                "Front": {"position": Gf.Vec3d(0, 0, 50000), "radius": 500},
                "Perspective": {"position": Gf.Vec3d(0.619767018788089, 160.83988653341711, 69.70316303867055), "radius": 500, "target": Gf.Vec3d(-0.4144133548040685, 151.30583510969362, -12.131635274479265)},
                "Right": {"position": Gf.Vec3d(-50000, 0, -1.1102230246251565e-11), "radius": 500},
                "Top": {"position": Gf.Vec3d(-4.329780281177466e-12, 50000, 1.1102230246251565e-11), "radius": 500},
                "boundCamera": "/World/Camera",
            }
        }
        stage.GetRootLayer().customLayerData = customLayerData

    def create(self, stage):
        self.set_initial_camera(stage)

        scene_asset_path = demo.get_demo_asset_path("scarfTest2/scarfTest_physX_femCloth.usd")

        stage.GetDefaultPrim().GetReferences().AddReference(scene_asset_path)
        defaultPrimPath = stage.GetDefaultPrim().GetPath()
        scenePrim = stage.GetPrimAtPath(defaultPrimPath.AppendChild("PhysicsScene"))
