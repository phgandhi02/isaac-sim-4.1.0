from pxr import Gf, Sdf, UsdGeom
from omni.physx.scripts import physicsUtils, particleUtils, utils
import omni.physxdemos as demo
import omni.physx.bindings._physx as physx_settings_bindings

class ClothDeckChairDemo(demo.Base):
    title = "Cloth Deck Chair"
    category = demo.Categories.PARTICLES
    short_description = "Fan blowing at a particle-cloth deck chair"
    description = "Deck chair with position-based-dynamics cloth being blown at by a fan."

    kit_settings = {
        "persistent/app/viewport/displayOptions": demo.get_viewport_minimal_display_options_int(),
        physx_settings_bindings.SETTING_MIN_FRAME_RATE: 30,
        physx_settings_bindings.SETTING_UPDATE_VELOCITIES_TO_USD: False,
        physx_settings_bindings.SETTING_UPDATE_TO_USD: True,
        "rtx/post/aa/op": 3,
        "rtx/post/dlss/execMode": 1,
        "rtx/translucency/maxRefractionBounces": 6,
    }

    def __init__(self):
        self.demo_base_usd_url = demo.get_demo_asset_path("ClothDeckChair/DeckChairWithFan.usd")

    def create(self, stage):

        defaultPrimPath, scene = demo.setup_physics_scene(self, stage, upAxis = UsdGeom.Tokens.y)

        room = demo.get_demo_room(self, stage, zoom = 0.3, camYaw = 0.7, camPitch = 0.45, camElevation = 125.0, hasTable = False, roomTemplateStr = "arcadeInverse", pathsToHide = [
            defaultPrimPath + "/DomeLight",
            defaultPrimPath + "/DistantLight",
            defaultPrimPath + "/GroundPlane_01",
            defaultPrimPath + "/DemoCamera"
        ])
