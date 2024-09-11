#from pxr import Gf, UsdGeom, UsdPhysics, PhysxSchema
#import omni.physx
from omni.physxtests.utils.physicsBase import PhysicsKitStageAsyncTestCase
from pathlib import Path
from omni.physxpvd.bindings import _physxPvd
import omni.usd

from omni.physxpvd.scripts.extension import PhysxPvdExtension

from omni.physxpvd.scripts.omniusd_to_physxusd.omniusd_to_physxusd import ConvertOmniPvdToPhysXUSD

class OmniPVDBaseTests(PhysicsKitStageAsyncTestCase):
    async def test_ovd_to_usd(self):

        self._physxPvdInterface = _physxPvd.acquire_physx_pvd_interface()

        filePath2 = str(Path(__file__).parent.parent.parent.parent.parent.joinpath("data/various.ovd"))
        print(filePath2)
        upAxis = 1
        usdaType = 1

        # Just test the reading by passing an empty string for the output path
        self.assertTrue(self._physxPvdInterface.ovd_to_usd(filePath2, "", upAxis, usdaType))
        
        self._physxPvdInterface = None

    async def test_window(self):
        PhysxPvdExtension.instance._menu.show_window()

    async def test_omniusd_to_physxusd(self):

        filePath2 = str(Path(__file__).parent.parent.parent.parent.parent.joinpath("data/various_omnipvd_usd/stage.usda"))
        print(filePath2)

        # Load the stage to be the active stage
        omni.usd.get_context().open_stage(filePath2)
        
        # Convert to PhysX USD, but as a stage in memory for the destination
        toPhysxUSDConverter = ConvertOmniPvdToPhysXUSD()

        outputStage = toPhysxUSDConverter.convert("", True)
        self.assertTrue(outputStage != None)
        self.assertTrue(outputStage.GetPrimAtPath("/rigiddynamic/PxActor_1_World_boxActor"))

        outputStage = None

        toPhysxUSDConverter = None
