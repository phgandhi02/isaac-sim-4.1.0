import omni.kit.test
from pxr import UsdUtils, Usd
from omni.usdphysicstests.scripts.tests import utils
from omni.usdphysicstests import get_usd_physics_test_interface


class UsdPhysicsBaseTest(omni.kit.test.AsyncTestCase):

    def report_callback(self, dict):
        prim_dict = self.expected_prims.get(dict["prim_path"] + "/" + dict["object_type"])
        if prim_dict is not None:
            for key, val in prim_dict.items():
                report_val = dict.get(key)
                if val is not None or report_val is not None:
                    self.assertTrue(report_val is not None)
                    if report_val is not None:
                        if not utils.compare_values(report_val, val):
                            print("Failed to compare reported and expected values: " + str(report_val) + "  " + str(val))
                            self.assertTrue(False)
            prim_dict["parsed"] = True

    async def parse(self, fileName):        
        self.stage_id = utils.open_usd(fileName)

        testI = get_usd_physics_test_interface()

        testI.report_object_desc_callback(self.report_callback)
        testI.attach_stage(self.stage_id)
        testI.deattach_stage()
        testI.report_object_desc_callback(None)

        for dictVal in self.expected_prims.values():
            parsed_correctly = False
            for key, val in dictVal.items():
                if key == "parsed" and val:
                    parsed_correctly = True
            self.assertTrue(parsed_correctly)

    async def tearDown(self):
        cache = UsdUtils.StageCache.Get()
        stage = cache.Find(Usd.StageCache.Id.FromLongInt(self.stage_id))
        cache.Erase(stage)
