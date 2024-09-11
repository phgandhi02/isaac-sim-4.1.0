from omni.kvdb.bindings import _kvdb
import omni.kit.test
import carb
import time


# Register with the testing system.
class Test(omni.kit.test.AsyncTestCaseFailOnLogError):
    # Method called when the tests start up.  Do any initialization necessary.
    async def setUp(self):
        pass

    # Method called when the tests are shut down. Do any cleanup necessary.
    async def tearDown(self):
        pass

