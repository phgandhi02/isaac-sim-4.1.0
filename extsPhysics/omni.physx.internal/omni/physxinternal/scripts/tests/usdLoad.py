import os
import omni.usd
import omni.kit.test
import omni.kit.stage_templates
from omni.physx import get_physxunittests_interface, get_physx_simulation_interface
from omni.physxtests import utils
from omni.physxtests.utils.physicsBase import PhysicsBaseAsyncTestCase
from pxr import Usd, UsdUtils


class PhysXUSDLoadTest(PhysicsBaseAsyncTestCase):
    def open_usd(self, filename):        
        data_path = "../../../../../data/tests"
        schema_folder = os.path.abspath(os.path.normpath(os.path.join(__file__, data_path)))
        schema_folder = schema_folder.replace("\\", "/") + "/"        
    
        stage = Usd.Stage.Open(schema_folder + "/" + filename + ".usda")
        cache = UsdUtils.StageCache.Get()
        cache.Insert(stage)
        return cache.GetId(stage).ToLongInt()             

    def load_usd_file(
        self,
        fileName,
        numDynRig,
        numStatRig,
        numKinRig,
        numSphreShapes,
        numBoxShps,
        numCapsShapes,
        numCylShapes,
        numConvexShapes,
        numTriShapes,
        numPlShapes,
        numConeShapes,
    ):
        stage_id = self.open_usd(fileName)

        get_physx_simulation_interface().attach_stage(stage_id)

        physxUT = get_physxunittests_interface()
        physxUT.update(1.0 / 60.0, 1.0 / 60.0)
        simStats = physxUT.get_physics_stats()
        
        self.assertTrue(simStats["numDynamicRigids"] == numDynRig)
        self.assertTrue(simStats["numStaticRigids"] == numStatRig)
        self.assertTrue(simStats["numKinematicBodies"] == numKinRig)
        self.assertTrue(simStats["numSphereShapes"] == numSphreShapes)
        self.assertTrue(simStats["numBoxShapes"] == numBoxShps)
        self.assertTrue(simStats["numCapsuleShapes"] == numCapsShapes)
        self.assertTrue(simStats["numCylinderShapes"] == numCylShapes)
        self.assertTrue(simStats["numConvexShapes"] == numConvexShapes)
        self.assertTrue(simStats["numTriMeshShapes"] == numTriShapes)
        self.assertTrue(simStats["numPlaneShapes"] == numPlShapes)
        self.assertTrue(simStats["numConeShapes"] == numConeShapes)

        get_physx_simulation_interface().detach_stage()

    async def test_physics_USD(self):         
        self.load_usd_file("RigidBody_Schema_Tests/BoxOnPlane", 1, 1, 0, 0, 1, 0, 0, 0, 0, 1, 0)
        self.load_usd_file("RigidBody_Schema_Tests/BoxOnPlaneInstanced", 2, 1, 0, 0, 2, 0, 0, 0, 0, 1, 0)
        self.load_usd_file("RigidBody_Schema_Tests/MultiShapeBodyOnPlane", 1, 1, 0, 0, 2, 0, 0, 0, 0, 1, 0)
        self.load_usd_file("RigidBody_Schema_Tests/VariousShapesOnTrimesh", 6, 1, 0, 1, 1, 1, 1, 1, 1, 0, 1)

    async def test_physics_regression_USD(self):        
        # regression files        
        self.load_usd_file("Regression_Tests/box_ptinst_physx", 2, 1, 0, 0, 2, 0, 0, 0, 0, 1, 0)
