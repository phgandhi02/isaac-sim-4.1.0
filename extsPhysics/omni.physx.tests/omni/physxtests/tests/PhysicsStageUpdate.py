import omni.physx.scripts.utils as physxUtils
import omni.physx.scripts.physicsUtils as physicsUtils
from omni.physxtests.utils.physicsBase import PhysicsMemoryStageBaseAsyncTestCase, TestCategory
from omni.physxtests import utils
from omni.physx import get_physx_stage_update_interface
from omni.physxstageupdate import get_physx_stage_update_node_interface
from pxr import Gf, Sdf, UsdGeom, UsdPhysics, PhysxSchema, UsdUtils
import unittest
import carb


class PhysicsStageUpdateTestMemoryStage(PhysicsMemoryStageBaseAsyncTestCase):
    category = TestCategory.Core

    async def setUp(self):
        get_physx_stage_update_node_interface().detach_node()
        await super().setUp()        
        
    async def tearDown(self):
        self.release_stage(self._stage_attached)
        await super().tearDown()
        get_physx_stage_update_node_interface().attach_node()
        
    def create_base_scene(self, stage):
        UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
        UsdGeom.SetStageMetersPerUnit(stage, 0.01)

        defaultPrimPath = str(stage.GetDefaultPrim().GetPath())        
        UsdPhysics.Scene.Define(stage, defaultPrimPath + "/physicsScene")
        sphereActorPath = "/sphereActor"

        radius = 1.0
        position = Gf.Vec3f(0.0, 0.0, 4.0)
        orientation = Gf.Quatf(1.0)

        self.spherePrim = physicsUtils.add_rigid_sphere(stage, sphereActorPath, radius, position, orientation)

    async def test_stage_update_attached(self):
        stage = await self.new_stage(attach_stage=False)
        stage_id = UsdUtils.StageCache.Get().GetId(stage).ToLongInt()

        self.spherePrim = None
        self.create_base_scene(stage)
        
        stage_update_iface = get_physx_stage_update_interface()        
        
        stage_update_iface.on_attach(stage_id)
        self._check_physx_object_counts({"numSphereShapes": 0, "numDynamicRigids": 0})
        stage_update_iface.on_detach()
        
    async def test_stage_update_resume(self):
        stage = await self.new_stage(attach_stage=False)
        stage_id = UsdUtils.StageCache.Get().GetId(stage).ToLongInt()

        self.spherePrim = None
        self.create_base_scene(stage)
        
        stage_update_iface = get_physx_stage_update_interface()        
        
        stage_update_iface.on_attach(stage_id)
        self._check_physx_object_counts({"numSphereShapes": 0, "numDynamicRigids": 0})        
        
        stage_update_iface.on_resume(0.0)
        self._check_physx_object_counts({"numSphereShapes": 1, "numDynamicRigids": 1}) 
        stage_update_iface.on_detach()       
        
    async def test_stage_update_resume(self):
        stage = await self.new_stage(attach_stage=False)
        stage_id = UsdUtils.StageCache.Get().GetId(stage).ToLongInt()

        self.spherePrim = None
        self.create_base_scene(stage)
        
        stage_update_iface = get_physx_stage_update_interface()        
        
        stage_update_iface.on_attach(stage_id)
        self._check_physx_object_counts({"numSphereShapes": 0, "numDynamicRigids": 0})        
        
        stage_update_iface.on_resume(0.0)
        self._check_physx_object_counts({"numSphereShapes": 1, "numDynamicRigids": 1}) 
        stage_update_iface.on_detach()               
        
    async def test_stage_update_detach(self):
        stage = await self.new_stage(attach_stage=False)
        stage_id = UsdUtils.StageCache.Get().GetId(stage).ToLongInt()

        self.spherePrim = None
        self.create_base_scene(stage)
        
        stage_update_iface = get_physx_stage_update_interface()        
        
        stage_update_iface.on_attach(stage_id)
        self._check_physx_object_counts({"numSphereShapes": 0, "numDynamicRigids": 0})        
        
        stage_update_iface.on_resume(0.0)
        self._check_physx_object_counts({"numSphereShapes": 1, "numDynamicRigids": 1})                
        
        stage_update_iface.on_detach()
        self._check_physx_object_counts({"numSphereShapes": 0, "numDynamicRigids": 0})
        stage_update_iface.on_detach()        
        
    async def test_stage_update_reset(self):
        stage = await self.new_stage(attach_stage=False)
        stage_id = UsdUtils.StageCache.Get().GetId(stage).ToLongInt()

        self.spherePrim = None
        self.create_base_scene(stage)
        
        stage_update_iface = get_physx_stage_update_interface()        
        
        stage_update_iface.on_attach(stage_id)
        self._check_physx_object_counts({"numSphereShapes": 0, "numDynamicRigids": 0})        
        
        stage_update_iface.on_resume(0.0)
        self._check_physx_object_counts({"numSphereShapes": 1, "numDynamicRigids": 1})                
        
        stage_update_iface.on_reset()
        self._check_physx_object_counts({"numSphereShapes": 0, "numDynamicRigids": 0}) 
        stage_update_iface.on_detach()       
                
    async def test_stage_update_simulate_on(self):
        stage = await self.new_stage(attach_stage=False)
        stage_id = UsdUtils.StageCache.Get().GetId(stage).ToLongInt()
        epsilon = 0.02

        self.spherePrim = None
        self.create_base_scene(stage)
        
        stage_update_iface = get_physx_stage_update_interface()        
        
        stage_update_iface.on_attach(stage_id)
        self._check_physx_object_counts({"numSphereShapes": 0, "numDynamicRigids": 0})        
        
        stage_update_iface.on_resume(0.0)
        self._check_physx_object_counts({"numSphereShapes": 1, "numDynamicRigids": 1})                
        
        pos = self.spherePrim.GetAttribute("xformOp:translate").Get()
        self.assertTrue(abs(pos[2] - 4.0) < epsilon)        
        
        for i in range(10):
            stage_update_iface.on_update(i*0.02, 0.02, True)
            
        pos = self.spherePrim.GetAttribute("xformOp:translate").Get()
        self.assertTrue(pos[2] < 4.0)
        stage_update_iface.on_detach()
                
    async def test_stage_update_simulate_off(self):
        stage = await self.new_stage(attach_stage=False)
        stage_id = UsdUtils.StageCache.Get().GetId(stage).ToLongInt()
        epsilon = 0.02

        self.spherePrim = None
        self.create_base_scene(stage)
        
        stage_update_iface = get_physx_stage_update_interface()        
        
        stage_update_iface.on_attach(stage_id)
        self._check_physx_object_counts({"numSphereShapes": 0, "numDynamicRigids": 0})        
        
        stage_update_iface.on_resume(0.0)
        self._check_physx_object_counts({"numSphereShapes": 1, "numDynamicRigids": 1})                
        
        pos = self.spherePrim.GetAttribute("xformOp:translate").Get()
        self.assertTrue(abs(pos[2] - 4.0) < epsilon)        
        
        for i in range(10):
            stage_update_iface.on_update(i*0.02, 0.02, False)
            
        pos = self.spherePrim.GetAttribute("xformOp:translate").Get()
        self.assertTrue(abs(pos[2] - 4.0) < epsilon) 
        stage_update_iface.on_detach()       
        
    async def test_stage_update_simulate_reset(self):
        stage = await self.new_stage(attach_stage=False)
        stage_id = UsdUtils.StageCache.Get().GetId(stage).ToLongInt()
        epsilon = 0.02

        self.spherePrim = None
        self.create_base_scene(stage)
        
        stage_update_iface = get_physx_stage_update_interface()        
        
        stage_update_iface.on_attach(stage_id)
        self._check_physx_object_counts({"numSphereShapes": 0, "numDynamicRigids": 0})        
        
        stage_update_iface.on_resume(0.0)
        self._check_physx_object_counts({"numSphereShapes": 1, "numDynamicRigids": 1})                
        
        pos = self.spherePrim.GetAttribute("xformOp:translate").Get()
        self.assertTrue(abs(pos[2] - 4.0) < epsilon)        
        
        for i in range(10):
            stage_update_iface.on_update(i*0.02, 0.02, True)
            
        pos = self.spherePrim.GetAttribute("xformOp:translate").Get()
        self.assertTrue(pos[2] < 4.0)
        
        stage_update_iface.on_reset()
        
        pos = self.spherePrim.GetAttribute("xformOp:translate").Get()
        self.assertTrue(abs(pos[2] - 4.0) < epsilon) 
        stage_update_iface.on_detach()       
