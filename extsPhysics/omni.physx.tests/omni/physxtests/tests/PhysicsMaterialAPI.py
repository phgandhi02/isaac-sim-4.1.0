import omni.physx.scripts.utils as physicsBaseUtils
import omni.physx.scripts.physicsUtils as physicsUtils
from omni.physxtests.utils.physicsBase import PhysicsMemoryStageBaseAsyncTestCase, PhysicsKitStageAsyncTestCase, TestCategory
from omni.physx import get_physx_interface, get_physx_simulation_interface
from omni.physxtests import utils
from pxr import Sdf, Usd, Gf, UsdGeom, UsdPhysics, UsdUtils, UsdShade, PhysxSchema, Vt, PhysicsSchemaTools
import math
import carb
import asyncio
import os
import unittest


class PhysicsMaterialAPITestMemoryStage(PhysicsMemoryStageBaseAsyncTestCase):
    category = TestCategory.Core
    
    async def setup_scene(self, compound):
        stage = await self.new_stage()
        self.stage = stage

        UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
        UsdGeom.SetStageMetersPerUnit(stage, 0.01)

        # Physics scene
        UsdPhysics.Scene.Define(stage, Sdf.Path("/physicsScene"))

        # material
        materialPath = "/physicsMaterial"
        self.material_path = materialPath
        matShade = UsdShade.Material.Define(stage, materialPath)
        UsdPhysics.MaterialAPI.Apply(matShade.GetPrim())

        mesh_path = "/World/mesh"
        self.mesh_path = mesh_path
        
        if compound:
            # concave mesh for convex decomposition            
            concaveGeom = physicsUtils.create_mesh_concave(self.stage, mesh_path, 10.0)
            concaveGeom.AddTranslateOp().Set(Gf.Vec3f(0.0, 0.0, 100.0))
            UsdPhysics.RigidBodyAPI.Apply(concaveGeom.GetPrim())
            UsdPhysics.CollisionAPI.Apply(concaveGeom.GetPrim())
            meshCollisionAPI = UsdPhysics.MeshCollisionAPI.Apply(concaveGeom.GetPrim())
            meshCollisionAPI.CreateApproximationAttr().Set("convexDecomposition")
        else:
            # concave mesh for convex            
            concaveGeom = physicsUtils.create_mesh_cube(self.stage, mesh_path, 10.0)
            concaveGeom.AddTranslateOp().Set(Gf.Vec3f(0.0, 0.0, 100.0))
            UsdPhysics.RigidBodyAPI.Apply(concaveGeom.GetPrim())
            UsdPhysics.CollisionAPI.Apply(concaveGeom.GetPrim())
            meshCollisionAPI = UsdPhysics.MeshCollisionAPI.Apply(concaveGeom.GetPrim())
            meshCollisionAPI.CreateApproximationAttr().Set("convexHull")
   
        # Add material        
        physicsUtils.add_physics_material_to_prim(stage, stage.GetPrimAtPath(Sdf.Path(mesh_path)), Sdf.Path(materialPath))    

    async def test_material_remove_first_compound_shape(self):
        await self.setup_scene(True)
        
        for _ in range(2):
            self.step()            
        utils.check_stats(self, {"numConvexShapes": 4, "numDynamicRigids": 1})

        self.stage.RemovePrim(self.material_path)

        for _ in range(2):
            self.step()            
            
        self.stage.RemovePrim(self.mesh_path)
        
        for _ in range(2):
            self.step()            
        utils.check_stats(self, {"numConvexShapes": 0, "numDynamicRigids": 0})

    async def test_material_remove_second_compound_shape(self):
        await self.setup_scene(True)
        
        for _ in range(2):
            self.step()            
        utils.check_stats(self, {"numConvexShapes": 4, "numDynamicRigids": 1})
            
        self.stage.RemovePrim(self.mesh_path)

        for _ in range(2):
            self.step()            

        self.stage.RemovePrim(self.material_path)
        
        for _ in range(2):
            self.step()            
        utils.check_stats(self, {"numConvexShapes": 0, "numDynamicRigids": 0})

    async def test_material_remove_first_shape(self):
        await self.setup_scene(False)
        
        for _ in range(2):
            self.step()            
        utils.check_stats(self, {"numConvexShapes": 1, "numDynamicRigids": 1})

        self.stage.RemovePrim(self.material_path)

        for _ in range(2):
            self.step()            
            
        self.stage.RemovePrim(self.mesh_path)
        
        for _ in range(2):
            self.step()            
        utils.check_stats(self, {"numConvexShapes": 0, "numDynamicRigids": 0})

    async def test_material_remove_second_shape(self):
        await self.setup_scene(False)
        
        for _ in range(2):
            self.step()            
        utils.check_stats(self, {"numConvexShapes": 1, "numDynamicRigids": 1})
            
        self.stage.RemovePrim(self.mesh_path)

        for _ in range(2):
            self.step()            

        self.stage.RemovePrim(self.material_path)
        
        for _ in range(2):
            self.step()            
        utils.check_stats(self, {"numConvexShapes": 0, "numDynamicRigids": 0})