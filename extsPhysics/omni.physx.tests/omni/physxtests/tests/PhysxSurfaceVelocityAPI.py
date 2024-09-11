import omni.physx
import omni.physx.scripts.utils as physicsBaseUtils
import omni.physx.scripts.physicsUtils as physicsUtils
from omni.physxtests.utils.physicsBase import PhysicsMemoryStageBaseAsyncTestCase, PhysicsKitStageAsyncTestCase, TestCategory
from omni.physx import get_physx_interface, get_physx_simulation_interface
from omni.physxtests import utils
from pxr import Sdf, Usd, Gf, UsdGeom, UsdPhysics, UsdUtils, PhysxSchema, Vt, PhysicsSchemaTools
import math
import carb
import os
import unittest


class PhysicsSurfaceVelocityTestMemoryStage(PhysicsMemoryStageBaseAsyncTestCase):
    category = TestCategory.Core

    def setup_stage(self, stage, scenario, linear):
        size = 100.0

        # Physics scene
        scene = UsdPhysics.Scene.Define(stage, "/physicsScene")

        # Kinematic
        boxActorPath = "/kinematicActor"
        cubeGeom = UsdGeom.Cube.Define(stage, boxActorPath)
        kinematicCubePrim = stage.GetPrimAtPath(boxActorPath)
        cubeGeom.CreateSizeAttr(size)
        cubeGeom.AddScaleOp().Set(Gf.Vec3f(1.0,1.0,20.0)) 

        UsdPhysics.CollisionAPI.Apply(kinematicCubePrim)
        self.physicsAPI = UsdPhysics.RigidBodyAPI.Apply(kinematicCubePrim)
        self.physicsAPI.CreateKinematicEnabledAttr().Set(True)       

        # target velocities 
        targetSurfaceVelocity = Gf.Vec3f(0.0, 0.0, 80.0)
        targetAngularSurfaceVelocity = Gf.Vec3f(0.0, 200.0, 0.0)

        # dynamic actor
        size = 50.0
        boxActorPath = "/boxActor"
        
        position = Gf.Vec3f(0.0, 8.0, 0.0)

        cubeGeom = UsdGeom.Cube.Define(stage, boxActorPath)
        cubePrim = stage.GetPrimAtPath(boxActorPath)
        cubeGeom.CreateSizeAttr(size)
        cubeGeom.AddTranslateOp().Set(position)
        UsdPhysics.CollisionAPI.Apply(cubePrim)
        self.rigidBodyAPI = UsdPhysics.RigidBodyAPI.Apply(cubePrim)

        if scenario == "kinematic_legacy":
            # kinematic box below with velocity set
            if linear:
                self.physicsAPI.GetVelocityAttr().Set(targetSurfaceVelocity)
            else:
                self.physicsAPI.GetAngularVelocityAttr().Set(targetAngularSurfaceVelocity)
        elif scenario == "kinematic":
            self.surfaceVelocityAPI = PhysxSchema.PhysxSurfaceVelocityAPI.Apply(kinematicCubePrim)
            if linear:
                self.surfaceVelocityAPI.GetSurfaceVelocityAttr().Set(targetSurfaceVelocity)
            else:
                self.surfaceVelocityAPI.GetSurfaceAngularVelocityAttr().Set(targetAngularSurfaceVelocity)
        elif scenario == "dynamic":
            self.surfaceVelocityAPI = PhysxSchema.PhysxSurfaceVelocityAPI.Apply(cubePrim)
            self.surfaceVelocityAPI.GetSurfaceVelocityLocalSpaceAttr().Set(False)
            if linear:
                self.surfaceVelocityAPI.GetSurfaceVelocityAttr().Set(targetSurfaceVelocity)
            else:
                self.surfaceVelocityAPI.GetSurfaceAngularVelocityAttr().Set(targetAngularSurfaceVelocity)

    async def test_physics_kinematic_surface_velocity_legacy(self):
        stage = await self.new_stage()

        self.setup_stage(stage,"kinematic_legacy", True)

        for _ in range(50):
            self.step()
            
        velocity = self.rigidBodyAPI.GetVelocityAttr().Get()
        print(velocity)
        self.assertTrue(velocity[2] > 40.0)

    async def test_physics_kinematic_surface_cylinder_angular_velocity_legacy(self):
        stage = await self.new_stage()
        # Building a scene with a flat disk (made out of a scaled cylinder) and a cube that rotates around its perimeter
        # We check that velocity goes in one direction and after some time it reverses (as it's after turning point)
        scene = UsdPhysics.Scene.Define(stage, "/physicsScene")

        cylinderActorPath = "/kinematicActor"
        cylinderGeom = UsdGeom.Cylinder.Define(stage, cylinderActorPath)
        cylinderGeom.GetAxisAttr().Set("Y") 
        cylinderPrim = stage.GetPrimAtPath(cylinderActorPath)
        cylinderGeom.AddScaleOp().Set(Gf.Vec3f(20.0,1.0,20.0)) 

        UsdPhysics.CollisionAPI.Apply(cylinderPrim)
        physicsAPI = UsdPhysics.RigidBodyAPI.Apply(cylinderPrim)
        physicsAPI.CreateKinematicEnabledAttr().Set(True)
        # Huge angular velocity to avoid needing doing too many steps in the test
        targetAngularSurfaceVelocity = Gf.Vec3f(0.0, 200.0, 0.0)
        physicsAPI.GetAngularVelocityAttr().Set(targetAngularSurfaceVelocity)

        # dynamic actor
        size = 1.0
        boxActorPath = "/boxActor"

        position = Gf.Vec3f(15.0, 2.0, 0.0)

        cubeGeom = UsdGeom.Cube.Define(stage, boxActorPath)
        cubePrim = stage.GetPrimAtPath(boxActorPath)
        cubeGeom.CreateSizeAttr(size)
        cubeGeom.AddTranslateOp().Set(position)
        UsdPhysics.CollisionAPI.Apply(cubePrim)
        rigidBodyAPI = UsdPhysics.RigidBodyAPI.Apply(cubePrim)

        for _ in range(10):
            self.step()
            
        velocity = rigidBodyAPI.GetVelocityAttr().Get()
        print(velocity)
        # Velocity is along negative Z
        self.assertTrue(velocity[2] < -20)

        for _ in range(30):
            self.step()
        velocity = rigidBodyAPI.GetVelocityAttr().Get()
        # Velocity turned direction to positive Z
        self.assertTrue(velocity[2] > 20)
        print(velocity)

    async def test_physics_kinematic_surface_angular_velocity_legacy(self):
        stage = await self.new_stage()

        self.setup_stage(stage,"kinematic_legacy", False)

        for _ in range(10):
            self.step()
            
        angular_velocity = self.rigidBodyAPI.GetAngularVelocityAttr().Get()
        print(angular_velocity)
        self.assertTrue(angular_velocity[1] > 180.0)

    async def test_physics_kinematic_surface_switch_velocity_legacy(self):
        stage = await self.new_stage()
        
        self.setup_stage(stage,"kinematic_legacy", True)

        self.physicsAPI.GetVelocityAttr().Set(Gf.Vec3f(0.0))

        for _ in range(10):
            self.step()

        velocity = self.rigidBodyAPI.GetVelocityAttr().Get()
        print(velocity)
        self.assertTrue(velocity[2] < 40.0)

        targetSurfaceVelocity = Gf.Vec3f(0.0, 0.0, 80.0)
        self.physicsAPI.GetVelocityAttr().Set(targetSurfaceVelocity)

        for _ in range(10):
            self.step()
            
        velocity = self.rigidBodyAPI.GetVelocityAttr().Get()
        print(velocity)
        self.assertTrue(velocity[2] > 40.0)
        
    async def test_physics_kinematic_surface_velocity(self):
        stage = await self.new_stage()

        self.setup_stage(stage,"kinematic", True)

        for _ in range(50):
            self.step()
            
        velocity = self.rigidBodyAPI.GetVelocityAttr().Get()
        print(velocity)
        self.assertTrue(velocity[2] > 40.0)

    async def test_physics_kinematic_surface_angular_velocity(self):
        stage = await self.new_stage()

        self.setup_stage(stage,"kinematic", False)

        for _ in range(10):
            self.step()
            
        angular_velocity = self.rigidBodyAPI.GetAngularVelocityAttr().Get()
        print(angular_velocity)
        self.assertTrue(angular_velocity[1] > 180.0)

    async def test_physics_kinematic_surface_switch_velocity(self):
        stage = await self.new_stage()
        
        self.setup_stage(stage,"kinematic", True)

        self.surfaceVelocityAPI.GetSurfaceVelocityAttr().Set(Gf.Vec3f(0.0))

        for _ in range(10):
            self.step()

        velocity = self.rigidBodyAPI.GetVelocityAttr().Get()
        print(velocity)
        self.assertTrue(velocity[2] < 40.0)

        targetSurfaceVelocity = Gf.Vec3f(0.0, 0.0, 80.0)
        self.surfaceVelocityAPI.GetSurfaceVelocityAttr().Set(targetSurfaceVelocity)

        for _ in range(10):
            self.step()
            
        velocity = self.rigidBodyAPI.GetVelocityAttr().Get()
        print(velocity)
        self.assertTrue(velocity[2] > 40.0)


    async def test_physics_kinematic_surface_switch_angular_velocity(self):
        stage = await self.new_stage()
        
        self.setup_stage(stage,"kinematic", False)

        for _ in range(10):
            self.step()

        velocity = self.rigidBodyAPI.GetAngularVelocityAttr().Get()
        print(velocity)
        self.assertTrue(velocity[1] < 201 and velocity[1] > 190)

        targetAngularSurfaceVelocity = Gf.Vec3f(0.0, 250.0, 0.0)
        self.surfaceVelocityAPI.GetSurfaceAngularVelocityAttr().Set(targetAngularSurfaceVelocity)

        for _ in range(10):
            self.step()
            
        velocity = self.rigidBodyAPI.GetAngularVelocityAttr().Get()
        print(velocity)
        self.assertTrue(velocity[1] > 240 and velocity[1] < 251)


    async def test_physics_kinematic_surface_enable_velocity(self):
        stage = await self.new_stage()
        
        self.setup_stage(stage,"kinematic", True)

        self.surfaceVelocityAPI.GetSurfaceVelocityEnabledAttr().Set(False)

        for _ in range(10):
            self.step()

        velocity = self.rigidBodyAPI.GetVelocityAttr().Get()
        print(velocity)
        self.assertTrue(velocity[2] < 40.0)

        self.surfaceVelocityAPI.GetSurfaceVelocityEnabledAttr().Set(True)

        for _ in range(10):
            self.step()
            
        velocity = self.rigidBodyAPI.GetVelocityAttr().Get()
        print(velocity)
        self.assertTrue(velocity[2] > 40.0)

    async def test_physics_dynamic_surface_velocity(self):
        stage = await self.new_stage()

        self.setup_stage(stage,"dynamic", True)

        for _ in range(50):
            self.step()
            
        velocity = self.rigidBodyAPI.GetVelocityAttr().Get()
        print(velocity)
        self.assertTrue(velocity[2] > 40.0)

    async def test_physics_dynamic_surface_angular_velocity(self):
        stage = await self.new_stage()

        self.setup_stage(stage,"dynamic", False)

        for _ in range(10):
            self.step()
            
        angular_velocity = self.rigidBodyAPI.GetAngularVelocityAttr().Get()
        print(angular_velocity)
        self.assertTrue(angular_velocity[1] > 180.0)

    async def test_physics_dynamic_surface_switch_velocity(self):
        stage = await self.new_stage()
        
        self.setup_stage(stage,"dynamic", True)

        self.surfaceVelocityAPI.GetSurfaceVelocityAttr().Set(Gf.Vec3f(0.0))

        for _ in range(10):
            self.step()

        velocity = self.rigidBodyAPI.GetVelocityAttr().Get()
        print(velocity)
        self.assertTrue(velocity[2] < 40.0)

        targetSurfaceVelocity = Gf.Vec3f(0.0, 0.0, 80.0)
        self.surfaceVelocityAPI.GetSurfaceVelocityAttr().Set(targetSurfaceVelocity)

        for _ in range(10):
            self.step()
            
        velocity = self.rigidBodyAPI.GetVelocityAttr().Get()
        print(velocity)
        self.assertTrue(velocity[2] > 40.0)


    async def test_physics_dynamic_surface_switch_angular_velocity(self):
        stage = await self.new_stage()
        
        self.setup_stage(stage,"dynamic", False)

        for _ in range(10):
            self.step()

        velocity = self.rigidBodyAPI.GetAngularVelocityAttr().Get()
        print(velocity)
        self.assertTrue(velocity[1] < 201 and velocity[1] > 190)

        targetAngularSurfaceVelocity = Gf.Vec3f(0.0, 250.0, 0.0)
        self.surfaceVelocityAPI.GetSurfaceAngularVelocityAttr().Set(targetAngularSurfaceVelocity)

        for _ in range(10):
            self.step()
            
        velocity = self.rigidBodyAPI.GetAngularVelocityAttr().Get()
        print(velocity)
        self.assertTrue(velocity[1] > 240 and velocity[1] < 251)


    async def test_physics_dynamic_surface_enable_velocity(self):
        stage = await self.new_stage()
        
        self.setup_stage(stage,"dynamic", True)

        self.surfaceVelocityAPI.GetSurfaceVelocityEnabledAttr().Set(False)

        for _ in range(10):
            self.step()

        velocity = self.rigidBodyAPI.GetVelocityAttr().Get()
        print(velocity)
        self.assertTrue(velocity[2] < 40.0)

        self.surfaceVelocityAPI.GetSurfaceVelocityEnabledAttr().Set(True)

        for _ in range(10):
            self.step()
            
        velocity = self.rigidBodyAPI.GetVelocityAttr().Get()
        print(velocity)
        self.assertTrue(velocity[2] > 40.0)        