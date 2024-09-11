# Copyright (c) 2018-2023, NVIDIA CORPORATION. All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto. Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.
#

import asyncio
import os

import carb
import carb.tokens
import numpy as np
import omni.kit.commands

# NOTE:
#   omni.kit.test - std python's unittest module with additional wrapping to add suport for async/await tests
#   For most things refer to unittest docs: https://docs.python.org/3/library/unittest.html
import omni.kit.test
import omni.kit.usd
import omni.physics.tensors
from omni.isaac.core.prims.rigid_prim import RigidPrim
from omni.isaac.core.utils.physics import simulate_async

# Import extension python module we are testing with absolute import path, as if we are external user (other extension)
from omni.isaac.surface_gripper._surface_gripper import Surface_Gripper, Surface_Gripper_Properties
from pxr import Gf, Sdf, Tf, Usd, UsdGeom, UsdLux, UsdPhysics


# Having a test class dervived from omni.kit.test.AsyncTestCase declared on the root of module will make it auto-discoverable by omni.kit.test
class TestSurfaceGripper(omni.kit.test.AsyncTestCase):
    async def createRigidCube(self, boxActorPath, mass, scale, position, rotation, color):
        p = Gf.Vec3f(position[0], position[1], position[2])
        orientation = Gf.Quatf(rotation[3], rotation[0], rotation[1], rotation[2])
        color = Gf.Vec3f(color[0] / 255.0, color[1] / 255.0, color[2] / 255.0)
        size = 1.0
        scale = Gf.Vec3f(scale[0], scale[1], scale[2])

        cubeGeom = UsdGeom.Cube.Define(self.stage, boxActorPath)
        cubePrim = self.stage.GetPrimAtPath(boxActorPath)
        cubeGeom.CreateSizeAttr(size)
        cubeGeom.AddTranslateOp().Set(p)
        cubeGeom.AddOrientOp().Set(orientation)
        cubeGeom.AddScaleOp().Set(scale)
        cubeGeom.CreateDisplayColorAttr().Set([color])
        await omni.kit.app.get_app().next_update_async()
        UsdPhysics.CollisionAPI.Apply(cubePrim)
        rigid_api = UsdPhysics.RigidBodyAPI.Apply(cubePrim)

        if mass > 0:
            massAPI = UsdPhysics.MassAPI.Apply(cubePrim)
            massAPI.CreateMassAttr(mass)
        else:
            rigid_api.CreateKinematicEnabledAttr(True)

        await omni.kit.app.get_app().next_update_async()
        UsdPhysics.CollisionAPI(cubePrim)

    # Helper for setting up the physics stage
    async def setup_physics(self, box1_props):
        # Set Up Physics scene

        UsdGeom.SetStageUpAxis(self.stage, UsdGeom.Tokens.z)
        UsdGeom.SetStageMetersPerUnit(self.stage, 1.0)
        self._scene = UsdPhysics.Scene.Define(self.stage, Sdf.Path("/physicsScene"))
        self._scene.CreateGravityDirectionAttr().Set(Gf.Vec3f(0.0, 0.0, -1.0))
        self._scene.CreateGravityMagnitudeAttr().Set(9.81)

        # Create two cubes and set them to be rigid bodies

        # box0
        await self.createRigidCube(*self.box0_props)
        # Box1
        await self.createRigidCube(*box1_props)

        # d6FixedJoint = UsdPhysics.Joint.Define(self.stage, "/box0/d6FixedJoint")
        # d6FixedJoint.CreateBody0Rel().SetTargets(["/box0"])

    # Before running each test
    async def setUp(self):
        self.box0 = "/box0"
        self.box1 = "/box1"
        self.box0_props = [self.box0, 0.0, [1, 1, 2.0], [-0.50, 0, 1.00], [0, 0, 0, 1], [80, 80, 255]]
        self.box1_props = [self.box1, 1.0, [0.1, 0.1, 0.1], [0.06, 0, 2.04], [0, 0, 0, 1], [255, 80, 80]]
        self.d6FixedJoint = "/box0/d6FixedJoint"

        self.sgp = Surface_Gripper_Properties()
        self.sgp.d6JointPath = self.d6FixedJoint
        self.sgp.parentPath = self.box0
        self.sgp.offset = omni.physics.tensors.Transform()
        self.sgp.offset.p.x = 0.501
        self.sgp.offset.p.z = 1.00
        self.sgp.gripThreshold = 0.02
        self.sgp.forceLimit = 1.0e3
        self.sgp.torqueLimit = 1.0e3
        self.sgp.bendAngle = np.pi / 4
        self.sgp.stiffness = 1.0e4
        self.sgp.damping = 1.0e3

        self.surface_gripper = None

        await omni.usd.get_context().new_stage_async()
        self.stage = omni.usd.get_context().get_stage()
        self._timeline = omni.timeline.get_timeline_interface()

        pass

    # After running each test
    async def tearDown(self):
        # Because lifetime for this joint object is managed by the DC plugin and not usd the order
        # that dc and the gripper are cleaned up matters. First remove the surface gripper and
        # then call stop and then cleanup dc
        self.surface_gripper = None
        # await omni.kit.app.get_app().next_update_async()
        # self._timeline.stop()
        await omni.kit.app.get_app().next_update_async()
        pass

    # Actual test, notice it is "async" function, so "await" can be used if needed
    async def test_create_surface_gripper(self):

        self.surface_gripper = Surface_Gripper()
        assert self.surface_gripper is not None

        pass

    async def test_initialize_surface_gripper(self):

        await self.setup_physics(self.box1_props)
        self.surface_gripper = Surface_Gripper()
        # Start Simulation and wait
        self._timeline.play()
        await simulate_async(0.125)
        await omni.kit.app.get_app().next_update_async()
        self.assertTrue(self.surface_gripper.initialize(self.sgp))
        pass

    async def test_close_surface_gripper(self):

        await self.setup_physics(self.box1_props)
        self.surface_gripper = Surface_Gripper()
        # Start Simulation and wait
        self._timeline.play()

        await simulate_async(0.125)
        await omni.kit.app.get_app().next_update_async()

        self.sgp.forceLimit = 1.0e10
        self.sgp.torqueLimit = 1.0e10
        self.sgp.stiffness = 1.0e10

        self.surface_gripper.initialize(self.sgp)
        box1 = RigidPrim(self.box1)
        box1.set_world_pose(position=self.box1_props[3], orientation=self.box1_props[4])
        box1.set_linear_velocity([0.0, 0.0, 0.0])
        box1.set_angular_velocity([0.0, 0.0, 0.0])

        self.assertTrue(self.surface_gripper.close())
        # use 100 instead of 30, to make sure the joint doesn't break
        for i in range(100):
            await omni.kit.app.get_app().next_update_async()
            self.surface_gripper.update()
        self.assertTrue(self.surface_gripper.is_closed())
        await simulate_async(1.0)
        await omni.kit.app.get_app().next_update_async()
        position, orientation = box1.get_world_pose()
        self.assertGreater(position[2], 2.00)

        # Check to make sure that pause and then play does not break joint
        self._timeline.pause()
        await simulate_async(0.125)
        await omni.kit.app.get_app().next_update_async()
        self._timeline.play()
        await simulate_async(0.5)
        await omni.kit.app.get_app().next_update_async()
        self.surface_gripper.update()
        position, orientation = box1.get_world_pose()
        self.assertGreater(position[2], 2.00)
        self.assertTrue(self.surface_gripper.is_closed())
        pass

    async def test_close_stop_close(self):
        await self.test_close_surface_gripper()
        self._timeline.stop()
        await simulate_async(0.5)
        await omni.kit.app.get_app().next_update_async()

        self._timeline.play()
        await omni.kit.app.get_app().next_update_async()
        box1 = RigidPrim(self.box1)
        box1.set_world_pose(position=self.box1_props[3], orientation=self.box1_props[4])
        box1.set_linear_velocity([0.0, 0.0, 0.0])
        box1.set_angular_velocity([0.0, 0.0, 0.0])

        self.assertTrue(self.surface_gripper.close())
        # use 100 instead of 30, to make sure the joint doesn't break
        for i in range(100):
            await omni.kit.app.get_app().next_update_async()
            self.surface_gripper.update()
        self.assertTrue(self.surface_gripper.is_closed())
        await simulate_async(1.0)
        await omni.kit.app.get_app().next_update_async()
        position, orientation = box1.get_world_pose()
        self.assertGreater(position[2], 2.000)

        # Check to make sure that pause and then play does not break joint
        self._timeline.pause()
        await simulate_async(0.125)
        await omni.kit.app.get_app().next_update_async()
        self._timeline.play()
        await simulate_async(0.5)
        await omni.kit.app.get_app().next_update_async()
        self.surface_gripper.update()
        position, orientation = box1.get_world_pose()
        self.assertGreater(position[2], 2.000)
        self.assertTrue(self.surface_gripper.is_closed())
        pass

    async def test_close_surface_gripper_and_move(self):
        self.box0_props[1] = 3.000
        self.box0_props[2] = [0.1, 0.1, 0.1]
        self.box0_props[3] = [-0.05, 0, 0.05]
        box1_props = [self.box1, 0.0100, [0.1, 0.1, 0.1], [0.05, 0, 0.05], [0, 0, 0, 1], [255, 80, 80]]

        await self.setup_physics(box1_props)

        # Disable gravity for this test
        self._scene.CreateGravityMagnitudeAttr().Set(0.0)

        self.surface_gripper = Surface_Gripper()
        # Start Simulation and wait
        self._timeline.play()

        await simulate_async(0.125)
        await omni.kit.app.get_app().next_update_async()
        self.sgp.offset.p.x = 0.051
        self.sgp.offset.p.z = 0
        self.sgp.gripThreshold = 2
        self.sgp.forceLimit = 1.0e10
        self.sgp.torqueLimit = 1.0e10
        self.sgp.bendAngle = 0
        self.sgp.stiffness = 1.0e10
        self.sgp.damping = 1.0e10

        self.surface_gripper.initialize(self.sgp)
        box0 = RigidPrim(self.box0)
        box1 = RigidPrim(self.box1)
        box1.set_world_pose(position=box1_props[3], orientation=box1_props[4])
        box1.set_linear_velocity([0.0, 0.0, 0.0])
        box1.set_angular_velocity([0.0, 0.0, 0.0])
        self.assertTrue(self.surface_gripper.close())
        for i in range(100):
            await omni.kit.app.get_app().next_update_async()
            box0.set_linear_velocity([0, 0.0, 5.00])
            box0.set_angular_velocity([-20.0, 0.0, 10])
            self.surface_gripper.update()
        self.assertTrue(self.surface_gripper.is_closed())

        v0 = box0.get_linear_velocity()
        v1 = box1.get_angular_velocity()

        # Check if both objects are moving
        self.assertGreater(np.linalg.norm(v0), 0.01)
        self.assertGreater(np.linalg.norm(v1), 0.01)

        pass

    async def test_close_offset_surface_gripper(self):

        self.box0_props[4] = (0, 0, 0.70701, -0.70701)
        self.box1_props = [self.box1, 0.10, [0.1, 0.1, 0.1], [0.06, 0, 2.04], [0, 0, 0, 1], [255, 80, 80]]
        self.sgp.offset.p = (0, 0.501, 1.00)
        self.sgp.offset.r = (0, 0, 0.7071, 0.7071)
        self.sgp.forceLimit = 1.0e10
        self.sgp.torqueLimit = 1.0e10
        self.sgp.stiffness = 1.0e10

        await self.test_close_surface_gripper()
        pass

    async def test_close_out_of_reach_surface_gripper(self):

        box1_props = [self.box1, 0.010, [0.1, 0.1, 0.1], [0.08, 0, 2.04], [0, 0, 0, 1], [255, 80, 80]]

        await self.setup_physics(box1_props)
        self.surface_gripper = Surface_Gripper()
        # Start Simulation and wait
        self._timeline.play()

        await simulate_async(0.125)
        await omni.kit.app.get_app().next_update_async()
        self.surface_gripper.initialize(self.sgp)
        box1 = RigidPrim(self.box1)
        box1.set_world_pose(position=box1_props[3], orientation=box1_props[4])
        box1.set_linear_velocity([0.0, 0.0, 0.0])
        box1.set_angular_velocity([0.0, 0.0, 0.0])

        self.assertFalse(self.surface_gripper.close())
        for i in range(100):
            await omni.kit.app.get_app().next_update_async()
            self.surface_gripper.update()
        self.assertFalse(self.surface_gripper.is_closed())

        position, orientation = box1.get_world_pose()
        self.assertLess(position[2], 2.00)

        pass

    async def test_open_surface_gripper(self):

        await self.test_close_surface_gripper()

        box1 = RigidPrim(self.box1)
        self.surface_gripper.open()
        await omni.kit.app.get_app().next_update_async()
        self.surface_gripper.update()
        await simulate_async(1)
        await omni.kit.app.get_app().next_update_async()
        self.assertFalse(self.surface_gripper.is_closed())
        lin_vel = box1.get_linear_velocity()
        self.assertGreater(np.linalg.norm(lin_vel), 0)
        position, orientation = box1.get_world_pose()
        self.assertLess(position[2], 200)

        # Check to make sure that pause and then play does not close joint
        self._timeline.pause()
        await simulate_async(0.125)
        await omni.kit.app.get_app().next_update_async()
        self._timeline.play()
        await simulate_async(0.5)
        await omni.kit.app.get_app().next_update_async()
        position, orientation = box1.get_world_pose()
        self.assertLess(position[2], 200)
        self.assertFalse(self.surface_gripper.is_closed())
        pass

    async def test_break_surface_gripper(self):

        box1_props = [self.box1, 100, [0.1, 0.1, 0.02], [0.06, 0, 2.005], [0, 0, 0, 1], [255, 80, 80]]

        await self.setup_physics(box1_props)
        self.surface_gripper = Surface_Gripper()
        # Start Simulation and wait
        self._timeline.play()
        await simulate_async(0.125)
        await omni.kit.app.get_app().next_update_async()
        self.surface_gripper.initialize(self.sgp)
        box1 = RigidPrim(self.box1)
        box1.set_world_pose(position=[0.06, 0, 2.005], orientation=box1_props[4])
        box1.set_linear_velocity([0.0, 0.0, 0.0])
        box1.set_angular_velocity([0.0, 0.0, 0.0])

        self.assertTrue(self.surface_gripper.close())
        await omni.kit.app.get_app().next_update_async()

        i = 0
        while i < 600:
            i = i + 1
            await omni.kit.app.get_app().next_update_async()
            self.surface_gripper.update()
            if not self.surface_gripper.is_closed():
                break
        self.assertFalse(self.surface_gripper.is_closed())

        lin_vel = box1.get_linear_velocity()
        self.assertGreater(np.linalg.norm(lin_vel), 0)
        pass

    async def test_bend_surface_gripper(self):
        from omni.isaac.utils._isaac_utils import math as mu

        box1_props = [self.box1, 0.200, [0.1, 0.1, 0.1], [0.06, 0, 2.04], [0, 0, 0, 1], [255, 80, 80]]

        await self.setup_physics(box1_props)
        self.surface_gripper = Surface_Gripper()
        # Start Simulation and wait
        self._timeline.play()
        await simulate_async(0.125)
        await omni.kit.app.get_app().next_update_async()
        self.sgp.forceLimit = 1.0e24
        self.sgp.torqueLimit = 1.0e24
        self.sgp.stiffness = 1.0e-1
        self.sgp.damping = 1.0e-2
        self.surface_gripper.initialize(self.sgp)
        box1 = RigidPrim(self.box1)
        box1.set_world_pose(position=[0.06, 0, 2.04], orientation=box1_props[4])
        box1.set_angular_velocity([0.0, 0.0, 0.0])
        self.assertTrue(self.surface_gripper.close())
        await omni.kit.app.get_app().next_update_async()

        i = 0
        rx1 = (1, 0, 0)
        rx2 = (1, 0, 0)
        while i < 300:
            i += 1
            await omni.kit.app.get_app().next_update_async()
            position, orientation = box1.get_world_pose()
            rx1 = mu.get_basis_vector_x(orientation)
            rx2 = mu.get_basis_vector_x(box1_props[4])
            self.surface_gripper.update()

        self.assertLess(abs(mu.dot(rx2, rx1)), 0.9)
        box1.set_linear_velocity([0.0, 0.0, 0.0])
        while i < 400:
            i += 1
            await omni.kit.app.get_app().next_update_async()
            self.surface_gripper.update()

        self.assertTrue(self.surface_gripper.is_closed())
        self.assertGreater(position[2], 1.90)
        pass

    async def test_fixed_surface_gripper(self):
        from omni.isaac.utils._isaac_utils import math as mu

        box1_props = [self.box1, 0.100, [0.1, 0.1, 0.1], [0.06, 0, 2.04], [0, 0, 0, 1], [255, 80, 80]]

        await self.setup_physics(box1_props)
        self.surface_gripper = Surface_Gripper()
        # Start Simulation and wait
        self._timeline.play()
        await simulate_async(0.125)
        await omni.kit.app.get_app().next_update_async()
        self.sgp.forceLimit = 1.0e24
        self.sgp.torqueLimit = 1.0e24
        self.sgp.stiffness = 1.0
        self.sgp.damping = 1.0
        self.sgp.bendAngle = 0
        self.surface_gripper.initialize(self.sgp)
        box1 = RigidPrim(self.box1)
        box1.set_world_pose(position=[0.06, 0, 2.04], orientation=box1_props[4])
        box1.set_angular_velocity([0.0, 0.0, 0.0])
        self.assertTrue(self.surface_gripper.close())
        await omni.kit.app.get_app().next_update_async()

        i = 0
        rx1 = (1, 0, 0)
        rx2 = (1, 0, 0)
        while i < 300:
            i += 1
            await omni.kit.app.get_app().next_update_async()
            position, orientation = box1.get_world_pose()
            rx1 = mu.get_basis_vector_x(orientation)
            rx2 = mu.get_basis_vector_x(box1_props[4])
            self.surface_gripper.update()

        self.assertGreater(abs(mu.dot(rx2, rx1)), 0.99)
        self.assertTrue(self.surface_gripper.is_closed())
        self.assertGreater(position[2], 2.00)
        pass
