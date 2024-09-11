# Copyright (c) 2021-2023, NVIDIA CORPORATION. All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto. Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.
#

import asyncio
import os
from typing import List

import carb
import numpy as np
import omni.kit.test
from omni.isaac.core.articulations import Articulation
from omni.isaac.core.prims.xform_prim import XFormPrim
from omni.isaac.core.utils.prims import get_prim_at_path
from omni.isaac.core.utils.stage import (
    add_reference_to_stage,
    create_new_stage_async,
    get_current_stage,
    update_stage_async,
)
from omni.isaac.core.utils.types import ArticulationAction
from omni.isaac.core.world import World
from omni.isaac.nucleus import get_assets_root_path_async
from omni.isaac.robot_assembler import AssembledRobot, RobotAssembler
from pxr import PhysxSchema, Sdf, UsdLux, UsdPhysics


# Having a test class derived from omni.kit.test.AsyncTestCase declared on the root of module will
# make it auto-discoverable by omni.kit.test
class TestRobotAssembler(omni.kit.test.AsyncTestCase):
    # Before running each test
    async def setUp(self):
        self._physics_fps = 60
        self._physics_dt = 1 / self._physics_fps  # duration of physics frame in seconds

        self._timeline = omni.timeline.get_timeline_interface()

        carb.settings.get_settings().set_bool("/app/runLoops/main/rateLimitEnabled", True)
        carb.settings.get_settings().set_int("/app/runLoops/main/rateLimitFrequency", self._physics_fps)
        carb.settings.get_settings().set_int("/persistent/simulation/minFrameRate", self._physics_fps)
        omni.timeline.get_timeline_interface().set_target_framerate(self._physics_fps)

        ext_manager = omni.kit.app.get_app().get_extension_manager()
        ext_id = ext_manager.get_enabled_extension_id("omni.isaac.robot_assembler")
        self._robot_assembler_data_path = os.path.join(ext_manager.get_extension_path(ext_id), "data", "test_assets")

        await create_new_stage_async()
        omni.usd.get_context().get_stage().SetTimeCodesPerSecond(self._physics_fps)

        await update_stage_async()

        pass

    # After running each test
    async def tearDown(self):
        self._timeline.stop()
        while omni.usd.get_context().get_stage_loading_status()[2] > 0:
            print("tearDown, assets still loading, waiting to finish...")
            await asyncio.sleep(1.0)
        await update_stage_async()
        World.clear_instance()
        pass

    async def _create_light(self):
        sphereLight = UsdLux.SphereLight.Define(get_current_stage(), Sdf.Path("/World/SphereLight"))
        sphereLight.CreateRadiusAttr(2)
        sphereLight.CreateIntensityAttr(100000)
        XFormPrim(sphereLight.GetPath().pathString).set_world_pose([6.5, 0, 12])

    async def _prepare_stage(self, robots: List[Articulation]):
        # Set settings to ensure deterministic behavior
        # Initialize the robot
        # Play the timeline

        self._timeline.stop()

        world = World()

        await world.initialize_simulation_context_async()
        await self._create_light()

        self._timeline.play()
        await update_stage_async()

        for robot in robots:
            robot.initialize()
            robot.disable_gravity()
            robot.set_solver_position_iteration_count(64)
            robot.set_solver_velocity_iteration_count(64)

        await update_stage_async()

    async def _assert_assembled(self, base_prim, attach_prim):
        # Assert that the attach prim moves when the base prim moves
        # Move the prim somewhat smoothly across the stage to avoid physics errors

        base_prim_xform = XFormPrim(base_prim)
        attach_prim_xform = XFormPrim(attach_prim)

        base_position = base_prim_xform.get_world_pose()[0]
        attach_position = attach_prim_xform.get_world_pose()[0]

        offset = np.array([1, -0.5, 1])
        for off in np.linspace(base_position, base_position + offset, num=20, endpoint=True):
            base_prim_xform.set_world_pose(off)
            await update_stage_async()
        for i in range(5):
            await update_stage_async()

        self.assertTrue(
            np.allclose(base_prim_xform.get_world_pose()[0] - base_position, offset, atol=0.01),
            str(base_prim_xform.get_world_pose()[0] - base_position),
        )
        self.assertTrue(
            np.allclose(attach_prim_xform.get_world_pose()[0] - attach_position, offset, atol=0.05),
            str(attach_prim_xform.get_world_pose()[0] - attach_position) + str(offset),
        )

        for off in np.linspace(base_position + offset, base_position, num=20, endpoint=True):
            base_prim_xform.set_world_pose(off)
            await update_stage_async()
        for i in range(5):
            await update_stage_async()

        self.assertTrue(
            np.allclose(attach_prim_xform.get_world_pose()[0], attach_position, atol=0.05),
            str(attach_prim_xform.get_world_pose()[0]) + str(attach_position),
        )

    async def _assert_not_assembled(self, base_prim, attach_prim):
        # Assert that the two prims do not move together

        base_prim_xform = XFormPrim(base_prim)
        attach_prim_xform = XFormPrim(attach_prim)

        base_position = base_prim_xform.get_world_pose()[0]
        attach_position = attach_prim_xform.get_world_pose()[0]

        offset = np.array([1, -0.5, 1])
        base_prim_xform.set_world_pose(base_position + offset)
        for i in range(10):
            await update_stage_async()

        self.assertTrue(
            np.allclose(attach_prim_xform.get_world_pose()[0], attach_position, atol=0.001),
            f"{attach_prim_xform.get_world_pose()[0]} {attach_position}",
        )

        base_prim_xform.set_world_pose(base_position)
        await update_stage_async()

    async def _wait_n_frames(self, n):
        for i in range(n):
            await update_stage_async()

    async def testRobotToRobotAssembleTopLevelRoots(self):
        await self._testRobotToRobotAssemble("/World/ur10e", "/World/allegro_hand")

    async def testRobotToRobotAssembleTopLevelToRootJoint(self):
        await self._testRobotToRobotAssemble("/World/ur10e", "/World/allegro_hand/root_joint")

    async def testRobotToRobotAssembleRootJointToTopLevel(self):
        await self._testRobotToRobotAssemble("/World/ur10e/root_joint", "/World/allegro_hand")

    async def testRobotToRobotAssembleRootJointToRootJoint(self):
        await self._testRobotToRobotAssemble("/World/ur10e/root_joint", "/World/allegro_hand/root_joint")

    async def testRobotToRobotAssembleTopLevelToBaseLink(self):
        # This will fail disassemble because the allegro root joint disagrees with its
        # final position and there is a resulting physics constraint violation.  The test is hard-coded
        # to skip the disassembly.
        await self._testRobotToRobotAssemble("/World/ur10e", "/World/allegro_hand/allegro_mount")

    async def testRobotToRobotAssembleRootJointToBaseLink(self):
        # This will fail disassemble because the allegro root joint disagrees with its
        # final position and there is a resulting physics constraint violation.  The test is hard-coded
        # to skip the disassembly.
        await self._testRobotToRobotAssemble("/World/ur10e/root_joint", "/World/allegro_hand/allegro_mount")

    async def _testRobotToRobotAssemble(self, base_art_root, attach_art_root):
        assets_root_path = await get_assets_root_path_async()

        add_reference_to_stage(assets_root_path + "/Isaac/Robots/AllegroHand/allegro_hand.usd", "/World/allegro_hand")
        XFormPrim("/World/allegro_hand").set_world_pose(np.array([1.0, 0.0, 0.0]))

        add_reference_to_stage(assets_root_path + "/Isaac/Robots/UniversalRobots/ur10e/ur10e.usd", "/World/ur10e")
        XFormPrim("/World/ur10e").set_world_pose(np.array([-1.0, 0.0, 0.0]))

        # Move the Articulation root to different places in order to test that each location is handled correctly
        RobotAssembler.move_articulation_root(get_prim_at_path("/World/ur10e"), get_prim_at_path(base_art_root))
        RobotAssembler.move_articulation_root(
            get_prim_at_path("/World/allegro_hand"), get_prim_at_path(attach_art_root)
        )

        await update_stage_async()
        await update_stage_async()

        for single_robot in [False, True]:
            base_robot_path = "/World/ur10e"
            attach_robot_path = "/World/allegro_hand"
            base_robot_mount_frame = "/tool0"
            attach_robot_mount_frame = "/allegro_mount"
            fixed_joint_offset = np.array([0.0, 0.0, -0.1])
            fixed_joint_orient = np.array([0.956, 0.0, -0.0, 0.2935])

            robot_assembler = RobotAssembler()
            assembled_robot = robot_assembler.assemble_articulations(
                base_robot_path,
                attach_robot_path,
                base_robot_mount_frame,
                attach_robot_mount_frame,
                fixed_joint_offset,
                fixed_joint_orient,
                mask_all_collisions=True,
                single_robot=single_robot,
            )

            assembled_robot.set_fixed_joint_transform(np.array([0.0, 0.0, -0.15]), np.array([0.956, 0.0, -0.0, 0.2935]))

            num_frame_to_settle = 30

            # Controlling the resulting assembled robot is different depending on the single_robot flag
            if single_robot:
                # The robots will be considered to be part of a single Articulation at the base robot path
                controllable_single_robot = Articulation(base_robot_path)
                await self._prepare_stage([controllable_single_robot])

                joint_target = 0.3 * np.ones(22)
                controllable_single_robot.apply_action(ArticulationAction(joint_target))

                await self._wait_n_frames(num_frame_to_settle)

                self.assertTrue(
                    np.all(abs(controllable_single_robot.get_joint_positions() - joint_target) < 0.01),
                    controllable_single_robot.get_joint_positions(),
                )

            else:
                # The robots are controlled independently from each other
                base_robot = Articulation(base_robot_path)
                attach_robot = Articulation(attach_robot_path)
                await self._prepare_stage([base_robot, attach_robot])

                base_robot_joint_target = np.array([0.3, 0, 0, 0, 0, 0])
                base_robot.apply_action(ArticulationAction(base_robot_joint_target))
                hand_joint_target = np.zeros(16)
                hand_joint_target[3] = 1
                attach_robot.apply_action(ArticulationAction(hand_joint_target))

                await self._wait_n_frames(num_frame_to_settle)
                self.assertTrue(np.all(abs(base_robot.get_joint_positions() - base_robot_joint_target) < 0.01))
                self.assertTrue(np.all(abs(attach_robot.get_joint_positions() - hand_joint_target) < 0.01))

            attach_robot_mount_path = attach_robot_path + attach_robot_mount_frame

            await self._assert_assembled(base_robot_path, attach_robot_mount_path)

            if attach_art_root == "/World/allegro_hand/allegro_mount":
                # The allegro will explode after disassembling because of a fixed joint constraint violation.
                # There is nothing within the scope of the robot assembler to fix.
                continue

            assembled_robot.disassemble()
            await self._assert_not_assembled(base_robot_path, attach_robot_mount_path)

            await update_stage_async()

            # Make sure that the Articulation Root APIs were put back on disassemble()
            base_art_prim = get_prim_at_path(base_art_root)
            self.assertTrue(base_art_prim.HasAPI(UsdPhysics.ArticulationRootAPI))
            self.assertTrue(base_art_prim.HasAPI(PhysxSchema.PhysxArticulationAPI))
            self.assertTrue(base_art_prim.GetProperty("physxArticulation:articulationEnabled").IsValid())

            attach_art_prim = get_prim_at_path(attach_art_root)
            self.assertTrue(attach_art_prim.HasAPI(UsdPhysics.ArticulationRootAPI))
            self.assertTrue(attach_art_prim.HasAPI(PhysxSchema.PhysxArticulationAPI))
            self.assertTrue(attach_art_prim.GetProperty("physxArticulation:articulationEnabled").IsValid())

    async def testRobotToRigidBodyAssemble(self):
        assets_root_path = await get_assets_root_path_async()

        add_reference_to_stage(assets_root_path + "/Isaac/Props/Mounts/ur10_mount.usd", "/World/ur10_mount")
        XFormPrim("/World/ur10_mount").set_world_pose(np.array([-1.0, 0.0, 0.0]))

        add_reference_to_stage(assets_root_path + "/Isaac/Robots/UniversalRobots/ur10e/ur10e.usd", "/World/ur10e")
        XFormPrim("/World/ur10e").set_world_pose(np.array([1.0, 0.0, 0.0]))

        await update_stage_async()

        robot_assembler = RobotAssembler()
        base_robot_path = "/World/ur10_mount"
        robot_assembler.convert_prim_to_rigid_body(base_robot_path)
        attach_robot_path = "/World/ur10e"
        base_robot_mount_frame = ""
        attach_robot_mount_frame = "/base_link"
        fixed_joint_offset = np.array([0.0, 0.0, 0.0])
        fixed_joint_orient = np.array([1.0, 0.0, 0.0, 0.0])

        assembled_bodies = robot_assembler.assemble_rigid_bodies(
            base_robot_path,
            attach_robot_path,
            base_robot_mount_frame,
            attach_robot_mount_frame,
            fixed_joint_offset,
            fixed_joint_orient,
            mask_all_collisions=True,
        )

        offset, orient = assembled_bodies.get_fixed_joint_transform()
        assembled_bodies.set_fixed_joint_transform(np.array([0, 0, -0.2]), np.array([1, 0, 0, 0]))

        await self._prepare_stage([Articulation(attach_robot_path)])

        attach_robot_mount_path = attach_robot_path + attach_robot_mount_frame

        await self._assert_assembled(base_robot_path, attach_robot_mount_path)
        assembled_bodies.disassemble()
        await self._assert_not_assembled(base_robot_path, attach_robot_mount_path)

        # Switch the order of base and robot
        assembled_bodies = robot_assembler.assemble_rigid_bodies(
            attach_robot_path,
            base_robot_path,
            attach_robot_mount_frame,
            base_robot_mount_frame,
            fixed_joint_offset,
            fixed_joint_orient,
            mask_all_collisions=True,
        )

        await self._prepare_stage([Articulation(attach_robot_path)])

        await self._assert_assembled(attach_robot_path, base_robot_path + "/assembler_mount_frame")
        assembled_bodies.disassemble()
        await self._assert_not_assembled(attach_robot_path, base_robot_path + "/assembler_mount_frame")

    async def testRigidBodyToRigidBodyAssemble(self):
        assets_root_path = await get_assets_root_path_async()

        await self._create_light()

        add_reference_to_stage(assets_root_path + "/Isaac/Props/Mounts/ur10_mount.usd", "/World/ur10_mount")
        XFormPrim("/World/ur10_mount").set_world_pose(np.array([-1.0, 0.0, 0.0]))

        add_reference_to_stage(assets_root_path + "/Isaac/Props/Mounts/ur10_mount.usd", "/World/ur10_mount_01")
        XFormPrim("/World/ur10_mount_01").set_world_pose(np.array([1.0, 0.0, 0.0]))
        await update_stage_async()

        robot_assembler = RobotAssembler()
        base_robot_path = "/World/ur10_mount"
        robot_assembler.convert_prim_to_rigid_body(base_robot_path)
        attach_robot_path = "/World/ur10_mount_01"
        robot_assembler.convert_prim_to_rigid_body(attach_robot_path)
        base_robot_mount_frame = ""
        attach_robot_mount_frame = ""
        fixed_joint_offset = np.array([0.0, 0.0, 0.522])
        fixed_joint_orient = np.array([1.0, 0.0, 0.0, 0.0])
        assembled_bodies = robot_assembler.assemble_rigid_bodies(
            base_robot_path,
            attach_robot_path,
            base_robot_mount_frame,
            attach_robot_mount_frame,
            fixed_joint_offset,
            fixed_joint_orient,
            mask_all_collisions=True,
        )
        self._timeline.play()

        await self._assert_assembled(base_robot_path, attach_robot_path)
        assembled_bodies.disassemble()

        await self._assert_not_assembled(base_robot_path, attach_robot_path)
