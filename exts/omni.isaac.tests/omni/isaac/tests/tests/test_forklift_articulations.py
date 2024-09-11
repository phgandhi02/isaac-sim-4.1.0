# Copyright (c) 2018-2024, NVIDIA CORPORATION. All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto. Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.
#

import carb
import omni.graph.core as og
import omni.kit.test
import usdrt.Sdf
from omni.isaac.core import World
from omni.isaac.core.prims import RigidPrim
from omni.isaac.core.utils.prims import delete_prim, get_prim_at_path
from omni.isaac.core.utils.stage import open_stage_async
from omni.isaac.nucleus import get_assets_root_path_async


class TestForkliftArticulations(omni.kit.test.AsyncTestCase):
    # Before running each test
    async def setUp(self):
        self._timeline = omni.timeline.get_timeline_interface()

        self._assets_root_path = await get_assets_root_path_async()
        if self._assets_root_path is None:
            carb.log_error("Could not find Isaac Sim assets folder")
            return

        self.usd_path = self._assets_root_path + "/Isaac/Robots/Forklift/forklift_c.usd"
        (result, _) = await open_stage_async(self.usd_path)
        self.stage = omni.usd.get_context().get_stage()

        self.assertTrue(result)
        await omni.kit.app.get_app().next_update_async()

        self.my_world = World(stage_units_in_meters=1.0)
        await self.my_world.initialize_simulation_context_async()

        self.graph_path = "/ActionGraph"

        if get_prim_at_path(self.graph_path):
            delete_prim(self.graph_path)

        keys = og.Controller.Keys
        og.Controller.edit(
            {"graph_path": self.graph_path, "evaluator_name": "execution"},
            {
                keys.CREATE_NODES: [
                    ("OnPlaybackTick", "omni.graph.action.OnPlaybackTick"),
                    ("ArticulationController", "omni.isaac.core_nodes.IsaacArticulationController"),
                    ("WritePrimAttributeLeft", "omni.graph.nodes.WritePrimAttribute"),
                    ("WritePrimAttributeRight", "omni.graph.nodes.WritePrimAttribute"),
                    ("WritePrimAttributeLift", "omni.graph.nodes.WritePrimAttribute"),
                    ("SteeringAngle", "omni.graph.nodes.ConstantDouble"),
                    ("LiftPosition", "omni.graph.nodes.ConstantDouble"),
                ],
                keys.CONNECT: [
                    ("OnPlaybackTick.outputs:tick", "ArticulationController.inputs:execIn"),
                    ("OnPlaybackTick.outputs:tick", "WritePrimAttributeLeft.inputs:execIn"),
                    ("OnPlaybackTick.outputs:tick", "WritePrimAttributeRight.inputs:execIn"),
                    ("OnPlaybackTick.outputs:tick", "WritePrimAttributeLift.inputs:execIn"),
                    ("SteeringAngle.inputs:value", "WritePrimAttributeLeft.inputs:value"),
                    ("SteeringAngle.inputs:value", "WritePrimAttributeRight.inputs:value"),
                    ("LiftPosition.inputs:value", "WritePrimAttributeLift.inputs:value"),
                ],
                keys.SET_VALUES: [
                    ("SteeringAngle.inputs:value", 0.0),
                    ("LiftPosition.inputs:value", 0.0),
                    (
                        "WritePrimAttributeLeft.inputs:prim",
                        [usdrt.Sdf.Path("/SM_Forklift_C01_01/left_rotator_joint")],
                    ),
                    ("WritePrimAttributeLeft.inputs:name", "drive:angular:physics:targetPosition"),
                    (
                        "WritePrimAttributeRight.inputs:prim",
                        [usdrt.Sdf.Path("/SM_Forklift_C01_01/right_rotator_joint")],
                    ),
                    ("WritePrimAttributeRight.inputs:name", "drive:angular:physics:targetPosition"),
                    (
                        "WritePrimAttributeLift.inputs:prim",
                        [usdrt.Sdf.Path("/SM_Forklift_C01_01/lift_joint")],
                    ),
                    ("WritePrimAttributeLift.inputs:name", "drive:linear:physics:targetPosition"),
                    ("ArticulationController.inputs:robotPath", "/SM_Forklift_C01_01"),
                    ("ArticulationController.inputs:velocityCommand", [0.0, 0.0]),
                    (
                        "ArticulationController.inputs:jointNames",
                        [
                            "left_back_wheel_joint",
                            "right_back_wheel_joint",
                        ],
                    ),
                ],
            },
        )

        omni.timeline.get_timeline_interface().set_time_codes_per_second(60)
        pass

    # After running each test
    async def tearDown(self):
        self.my_world.stop()
        await omni.kit.app.get_app().next_update_async()
        while omni.usd.get_context().get_stage_loading_status()[2] > 0:
            await omni.kit.app.get_app().next_update_async()
        pass

    async def test_forklift_forward(self):
        body_prim = RigidPrim("/SM_Forklift_C01_01/body")

        og.Controller.attribute(self.graph_path + "/ArticulationController.inputs:velocityCommand").set([5.0, 5.0])

        # start the timeline
        self.my_world.play()
        await omni.kit.app.get_app().next_update_async()
        pos = body_prim.get_current_dynamic_state().position

        # wait for 200 frames
        for _ in range(200):
            await omni.kit.app.get_app().next_update_async()

        new_pos = body_prim.get_current_dynamic_state().position

        # check if the forklift moved
        self.assertAlmostEqual(pos[0], new_pos[0], delta=1)
        self.assertNotAlmostEqual(pos[1], new_pos[1], delta=1)
        self.assertAlmostEqual(pos[2], new_pos[2], delta=1)

    async def test_forklift_reverse(self):
        body_prim = RigidPrim("/SM_Forklift_C01_01/body")

        og.Controller.attribute(self.graph_path + "/ArticulationController.inputs:velocityCommand").set([-5.0, -5.0])

        # start the timeline
        self.my_world.play()
        await omni.kit.app.get_app().next_update_async()
        pos = body_prim.get_current_dynamic_state().position

        # wait for 200 frames
        for _ in range(200):
            await omni.kit.app.get_app().next_update_async()

        new_pos = body_prim.get_current_dynamic_state().position

        # check if the forklift moved
        self.assertAlmostEqual(pos[0], new_pos[0], delta=1)
        self.assertNotAlmostEqual(pos[1], new_pos[1], delta=1)
        self.assertAlmostEqual(pos[2], new_pos[2], delta=1)

    async def test_forklift_reverse_turn(self):
        body_prim = RigidPrim("/SM_Forklift_C01_01/body")

        og.Controller.attribute(self.graph_path + "/ArticulationController.inputs:velocityCommand").set([-5.0, -5.0])
        og.Controller.attribute(self.graph_path + "/SteeringAngle.inputs:value").set(20.0)

        # start the timeline
        self.my_world.play()
        await omni.kit.app.get_app().next_update_async()
        pos = body_prim.get_current_dynamic_state().position

        # wait for 200 frames
        for _ in range(200):
            await omni.kit.app.get_app().next_update_async()

        new_pos = body_prim.get_current_dynamic_state().position

        # check if the forklift moved
        self.assertNotAlmostEqual(pos[0], new_pos[0], delta=1)
        self.assertNotAlmostEqual(pos[1], new_pos[1], delta=1)
        self.assertAlmostEqual(pos[2], new_pos[2], delta=1)

    async def test_forklift_lift(self):
        lift_prim = RigidPrim("/SM_Forklift_C01_01/lift")

        og.Controller.attribute(self.graph_path + "/LiftPosition.inputs:value").set(1.0)

        # start the timeline
        self.my_world.play()
        await omni.kit.app.get_app().next_update_async()
        pos = lift_prim.get_current_dynamic_state().position

        # wait for 60 frames
        for _ in range(60):
            await omni.kit.app.get_app().next_update_async()

        new_pos = lift_prim.get_current_dynamic_state().position

        # check if the forklift moved
        self.assertAlmostEqual(pos[0], new_pos[0], delta=1)
        self.assertAlmostEqual(pos[1], new_pos[1], delta=1)
        self.assertNotAlmostEqual(pos[2], new_pos[2], delta=0.5)
