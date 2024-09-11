# Copyright (c) 2018-2023, NVIDIA CORPORATION. All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto. Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.
#


from re import I

import carb
import omni.graph.core as og
import omni.graph.core.tests as ogts
import omni.kit.test
from omni.isaac.core.robots import Robot
from omni.isaac.core.utils.physics import simulate_async
from omni.isaac.core.utils.stage import open_stage_async
from omni.isaac.nucleus import get_assets_root_path
from omni.isaac.wheeled_robots.controllers.holonomic_controller import HolonomicController


class TestHolonomicController(omni.kit.test.AsyncTestCase):
    async def setUp(self):
        pass

    # ----------------------------------------------------------------------
    async def tearDown(self):
        pass

    # ----------------------------------------------------------------------

    async def test_holonomic_drive(self):
        wheel_radius = [0.04, 0.04, 0.04]
        wheel_orientations = [[0, 0, 0, 1], [0.866, 0, 0, -0.5], [0.866, 0, 0, 0.5]]
        wheel_positions = [
            [-0.0980432, 0.000636773, -0.050501],
            [0.0493475, -0.084525, -0.050501],
            [0.0495291, 0.0856937, -0.050501],
        ]
        mecanum_angles = [90, 90, 90]
        velocity_command = [1.0, 1.0, 0.1]

        controller = HolonomicController(
            "test_controller", wheel_radius, wheel_positions, wheel_orientations, mecanum_angles
        )
        actions = controller.forward(velocity_command)
        self.assertAlmostEqual(actions.joint_velocities[0], -25.105, delta=0.001)
        self.assertAlmostEqual(actions.joint_velocities[1], 14.3182, delta=0.001)
        self.assertAlmostEqual(actions.joint_velocities[2], -14.5417, delta=0.001)


class TestHolonomicControllerOgn(ogts.OmniGraphTestCase):
    async def setUp(self):
        """Set up  test environment, to be torn down when done"""
        await omni.usd.get_context().new_stage_async()
        self._timeline = omni.timeline.get_timeline_interface()

    # ----------------------------------------------------------------------
    async def tearDown(self):
        """Get rid of temporary data used by the test"""
        await omni.kit.stage_templates.new_stage_async()
        self._timeline = None

    # ----------------------------------------------------------------------
    async def test_holonomic_drive_ogn(self):
        (test_holo_graph, [holo_node, _, _, _, array_node], _, _) = og.Controller.edit(
            {"graph_path": "/ActionGraph"},
            {
                og.Controller.Keys.CREATE_NODES: [
                    ("HolonomicController", "omni.isaac.wheeled_robots.HolonomicController"),
                    ("XVelocity", "omni.graph.nodes.ConstantDouble"),
                    ("YVelocity", "omni.graph.nodes.ConstantDouble"),
                    ("Rotation", "omni.graph.nodes.ConstantDouble"),
                    ("VelocityCommands", "omni.graph.nodes.MakeVector3"),
                ],
                og.Controller.Keys.SET_VALUES: [
                    ("HolonomicController.inputs:wheelRadius", [0.04, 0.04, 0.04]),
                    (
                        "HolonomicController.inputs:wheelPositions",
                        [
                            [-0.0980432, 0.000636773, -0.050501],
                            [0.0493475, -0.084525, -0.050501],
                            [0.0495291, 0.0856937, -0.050501],
                        ],
                    ),
                    (
                        "HolonomicController.inputs:wheelOrientations",
                        [[0, 0, 0, 1], [0.866, 0, 0, -0.5], [0.866, 0, 0, 0.5]],
                    ),
                    ("HolonomicController.inputs:mecanumAngles", [90, 90, 90]),
                    ("XVelocity.inputs:value", 1.0),
                    ("YVelocity.inputs:value", 1.0),
                    ("Rotation.inputs:value", 0.1),
                ],
                og.Controller.Keys.CONNECT: [
                    ("XVelocity.inputs:value", "VelocityCommands.inputs:x"),
                    ("YVelocity.inputs:value", "VelocityCommands.inputs:y"),
                    ("Rotation.inputs:value", "VelocityCommands.inputs:z"),
                    ("VelocityCommands.outputs:tuple", "HolonomicController.inputs:inputVelocity"),
                ],
            },
        )

        await og.Controller.evaluate(test_holo_graph)
        self.assertAlmostEqual(
            og.Controller(og.Controller.attribute("outputs:jointVelocityCommand", holo_node)).get()[0],
            -25.1053,
            delta=0.001,
        )
        self.assertAlmostEqual(
            og.Controller(og.Controller.attribute("outputs:jointVelocityCommand", holo_node)).get()[1],
            14.3182,
            delta=0.001,
        )
        self.assertAlmostEqual(
            og.Controller(og.Controller.attribute("outputs:jointVelocityCommand", holo_node)).get()[2],
            -14.5417,
            delta=0.001,
        )

    # ----------------------------------------------------------------------
    async def test_holonomic_drive_ogn_reset(self):
        (test_holo_graph, [holo_node, _], _, _) = og.Controller.edit(
            {"graph_path": "/ActionGraph"},
            {
                og.Controller.Keys.CREATE_NODES: [
                    ("HolonomicController", "omni.isaac.wheeled_robots.HolonomicController"),
                    ("OnPlaybackTick", "omni.graph.action.OnPlaybackTick"),
                ],
                og.Controller.Keys.SET_VALUES: [
                    ("HolonomicController.inputs:wheelRadius", [0.04, 0.04, 0.04]),
                    (
                        "HolonomicController.inputs:wheelPositions",
                        [
                            [-0.0980432, 0.000636773, -0.050501],
                            [0.0493475, -0.084525, -0.050501],
                            [0.0495291, 0.0856937, -0.050501],
                        ],
                    ),
                    (
                        "HolonomicController.inputs:wheelOrientations",
                        [[0, 0, 0, 1], [0.866, 0, 0, -0.5], [0.866, 0, 0, 0.5]],
                    ),
                    ("HolonomicController.inputs:mecanumAngles", [90, 90, 90]),
                    ("HolonomicController.inputs:inputVelocity", [1.0, 1.0, 0.1]),
                ],
                og.Controller.Keys.CONNECT: [
                    ("OnPlaybackTick.outputs:tick", "HolonomicController.inputs:execIn"),
                ],
            },
        )

        self._timeline.play()
        await omni.kit.app.get_app().next_update_async()

        self.assertAlmostEqual(
            og.Controller(og.Controller.attribute("outputs:jointVelocityCommand", holo_node)).get()[0],
            -25.1053,
            delta=0.01,
        )
        self.assertAlmostEqual(
            og.Controller(og.Controller.attribute("outputs:jointVelocityCommand", holo_node)).get()[1],
            14.3182,
            delta=0.01,
        )
        self.assertAlmostEqual(
            og.Controller(og.Controller.attribute("outputs:jointVelocityCommand", holo_node)).get()[2],
            -14.5417,
            delta=0.01,
        )

        self._timeline.stop()
        await omni.kit.app.get_app().next_update_async()

        self.assertEqual(
            og.Controller(og.Controller.attribute("outputs:jointVelocityCommand", holo_node)).get()[0], 0.0
        )
        self.assertEqual(
            og.Controller(og.Controller.attribute("outputs:jointVelocityCommand", holo_node)).get()[1], 0.0
        )
        self.assertEqual(
            og.Controller(og.Controller.attribute("outputs:jointVelocityCommand", holo_node)).get()[2], 0.0
        )
