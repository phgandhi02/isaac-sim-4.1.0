# Copyright (c) 2018-2023, NVIDIA CORPORATION. All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto. Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.
#

import os

import carb
import omni.graph.core as og
import omni.kit.app
import omni.kit.test
from omni.isaac.core.utils.physics import simulate_async
from omni.isaac.core.utils.stage import add_reference_to_stage, create_new_stage_async
from omni.isaac.nucleus import get_assets_root_path_async
from pxr import Sdf


class TestTransformListener(omni.kit.test.AsyncTestCase):
    # Before running each test
    async def setUp(self):
        await create_new_stage_async()
        self._timeline = omni.timeline.get_timeline_interface()

    # After running each test
    async def tearDown(self):
        self._timeline = None

    # ----------------------------------------------------------------------
    async def test_transform_listener(self):
        # add robot
        assets_root_path = await get_assets_root_path_async()
        if assets_root_path is None:
            carb.log_error("Could not find Isaac Sim assets folder")
            return
        asset_path = assets_root_path + "/Isaac/Robots/Franka/franka_alt_fingers.usd"
        add_reference_to_stage(usd_path=asset_path, prim_path="/World/panda")

        # define graph to publish /tf
        (test_graph, new_nodes, _, _) = og.Controller.edit(
            {"graph_path": "/ActionGraph", "evaluator_name": "execution"},
            {
                og.Controller.Keys.CREATE_NODES: [
                    ("OnPlaybackTick", "omni.graph.action.OnPlaybackTick"),
                    ("PublishTransformTree", "omni.isaac.ros2_bridge.ROS2PublishTransformTree"),
                ],
                og.Controller.Keys.SET_VALUES: [
                    ("PublishTransformTree.inputs:topicName", "tf"),
                    ("PublishTransformTree.inputs:targetPrims", [Sdf.Path("/World/panda")]),
                ],
                og.Controller.Keys.CONNECT: [
                    ("OnPlaybackTick.outputs:tick", "PublishTransformTree.inputs:execIn"),
                ],
            },
        )
        subscriber_node = new_nodes[-1]

        # load plugin
        ext_path = omni.kit.app.get_app().get_extension_manager().get_extension_path_by_module(__name__)
        print(ext_path)
        carb.get_framework().load_plugins(
            loaded_file_wildcards=["omni.isaac.transform_listener.plugin"],
            search_paths=[os.path.abspath(os.path.join(ext_path, "bin"))],
        )

        # load the transform listener
        from .. import _transform_listener as module

        interface = module.acquire_transform_listener_interface()
        interface.initialize(os.environ.get("ROS_DISTRO", "").lower())

        # run the simulation
        self._timeline.play()
        for _ in range(10):
            await omni.kit.app.get_app().next_update_async()
            interface.spin()

        frames, transforms, relations = interface.get_transforms("panda_link0")
        # print(frames, transforms, relations)

        interface.finalize()
        module.release_transform_listener_interface(interface)

        # check frames
        gt_frames = [
            "panda_link0",
            "panda_link1",
            "panda_link2",
            "panda_link3",
            "panda_link4",
            "panda_link5",
            "panda_link6",
            "panda_link7",
            "panda_link8",
            "panda_hand",
            "panda_leftfinger",
            "panda_rightfinger",
        ]
        for frame in gt_frames:
            self.assertIn(frame, frames)

        # check transforms
        self.assertTupleEqual(transforms.get("panda_link0", tuple()), ((0.0, 0.0, 0.0), (0.0, 0.0, 0.0, 1.0)))

        # check relations
        gt_relations = [
            ("panda_link1", "panda_link0"),
            ("panda_link2", "panda_link1"),
            ("panda_link3", "panda_link2"),
            ("panda_link4", "panda_link3"),
            ("panda_link5", "panda_link4"),
            ("panda_link6", "panda_link5"),
            ("panda_link7", "panda_link6"),
            ("panda_link8", "panda_link7"),
            ("panda_hand", "panda_link8"),
            ("panda_leftfinger", "panda_hand"),
            ("panda_rightfinger", "panda_hand"),
        ]
        for relation in gt_relations:
            self.assertIn(relation, relations)
