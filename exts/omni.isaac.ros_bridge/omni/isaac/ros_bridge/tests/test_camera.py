# Copyright (c) 2018-2024, NVIDIA CORPORATION. All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto. Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.
#

import asyncio
import gc
import math

import carb
import omni.graph.core as og

# Import extension python module we are testing with absolute import path, as if we are external user (other extension)
import omni.kit.commands

# NOTE:
#   omni.kit.test - std python's unittest module with additional wrapping to add support for async/await tests
#   For most things refer to unittest docs: https://docs.python.org/3/library/unittest.html
import omni.kit.test
import omni.kit.usd
import omni.kit.viewport.utility
from omni.isaac.core.objects import VisualCuboid
from omni.isaac.core.utils.physics import simulate_async
from omni.isaac.core.utils.semantics import add_update_semantics
from omni.isaac.core.utils.stage import open_stage_async
from omni.isaac.core.utils.viewports import set_camera_view
from omni.isaac.nucleus import get_assets_root_path_async
from pxr import Gf, Sdf

from .common import add_carter_ros, add_cube, wait_for_rosmaster_async


# Having a test class derived from omni.kit.test.AsyncTestCase declared on the root of module will make it auto-discoverable by omni.kit.test
class TestRosCamera(omni.kit.test.AsyncTestCase):
    # Before running each test
    async def setUp(self):
        import rospy
        from omni.isaac.ros_bridge.scripts.roscore import Roscore

        omni.usd.get_context().new_stage()
        self._timeline = omni.timeline.get_timeline_interface()

        ext_manager = omni.kit.app.get_app().get_extension_manager()
        ext_id = ext_manager.get_enabled_extension_id("omni.isaac.ros_bridge")
        self._ros_extension_path = ext_manager.get_extension_path(ext_id)

        self._assets_root_path = await get_assets_root_path_async()
        if self._assets_root_path is None:
            carb.log_error("Could not find Isaac Sim assets folder")
            return
        kit_folder = carb.tokens.get_tokens_interface().resolve("${kit}")

        self._physics_rate = 60
        carb.settings.get_settings().set_bool("/app/runLoops/main/rateLimitEnabled", True)
        carb.settings.get_settings().set_int("/app/runLoops/main/rateLimitFrequency", int(self._physics_rate))
        carb.settings.get_settings().set_int("/persistent/simulation/minFrameRate", int(self._physics_rate))
        await omni.kit.app.get_app().next_update_async()

        self._roscore = Roscore()
        await wait_for_rosmaster_async()
        await omni.kit.app.get_app().next_update_async()

        try:
            rospy.init_node("isaac_sim_test_rospy", anonymous=True, disable_signals=True, log_level=rospy.ERROR)
        except rospy.exceptions.ROSException as e:
            print("Node has already been initialized, do nothing")

        # acquire the viewport window
        viewport_api = omni.kit.viewport.utility.get_active_viewport()
        # Set viewport resolution, changes will occur on next frame
        viewport_api.set_texture_resolution((1280, 720))
        await omni.kit.app.get_app().next_update_async()

        pass

    # After running each test
    async def tearDown(self):
        self._timeline.stop()
        await omni.kit.app.get_app().next_update_async()
        while omni.usd.get_context().get_stage_loading_status()[2] > 0:
            print("tearDown, assets still loading, waiting to finish...")
            await asyncio.sleep(1.0)
        # rospy.signal_shutdown("test_complete")
        self._roscore.shutdown()
        self._roscore = None
        self._timeline = None
        gc.collect()
        pass

    async def test_camera(self):
        scene_path = "/Isaac/Environments/Grid/default_environment.usd"
        await open_stage_async(self._assets_root_path + scene_path)
        self._timeline = omni.timeline.get_timeline_interface()
        self._timeline.set_auto_update(True)

        cube_1 = VisualCuboid("/cube_1", position=[0, 0, 0], scale=[1.5, 1, 1])

        add_update_semantics(cube_1.prim, "Cube0")

        import rospy

        viewport_api = omni.kit.viewport.utility.get_active_viewport()
        render_product_path = viewport_api.get_render_product_path()

        try:
            og.Controller.edit(
                {"graph_path": "/ActionGraph", "evaluator_name": "execution"},
                {
                    og.Controller.Keys.CREATE_NODES: [
                        ("OnPlaybackTick", "omni.graph.action.OnPlaybackTick"),
                        ("RGBPublish", "omni.isaac.ros_bridge.ROS1CameraHelper"),
                        ("DepthPublish", "omni.isaac.ros_bridge.ROS1CameraHelper"),
                        ("DepthPclPublish", "omni.isaac.ros_bridge.ROS1CameraHelper"),
                        ("InstancePublish", "omni.isaac.ros_bridge.ROS1CameraHelper"),
                        ("SemanticPublish", "omni.isaac.ros_bridge.ROS1CameraHelper"),
                        ("Bbox2dTightPublish", "omni.isaac.ros_bridge.ROS1CameraHelper"),
                        ("Bbox2dLoosePublish", "omni.isaac.ros_bridge.ROS1CameraHelper"),
                        ("Bbox3dPublish", "omni.isaac.ros_bridge.ROS1CameraHelper"),
                        ("CameraInfoPublish", "omni.isaac.ros_bridge.ROS1CameraHelper"),
                    ],
                    og.Controller.Keys.SET_VALUES: [
                        ("RGBPublish.inputs:renderProductPath", render_product_path),
                        ("RGBPublish.inputs:topicName", "rgb"),
                        ("RGBPublish.inputs:type", "rgb"),
                        ("RGBPublish.inputs:resetSimulationTimeOnStop", True),
                        ("DepthPublish.inputs:renderProductPath", render_product_path),
                        ("DepthPublish.inputs:topicName", "depth"),
                        ("DepthPublish.inputs:type", "depth"),
                        ("DepthPublish.inputs:resetSimulationTimeOnStop", True),
                        ("DepthPclPublish.inputs:renderProductPath", render_product_path),
                        ("DepthPclPublish.inputs:topicName", "depth_pcl"),
                        ("DepthPclPublish.inputs:type", "depth_pcl"),
                        ("DepthPclPublish.inputs:resetSimulationTimeOnStop", True),
                        ("InstancePublish.inputs:renderProductPath", render_product_path),
                        ("InstancePublish.inputs:topicName", "instance_segmentation"),
                        ("InstancePublish.inputs:type", "instance_segmentation"),
                        ("InstancePublish.inputs:resetSimulationTimeOnStop", True),
                        ("SemanticPublish.inputs:renderProductPath", render_product_path),
                        ("SemanticPublish.inputs:topicName", "semantic_segmentation"),
                        ("SemanticPublish.inputs:type", "semantic_segmentation"),
                        ("SemanticPublish.inputs:resetSimulationTimeOnStop", True),
                        ("Bbox2dTightPublish.inputs:renderProductPath", render_product_path),
                        ("Bbox2dTightPublish.inputs:topicName", "bbox_2d_tight"),
                        ("Bbox2dTightPublish.inputs:type", "bbox_2d_tight"),
                        ("Bbox2dTightPublish.inputs:resetSimulationTimeOnStop", True),
                        ("Bbox2dLoosePublish.inputs:renderProductPath", render_product_path),
                        ("Bbox2dLoosePublish.inputs:topicName", "bbox_2d_loose"),
                        ("Bbox2dLoosePublish.inputs:type", "bbox_2d_loose"),
                        ("Bbox2dLoosePublish.inputs:resetSimulationTimeOnStop", True),
                        ("Bbox3dPublish.inputs:renderProductPath", render_product_path),
                        ("Bbox3dPublish.inputs:topicName", "bbox_3d"),
                        ("Bbox3dPublish.inputs:type", "bbox_3d"),
                        ("Bbox3dPublish.inputs:resetSimulationTimeOnStop", True),
                        ("CameraInfoPublish.inputs:renderProductPath", render_product_path),
                        ("CameraInfoPublish.inputs:topicName", "camera_info"),
                        ("CameraInfoPublish.inputs:type", "camera_info"),
                        ("CameraInfoPublish.inputs:resetSimulationTimeOnStop", True),
                    ],
                    og.Controller.Keys.CONNECT: [
                        ("OnPlaybackTick.outputs:tick", "RGBPublish.inputs:execIn"),
                        ("OnPlaybackTick.outputs:tick", "CameraInfoPublish.inputs:execIn"),
                        ("OnPlaybackTick.outputs:tick", "DepthPublish.inputs:execIn"),
                        ("OnPlaybackTick.outputs:tick", "DepthPclPublish.inputs:execIn"),
                        ("OnPlaybackTick.outputs:tick", "InstancePublish.inputs:execIn"),
                        ("OnPlaybackTick.outputs:tick", "SemanticPublish.inputs:execIn"),
                        ("OnPlaybackTick.outputs:tick", "Bbox2dTightPublish.inputs:execIn"),
                        ("OnPlaybackTick.outputs:tick", "Bbox2dLoosePublish.inputs:execIn"),
                        ("OnPlaybackTick.outputs:tick", "Bbox3dPublish.inputs:execIn"),
                    ],
                },
            )
        except Exception as e:
            print(e)

        # acquire the viewport window

        # Set viewport resolution, changes will occur on next frame
        viewport_api.set_texture_resolution((800, 600))

        await omni.kit.app.get_app().next_update_async()

        from sensor_msgs.msg import CameraInfo, Image, PointCloud2
        from vision_msgs.msg import Detection2DArray, Detection3DArray

        self._camera_info = None
        self._rgb = None
        self._depth = None
        self._depth_pcl = None
        self._instance_segmentation = None
        self._semantic_segmentation = None
        self._bbox_2d_tight = None
        self._bbox_2d_loose = None
        self._bbox_3d = None

        def camera_info_callback(data):
            self._camera_info = data

        def rgb_callback(data):
            self._rgb = data

        def depth_callback(data):
            self._depth = data

        def depth_pcl_callback(data):
            self._depth_pcl = data

        def instance_segmentation_callback(data):
            self._instance_segmentation = data

        def semantic_segmentation_callback(data):
            self._semantic_segmentation = data

        def bbox_2d_tight_callback(data):
            self._bbox_2d_tight = data

        def bbox_2d_loose_callback(data):
            self._bbox_2d_loose = data

        def bbox_3d_callback(data):
            self._bbox_3d = data

        camera_info_sub = rospy.Subscriber("camera_info", CameraInfo, camera_info_callback)
        rgb_sub = rospy.Subscriber("rgb", Image, rgb_callback)
        depth_sub = rospy.Subscriber("depth", Image, depth_callback)
        depth_pcl_sub = rospy.Subscriber("depth_pcl", PointCloud2, depth_pcl_callback)
        instance_segmentation_sub = rospy.Subscriber("instance_segmentation", Image, instance_segmentation_callback)
        semantic_segmentation_sub = rospy.Subscriber("semantic_segmentation", Image, semantic_segmentation_callback)
        bbox_2d_tight_sub = rospy.Subscriber("bbox_2d_tight", Detection2DArray, bbox_2d_tight_callback)
        bbox_2d_loose_sub = rospy.Subscriber("bbox_2d_loose", Detection2DArray, bbox_2d_loose_callback)
        bbox_3d_sub = rospy.Subscriber("bbox_3d", Detection3DArray, bbox_3d_callback)

        await asyncio.sleep(2.0)

        omni.kit.commands.execute(
            "ChangeProperty", prop_path=Sdf.Path("/OmniverseKit_Persp.horizontalAperture"), value=6.0, prev=0
        )

        # omni.kit.commands.execute(
        #     "ChangeProperty", prop_path=Sdf.Path("/OmniverseKit_Persp.verticalAperture"), value=4.5, prev=0
        # )
        import time

        system_time = time.time()
        self._timeline.play()
        await omni.kit.app.get_app().next_update_async()
        await simulate_async(1)
        for _ in range(10):
            if self._camera_info is None or self._bbox_2d_tight is None or self._bbox_3d is None:
                await simulate_async(1)

        self.assertEqual(self._camera_info.width, 800)
        self.assertEqual(self._camera_info.height, 600)
        self.assertAlmostEqual(self._camera_info.P[0], self._camera_info.P[5], delta=1.5)
        self.assertAlmostEqual(self._camera_info.K[0], self._camera_info.K[4], delta=1.5)
        self.assertGreaterEqual(self._camera_info.header.stamp.secs, 1)
        self.assertLess(self._camera_info.header.stamp.secs, system_time / 2.0)
        self.assertIsNotNone(self._rgb)
        self.assertIsNotNone(self._depth)
        self.assertIsNotNone(self._depth_pcl)
        self.assertIsNotNone(self._instance_segmentation)
        self.assertIsNotNone(self._semantic_segmentation)
        self.assertIsNotNone(self._bbox_2d_tight)
        self.assertIsNotNone(self._bbox_2d_loose)
        self.assertIsNotNone(self._bbox_3d)
        # self.assertAlmostEqual(self._camera_info.K[0], self._camera_info.K[4], delta=1.5)

        self._timeline.stop()
        # make sure all previous messages are cleared
        await asyncio.sleep(2.0)
        await simulate_async(1)
        self._camera_info = None
        self._rgb = None
        self._depth = None
        self._depth_pcl = None
        self._instance_segmentation = None
        self._semantic_segmentation = None
        self._bbox_2d_tight = None
        self._bbox_2d_loose = None
        self._bbox_3d = None

        omni.kit.commands.execute(
            "ChangeProperty", prop_path=Sdf.Path("/OmniverseKit_Persp.horizontalAperture"), value=6, prev=0
        )

        # Vertical apertures are computed off of horizontal aperture.
        # omni.kit.commands.execute(
        #     "ChangeProperty", prop_path=Sdf.Path("/OmniverseKit_Persp.verticalAperture"), value=6, prev=0
        # )

        await omni.kit.app.get_app().next_update_async()
        system_time = time.time()
        self._timeline.play()
        await omni.kit.app.get_app().next_update_async()
        await simulate_async(1)
        for _ in range(10):
            if self._camera_info is None:
                await simulate_async(1)

        self.assertAlmostEqual(self._camera_info.P[0], 2419, delta=1)

        # Fy = Fx, square pixels
        self.assertAlmostEqual(self._camera_info.P[5], 2419, delta=1)
        self.assertGreaterEqual(self._camera_info.header.stamp.secs, 1)
        self.assertLess(self._camera_info.header.stamp.secs, system_time / 2.0)

        self.assertIsNotNone(self._rgb)
        self.assertIsNotNone(self._depth)
        self.assertIsNotNone(self._depth_pcl)
        self.assertIsNotNone(self._instance_segmentation)
        self.assertIsNotNone(self._semantic_segmentation)
        self.assertIsNotNone(self._bbox_2d_tight)
        self.assertIsNotNone(self._bbox_2d_loose)
        self.assertIsNotNone(self._bbox_3d)

        self._timeline.stop()
        # make sure all previous messages are cleared
        await asyncio.sleep(2.0)
        await simulate_async(1)
        self._camera_info = None
        self._rgb = None
        self._depth = None
        self._depth_pcl = None
        self._instance_segmentation = None
        self._semantic_segmentation = None
        self._bbox_2d_tight = None
        self._bbox_2d_loose = None
        self._bbox_3d = None

        # Turn on SystemTime for timestamp of all camera publishers
        og.Controller.attribute("/ActionGraph/RGBPublish" + ".inputs:useSystemTime").set(True)
        og.Controller.attribute("/ActionGraph/DepthPublish" + ".inputs:useSystemTime").set(True)
        og.Controller.attribute("/ActionGraph/DepthPclPublish" + ".inputs:useSystemTime").set(True)
        og.Controller.attribute("/ActionGraph/InstancePublish" + ".inputs:useSystemTime").set(True)
        og.Controller.attribute("/ActionGraph/SemanticPublish" + ".inputs:useSystemTime").set(True)
        og.Controller.attribute("/ActionGraph/Bbox2dTightPublish" + ".inputs:useSystemTime").set(True)
        og.Controller.attribute("/ActionGraph/Bbox2dLoosePublish" + ".inputs:useSystemTime").set(True)
        og.Controller.attribute("/ActionGraph/Bbox3dPublish" + ".inputs:useSystemTime").set(True)
        og.Controller.attribute("/ActionGraph/CameraInfoPublish" + ".inputs:useSystemTime").set(True)

        await omni.kit.app.get_app().next_update_async()
        system_time = time.time()
        self._timeline.play()
        await omni.kit.app.get_app().next_update_async()
        await simulate_async(1)
        for _ in range(10):
            if self._camera_info is None:
                await simulate_async(1)

        self.assertAlmostEqual(self._camera_info.P[0], 2419, delta=1)

        # Fy = Fx, square pixels
        self.assertAlmostEqual(self._camera_info.P[5], 2419, delta=1)

        self.assertIsNotNone(self._rgb)
        self.assertIsNotNone(self._depth)
        self.assertIsNotNone(self._depth_pcl)
        self.assertIsNotNone(self._instance_segmentation)
        self.assertIsNotNone(self._semantic_segmentation)
        self.assertIsNotNone(self._bbox_2d_tight)
        self.assertIsNotNone(self._bbox_2d_loose)
        self.assertIsNotNone(self._bbox_3d)

        self.assertGreaterEqual(self._camera_info.header.stamp.secs, system_time)
        self.assertGreaterEqual(self._rgb.header.stamp.secs, system_time)
        self.assertGreaterEqual(self._depth.header.stamp.secs, system_time)
        self.assertGreaterEqual(self._depth_pcl.header.stamp.secs, system_time)
        self.assertGreaterEqual(self._instance_segmentation.header.stamp.secs, system_time)
        self.assertGreaterEqual(self._semantic_segmentation.header.stamp.secs, system_time)
        self.assertGreaterEqual(self._bbox_2d_tight.header.stamp.secs, system_time)
        self.assertGreaterEqual(self._bbox_2d_loose.header.stamp.secs, system_time)
        self.assertGreaterEqual(self._bbox_3d.header.stamp.secs, system_time)

        camera_info_sub.unregister()
        rgb_sub.unregister()
        depth_sub.unregister()
        depth_pcl_sub.unregister()
        instance_segmentation_sub.unregister()
        semantic_segmentation_sub.unregister()
        bbox_2d_tight_sub.unregister()
        bbox_2d_loose_sub.unregister()
        bbox_3d_sub.unregister()

    async def test_bbox(self):
        cube_1 = VisualCuboid("/cube_1", position=[2, 0, 0], scale=[1.5, 1, 1])
        cube_2 = VisualCuboid("/cube_2", position=[-1.5, 0, 0], scale=[1, 2, 1])
        cube_3 = VisualCuboid("/cube_3", position=[100, 0, 0], scale=[1, 1, 3])
        cube_4 = VisualCuboid("/cube_4", position=[0, 1, 0], scale=[1, 1, 3])
        add_update_semantics(cube_1.prim, "Cube0")
        add_update_semantics(cube_2.prim, "Cube1")
        add_update_semantics(cube_3.prim, "Cube2")
        add_update_semantics(cube_4.prim, "Cube3")
        set_camera_view(eye=[0, -6, 0.5], target=[0, 0, 0.5], camera_prim_path="/OmniverseKit_Persp")
        import json

        import rospy

        # acquire the viewport window
        viewport_api = omni.kit.viewport.utility.get_active_viewport()
        render_product_path = viewport_api.get_render_product_path()

        try:
            og.Controller.edit(
                {"graph_path": "/ActionGraph", "evaluator_name": "execution"},
                {
                    og.Controller.Keys.CREATE_NODES: [
                        ("OnPlaybackTick", "omni.graph.action.OnPlaybackTick"),
                        ("Bbox2dTightPublish", "omni.isaac.ros_bridge.ROS1CameraHelper"),
                        ("Bbox2dLoosePublish", "omni.isaac.ros_bridge.ROS1CameraHelper"),
                        ("Bbox3dPublish", "omni.isaac.ros_bridge.ROS1CameraHelper"),
                        ("InstancePublish", "omni.isaac.ros_bridge.ROS1CameraHelper"),
                        ("SemanticPublish", "omni.isaac.ros_bridge.ROS1CameraHelper"),
                    ],
                    og.Controller.Keys.SET_VALUES: [
                        ("InstancePublish.inputs:renderProductPath", render_product_path),
                        ("InstancePublish.inputs:topicName", "instance_segmentation"),
                        ("InstancePublish.inputs:type", "instance_segmentation"),
                        ("InstancePublish.inputs:resetSimulationTimeOnStop", True),
                        ("SemanticPublish.inputs:renderProductPath", render_product_path),
                        ("SemanticPublish.inputs:topicName", "semantic_segmentation"),
                        ("SemanticPublish.inputs:type", "semantic_segmentation"),
                        ("SemanticPublish.inputs:resetSimulationTimeOnStop", True),
                        ("Bbox2dTightPublish.inputs:renderProductPath", render_product_path),
                        ("Bbox2dTightPublish.inputs:topicName", "bbox_2d_tight"),
                        ("Bbox2dTightPublish.inputs:type", "bbox_2d_tight"),
                        ("Bbox2dTightPublish.inputs:resetSimulationTimeOnStop", True),
                        ("Bbox2dLoosePublish.inputs:renderProductPath", render_product_path),
                        ("Bbox2dLoosePublish.inputs:topicName", "bbox_2d_loose"),
                        ("Bbox2dLoosePublish.inputs:type", "bbox_2d_loose"),
                        ("Bbox2dLoosePublish.inputs:resetSimulationTimeOnStop", True),
                        ("Bbox3dPublish.inputs:renderProductPath", render_product_path),
                        ("Bbox3dPublish.inputs:topicName", "bbox_3d"),
                        ("Bbox3dPublish.inputs:type", "bbox_3d"),
                        ("Bbox3dPublish.inputs:resetSimulationTimeOnStop", True),
                        # enable semantics
                        ("InstancePublish.inputs:enableSemanticLabels", True),
                        ("InstancePublish.inputs:semanticLabelsTopicName", "semantic_labels_instance"),
                        ("SemanticPublish.inputs:enableSemanticLabels", True),
                        ("SemanticPublish.inputs:semanticLabelsTopicName", "semantic_labels_semantic"),
                        ("Bbox2dTightPublish.inputs:enableSemanticLabels", True),
                        ("Bbox2dTightPublish.inputs:semanticLabelsTopicName", "semantic_labels_tight"),
                        ("Bbox2dLoosePublish.inputs:enableSemanticLabels", True),
                        ("Bbox2dLoosePublish.inputs:semanticLabelsTopicName", "semantic_labels_loose"),
                        ("Bbox3dPublish.inputs:enableSemanticLabels", True),
                        ("Bbox3dPublish.inputs:semanticLabelsTopicName", "semantic_labels_3d"),
                    ],
                    og.Controller.Keys.CONNECT: [
                        ("OnPlaybackTick.outputs:tick", "InstancePublish.inputs:execIn"),
                        ("OnPlaybackTick.outputs:tick", "SemanticPublish.inputs:execIn"),
                        ("OnPlaybackTick.outputs:tick", "Bbox2dTightPublish.inputs:execIn"),
                        ("OnPlaybackTick.outputs:tick", "Bbox2dLoosePublish.inputs:execIn"),
                        ("OnPlaybackTick.outputs:tick", "Bbox3dPublish.inputs:execIn"),
                    ],
                },
            )
        except Exception as e:
            print(e)

        await omni.kit.app.get_app().next_update_async()

        from std_msgs.msg import String
        from vision_msgs.msg import Detection2DArray, Detection3DArray

        self._bbox_2d_tight = None
        self._bbox_2d_loose = None
        self._bbox_3d = None
        self._semantic_data_instance = None
        self._semantic_data_semantic = None
        self._semantic_data_3d = None
        self._semantic_data_tight = None
        self._semantic_data_loose = None

        def bbox_2d_tight_callback(data):
            self._bbox_2d_tight = data

        def bbox_2d_loose_callback(data):
            self._bbox_2d_loose = data

        def bbox_3d_callback(data):
            self._bbox_3d = data

        def semantic_callback_instance(data):
            self._semantic_data_instance = data

        def semantic_callback_semantic(data):
            self._semantic_data_semantic = data

        def semantic_callback_3d(data):
            self._semantic_data_3d = data

        def semantic_callback_tight(data):
            self._semantic_data_tight = data

        def semantic_callback_loose(data):
            self._semantic_data_loose = data

        bbox_2d_tight_sub = rospy.Subscriber("bbox_2d_tight", Detection2DArray, bbox_2d_tight_callback)
        bbox_2d_loose_sub = rospy.Subscriber("bbox_2d_loose", Detection2DArray, bbox_2d_loose_callback)

        bbox_3d_sub = rospy.Subscriber("bbox_3d", Detection3DArray, bbox_3d_callback)
        semantic_labels_instance_sub = rospy.Subscriber("semantic_labels_instance", String, semantic_callback_instance)
        semantic_labels_semantic_sub = rospy.Subscriber("semantic_labels_semantic", String, semantic_callback_semantic)
        semantic_labels_3d_sub = rospy.Subscriber("semantic_labels_3d", String, semantic_callback_3d)
        semantic_labels_tight_sub = rospy.Subscriber("semantic_labels_tight", String, semantic_callback_tight)
        semantic_labels_loose_sub = rospy.Subscriber("semantic_labels_loose", String, semantic_callback_loose)

        await asyncio.sleep(2.0)

        self._timeline.play()
        await omni.kit.app.get_app().next_update_async()
        await simulate_async(1)
        for _ in range(10):
            if self._bbox_3d is None or self._semantic_data_3d is None:
                await simulate_async(1)

        self.assertIsNotNone(self._bbox_2d_tight)
        self.assertIsNotNone(self._bbox_2d_loose)
        self.assertIsNotNone(self._bbox_3d)
        self.assertIsNotNone(self._semantic_data_instance)
        self.assertIsNotNone(self._semantic_data_semantic)
        self.assertIsNotNone(self._semantic_data_3d)
        self.assertIsNotNone(self._semantic_data_tight)
        self.assertIsNotNone(self._semantic_data_loose)

        detections = self._bbox_3d.detections
        semantic_instance_dict = json.loads(self._semantic_data_instance.data)
        semantic_semantic_dict = json.loads(self._semantic_data_semantic.data)
        semantic_tight_dict = json.loads(self._semantic_data_tight.data)
        semantic_loose_dict = json.loads(self._semantic_data_loose.data)
        semantic_3d_dict = json.loads(self._semantic_data_3d.data)
        print(semantic_instance_dict)
        print(semantic_semantic_dict)
        print(semantic_tight_dict)
        print(semantic_loose_dict)
        print(semantic_3d_dict)
        self.assertEqual(semantic_instance_dict["0"], "BACKGROUND")
        self.assertEqual(semantic_instance_dict["1"], "UNLABELLED")
        self.assertEqual(semantic_instance_dict["2"], "/cube_1")
        self.assertEqual(semantic_instance_dict["3"], "/cube_2")
        self.assertEqual(semantic_instance_dict["5"], "/cube_4")

        self.assertEqual(semantic_semantic_dict["0"]["class"], "BACKGROUND")
        self.assertEqual(len(semantic_semantic_dict.keys()), 6)  # (background + unlabeled + 3 cubes + timestamp)

        # all times should match
        self.assertDictEqual(semantic_3d_dict["time_stamp"], semantic_instance_dict["time_stamp"])
        self.assertDictEqual(semantic_3d_dict["time_stamp"], semantic_semantic_dict["time_stamp"])

        # bbox semantics should match
        self.assertEqual(self._semantic_data_3d.data, self._semantic_data_tight.data)
        self.assertEqual(self._semantic_data_3d.data, self._semantic_data_loose.data)

        self.assertEqual(semantic_3d_dict["0"]["class"], "cube0")
        self.assertEqual(semantic_3d_dict["1"]["class"], "cube1")
        self.assertEqual(semantic_3d_dict["2"]["class"], "cube3")

        # there should be 3 bboxes
        self.assertEqual(len(detections), 3)
        self.assertEqual(detections[0].results[0].id, 0)
        self.assertEqual(detections[1].results[0].id, 1)
        self.assertEqual(detections[2].results[0].id, 2)

        self.assertEqual(detections[0].bbox.size.x, 1.5)
        self.assertEqual(detections[0].bbox.size.y, 1)
        self.assertEqual(detections[0].bbox.size.z, 1)

        self.assertEqual(detections[1].bbox.size.x, 1)
        self.assertEqual(detections[1].bbox.size.y, 2)
        self.assertEqual(detections[1].bbox.size.z, 1)

        self.assertEqual(detections[2].bbox.size.x, 1)
        self.assertEqual(detections[2].bbox.size.y, 1)
        self.assertEqual(detections[2].bbox.size.z, 3)

        self.assertEqual(detections[0].bbox.center.position.x, 2)
        self.assertEqual(detections[0].bbox.center.position.y, 0)
        self.assertEqual(detections[0].bbox.center.position.z, 0)

        self.assertEqual(detections[1].bbox.center.position.x, -1.5)
        self.assertEqual(detections[1].bbox.center.position.y, 0)
        self.assertEqual(detections[1].bbox.center.position.z, 0)

        self.assertEqual(detections[2].bbox.center.position.x, 0)
        self.assertEqual(detections[2].bbox.center.position.y, 1)
        self.assertEqual(detections[2].bbox.center.position.z, 0)

        detections = self._bbox_2d_tight.detections
        self.assertEqual(len(detections), 3)

        print(detections[0].results)
        print(detections[1].results)
        print(detections[2].results)

        self.assertEqual(detections[0].results[0].id, 0)
        self.assertEqual(detections[1].results[0].id, 1)
        self.assertEqual(detections[2].results[0].id, 2)

        self.assertEqual(detections[0].bbox.size_x, 340.0)
        self.assertEqual(detections[0].bbox.size_y, 201.0)

        self.assertEqual(detections[1].bbox.size_x, 284.0)
        self.assertEqual(detections[1].bbox.size_y, 221.0)

        self.assertEqual(detections[2].bbox.size_x, 169.0)
        self.assertEqual(detections[2].bbox.size_y, 511.0)

        self.assertEqual(detections[0].bbox.center.x, 1023.0)
        self.assertEqual(detections[0].bbox.center.y, 460.5)
        self.assertEqual(detections[0].bbox.center.theta, 0)

        self.assertEqual(detections[1].bbox.center.x, 339.0)
        self.assertEqual(detections[1].bbox.center.y, 470.5)
        self.assertEqual(detections[1].bbox.center.theta, 0)

        self.assertEqual(detections[2].bbox.center.x, 639.5)
        self.assertEqual(detections[2].bbox.center.y, 444.5)
        self.assertEqual(detections[2].bbox.center.theta, 0)

        detections = self._bbox_2d_loose.detections
        self.assertEqual(len(detections), 3)

        self.assertEqual(detections[0].results[0].id, 0)
        self.assertEqual(detections[1].results[0].id, 1)
        self.assertEqual(detections[2].results[0].id, 2)

        self.assertEqual(detections[0].bbox.size_x, 340.0)
        self.assertEqual(detections[0].bbox.size_y, 201.0)

        self.assertEqual(detections[1].bbox.size_x, 284.0)
        self.assertEqual(detections[1].bbox.size_y, 221.0)

        self.assertEqual(detections[2].bbox.size_x, 169.0)
        self.assertEqual(detections[2].bbox.size_y, 511.0)

        self.assertEqual(detections[0].bbox.center.x, 1023.0)
        self.assertEqual(detections[0].bbox.center.y, 460.5)
        self.assertEqual(detections[0].bbox.center.theta, 0)

        self.assertEqual(detections[1].bbox.center.x, 339.0)
        self.assertEqual(detections[1].bbox.center.y, 470.5)
        self.assertEqual(detections[1].bbox.center.theta, 0)

        self.assertEqual(detections[2].bbox.center.x, 639.5)
        self.assertEqual(detections[2].bbox.center.y, 444.5)
        self.assertEqual(detections[2].bbox.center.theta, 0)

        bbox_2d_tight_sub.unregister()
        bbox_2d_loose_sub.unregister()
        bbox_3d_sub.unregister()
        semantic_labels_instance_sub.unregister()
        semantic_labels_semantic_sub.unregister()
        semantic_labels_3d_sub.unregister()
        semantic_labels_tight_sub.unregister()
        semantic_labels_loose_sub.unregister()

    async def test_empty_semantics(self):
        cube_3 = VisualCuboid("/cube_3", position=[100, 0, 0], scale=[1, 1, 3])
        add_update_semantics(cube_3.prim, "Cube2")
        set_camera_view(eye=[0, -6, 0.5], target=[0, 0, 0.5], camera_prim_path="/OmniverseKit_Persp")
        import json

        import rospy

        viewport_api = omni.kit.viewport.utility.get_active_viewport()
        render_product_path = viewport_api.get_render_product_path()

        try:
            og.Controller.edit(
                {"graph_path": "/ActionGraph", "evaluator_name": "execution"},
                {
                    og.Controller.Keys.CREATE_NODES: [
                        ("OnPlaybackTick", "omni.graph.action.OnPlaybackTick"),
                        ("Bbox3dPublish", "omni.isaac.ros_bridge.ROS1CameraHelper"),
                    ],
                    og.Controller.Keys.SET_VALUES: [
                        ("Bbox3dPublish.inputs:renderProductPath", render_product_path),
                        ("Bbox3dPublish.inputs:topicName", "bbox_3d"),
                        ("Bbox3dPublish.inputs:type", "bbox_3d"),
                        ("Bbox3dPublish.inputs:resetSimulationTimeOnStop", True),
                        # enable semantics
                        ("Bbox3dPublish.inputs:enableSemanticLabels", True),
                        ("Bbox3dPublish.inputs:semanticLabelsTopicName", "semantic_labels"),
                    ],
                    og.Controller.Keys.CONNECT: [("OnPlaybackTick.outputs:tick", "Bbox3dPublish.inputs:execIn")],
                },
            )
        except Exception as e:
            print(e)

        await omni.kit.app.get_app().next_update_async()

        from std_msgs.msg import String
        from vision_msgs.msg import Detection2DArray, Detection3DArray

        self._bbox_3d = None
        self._semantic_data = None

        def bbox_3d_callback(data):
            self._bbox_3d = data

        def semantic_callback(data):
            self._semantic_data = data

        bbox_3d_sub = rospy.Subscriber("bbox_3d", Detection3DArray, bbox_3d_callback)
        semantic_labels_sub = rospy.Subscriber("semantic_labels", String, semantic_callback)

        await asyncio.sleep(2.0)

        self._timeline.play()
        await omni.kit.app.get_app().next_update_async()
        await simulate_async(1)
        for _ in range(10):
            if self._semantic_data is None:
                await simulate_async(1)

        self.assertIsNotNone(self._semantic_data)

        semantic_dict = json.loads(self._semantic_data.data)
        self.assertTrue("time_stamp" in semantic_dict)
        self.assertFalse("0" in semantic_dict)

        bbox_3d_sub.unregister()
        semantic_labels_sub.unregister()
