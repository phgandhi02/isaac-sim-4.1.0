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
import random
import sys

import carb
import cv2
import numpy as np
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
from omni.isaac.core.prims.xform_prim import XFormPrim
from omni.isaac.core.utils.physics import simulate_async
from omni.isaac.core.utils.semantics import add_update_semantics
from omni.isaac.core.utils.stage import add_reference_to_stage, get_current_stage, open_stage_async
from omni.isaac.core.utils.viewports import set_camera_view
from omni.isaac.nucleus import get_assets_root_path_async
from omni.isaac.sensor import Camera
from pxr import Gf, Sdf, UsdLux

from .common import add_carter_ros, add_cube, get_qos_profile


# Having a test class derived from omni.kit.test.AsyncTestCase declared on the root of module will make it auto-discoverable by omni.kit.test
class TestRos2CameraInfo(omni.kit.test.AsyncTestCase):
    # Before running each test
    async def setUp(self):

        import rclpy

        omni.usd.get_context().new_stage()
        self._timeline = omni.timeline.get_timeline_interface()

        ext_manager = omni.kit.app.get_app().get_extension_manager()
        ext_id = ext_manager.get_enabled_extension_id("omni.isaac.ros2_bridge")
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
        rclpy.init()

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
        import rclpy

        while omni.usd.get_context().get_stage_loading_status()[2] > 0:
            print("tearDown, assets still loading, waiting to finish...")
            await asyncio.sleep(1.0)

        self._timeline = None
        rclpy.shutdown()
        gc.collect()
        pass

    # def cvtype2_to_dtype_with_channels(self, cvtype):
    #     # given CV_8UC3 for RGB8
    #     #
    #     CV_CN_SHIFT = 3
    #     CV_CN_MAX = 512
    #     CV_MAT_CN_MASK = ((CV_CN_MAX - 1)) << CV_CN_SHIFT
    #     CV_MAT_CN = ((((flags) & CV_MAT_CN_MASK) >> CV_CN_SHIFT) + 1)

    #     CV_MAT_C
    #     from cv_bridge.boost.cv_bridge_boost import CV_MAT_CNWrap, CV_MAT_DEPTHWrap
    #     return self.cvdepth_to_numpy_depth[CV_MAT_DEPTHWrap(cvtype)], CV_MAT_CNWrap(cvtype)

    # def encoding_to_cvtype2(self, encoding):
    #     # returns CV_8UC3 for RGB8
    #     from cv_bridge.boost.cv_bridge_boost import getCvType

    #     try:
    #         return getCvType(encoding)
    #     except RuntimeError as e:
    #         raise CvBridgeError(e)

    # def encoding_to_dtype_with_channels(self, encoding):
    #     return self.cvtype2_to_dtype_with_channels(self.encoding_to_cvtype2(encoding))

    def imgmsg_to_cv2(self, img_msg):
        # encoding for RGB images is Type_RGB8 (token = rgb8) by default
        # dtype, n_channels = self.encoding_to_dtype_with_channels(img_msg.encoding)
        dtype = "uint8"
        n_channels = 3
        dtype = np.dtype(dtype)
        dtype = dtype.newbyteorder(">" if img_msg.is_bigendian else "<")

        img_buf = np.asarray(img_msg.data, dtype=dtype) if isinstance(img_msg.data, list) else img_msg.data

        if n_channels == 1:
            im = np.ndarray(shape=(img_msg.height, int(img_msg.step / dtype.itemsize)), dtype=dtype, buffer=img_buf)
            im = np.ascontiguousarray(im[: img_msg.height, : img_msg.width])
        else:
            im = np.ndarray(
                shape=(img_msg.height, int(img_msg.step / dtype.itemsize / n_channels), n_channels),
                dtype=dtype,
                buffer=img_buf,
            )
            im = np.ascontiguousarray(im[: img_msg.height, : img_msg.width, :])

        # If the byte order is different between the message and the system.
        if img_msg.is_bigendian == (sys.byteorder == "little"):
            im = im.byteswap().newbyteorder()

        return im

    async def test_monocular_camera_info(self):
        scene_path = "/Isaac/Environments/Grid/default_environment.usd"
        await open_stage_async(self._assets_root_path + scene_path)

        camera_path = "/Isaac/Sensors/LeopardImaging/Hawk/hawk_v1.1_nominal.usd"
        add_reference_to_stage(usd_path=camera_path, prim_path="/Hawk")

        import rclpy
        import usdrt.Sdf

        try:
            og.Controller.edit(
                {"graph_path": "/ActionGraph", "evaluator_name": "execution"},
                {
                    og.Controller.Keys.CREATE_NODES: [
                        ("OnPlaybackTick", "omni.graph.action.OnPlaybackTick"),
                        ("CreateRenderProduct", "omni.isaac.core_nodes.IsaacCreateRenderProduct"),
                        ("CameraInfoPublish", "omni.isaac.ros2_bridge.ROS2CameraInfoHelper"),
                    ],
                    og.Controller.Keys.SET_VALUES: [
                        ("CreateRenderProduct.inputs:cameraPrim", [usdrt.Sdf.Path("/Hawk/left/camera_left")]),
                        ("CreateRenderProduct.inputs:height", 1200),
                        ("CreateRenderProduct.inputs:width", 1920),
                        ("CameraInfoPublish.inputs:topicName", "camera_info"),
                        ("CameraInfoPublish.inputs:resetSimulationTimeOnStop", True),
                    ],
                    og.Controller.Keys.CONNECT: [
                        ("OnPlaybackTick.outputs:tick", "CreateRenderProduct.inputs:execIn"),
                        ("CreateRenderProduct.outputs:execOut", "CameraInfoPublish.inputs:execIn"),
                        ("CreateRenderProduct.outputs:renderProductPath", "CameraInfoPublish.inputs:renderProductPath"),
                    ],
                },
            )
        except Exception as e:
            print(e)
        await omni.kit.app.get_app().next_update_async()

        from sensor_msgs.msg import CameraInfo

        self._camera_info = None

        def camera_info_callback(data):
            self._camera_info = data

        node = rclpy.create_node("camera_tester")
        camera_info_sub = node.create_subscription(CameraInfo, "camera_info", camera_info_callback, get_qos_profile())

        await omni.kit.app.get_app().next_update_async()

        def spin():
            rclpy.spin_once(node, timeout_sec=0.1)

        import time

        system_time = time.time()

        for num in range(5):
            print(f"Play #{num+1}")
            self._timeline.play()
            await omni.kit.app.get_app().next_update_async()
            await simulate_async(1.5, callback=spin)
            for _ in range(10):
                if self._camera_info is None:
                    await simulate_async(1, callback=spin)

            self.assertIsNotNone(self._camera_info)

            self.assertEqual(self._camera_info.width, 1920)
            self.assertEqual(self._camera_info.height, 1200)
            self.assertGreaterEqual(self._camera_info.header.stamp.sec, 1)
            self.assertLess(self._camera_info.header.stamp.sec, system_time / 2.0)

            # fx = width * focalLength / horizontalAperture
            # fy = height * focalLength / verticalAperture
            # cx = width * 0.5
            # cy = height * 0.5
            # camera_info["k"] = np.asarray([[fx, 0.0, cx], [0.0, fy, cy], [0.0, 0.0, 1.0]])
            # camera_info["r"] = np.eye(N=3, dtype=float)
            # camera_info["p"] = np.concatenate((camera_info["k"], np.zeros(shape=[3, 1], dtype=float)), axis=1)

            # Test contents of k matrix (function of width, height, focal length, apertures)
            self.assertAlmostEqual(self._camera_info.k[0], 1920.0 * 2.87343 / 5.76, places=2)
            self.assertAlmostEqual(self._camera_info.k[1], 0.0)
            self.assertAlmostEqual(self._camera_info.k[2], 1920.0 * 0.5)
            self.assertAlmostEqual(self._camera_info.k[3], 0.0)
            self.assertAlmostEqual(self._camera_info.k[4], 1200.0 * 2.87343 / 3.6, places=2)
            self.assertAlmostEqual(self._camera_info.k[5], 1200.0 * 0.5)
            self.assertAlmostEqual(self._camera_info.k[6], 0.0)
            self.assertAlmostEqual(self._camera_info.k[7], 0.0)
            self.assertAlmostEqual(self._camera_info.k[8], 1.0)

            # Test if r matrix is identity
            for i in range(3):
                for j in range(3):
                    self.assertAlmostEqual(self._camera_info.r[i * 3 + j], 1.0 if i == j else 0.0)

            # Test if p matrix is k matrix concatenated with 1x3 0 vector
            for i in range(3):
                for j in range(3):
                    self.assertAlmostEqual(self._camera_info.p[i * 4 + j], self._camera_info.k[i * 3 + j])
                self.assertAlmostEqual(self._camera_info.p[i * 4 + 3], 0.0)

            # Test distortion model and coefficients
            self.assertAlmostEqual(self._camera_info.distortion_model, "rational_polynomial")
            distortion_coefficients = [0.147811, -0.032313, -0.000194, -0.000035, 0.008823, 0.517913, -0.06708, 0.01695]
            for i in range(len(distortion_coefficients)):
                self.assertAlmostEqual(self._camera_info.d[i], distortion_coefficients[i])
            self._timeline.stop()

            # make sure all previous messages are cleared
            await omni.kit.app.get_app().next_update_async()
            spin()
            await omni.kit.app.get_app().next_update_async()
            self._camera_info = None

    async def test_stereo_camera_info(self):

        camera_path = self._assets_root_path + "/Isaac/Sensors/LeopardImaging/Hawk/hawk_v1.1_nominal.usd"
        add_reference_to_stage(usd_path=camera_path, prim_path="/Hawk")

        checkerboard_path = (
            self._assets_root_path
            + "/Projects/isaac_amr_envoy/scenes/calibration/checkerboard/checkere_board_isaac_sim.usd"
        )
        add_reference_to_stage(usd_path=checkerboard_path, prim_path="/calibration_target")
        XFormPrim("/calibration_target", name="calibration_target", position=[1.0, -0.075, -0.6])

        def add_light(name: str, z: float):
            sphereLight = UsdLux.SphereLight.Define(get_current_stage(), Sdf.Path(f"/World/SphereLight_{name}"))
            sphereLight.CreateRadiusAttr(6)
            sphereLight.CreateIntensityAttr(10000)
            XFormPrim(str(sphereLight.GetPath())).set_world_pose([0.5, -0.075, z])

        add_light("top", 12)
        add_light("bottom", -12)

        import rclpy
        import usdrt.Sdf

        try:
            og.Controller.edit(
                {"graph_path": "/ActionGraph", "evaluator_name": "execution"},
                {
                    og.Controller.Keys.CREATE_NODES: [
                        ("OnPlaybackTick", "omni.graph.action.OnPlaybackTick"),
                        ("RunOneSimulationFrame", "omni.isaac.core_nodes.OgnIsaacRunOneSimulationFrame"),
                        ("CreateRenderProductLeft", "omni.isaac.core_nodes.IsaacCreateRenderProduct"),
                        ("CreateRenderProductRight", "omni.isaac.core_nodes.IsaacCreateRenderProduct"),
                        ("CameraInfoPublish", "omni.isaac.ros2_bridge.ROS2CameraInfoHelper"),
                        ("RGBPublishLeft", "omni.isaac.ros2_bridge.ROS2CameraHelper"),
                        ("RGBPublishRight", "omni.isaac.ros2_bridge.ROS2CameraHelper"),
                    ],
                    og.Controller.Keys.SET_VALUES: [
                        ("CreateRenderProductLeft.inputs:cameraPrim", [usdrt.Sdf.Path("/Hawk/left/camera_left")]),
                        ("CreateRenderProductLeft.inputs:height", 1200),
                        ("CreateRenderProductLeft.inputs:width", 1920),
                        ("CreateRenderProductRight.inputs:cameraPrim", [usdrt.Sdf.Path("/Hawk/right/camera_right")]),
                        ("CreateRenderProductRight.inputs:height", 1200),
                        ("CreateRenderProductRight.inputs:width", 1920),
                        ("CameraInfoPublish.inputs:topicName", "camera_info_left"),
                        ("CameraInfoPublish.inputs:topicNameRight", "camera_info_right"),
                        ("CameraInfoPublish.inputs:resetSimulationTimeOnStop", True),
                        ("RGBPublishLeft.inputs:topicName", "rgb_left"),
                        ("RGBPublishLeft.inputs:type", "rgb"),
                        ("RGBPublishLeft.inputs:resetSimulationTimeOnStop", True),
                        ("RGBPublishRight.inputs:topicName", "rgb_right"),
                        ("RGBPublishRight.inputs:type", "rgb"),
                        ("RGBPublishRight.inputs:resetSimulationTimeOnStop", True),
                    ],
                    og.Controller.Keys.CONNECT: [
                        ("OnPlaybackTick.outputs:tick", "RunOneSimulationFrame.inputs:execIn"),
                        ("RunOneSimulationFrame.outputs:step", "CreateRenderProductLeft.inputs:execIn"),
                        ("RunOneSimulationFrame.outputs:step", "CreateRenderProductRight.inputs:execIn"),
                        ("CreateRenderProductLeft.outputs:execOut", "CameraInfoPublish.inputs:execIn"),
                        (
                            "CreateRenderProductLeft.outputs:renderProductPath",
                            "CameraInfoPublish.inputs:renderProductPath",
                        ),
                        ("CreateRenderProductRight.outputs:execOut", "CameraInfoPublish.inputs:execIn"),
                        (
                            "CreateRenderProductRight.outputs:renderProductPath",
                            "CameraInfoPublish.inputs:renderProductPathRight",
                        ),
                        ("CreateRenderProductLeft.outputs:execOut", "RGBPublishLeft.inputs:execIn"),
                        (
                            "CreateRenderProductLeft.outputs:renderProductPath",
                            "RGBPublishLeft.inputs:renderProductPath",
                        ),
                        ("CreateRenderProductRight.outputs:execOut", "RGBPublishRight.inputs:execIn"),
                        (
                            "CreateRenderProductRight.outputs:renderProductPath",
                            "RGBPublishRight.inputs:renderProductPath",
                        ),
                    ],
                },
            )
        except Exception as e:
            print(e)
        await omni.kit.app.get_app().next_update_async()

        from sensor_msgs.msg import CameraInfo, Image

        self._camera_info_left = None
        self._image_left = None
        self._camera_info_right = None
        self._image_right = None

        def camera_info_left_callback(data):
            self._camera_info_left = data

        def image_left_callback(data):
            self._image_left = data

        def camera_info_right_callback(data):
            self._camera_info_right = data

        def image_right_callback(data):
            self._image_right = data

        node_left = rclpy.create_node("camera_tester_left")
        camera_info_sub_left = node_left.create_subscription(
            CameraInfo, "camera_info_left", camera_info_left_callback, get_qos_profile()
        )
        rgb_sub_left = node_left.create_subscription(Image, "rgb_left", image_left_callback, get_qos_profile())

        node_right = rclpy.create_node("camera_tester_right")
        camera_info_sub_right = node_right.create_subscription(
            CameraInfo, "camera_info_right", camera_info_right_callback, get_qos_profile()
        )
        rgb_sub_right = node_right.create_subscription(Image, "rgb_right", image_right_callback, get_qos_profile())

        await omni.kit.app.get_app().next_update_async()

        def spin_left():
            rclpy.spin_once(node_left, timeout_sec=0.1)

        def spin_right():
            rclpy.spin_once(node_right, timeout_sec=0.1)

        import time

        system_time = time.time()

        self._timeline.play()
        await omni.kit.app.get_app().next_update_async()
        await simulate_async(1.5, callback=spin_right)
        await simulate_async(1.5, callback=spin_left)

        self.assertIsNotNone(self._camera_info_left)
        self.assertIsNotNone(self._camera_info_right)

        self.assertEqual(self._camera_info_left.width, 1920)
        self.assertEqual(self._camera_info_left.height, 1200)
        self.assertGreaterEqual(self._camera_info_left.header.stamp.sec, 0)
        self.assertLess(self._camera_info_left.header.stamp.sec, system_time / 2.0)

        self.assertEqual(self._camera_info_right.width, 1920)
        self.assertEqual(self._camera_info_right.height, 1200)
        self.assertGreaterEqual(self._camera_info_right.header.stamp.sec, 0)
        self.assertLess(self._camera_info_right.header.stamp.sec, system_time / 2.0)

        # Test contents of k matrix (function of width, height, focal length, apertures)
        self.assertAlmostEqual(self._camera_info_left.k[0], 1920.0 * 2.87343 / 5.76, places=2)
        self.assertAlmostEqual(self._camera_info_left.k[1], 0.0)
        self.assertAlmostEqual(self._camera_info_left.k[2], 1920.0 * 0.5)
        self.assertAlmostEqual(self._camera_info_left.k[3], 0.0)
        self.assertAlmostEqual(self._camera_info_left.k[4], 1200.0 * 2.87343 / 3.6, places=2)
        self.assertAlmostEqual(self._camera_info_left.k[5], 1200.0 * 0.5)
        self.assertAlmostEqual(self._camera_info_left.k[6], 0.0)
        self.assertAlmostEqual(self._camera_info_left.k[7], 0.0)
        self.assertAlmostEqual(self._camera_info_left.k[8], 1.0)

        self.assertAlmostEqual(self._camera_info_right.k[0], 1920.0 * 2.87798 / 5.76, places=2)
        self.assertAlmostEqual(self._camera_info_right.k[1], 0.0)
        self.assertAlmostEqual(self._camera_info_right.k[2], 1920.0 * 0.5)
        self.assertAlmostEqual(self._camera_info_right.k[3], 0.0)
        self.assertAlmostEqual(self._camera_info_right.k[4], 1200.0 * 2.87798 / 3.6, places=2)
        self.assertAlmostEqual(self._camera_info_right.k[5], 1200.0 * 0.5)
        self.assertAlmostEqual(self._camera_info_right.k[6], 0.0)
        self.assertAlmostEqual(self._camera_info_right.k[7], 0.0)
        self.assertAlmostEqual(self._camera_info_right.k[8], 1.0)

        # Test if r matrix is identity
        for i in range(3):
            for j in range(3):
                self.assertAlmostEqual(self._camera_info_left.r[i * 3 + j], 1.0 if i == j else 0.0)
                self.assertAlmostEqual(self._camera_info_right.r[i * 3 + j], 1.0 if i == j else 0.0)

        # Test newly-calibrated p matrix (right only)
        for i in range(12):
            if i == 2:
                continue
            elif i == 3:
                self.assertAlmostEqual(self._camera_info_left.p[i], 0.0)
                self.assertAlmostEqual(self._camera_info_right.p[i], -143.7853577)
            else:
                self.assertAlmostEqual(self._camera_info_left.p[i], self._camera_info_right.p[i])

        # Test distortion model and coefficients
        self.assertAlmostEqual(self._camera_info_left.distortion_model, "rational_polynomial")
        self.assertAlmostEqual(self._camera_info_right.distortion_model, "rational_polynomial")
        distortion_coefficients_left = [
            0.147811,
            -0.032313,
            -0.000194,
            -0.000035,
            0.008823,
            0.517913,
            -0.06708,
            0.01695,
        ]
        distortion_coefficients_right = [
            6.815791,
            5.172144,
            -0.000246,
            -0.000128,
            0.353267,
            7.180808,
            7.640372,
            1.596375,
        ]
        for i in range(len(distortion_coefficients_left)):
            self.assertAlmostEqual(self._camera_info_left.d[i], distortion_coefficients_left[i], places=6)
            self.assertAlmostEqual(self._camera_info_right.d[i], distortion_coefficients_right[i], places=6)
        self._camera_info_left.d = [
            0.147811,
            -0.032313,
            -0.000194,
            -0.000035,
            0.008823,
            0.517913,
            -0.06708,
            0.01695,
        ]
        self._camera_info_right.d = [
            6.815791,
            5.172144,
            -0.000246,
            -0.000128,
            0.353267,
            7.180808,
            7.640372,
            1.596375,
        ]

        def get_rectified_image(image_msg_raw, camera_info_msg, side):
            # Convert ROS2 image message data buffer to CV2 image
            image_raw = self.imgmsg_to_cv2(image_msg_raw)
            # Initialize the mapping arrays to rectify the raw image
            k = np.reshape(np.array(camera_info_msg.k), (3, 3))
            r = np.reshape(np.array(camera_info_msg.r), (3, 3))
            p = np.reshape(np.array(camera_info_msg.p), (3, 4))
            map1, map2 = cv2.initUndistortRectifyMap(
                cameraMatrix=k,
                distCoeffs=np.array(camera_info_msg.d),
                R=r,
                newCameraMatrix=p,
                size=(camera_info_msg.width, camera_info_msg.height),
                m1type=cv2.CV_32FC1,
            )
            # Return the rectified image
            cv2.imwrite(f"{side}_image_raw.png", image_raw)
            return cv2.remap(src=image_raw, map1=map1, map2=map2, interpolation=cv2.INTER_LANCZOS4)

        left_image_rect = get_rectified_image(self._image_left, self._camera_info_left, "left")
        right_image_rect = get_rectified_image(self._image_right, self._camera_info_right, "right")

        chessboard_dims = (6, 10)

        VISUALIZE = False
        # Comparison logic borrowed from Isaac ROS
        if VISUALIZE:
            cv2.imwrite("left_image_rect.png", left_image_rect)
            cv2.imwrite("right_image_rect.png", right_image_rect)
        # left_image_rect_gray = cv2.cvtColor(left_image_rect, cv2.COLOR_RGB2GRAY)
        left_ret, left_corners = cv2.findChessboardCorners(left_image_rect, chessboard_dims, None)
        self.assertTrue(left_ret, "Couldn't find chessboard corners in output left image!")
        # right_image_rect_gray = cv2.cvtColor(right_image_rect, cv2.COLOR_RGB2GRAY)
        right_ret, right_corners = cv2.findChessboardCorners(right_image_rect, chessboard_dims, None)
        self.assertTrue(right_ret, "Couldn't find chessboard corners in output right image!")
        # Extract the x and y coordinates of the corners in left_corners and right_corners
        x_coords_left = [c[0][0] for c in left_corners]
        y_coords_left = [c[0][1] for c in left_corners]
        x_coords_right = [c[0][0] for c in right_corners]
        y_coords_right = [c[0][1] for c in right_corners]
        if VISUALIZE:
            # Draw lines of the same color at the average row value for all corners
            # in the left and right image
            cv2.drawChessboardCorners(left_image_rect, chessboard_dims, left_corners, left_ret)
            cv2.drawChessboardCorners(right_image_rect, chessboard_dims, right_corners, right_ret)
            # Draw randomly colored lines connecting the corresponding corners
            for i in range(min(len(left_corners), len(right_corners))):
                average_y = (y_coords_left[i] + y_coords_right[i]) / 2
                pt1 = (0, int(average_y))
                pt2 = (left_image_rect.shape[1], int(average_y))
                random_color = (random.randint(0, 255), random.randint(0, 255), random.randint(0, 255))
                cv2.line(left_image_rect, pt1, pt2, random_color, 1)
                cv2.line(right_image_rect, pt1, pt2, random_color, 1)
            cv2.imwrite("left_image_rect.png", left_image_rect)
            cv2.imwrite("right_image_rect.png", right_image_rect)

        """
        Test 1:
        Check if the row values of the same corners of the chessboard
        in the left and right images are within threshold
        """
        # Compute the differences in row values
        row_diffs = [
            abs(y_coords_left[i] - y_coords_right[i]) for i in range(min(len(left_corners), len(right_corners)))
        ]
        # Allows test pass if same features are within of 4 pixels in both images
        CORNER_ROW_DIFF_THRESHOLD = 4
        if VISUALIZE:
            print("CORNER_ROW_DIFF_THRESHOLD :")
            print(CORNER_ROW_DIFF_THRESHOLD)
            print("row_diffs :")
            print(row_diffs)
        self.assertFalse(
            any(diff > CORNER_ROW_DIFF_THRESHOLD for diff in row_diffs),
            "Difference between corners row values in left and right images" "are not within threshold",
        )

        """
        Test 2:
        Check if the slopes of the lines connecting each corresponding corners in
        the left and right images are within threshold from the mean.
        This test checks if these lines(epipolar lines) are parallel
        """
        # Compute the slopes of lines between corresponding corners
        slopes = [
            (y_coords_right[i] - y_coords_left[i]) / (x_coords_right[i] - x_coords_left[i])
            for i in range(min(len(left_corners), len(right_corners)))
        ]
        s = np.array(slopes)
        # Create a list of difference betwen the slope and average slope
        mean_slope_diffs = abs(slopes - (sum(slopes) / len(slopes)))
        EPIPOLAR_LINES_SLOPE_DIFF_THRESHOLD = 0.005
        if VISUALIZE:
            print("EPIPOLAR_LINES_SLOPE_DIFF_THRESHOLD :")
            print(EPIPOLAR_LINES_SLOPE_DIFF_THRESHOLD)
            print("mean_slope_diffs :")
        # TODO: expand test to be any slopes above threshold
        self.assertLessEqual(
            sum(mean_slope_diffs > EPIPOLAR_LINES_SLOPE_DIFF_THRESHOLD),
            2,
            "Epipolar lines are not parallel!",
        )

        # make sure all previous messages are cleared
        self._timeline.stop()
        await omni.kit.app.get_app().next_update_async()
        spin_left()
        spin_right()
        await omni.kit.app.get_app().next_update_async()
        self._camera_info_left = None
        self._camera_info_right = None
        self._image_left = None
        self._image_right = None

        node_left.destroy_node()
        node_right.destroy_node()
