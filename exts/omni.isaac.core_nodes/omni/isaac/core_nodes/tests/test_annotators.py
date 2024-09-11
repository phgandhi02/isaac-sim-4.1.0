# Copyright (c) 2018-2023, NVIDIA CORPORATION. All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto. Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.
#


import carb
import numpy as np
import omni.graph.core as og
import omni.graph.core.tests as ogts
import omni.kit.test
import omni.replicator.core as rep
from omni.isaac.core.objects.ground_plane import GroundPlane
from omni.isaac.core.robots import Robot
from omni.isaac.core.utils.stage import get_current_stage, open_stage_async
from omni.isaac.core.utils.viewports import set_camera_view
from omni.isaac.nucleus import get_assets_root_path
from omni.kit.viewport.utility import get_active_viewport
from pxr import Sdf, UsdLux


class TestAnnotators(omni.kit.test.AsyncTestCase):
    async def setUp(self):
        """Set up  test environment, to be torn down when done"""
        await omni.usd.get_context().new_stage_async()
        self._timeline = omni.timeline.get_timeline_interface()
        self._viewport_api = get_active_viewport()
        self._render_product_path = self._viewport_api.get_render_product_path()

        ground_plane = GroundPlane("/World/ground_plane", visible=True)
        self._stage = get_current_stage()
        distantLight = UsdLux.DistantLight.Define(self._stage, Sdf.Path("/DistantLight"))
        # action_registry = omni.kit.actions.core.get_action_registry()
        # self._action = action_registry.get_action("omni.kit.viewport.actions", "toggle_grid_visibility")
        # self._action.execute(viewport_api=self._viewport_api, visible=False)
        self._stage = omni.usd.get_context().get_stage()
        self._timeline = omni.timeline.get_timeline_interface()
        self._stage.SetTimeCodesPerSecond(60)
        self._timeline.set_target_framerate(60)
        set_camera_view(eye=[-6, 0, 6.5], target=[-6, 0, -1], camera_prim_path="/OmniverseKit_Persp")
        await omni.kit.app.get_app().next_update_async()

    # ----------------------------------------------------------------------
    async def tearDown(self):
        # self._action.execute(viewport_api=self._viewport_api, visible=True)
        pass

    # ----------------------------------------------------------------------
    async def test_noop(self):
        annotator = rep.AnnotatorRegistry.get_annotator("IsaacNoop")
        annotator.attach([self._render_product_path])
        self._timeline.play()
        await omni.kit.app.get_app().next_update_async()
        await omni.kit.app.get_app().next_update_async()
        annotator.detach()

    # async def test_read_camera_info(self):
    #     annotator = rep.AnnotatorRegistry.get_annotator("IsaacReadCameraInfo")
    #     annotator.attach([self._render_product_path])
    #     self._timeline.play()
    #     await omni.kit.app.get_app().next_update_async()
    #     await omni.kit.app.get_app().next_update_async()
    #     data = annotator.get_data()
    #     # print(data)
    #     self.assertAlmostEqual(data["focalLength"], 18.14756202697754)
    #     annotator.detach()

    async def test_read_times(self):
        annotator = rep.AnnotatorRegistry.get_annotator("IsaacReadTimes")
        annotator.attach([self._render_product_path])

        self._timeline.play()
        await omni.syntheticdata.sensors.next_render_simulation_async(self._render_product_path, 10)
        data = annotator.get_data()
        self.assertAlmostEqual(data["simulationTime"], 0.01666666753590107 * 10)
        # print(data)
        annotator.detach()

    # TODO: this test won't work unless something consumes the time output so the node is executed
    # async def test_read_simulation_time(self):
    #     annotator = rep.AnnotatorRegistry.get_annotator("IsaacReadSimulationTime")
    #     annotator.attach([self._render_product_path])

    #     self._timeline.play()
    #     await omni.kit.app.get_app().next_update_async()
    #     await omni.kit.app.get_app().next_update_async()
    #     await omni.kit.app.get_app().next_update_async()
    #     await omni.kit.app.get_app().next_update_async()
    #     await omni.kit.app.get_app().next_update_async()
    #     await omni.kit.app.get_app().next_update_async()
    #     data = annotator.get_data()
    #     # self.assertAlmostEqual(data["simulationTime"], 0.1666666753590107)
    #     print(data)
    #     # annotator.detach()

    async def test_convert_rgba_to_rgb(self):
        import omni.syntheticdata._syntheticdata as sd

        rv = omni.syntheticdata.SyntheticData.convert_sensor_type_to_rendervar(sd.SensorType.Rgb.name)
        annotator = rep.AnnotatorRegistry.get_annotator(rv + "IsaacConvertRGBAToRGB")
        annotator.attach([self._render_product_path])

        self._timeline.play()
        await omni.syntheticdata.sensors.next_render_simulation_async(self._render_product_path, 10)
        data = annotator.get_data()
        self.assertTrue(np.all(data["data"] > 150))
        annotator.detach()

    async def test_convert_depth_to_point_cloud(self):
        import omni.syntheticdata._syntheticdata as sd

        rv = omni.syntheticdata.SyntheticData.convert_sensor_type_to_rendervar(sd.SensorType.DistanceToImagePlane.name)
        annotator = rep.AnnotatorRegistry.get_annotator(rv + "IsaacConvertDepthToPointCloud")
        annotator.attach([self._render_product_path])

        self._timeline.play()
        await omni.syntheticdata.sensors.next_render_simulation_async(self._render_product_path, 10)
        data = annotator.get_data()
        self.assertTrue(np.all(np.linalg.norm(data["data"], axis=1) > 0))

        annotator.detach()
