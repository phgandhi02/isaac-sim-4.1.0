# Copyright (c) 2021-2023, NVIDIA CORPORATION. All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto. Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.
#

import carb
import numpy as np
import omni.kit.test
from omni.isaac.core.utils.viewports import (
    destroy_all_viewports,
    get_id_from_index,
    get_intrinsics_matrix,
    get_viewport_names,
    get_window_from_id,
    set_camera_view,
    set_intrinsics_matrix,
)
from omni.kit.viewport.utility import (
    create_viewport_window,
    get_active_viewport,
    get_active_viewport_window,
    get_num_viewports,
)
from pxr import Sdf


class TestViewports(omni.kit.test.AsyncTestCase):
    # Before running each test
    async def setUp(self):
        await omni.usd.get_context().new_stage_async()
        pass

    # After running each test
    async def tearDown(self):
        pass

    async def test_get_intrinsics(self):
        viewport_api = get_active_viewport()
        viewport_api.set_texture_resolution((800, 600))
        await omni.kit.app.get_app().next_update_async()
        matrix = get_intrinsics_matrix(viewport_api)

        # Square pixel, vertical aperture is computed from horizontal aperture
        # omni.kit.commands.execute(
        #     "ChangeProperty", prop_path=Sdf.Path("/OmniverseKit_Persp.verticalAperture"), value=6, prev=0
        # )
        omni.kit.commands.execute(
            "ChangeProperty", prop_path=Sdf.Path("/OmniverseKit_Persp.horizontalAperture"), value=6, prev=0
        )

        await omni.kit.app.get_app().next_update_async()
        matrix = get_intrinsics_matrix(viewport_api)

        # Square pixel, fy = fx
        self.assertAlmostEqual(matrix[0, 0], 2419, delta=1)
        self.assertAlmostEqual(matrix[0, 2], 400, delta=1)
        self.assertAlmostEqual(matrix[1, 1], 2419, delta=1)
        self.assertAlmostEqual(matrix[1, 2], 300, delta=1)
        pass

    async def test_set_intrinsics(self):
        viewport_api = get_active_viewport()
        viewport_api.set_texture_resolution((800, 600))
        await omni.kit.app.get_app().next_update_async()
        matrix = get_intrinsics_matrix(viewport_api)

        # fx, fy should be the same (square pixels)
        matrix = np.array([[3871, 0.0, 400.0], [0.0, 3871, 300.0], [0.0, 0.0, 1.0]])
        set_intrinsics_matrix(viewport_api, matrix)
        await omni.kit.app.get_app().next_update_async()
        matrix = get_intrinsics_matrix(viewport_api)
        self.assertAlmostEqual(matrix[0, 0], 3871, delta=1)
        self.assertAlmostEqual(matrix[0, 2], 400, delta=1)
        self.assertAlmostEqual(matrix[1, 1], 3871, delta=1)
        self.assertAlmostEqual(matrix[1, 2], 300, delta=1)
        pass

    async def test_get_viewport_names(self):
        self.assertEquals(len(get_viewport_names()), 1)
        await omni.kit.app.get_app().next_update_async()
        window_1 = create_viewport_window()
        await omni.kit.app.get_app().next_update_async()
        window_2 = create_viewport_window()
        await omni.kit.app.get_app().next_update_async()
        self.assertEquals(len(get_viewport_names()), 3)
        window_1.destroy()
        await omni.kit.app.get_app().next_update_async()
        window_2.destroy()
        await omni.kit.app.get_app().next_update_async()

    async def test_get_window_from_id(self):
        window_0 = get_active_viewport_window()
        await omni.kit.app.get_app().next_update_async()
        window_1 = create_viewport_window()
        await omni.kit.app.get_app().next_update_async()
        window_2 = create_viewport_window()
        await omni.kit.app.get_app().next_update_async()

        print(window_0.title, window_0.viewport_api.id)
        print(window_1.title, window_1.viewport_api.id)
        print(window_2.title, window_2.viewport_api.id)

        window_test = get_window_from_id(window_1.viewport_api.id)
        self.assertEquals(window_test.title, window_1.title)
        window_test = get_window_from_id(1000)
        self.assertIsNone(window_test)
        window_1.destroy()
        await omni.kit.app.get_app().next_update_async()
        window_2.destroy()
        await omni.kit.app.get_app().next_update_async()

    async def test_get_id_from_index(self):
        window_0 = get_active_viewport_window()
        await omni.kit.app.get_app().next_update_async()
        # get the first viewport and check if titles match
        window_test = get_window_from_id(get_id_from_index(0))
        self.assertEquals(window_test.title, window_0.title)
        # second viewport should not exist yet
        window_test = get_window_from_id(get_id_from_index(1))
        self.assertIsNone(window_test)
        # create second viewport
        window_1 = create_viewport_window()
        await omni.kit.app.get_app().next_update_async()

        window_test = get_window_from_id(get_id_from_index(1))
        self.assertIsNotNone(window_test)
        self.assertEquals(window_test.title, window_1.title)
        window_1.destroy()

    async def test_create_destroy_window(self):
        from omni.kit.viewport.utility import create_viewport_window

        window_1 = create_viewport_window()
        for i in range(10):
            await omni.kit.app.get_app().next_update_async()
        window_1.destroy()
        await omni.kit.app.get_app().next_update_async()

    async def test_destroy_windows(self):
        from omni.kit.viewport.utility import create_viewport_window

        window_1 = create_viewport_window()
        window_2 = create_viewport_window()
        window_3 = create_viewport_window()
        window_4 = create_viewport_window()
        for i in range(10):
            await omni.kit.app.get_app().next_update_async()

        destroy_all_viewports(None, False)

    async def test_set_camera_view_perspective(self):
        import numpy as np

        viewport_api = get_active_viewport()
        camera_prim_path = "/OmniverseKit_Persp"
        prim = viewport_api.stage.GetPrimAtPath(camera_prim_path)
        rotate_prop = prim.GetAttribute("xformOp:rotateXYZ")
        pos_prop = prim.GetAttribute("xformOp:translate")

        set_camera_view(eye=np.array([0, 0, 0]), target=np.array([0, 0, 10]))
        await omni.kit.app.get_app().next_update_async()
        self.assertEqual(pos_prop.Get(), (0, 0, 0))
        self.assertEqual(rotate_prop.Get(), (0, 180, 0))

        set_camera_view(eye=np.array([5, 5, 10]), target=np.array([5, 5, 0]))
        await omni.kit.app.get_app().next_update_async()
        self.assertEqual(pos_prop.Get(), (5, 5, 10))
        self.assertEqual(rotate_prop.Get(), (0, 0, 0))

        set_camera_view(eye=np.array([5, 5, 5]), target=np.array([0, 0, 0]))
        await omni.kit.app.get_app().next_update_async()
        self.assertEqual(pos_prop.Get(), (5, 5, 5))
        self.assertAlmostEqual(rotate_prop.Get()[0], np.rad2deg(np.arctan(2**0.5)), delta=0.01)
        self.assertAlmostEqual(rotate_prop.Get()[1], 0, delta=0.01)
        self.assertAlmostEqual(rotate_prop.Get()[2], 135, delta=0.01)

        set_camera_view(eye=np.array([0, 0, 0]), target=np.array([0, 0, 1]), camera_prim_path="/OmniverseKit_Persp")
        set_camera_view(eye=np.array([0, 0, 0]), target=np.array([0, 0, 1]), camera_prim_path="/OmniverseKit_Top")
        set_camera_view(eye=np.array([0, 0, 0]), target=np.array([0, 0, 1]), camera_prim_path="/OmniverseKit_Right")
        set_camera_view(eye=np.array([0, 0, 0]), target=np.array([0, 0, 1]), camera_prim_path="/OmniverseKit_Front")

    async def test_set_camera_view_camera_prim(self):
        # create a camera prim using omni.isaac.sensor
        import numpy as np
        from omni.isaac.sensor import Camera
        from pxr import Gf, UsdGeom

        viewport_api = get_active_viewport()
        camera_prim_path = "/Camera"
        camera = Camera(prim_path=camera_prim_path, render_product_path=viewport_api.get_render_product_path())

        # TODO 106: calling initialize crashes, disabling because its not necessary for the test
        # camera.initialize()
        await omni.kit.app.get_app().next_update_async()
        prim = viewport_api.stage.GetPrimAtPath(camera_prim_path)
        rotate_prop = prim.GetAttribute("xformOp:orient")
        pos_prop = prim.GetAttribute("xformOp:translate")

        set_camera_view(eye=np.array([0, 0, 0]), target=np.array([0, 0, 10]), camera_prim_path=camera_prim_path)
        await omni.kit.app.get_app().next_update_async()
        self.assertEqual(pos_prop.Get(), (0, 0, 0))
        self.assertEqual(rotate_prop.Get(), Gf.Quatd(0, Gf.Vec3d(0, 1, 0)))

        set_camera_view(eye=np.array([5, 5, 10]), target=np.array([5, 5, 0]), camera_prim_path=camera_prim_path)
        await omni.kit.app.get_app().next_update_async()
        self.assertEqual(pos_prop.Get(), (5, 5, 10))
        self.assertEqual(rotate_prop.Get(), Gf.Quatd(1, Gf.Vec3d(0, 0, 0)))

        set_camera_view(eye=np.array([5, 5, 5]), target=np.array([0, 0, 0]), camera_prim_path=camera_prim_path)
        await omni.kit.app.get_app().next_update_async()
        self.assertEqual(pos_prop.Get(), (5, 5, 5))
        self.assertAlmostEqual(rotate_prop.Get().GetReal(), 0.340, delta=0.01)
        self.assertAlmostEqual(rotate_prop.Get().GetImaginary()[0], 0.176, delta=0.01)
        self.assertAlmostEqual(rotate_prop.Get().GetImaginary()[1], 0.425, delta=0.01)
        self.assertAlmostEqual(rotate_prop.Get().GetImaginary()[2], 0.820, delta=0.01)

        camera = None

        stage = omni.usd.get_context().get_stage()
        camera_rot_xyz = stage.DefinePrim("/World/Camera_RotXYZ", "Camera")
        UsdGeom.Xformable(camera_rot_xyz).AddTranslateOp().Set((0.0, 0.0, 0.0))
        UsdGeom.Xformable(camera_rot_xyz).AddRotateXYZOp().Set((0.0, 0.0, 0.0))

        camera_rot_yxz = stage.DefinePrim("/World/Camera_RotYXZ", "Camera")
        UsdGeom.Xformable(camera_rot_yxz).AddTranslateOp().Set((0.0, 0.0, 1.0))
        UsdGeom.Xformable(camera_rot_yxz).AddRotateYXZOp().Set((0.0, 0.0, 0.0))

        camera_orient = stage.DefinePrim("/World/Camera_Orient", "Camera")
        UsdGeom.Xformable(camera_orient).AddTranslateOp().Set((0.0, 0.0, 2.0))
        UsdGeom.Xformable(camera_orient).AddOrientOp().Set(Gf.Quatf(1.0, 0.0, 0.0, 0.0))

        set_camera_view(eye=np.array([0, 0, 0]), target=np.array([0, 0, 1]), camera_prim_path=camera_rot_xyz.GetPath())
        set_camera_view(eye=np.array([0, 0, 0]), target=np.array([0, 0, 1]), camera_prim_path=camera_rot_yxz.GetPath())
        set_camera_view(eye=np.array([0, 0, 0]), target=np.array([0, 0, 1]), camera_prim_path=camera_orient.GetPath())
