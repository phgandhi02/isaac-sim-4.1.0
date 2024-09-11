# Copyright (c) 2018-2023, NVIDIA CORPORATION. All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto. Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.
#

from pathlib import Path

import carb
import carb.settings
import carb.tokens
import omni.kit.commands

# NOTE:
#   omni.kit.test - std python's unittest module with additional wrapping to add suport for async/await tests
#   For most things refer to unittest docs: https://docs.python.org/3/library/unittest.html
import omni.kit.test
import omni.usd
from omni.isaac.core.objects import DynamicCuboid
from omni.isaac.core.utils.prims import get_prim_path
from omni.isaac.core.utils.stage import clear_stage, create_new_stage, traverse_stage
from omni.isaac.core.utils.viewports import set_camera_view
from omni.isaac.range_sensor import _range_sensor
from omni.isaac.sensor import _sensor
from omni.kit.mainwindow import get_main_window
from omni.kit.ui_test.menu import *
from omni.kit.ui_test.query import *
from omni.kit.viewport.utility import get_active_viewport, get_active_viewport_window
from omni.ui.tests.test_base import OmniUiTest
from pxr import UsdGeom, UsdPhysics

EXTENSION_FOLDER_PATH = Path(omni.kit.app.get_app().get_extension_manager().get_extension_path_by_module(__name__))
TEST_DATA_PATH = EXTENSION_FOLDER_PATH.joinpath("data/tests")

# Having a test class dervived from omni.kit.test.AsyncTestCase declared on the root of module will make it auto-discoverable by omni.kit.test
# OmniUiTest is derived from omni.kit.test.AsyncTestCase
class TestMenuAssets(OmniUiTest):
    # Before running each test
    async def setUp(self):
        self._timeline = omni.timeline.get_timeline_interface()
        result = create_new_stage()
        # Make sure the stage loaded
        self.assertTrue(result)
        await omni.kit.app.get_app().next_update_async()
        self._golden_img_dir = TEST_DATA_PATH.absolute().joinpath("golden_img").absolute()

        self.usd_selection: omni.usd.Selection = omni.usd.get_context().get_selection()
        # list all the Isaac Sim's Robot assets' menu path
        window = get_main_window()
        self.menu_dict = await get_context_menu(window._ui_main_window.main_menu_bar, get_all=False)
        self.lidar_sensor_interface = _range_sensor.acquire_lidar_sensor_interface()
        self.ultrasonic_sensor_interface = _range_sensor.acquire_ultrasonic_sensor_interface()
        self.generic_sensor_interface = _range_sensor.acquire_generic_sensor_interface()
        self.contact_sensor_interface = _sensor.acquire_contact_sensor_interface()
        self.imu_sensor_interface = _sensor.acquire_imu_sensor_interface()
        self.lightbeam_sensor_interface = _sensor.acquire_lightbeam_sensor_interface()
        self.carb_settings = carb.settings.get_settings()
        self.carb_settings.set("/rtx/rendermode", "RayTracedLighting")
        self.carb_settings.set("/rtx-transient/resourcemanager/enableTextureStreaming", False)
        pass

    # After running each test
    async def tearDown(self):
        await omni.kit.app.get_app().next_update_async()
        # In some cases the test will end before the asset is loaded, in this case wait for assets to load
        while omni.usd.get_context().get_stage_loading_status()[2] > 0:
            await omni.kit.app.get_app().next_update_async()
        await super().tearDown()
        pass

    # Actual test, notice it is "async" function, so "await" can be used if needed
    async def test_loading_robots(self):

        self.robot_menu_dict = self.menu_dict["Create"]["Isaac"]["Robots"]
        self.ee_menu_dict = self.menu_dict["Create"]["Isaac"]["End Effectors"]

        ## check everything under "Robot"

        robot_root_path = "Create/Isaac/Robots"
        ee_root_path = "Create/Isaac/End Effectors"

        def get_menu_path(d, path, result, root_path):

            for key, value in d.items():
                if key != "_":
                    new_path = path + "/" + str(key)
                if isinstance(value, dict):
                    get_menu_path(value, new_path, result, root_path)
                elif isinstance(value, str):
                    result.append(root_path + path + "/" + str(value))
                elif isinstance(value, list):
                    for robot in value:
                        result.append(root_path + path + "/" + str(robot))
            return result

        empty_list = []
        empty_path = ""
        robot_menu_list = get_menu_path(self.robot_menu_dict, empty_path, empty_list, robot_root_path)
        empty_list = []
        empty_path = ""
        ee_menu_list = get_menu_path(self.ee_menu_dict, empty_path, empty_list, ee_root_path)

        test_list = robot_menu_list + ee_menu_list

        # surface gripper is just a graph
        skip_list = ["Create/Isaac/End Effectors/Surface Gripper"]

        failed_robots = []
        for test_path in test_list:
            print(test_path)
            if test_path in skip_list:
                print("skipping ", test_path)
                continue

            # for each item on the robot's asset path, load it and check if successful by checking if there is an articulation on stage
            clear_stage()
            await omni.kit.app.get_app().next_update_async()
            await omni.kit.app.get_app().next_update_async()
            has_robot = False
            await menu_click(test_path, human_delay_speed=10)
            for i in range(20):
                await omni.kit.app.get_app().next_update_async()

            # # waiting for stage to load
            while omni.usd.get_context().get_stage_loading_status()[2] > 0:
                await omni.kit.app.get_app().next_update_async()

            # # once stage loading finishes, check if there's an articulation root on stage
            for prim in traverse_stage():
                if prim.HasAPI(UsdPhysics.ArticulationRootAPI):
                    has_robot = True
                    prim_path = get_prim_path(prim)
                    print(f"articulation root found at {prim_path}")

            if not has_robot:
                print(f"failed to find articulation at {test_path}")
                failed_robots.append(test_path)

        print("Failed:", failed_robots)

        # if failed_robot array has 0 entries, then test passed
        self.assertEqual(len(failed_robots), 0)

    async def test_apriltag_menu(self):
        apriltag_root_path = "Create/Isaac/April Tag"

        def get_menu_path(d, path, result, root_path):

            for key, value in d.items():
                if key != "_":
                    new_path = path + "/" + str(key)
                if isinstance(value, dict):
                    get_menu_path(value, new_path, result, root_path)
                elif isinstance(value, str):
                    result.append(root_path + path + "/" + str(value))
                elif isinstance(value, list):
                    for robot in value:
                        result.append(root_path + path + "/" + str(robot))
            return result

        empty_list = []
        empty_path = ""
        apriltag_menu_list = get_menu_path(
            self.menu_dict["Create"]["Isaac"]["April Tag"], empty_path, empty_list, apriltag_root_path
        )

        # for each item on the robot's asset path, load it and check if successful by checking if there is an articulation on stage
        clear_stage()
        await omni.kit.app.get_app().next_update_async()
        await omni.kit.app.get_app().next_update_async()
        await menu_click(apriltag_menu_list[0], human_delay_speed=10)
        await omni.kit.app.get_app().next_update_async()

        omni.kit.commands.execute("CreateMeshPrimWithDefaultXform", prim_type="Cube", above_ground=True)

        await omni.kit.app.get_app().next_update_async()
        omni.kit.commands.execute(
            "BindMaterial", material_path="/Looks/AprilTag", prim_path=["/Cube"], strength=["weakerThanDescendants"]
        )

        await omni.kit.app.get_app().next_update_async()

    async def test_loading_environment(self):
        from omni.kit.viewport.utility.tests.capture import capture_viewport_and_wait, finalize_capture_and_compare

        self.environment_menu_dict = self.menu_dict["Create"]["Isaac"]["Environments"]
        ## check everything under "Environment"
        environment_root_path = "Create/Isaac/Environments"

        def get_menu_path(d, path, result, root_path):
            for key, value in d.items():
                if key != "_":
                    new_path = path + "/" + str(key)
                if isinstance(value, dict):
                    get_menu_path(value, new_path, result, root_path)
                elif isinstance(value, str):
                    result.append(root_path + path + "/" + str(value))
                elif isinstance(value, list):
                    for environment in value:
                        result.append(root_path + path + "/" + str(environment))
            return result

        empty_list = []
        skip_list = []
        failed_environments = []

        empty_path = ""
        environment_menu_list = get_menu_path(self.environment_menu_dict, empty_path, empty_list, environment_root_path)
        try:
            carb.windowing.acquire_windowing_interface()
        except:
            carb.log_error("unable to acquire windowing interface")
            return

        for test_path in environment_menu_list:
            if test_path in skip_list:
                print("skipping ", test_path)
                continue

            create_new_stage()
            await omni.kit.app.get_app().next_update_async()
            await omni.kit.app.get_app().next_update_async()
            await menu_click(test_path, human_delay_speed=10)
            for i in range(5):
                await omni.kit.app.get_app().next_update_async()

            # waiting for stage to load
            while omni.usd.get_context().get_stage_loading_status()[2] > 0:
                await omni.kit.app.get_app().next_update_async()

            await omni.kit.app.get_app().next_update_async()
            # the golden image test file name and test file name are the last section of the path
            golden_img_name = test_path.split("/")[-1] + ".png"
            viewport_api = get_active_viewport()
            viewport_api.resolution = (1280, 720)
            self.usd_selection.clear_selected_prim_paths()
            await omni.kit.app.get_app().next_update_async()
            # set camera position
            if "Office" in test_path or "Hospital" in test_path or "Warehouse" in test_path:
                set_camera_view(eye=[-4, 4, 2], target=[0, 0, 1])
            else:
                set_camera_view(eye=[3, -3, 3], target=[0, 0, 0])

            for i in range(200):
                await omni.kit.app.get_app().next_update_async()
            # capture and compare with golden image
            output_dir = Path(omni.kit.test.get_test_output_path())
            await capture_viewport_and_wait(image_name=golden_img_name, output_img_dir=output_dir, viewport=None)
            diff = finalize_capture_and_compare(
                image_name=golden_img_name,
                output_img_dir=output_dir,
                golden_img_dir=self._golden_img_dir,
                threshold=1000,
            )
            print("DIFF:", test_path, diff)
            self.assertIsNotNone(diff)
            self.assertLess(diff, 1000)

            # count the number of prims in the stage
            num_prims = 0
            for prim in traverse_stage():
                num_prims += 1

            # There should be at least 9 prims
            # `/Render/Vars/LdrColor` `/Render/Vars` `/Render/RenderProduct_omni_kit_widget_viewport_ViewportTexture_0`
            # `/Render` `/OmniverseKit_Right` `/OmniverseKit_Top` `/OmniverseKit_Front` `/OmniverseKit_Persp`
            # `at least 1 environment prim`

            if num_prims < 9:
                print(f"failed to find any prims at {test_path}")
                failed_environments.append(test_path)
        print(failed_environments)
        # if failed_environments array has 0 entries, then test passed
        self.assertEqual(len(failed_environments), 0)

    async def test_loading_sensors(self):
        self.sensor_menu_dict = self.menu_dict["Create"]["Isaac"]["Sensors"]
        sensor_root_path = "Create/Isaac/Sensors"

        def get_menu_path(d, path, result, root_path):
            for key, value in d.items():
                if key != "_":
                    new_path = path + "/" + str(key)
                if isinstance(value, dict):
                    get_menu_path(value, new_path, result, root_path)
                elif isinstance(value, str):
                    result.append(root_path + path + "/" + str(value))
                elif isinstance(value, list):
                    for sensor in value:
                        result.append(root_path + path + "/" + str(sensor))
            return result

        empty_list = []
        skip_list = ["Create/Isaac/Sensors/Ultrasonic/Emitter", "Create/Isaac/Sensors/Ultrasonic/FiringGroup"]
        failed_sensors = []

        # each type of sensor will get a different prim type test, RGBD and RTX will check for camera prim
        sensor_test_types = ["Rotating", "Generic", "Ultrasonic", "Contact", "Imu", "RGBD", "RTX", "LightBeam"]

        empty_path = ""
        sensor_menu_list = get_menu_path(self.sensor_menu_dict, empty_path, empty_list, sensor_root_path)

        for test_path in sensor_menu_list:
            if test_path in skip_list:
                print("skipping ", test_path)
                continue

            clear_stage()
            await omni.kit.app.get_app().next_update_async()
            await omni.kit.app.get_app().next_update_async()

            for test_type in sensor_test_types:
                if test_type in test_path:
                    sensor_test = test_type

            # IMU and contact sensors interface have to be attached to a rigid body parent to function
            if sensor_test == "Contact" or sensor_test == "Imu":
                cube = DynamicCuboid(prim_path="/Cube")
                self.usd_selection.set_selected_prim_paths(["/Cube"], True)
            print("clicking ", test_path)
            await menu_click(test_path, human_delay_speed=32)
            self._timeline.play()
            for i in range(20):
                await omni.kit.app.get_app().next_update_async()

            # # waiting for stage to load
            while omni.usd.get_context().get_stage_loading_status()[2] > 0:
                await omni.kit.app.get_app().next_update_async()

            num_prims = 0
            sensor_passed = False

            # count the number of prims on the stage, should be greater than 1
            # also for each prim, look for the matching type of sensor
            for prim in traverse_stage():
                num_prims += 1
                if sensor_test == "Rotating" and self.lidar_sensor_interface.is_lidar_sensor(get_prim_path(prim)):
                    sensor_passed = True

                elif sensor_test == "Generic" and self.generic_sensor_interface.is_generic_sensor(get_prim_path(prim)):
                    sensor_passed = True

                elif sensor_test == "Ultrasonic" and self.ultrasonic_sensor_interface.is_ultrasonic_sensor(
                    get_prim_path(prim)
                ):
                    sensor_passed = True

                elif sensor_test == "Contact" and self.contact_sensor_interface.is_contact_sensor(get_prim_path(prim)):
                    sensor_passed = True

                elif sensor_test == "Imu" and self.imu_sensor_interface.is_imu_sensor(get_prim_path(prim)):
                    sensor_passed = True

                elif (sensor_test == "RGBD" or sensor_test == "RTX") and prim.IsA(UsdGeom.Camera):
                    sensor_passed = True

                elif sensor_test == "LightBeam" and self.lightbeam_sensor_interface.is_lightbeam_sensor(
                    get_prim_path(prim)
                ):
                    sensor_passed = True

                if sensor_passed:
                    break

            self._timeline.stop()
            await omni.kit.app.get_app().next_update_async()
            # expected prims on stage
            # `/Render/Vars/LdrColor` `/Render/Vars` `/Render/RenderProduct_omni_kit_widget_viewport_ViewportTexture_0`
            # `/Render` `/OmniverseKit_Right` `/OmniverseKit_Top` `/OmniverseKit_Front` `/OmniverseKit_Persp`
            # `at least 1 Sensor prim`
            # if is_foo_sensor() is called on an non sensor prim, there maybe an error message generated that can be ignored

            if not sensor_passed:
                print(f"{test_path} did not pass, missing prim or wrong prim type")
                failed_sensors.append(test_path)

        print(failed_sensors)

        # if failed_sensors array has 0 entries, then test passed
        self.assertEqual(len(failed_sensors), 0)
