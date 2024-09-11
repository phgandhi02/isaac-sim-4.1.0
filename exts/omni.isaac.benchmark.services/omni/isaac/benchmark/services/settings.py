# Copyright (c) 2024, NVIDIA CORPORATION. All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto. Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.
#
import json
import os
import sys
from dataclasses import asdict, dataclass, field
from typing import List, Optional, Type

import carb
import omni.client
import omni.kit.test

from . import utils

logger = utils.set_up_logging(__name__)


@dataclass
class BenchmarkSettings:
    """
    Settings for a specific Benchmark (which may generate many metrics)

    This class also acts as an interface, i.e can be augmented for specific use cases
    """

    # BASIC SETTINGS
    # Required name for test.
    test_name: str = ""
    # Render resolution.
    image_width: int = 1280
    image_height: int = 720
    # HDR setting.
    hdr: bool = False
    # Download a local copy of the stage to open.
    local: bool = True
    # Force creates a new golden image to upload to the server.
    overwrite_golden_image: bool = False
    # Stage to load. Format: omniverse://{server}/{path}/{stage.usd}.
    stage_path: str = ""
    # Set to True if stage contains MDLs (required for ASYNC; not required for SYNC).
    mdl: bool = True
    # Number of ASSETS_LOADED events if mdl is True (required for ASYNC; not required for SYNC).
    num_assets_loaded: int = 1
    # Seconds before Kit will close if stage did not load correctly.
    load_timeout: int = 300
    # Seconds to wait after scene is loaded.
    wait_after_load: float = 5
    # Path to camera to use after stage load.
    camera_path: str = ""
    # Seconds to sample FPS, CPU trace, and autoplay time. Not used if there is scope or autoplay_scene_length.
    sample_time: int = 10
    # Play scene for sample_time (10s default).
    autoplay: bool = False
    # Play scene for entire scene length.
    autoplay_scene_length: bool = False
    # DSSIM threshold value. Increased default from 0.01 to 0.08 since tests are mainly for performance.
    dssim_threshold: float = 0.08
    # Enables legacy FPS and Frame Time capture for backwards compatibility.
    legacy_fps_and_frametime: bool = True

    # OMNIGRAPH SPEICIFC
    timeline_fast_mode: bool = False

    # DATARECODERS
    # Recorders that start at the start of the test and ends at the end of the test.
    full_session_recorders: List[str] = field(
        default_factory=lambda: [
            "ActivityMonitor",
        ]
    )
    # Recorders that start before the scene loads and ends before the scene loads.
    scene_initial_recorders: List[str] = field(default_factory=lambda: [])
    # Recorders for frametime that start before the scene loads and ends before the scene loads.
    initial_frame_update_loop_recorders: List[str] = field(default_factory=lambda: ["IsaacFrametimeRecorder"])
    # Recorders for frametime that start after the scene loads while idle and ends based on settings.
    frame_update_loop_recorders: List[str] = field(default_factory=lambda: ["IsaacFrametimeRecorder"])
    # Recorders that start within the frame_update_loop_recorders (above) and starts before the frametime loop and
    # ends after the frametime loop. For example, enabling CarbTracingProfiler to capture only animated portions of the
    # scene.
    inner_frame_update_loop_recorders: List[str] = field(default_factory=lambda: ["CarbTracingProfiler"])
    # Recorders that start after frametime capture is complete.
    scene_post_idle_time_recorders: List[str] = field(
        default_factory=lambda: [
            "MemoryRecorder",
            "CPUStatsRecorder",
        ]
    )

    def __post_init__(self):
        """
        we read /exts/omni.kit.tests.benchmark/defaults from the current extension and see if it contains
        any overrides for any of the default values above, if so we use them.

        These can be override in the [[test]] of each benchmark.

        So - 3 levels - 1. defaults in code above 2. defaults in setttings of extension 3. explicit value in [[test]]
        """
        field_dict = asdict(self)
        settings = carb.settings.get_settings()
        default_overrides = settings.get_settings_dictionary("/exts/omni.kit.tests.benchmark/defaults")
        if default_overrides:
            for k, v in default_overrides.get_dict().items():
                if k in field_dict:
                    setattr(self, k, v)


def get_benchmark_settings_from_extension(
    extension_dotted_name: str, extension_info_dict=None, settings_cls: Type[BenchmarkSettings] = BenchmarkSettings
) -> List[List[BenchmarkSettings]]:
    """Read the configuration data for the benchmarks from the [[test]] section of the extension.toml
    extension_info_dict can be passed in if you already have details of the extension.

    Args:
        extension_dotted_name (str): The extension dotted name, for example omni.kit.tests.benchmark_large_scenes"
        extension_info_dict (dict): Overrides the extension configuration
        settings_cls (BenchmarkSettings): A class for the settings, usually the default but can be subclassed

    Returns:
        list: All benchmark settings found
    """
    all_settings = []
    if not extension_info_dict:
        manager = omni.kit.app.get_app().get_extension_manager()  # type: ignore
        ext_id = manager.get_extension_id_by_module(extension_dotted_name)
        extension_info_dict = manager.get_extension_dict(ext_id)
        if not extension_info_dict:
            logger.error(f"Couldn't find extension info for {extension_dotted_name} {ext_id}")
            return all_settings

    test_settings = extension_info_dict["test"]
    for setting in test_settings:
        # Converts each config file to an BenchmarkTestSettings object.
        settings_inst = settings_cls()
        for key, value in setting.items():
            if hasattr(settings_inst, key):
                setattr(settings_inst, key, value)
        settings_inst.test_name = setting["name"]
        if not settings_inst.test_name or not settings_inst.stage_path:
            logger.error(f"Couldn't find test_name/stage_path attributes for test {setting} skipping")
        else:
            logger.debug(f"found test {settings_inst.test_name}")
            # NOTE: Note sure why we're returning a list of a list
            all_settings.append([settings_inst])
    return all_settings


def generate_cls_name_from_settings(cls, num, params_dict) -> str:
    """Get name for unittest from test_name or stage_path if test_name is blank."""
    test_name = params_dict["settings"].test_name
    if test_name == "":
        stage_path = params_dict["settings"].stage_path
        filename = stage_path.split("/")[-1]
        path_name = os.path.splitext(filename)[0]
        return path_name
    else:
        return test_name


"""Legacy functions below, when we used to get our config from Nucleus JSON instead of the extension TOML"""


def get_all_nucleus_config_files(root_folder: str) -> List[str]:
    """Get list of all config files from given root folder."""
    if not root_folder.endswith("/"):
        root_folder += "/"

    logger.info(f"get_all_nucleus_config_files: Root folder = {root_folder}")
    all_stage_folders = []  # List of all stage folders.
    all_config_files = []  # List of all config files.

    # Find all stage folders in the root_folder and add them to all_stage_folders.
    result, stage_folder_list = omni.client.list(root_folder)
    if result == omni.client.Result.OK:
        for stage_name in stage_folder_list:
            stage_folder = root_folder + stage_name.relative_path + "/"
            all_stage_folders.append(stage_folder)

    # Find all config files in all_stage_folders and add them to all_config_files.
    for stage_folder in all_stage_folders:
        result, stage_files_list = omni.client.list(stage_folder)
        if result == omni.client.Result.OK:
            for file in stage_files_list:
                if ".json" in file.relative_path:
                    config_file = stage_folder + file.relative_path
                    all_config_files.append(config_file)

    logger.info(f"get_all_nucleus_config_files: Returning {len(all_config_files)} results")
    return all_config_files


def _read_json_config_file_from_nucleus(
    remote_path: str, settings_cls: Type[BenchmarkSettings]
) -> Optional[BenchmarkSettings]:
    """Get BenchmarkTestSettings from given remote path config file."""
    result, _, content = omni.client.read_file(remote_path)
    settings_cls_instance = None
    if result == omni.client.Result.OK:
        # Use json to read file content.
        test_list = json.loads(memoryview(content).tobytes())["test_list"]
        for test in test_list:
            settings_cls_instance = settings_cls()
            for key in test:
                value = test[key]
                setattr(settings_cls_instance, key, value)
        logger.debug(
            f"_read_json_config_file_from_nucleus: Created BenchmarkTestSettings {settings_cls_instance.test_name}"
        )
        return settings_cls_instance

    logger.warning("_read_json_config_file_from_nucleus: Failed to load config file.")
    return settings_cls_instance


def get_benchmark_settings_from_nucleus_config(
    settings_cls: Type[BenchmarkSettings] = BenchmarkSettings,
) -> List[List[BenchmarkSettings]]:
    """
    Given a root scene folder provided via a setting, read the configuration data from a .json folder
    found in each subdirectory

    """
    all_settings = []
    # Get a list of all config files in the root_folder.
    settings = carb.settings.get_settings()
    root_folder = settings.get("/exts/omni.kit.tests.benchmark/inputs/root_scene_folder")

    all_configs = get_all_nucleus_config_files(root_folder)

    for config in all_configs:
        # Converts each config file to an BenchmarkTestSettings object.
        settings = _read_json_config_file_from_nucleus(config, settings_cls)
        all_settings.append([settings])
    return all_settings


def get_benchmark_settings_from_json_file(
    self, config_path: str, settings_cls: Type[BenchmarkSettings]
) -> BenchmarkSettings:
    """
    This returns settings from a single JSON File
    """
    with open(config_path, "r") as config:
        test_list = json.load(config)["test_list"]
    settings_inst = settings_cls()
    for test in test_list:
        for key in test:
            value = test[key]
            setattr(settings_inst, key, value)
    return settings_inst
