# Copyright (c) 2024, NVIDIA CORPORATION. All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto. Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.
#
import asyncio
import dataclasses
import functools
import inspect
import logging
import os
import platform
import stat
import sys
import time
import traceback
import urllib
from enum import Enum, auto
from pathlib import Path
from typing import List, Tuple, Union

import carb
import omni.kit.app

original_persistent_settings = {}
settings_interface = None


logger = logging.getLogger(__name__)


def set_up_logging(name) -> logging.Logger:
    """Set up logger."""
    fmt = "{asctime} [{relativeCreated:,.0f}ms] [{levelname}] [{name}] {message}"
    datfmt = "%Y-%m-%d %H:%M:%S"
    style = "{"
    formatter = logging.Formatter(fmt, datfmt, style)
    formatter.converter = time.gmtime
    stdout_handler = logging.StreamHandler(sys.stdout)
    stdout_handler.setFormatter(formatter)
    logger = logging.getLogger(name)
    logger.handlers = [stdout_handler]
    logger.propagate = False
    logger.setLevel(logging.INFO)
    return logger


def set_persistent_setting(name, value, type):
    """
    NOTE: looks like this is not used in practice?
    """
    global original_persistent_settings, settings_interface

    _orig = settings_interface.get(name)  # noqa
    original_persistent_settings[name] = {"value": _orig, "type": type}

    _set_settings_value(name, value, type)


def restore_persistent_settings():
    for name, _dict in original_persistent_settings.items():
        _set_settings_value(name, _dict["value"], _dict["type"])


def _set_settings_value(name, value, type):
    global settings_interface
    settings_interface.set(name, value)


@functools.lru_cache(maxsize=10)
def generate_event_map():
    event_map = {}
    for _val in dir(omni.usd.StageEventType):
        if _val.isupper():
            event_map[int(getattr(omni.usd.StageEventType, _val))] = _val
    return event_map


async def stage_event() -> int:
    """Calls `kit.stage_event` with logging"""
    result = await omni.usd.get_context().next_stage_event_async()
    event, _ = result
    event = int(event)
    carb.log_info(f"*** omni.kit.tests.basic_validation: stage_event() -> ({generate_event_map()[event]}, {_})")
    return event


async def capture_next_frame(app, capture_file_path: str):
    """
    capture that works with old (editor-based) capture and new Kit 2.0 approach also
    Not all Create's seem to have the new API available (e.g "omni.create.kit")
    """

    _renderer = None
    _viewport_interface = None

    try:
        import omni.kit.viewport_legacy
        import omni.renderer_capture
    except ImportError as ie:
        carb.log_error(f"*** screenshot: capture_next_frame: can't load {ie}")

    _renderer = omni.renderer_capture.acquire_renderer_capture_interface()
    _viewport_interface = omni.kit.viewport_legacy.acquire_viewport_interface()
    viewport_ldr_rp = _viewport_interface.get_viewport_window(None).get_drawable_ldr_resource()

    # TODO: Probably need to put a cap on this so doesnt hang forever
    # Wait until the viewport has valid resources
    while viewport_ldr_rp == None:
        await app.next_update_async()
        viewport_ldr_rp = _viewport_interface.get_viewport_window(None).get_drawable_ldr_resource()

    _renderer.capture_next_frame_rp_resource(capture_file_path, viewport_ldr_rp)
    await app.next_update_async()
    _renderer.wait_async_capture()
    print("written", capture_file_path)


def omni_url_parser(url: str):
    res = urllib.parse.urlparse(url)
    username = os.getenv("OMNI_USER", default="test")
    password = os.getenv("OMNI_PASS", default=username)
    return res.netloc, username, password, res.path


async def load_stage(stage_path: str, syncloads: bool, num_assets_loaded: int = 2):
    """
    looks like this is not being used (in benchmark at least)
    """

    start = time.time()
    success, explanation = await omni.usd.get_context().open_stage_async(stage_path)
    carb.log_info(f"*** omni.kit.tests.basic_validation: Initial stage load success: {success}")
    if not success:
        raise RuntimeError(explanation)

    # we'll try to track all the ASSETS_LOADED events to figure out when the MDLs
    # are complete
    assets_loaded_count = 0
    required_assets_loaded = 1
    if not syncloads:
        required_assets_loaded = int(num_assets_loaded)

    if required_assets_loaded == 0:
        load_time = time.time() - start
        carb.log_info("*** omni.kit.tests.basic_validation: Not waiting for ASSETS LOADED at all, stage load complete.")
        return load_time

    carb.log_info(f"*** omni.kit.tests.basic_validation: Waiting for {required_assets_loaded} ASSETS LOADED event(s)")
    while True:
        event = await stage_event()
        # TODO: compare to actual enum value when Kit fixes its return types
        if event == int(omni.usd.StageEventType.ASSETS_LOADED):
            assets_loaded_count += 1
            carb.log_info(f"*** omni.kit.tests.basic_validation: Received ASSETS_LOADED #{assets_loaded_count}")
            # The user can specify how many assets_loaded to wait for in async mode
            if assets_loaded_count < required_assets_loaded:
                continue
            carb.log_info(
                f"*** omni.kit.tests.basic_validation: Met threshold of {required_assets_loaded}, all assets loaded"
            )
            break
        # error that something went wrong
        elif event == int(omni.usd.StageEventType.OPEN_FAILED):
            raise RuntimeError("Received OPEN_FAILED")
        elif event == int(omni.usd.StageEventType.ASSETS_LOAD_ABORTED):
            raise RuntimeError("Received ASSETS_LOAD_ABORTED")
        elif event == int(omni.usd.StageEventType.CLOSING):
            raise SystemExit("Received CLOSING")
        elif event == int(omni.usd.StageEventType.CLOSED):
            raise SystemExit("Received CLOSED")

    load_time = time.time() - start
    return load_time


def getStageDefaultPrimPath(stage):
    """
    Helper function used for getting default prim path for any given stage.
    """
    if stage.HasDefaultPrim():
        return stage.GetDefaultPrim().GetPath()
    else:
        from pxr import Sdf

        return Sdf.Path.absoluteRootPath


class LogErrorChecker:
    """Automatically subscribes to logging events and monitors if error were produced during the test."""

    def __init__(self):
        # Setup this test case to fail if any error is produced
        self._error_count = 0

        def on_log_event(e):
            if e.payload["level"] >= carb.logging.LEVEL_ERROR:
                self._error_count = self._error_count + 1

        self._log_stream = omni.kit.app.get_app().get_log_event_stream()
        self._log_sub = self._log_stream.create_subscription_to_pop(on_log_event, name="test log event")

    def shutdown(self):
        self._log_stream = None
        self._log_sub = None

    def get_error_count(self):
        self._log_stream.pump()
        return self._error_count


def get_calling_test_id() -> str:
    """
    starting with the current stack frame, see if we are being called by
    a unit test. If so, try and find the fully qualified name of the test

    NOTE: This relies on conventions and a bit of hackery. Don't rely on
    it 100%
    """

    def get_class_from_frame(fr):
        args, _, _, value_dict = inspect.getargvalues(fr)
        # we check the first parameter for the frame function is
        # named 'self'
        if len(args) and args[0] == "self":
            # in that case, 'self' will be referenced in value_dict
            instance = value_dict.get("self", None)
            if instance:
                # return its class
                class_name = getattr(instance, "__class__", None)
                # print("get_class_from_frame, returning", class_name)
                return class_name
        # return None otherwise
        # print("get_class_from_frame, returning none")
        return None

    for frame, _ in traceback.walk_stack(None):
        func_name = frame.f_code.co_name
        if func_name.startswith("test_"):
            cls = get_class_from_frame(frame)
            if cls:
                name = f"{cls.__module__}.{cls.__name__}.{func_name}"
                return name
            break
    return ""


def ensure_dir(file_path: Union[str, Path]) -> None:
    """Creates directory if it doesn't exist."""
    if not os.path.exists(file_path):
        logger.info(f"Creating dir {file_path}")
        os.makedirs(file_path)


def get_kit_version_branch() -> Tuple[str, str, str]:
    """Get Kit branch in the format of version_branch"""
    # Example:
    # 103.1+release.1979.086b7ede.tc -> 103.1_release
    app = omni.kit.app.get_app()
    build_version = app.get_build_version()
    version = build_version.split("+")[0]
    branch = build_version.split("+")[1].split(".")[0]
    version_branch = version + "_" + branch
    return version, branch, version_branch


class SyncMode(Enum):
    SYNC = auto()
    ASYNC = auto()
    AMBIGUOUS = auto()


class SampledScope:
    """Sample metrics during the lifetime of a context."""

    def __init__(self):
        loop = asyncio.get_event_loop()
        self.f_start = loop.create_future()
        self.f_end = loop.create_future()

    def done(self):
        return self.f_end.done()

    def wait_until_started(self):
        self.f_start

    async def wait_until_ended(self):
        await self.f_end

    def __enter__(self):
        self.f_start.set_result(True)
        logger.info("scene scope started")
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        self.f_end.set_result(True)
        logger.info("scene scope stopped")


# Run a given number of app updates after loading a stage to fully loaded materials/textures and co.
# early stop if a frame time threshold (frametime_threshold) is reached
# or if the time ratio (time_ratio_treshold) between the current and the previous frame is reached
# e.g. current frame needed X times less time than the previous one
async def wait_until_stage_is_fully_loaded_async(
    max_frames=10, frametime_threshold=0.1, time_ratio_treshold=5, verbose=False
):
    prev_frametime = 0
    for i in range(max_frames):
        start_time = time.time()
        await omni.kit.app.get_app().next_update_async()
        elapsed_time = time.time() - start_time
        if verbose:
            print(f"Frame {i} frametime: {elapsed_time}")
        if elapsed_time < frametime_threshold or elapsed_time * time_ratio_treshold < prev_frametime:
            if verbose:
                print(f"Stage fully loaded at frame {i}, last frametime: {elapsed_time}")
            break
        prev_frametime = elapsed_time


# Run a given number of app updates after loading a stage to fully loaded materials/textures and co.
# early stop if a frame time threshold (frametime_threshold) is reached
# or if the time ratio (time_ratio_treshold) between the current and the previous frame is reached
# e.g. current frame needed X times less time than the previous one
def wait_until_stage_is_fully_loaded(max_frames=10, frametime_threshold=0.1, time_ratio_treshold=5, verbose=False):
    prev_frametime = 0
    for i in range(max_frames):
        start_time = time.time()
        omni.kit.app.get_app().update()
        elapsed_time = time.time() - start_time
        if verbose:
            print(f"Frame {i} frametime: {elapsed_time}")
        if elapsed_time < frametime_threshold or elapsed_time * time_ratio_treshold < prev_frametime:
            if verbose:
                print(f"Stage fully loaded at frame {i}, last frametime: {elapsed_time}")
            break
        prev_frametime = elapsed_time
