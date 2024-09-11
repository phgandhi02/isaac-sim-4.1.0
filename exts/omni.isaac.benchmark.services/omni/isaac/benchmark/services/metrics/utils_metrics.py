# Copyright (c) 2024, NVIDIA CORPORATION. All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto. Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.
#
import calendar
import copy
import hashlib
import json
import os
import pathlib
import platform
import subprocess
import time
import uuid
import xml.etree.ElementTree as ET
from dataclasses import asdict, dataclass
from datetime import datetime
from getpass import getuser
from socket import gethostname
from typing import Any, Dict, List, Optional, Tuple

import carb
import omni.kit
import psutil
import yaml
from omni.isaac.benchmark.services import execution
from omni.isaac.benchmark.services.metrics.schemas import (
    GPU,
    Application,
    BenchData,
    Benchmark,
    BenchmarkIdentifier,
    CPUConfiguration,
    ExecutionEnvironment,
    GPUConfiguration,
    HardwareConfiguration,
    MemoryConfiguration,
    Metric,
    OSConfiguration,
)

from .. import utils

logger = utils.set_up_logging(__name__)


def get_execution_environment() -> Tuple[str, str]:
    """Creates a source and a build id for metrics api"""
    nvm_task_id = os.getenv("NVM_TASK_ID")
    if nvm_task_id is None:
        return "Other", datetime.utcnow().isoformat()
    return "GtlTask", nvm_task_id


def get_execution_purpose() -> str:
    """Why are we executing  this run? Bisect, production, local test etc?"""
    env_type = ""
    exec_env = execution.TestExecutionEnvironment.get_instance()
    if isinstance(exec_env, execution.LocalExecutionEnvironment):
        env_type = "local"
    elif isinstance(exec_env, execution.ETMExecutionEnvironment):
        env_type = os.getenv("KB2_PURPOSE", "")
        # backward compatibility
        if not env_type:
            env_type = os.getenv("ETM_PURPOSE", "")
    elif isinstance(exec_env, execution.TeamCityExecutionEnvironment):
        env_type = os.getenv("KB2_PURPOSE", "")
    return env_type


def get_kit_info() -> Tuple[str, str, str]:
    """Get Kit information.

    # Example:
    # full_branch = 103.1-release-Windows-10 or 103.1-release-Linux-20
    # kit_build_version = 103.1+release.1979.086b7ede.tc
    # kit_build_number = 1979
    """
    kit_build_version = omni.kit.app.get_app().get_build_version()
    kit_build_number = kit_build_version.rsplit(".", 3)[1]
    version = kit_build_version.split("+")[0]
    branch = kit_build_version.split("+")[1].split(".")[0]

    operating_system = platform.system()
    operating_system_version = "Unknown"
    if operating_system == "Windows":
        operating_system_version = platform.version().split(".")[0]
    elif operating_system == "Linux":
        # Supported for Ubuntu. Not tested for CentOS.
        # Doesn't work on TC machines anymore
        try:
            operating_system_version = platform.version().split("~")[1].split(".")[0]
        except Exception:
            pass

    full_branch = version + "-" + branch + "-" + operating_system + "-" + operating_system_version
    return full_branch, kit_build_version, kit_build_number


def get_app_info() -> Tuple[str, str, str]:
    """Get App name and version.

    # Example:
    # app_name = kit or Create.Next
    # app_version = 104.0+master or 2022.3.0-alpha.62+daily
    # app_build = 4336.9b7f4dfd.tc
    """
    settings = carb.settings.get_settings()
    app_name = settings.get("/app/name")
    yaml_contents = get_package_info_yaml()
    try:
        ci_build_number = yaml_contents["CI Build Number"]
        app_version = ci_build_number.rsplit(".", 3)[0]
        app_build = ".".join(ci_build_number.rsplit(".", 3)[1:4])
        return app_name, app_version, app_build
    except Exception as e:
        logger.warn(f"Unable to find app_version and/or app_build. {str(e)}")
        return app_name, "Unknown", "Unknown"


def get_kit_build_date() -> Optional[int]:
    """Get the Kit build date from PACKAGE-INFO.yaml or Kit executable if not found."""
    yaml_contents = get_package_info_yaml()
    try:
        time = yaml_contents["Time"]
        logger.info(f"PACKAGE-INFO.yaml Time = {time}")
        # Convert time from "Wed May 25 13:38:50 2022" to UNIX epoch time in millisecond.
        epoch_time_ms = int(calendar.timegm(datetime.strptime(time, "%a %b %d %H:%M:%S %Y").timetuple()) * 1000)
        logger.info(f"PACKAGE-INFO.yaml Epoch Time (ms) = {epoch_time_ms}")
        return epoch_time_ms
    except Exception as e:
        logger.warn(f"Unable to find Time with PACKAGE-INFO.yaml. {str(e)}")
        return None


def get_package_info_yaml(yaml_path=None) -> dict:
    """Get PACKAGE-INFO.yaml contents."""
    etm_active = isinstance(execution.TestExecutionEnvironment.get_instance(), execution.ETMExecutionEnvironment)

    if yaml_path is None:
        if etm_active:
            yaml_path = str(pathlib.Path().resolve() / "PACKAGE-INFO.yaml")
        else:
            import carb.tokens

            yaml_path = str(pathlib.Path(carb.tokens.get_tokens_interface().resolve("${kit}")) / "PACKAGE-INFO.yaml")

    yaml_contents = {}
    try:
        logger.info(f"PACKAGE-INFO.yaml path = {yaml_path}")
        with open(yaml_path, "r") as f:
            yaml_contents = yaml.safe_load(f)
            logger.info(f"PACKAGE-INFO.yaml contents: {yaml_contents}")
    except Exception as e:
        logger.warn(f"Unable to find PACKAGE-INFO.yaml. {str(e)}")

    return yaml_contents


_run_uuid = uuid.uuid4().hex


def get_run_uuid():
    """
    If we're in a test subproc, then run_uuid was already set in parent proc.

    If we're the parent kit proc, we set the global run uuid here
    """
    settings = carb.settings.get_settings()
    run_uuid = settings.get("/exts/omni.kit.tests.benchmark/metrics/run_uuid")
    return run_uuid or _run_uuid


def get_telem_logdir(metrics_output_dir, test_name):
    return os.path.join(metrics_output_dir, "telem_logs", test_name)
