# Copyright (c) 2021-2023, NVIDIA CORPORATION. All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto. Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.
#

import builtins
import os
import sys

import carb

from .app_framework import AppFramework
from .simulation_app import SimulationApp

# check for isaacsim module
check_for_isaacsim_module = False
try:
    import omni.kit.app

    omni.kit.app.get_app()
except RuntimeError:
    check_for_isaacsim_module = True

if check_for_isaacsim_module and "isaacsim" not in sys.modules:
    import traceback

    carb.log_warn("")
    carb.log_warn("=========================== DEPRECATION WARNING ===========================")
    carb.log_warn("")
    stack = traceback.extract_stack()
    if stack:
        carb.log_warn("Traceback (most recent call last):")
        for item in traceback.extract_stack():
            if "<frozen" in item.filename:
                continue
            elif "omni.isaac.kit/omni/isaac/kit" in item.filename:
                break
            carb.log_warn(f"File '{item.filename}', line {item.lineno}")
            if item.line is not None:
                carb.log_warn(f"  {item.line}")
        carb.log_warn("")
    carb.log_warn("Please import 'isaacsim' before importing the 'omni.isaac.kit' extension...")
    carb.log_warn("  >>> import isaacsim")
    carb.log_warn("  >>> from omni.isaac.kit import SimulationApp")
    carb.log_warn("")
    carb.log_warn("or get the 'SimulationApp' class directly from the 'isaacsim' module")
    carb.log_warn("  >>> from isaacsim import SimulationApp")
    carb.log_warn("")
    carb.log_warn("===========================================================================")
    carb.log_warn("")

builtins.ISAAC_LAUNCHED_FROM_JUPYTER = (
    os.getenv("ISAAC_JUPYTER_KERNEL") is not None or os.getenv("ISAAC_JUPYTER_PYTHON_PACKAGE") is not None
)  # We set this in the kernel.json file or in the isaac-sim-kernel package

if builtins.ISAAC_LAUNCHED_FROM_JUPYTER:
    import nest_asyncio

    nest_asyncio.apply()
else:
    # Do a sanity check to see if we are running in an ipython env
    try:
        get_ipython()
        carb.log_warn(
            "Interactive python shell detected but ISAAC_JUPYTER_KERNEL was not set. Problems with asyncio may occur"
        )
        carb.log_warn("Please use Isaac Sim Python 3 kernel instead of the default Python 3 Kernel")
    except Exception:
        # We are probably not in an interactive shell
        pass
