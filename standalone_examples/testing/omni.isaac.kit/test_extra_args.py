# Copyright (c) 2020-2023, NVIDIA CORPORATION. All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto. Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.
#

from isaacsim import SimulationApp

# The most basic usage for creating a simulation app
kit = SimulationApp()

import carb

kit.update()

server_check = carb.settings.get_settings().get_as_string("/persistent/isaac/asset_root/default")

if server_check != "omniverse://ov-test-this-is-working":
    raise ValueError(f"isaac nucleus default setting not omniverse://ov-test-this-is-working, instead: {server_check}")


kit.close()  # Cleanup application
