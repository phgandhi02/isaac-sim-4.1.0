# Copyright (c) 2018-2024, NVIDIA CORPORATION. All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto. Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.
#

import omni.ext
import omni.kit.commands
from omni.isaac.wheeled_robots.bindings._omni_isaac_wheeled_robots import acquire_interface, release_interface


class Extension(omni.ext.IExt):
    def on_startup(self, ext_id: str):
        # we need to acquire the interface to actually load the plugin, otherwise the DifferentialController can't be found
        self.__interface = acquire_interface()

    def on_shutdown(self):
        release_interface(self.__interface)
