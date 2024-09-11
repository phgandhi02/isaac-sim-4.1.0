# Copyright (c) 2018-2023, NVIDIA CORPORATION. All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto. Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.
#
import omni.ext
import omni.kit.commands

from ..bindings import _occupancy_map


class Extension(omni.ext.IExt):
    def on_startup(self, ext_id: str):
        self._interface = _occupancy_map.acquire_occupancy_map_interface()

    def on_shutdown(self):
        _occupancy_map.release_occupancy_map_interface(self._interface)
