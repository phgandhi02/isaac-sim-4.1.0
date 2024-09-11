# Copyright (c) 2018-2023, NVIDIA CORPORATION. All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto. Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.
#

import omni.ext

from .. import _range_sensor


class Extension(omni.ext.IExt):
    def on_startup(self, ext_id: str):
        self._lidar = _range_sensor.acquire_lidar_sensor_interface()
        self._ultrasonic = _range_sensor.acquire_ultrasonic_sensor_interface()
        self._generic = _range_sensor.acquire_generic_sensor_interface()

    def on_shutdown(self):
        _range_sensor.release_lidar_sensor_interface(self._lidar)
        _range_sensor.release_ultrasonic_sensor_interface(self._ultrasonic)
        _range_sensor.release_generic_sensor_interface(self._generic)
