# Copyright (c) 2018-2024, NVIDIA CORPORATION. All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto. Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.
#


import omni.ext
import omni.usd
from omni.isaac.ros2_bridge.scripts.og_shortcuts.og_rtx_sensors import Ros2CameraGraph, Ros2RtxLidarGraph
from omni.isaac.ros2_bridge.scripts.og_shortcuts.og_utils import (
    Ros2ClockGraph,
    Ros2GenericPubGraph,
    Ros2JointStatesGraph,
    Ros2OdometryGraph,
    Ros2TfPubGraph,
)
from omni.isaac.ui.menu import make_menu_item_description
from omni.kit.menu.utils import MenuItemDescription, add_menu_items, remove_menu_items


class Extension(omni.ext.IExt):
    def on_startup(self, ext_id: str):
        self.window_handle = None
        ros_og_menu = [
            make_menu_item_description(ext_id, "ROS2 Camera", onclick_fun=self._open_camera_sensor),
            make_menu_item_description(ext_id, "ROS2 RTX Lidar", onclick_fun=self._open_rtx_lidar_sensor),
            make_menu_item_description(ext_id, "ROS2 TF Publisher", onclick_fun=self._open_pub_tf),
            make_menu_item_description(ext_id, "ROS2 Odometry Publisher", onclick_fun=self._open_odometry_publisher),
            make_menu_item_description(ext_id, "ROS2 JointStates", onclick_fun=self._open_joint_states_pubsub),
            make_menu_item_description(ext_id, "ROS2 Clock", onclick_fun=self._open_clock),
            make_menu_item_description(ext_id, "ROS2 Generic Publisher", onclick_fun=self._open_rtf),
        ]

        self._menu_items = [
            MenuItemDescription(
                name="Common Omnigraphs",
                sub_menu=ros_og_menu,
            )
        ]

        add_menu_items(self._menu_items, "Isaac Utils")

    def on_shutdown(self):
        remove_menu_items(self._menu_items, "Isaac Utils")
        if self.window_handle:
            self.window_handle.visible = False

    def _open_clock(self):
        if self.window_handle:
            self.window_handle.visible = False
        clock_graph = Ros2ClockGraph()
        self.window_handle = clock_graph.create_clock_graph()

    def _open_rtf(self):
        if self.window_handle:
            self.window_handle.visible = False
        clock_graph = Ros2GenericPubGraph()
        self.window_handle = clock_graph.create_generic_pub_graph()

    def _open_camera_sensor(self):
        if self.window_handle:
            self.window_handle.visible = False
        camera_graph = Ros2CameraGraph()
        self.window_handle = camera_graph.create_camera_graph()

    def _open_rtx_lidar_sensor(self):
        if self.window_handle:
            self.window_handle.visible = False
        lidar_graph = Ros2RtxLidarGraph()
        self.window_handle = lidar_graph.create_lidar_graph()

    def _open_joint_states_pubsub(self):
        if self.window_handle:
            self.window_handle.visible = False
        js_graph = Ros2JointStatesGraph()
        self.window_handle = js_graph.create_jointstates_graph()

    def _open_pub_tf(self):
        if self.window_handle:
            self.window_handle.visible = False
        tf_pub_graph = Ros2TfPubGraph()
        self.window_handle = tf_pub_graph.create_tf_pub_graph()

    def _open_odometry_publisher(self):
        if self.window_handle:
            self.window_handle.visible = False
        odom_pub_graph = Ros2OdometryGraph()
        self.window_handle = odom_pub_graph.create_odometry_graph()
