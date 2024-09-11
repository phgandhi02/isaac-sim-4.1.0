# Copyright (c) 2020-2024, NVIDIA CORPORATION. All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto. Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.
#


import time

import omni.graph.core as og
import omni.kit.commands
import omni.usd
from pxr import Gf, UsdGeom


def add_physx_lidar(prim_path, translation=Gf.Vec3f(0, 0, 0), orientation=Gf.Vec4f(0, 0, 0, 0)):
    _, lidar = omni.kit.commands.execute(
        "RangeSensorCreateLidar",
        path=prim_path,
        parent=None,
        min_range=0.4,
        max_range=100.0,
        draw_points=True,
        draw_lines=True,
        horizontal_fov=360.0,
        vertical_fov=30.0,
        horizontal_resolution=0.4,
        vertical_resolution=4.0,
        rotation_rate=0.0,
        high_lod=False,
        yaw_offset=0.0,
    )
    lidar_prim = lidar.GetPrim()

    if "xformOp:translate" not in lidar_prim.GetPropertyNames():
        UsdGeom.Xformable(lidar_prim).AddTranslateOp()
    if "xformOp:orient" not in lidar_prim.GetPropertyNames():
        UsdGeom.Xformable(lidar_prim).AddOrientOp()

    lidar_prim.GetAttribute("xformOp:translate").Set(translation)
    lidar_prim.GetAttribute("xformOp:orient").Set(orientation)


def add_ros1_camera(render_product_path, graph_path, camera_topic, sim_camera_id, type="rgb"):
    (ros_camera_graph, _, _, _) = og.Controller.edit(
        {"graph_path": graph_path, "evaluator_name": "execution"},
        {
            og.Controller.Keys.CREATE_NODES: [
                ("OnTick", "omni.graph.action.OnTick"),
                ("cameraHelperRgb", "omni.isaac.ros_bridge.ROS1CameraHelper"),
            ],
            og.Controller.Keys.CONNECT: [
                ("OnTick.outputs:tick", "cameraHelperRgb.inputs:execIn"),
            ],
            og.Controller.Keys.SET_VALUES: [
                ("cameraHelperRgb.inputs:renderProductPath", render_product_path),
                ("cameraHelperRgb.inputs:frameId", sim_camera_id),
                ("cameraHelperRgb.inputs:topicName", camera_topic),
                ("cameraHelperRgb.inputs:type", type),
            ],
        },
    )

    return ros_camera_graph


def add_ros2_camera(render_product_path, graph_path, camera_topic, sim_camera_id, type="rgb"):
    (ros_camera_graph, _, _, _) = og.Controller.edit(
        {"graph_path": graph_path, "evaluator_name": "execution"},
        {
            og.Controller.Keys.CREATE_NODES: [
                ("OnTick", "omni.graph.action.OnTick"),
                ("cameraHelperRgb", "omni.isaac.ros2_bridge.ROS2CameraHelper"),
            ],
            og.Controller.Keys.CONNECT: [
                ("OnTick.outputs:tick", "cameraHelperRgb.inputs:execIn"),
            ],
            og.Controller.Keys.SET_VALUES: [
                ("cameraHelperRgb.inputs:renderProductPath", render_product_path),
                ("cameraHelperRgb.inputs:frameId", sim_camera_id),
                ("cameraHelperRgb.inputs:topicName", camera_topic),
                ("cameraHelperRgb.inputs:type", type),
            ],
        },
    )

    return ros_camera_graph
