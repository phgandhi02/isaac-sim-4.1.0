# Copyright (c) 2024, NVIDIA CORPORATION. All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto. Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.
#

import argparse

parser = argparse.ArgumentParser()
parser.add_argument("--num-robots", type=int, default=1, help="Number of robots")
parser.add_argument(
    "--enable-3d-lidar", type=int, default=0, choices=range(0, 1 + 1), help="Number of 3D lidars to enable, per robot."
)
parser.add_argument(
    "--enable-2d-lidar", type=int, default=0, choices=range(0, 2 + 1), help="Number of 2D lidars to enable, per robot."
)
parser.add_argument(
    "--enable-hawks",
    type=int,
    default=0,
    choices=range(0, 4 + 1),
    help="Number of Hawk camera stereo pairs to enable, per robot.",
)
parser.add_argument("--num-gpus", type=int, default=1, help="Number of GPUs on machine.")
parser.add_argument("--num-frames", type=int, default=600, help="Number of frames to run benchmark for")
parser.add_argument(
    "--backend-type",
    default="OsmoKPIFile",
    choices=["LocalLogMetrics", "JSONFileMetrics", "OsmoKPIFile"],
    help="Benchmarking backend, defaults",
)

args, unknown = parser.parse_known_args()

n_robot = args.num_robots
enable_3d_lidar = args.enable_3d_lidar
enable_2d_lidar = args.enable_2d_lidar
enable_hawks = args.enable_hawks
n_gpu = args.num_gpus
n_frames = args.num_frames

import numpy as np
from isaacsim import SimulationApp

simulation_app = SimulationApp({"headless": True, "max_gpu_count": n_gpu})


import carb
import omni
import omni.graph.core as og
import omni.kit.test
from omni.isaac.core import PhysicsContext
from omni.isaac.core.utils.extensions import enable_extension
from omni.isaac.core.utils.types import ArticulationAction
from omni.isaac.core.utils.viewports import set_camera_view
from omni.isaac.wheeled_robots.robots import WheeledRobot

enable_extension("omni.isaac.benchmark.services")
from omni.isaac.benchmark.services import BaseIsaacBenchmark

# Create the benchmark
benchmark = BaseIsaacBenchmark(
    benchmark_name="benchmark_robots_nova_carter_ros2",
    workflow_metadata={
        "metadata": [
            {"name": "num_hawks", "data": enable_hawks},
            {"name": "num_2d_lidars", "data": enable_2d_lidar},
            {"name": "num_3d_lidars", "data": enable_3d_lidar},
            {"name": "num_robots", "data": n_robot},
            {"name": "num_gpus", "data": n_gpu},
        ]
    },
    backend_type=args.backend_type,
)
benchmark.set_phase("loading", start_recording_frametime=False, start_recording_runtime=True)

enable_extension("omni.isaac.ros2_bridge")
omni.kit.app.get_app().update()

robot_path = "/Isaac/Samples/ROS2/Robots/Nova_Carter_ROS.usd"
scene_path = "/Isaac/Environments/Simple_Warehouse/full_warehouse.usd"

benchmark.fully_load_stage(benchmark.assets_root_path + scene_path)

stage = omni.usd.get_context().get_stage()
PhysicsContext(physics_dt=1.0 / 60.0)
set_camera_view(eye=[-6, -15.5, 6.5], target=[-6, 10.5, -1], camera_prim_path="/OmniverseKit_Persp")

lidars_2d = ["/front_2d_lidar_render_product", "/back_2d_lidar_render_product"]
hawk_actiongraphs = ["/front_hawk", "/left_hawk", "/right_hawk", "/back_hawk"]

robots = []
for i in range(n_robot):
    robot_prim_path = "/Robots/Robot_" + str(i)
    robot_usd_path = benchmark.assets_root_path + robot_path
    # position the robot robot
    MAX_IN_LINE = 10
    robot_position = np.array([-2 * (i % MAX_IN_LINE), -2 * np.floor(i / MAX_IN_LINE), 0])
    current_robot = WheeledRobot(
        prim_path=robot_prim_path,
        wheel_dof_names=["joint_wheel_left", "joint_wheel_right"],
        create_robot=True,
        usd_path=robot_usd_path,
        position=robot_position,
    )

    omni.kit.app.get_app().update()
    omni.kit.app.get_app().update()

    for i in range(len(lidars_2d)):
        if i < enable_2d_lidar:
            og.Controller.attribute(robot_prim_path + "/ros_lidars" + lidars_2d[i] + ".inputs:enabled").set(True)
        else:
            og.Controller.attribute(robot_prim_path + "/ros_lidars" + lidars_2d[i] + ".inputs:enabled").set(False)

    if enable_3d_lidar > 0:
        og.Controller.attribute(robot_prim_path + "/ros_lidars/front_3d_lidar_render_product.inputs:enabled").set(True)
    else:
        og.Controller.attribute(robot_prim_path + "/ros_lidars/front_3d_lidar_render_product.inputs:enabled").set(False)

    for i in range(len(hawk_actiongraphs)):
        if i < enable_hawks:
            og.Controller.attribute(
                robot_prim_path + hawk_actiongraphs[i] + "/left_camera_render_product" + ".inputs:enabled"
            ).set(True)
            og.Controller.attribute(
                robot_prim_path + hawk_actiongraphs[i] + "/right_camera_render_product" + ".inputs:enabled"
            ).set(True)
        else:
            og.Controller.attribute(
                robot_prim_path + hawk_actiongraphs[i] + "/left_camera_render_product" + ".inputs:enabled"
            ).set(False)
            og.Controller.attribute(
                robot_prim_path + hawk_actiongraphs[i] + "/right_camera_render_product" + ".inputs:enabled"
            ).set(False)

    robots.append(current_robot)

# Set this to true so that we always publish regardless of subscribers
carb.settings.get_settings().set_bool("/exts/omni.isaac.ros2_bridge/publish_without_verification", True)

timeline = omni.timeline.get_timeline_interface()
timeline.play()
omni.kit.app.get_app().update()

for robot in robots:
    robot.initialize()
    # start the robot rotating in place so not to run into each
    robot.apply_wheel_actions(
        ArticulationAction(joint_positions=None, joint_efforts=None, joint_velocities=5 * np.array([0, 1]))
    )

omni.kit.app.get_app().update()
omni.kit.app.get_app().update()

benchmark.store_measurements()
# perform benchmark
benchmark.set_phase("benchmark")


for _ in range(1, n_frames):
    omni.kit.app.get_app().update()

benchmark.store_measurements()
benchmark.stop()

timeline.stop()
simulation_app.close()
