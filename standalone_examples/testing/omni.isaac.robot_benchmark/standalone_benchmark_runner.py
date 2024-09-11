# Copyright (c) 2021-2023, NVIDIA CORPORATION. All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto. Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.
#

import argparse
import json
import os
import signal
import time

import carb
import numpy as np

_omni = None
_kit = None


"""
This script acts as an example for how one can run the benchmark extension from a script.
This affords multiple advantages:
    Benchmarks run much faster from this script because most of Sim is not loaded.
        (once the assets are loaded and kit has started up, which can take a while the first time)
    Environment and MotionPolicy parameters can be changed programmatically between tests.
    Running from a script enables logging of test data for quantitative analysis

To run this script with Isaac Sim's python environment:
    /path/to/sim/_build/linux-x86_64/release/python.sh standalone_benchmark_runner.py {args}

Run with -h to see command line arguments.

----------------------------------------------------------

This script will run every permutation of Environments, Robots, and Controllers that are passed in by name as 
command line arguments.  Any combinations that are not possible (becuase a controller hasn't been implemented for a
specific robot) will be skipped and will not be found in the logged file

RobotBenchmark writes logs to a json file such that:
    After each test concludes with reaching the final waypoint or timing out:
        A Header is written for the test with
            environment, robot, and controller names
            Success or failure of test
            Index of test
        
        A test includes a list of logged dictionaries with information about every frame:
            frame_number
            robot cspace position
            end effector position
            target position
    
    To see the exact information logged in RobotBenchmark, look at the functions
    robot_benchmark._log_header() and robot_benchmark._log_frame()

There is not currently a way to modify what gets logged except to directly change those functions

---------------------------------------------------------

If the user navigates to robot_benchmark/user_template/example.py and sets RUN_EXAMPLE=True, they can run this script using 
the Example Robot and Example Controller with the line:

/path/to/sim/_build/linux-x86_64/release/python.sh source/extensions/omni.isaac.robot_benchmark/omni/isaac/robot_benchmark/standalone_benchmark_runner.py --env_names "Cubby" -r "Example Robot" -p "Example Controller" -n 1

This will run a single trial in the Cubby environment using the "Example Robot" (A floating Franka) and the "Example Controller (RMPflow under another name)"
"""


def get_robot_options(benchmark_robot_registry, env_creator, env_name):
    """
    Given the environment selected, return a list of the robots that have at least one
    motion policy configured in the motion_generation extension, and are not explicitly excluded for this
    environment
    """
    robot_exclusion_list = env_creator.get_robot_exclusion_list(env_name)

    all_robot_options = benchmark_robot_registry.get_robot_options()

    robot_options = []
    for op in all_robot_options:
        if op not in robot_exclusion_list:
            robot_options.append(op)
    return robot_options


def get_motion_controller_options(benchmark_robot_registry, env_creator, env_name, robot_name):
    """
    Given the robot selected, return the motion policies that have default configs
    for the robot in the motion_generation extension
    """

    controller_exclusion_list = env_creator.get_motion_policy_exclusion_list(env_name)

    all_controller_options = benchmark_robot_registry.get_robot_loader(robot_name).get_benchmark_controller_names()

    controller_options = []
    for op in all_controller_options:
        if op not in controller_exclusion_list:
            controller_options.append(op)
    return controller_options


def get_environment_params(env_name, robot_name):
    # Loads robot-specific parameters (if any) for an environment that are stored in the ../benchmark_config directory
    # For example, the Cubby environment is rotated for the UR10

    benchmark_config_dir = os.path.join(_benchmark_extension_path, "benchmark_config")
    with open(os.path.join(benchmark_config_dir, "benchmark_config_map.json")) as benchmark_config_map:
        benchmark_config_map = json.load(benchmark_config_map)

    local_env_config_path = (
        benchmark_config_map.get(robot_name, {}).get("environment_config_paths", {}).get(env_name, None)
    )
    if local_env_config_path is None:
        return {}

    env_config_path = os.path.join(benchmark_config_dir, local_env_config_path)

    if os.path.exists(env_config_path):
        with open(env_config_path) as env_file:
            config = json.load(env_file)
    else:
        carb.log_error(
            "Invalid path to config file specified in benchmark_config_map.json for the "
            + robot_name
            + " in the "
            + env_name
            + "environment"
        )
        config = {}

    return config


def run_benchmark(env_name, robot_name, policy_name, num_trials, benchmark_logger, start_ind):
    from omni.isaac.benchmark_environments.environments import EnvironmentCreator
    from omni.isaac.core import World
    from omni.isaac.core.utils.viewports import set_camera_view
    from omni.isaac.robot_benchmark.benchmark_robots import BenchmarkRobotRegistry
    from omni.isaac.robot_benchmark.robot_benchmarking import RobotBenchmark

    print(
        "Running Benchmark: ({env_name},{robot_name},{policy_name})".format(
            env_name=env_name, robot_name=robot_name, policy_name=policy_name
        )
    )

    _omni.usd.get_context().new_stage()

    _world = World(stage_units_in_meters=1.0)
    _world.reset()

    benchmark_robot_registry = BenchmarkRobotRegistry()
    env_creator = EnvironmentCreator()

    env_kwargs = get_environment_params(env_name, robot_name)
    env = env_creator.create_environment(env_name, **env_kwargs)

    if robot_name not in get_robot_options(benchmark_robot_registry, env_creator, env_name):
        carb.log_warning(
            "Robot "
            + robot_name
            + " is not compatible with environment "
            + env_name
            + " and will be skipped in this environment"
        )
        return

    robot_loader = benchmark_robot_registry.get_robot_loader(robot_name)

    if policy_name not in get_motion_controller_options(benchmark_robot_registry, env_creator, env_name, robot_name):
        carb.log_warning(
            "Policy "
            + policy_name
            + " is not compatible with robot,environment pair ("
            + env_name
            + ","
            + robot_name
            + ") and will be skipped"
        )
        return

    benchmark = RobotBenchmark()

    set_camera_view(eye=env.camera_position, target=env.camera_target, camera_prim_path="/OmniverseKit_Persp")

    benchmark.initialize_test(env, robot_loader, policy_name, benchmark_logger=benchmark_logger, start_ind=start_ind)

    _world.step(render=_render)
    benchmark.step(1.0 / _fps)
    benchmark.toggle_testing()

    while benchmark.test_ind < num_trials:
        _world.step(render=_render)
        benchmark.step(1.0 / _fps)
        if not _simulation_app.is_running():
            _simulation_app.close()

    benchmark_logger.write_to_json()


def main(args):
    CUSTOM_CONFIG = {
        "renderer": "RayTracedLighting",
        "headless": args.headless,
        "experience": f'{os.environ["EXP_PATH"]}/omni.isaac.sim.python.kit',
    }
    from isaacsim import SimulationApp

    simulation_app = SimulationApp(CUSTOM_CONFIG)

    import omni
    from omni.isaac.core import World

    # make kit and omni globals to make calling functions cleaner
    global _simulation_app, _world, _omni, _render
    _simulation_app = simulation_app
    # _world = world
    _omni = omni
    _render = not args.headless

    from omni.isaac.robot_benchmark.benchmark_logger import BenchmarkLogger

    ext_manager = omni.kit.app.get_app().get_extension_manager()
    ext_manager.set_extension_enabled_immediate("omni.isaac.robot_benchmark", True)
    ext_id = ext_manager.get_enabled_extension_id("omni.isaac.robot_benchmark")
    benchmark_extension_path = ext_manager.get_extension_path(ext_id)

    global _benchmark_extension_path, _fps
    _benchmark_extension_path = benchmark_extension_path
    _fps = 60

    benchmark_logger = BenchmarkLogger(args.abs_path)

    for env_name in args.env_names:
        for robot_name in args.robot_names:
            for policy_name in args.policy_names:
                run_benchmark(env_name, robot_name, policy_name, args.num_trials, benchmark_logger, args.start_ind)

    simulation_app.close()


if __name__ == "__main__":

    parser = argparse.ArgumentParser()

    parser.add_argument("-H", "--headless", help="run in headless mode (no GUI)", action="store_true", default=False)
    parser.add_argument(
        "abs_path",
        type=str,
        help="Absolute path to output log file. Using '~' in your absolute path will refernce the omni_isaac_sim directory on your machine rather than the home directory",
    )

    parser.add_argument(
        "-env",
        "--env_names",
        type=str,
        nargs="+",
        default=["Cubby", "Static Cage"],
        help="name of environments to run",
        dest="env_names",
    )

    parser.add_argument(
        "-r",
        "--robot_names",
        type=str,
        nargs="+",
        default=["Franka"],
        help="name of robots to run in each environment",
        dest="robot_names",
    )

    parser.add_argument(
        "-p",
        "--policy_names",
        type=str,
        nargs="+",
        default=["RMPflow", "RRT+RMP Taskspace Carrot"],
        dest="policy_names",
        help="name of policies to run on each robot in each environment",
    )

    parser.add_argument(
        "-n",
        "--num_trials",
        type=int,
        default=30,
        dest="num_trials",
        help="number of trials to run for each (environment,robot,policy) pair",
    )

    parser.add_argument(
        "-s",
        "--start_ind",
        type=int,
        default=0,
        dest="start_ind",
        help="Index of trial on which to start.  Starting on trial 3 with --num_trials = 5 would result in trials 3 and 4 being run.",
    )

    args = parser.parse_known_args()

    main(args)
