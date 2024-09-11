import argparse

parser = argparse.ArgumentParser()
parser.add_argument("--num-envs", type=int, default=1, help="Number of environments to clone.")
parser.add_argument("--num-gpus", type=int, default=1, help="Number of GPUs on machine.")
parser.add_argument(
    "--backend-type",
    default="OsmoKPIFile",
    choices=["LocalLogMetrics", "JSONFileMetrics", "OsmoKPIFile"],
    help="Benchmarking backend, defaults",
)

args, unknown = parser.parse_known_args()

n_envs = args.num_envs
n_gpu = args.num_gpus

from isaacsim import SimulationApp

simulation_app = SimulationApp({"headless": True})

from omni.isaac.core.utils.extensions import enable_extension

enable_extension("omni.isaac.benchmark.services")

import sys

import carb
import numpy as np
from omni.isaac.benchmark.services import BaseIsaacBenchmark
from omni.isaac.cloner import GridCloner
from omni.isaac.core import World
from omni.isaac.core.articulations import Articulation, ArticulationView
from omni.isaac.core.objects import DynamicCuboid, VisualCuboid
from omni.isaac.core.prims import GeometryPrimView, RigidPrimView, XFormPrim
from omni.isaac.core.utils.stage import add_reference_to_stage, get_stage_units
from omni.isaac.core.utils.types import ArticulationAction
from omni.isaac.nucleus import get_assets_root_path


def define_environment():
    XFormPrim(prim_path="/World/env_0", position=np.array([0.0, 0.0, 0.0]))
    cube_1 = VisualCuboid(
        prim_path="/World/env_0/new_cube_1",
        name="visual_cube",
        position=np.array([0, 0, 0.5]),
        size=0.3,
        color=np.array([255, 255, 255]),
    )

    cube_2 = DynamicCuboid(
        prim_path="/World/env_0/new_cube_2",
        name="cube_1",
        position=np.array([0, 0, 1.0]),
        scale=np.array([0.6, 0.5, 0.2]),
        size=1.0,
        color=np.array([255, 0, 0]),
    )

    cube_3 = DynamicCuboid(
        prim_path="/World/env_0/new_cube_3",
        name="cube_2",
        position=np.array([0, 0, 3.0]),
        scale=np.array([0.1, 0.1, 0.1]),
        size=1.0,
        color=np.array([0, 0, 255]),
        linear_velocity=np.array([0, 0, 0.4]),
    )

    assets_root_path = get_assets_root_path()
    if assets_root_path is None:
        carb.log_error("Could not find Isaac Sim assets folder")
        simulation_app.close()
        sys.exit()
    asset_path = assets_root_path + "/Isaac/Robots/Franka/franka_alt_fingers.usd"
    add_reference_to_stage(usd_path=asset_path, prim_path="/World/env_0/Franka_1")
    add_reference_to_stage(usd_path=asset_path, prim_path="/World/env_0/Franka_2")
    XFormPrim(prim_path="/World/env_0/Franka_1", name="my_franka_1", position=np.array([0.0, 2.0, 0.0]))
    XFormPrim(prim_path="/World/env_0/Franka_2", name="my_franka_2", position=np.array([0.0, -2.0, 0.0]))


def clone_environments():
    cloner = GridCloner(spacing=1)
    cloner.define_base_env("/World")
    prim_paths = cloner.generate_paths("/World/env", n_envs)
    cloner.clone(source_prim_path="/World/env_0", prim_paths=prim_paths, replicate_physics=True, copy_from_source=False)


def create_cube_views():
    my_world.scene.add(
        GeometryPrimView(
            prim_paths_expr="/World/env_*/new_cube_1",
            name="visual_cube_view",
        )
    )
    my_world.scene.add(RigidPrimView(prim_paths_expr="/World/env_*/new_cube_2", name="rigid_cube_view_1"))
    my_world.scene.add(RigidPrimView(prim_paths_expr="/World/env_*/new_cube_3", name="rigid_cube_view_2"))


def create_articulation_views():
    my_world.scene.add(
        ArticulationView(
            prim_paths_expr="/World/env_*/Franka_1",
            name="articulation_view_1",
        )
    )
    my_world.scene.add(
        ArticulationView(
            prim_paths_expr="/World/env_*/Franka_2",
            name="articulation_view_2",
        )
    )


# Create the benchmark
benchmark = BaseIsaacBenchmark(
    benchmark_name="benchmark_world",
    workflow_metadata={"metadata": []},
    backend_type=args.backend_type,
)

benchmark.set_phase("world_creation", start_recording_frametime=False, start_recording_runtime=True)
my_world = World(stage_units_in_meters=1.0)
benchmark.store_measurements()

my_world.scene.add_default_ground_plane()


benchmark.set_phase("env_creation", start_recording_frametime=False, start_recording_runtime=True)
define_environment()
benchmark.store_measurements()

benchmark.set_phase("env_cloning", start_recording_frametime=False, start_recording_runtime=True)
clone_environments()
benchmark.store_measurements()

benchmark.set_phase("cube_views_creation", start_recording_frametime=False, start_recording_runtime=True)
create_cube_views()
benchmark.store_measurements()

benchmark.set_phase("articulation_views_creation", start_recording_frametime=False, start_recording_runtime=True)
create_articulation_views()
benchmark.store_measurements()

benchmark.set_phase("get_world_pose_articulation_no_sim", start_recording_frametime=False, start_recording_runtime=True)
articulation_view_1 = my_world.scene.get_object("articulation_view_1")
articulation_view_1.get_world_poses()
benchmark.store_measurements()

benchmark.set_phase("world_resetting", start_recording_frametime=False, start_recording_runtime=True)
my_world.reset()
benchmark.store_measurements()

benchmark.set_phase("world_step_w_render", start_recording_frametime=True, start_recording_runtime=False)
for i in range(100):
    articulation_view_1.set_joint_position_targets(positions=np.random.randn(n_envs, 9))
    my_world.step(render=True)
benchmark.store_measurements()

benchmark.set_phase("world_step_no_render", start_recording_frametime=True, start_recording_runtime=False)
for i in range(100):
    articulation_view_1.set_joint_position_targets(positions=np.random.randn(n_envs, 9))
    my_world.step(render=False)
benchmark.store_measurements()

benchmark.set_phase("get_world_pose_articulation_w_sim", start_recording_frametime=False, start_recording_runtime=True)
articulation_view_1.get_world_poses()
benchmark.store_measurements()


benchmark.stop()
simulation_app.close()
