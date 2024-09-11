# Copyright (c) 2021-2023, NVIDIA CORPORATION. All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto. Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.
#

from isaacsim import SimulationApp

simulation_app = SimulationApp({"headless": True})

from omni.isaac.core import SimulationContext
from omni.isaac.core.articulations import Articulation
from omni.isaac.core.utils.stage import add_reference_to_stage
from omni.isaac.nucleus import get_assets_root_path

assets_root_path = get_assets_root_path()
asset_path = assets_root_path + "/Isaac/Robots/Franka/franka_alt_fingers.usd"

simulation_context = SimulationContext()
add_reference_to_stage(asset_path, "/Franka")

# need to initialize physics getting any articulation..etc
simulation_context.initialize_physics()
art = Articulation("/Franka")
art.initialize()
dof_ptr = art.get_dof_index("panda_joint2")

simulation_context.play()


def step_callback_1(step_size):
    art.set_joint_positions([-1.5], [dof_ptr])
    return


def step_callback_2(step_size):
    print(
        "Current joint 2 position @ step "
        + str(simulation_context.current_time_step_index)
        + " : "
        + str(art.get_joint_positions([dof_ptr])[0])
    )
    print("TIME: ", simulation_context.current_time)
    return


def render_callback(event):
    print("Render Frame")


simulation_context.add_physics_callback("physics_callback_1", step_callback_1)
simulation_context.add_physics_callback("physics_callback_2", step_callback_2)
simulation_context.add_render_callback("render_callback", render_callback)
# Simulate 60 timesteps
for i in range(60):
    print("step", i)
    simulation_context.step(render=False)
# Render one frame
simulation_context.render()

simulation_context.stop()
simulation_app.close()
