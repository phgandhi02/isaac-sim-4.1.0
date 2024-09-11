# Copyright (c) 2022-2023, NVIDIA CORPORATION. All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto. Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.
#

# import stable baselines
import carb

try:
    from stable_baselines3 import PPO
except Exception as e:
    carb.log_error(e)
    carb.log_error(
        "please install stable-baselines3 in the current python environment or run the following to install into the builtin python environment ./python.sh -m pip install stable-baselines3 "
    )
    exit()

# create isaac environment
from omni.isaac.gym.vec_env import VecEnvBase

env = VecEnvBase(headless=False)

# create task and register task
from cartpole_task import CartpoleTask

task = CartpoleTask(name="Cartpole")
env.set_task(task, backend="torch")

# Run inference on the trained policy
model = PPO.load("ppo_cartpole")
env._world.reset()
obs, _ = env.reset()
while env._simulation_app.is_running():
    action, _states = model.predict(obs)
    obs, rewards, terminated, truncated, info = env.step(action)

env.close()
