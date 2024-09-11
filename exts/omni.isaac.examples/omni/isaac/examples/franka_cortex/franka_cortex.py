# Copyright (c) 2022-2023, NVIDIA CORPORATION. All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto. Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.
#


import carb
import numpy as np
import omni
from omni.isaac.core.objects import DynamicCuboid, VisualCuboid
from omni.isaac.cortex.cortex_utils import load_behavior_module
from omni.isaac.cortex.cortex_world import Behavior, CortexWorld, LogicalStateMonitor
from omni.isaac.cortex.dfb import DfDiagnosticsMonitor
from omni.isaac.cortex.robot import CortexFranka, add_franka_to_stage
from omni.isaac.cortex.tools import SteadyRate
from omni.isaac.examples.cortex.cortex_base import CortexBase


class CubeSpec:
    def __init__(self, name, color):
        self.name = name
        self.color = np.array(color)


class ContextStateMonitor(DfDiagnosticsMonitor):
    """
    State monitor to read the context and pass it to the UI.
    For these behaviors, the context has a `diagnostic_message` that contains the text to be displayed, and each
    behavior implements its own monitor to update that.

    """

    def __init__(self, print_dt, diagnostic_fn=None):
        super().__init__(print_dt=print_dt)
        self.diagnostic_fn = diagnostic_fn

    def print_diagnostics(self, context):
        if self.diagnostic_fn:
            self.diagnostic_fn(context)


class FrankaCortex(CortexBase):
    def __init__(self, monitor_fn=None):
        super().__init__()
        self._monitor_fn = monitor_fn
        self.behavior = None
        self.robot = None
        self.context_monitor = ContextStateMonitor(print_dt=0.25, diagnostic_fn=self._on_monitor_update)

    def setup_scene(self):
        world = self.get_world()
        self.robot = world.add_robot(add_franka_to_stage(name="franka", prim_path="/World/Franka"))

        obs_specs = [
            CubeSpec("RedCube", [0.7, 0.0, 0.0]),
            CubeSpec("BlueCube", [0.0, 0.0, 0.7]),
            CubeSpec("YellowCube", [0.7, 0.7, 0.0]),
            CubeSpec("GreenCube", [0.0, 0.7, 0.0]),
        ]
        width = 0.0515
        for i, (x, spec) in enumerate(zip(np.linspace(0.3, 0.7, len(obs_specs)), obs_specs)):
            obj = world.scene.add(
                DynamicCuboid(
                    prim_path="/World/Obs/{}".format(spec.name),
                    name=spec.name,
                    size=width,
                    color=spec.color,
                    position=np.array([x, -0.4, width / 2]),
                )
            )
            self.robot.register_obstacle(obj)
        world.scene.add_default_ground_plane()

    async def load_behavior(self, behavior):
        world = self.get_world()
        self.behavior = behavior
        self.decider_network = load_behavior_module(self.behavior).make_decider_network(self.robot)
        self.decider_network.context.add_monitor(self.context_monitor.monitor)
        world.add_decider_network(self.decider_network)

    def clear_behavior(self):
        world = self.get_world()
        world._logical_state_monitors.clear()
        world._behaviors.clear()

    async def setup_post_load(self, soft=False):
        world = self.get_world()
        prim_path = "/World/Franka"
        if not self.robot:
            self.robot = world._robots["franka"]
        self.decider_network = load_behavior_module(self.behavior).make_decider_network(self.robot)
        self.decider_network.context.add_monitor(self.context_monitor.monitor)
        world.add_decider_network(self.decider_network)
        await omni.kit.app.get_app().next_update_async()

    def _on_monitor_update(self, context):
        diagnostic = ""
        decision_stack = ""
        if hasattr(context, "diagnostics_message"):
            diagnostic = context.diagnostics_message
        if self.decider_network._decider_state.stack:
            decision_stack = "\n".join(
                [
                    "{0}{1}".format("  " * i, element)
                    for i, element in enumerate(str(i) for i in self.decider_network._decider_state.stack)
                ]
            )

        if self._monitor_fn:
            self._monitor_fn(diagnostic, decision_stack)

    def _on_physics_step(self, step_size):
        world = self.get_world()

        world.step(False, False)

    async def on_event_async(self):
        world = self.get_world()
        await omni.kit.app.get_app().next_update_async()
        world.reset_cortex()
        world.add_physics_callback("sim_step", self._on_physics_step)
        await world.play_async()

    async def setup_pre_reset(self):
        world = self.get_world()
        if world.physics_callback_exists("sim_step"):
            world.remove_physics_callback("sim_step")

    def world_cleanup(self):
        pass
