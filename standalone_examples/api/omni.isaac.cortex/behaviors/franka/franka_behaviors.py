# Copyright (c) 2023, NVIDIA CORPORATION. All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto. Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.
#
from omni.isaac.cortex.dfb import DfDiagnosticsMonitor
from omni.isaac.cortex.sample_behaviors.franka import (
    block_stacking_behavior,
    peck_decider_network,
    peck_game,
    peck_state_machine,
)
from omni.isaac.cortex.sample_behaviors.franka.simple import simple_decider_network, simple_state_machine

behaviors = {
    "block_stacking_behavior": block_stacking_behavior,
    "peck_decider_network": peck_decider_network,
    "peck_game": peck_game,
    "peck_state_machine": peck_state_machine,
    "simple_decider_network": simple_decider_network,
    "simple_state_machine": simple_state_machine,
}


class ContextStateMonitor(DfDiagnosticsMonitor):
    """
    State monitor to read the context and pass it to the UI.
    For these behaviors, the context has a `diagnostic_message` that contains the text to be displayed, and each
    behavior implements its own monitor to update that.

    """

    def __init__(self, print_dt, diagnostic_fn=None):
        super().__init__(print_dt=print_dt)

    def print_diagnostics(self, context):
        if hasattr(context, "diagnostics_message"):
            print("====================================")
            print(context.diagnostics_message)
