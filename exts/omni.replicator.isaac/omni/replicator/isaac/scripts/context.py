# Copyright (c) 2022-2024, NVIDIA CORPORATION. All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto. Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.
#
import omni.graph.core as og
from omni.replicator.core.utils import utils

_context = None


class ReplicatorIsaacContext:
    def __init__(self, num_envs, action_graph_entry_node):
        self._num_envs = num_envs
        self._action_graph_entry_node = action_graph_entry_node
        self._reset_inds = None
        self.trigger = False

        controller = og.Controller()
        self._graph = controller.graph(utils.GRAPH_PATH)
        self._tendon_attribute_stack = [None]

    def trigger_randomization(self, reset_inds):
        self.trigger = True
        self._reset_inds = reset_inds
        self._action_graph_entry_node.request_compute()
        self._graph.evaluate()

    @property
    def reset_inds(self):
        return self._reset_inds

    def get_tendon_exec_context(self):
        return self._tendon_attribute_stack[-1]

    def add_tendon_exec_context(self, node):
        self._tendon_attribute_stack.append(node)


def initialize_context(num_envs, action_graph_entry_node):
    global _context
    _context = ReplicatorIsaacContext(num_envs, action_graph_entry_node)


def get_reset_inds():
    return _context.reset_inds


def trigger_randomization(reset_inds):
    _context.trigger_randomization(reset_inds)
