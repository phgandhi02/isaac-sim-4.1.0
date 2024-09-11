# Copyright (c) 2020-2023, NVIDIA CORPORATION. All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto. Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.
#

import carb
import omni
from omni.isaac.core_nodes.ogn.OgnIsaacGetViewportRenderProductDatabase import OgnIsaacGetViewportRenderProductDatabase
from omni.kit.viewport.utility import get_viewport_from_window_name


class OgnIsaacGetViewportRenderProductInternalState:
    def __init__(self):
        viewport = None


class OgnIsaacGetViewportRenderProduct:
    """
    Isaac Sim Create Hydra Texture
    """

    @staticmethod
    def internal_state():
        return OgnIsaacGetViewportRenderProductInternalState()

    @staticmethod
    def compute(db) -> bool:
        state = db.per_instance_state
        viewport_api = get_viewport_from_window_name(db.inputs.viewport)
        if viewport_api:
            db.per_instance_state.viewport = viewport_api
        if db.per_instance_state.viewport == None:
            carb.log_warn("viewport name {db.inputs.viewport} not found")
            db.per_instance_state.initialized = False
            return False

        viewport = db.per_instance_state.viewport
        db.outputs.renderProductPath = viewport.get_render_product_path()
        db.outputs.execOut = omni.graph.core.ExecutionAttributeState.ENABLED
        return True

    @staticmethod
    def release_instance(node, graph_instance_id):
        try:
            state = OgnIsaacGetViewportRenderProductDatabase.per_instance_internal_state(node)
        except Exception:
            state = None
            pass

        if state is not None:
            state.viewport = None
