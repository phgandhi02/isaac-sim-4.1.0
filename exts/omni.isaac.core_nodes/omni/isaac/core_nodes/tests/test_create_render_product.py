# Copyright (c) 2018-2023, NVIDIA CORPORATION. All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto. Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.
#


import carb
import omni.graph.core as og
import omni.graph.core.tests as ogts
import omni.kit.test
from omni.isaac.core.robots import Robot
from omni.isaac.core.utils.stage import get_current_stage, open_stage_async
from omni.isaac.core.utils.viewports import get_viewport_names
from omni.isaac.nucleus import get_assets_root_path_async
from pxr import UsdRender


class TestCreateRenderProduct(ogts.OmniGraphTestCase):
    async def setUp(self):
        """Set up  test environment, to be torn down when done"""
        await omni.usd.get_context().new_stage_async()
        self._timeline = omni.timeline.get_timeline_interface()
        # add franka robot for test
        assets_root_path = await get_assets_root_path_async()
        if assets_root_path is None:
            carb.log_error("Could not find Isaac Sim assets folder")
            return
        (result, error) = await open_stage_async(assets_root_path + "/Isaac/Robots/Franka/franka.usd")

    # ----------------------------------------------------------------------
    async def tearDown(self):
        """Get rid of temporary data used by the test"""
        # await omni.kit.stage_templates.new_stage_async()

    # ----------------------------------------------------------------------
    async def test_create_render_product(self):
        (test_graph, new_nodes, _, _) = og.Controller.edit(
            {"graph_path": "/ActionGraph", "evaluator_name": "execution"},
            {
                og.Controller.Keys.CREATE_NODES: [
                    ("OnTick", "omni.graph.action.OnTick"),
                    ("createRP1", "omni.isaac.core_nodes.IsaacCreateRenderProduct"),
                    ("createRP2", "omni.isaac.core_nodes.IsaacCreateRenderProduct"),
                ],
                og.Controller.Keys.CONNECT: [
                    ("OnTick.outputs:tick", "createRP1.inputs:execIn"),
                    ("OnTick.outputs:tick", "createRP2.inputs:execIn"),
                ],
                og.Controller.Keys.SET_VALUES: [
                    ("createRP1.inputs:cameraPrim", "/OmniverseKit_Persp"),
                    ("createRP2.inputs:cameraPrim", "/OmniverseKit_Persp"),
                    ("createRP2.inputs:enabled", False),
                ],
            },
        )
        self._stage = get_current_stage()
        self._timeline.play()
        await omni.kit.app.get_app().next_update_async()
        await omni.kit.app.get_app().next_update_async()
        await omni.kit.app.get_app().next_update_async()

        rp_1 = UsdRender.Product(self._stage.GetPrimAtPath("/Render/RenderProduct_Replicator"))
        rp_2 = UsdRender.Product(self._stage.GetPrimAtPath("/Render/RenderProduct_Replicator_01"))
        self.assertTrue(rp_1)
        self.assertFalse(rp_2)

        og.Controller.attribute("inputs:width", new_nodes[1]).set(700)
        await omni.kit.app.get_app().next_update_async()
        await omni.kit.app.get_app().next_update_async()
        self.assertEqual(rp_1.GetResolutionAttr().Get(), (700, 720))

        og.Controller.attribute("inputs:cameraPrim", new_nodes[1]).set(["/OmniverseKit_Top"])
        await omni.kit.app.get_app().next_update_async()
        await omni.kit.app.get_app().next_update_async()
        self.assertEqual(rp_1.GetCameraRel().GetTargets()[0], "/OmniverseKit_Top")

        self._timeline.stop()
        await omni.kit.app.get_app().next_update_async()
