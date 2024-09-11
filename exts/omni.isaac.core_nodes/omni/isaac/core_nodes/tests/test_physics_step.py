import carb
import omni.graph.core as og
import omni.kit.test
from omni.isaac.core_nodes.bindings import _omni_isaac_core_nodes
from omni.isaac.nucleus import get_assets_root_path_async
from pxr import Sdf


class TestPhysicsStep(omni.kit.test.AsyncTestCase):
    async def setUp(self):
        """Set up  test environment, to be torn down when done"""
        self._timeline = omni.timeline.get_timeline_interface()
        self._core_nodes = _omni_isaac_core_nodes.acquire_interface()
        await omni.usd.get_context().new_stage_async()

    # ----------------------------------------------------------------------
    async def tearDown(self):
        """Get rid of temporary data used by the test"""
        await omni.kit.stage_templates.new_stage_async()

    async def test_physics_step_node(self):
        carb.settings.get_settings().set_bool("/app/player/useFixedTimeStepping", True)
        carb.settings.get_settings().set_bool("/app/runLoops/main/rateLimitEnabled", False)
        carb.settings.get_settings().set_int("/app/runLoops/main/rateLimitFrequency", 60)
        # carb.settings.get_settings().set_int("/persistent/simulation/minFrameRate", 0)
        omni.timeline.get_timeline_interface().set_target_framerate(60)
        omni.timeline.get_timeline_interface().set_time_codes_per_second(20)

        stage = omni.usd.get_context().get_stage()

        stage.DefinePrim("/Cube", "Cube")
        keys = og.Controller.Keys
        (self._clock_graph, _, _, _) = og.Controller.edit(
            {
                "graph_path": "/physics_step",
                "pipeline_stage": og.GraphPipelineStage.GRAPH_PIPELINE_STAGE_ONDEMAND,
            },
            {
                keys.CREATE_NODES: [
                    ("physics_step", "omni.isaac.core_nodes.OnPhysicsStep"),
                    ("read_prim_attr", "omni.graph.nodes.ReadPrimAttribute"),
                    ("constant_float", "omni.graph.nodes.ConstantFloat"),
                    ("add", "omni.graph.nodes.Add"),
                    ("write_prim_attr", "omni.graph.nodes.WritePrimAttribute"),
                ],
                keys.CONNECT: [
                    ("physics_step.outputs:step", "write_prim_attr.inputs:execIn"),
                    ("read_prim_attr.outputs:value", "add.inputs:a"),
                    ("constant_float.inputs:value", "add.inputs:b"),
                    ("add.outputs:sum", "write_prim_attr.inputs:value"),
                ],
                keys.SET_VALUES: [
                    ("read_prim_attr.inputs:prim", Sdf.Path("/Cube")),
                    ("read_prim_attr.inputs:name", "size"),
                    ("write_prim_attr.inputs:prim", Sdf.Path("/Cube")),
                    ("write_prim_attr.inputs:name", "size"),
                    ("constant_float.inputs:value", 1.0),
                ],
            },
        )
        steps = self._core_nodes.get_physics_num_steps()
        self.assertEqual(steps, 0)
        self._timeline.play()
        for i in range(10):
            await omni.kit.app.get_app().next_update_async()
            # Check according to the core_nodes.get_physics_num_steps that the cube size grew by the same size as the number of steps.
            steps = self._core_nodes.get_physics_num_steps()
            cube_size = stage.GetAttributeAtPath("/Cube.size").Get()
            self.assertEqual(cube_size, 2.0 + steps)

        self._timeline.stop()
