import asyncio

import carb
import numpy as np
import omni.graph.core as og
import omni.isaac.core.utils.stage as stage_utils
import omni.kit.commands
import omni.kit.test
import usdrt.Sdf
from omni.isaac.core import World
from omni.isaac.core.objects import DynamicCuboid, GroundPlane
from omni.isaac.core.utils.physics import simulate_async
from omni.isaac.core.utils.prims import delete_prim, get_prim_at_path
from omni.isaac.core.utils.stage import create_new_stage_async
from omni.physx.scripts import physicsUtils
from pxr import Gf


class TestContactSensorOgn(omni.kit.test.AsyncTestCase):
    async def setUp(self):
        await create_new_stage_async()
        await self.setup_environment()
        await self.setup_ogn()
        physics_rate = 60
        self.my_world = World(stage_units_in_meters=1.0, physics_dt=1.0 / physics_rate, rendering_dt=1.0 / physics_rate)
        await self.my_world.initialize_simulation_context_async()

    async def tearDown(self):
        if self.my_world:
            self.my_world.stop()
            self.my_world.clear_instance()
        await omni.kit.app.get_app().next_update_async()
        while omni.usd.get_context().get_stage_loading_status()[2] > 0:
            # print("tearDown, assets still loading, waiting to finish...")
            await asyncio.sleep(1.0)
        await omni.kit.app.get_app().next_update_async()
        pass

    async def setup_environment(self):
        plane = GroundPlane(prim_path="/World/GroundPlane", z_position=0)

        prim = DynamicCuboid(prim_path="/World/Cube", mass=1.0, position=np.array([0, 0, 1.0]))
        contactReportAPI = physicsUtils.PhysxSchema.PhysxContactReportAPI.Apply(get_prim_at_path("/World/Cube"))
        contactReportAPI.CreateThresholdAttr().Set(0)
        result, sensor = omni.kit.commands.execute(
            "IsaacSensorCreateContactSensor",
            path="/contact_sensor",
            parent="/World/Cube",
            max_threshold=10000000,
        )
        pass

    async def setup_ogn(self):
        self.graph_path = "/TestGraph"
        self.prim_path = "/World/Cube/contact_sensor"

        if get_prim_at_path(self.graph_path):
            delete_prim(self.graph_path)

        keys = og.Controller.Keys
        try:
            og.Controller.edit(
                {"graph_path": self.graph_path, "evaluator_name": "execution"},
                {
                    keys.CREATE_NODES: [
                        ("OnPlaybackTick", "omni.graph.action.OnPlaybackTick"),
                        ("ReadContactNode", "omni.isaac.sensor.IsaacReadContactSensor"),
                    ],
                    keys.CONNECT: [
                        ("OnPlaybackTick.outputs:tick", "ReadContactNode.inputs:execIn"),
                    ],
                    keys.SET_VALUES: [
                        (
                            "ReadContactNode.inputs:csPrim",
                            [usdrt.Sdf.Path("/World/Cube/contact_sensor")],
                        ),
                    ],
                },
            )
        except Exception as e:
            print(e)

    # verifying force value and sensor time are non-zero in valid case
    async def test_valid__contact_sensor_ogn(self):
        # must play, stop, and play simulation for force value to be properly recorded
        self.my_world.play()
        await omni.kit.app.get_app().next_update_async()

        await simulate_async(1.0)

        force_value = og.Controller.attribute(self.graph_path + "/ReadContactNode.outputs:value").get()
        self.assertNotEqual(force_value, 0.0)

        sensor_time = og.Controller.attribute(self.graph_path + "/ReadContactNode.outputs:sensorTime").get()
        self.assertNotEqual(sensor_time, 0.0)

    # verifying force value and sensor time equal zero for invalid case
    async def test_invalid_contact_sensor_ogn(self):
        og.Controller.set(
            og.Controller.attribute(self.graph_path + "/ReadContactNode.inputs:csPrim"), [usdrt.Sdf.Path("/World/Cube")]
        )
        self.my_world.play()
        await simulate_async(0.5)
        force_val = og.Controller.attribute(self.graph_path + "/ReadContactNode.outputs:value").get()
        self.assertEqual(force_val, 0.0)

        sensor_time = og.Controller.attribute(self.graph_path + "/ReadContactNode.outputs:sensorTime").get()
        self.assertEqual(sensor_time, 0.0)
