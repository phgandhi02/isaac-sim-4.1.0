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
from omni.isaac.sensor import _sensor
from pxr import Gf


class TestIMUSensorOgn(omni.kit.test.AsyncTestCase):
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

        result, sensor = omni.kit.commands.execute(
            "IsaacSensorCreateImuSensor",
            path="/imu_sensor",
            parent="/World/Cube",
        )
        pass

    async def setup_ogn(self):
        self.graph_path = "/TestGraph"
        self.prim_path = "/World/Cube/imu_sensor"

        if get_prim_at_path(self.graph_path):
            delete_prim(self.graph_path)

        keys = og.Controller.Keys
        try:
            og.Controller.edit(
                {"graph_path": self.graph_path, "evaluator_name": "execution"},
                {
                    keys.CREATE_NODES: [
                        ("OnPlaybackTick", "omni.graph.action.OnPlaybackTick"),
                        ("ReadIMUNode", "omni.isaac.sensor.IsaacReadIMU"),
                    ],
                    keys.CONNECT: [
                        ("OnPlaybackTick.outputs:tick", "ReadIMUNode.inputs:execIn"),
                    ],
                },
            )
        except Exception as e:
            print(e)

    # verifying linear acceleration values are approximately equal to  zero for x and y and 9.81 for z in valid case
    # verifying sensor time is non-zero for the valid case
    async def test_valid_imu_sensor_ogn(self):
        og.Controller.set(
            og.Controller.attribute(self.graph_path + "/ReadIMUNode.inputs:imuPrim"),
            [usdrt.Sdf.Path("/World/Cube/imu_sensor")],
        )

        self.my_world.play()
        await simulate_async(0.5)
        lin_acc = og.Controller.attribute(self.graph_path + "/ReadIMUNode.outputs:linAcc").get()
        self.assertAlmostEqual(lin_acc[2], 9.81, places=2)
        self.assertAlmostEqual(lin_acc[0], 0.0, places=2)
        self.assertAlmostEqual(lin_acc[1], 0.0, places=2)

        sensor_time = og.Controller.attribute(self.graph_path + "/ReadIMUNode.outputs:sensorTime").get()
        self.assertNotEqual(sensor_time, 0.0)

    # verifying that linear acceleration and sensor time equal zero in invalid case
    async def test_invalid_imu_sensor_ogn(self):
        self.my_world.play()
        await simulate_async(0.5)

        lin_acc = og.Controller.attribute(self.graph_path + "/ReadIMUNode.outputs:linAcc").get()
        self.assertEqual(lin_acc[2], 0.0)
        self.assertEqual(lin_acc[0], 0.0)
        self.assertEqual(lin_acc[1], 0.0)

        sensor_time = og.Controller.attribute(self.graph_path + "/ReadIMUNode.outputs:sensorTime").get()
        self.assertAlmostEqual(sensor_time, 0.0)
