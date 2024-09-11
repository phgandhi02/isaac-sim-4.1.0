# Copyright (c) 2024, NVIDIA CORPORATION. All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto. Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.
#
import omni
from omni.isaac.benchmark.services import BaseIsaacBenchmarkAsync
from omni.isaac.benchmark.services.metrics.measurements import (
    BooleanMeasurement,
    DictMeasurement,
    ListMeasurement,
    SingleMeasurement,
)
from omni.isaac.core import SimulationContext


class TestBaseIsaacBenchmarkAsync(BaseIsaacBenchmarkAsync):
    async def setUp(self):
        await super().setUp(backend_type="LocalLogMetrics")
        pass

    async def tearDown(self):
        await super().tearDown()
        pass

    async def test_base_isaac_benchmark(self):
        self.benchmark_name = "test_base_isaac_benchmark"
        self.set_phase("loading", False, True)
        await omni.kit.app.get_app().next_update_async()
        await omni.kit.app.get_app().next_update_async()
        simulation_context = SimulationContext()
        await simulation_context.initialize_simulation_context_async()
        await self.store_measurements()
        simulation_context.play()

        self.set_phase("benchmark")
        for frame in range(10):
            await omni.kit.app.get_app().next_update_async()
        await self.store_measurements()

    async def test_store_custom_measurement(self):
        self.benchmark_name = "test_custom_measurements"
        self.set_phase("loading", False, True)
        await omni.kit.app.get_app().next_update_async()
        await omni.kit.app.get_app().next_update_async()
        simulation_context = SimulationContext()
        await simulation_context.initialize_simulation_context_async()
        await self.store_measurements()
        simulation_context.play()
        await omni.kit.app.get_app().next_update_async()

        # Store different types of measurements
        measurement = BooleanMeasurement(name="bool_measure", bvalue=True)
        await self.store_custom_measurement("phase_1", measurement)

        measurement = SingleMeasurement(name="single_measure_1", value=1.23, unit="ms")
        await self.store_custom_measurement("phase_1", measurement)

        measurement = DictMeasurement(name="dict_measure", value={"key": "value"})
        await self.store_custom_measurement("phase_2", measurement)

        measurement = SingleMeasurement(name="single_measure_2", value=4.56, unit="ms")
        await self.store_custom_measurement("phase_3", measurement)

        measurement = ListMeasurement(name="list_measure", value=[1, 2, 3])
        await self.store_custom_measurement("phase_4", measurement)
        await omni.kit.app.get_app().next_update_async()
