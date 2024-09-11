# Copyright (c) 2023-2024, NVIDIA CORPORATION. All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto. Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.
#
import time
from pathlib import Path
from typing import TYPE_CHECKING, Optional

import psutil

if TYPE_CHECKING:
    from omni.isaac.benchmark.services.settings import BenchmarkSettings

import carb
import omni.kit.app as omni_kit_app
from omni.isaac.benchmark.services.datarecorders import cpu, frametime, interface, memory
from omni.isaac.benchmark.services.metrics import measurements

from .collectors import IsaacUpdateFrametimeCollector


class IsaacFrameTimeRecorder(interface.MeasurementDataRecorder):
    def __init__(
        self,
        context: Optional[interface.InputContext] = None,
        root_dir: Optional[Path] = None,
        benchmark_settings: Optional["BenchmarkSettings"] = None,
    ):
        self.context = context
        self.root_dir = root_dir
        self.benchmark_settings = benchmark_settings
        self.frametime_collector = IsaacUpdateFrametimeCollector()
        self.phase = None

        self.real_time_start = None
        self.elapsed_real_time = None

    def start_collecting(self):
        self.phase = self.context.phase
        self.real_time_start = time.perf_counter_ns()
        self.frametime_collector.start_collecting()

    def stop_collecting(self):
        if self.real_time_start is None:
            # Frametime collection never began, so skip.
            return
        self.elapsed_real_time = (time.perf_counter_ns() - self.real_time_start) / 1000000
        self.frametime_collector.stop_collecting()

    def get_data(self):
        if self.phase != self.context.phase:
            return interface.MeasurementData(measurements=[])

        frametime_stats = frametime.FrametimeStats()
        frametime_stats.app_frametime_samples = self.frametime_collector.app_frametimes_ms
        frametime_stats.physics_frametime_samples = self.frametime_collector.physics_frametimes_ms
        frametime_stats.gpu_frametime_samples = self.frametime_collector.gpu_frametimes_ms
        frametime_stats.calc_stats()

        measurements_out = []

        measurements_out.append(
            measurements.SingleMeasurement(
                name=f"Mean Render Thread Frametime",
                value=frametime_stats.app_stats["mean"],
                unit="ms",
            )
        )
        measurements_out.append(
            measurements.SingleMeasurement(
                name=f"Stdev Render Thread Frametime",
                value=frametime_stats.app_stats["stdev"],
                unit="ms",
            )
        )
        measurements_out.append(
            measurements.SingleMeasurement(
                name=f"Min Render Thread Frametime",
                value=frametime_stats.app_stats["min"],
                unit="ms",
            )
        )
        measurements_out.append(
            measurements.SingleMeasurement(
                name=f"Max Render Thread Frametime",
                value=frametime_stats.app_stats["max"],
                unit="ms",
            )
        )
        measurements_out.append(
            measurements.ListMeasurement(
                name=f"Render Thread Frametime Samples",
                value=frametime_stats.app_frametime_samples,
            )
        )

        measurements_out.append(
            measurements.SingleMeasurement(
                name=f"Mean Physics Frametime",
                value=frametime_stats.physics_stats["mean"],
                unit="ms",
            )
        )
        measurements_out.append(
            measurements.SingleMeasurement(
                name=f"Stdev Physics Frametime",
                value=frametime_stats.physics_stats["stdev"],
                unit="ms",
            )
        )
        measurements_out.append(
            measurements.SingleMeasurement(
                name=f"Min Physics Frametime",
                value=frametime_stats.physics_stats["min"],
                unit="ms",
            )
        )
        measurements_out.append(
            measurements.SingleMeasurement(
                name=f"Max Physics Frametime",
                value=frametime_stats.physics_stats["max"],
                unit="ms",
            )
        )
        measurements_out.append(
            measurements.ListMeasurement(
                name=f"Physics Frametime Samples",
                value=frametime_stats.physics_frametime_samples,
            )
        )

        measurements_out.append(
            measurements.SingleMeasurement(
                name=f"Mean GPU Frametime", value=frametime_stats.gpu_stats["mean"], unit="ms"
            )
        )

        measurements_out.append(
            measurements.SingleMeasurement(
                name=f"Stdev GPU Frametime", value=frametime_stats.gpu_stats["stdev"], unit="ms"
            )
        )

        measurements_out.append(
            measurements.SingleMeasurement(name=f"Min GPU Frametime", value=frametime_stats.gpu_stats["min"], unit="ms")
        )

        measurements_out.append(
            measurements.SingleMeasurement(name=f"Max GPU Frametime", value=frametime_stats.gpu_stats["max"], unit="ms")
        )

        measurements_out.append(
            measurements.ListMeasurement(name=f"GPU Frametime Samples", value=frametime_stats.gpu_frametime_samples)
        )

        measurements_out.append(
            measurements.SingleMeasurement(
                name=f"Real Time Factor",
                value=self.frametime_collector.elapsed_sim_time / self.elapsed_real_time,
                unit="",
            )
        )
        return interface.MeasurementData(measurements=measurements_out)


class IsaacMemoryRecorder(memory.MemoryRecorder):
    def __init__(
        self,
        context: Optional[interface.InputContext] = None,
        root_dir: Optional[Path] = None,
        benchmark_settings: Optional["BenchmarkSettings"] = None,
    ):
        self.context = context
        self.root_dir = root_dir
        self.benchmark_settings = benchmark_settings
        super().__init__(context, root_dir, benchmark_settings)

    def get_data(self) -> interface.MeasurementData:
        (
            cpu_load,
            rss,
            vms,
            uss,
            pb,
            tracked_gpu_memory,
            dedicated_gpu_memory,
        ) = self.get_hardware_stats()

        m1 = measurements.SingleMeasurement(name=f"System Memory RSS", value=rss, unit="GB")
        m2 = measurements.SingleMeasurement(name=f"System Memory VMS", value=vms, unit="GB")
        m3 = measurements.SingleMeasurement(name=f"System Memory USS", value=uss, unit="GB")
        m4 = measurements.SingleMeasurement(name=f"GPU Memory Tracked", value=tracked_gpu_memory, unit="GB")
        m5 = measurements.SingleMeasurement(name=f"GPU Memory Dedicated", value=dedicated_gpu_memory, unit="GB")
        measurements_out = [m1, m2, m3, m4, m5]

        # Only capture System Memory PB for Windows.
        # if platform.system() == "Windows":
        #     measurements_out.append(
        #         measurements.SingleMeasurement(name=f"System Memory PB", value=pb, unit="GB")
        #     )

        return interface.MeasurementData(measurements=measurements_out)


class IsaacCPUStatsRecorder(cpu.CPUStatsRecorder):
    def __init__(
        self,
        context: Optional[interface.InputContext] = None,
        root_dir: Optional[Path] = None,
        benchmark_settings: Optional["BenchmarkSettings"] = None,
    ):
        self.context = context
        self.root_dir = root_dir
        self.benchmark_settings = benchmark_settings
        super().__init__(context, root_dir, benchmark_settings)

    def get_data(self) -> interface.MeasurementData:
        (
            cpu_iowait_pct,
            cpu_system_pct,
            cpu_user_pct,
            cpu_idle_pct,
        ) = cpu.get_cpu_usage_in_pct(self.cpu_iowait, self.cpu_system, self.cpu_user, self.cpu_idle)

        m1 = measurements.SingleMeasurement(name=f"System CPU iowait", value=cpu_iowait_pct, unit="%")
        m2 = measurements.SingleMeasurement(name=f"System CPU system", value=cpu_system_pct, unit="%")
        m3 = measurements.SingleMeasurement(name=f"System CPU user", value=cpu_user_pct, unit="%")
        m4 = measurements.SingleMeasurement(name=f"System CPU idle", value=cpu_idle_pct, unit="%")

        return interface.MeasurementData(measurements=[m1, m2, m3, m4])


class IsaacRuntimeRecorder(interface.MeasurementDataRecorder):
    def __init__(
        self,
        context: Optional[interface.InputContext] = None,
        root_dir: Optional[Path] = None,
        benchmark_settings: Optional["BenchmarkSettings"] = None,
    ):
        self.context = context
        self.root_dir = root_dir
        self.benchmark_settings = benchmark_settings
        self.start = 0.0
        self.elapsed_time = None
        self.phase = None

    def start_time(self):
        self.phase = self.context.phase
        self.start = omni_kit_app.get_app().get_time_since_start_ms()

    def stop_time(self):
        if self.phase is None:
            self.phase = self.context.phase
        self.elapsed_time = omni_kit_app.get_app().get_time_since_start_ms() - self.start

    def get_data(self):
        if self.phase != self.context.phase:
            return interface.MeasurementData(measurements=[])

        m1 = measurements.SingleMeasurement(name=f"Runtime", value=self.elapsed_time, unit="ms")
        return interface.MeasurementData(measurements=[m1])


class IsaacHardwareSpecRecorder(interface.MeasurementDataRecorder):
    def __init__(self, context: Optional[interface.InputContext] = None):
        self.context = context

    def get_data(self):
        import torch

        device_names = [torch.cuda.get_device_name(d) for d in range(torch.cuda.device_count())]

        if len(set(device_names)) > 1:
            carb.log_warn(f"Detected multiple GPU types: {device_names}.")
            carb.log_warn(f"Only recording GPU 0 type: {device_names[0]}")

        measurements_out = []

        measurements_out.append(measurements.SingleMeasurement(name=f"num_cpus", value=psutil.cpu_count(), unit=""))
        measurements_out.append(measurements.SingleMeasurement(name=f"gpu_device_name", value=device_names[0], unit=""))

        return interface.MeasurementData(measurements_out)
