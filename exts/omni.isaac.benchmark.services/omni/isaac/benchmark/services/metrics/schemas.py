# Copyright (c) 2024, NVIDIA CORPORATION. All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto. Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.
#
import json
from dataclasses import dataclass
from typing import Any, List

import omni.kit.app


@dataclass
class OSConfiguration:
    platform: str  # Windows
    os: str  # Windows10
    os_major: str  # 10
    architecture: str  # 64bit


@dataclass
class GPU:
    number: int  # 0
    gpu_id: str  # 00000000:01:00.0'
    product_architecture: str  # Ampere
    product_brand: str  # NVIDIA RTX
    product_name: str  # NVIDIA RTX A5000 Laptop GPU


@dataclass
class GPUConfiguration:
    cuda_version: float  # 11.7
    driver_version: str  # 516.94
    gpus: List[GPU]
    primary_gpu: GPU
    num_gpus: int  # 1


@dataclass
class CPUConfiguration:
    model: str  # Intel64 Family 6 Model 141 Stepping 1, GenuineIntel


@dataclass
class MemoryConfiguration:
    ram_gb: float  # 63.7


@dataclass
class HardwareConfiguration:
    gpu_configuration: GPUConfiguration
    cpu_configuration: CPUConfiguration
    memory_configuration: MemoryConfiguration


@dataclass
class Application:
    name: str
    name_full: str
    kit_file: str
    version_minor: str  # 2022.3/104.0
    version_major_minor_patch: str  # 2022.3.0/104.0.90576
    version_full: str  # 2022.3.0-alpha.62/104.0.90576.83bc9102
    build_id: str  # 2022.3.0-alpha.62+daily.4861.b30d772c.tc/104.0+master.90576.83bc9102.tc
    kit_version_minor: str  # 104.0
    kit_version_patch: str  # 104.0.90581
    kit_build_id: str  # 104.0+master.90581.7e6b9494.tc
    package_name: str  # create-launcher/OmniverseKit
    package_full: str  # 2022.3.0-alpha.62+daily.4861.b30d772c.tc.windows-x86_64.release/104.0+master.90576.83bc9102.tc.windows-x86_64.release
    build_date: int  # 1666035333116


@dataclass
class ExecutionEnvironment:
    primary_system: str
    primary_id: str
    primary_url: str
    secondary_system: str
    secondary_id: str
    secondary_url: str
    extension_identifier: str  # 1.2.9
    etm_identifier: str
    input_build_url: str
    input_build_id: str
    hostname: str  # SC-OMNI-01


@dataclass
class BenchmarkIdentifier:
    run_uuid: str  # afe6b8d9f07f469397b300e7f7e026e5


@dataclass
class Benchmark:
    name: str  # Empty_Scene
    asset_url: str  # omniverse://kit-test-content.ov.nvidia.com/Projects/ov-perf-scenes/Kit_Benchmark_2/Empty_Scene/Empty_Scene.usd
    version_identifier: str  # 464b44d488cd32db3161888e01fb3611aa441f0dfc42e397c0c113f483b369ce
    checkpoint: int  # 1
    dssim_status: bool
    dssim: float  # 0.01
    resolution: str  # 720p


@dataclass
class Metric:
    name: str  # b_stage_local
    value: Any  # true


@dataclass
class BenchData:
    ts_created: int  # 1666035333116
    test_name: str  # Kit Benchmark
    schema: str  # stage_metric_v3
    hardware_configuration: HardwareConfiguration
    os_configuration: OSConfiguration
    application: Application
    execution_environment: ExecutionEnvironment
    benchmark_identifier: BenchmarkIdentifier
    benchmark: Benchmark
    metric: Metric

    def get_fingerprint(self) -> str:
        """Get session hash."""
        import hashlib

        params = {
            "name": self.application.name,  # Create/Kit
            "version_minor": self.application.version_minor,  # 2022.2/103.5
            "kit_version_minor": self.application.kit_version_minor,  # 104.0
            "platform": self.os_configuration.platform,  # Windows/Linux
            "os": self.os_configuration.os,  # Windows10, Ubuntu18
            "architecture": self.os_configuration.architecture,  # X86_64
            "cpu": self.hardware_configuration.cpu_configuration.model,  # Intel64 Family 6 Model 158 Stepping 13, GenuineIntel
            "gpu": self.hardware_configuration.gpu_configuration.primary_gpu.product_name,  # NVIDIA RTX A5000 Laptop GPU
            "driver": str(self.hardware_configuration.gpu_configuration.driver_version),  # 516.94
            "cuda": self.hardware_configuration.gpu_configuration.cuda_version,  # 11.7
            "python": omni.kit.app.get_app().get_platform_info()["python_version"],  # cp37
        }

        h = hashlib.sha256(json.dumps(params, sort_keys=True).encode("utf-8")).hexdigest()[:8]
        return h
