# Copyright (c) 2024, NVIDIA CORPORATION. All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto. Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.
#
from dataclasses import dataclass, field
from pathlib import Path
from typing import TYPE_CHECKING, List, Optional, Tuple, Type, Union

from ..metrics.measurements import Measurement, MetadataBase
from ..utils import SampledScope, SyncMode

if TYPE_CHECKING:
    from ..settings import BenchmarkSettings


@dataclass
class MeasurementData:
    """
    This is the return type for the MeasurementDataRecorder get_data() method
    """

    measurements: List[Measurement] = field(default_factory=lambda: [])
    metadata: List[MetadataBase] = field(default_factory=lambda: [])
    artefacts: List[Tuple[Path, str]] = field(default_factory=lambda: [])  # (path, artefact-label)


@dataclass
class InputContext:
    """
    this is the miscellaneous input data required by recorders to operate
    """

    artifact_prefix: str = ""
    kit_version: str = ""
    phase: str = ""
    sync_mode: SyncMode = SyncMode.ASYNC
    scope: Optional[SampledScope] = None


class MeasurementDataRecorder:
    """
    Interface/Base class for recording metrics, metadata and filed based artifacts, there are 2 basic use cases:
    1. things we measure at a specific point in time, and don't require accumulation over a period
    2. measurements like profilers or other sampling-based systems that gather data over a time period

    It's mostly up to the call site to work out when/where each recorder starts. For classes that gather data, the gathering
    normally starts from when the class is initialized
    """

    def __init__(
        self,
        context: Optional[InputContext] = None,
        root_dir: Optional[Path] = None,
        benchmark_settings: Optional["BenchmarkSettings"] = None,
    ):
        """
        Note that although the arguments are optional here, they may not be in a derived class
        """
        pass

    def get_data(self) -> MeasurementData:
        return MeasurementData()


class MeasurementDataRecorderRegistry:

    name_to_class = {}

    @classmethod
    def add(cls, name: str, recorder: Type[MeasurementDataRecorder]) -> None:
        cls.name_to_class[name] = recorder

    @classmethod
    def get(cls, name: str) -> Union[Type[MeasurementDataRecorder], None]:
        return cls.name_to_class.get(name)

    @classmethod
    def get_many(cls, names: List[str]) -> List[Type[MeasurementDataRecorder]]:
        classes = [cls.get(x) for x in names]
        return [c for c in classes if c is not None]
