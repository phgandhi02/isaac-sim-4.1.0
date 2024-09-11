# Copyright (c) 2024, NVIDIA CORPORATION. All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto. Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.
#
import json
import logging
import os
from dataclasses import dataclass, field
from pathlib import Path
from typing import Dict, List, Union

logger = logging.getLogger(__name__)


@dataclass
class Measurement(object):
    name: str


@dataclass
class SingleMeasurement(Measurement):
    """
    represents a float measurement - maybe it should be called that?
    """

    value: float
    unit: str
    type: str = "single"


@dataclass
class BooleanMeasurement(Measurement):
    bvalue: bool
    type: str = "boolean"


@dataclass
class DictMeasurement(Measurement):
    value: Dict
    type: str = "dict"


@dataclass
class ListMeasurement(Measurement):
    value: List
    type: str = "list"

    def __repr__(self):
        return f"{self.__class__.__name__}(name={self.name!r}, length={len(self.value)})"


@dataclass
class MetadataBase(object):
    name: str


@dataclass
class StringMetadata(MetadataBase):
    data: str
    type: str = "string"


@dataclass
class IntMetadata(MetadataBase):
    data: int
    type: str = "int"


@dataclass
class FloatMetadata(MetadataBase):
    data: float
    type: str = "float"


@dataclass
class DictMetadata(MetadataBase):
    data: Dict
    type: str = "dict"


@dataclass
class TestPhase(object):
    """
    represent a single test phase which may have many metrics associated with it
    """

    phase_name: str
    measurements: List[Measurement] = field(default_factory=list)
    metadata: List[MetadataBase] = field(default_factory=list)

    def get_metadata_field(self, name, default=KeyError):
        """Get a metadata field's value.

        Args:
            name: Field name. Note that fields are named internally like 'Empty_Scene Stage DSSIM Status', however
                `name` is case-insensitive, and drops the stage name. In this eg it would be 'stage dssim status'.
        """
        name = name.lower()
        for m in self.metadata:
            name2 = m.name.replace(self.phase_name, "").strip().lower()
            if name == name2:
                return m.data

        if default is KeyError:
            raise KeyError(name)
        else:
            return default

    @classmethod
    def metadata_from_dict(cls, m: Dict) -> List[MetadataBase]:
        metadata = []
        metadata_mapping = {str: StringMetadata, int: IntMetadata, float: FloatMetadata, dict: DictMetadata}
        for meas in m["metadata"]:
            if "data" in meas:
                metadata_type = metadata_mapping.get(type(meas["data"]))
                if metadata_type:
                    curr_meta = metadata_type(name=meas["name"], data=meas["data"])
                    metadata.append(curr_meta)
        return metadata

    @classmethod
    def from_json(cls, m: Dict) -> "TestPhase":
        """
        Deserialize measurements and metadata.
        """
        curr_run = TestPhase(m["phase_name"])

        for meas in m["measurements"]:
            if "value" in meas:
                if isinstance(meas["value"], float):
                    curr_meas = SingleMeasurement(name=meas["name"], value=meas["value"], unit=meas["unit"])
                    curr_run.measurements.append(curr_meas)
                elif isinstance(meas["value"], dict):
                    curr_meas = DictMeasurement(name=meas["name"], value=meas["value"])
                    curr_run.measurements.append(curr_meas)
                elif isinstance(meas["value"], list):
                    curr_meas = ListMeasurement(name=meas["name"], value=meas["value"])
                    curr_run.measurements.append(curr_meas)
            elif "bvalue" in meas:
                curr_meas = BooleanMeasurement(name=meas["name"], bvalue=meas["bvalue"])
                curr_run.measurements.append(curr_meas)

            curr_run.metadata = TestPhase.metadata_from_dict(m["metadata"])
        return curr_run

    @classmethod
    def aggregate_json_files(cls, json_folder_path: Union[str, Path]) -> List["TestPhase"]:
        """
        When we write multiple JSON files containing test phases to
        a single folder, this will aggregate them and return all of
        the test phases
        """
        # Gather the separate metrics files for each test
        test_runs = []
        metric_files = os.listdir(json_folder_path)
        for f in metric_files:
            metric_path = os.path.join(json_folder_path, f)
            if os.path.isfile(metric_path):
                if f.startswith("metrics") and f.endswith(".json"):
                    with open(metric_path, "r") as json_file:
                        try:
                            test_run_json_list = json.load(json_file)
                            for m in test_run_json_list:
                                run = cls.from_json(m)
                                test_runs.append(run)
                        except json.JSONDecodeError:
                            logger.exception(
                                f'aggregate_json_files, problems parsing field {f} with content "{json_file.read()}"'
                            )
        return test_runs


class TestPhaseEncoder(json.JSONEncoder):
    def default(self, o):
        return o.__dict__
