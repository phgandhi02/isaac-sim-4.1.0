# Copyright (c) 2024, NVIDIA CORPORATION. All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto. Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.
#
import math
import statistics
from dataclasses import dataclass, field
from pathlib import Path
from typing import TYPE_CHECKING, List, Optional

from .. import utils

if TYPE_CHECKING:
    from ..settings import BenchmarkSettings

logger = utils.set_up_logging(__name__)


@dataclass
class FrametimeStats:
    app_frametime_samples: List[float] = field(default_factory=list)
    gpu_frametime_samples: List[float] = field(default_factory=list)

    app_stats = {}
    physics_stats = {}
    gpu_stats = {}

    def _percentile_inc(self, values: List, percent: float, key=lambda x: x) -> float:
        """
        Find the percentile of a list of values.

        Args:
            values: is a list of values. Note N MUST BE already sorted.
            percent: a float value from 0.0 to 1.0.
            key: optional key function to compute value from each element of N.

        Returns:
            The percentile of the values
        """
        k = (len(values) - 1) * percent
        f = math.floor(k)
        c = math.ceil(k)
        if f == c:
            return key(values[int(k)])
        d0 = key(values[int(f)]) * (c - k)
        d1 = key(values[int(c)]) * (k - f)
        return d0 + d1

    def get_one_percent_high(self, values: List) -> float:
        """
        Given a list of floats, return the average of the largest 1% values.

        Args:
            values: A list of floats

        Returns:
            An average of the 1% largest values
        """
        ninety_nine_p = self._percentile_inc(sorted(values), 0.99)
        return statistics.mean([x for x in values if x >= ninety_nine_p])

    def stats_helper(self, metric: List[float]):
        result = {"mean": 0, "median": 0, "stdev": 0, "min": 0, "max": 0, "one_percent": 0}
        try:
            result["mean"] = round(statistics.mean(metric), 2)
            result["median"] = round(statistics.median(metric), 2)
            result["stdev"] = round(statistics.stdev(metric), 2)
            result["min"] = round(min(metric), 2)
            result["max"] = round(max(metric), 2)
            result["one_percent"] = round(self.get_one_percent_high(metric), 2)
        except Exception as e:
            logger.warn(f"Unable to calculate frametime stats: {e}")
        return result

    def calc_stats(self) -> None:
        self.app_stats = self.stats_helper(self.app_frametime_samples)
        self.physics_stats = self.stats_helper(self.physics_frametime_samples)
        self.gpu_stats = self.stats_helper(self.gpu_frametime_samples)
