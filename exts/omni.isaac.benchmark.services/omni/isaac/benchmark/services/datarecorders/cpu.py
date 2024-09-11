# Copyright (c) 2024, NVIDIA CORPORATION. All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto. Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.
#
import platform
from pathlib import Path
from typing import TYPE_CHECKING, Optional, Tuple

import psutil

from ..metrics.measurements import SingleMeasurement
from .interface import InputContext, MeasurementData, MeasurementDataRecorder

if TYPE_CHECKING:
    from ..settings import BenchmarkSettings


def get_cpu_counters() -> Tuple[float, float, float, float]:
    """
    Get counters from the CPU so that we can see our compute stats.

    On windows we report iowait as 0.0

    Returns:
        Tuple of floats with the percentage of time used on iowait, system, user and idle
    """
    current_cpu_counters = psutil.cpu_times()
    if platform.system() == "Windows":
        current_iowait_counter = 0.0
    else:
        current_iowait_counter = current_cpu_counters.iowait
    current_system_counter = current_cpu_counters.system
    current_user_counter = current_cpu_counters.user
    current_idle_counter = current_cpu_counters.idle
    return current_iowait_counter, current_system_counter, current_user_counter, current_idle_counter


def get_cpu_usage_in_pct(
    iowait_counter: float, system_counter: float, user_counter: float, idle_counter: float
) -> Tuple[float, float, float, float]:
    """
    Get how long we spent on each mode in the cpu in this benchmark run

    Args:
        iowait_counter (float): The iowait counter that we want to compare against
        system_counter (float): The iowait counter that we want to compare against
        user_counter (float): The iowait counter that we want to compare against
        idle_counter (float): The iowait counter that we want to compare against

    Returns:
        Tuple of floats with the percentage of time used on iowait, system, user and idle
    """
    previous_counter_total = iowait_counter + system_counter + user_counter + idle_counter

    current_cpu_counters = psutil.cpu_times()
    current_user_counter = current_cpu_counters.user
    if platform.system() == "Windows":
        current_iowait_counter = 0.0
    else:
        current_iowait_counter = current_cpu_counters.iowait

    current_system_counter = current_cpu_counters.system
    current_idle_counter = current_cpu_counters.idle
    counter_total = current_user_counter + current_iowait_counter + current_system_counter + current_idle_counter

    time_spent_total = counter_total - previous_counter_total

    time_spent_user = current_user_counter - user_counter
    time_spent_iowait = current_iowait_counter - iowait_counter
    time_spent_system = current_system_counter - system_counter
    time_spent_idle = current_idle_counter - idle_counter

    # Avoid dividing by zero, will only happen if we call this too quickly after getting the counters e.g: tests
    if time_spent_total == 0:
        time_spent_total = 0.0001

    return (
        round(time_spent_iowait / time_spent_total, 2) * 100,
        round(time_spent_system / time_spent_total, 2) * 100,
        round(time_spent_user / time_spent_total, 2) * 100,
        round(time_spent_idle / time_spent_total, 2) * 100,
    )


class CPUStatsRecorder(MeasurementDataRecorder):
    """
    This just outputs some simple CPU load stats... not sure how useful it is?

    """

    def __init__(
        self,
        context: Optional[InputContext] = None,
        root_dir: Optional[Path] = None,
        benchmark_settings: Optional["BenchmarkSettings"] = None,
    ):

        # Get cpu counters before starting the benchmark.
        (
            self.cpu_iowait,
            self.cpu_system,
            self.cpu_user,
            self.cpu_idle,
        ) = get_cpu_counters()

    def get_data(self) -> MeasurementData:

        (
            cpu_iowait_pct,
            cpu_system_pct,
            cpu_user_pct,
            cpu_idle_pct,
        ) = get_cpu_usage_in_pct(self.cpu_iowait, self.cpu_system, self.cpu_user, self.cpu_idle)

        m1 = SingleMeasurement(name="System CPU iowait", value=cpu_iowait_pct, unit="%")
        m2 = SingleMeasurement(name="System CPU system", value=cpu_system_pct, unit="%")
        m3 = SingleMeasurement(name="System CPU user", value=cpu_user_pct, unit="%")
        m4 = SingleMeasurement(name="System CPU idle", value=cpu_idle_pct, unit="%")

        return MeasurementData(measurements=[m1, m2, m3, m4])
