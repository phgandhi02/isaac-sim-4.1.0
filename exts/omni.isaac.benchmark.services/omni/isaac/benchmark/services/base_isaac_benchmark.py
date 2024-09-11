# Copyright (c) 2018-2024, NVIDIA CORPORATION. All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto. Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.
#


import os
import tempfile
import time

import carb
from omni.isaac.benchmark.services import utils
from omni.isaac.benchmark.services.datarecorders import interface
from omni.isaac.benchmark.services.metrics import backend, measurements
from omni.isaac.core.utils.stage import open_stage
from omni.isaac.nucleus import get_assets_root_path

from .recorders import *
from .utils import wait_until_stage_is_fully_loaded

logger = utils.set_up_logging(__name__)


def set_sync_mode():
    """
    Sync mode sets settings that blocks the app until all materials are fully loaded
    """
    carb_settings = carb.settings.get_settings()
    if carb_settings.get("/app/asyncRendering"):
        carb.log_warn("Async rendering is enabled, setting sync mode might not work")
    carb_settings.set("/omni.kit.plugin/syncUsdLoads", True)
    carb_settings.set("/rtx-defaults/materialDb/syncLoads", True)
    carb_settings.set("/rtx-defaults/hydra/materialSyncLoads", True)


class BaseIsaacBenchmark:
    """
    Front-end for benchmarking standalone scripts and other non-async snippets.

    By default this class will collect hardware performance data (see recorders), broken up by "phase". Typical structure looks like:

    .. code-block:: python

        benchmark = BaseIsaacBenchmark(benchmark_name=..., workflow_metadata=...)
        benchmark.set_phase("loading")
        # load stage, configure sim, etc.
        benchmark.store_measurements()
        benchmark.set_phase("benchmark")
        # Actual code being benchmarked (running the sim for N frames, cloning an object, etc.)
        benchmark.store_measurements()
        benchmark.stop() # Shuts down benchmark, writes metrics to file

    You can set any number of phases.

    """

    def __init__(
        self,
        benchmark_name: str = "BaseIsaacBenchmark",
        backend_type: str = "OsmoKPIFile",
        workflow_metadata: dict = {},
    ):
        """

        Args:
            benchmark_name (str, optional): Name of benchmark - will be printed in outputs. Defaults to
                "BaseIsaacBenchmark".
            backend_type (str, optional): Type of backend used to collect and print metrics. Supported values provided
                in metrics.backend. Defaults to "OsmoKPIFile".
            workflow_metadata (dict, optional): Metadata describing benchmark (eg. number of GPUs, number of cameras,
                etc.) Most useful for OsmoKPIFile backend. Expected as JSON-style input - nested dictionary of
                {"metadata": [{"name": <name>, "data": <value>}, ...]}. Defaults to {}.
        """
        self.benchmark_name = benchmark_name
        self._test_phases = []

        self.settings = carb.settings.get_settings()
        prefix = self._get_output_file_prefix(benchmark_name)
        version, _, _ = utils.get_kit_version_branch()

        self.assets_root_path = get_assets_root_path()
        if self.assets_root_path is None:
            carb.log_error("Could not find Isaac Sim assets folder")
            return

        self.context = interface.InputContext(
            artifact_prefix=prefix,
            kit_version=version,
            phase="benchmark",
            sync_mode=self._get_sync_mode(),
        )

        self.frametime_recorder = IsaacFrameTimeRecorder(self.context)
        self.runtime_recorder = IsaacRuntimeRecorder(self.context)
        self.recorders = [
            IsaacMemoryRecorder(self.context),
            IsaacCPUStatsRecorder(self.context),
            IsaacHardwareSpecRecorder(self.context),
            self.frametime_recorder,
            self.runtime_recorder,
        ]

        self._metrics_output_folder = self.settings.get(
            "/exts/omni.isaac.benchmark.services/metrics/metrics_output_folder"
        )

        if not self._metrics_output_folder:
            self._metrics_output_folder = tempfile.gettempdir()

        # Get metrics backend based on user-provided type
        logger.info(f"Using metrics backend = {backend_type}")
        self._metrics = backend.MetricsBackend.get_instance(instance_type=backend_type)

        # Generate workflow-level metadata
        self._metadata = [measurements.StringMetadata(name="workflow_name", data=self.benchmark_name)]
        if "metadata" in workflow_metadata:
            self._metadata.extend(measurements.TestPhase.metadata_from_dict(workflow_metadata))
        elif workflow_metadata:
            logger.warning(
                "workflow_metadata provided, but missing expected `metadata' entry. Metadata will not be read."
            )

        logger.info(f"Local folder location = {self._metrics_output_folder}")
        logger.info("Starting")
        self.benchmark_start_time = time.time()
        self.test_mode = os.getenv("ISAAC_TEST_MODE") == "1"
        logger.info(f"Test mode = {'true' if self.test_mode else 'false'}")
        pass

    def stop(self):
        """
        Stop benchmarking and write accumulated metrics to file.
        """

        if not os.path.exists(self._metrics_output_folder):
            os.mkdir(path=self._metrics_output_folder)
        randomize_filename_prefix = self.settings.get(
            "/exts/omni.isaac.benchmark.services/metrics/randomize_filename_prefix"
        )
        logger.info("Stopping")
        logger.info("Writing metrics data.")
        logger.info(f"Metrics type = {type(self._metrics).__name__}")
        # Finalize by adding all test phases to the backend metrics
        for test_phase in self._test_phases:
            self._metrics.add_metrics(test_phase)
        self._metrics.finalize(self._metrics_output_folder, randomize_filename_prefix)

        self.test_run = None
        self.recorders = None
        self.context = None
        pass

    def _get_output_file_prefix(self, test_phase: str) -> str:
        """
        uniquefies artifact file names (so e.g if we support multiple resolutions they are included in the name
        """
        version, _, _ = utils.get_kit_version_branch()
        return f"{test_phase}_{version}"

    def _get_sync_mode(self) -> utils.SyncMode:
        """Checks if we are in sync mode."""
        async_rendering = self.settings.get("/app/asyncRendering")
        usd_sync_loads = self.settings.get("/omni.kit.plugin/syncUsdLoads")

        # On Viewport 2.0 the `rtx` settings are now under `rtx-defaults`.
        materialdb_sync_loads = self.settings.get("/rtx/materialDb/syncLoads")
        if materialdb_sync_loads is None:
            materialdb_sync_loads = self.settings.get("/rtx-defaults/materialDb/syncLoads")

        hydra_material_sync_loads = self.settings.get("/rtx/hydra/materialSyncLoads")
        if hydra_material_sync_loads is None:
            hydra_material_sync_loads = self.settings.get("/rtx-defaults/hydra/materialSyncLoads")

        if not async_rendering and materialdb_sync_loads and usd_sync_loads and hydra_material_sync_loads:
            return utils.SyncMode.SYNC
        elif async_rendering and not materialdb_sync_loads and not usd_sync_loads and not hydra_material_sync_loads:
            return utils.SyncMode.ASYNC

        logger.info(
            f"ambiguous combination of async/sync flags {async_rendering} {materialdb_sync_loads} {usd_sync_loads} {hydra_material_sync_loads}"
        )
        return utils.SyncMode.AMBIGUOUS

    def set_phase(
        self, phase: str, start_recording_frametime: bool = True, start_recording_runtime: bool = True
    ) -> None:
        """Sets benchmarking phase. Turns on frametime and runtime collection.

        Args:
            phase (str): Name of phase, used in output.
            start_recording_frametime (bool): False to not start recording frametime at start of phase. Default True.
            start_recording_runtime (bool): False to not start recording runtime at start of phase. Default True.
        """
        logger.info(f"Starting phase: {phase}")
        self.context.phase = phase
        if start_recording_frametime:
            self.frametime_recorder.start_collecting()
        if start_recording_runtime:
            self.runtime_recorder.start_time()

    def store_measurements(self, stop_recording_time: bool = True) -> None:
        """Stores measurements, metadata, and artifacts collected by all recorders during the previous phase.
        Optionally, ends frametime and runtime collection.

        Args:
            stop_recording_time (bool): False to not stop recording runtime and frametime at end of phase. Default True.
        """
        if stop_recording_time:
            self.frametime_recorder.stop_collecting()
            self.runtime_recorder.stop_time()

        # Retrieve metrics, metadata, and artifacts from the recorders
        run_measurements = []
        run_metadata = []
        run_artifacts = []
        for m in self.recorders:
            data = m.get_data()
            run_measurements.extend(data.measurements)
            run_metadata.extend(data.metadata)
            run_artifacts.extend(data.artefacts)
        # Create a new test phase to store these measurements
        test_phase = measurements.TestPhase(
            phase_name=self.context.phase, measurements=run_measurements, metadata=run_metadata
        )
        # Update test phase metadata with phase name and benchmark metadata
        test_phase.metadata.extend(self._metadata)
        test_phase.metadata.append(measurements.StringMetadata(name="phase", data=self.context.phase))
        self._test_phases.append(test_phase)

    def fully_load_stage(self, usd_path: str) -> None:
        """Loads provided USD stage, blocking execution until it is fully loaded.

        Args:
            usd_path (str): Path to USD stage.
        """
        open_stage(usd_path)
        wait_until_stage_is_fully_loaded()

    def store_custom_measurement(self, phase_name: str, custom_measurement: measurements) -> None:
        """Stores the custom measurement specificed in the benchmark.

        Args:
            phase (str): The phase name to which the measurement belongs.
            measurement (Measurement): The measurement object to store.
        """
        # Check if the phase already exists
        existing_phase = next((phase for phase in self._test_phases if phase.phase_name == phase_name), None)

        if existing_phase:
            # Add the custom measurement to the existing phase
            existing_phase.measurements.append(custom_measurement)
            logger.info(f"Stored {custom_measurement} for phase '{phase_name}'")
        else:
            # If the phase does not exist, create a new test phase
            new_test_phase = measurements.TestPhase(
                phase_name=phase_name, measurements=[custom_measurement], metadata=[]
            )
            # Update test phase metadata with phase name and benchmark metadata
            new_test_phase.metadata.extend(self._metadata)
            new_test_phase.metadata.append(measurements.StringMetadata(name="phase", data=phase_name))

            # Add the new test phase to the list of test phases
            self._test_phases.append(new_test_phase)

            logger.info(f"Created new phase '{phase_name}' and stored {custom_measurement}")
