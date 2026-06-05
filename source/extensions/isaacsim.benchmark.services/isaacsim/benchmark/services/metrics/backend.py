# SPDX-FileCopyrightText: Copyright (c) 2024-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
# http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Metrics backend implementations for benchmark results."""

import copy
import json
import os
import shutil
import tempfile
from datetime import datetime as dt
from pathlib import Path
from typing import Any

import carb
import omni.kit.app
import omni.structuredlog
import toml  # type: ignore[import-untyped]
from isaacsim.core.version import get_version

from .. import utils
from . import measurements

logger = utils.set_up_logging(__name__)


class MetricsBackendInterface:
    """Interface for metrics backend implementations."""

    def add_metrics(self, test_phase: measurements.TestPhase) -> None:
        """Accumulate metrics for a test phase.

        Args:
            test_phase: Test phase to add.

        Example:

        .. code-block:: python

            backend.add_metrics(test_phase)
        """

    def finalize(self, metrics_output_folder: str, randomize_filename_prefix: bool = False, **kwargs: Any) -> None:
        """Write metrics data to files and clear state.

        Args:
            metrics_output_folder: Folder for output files.
            randomize_filename_prefix: True to randomize output file prefix. Defaults to False.
            **kwargs: Additional backend-specific options.

        Example:

        .. code-block:: python

            backend.finalize("/tmp/metrics")
        """


class KitGenericTelemetry(MetricsBackendInterface):
    """Use the Kit Telemetry System to store metrics."""

    def __init__(self) -> None:
        self._temp_dir = tempfile.mkdtemp(prefix="kit_telemetry_")
        privacy_toml_path: str = str(Path(self._temp_dir) / "privacy.toml")

        logger.info(f"Creating privacy.toml in temporary directory: {privacy_toml_path}")
        data = {
            "privacy": {
                "performance": True,
                "personalization": True,
                "usage": True,
                "userId": "perflab1",
                "extraDiagnosticDataOptIn": "externalBuilds",  # Kit 103
                "nvDiagnosticDataOptIn": "externalBuilds",  # Kit 102
                "eula": {"version": "2.0"},
                "gdpr": {"version": "1.0"},
            }
        }
        with open(privacy_toml_path, "w") as toml_file:
            toml.dump(data, toml_file)

        self._privacy_toml_path = privacy_toml_path

        settings = carb.settings.get_settings()
        if settings:
            settings.set("/structuredLog/privacySettingsFile", privacy_toml_path)
            logger.info(f"Set /structuredLog/privacySettingsFile to {privacy_toml_path}")

        # Force reload from specified location
        struct_log_settings = omni.structuredlog.IStructuredLogSettings()
        if struct_log_settings:
            struct_log_settings.load_privacy_settings()

    def cleanup(self) -> None:
        """Clean up the temporary directory containing privacy.toml.

        Example:

        .. code-block:: python

            backend.cleanup()
        """
        if hasattr(self, "_temp_dir") and os.path.exists(self._temp_dir):
            try:
                shutil.rmtree(self._temp_dir)
                logger.info(f"Removed temporary telemetry directory: {self._temp_dir}")
            except Exception as e:
                logger.warning(f"Failed to remove temporary telemetry directory: {e}")

    def add_metrics(self, test_phase: measurements.TestPhase) -> None:
        """Send a telemetry event for the provided test phase.

        Args:
            test_phase: Test phase to record.

        Example:

        .. code-block:: python

            backend.add_metrics(test_phase)
        """
        event_type = ("omni.kit.tests.benchmark@run_benchmark-dev",)
        # TOOD: this needs to be rewritten if we ever want to use it
        omni.kit.app.send_telemetry_event(
            event_type=event_type, duration=0.0, data1="", data2=1, value1=0.0, value2=0.0
        )

    def finalize(self, metrics_output_folder: str, randomize_filename_prefix: bool = False, **kwargs: Any) -> None:
        """Finalize telemetry backend resources.

        Args:
            metrics_output_folder: Folder for output files.
            randomize_filename_prefix: True to randomize output file prefix. Defaults to False.
            **kwargs: Additional backend-specific options.

        Example:

        .. code-block:: python

            backend.finalize("/tmp/metrics")
        """
        self.cleanup()


class LocalLogMetrics(MetricsBackendInterface):
    """Log metrics to the console."""

    def add_metrics(self, test_phase: measurements.TestPhase) -> None:
        """Log the provided test phase.

        Args:
            test_phase: Test phase to log.

        Example:

        .. code-block:: python

            backend.add_metrics(test_phase)
        """
        logger.info(f"LocalLogMetricsEvent::add_metrics {test_phase}")


class JSONFileMetrics(MetricsBackendInterface):
    """Write metrics to a JSON file at the end of a session."""

    def __init__(self) -> None:
        self.data: list[measurements.TestPhase] = []
        self.test_name = ""

    def add_metrics(self, test_phase: measurements.TestPhase) -> None:
        """Accumulate a test phase for later serialization.

        Args:
            test_phase: Test phase to add.

        Example:

        .. code-block:: python

            backend.add_metrics(test_phase)
        """
        self.data.append(copy.deepcopy(test_phase))

    def finalize(self, metrics_output_folder: str, randomize_filename_prefix: bool = False, **kwargs: Any) -> None:
        """Write metrics data to a JSON file.

        Args:
            metrics_output_folder: Folder for output files.
            randomize_filename_prefix: True to randomize output file prefix. Defaults to False.
            **kwargs: Additional backend-specific options.

        Example:

        .. code-block:: python

            backend.finalize("/tmp/metrics")
        """
        if not self.data:
            logger.warning("No test data to write. Skipping metrics file generation.")
            return

        # Append test name to measurement name as OVAT needs to uniquely identify
        for test_phase in self.data:
            test_name = test_phase.get_metadata_field("workflow_name")
            # Store the test name
            if test_name != self.test_name:
                if self.test_name:
                    logger.warning(
                        f"Nonempty test name {self.test_name} different from name {test_name} provided by test phase."
                    )
                self.test_name = test_name
                logger.info(f"Setting test name to {self.test_name}")

            phase_name = test_phase.get_metadata_field("phase")
            for measurement in test_phase.measurements:
                measurement.name = f"{test_name} {phase_name} {measurement.name}"

            for metadata in test_phase.metadata:
                metadata.name = f"{test_name} {phase_name} {metadata.name}"

        json_data = json.dumps(self.data, indent=4, cls=measurements.TestPhaseEncoder)

        # Generate the output filename
        if randomize_filename_prefix:
            _, metrics_filename_out = tempfile.mkstemp(
                dir=metrics_output_folder, prefix=f"metrics_{self.test_name}", suffix=".json"
            )
            metrics_path = Path(metrics_filename_out)
        else:
            metrics_path = Path(metrics_output_folder) / f"metrics_{self.test_name}.json"

        with open(metrics_path, "w") as f:
            logger.info(f"Writing metrics to {metrics_path}")
            f.write(json_data)

        self.data.clear()


class OsmoKPIFile(MetricsBackendInterface):
    """Write per-phase KPI documents for Osmo ingestion.

    Only SingleMeasurement metrics and metadata are written as key-value pairs.
    """

    def __init__(self) -> None:
        self._test_phases: list[measurements.TestPhase] = []

    def add_metrics(self, test_phase: measurements.TestPhase) -> None:
        """Adds provided test_phase to internal list of test_phases.

        Args:
            test_phase: Current test phase.

        Example:

        .. code-block:: python

            backend.add_metrics(test_phase)
        """
        self._test_phases.append(test_phase)

    def finalize(self, metrics_output_folder: str, randomize_filename_prefix: bool = False, **kwargs: Any) -> None:
        """Write metrics to output file(s).

        Each test phase's SingleMeasurement metrics and metadata are written to an output JSON file, at path
        `[metrics_output_folder]/[optional random prefix]kpis_{test_name}_{test_phase}.json`.

        Args:
            metrics_output_folder: Output folder in which metrics files will be stored.
            randomize_filename_prefix: True to randomize filename prefix. Defaults to False.
            **kwargs: Additional backend-specific options.

        Example:

        .. code-block:: python

            backend.finalize("/tmp/metrics")
        """
        for test_phase in self._test_phases:
            # Retrieve useful metadata from test_phase
            test_name = test_phase.get_metadata_field("workflow_name")
            phase_name = test_phase.get_metadata_field("phase")

            osmo_kpis: dict[str, object] = {}
            log_statements = [f"{phase_name} KPIs:"]
            # Add metadata as KPIs
            for metadata in test_phase.metadata:
                osmo_kpis[metadata.name] = metadata.data
                log_statements.append(f"{metadata.name}: {metadata.data}")
            # Add single measurements as KPIs
            for measurement in test_phase.measurements:
                if isinstance(measurement, measurements.SingleMeasurement):
                    osmo_kpis[measurement.name] = measurement.value
                    log_statements.append(f"{measurement.name}: {measurement.value} {measurement.unit}")
            # Log all KPIs to console
            logger.info("\n" + "\n".join(log_statements))
            # Generate the output filename
            if randomize_filename_prefix:
                _, metrics_filename_out = tempfile.mkstemp(
                    dir=metrics_output_folder, prefix=f"kpis_{test_name}_{phase_name}", suffix=".json"
                )
                metrics_path = Path(metrics_filename_out)
            else:
                metrics_path = Path(metrics_output_folder) / f"kpis_{test_name}_{phase_name}.json"
            # Dump key-value pairs (fields) to the JSON document
            json_data = json.dumps(osmo_kpis, indent=4)
            with open(metrics_path, "w") as f:
                logger.info(f"Writing KPIs to {metrics_path}")
                f.write(json_data)


class OmniPerfKPIFile(MetricsBackendInterface):
    """Write KPI metrics for upload to a PostgreSQL database."""

    def __init__(self) -> None:
        self._test_phases: list[measurements.TestPhase] = []

    def add_metrics(self, test_phase: measurements.TestPhase) -> None:
        """Adds provided test_phase to internal list of test_phases.

        Args:
            test_phase: Current test phase.

        Example:

        .. code-block:: python

            backend.add_metrics(test_phase)
        """
        self._test_phases.append(test_phase)

    def finalize(self, metrics_output_folder: str, randomize_filename_prefix: bool = False, **kwargs: Any) -> None:
        """Write metrics to output file(s).

        Measurement metrics and metadata are written to an output JSON file, at path
        `[metrics_output_folder]/[optional random prefix]kpis_{test_name}.json`.

        Args:
            metrics_output_folder: Output folder in which metrics file will be stored.
            randomize_filename_prefix: True to randomize filename prefix. Defaults to False.
            **kwargs: Additional backend-specific options.

        Example:

        .. code-block:: python

            backend.finalize("/tmp/metrics")
        """
        if not self._test_phases:
            logger.warning("No test phases to write. Skipping metrics file generation.")
            return

        workflow_data: dict[str, object] = {}
        app_version = get_version()
        workflow_data["App Info"] = [app_version[0], app_version[1], app_version[-1]]
        workflow_data["timestamp"] = dt.now().isoformat()

        workflow_data["Kit"] = utils.get_kit_version_branch()[2]  # get kit version

        test_name = None
        for test_phase in self._test_phases:
            # Retrieve useful metadata from test_phase
            test_name = test_phase.get_metadata_field("workflow_name")
            phase_name = test_phase.get_metadata_field("phase")

            phase_data: dict[str, object] = {}
            log_statements = [f"{phase_name} Metrics:"]
            # Add metadata as metrics
            for metadata in test_phase.metadata:
                phase_data[metadata.name] = metadata.data
                log_statements.append(f"{metadata.name}: {metadata.data}")
            # Add measurements as metrics
            for measurement in test_phase.measurements:
                if isinstance(measurement, measurements.SingleMeasurement):
                    log_statements.append(f"{measurement.name}: {measurement.value} {measurement.unit}")
                    phase_data[measurement.name] = measurement.value
            # Log all metrics to console
            logger.info("\n" + "\n".join(log_statements))

            workflow_data[phase_name] = phase_data

        # Generate the output filename
        if randomize_filename_prefix:
            _, metrics_filename_out = tempfile.mkstemp(
                dir=metrics_output_folder, prefix=f"kpis_{test_name}", suffix=".json"
            )
            metrics_path = Path(metrics_filename_out)
        else:
            metrics_path = Path(metrics_output_folder) / f"kpis_{test_name}.json"
        # Dump key-value pairs (fields) to the JSON document
        json_data = json.dumps(workflow_data, indent=4)
        with open(metrics_path, "w") as f:
            logger.info(f"Writing metrics to {metrics_path}")
            f.write(json_data)


class MetricsBackend:
    """Factory for metrics backend implementations."""

    @classmethod
    def get_instance(
        cls,
        instance_type: str | None = None,
    ) -> MetricsBackendInterface:
        """Return a backend instance for the requested type.

        Args:
            instance_type: Backend type name. Defaults to None.

        Returns:
            Metrics backend instance.

        Example:

        .. code-block:: python

            backend = MetricsBackend.get_instance("JSONFileMetrics")
        """
        if instance_type == "KitGenericTelemetry":
            return KitGenericTelemetry()
        elif instance_type == "LocalLogMetrics":
            return LocalLogMetrics()
        elif instance_type == "JSONFileMetrics":
            return JSONFileMetrics()
        elif instance_type == "OsmoKPIFile":
            return OsmoKPIFile()
        elif instance_type == "OmniPerfKPIFile":
            return OmniPerfKPIFile()
        else:
            return JSONFileMetrics()
