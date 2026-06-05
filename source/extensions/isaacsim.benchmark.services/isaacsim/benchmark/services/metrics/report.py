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

"""Formatted reporting for benchmark metric phases."""

import textwrap

from . import measurements

# List of other "metadata" metrics to be filtered out of each phase
additional_metadata = ["num_cpus", "gpu_device_name"]
# List of metrics to remove from the summary report
default_exclusions = [
    "System CPU iowait",
    "System CPU system",
    "System CPU user",
    "System CPU idle",
    "GPU Memory Dedicated",
]


class Report:
    """Format benchmark metrics into a human-readable report."""

    def __init__(self) -> None:
        self._test_phases: list[measurements.TestPhase] = []
        self._phase_data: list[dict[str, list[str]]] = []
        self._addt_metadata: list[str] = []
        self._report_width = 50  # char width
        self._frametime_metrics: dict[str, dict[str, str]] = {}
        self._cpu_metrics: dict[str, dict[str, str]] = {}
        self._use_section_headers = False  # Set to True for section headers within phases

    def add_metric_phase(self, test_phase: measurements.TestPhase) -> None:
        """Add a test phase and pre-format its output lines.

        Args:
            test_phase: Current test phase.

        Example:

        .. code-block:: python

            report.add_metric_phase(test_phase)
        """
        self._test_phases.append(test_phase)
        formatted_lines = self.process_phase(test_phase)
        self._phase_data.append({test_phase.get_metadata_field("phase"): formatted_lines})

    def process_phase(self, test_phase: measurements.TestPhase) -> list[str]:
        """Process metric data for a phase into formatted strings.

        Args:
            test_phase: Current test phase.

        Returns:
            List of formatted strings containing metric data for the phase.

        Example:

        .. code-block:: python

            lines = report.process_phase(test_phase)
        """
        logs = []
        phase_name = f"Phase: {test_phase.get_metadata_field('phase')}"
        logs.extend(self._boxed_inner_lines(phase_name))

        # Categorize measurements for output
        performance_metrics = []
        memory_metrics = []
        custom_metrics = []

        for measurement in test_phase.measurements:
            if isinstance(measurement, measurements.SingleMeasurement):
                if measurement.name in additional_metadata:
                    self._add_metadata(measurement)
                elif measurement.name not in default_exclusions:
                    # Check for standard frametime metrics (Mean/Stdev/Min/Max [Type] Frametime)
                    if measurement.name.endswith(" Frametime") and any(
                        measurement.name.startswith(prefix) for prefix in ["Mean", "Stdev", "Min", "Max"]
                    ):
                        self._process_frametime_metric(measurement)
                    elif "Runtime" in measurement.name:
                        performance_metrics.append(measurement)
                    elif (
                        "Memory" in measurement.name
                        or "USS" in measurement.name
                        or "RSS" in measurement.name
                        or "VMS" in measurement.name
                    ):
                        memory_metrics.append(measurement)
                    elif "CPU Usage" in measurement.name or "Single Core Usage" in measurement.name:
                        # Process CPU metrics into table format
                        self._process_cpu_metric(measurement)
                    else:
                        # Custom metrics (e.g., Replicator FPS)
                        custom_metrics.append(measurement)

        # 1. Performance metrics (Runtime always first)
        if performance_metrics:
            runtime_metrics = [m for m in performance_metrics if "Runtime" in m.name]
            other_perf = [m for m in performance_metrics if "Runtime" not in m.name]
            for m in runtime_metrics:
                logs.extend(self._format_measurement(m))
            for m in sorted(other_perf, key=lambda x: x.name):
                logs.extend(self._format_measurement(m))

        # 2. Custom metrics (grouped by prefix)
        if custom_metrics:
            # Sort custom metrics by prefix to group related metrics together
            def get_metric_prefix(name: str) -> str:
                return name.split()[0] if " " in name else name

            for m in sorted(custom_metrics, key=lambda x: (get_metric_prefix(x.name), x.name)):
                logs.extend(self._format_measurement(m))

        # 3. Memory metrics
        if memory_metrics:
            for m in sorted(memory_metrics, key=lambda x: x.name):
                logs.extend(self._format_measurement(m))

        # 4. CPU metrics (grouped table)
        if self._cpu_metrics:
            logs.extend(self.get_cpu_metrics())
            self._cpu_metrics.clear()

        # 5. Frametime metrics (grouped table at end)
        if self._frametime_metrics:
            # Add spacer line for visual separation from CPU table
            if self._cpu_metrics or memory_metrics:
                logs.append(f"| {'':<{self._report_width}} |")
            logs.extend(self.get_frametime_metrics())
            self._frametime_metrics.clear()

        return logs

    def _wrap_inner(self, text: str) -> list[str]:
        """Split text into lines that fit the boxed inner width (no truncation)."""
        if len(text) <= self._report_width:
            return [text]
        return textwrap.wrap(
            text,
            width=self._report_width,
            break_long_words=True,
            break_on_hyphens=False,
        )

    def _boxed_inner_lines(self, inner: str) -> list[str]:
        """Full-width bordered lines for inner content, wrapping when needed."""
        return [f"| {row:<{self._report_width}} |" for row in self._wrap_inner(inner)]

    def _format_measurement(self, measurement: measurements.SingleMeasurement) -> list[str]:
        """Create formatted bordered lines for a measurement (may wrap).

        Args:
            measurement: Measurement object.

        Returns:
            One or more bordered lines containing metric data.
        """
        line = f"{measurement.name}: {measurement.value} {measurement.unit}".rstrip()
        return self._boxed_inner_lines(line)

    def _add_metadata(self, measurement: measurements.SingleMeasurement) -> None:
        """Add measurement to the metadata list.

        Args:
            measurement: Measurement data.
        """
        metadata = f"{measurement.name}: {measurement.value} {measurement.unit}"
        if metadata not in self._addt_metadata:
            self._addt_metadata.append(metadata)

    def _process_frametime_metric(self, measurement: measurements.SingleMeasurement) -> None:
        """Add frametime metric data to the frametime table.

        Args:
            measurement: Measurement data.
        """
        metric_type = measurement.name.split(" ")[0]  # min/max/mean/stddev
        frametime = measurement.name.split(" ")[1]  # render/physics/gpu
        if frametime not in self._frametime_metrics:
            self._frametime_metrics[frametime] = {}
        self._frametime_metrics[frametime][metric_type] = f"{measurement.value:.2f}"

    def _process_cpu_metric(self, measurement: measurements.SingleMeasurement) -> None:
        """Add CPU usage data to the CPU metrics table.

        Args:
            measurement: Measurement data.
        """
        # Extract metric type (Mean/Max/Min/Stdev) from name like "Mean CPU Usage"
        metric_type = measurement.name.split(" ")[0]  # Mean/Max/Min/Stdev
        if "Process" not in self._cpu_metrics:
            self._cpu_metrics["Process"] = {}
        self._cpu_metrics["Process"][metric_type] = f"{measurement.value:.2f}"

    def print_formatted_lines(self, phase: dict[str, list[str]]) -> None:
        """Print formatted metric data for a phase.

        Args:
            phase: Current measurement phase.

        Example:

        .. code-block:: python

            report.print_formatted_lines(phase)
        """
        for key, value in phase.items():
            for line in value:
                print(line)

    def add_separator(self) -> str:
        """Return a dashed separator line.

        Returns:
            Separator line string.

        Example:

        .. code-block:: python

            line = report.add_separator()
        """
        separator = "|" + "-" * (self._report_width + 2) + "|"
        return separator

    def print_metadata(self) -> None:
        """Print formatted benchmark metadata.

        Example:

        .. code-block:: python

            report.print_metadata()
        """
        for metadata in self._test_phases[0].metadata[:-1]:
            formatted = f"{metadata.name}: {metadata.data}"
            for boxed in self._boxed_inner_lines(formatted):
                print(boxed)
        for data in self._addt_metadata:
            for boxed in self._boxed_inner_lines(data):
                print(boxed)

    def get_frametime_metrics(self) -> list[str]:
        """Format frametime metric data as a table.

        Returns:
            List of formatted table lines.

        Example:

        .. code-block:: python

            lines = report.get_frametime_metrics()
        """
        logs = []
        label = f"{'Frametimes (ms):':<12}{'mean':>8} | {'stdev':>6} | {'min':>5} | {'max':>5}"
        logs.append(f"| {label:<{self._report_width}} |")
        for thread, metrics in self._frametime_metrics.items():
            mean = metrics.get("Mean", "-")
            stddev = metrics.get("Stdev", "-")
            min_val = metrics.get("Min", "-")
            max_val = metrics.get("Max", "-")
            line = f"{thread:<16}{mean:>8} | {stddev:>6} | {min_val:>5} | {max_val:>5}"
            logs.append(f"| {line:<{self._report_width}} |")
        return logs

    def get_cpu_metrics(self) -> list[str]:
        """Format CPU usage metric data as a table.

        Returns:
            List of formatted table lines.

        Example:

        .. code-block:: python

            lines = report.get_cpu_metrics()
        """
        logs = []
        # Compact header to fit within 50 char width
        # Format: label(11) + mean(8) + " | "(3) + stdev(6) + " | "(3) + min(6) + " | "(3) + max(8) = 48 chars
        label = f"{'CPU (%):':<11}{'mean':>8} | {'stdev':>6} | {'min':>6} | {'max':>8}"
        logs.append(f"| {label:<{self._report_width}} |")

        # Process row
        if "Process" in self._cpu_metrics:
            metrics = self._cpu_metrics["Process"]
            mean = metrics.get("Mean", "-")
            stddev = metrics.get("Stdev", "-")
            min_val = metrics.get("Min", "-")
            max_val = metrics.get("Max", "-")
            line = f"{'Process':<11}{mean:>8} | {stddev:>6} | {min_val:>6} | {max_val:>8}"
            logs.append(f"| {line:<{self._report_width}} |")

        return logs

    def create_report(self) -> None:
        """Print the full summary report.

        Example:

        .. code-block:: python

            report.create_report()
        """
        print(self.add_separator())
        title = "Summary Report"
        print(f"| {title:^{self._report_width}} |")
        print(self.add_separator())
        self.print_metadata()
        for phase in self._phase_data:
            print(self.add_separator())
            self.print_formatted_lines(phase)
        print(self.add_separator())
