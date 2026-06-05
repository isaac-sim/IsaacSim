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

"""Recorder for continuous process CPU usage sampling."""

import os
import statistics
import time
from typing import Any

import carb.eventdispatcher
import omni.kit.app
import psutil

from .. import utils
from ..metrics import measurements
from .interface import InputContext, MeasurementData, MeasurementDataRecorder, MeasurementDataRecorderRegistry

logger = utils.set_up_logging(__name__)


@MeasurementDataRecorderRegistry.register("cpu_continuous")
class CPUContinuousRecorder(MeasurementDataRecorder):
    """Continuously sample process CPU usage during benchmark execution.

    Tracks process-specific CPU usage, which can exceed 100% on multi-core
    systems. For example, 400% means the process is using 4 full cores.

    Args:
        context: Benchmark context.
        sample_interval_frames: Sample CPU every N frames.
    """

    def __init__(self, context: InputContext | None = None, sample_interval_frames: int = 10) -> None:
        self.context = context
        self.sample_interval_frames = sample_interval_frames
        self._samples: list[dict[str, float]] = []
        self._subscription = None
        self._frame_count: int = 0
        self._phase: str | None = None
        self._process = psutil.Process(os.getpid())

    def start_collecting(self) -> None:
        """Start continuous CPU sampling.

        Example:

        .. code-block:: python

            recorder.start_collecting()
        """
        if self.context:
            self._phase = self.context.phase

        self._samples = []
        self._frame_count = 0

        # Initialize process CPU tracking (non-blocking mode)
        self._process.cpu_percent(interval=None)

        dispatcher = carb.eventdispatcher.get_eventdispatcher()
        self._subscription = dispatcher.observe_event(
            event_name=omni.kit.app.GLOBAL_EVENT_PRE_UPDATE,
            on_event=self._on_app_update,
            observer_name="CPUContinuousRecorder._on_app_update",
        )
        logger.info(
            "CPUContinuousRecorder: Started collecting process CPU (sampling every %d frames)",
            self.sample_interval_frames,
        )

    def stop_collecting(self) -> None:
        """Stop continuous CPU sampling.

        Example:

        .. code-block:: python

            recorder.stop_collecting()
        """
        self._subscription = None
        logger.info("CPUContinuousRecorder: Stopped collecting. Collected %d samples", len(self._samples))

    def _on_app_update(self, _event: Any) -> None:
        """Sample CPU usage periodically.

        Args:
            _event: App update event payload.
        """
        self._frame_count += 1

        if self._frame_count % self.sample_interval_frames == 0:
            # Process CPU percentage (can be >100% on multi-core systems)
            process_cpu_percent = self._process.cpu_percent(interval=None)

            self._samples.append(
                {
                    "timestamp": time.perf_counter(),
                    "process_cpu_percent": process_cpu_percent,
                }
            )

    def get_data(self) -> MeasurementData:
        """Get CPU usage measurements.

        Returns:
            Collected CPU usage statistics.

        Example:

        .. code-block:: python

            data = recorder.get_data()
        """
        if self.context and self._phase != self.context.phase:
            return MeasurementData()

        if not self._samples:
            logger.warning("CPUContinuousRecorder: No samples collected")
            return MeasurementData()

        # Calculate process CPU statistics
        process_cpu_percents = [s["process_cpu_percent"] for s in self._samples]

        measurements_out = [
            measurements.SingleMeasurement(
                name="Mean CPU Usage", value=round(statistics.mean(process_cpu_percents), 2), unit="%"
            ),
            measurements.SingleMeasurement(name="Max CPU Usage", value=round(max(process_cpu_percents), 2), unit="%"),
            measurements.SingleMeasurement(name="Min CPU Usage", value=round(min(process_cpu_percents), 2), unit="%"),
            measurements.SingleMeasurement(
                name="Stdev CPU Usage",
                value=round(statistics.stdev(process_cpu_percents), 2) if len(process_cpu_percents) > 1 else 0.0,
                unit="%",
            ),
        ]

        return MeasurementData(measurements=measurements_out)
