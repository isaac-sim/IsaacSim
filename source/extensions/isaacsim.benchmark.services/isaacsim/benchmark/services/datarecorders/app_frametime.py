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

"""Recorder for application update frametime and FPS metrics."""

import time
from typing import Any

import carb.eventdispatcher
import omni.kit.app

from .. import utils
from ..metrics import measurements
from .interface import InputContext, MeasurementData, MeasurementDataRecorder, MeasurementDataRecorderRegistry
from .stats_utils import Stats

logger = utils.set_up_logging(__name__)


@MeasurementDataRecorderRegistry.register("app_frametime")
class AppFrametimeRecorder(MeasurementDataRecorder):
    """Record application update loop frametime and FPS.

    Args:
        context: Input context for the recorder.
    """

    def __init__(self, context: InputContext | None = None) -> None:
        self.context = context
        self._samples: list[float] = []
        self._last_timestamp_ns: int = 0
        self._sim_time_ms: float = 0.0
        self._real_time_start_ns: int = 0
        self._elapsed_real_time_ms: float = 0.0
        self._subscription = None
        self._phase: str | None = None

    def start_collecting(self) -> None:
        """Start collecting app frametime data.

        Example:

        .. code-block:: python

            recorder.start_collecting()
        """
        if self.context:
            self._phase = self.context.phase

        self._samples = []
        self._sim_time_ms = 0.0
        self._last_timestamp_ns = time.perf_counter_ns()
        self._real_time_start_ns = time.perf_counter_ns()

        dispatcher = carb.eventdispatcher.get_eventdispatcher()
        self._subscription = dispatcher.observe_event(
            event_name=omni.kit.app.GLOBAL_EVENT_PRE_UPDATE,
            on_event=self._on_app_update,
            observer_name="AppFrametimeRecorder._on_app_update",
        )
        logger.info("AppFrametimeRecorder: Started collecting")

    def stop_collecting(self) -> None:
        """Stop collecting app frametime data.

        Example:

        .. code-block:: python

            recorder.stop_collecting()
        """
        self._subscription = None

        if self._real_time_start_ns > 0:
            self._elapsed_real_time_ms = (time.perf_counter_ns() - self._real_time_start_ns) / 1_000_000

        if self._samples:
            self._samples.pop(0)

        logger.info("AppFrametimeRecorder: Stopped collecting. Collected %d samples", len(self._samples))

    @property
    def sample_count(self) -> int:
        """Get the number of collected samples.

        Returns:
            Number of frametime samples collected.

        Example:

        .. code-block:: python

            count = recorder.sample_count
        """
        return len(self._samples)

    @property
    def samples(self) -> list[float]:
        """Get the raw frametime samples in milliseconds.

        Returns:
            List of frametime samples (read-only access).

        Example:

        .. code-block:: python

            frametimes = recorder.samples
            mean_frametime = sum(frametimes) / len(frametimes)
        """
        return self._samples

    def _on_app_update(self, event: Any) -> None:
        """Callback for app update events.

        Args:
            event: App update event payload.
        """
        timestamp_ns = time.perf_counter_ns()
        frametime_ms = (timestamp_ns - self._last_timestamp_ns) / 1_000_000
        self._last_timestamp_ns = timestamp_ns
        self._samples.append(round(frametime_ms, 6))

        if event.payload and "dt" in event.payload:
            self._sim_time_ms += event.payload["dt"] * 1000

    def get_data(self) -> MeasurementData:
        """Get app frametime measurements.

        Returns:
            Collected app frametime statistics.

        Example:

        .. code-block:: python

            data = recorder.get_data()
        """
        if self.context and self._phase != self.context.phase:
            return MeasurementData()

        if not self._samples:
            logger.warning("AppFrametimeRecorder: No samples collected")
            return MeasurementData()

        stats = Stats.from_samples(self._samples, trim_outliers=False)
        measurements_out = [
            measurements.SingleMeasurement(name="Mean App_Update Frametime", value=stats.mean, unit="ms"),
            measurements.SingleMeasurement(name="Stdev App_Update Frametime", value=stats.stdev, unit="ms"),
            measurements.SingleMeasurement(name="Min App_Update Frametime", value=stats.min, unit="ms"),
            measurements.SingleMeasurement(name="Max App_Update Frametime", value=stats.max, unit="ms"),
            measurements.SingleMeasurement(
                name="Mean FPS", value=round(1000 / stats.mean, 3) if stats.mean > 0 else 0, unit="FPS"
            ),
            measurements.ListMeasurement(name="App_Update Frametime Samples", value=self._samples),
            measurements.SingleMeasurement(name="Num App Updates", value=len(self._samples), unit=""),
        ]

        if self._elapsed_real_time_ms > 0 and self._sim_time_ms > 0:
            real_time_factor = self._sim_time_ms / self._elapsed_real_time_ms
            measurements_out.append(
                measurements.SingleMeasurement(name="Real Time Factor", value=round(real_time_factor, 3), unit="")
            )

        return MeasurementData(measurements=measurements_out)
