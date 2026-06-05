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

"""Recorder for render thread frametime metrics."""

import time
from typing import Any

import carb.eventdispatcher

from .. import utils
from ..metrics import measurements
from .interface import InputContext, MeasurementData, MeasurementDataRecorder, MeasurementDataRecorderRegistry
from .stats_utils import Stats

logger = utils.set_up_logging(__name__)


@MeasurementDataRecorderRegistry.register("render_frametime")
class RenderFrametimeRecorder(MeasurementDataRecorder):
    """Record render thread frametime for async rendering scenarios.

    Args:
        context: Input context for the recorder.
    """

    def __init__(self, context: InputContext | None = None) -> None:
        self.context = context
        self._samples: list[float] = []
        self._last_timestamp_ns: int = 0
        self._subscription = None
        self._phase: str | None = None

    def start_collecting(self) -> None:
        """Start collecting render thread frametime data.

        Example:

        .. code-block:: python

            recorder.start_collecting()
        """
        if self.context:
            self._phase = self.context.phase

        self._samples = []
        self._last_timestamp_ns = time.perf_counter_ns()

        dispatcher = carb.eventdispatcher.get_eventdispatcher()
        self._subscription = dispatcher.observe_event(
            event_name="runloop:rendering_0:update",
            on_event=self._on_render_update,
            observer_name="RenderFrametimeRecorder._on_render_update",
        )
        logger.info("RenderFrametimeRecorder: Started collecting")

    def stop_collecting(self) -> None:
        """Stop collecting render thread frametime data.

        Example:

        .. code-block:: python

            recorder.stop_collecting()
        """
        self._subscription = None

        if self._samples:
            self._samples.pop(0)

        logger.info("RenderFrametimeRecorder: Stopped collecting. Collected %d samples", len(self._samples))

    @property
    def sample_count(self) -> int:
        """Number of collected frametime samples.

        Returns:
            Number of frametime samples collected.

        Example:

        .. code-block:: python

            count = recorder.sample_count
        """
        return len(self._samples)

    @property
    def samples(self) -> list[float]:
        """Raw frametime samples in milliseconds.

        Returns:
            List of frametime samples (read-only access).

        Example:

        .. code-block:: python

            frametimes = recorder.samples
            mean_frametime = sum(frametimes) / len(frametimes)
        """
        return self._samples

    def _on_render_update(self, _event: Any) -> None:
        """Callback for render update events.

        Args:
            _event: Render update event payload.
        """
        timestamp_ns = time.perf_counter_ns()
        frametime_ms = (timestamp_ns - self._last_timestamp_ns) / 1_000_000
        self._last_timestamp_ns = timestamp_ns
        self._samples.append(round(frametime_ms, 6))

    def get_data(self) -> MeasurementData:
        """Get render thread frametime measurements.

        Returns:
            Collected render frametime statistics.

        Example:

        .. code-block:: python

            data = recorder.get_data()
        """
        if self.context and self._phase != self.context.phase:
            return MeasurementData()

        if not self._samples:
            logger.info("RenderFrametimeRecorder: No samples collected (async rendering may not be enabled)")
            return MeasurementData()

        stats = Stats.from_samples(self._samples, trim_outliers=False)

        measurements_out = [
            measurements.SingleMeasurement(name="Mean Render Frametime", value=stats.mean, unit="ms"),
            measurements.SingleMeasurement(name="Stdev Render Frametime", value=stats.stdev, unit="ms"),
            measurements.SingleMeasurement(name="Min Render Frametime", value=stats.min, unit="ms"),
            measurements.SingleMeasurement(name="Max Render Frametime", value=stats.max, unit="ms"),
        ]

        if stats.mean > 0:
            measurements_out.append(
                measurements.SingleMeasurement(name="Render Thread FPS", value=round(1000 / stats.mean, 3), unit="FPS")
            )

        return MeasurementData(measurements=measurements_out)
