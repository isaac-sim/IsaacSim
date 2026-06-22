# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

"""Recorder for wall-clock interval between consecutive PhysX simulation steps."""

import time
from typing import Any

import carb
import omni.physics.core

from .. import utils
from ..metrics import measurements
from .interface import InputContext, MeasurementData, MeasurementDataRecorder, MeasurementDataRecorderRegistry
from .stats_utils import Stats

logger = utils.set_up_logging(__name__)

_SETTINGS_EXPORT_SAMPLES = "/exts/isaacsim.benchmark.services/physics_step_interval/export_samples"


@MeasurementDataRecorderRegistry.register("physics_step_interval")
class PhysicsStepIntervalRecorder(MeasurementDataRecorder):
    """Record wall-clock time between consecutive PhysX simulation steps.

    The ``PhysicsFrametimeRecorder`` measures how long each PhysX
    solve takes (compute duration), this recorder measures the wall-clock
    inter-arrival time -- how much real time passes between the completion
    of one physics step and the next.  The **Max Physics Step Interval**
    metric is the worst-case latency suitable for real-time guarantees
    (e.g. "PhysX must step at least once every 5 ms of wall clock").

    Args:
        context: Input context for the recorder.
    """

    def __init__(self, context: InputContext | None = None) -> None:
        self.context = context
        self._samples: list[float] = []
        self._last_step_ns: int = 0
        self._subscription = None
        self._physics_sim_iface = None
        self._phase: str | None = None

        try:
            self._physics_sim_iface = omni.physics.core.get_physics_simulation_interface()
        except Exception as e:
            logger.warning("PhysicsStepIntervalRecorder: Failed to get physics simulation interface: %s", e)

    def start_collecting(self) -> None:
        """Start recording intervals between physics step callbacks."""
        if self.context:
            self._phase = self.context.phase

        self._samples = []
        self._last_step_ns = time.perf_counter_ns()

        if self._physics_sim_iface:
            self._subscription = self._physics_sim_iface.subscribe_physics_on_step_events(
                on_update=self._on_physics_step,
                pre_step=False,
                order=0,
            )
            logger.info("PhysicsStepIntervalRecorder: Started collecting")
        else:
            logger.warning("PhysicsStepIntervalRecorder: Physics simulation interface not available")

    def stop_collecting(self) -> None:
        """Stop recording physics step intervals."""
        self._subscription = None

        # Drop the first sample -- the interval from start_collecting() to the
        # first physics step is not a meaningful inter-step measurement.
        if self._samples:
            self._samples.pop(0)

        logger.info("PhysicsStepIntervalRecorder: Stopped collecting. Collected %d samples", len(self._samples))

    @property
    def sample_count(self) -> int:
        """Return the number of collected interval samples."""
        return len(self._samples)

    @property
    def samples(self) -> list[float]:
        """Return collected interval samples in milliseconds."""
        return self._samples

    def _on_physics_step(self, _step_dt: float, _context: Any) -> None:
        now_ns = time.perf_counter_ns()
        interval_ms = (now_ns - self._last_step_ns) / 1_000_000
        self._last_step_ns = now_ns
        self._samples.append(round(interval_ms, 6))

    def get_data(self) -> MeasurementData:
        """Return physics step interval measurements.

        Returns:
            Measurement data containing interval statistics, or empty data if
            no samples were collected or the phase changed.
        """
        if self.context and self._phase != self.context.phase:
            return MeasurementData()

        if not self._samples:
            logger.info("PhysicsStepIntervalRecorder: No samples collected (physics may not be running)")
            return MeasurementData()

        # Use untrimmed stats
        stats = Stats.from_samples(self._samples, trim_outliers=False)

        out: list = [
            measurements.SingleMeasurement(name="Mean Physics_Step Frametime", value=stats.mean, unit="ms"),
            measurements.SingleMeasurement(name="Stdev Physics_Step Frametime", value=stats.stdev, unit="ms"),
            measurements.SingleMeasurement(name="Min Physics_Step Frametime", value=stats.min, unit="ms"),
            measurements.SingleMeasurement(name="Max Physics_Step Frametime", value=stats.max, unit="ms"),
            measurements.SingleMeasurement(name="P99 Physics_Step Frametime", value=stats.p99, unit="ms"),
            measurements.SingleMeasurement(name="Physics Step Count", value=len(self._samples), unit=""),
        ]

        settings = carb.settings.get_settings()
        if settings.get(_SETTINGS_EXPORT_SAMPLES):
            out.append(measurements.ListMeasurement(name="Physics_Step Frametime Samples", value=list(self._samples)))

        return MeasurementData(measurements=out)
