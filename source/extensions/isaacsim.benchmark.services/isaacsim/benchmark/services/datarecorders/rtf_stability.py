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

"""Recorder for windowed real-time factor (RTF) stability metrics."""

import time
from typing import Any

import carb
import carb.eventdispatcher
import omni.kit.app
import omni.timeline

from .. import utils
from ..metrics import measurements
from .interface import InputContext, MeasurementData, MeasurementDataRecorder, MeasurementDataRecorderRegistry
from .stats_utils import Stats

logger = utils.set_up_logging(__name__)

_SETTINGS_WINDOW_MS = "/exts/isaacsim.benchmark.services/rtf_stability/window_wall_ms"
_SETTINGS_EXPORT_SAMPLES = "/exts/isaacsim.benchmark.services/rtf_stability/export_window_samples"

# Reference bands for automated stability readout
_STABILITY_BAND_TIGHT = 0.01
_STABILITY_BAND_LOOSE = 0.10


def _band_core(samples: list[float], reference: float) -> tuple[float, float, float, int]:
    """Return max abs deviation, pct in tight/loose absolute bands around reference, max streak outside tight."""
    n = len(samples)
    max_abs = max(abs(s - reference) for s in samples)
    pct_tight = 100.0 * sum(1 for s in samples if abs(s - reference) <= _STABILITY_BAND_TIGHT) / float(n)
    pct_loose = 100.0 * sum(1 for s in samples if abs(s - reference) <= _STABILITY_BAND_LOOSE) / float(n)
    max_run_outside_tight = 0
    run = 0
    for s in samples:
        if abs(s - reference) > _STABILITY_BAND_TIGHT:
            run += 1
            max_run_outside_tight = max(max_run_outside_tight, run)
        else:
            run = 0
    return max_abs, pct_tight, pct_loose, max_run_outside_tight


def _stability_derived_metrics(samples: list[float], stats: Stats) -> list:
    """Absolute ±0.01/±0.10 bands vs phase mean windowed RTF; worst deviation and longest streak outside ±0.01."""
    mm_max, mm_pt, mm_pl, mm_streak = _band_core(samples, float(stats.mean))

    return [
        measurements.SingleMeasurement(
            name="Max Abs Windowed RTF Deviation from Mean", value=round(mm_max, 6), unit=""
        ),
        measurements.SingleMeasurement(
            name="Pct Windowed RTF Within 0.01 of Mean RTF", value=round(mm_pt, 2), unit="%"
        ),
        measurements.SingleMeasurement(
            name="Pct Windowed RTF Within 0.10 of Mean RTF", value=round(mm_pl, 2), unit="%"
        ),
        measurements.SingleMeasurement(
            name="Max Consecutive Windows Outside 0.01 Band of Mean RTF",
            value=mm_streak,
            unit="windows",
        ),
    ]


@MeasurementDataRecorderRegistry.register("rtf_stability")
class RtfStabilityRecorder(MeasurementDataRecorder):
    """Collect windowed RTF samples (sim time / wall time per wall-clock window).

    Uses the same simulation delta source as ``AppFrametimeRecorder`` (``dt`` on
    ``GLOBAL_EVENT_PRE_UPDATE``). If ``dt`` is missing, advances are taken from
    ``omni.timeline`` current time between updates.

    Emits mean/stdev/sample count plus derived stability readouts: absolute ±0.01/±0.10
    bands vs the phase mean windowed RTF, worst deviation from that mean, and the longest
    streak of consecutive windows outside the ±0.01 band.
    """

    def __init__(self, context: InputContext | None = None) -> None:
        self.context = context
        self._subscription = None
        self._phase: str | None = None
        self._rtf_samples: list[float] = []
        self._last_wall_ns: int = 0
        self._window_wall_ms: float = 0.0
        self._window_sim_ms: float = 0.0
        self._last_timeline_s: float | None = None
        self._logged_timeline_fallback: bool = False

    def start_collecting(self) -> None:
        """Start recording windowed real-time factor samples."""
        if self.context:
            self._phase = self.context.phase

        self._rtf_samples = []
        self._window_wall_ms = 0.0
        self._window_sim_ms = 0.0
        self._last_wall_ns = time.perf_counter_ns()
        self._last_timeline_s = None

        dispatcher = carb.eventdispatcher.get_eventdispatcher()
        self._subscription = dispatcher.observe_event(
            event_name=omni.kit.app.GLOBAL_EVENT_PRE_UPDATE,
            on_event=self._on_app_update,
            observer_name="RtfStabilityRecorder._on_app_update",
        )
        logger.info("RtfStabilityRecorder: Started collecting")

    def stop_collecting(self) -> None:
        """Stop recording windowed real-time factor samples."""
        self._subscription = None
        self._flush_window(force=True)
        logger.info("RtfStabilityRecorder: Stopped collecting. Collected %d window samples", len(self._rtf_samples))

    def _window_threshold_ms(self) -> float:
        settings = carb.settings.get_settings()
        w = settings.get(_SETTINGS_WINDOW_MS)
        if w is None or float(w) <= 0.0:
            return 100.0
        return float(w)

    def _on_app_update(self, event: Any) -> None:
        now_ns = time.perf_counter_ns()
        wall_slice_ms = (now_ns - self._last_wall_ns) / 1_000_000
        self._last_wall_ns = now_ns

        sim_slice_ms = 0.0
        if event.payload and "dt" in event.payload:
            sim_slice_ms = float(event.payload["dt"]) * 1000.0
        else:
            if not self._logged_timeline_fallback:
                logger.warning("RtfStabilityRecorder: PRE_UPDATE payload missing dt; using timeline time")
                self._logged_timeline_fallback = True
            try:
                tiface = omni.timeline.get_timeline_interface()
                t_s = float(tiface.get_current_time())
            except Exception:
                t_s = 0.0
            if self._last_timeline_s is not None:
                sim_slice_ms = (t_s - self._last_timeline_s) * 1000.0
            self._last_timeline_s = t_s

        self._window_wall_ms += wall_slice_ms
        self._window_sim_ms += sim_slice_ms

        if self._window_wall_ms >= self._window_threshold_ms():
            self._flush_window(force=False)

    def _flush_window(self, force: bool) -> None:
        min_wall = 1.0e-3
        threshold = self._window_threshold_ms()
        if self._window_wall_ms < min_wall:
            self._window_wall_ms = 0.0
            self._window_sim_ms = 0.0
            return
        if not force and self._window_wall_ms < threshold:
            return
        rtf = self._window_sim_ms / self._window_wall_ms
        self._rtf_samples.append(round(rtf, 6))
        self._window_wall_ms = 0.0
        self._window_sim_ms = 0.0

    def get_data(self) -> MeasurementData:
        """Return real-time factor stability measurements."""
        if self.context and self._phase != self.context.phase:
            return MeasurementData()

        if not self._rtf_samples:
            logger.warning("RtfStabilityRecorder: No window samples collected")
            return MeasurementData()

        stats = Stats.from_samples(self._rtf_samples, trim_outliers=False)
        out: list = [
            measurements.SingleMeasurement(name="Mean Windowed RTF", value=stats.mean, unit=""),
            measurements.SingleMeasurement(name="Stdev Windowed RTF", value=stats.stdev, unit=""),
            measurements.SingleMeasurement(name="Windowed RTF Sample Count", value=len(self._rtf_samples), unit=""),
        ]
        out.extend(_stability_derived_metrics(self._rtf_samples, stats))

        settings = carb.settings.get_settings()
        if settings.get(_SETTINGS_EXPORT_SAMPLES):
            out.append(measurements.ListMeasurement(name="Windowed RTF Samples", value=list(self._rtf_samples)))

        return MeasurementData(measurements=out)
