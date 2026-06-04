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

"""Recorder for GPU frametime statistics from Hydra engine."""

from typing import Any

import carb.eventdispatcher
import omni.kit.app

from .. import utils
from ..metrics import measurements
from .interface import InputContext, MeasurementData, MeasurementDataRecorder, MeasurementDataRecorderRegistry
from .stats_utils import Stats

logger = utils.set_up_logging(__name__)


@MeasurementDataRecorderRegistry.register("gpu_frametime")
class GPUFrametimeRecorder(MeasurementDataRecorder):
    """Record GPU render frametime from Hydra engine stats.

    Args:
        context: Input context for the recorder.
        enable_multi_gpu: Enable per-GPU sampling when multiple GPUs are present.
    """

    def __init__(self, context: InputContext | None = None, enable_multi_gpu: bool = False) -> None:
        self.context = context
        self.enable_multi_gpu = enable_multi_gpu
        self._samples: list[float] = []
        self._per_gpu_samples: list[list[float]] = []
        self._hydra_stats = None
        self._phase: str | None = None
        self._subscription = None

        try:
            from omni.hydra.engine.stats import HydraEngineStats

            self._hydra_stats = HydraEngineStats()
            logger.info("GPUFrametimeRecorder: HydraEngineStats initialized")
        except Exception as e:
            logger.warning(f"GPUFrametimeRecorder: Failed to initialize HydraEngineStats: {e}")

    def start_collecting(self) -> None:
        """Start collecting GPU frametime data.

        Example:

        .. code-block:: python

            recorder.start_collecting()
        """
        if self.context:
            self._phase = self.context.phase

        self._samples = []
        self._per_gpu_samples = []

        # Subscribe to app update events to sample GPU time every frame
        dispatcher = carb.eventdispatcher.get_eventdispatcher()
        self._subscription = dispatcher.observe_event(
            event_name=omni.kit.app.GLOBAL_EVENT_PRE_UPDATE,
            on_event=self._on_app_update,
            observer_name="GPUFrametimeRecorder._on_app_update",
        )
        logger.info("GPUFrametimeRecorder: Started collecting")

    def stop_collecting(self) -> None:
        """Stop collecting GPU frametime data.

        Example:

        .. code-block:: python

            recorder.stop_collecting()
        """
        self._subscription = None

        if self._samples:
            self._samples.pop(0)

        for gpu_list in self._per_gpu_samples:
            if gpu_list:
                gpu_list.pop(0)

        logger.info("GPUFrametimeRecorder: Stopped collecting. Collected %d samples", len(self._samples))

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

    def _on_app_update(self, _event: Any) -> None:
        """Sample GPU frametime on each app update.

        Args:
            _event: App update event payload.
        """
        self.sample_gpu_time()

    def sample_gpu_time(self) -> None:
        """Sample GPU frametime for the current frame.

        Example:

        .. code-block:: python

            recorder.sample_gpu_time()
        """
        if not self._hydra_stats:
            return

        avg_time, per_gpu_times = self._get_gpu_times()
        self._samples.append(round(avg_time, 6))

        if self.enable_multi_gpu and per_gpu_times:
            if len(self._per_gpu_samples) != len(per_gpu_times):
                self._per_gpu_samples = [[] for _ in per_gpu_times]

            for i, time_val in enumerate(per_gpu_times):
                self._per_gpu_samples[i].append(round(time_val, 6))

    def _get_gpu_times(self) -> tuple[float, list[float]]:
        """Get GPU times from Hydra stats.

        Returns:
            Tuple of (average_time_ms, per_gpu_times_ms).
        """
        if not self._hydra_stats:
            return 0.0, []

        try:
            device_nodes = self._hydra_stats.get_gpu_profiler_result()

            total_time = 0.0
            per_gpu_times = []

            for device in device_nodes:
                device_time = 0.0
                for node in device:
                    if node.get("indent", 0) == 0:
                        device_time += node.get("duration", 0.0)
                per_gpu_times.append(device_time)
                total_time += device_time

            avg_time = total_time / len(device_nodes) if device_nodes else 0.0
            return avg_time, per_gpu_times
        except Exception as e:
            logger.warning(f"GPUFrametimeRecorder: Error getting GPU times: {e}")
            return 0.0, []

    def get_data(self) -> MeasurementData:
        """Get GPU frametime measurements.

        Returns:
            Collected GPU frametime statistics.

        Example:

        .. code-block:: python

            data = recorder.get_data()
        """
        if self.context and self._phase != self.context.phase:
            return MeasurementData()

        if not self._samples:
            logger.warning("GPUFrametimeRecorder: No samples collected")
            return MeasurementData()

        stats = Stats.from_samples(self._samples, trim_outliers=False)
        measurements_out = [
            measurements.SingleMeasurement(name="Mean GPU Frametime", value=stats.mean, unit="ms"),
            measurements.SingleMeasurement(name="Stdev GPU Frametime", value=stats.stdev, unit="ms"),
            measurements.SingleMeasurement(name="Min GPU Frametime", value=stats.min, unit="ms"),
            measurements.SingleMeasurement(name="Max GPU Frametime", value=stats.max, unit="ms"),
            measurements.ListMeasurement(name="GPU Frametime Samples", value=self._samples),
        ]

        if self.enable_multi_gpu and len(self._per_gpu_samples) > 1:
            for gpu_idx, gpu_samples in enumerate(self._per_gpu_samples):
                if gpu_samples:
                    gpu_stats = Stats.from_samples(gpu_samples, trim_outliers=False)
                    measurements_out.extend(
                        [
                            measurements.SingleMeasurement(
                                name=f"Mean GPU{gpu_idx} Frametime", value=gpu_stats.mean, unit="ms"
                            ),
                            measurements.SingleMeasurement(
                                name=f"Stdev GPU{gpu_idx} Frametime", value=gpu_stats.stdev, unit="ms"
                            ),
                            measurements.SingleMeasurement(
                                name=f"Min GPU{gpu_idx} Frametime", value=gpu_stats.min, unit="ms"
                            ),
                            measurements.SingleMeasurement(
                                name=f"Max GPU{gpu_idx} Frametime", value=gpu_stats.max, unit="ms"
                            ),
                            measurements.ListMeasurement(name=f"GPU{gpu_idx} Frametime Samples", value=gpu_samples),
                        ]
                    )

        return MeasurementData(measurements=measurements_out)
