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

"""Recorder for PhysX simulation step frametime metrics."""

from typing import Any

import omni.physics.core

from .. import utils
from ..metrics import measurements
from .interface import InputContext, MeasurementData, MeasurementDataRecorder, MeasurementDataRecorderRegistry
from .stats_utils import Stats

logger = utils.set_up_logging(__name__)


@MeasurementDataRecorderRegistry.register("physics_frametime")
class PhysicsFrametimeRecorder(MeasurementDataRecorder):
    """Record PhysX simulation step frametime.

    Args:
        context: Input context for the recorder.
    """

    def __init__(self, context: InputContext | None = None) -> None:
        self.context = context
        self._samples: list[float] = []
        self._subscription = None
        self._physics_iface = None
        self._phase: str | None = None

        try:
            self._physics_iface = omni.physics.core.get_physics_benchmarks_interface()
        except Exception as e:
            logger.warning(f"PhysicsFrametimeRecorder: Failed to get physics interface: {e}")

    def start_collecting(self) -> None:
        """Start collecting physics frametime data.

        Example:

        .. code-block:: python

            recorder.start_collecting()
        """
        if self.context:
            self._phase = self.context.phase

        self._samples = []

        if self._physics_iface:
            self._subscription = self._physics_iface.subscribe_profile_stats_events(self._on_physics_stats)
            logger.info("PhysicsFrametimeRecorder: Started collecting")
        else:
            logger.warning("PhysicsFrametimeRecorder: Physics interface not available")

    def stop_collecting(self) -> None:
        """Stop collecting physics frametime data.

        Example:

        .. code-block:: python

            recorder.stop_collecting()
        """
        self._subscription = None

        if self._samples:
            self._samples.pop(0)

        logger.info("PhysicsFrametimeRecorder: Stopped collecting. Collected %d samples", len(self._samples))

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

    def _on_physics_stats(self, profile_stats: Any) -> None:
        """Callback for physics profile stats.

        Args:
            profile_stats: Physics profiling statistics payload.
        """
        for stat in profile_stats:
            if stat.zone_name == "PhysX Update":
                self._samples.append(stat.ms)

    def get_data(self) -> MeasurementData:
        """Get physics frametime measurements.

        Returns:
            Collected physics frametime statistics.

        Example:

        .. code-block:: python

            data = recorder.get_data()
        """
        if self.context and self._phase != self.context.phase:
            return MeasurementData()

        if not self._samples:
            logger.info("PhysicsFrametimeRecorder: No samples collected (physics may not be running)")
            return MeasurementData()

        stats = Stats.from_samples(self._samples, trim_outliers=False)

        return MeasurementData(
            measurements=[
                measurements.SingleMeasurement(name="Mean Physics Frametime", value=stats.mean, unit="ms"),
                measurements.SingleMeasurement(name="Stdev Physics Frametime", value=stats.stdev, unit="ms"),
                measurements.SingleMeasurement(name="Min Physics Frametime", value=stats.min, unit="ms"),
                measurements.SingleMeasurement(name="Max Physics Frametime", value=stats.max, unit="ms"),
                measurements.ListMeasurement(name="Physics Frametime Samples", value=self._samples),
            ]
        )
