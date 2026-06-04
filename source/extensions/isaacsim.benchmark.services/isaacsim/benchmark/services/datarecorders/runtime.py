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

"""Recorder for wall-clock runtime per benchmark phase."""

import omni.kit.app

from .. import utils
from ..metrics import measurements
from .interface import InputContext, MeasurementData, MeasurementDataRecorder, MeasurementDataRecorderRegistry

logger = utils.set_up_logging(__name__)


@MeasurementDataRecorderRegistry.register("runtime")
class RuntimeRecorder(MeasurementDataRecorder):
    """Record wall-clock runtime of a benchmark phase.

    Args:
        context: Input context for the recorder.
    """

    def __init__(self, context: InputContext | None = None) -> None:
        self.context = context
        self._start_ms: float = 0.0
        self._elapsed_ms: float | None = None
        self._phase: str | None = None

    def start_collecting(self) -> None:
        """Start timing.

        Example:

        .. code-block:: python

            recorder.start_collecting()
        """
        if self.context:
            self._phase = self.context.phase

        self._start_ms = omni.kit.app.get_app().get_time_since_start_ms()
        logger.info("RuntimeRecorder: Started timing at %fms", self._start_ms)

    def stop_collecting(self) -> None:
        """Stop timing.

        Example:

        .. code-block:: python

            recorder.stop_collecting()
        """
        self._elapsed_ms = omni.kit.app.get_app().get_time_since_start_ms() - self._start_ms
        logger.info("RuntimeRecorder: Stopped timing. Elapsed: %fms", self._elapsed_ms)

    def get_data(self) -> MeasurementData:
        """Get runtime measurement.

        Returns:
            Collected runtime measurement.

        Example:

        .. code-block:: python

            data = recorder.get_data()
        """
        if self.context and self._phase != self.context.phase:
            return MeasurementData()

        if self._elapsed_ms is None:
            logger.warning("RuntimeRecorder: No timing data collected")
            return MeasurementData()

        return MeasurementData(
            measurements=[measurements.SingleMeasurement(name="Runtime", value=round(self._elapsed_ms, 3), unit="ms")]
        )
