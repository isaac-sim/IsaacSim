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

"""Recorder for static hardware specifications."""

import psutil
import warp as wp

from .. import utils
from ..metrics import measurements
from .interface import InputContext, MeasurementData, MeasurementDataRecorder, MeasurementDataRecorderRegistry

logger = utils.set_up_logging(__name__)


@MeasurementDataRecorderRegistry.register("hardware")
class HardwareSpecRecorder(MeasurementDataRecorder):
    """Record hardware specifications such as CPU count and GPU model.

    Args:
        context: Input context for the recorder.
    """

    def __init__(self, context: InputContext | None = None) -> None:
        self.context = context

    def get_data(self) -> MeasurementData:
        """Get hardware specification measurements.

        Returns:
            Collected hardware specification measurements.

        Example:

        .. code-block:: python

            data = recorder.get_data()
        """
        device_names = [device.name for device in wp.get_cuda_devices()]

        if len(set(device_names)) > 1:
            logger.warning(f"Detected multiple GPU types: {device_names}")
            logger.warning(f"Only recording GPU 0 type: {device_names[0]}")

        gpu_name = device_names[0] if device_names else "Unknown"

        return MeasurementData(
            measurements=[
                measurements.SingleMeasurement(name="num_cpus", value=psutil.cpu_count(), unit=""),
                measurements.SingleMeasurement(name="gpu_device_name", value=gpu_name, unit=""),
            ]
        )
