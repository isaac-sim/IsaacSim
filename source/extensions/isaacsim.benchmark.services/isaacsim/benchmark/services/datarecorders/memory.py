# SPDX-FileCopyrightText: Copyright (c) 2024-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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
import os
import platform
from pathlib import Path
from typing import TYPE_CHECKING, Dict, Optional, Tuple

import omni.client
import omni.stats
import psutil

from .. import stats
from ..metrics import measurements
from ..utils import set_up_logging
from .interface import InputContext, MeasurementData, MeasurementDataRecorder

if TYPE_CHECKING:
    from ..settings import BenchmarkSettings


logger = set_up_logging(__name__)


class GPUStats(stats.OmniStats):
    """
    GPU Pipelines
    GPU Memory (Total, plus seperate for each GPU)
    GPU Utilization
    """

    def __init__(self):
        self._stats_if = omni.stats.get_stats_interface()
        scopes = self._stats_if.get_scopes()
        self._scopes = [x for x in scopes if x["name"] not in ["USD Stage", "RTX Scene"]]


class HydraEngineMemoryStats:
    """
    this needs omni.hydra.engine
    """

    names = ["Hydra Engine Mem Stats"]

    def __init__(self):
        self._memStat_nodes = get_mem_stats(True)

    def get_stats(self) -> Dict:
        stats_dict = {}
        for stat_node in self._memStat_nodes:
            memStat_name = stat_node["category"].replace(" ", "_")
            memStat_size = stat_node["size"]
            stats_dict[memStat_name] = memStat_size

        return {self.names[0]: stats_dict}


class MemoryRecorder(MeasurementDataRecorder):
    """
    Gathers some basic System Memory and GPU Memory stats
    """

    def get_data(self) -> MeasurementData:

        (
            rss,
            vms,
            uss,
            pb,
            tracked_gpu_memory,
            dedicated_gpu_memory,
        ) = self.get_hardware_stats()

        m1 = measurements.SingleMeasurement(name="System Memory RSS", value=rss, unit="GB")
        m2 = measurements.SingleMeasurement(name="System Memory VMS", value=vms, unit="GB")
        m3 = measurements.SingleMeasurement(name="System Memory USS", value=uss, unit="GB")
        m4 = measurements.SingleMeasurement(name="GPU Memory Tracked", value=tracked_gpu_memory, unit="GB")
        m5 = measurements.SingleMeasurement(name="GPU Memory Dedicated", value=dedicated_gpu_memory, unit="GB")
        measurements_out = [m1, m2, m3, m4, m5]

        # Only capture System Memory PB for Windows.
        if platform.system() == "Windows":
            measurements_out.append(measurements.SingleMeasurement(name="System Memory PB", value=pb, unit="GB"))

        return MeasurementData(measurements=measurements_out)

    def get_hardware_stats(
        self,
    ) -> Tuple[float, float, float, float, float, float, float]:
        """Get hardware stats."""

        # RAM used for kit.exe.
        process = psutil.Process(os.getpid())
        # Physical Memory Working Set
        rss_mb = process.memory_info().rss / (1024**2)  # MB
        rss = round(rss_mb / 1024, 3)  # GB
        # Virtual Memory Private Bytes
        vms_mb = process.memory_info().vms / (1024**2)  # MB
        vms = round(vms_mb / 1024, 3)  # GB
        # Unique Set Size
        uss_mb = process.memory_full_info().uss / (1024**2)  # MB
        uss = round(uss_mb / 1024, 3)  # GB

        # Private Bytes (Windows only)
        pb = 0  # Set pb to 0 if running on Linux
        if platform.system() == "Windows":
            pb_mb = process.memory_full_info().private / (1024**2)  # MB
            pb = round(pb_mb / 1024, 3)  # GB

        # GPU from profiler window.
        memStat_sort = True
        memStat_detail = False
        tracked_gpu_memory = 0.0
        memStat_nodes = None
        try:
            from omni.hydra.engine.stats import get_mem_stats

            memStat_nodes = get_mem_stats(memStat_detail)
        except:
            pass

        # Sort nodes in descending order based on time if requested
        if memStat_nodes is not None:
            if memStat_sort is True:
                memStat_nodes = sorted(memStat_nodes, key=lambda node: node["size"], reverse=True)

            for node in memStat_nodes:
                if node["category"] == "Total Physical GPU Memory":
                    tracked_gpu_memory = round(node["size"] / 1024, 3)  # MB to GB

        dedicated_gpu_memory = 0
        try:
            from omni.hydra.engine.stats import get_device_info

            devices = get_device_info()
            if len(devices) > 0:
                device = devices[0]
                dedicated_gpu_memory = round(device["usage"] / 1073741824, 3)  # bytes to GB
        except:
            pass

        return rss, vms, uss, pb, tracked_gpu_memory, dedicated_gpu_memory
