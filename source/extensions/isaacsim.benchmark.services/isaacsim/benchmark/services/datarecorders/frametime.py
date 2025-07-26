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
import math
import statistics
from dataclasses import dataclass, field
from pathlib import Path
from typing import TYPE_CHECKING, List, Optional

from .. import utils

if TYPE_CHECKING:
    from ..settings import BenchmarkSettings

logger = utils.set_up_logging(__name__)


@dataclass
class FrametimeStats:
    app_frametime_samples: List[float] = field(default_factory=list)
    gpu_frametime_samples: List[float] = field(default_factory=list)
    physics_frametime_samples: List[float] = field(default_factory=list)
    renderer_frametime_samples: List[float] = field(default_factory=list)

    # Multi-GPU support
    per_gpu_frametime_samples: List[List[float]] = field(default_factory=list)

    app_stats = {}
    physics_stats = {}
    gpu_stats = {}
    renderer_stats = {}
    per_gpu_stats = {}

    def _percentile_inc(self, values: List, percent: float, key=lambda x: x) -> float:
        """
        Find the percentile of a list of values.

        Args:
            values: is a list of values. Note N MUST BE already sorted.
            percent: a float value from 0.0 to 1.0.
            key: optional key function to compute value from each element of N.

        Returns:
            The percentile of the values
        """
        k = (len(values) - 1) * percent
        f = math.floor(k)
        c = math.ceil(k)
        if f == c:
            return key(values[int(k)])
        d0 = key(values[int(f)]) * (c - k)
        d1 = key(values[int(c)]) * (k - f)
        return d0 + d1

    def get_one_percent_high(self, values: List) -> float:
        """
        Given a list of floats, return the average of the largest 1% values.

        Args:
            values: A list of floats

        Returns:
            An average of the 1% largest values
        """
        ninety_nine_p = self._percentile_inc(sorted(values), 0.99)
        return statistics.mean([x for x in values if x >= ninety_nine_p])

    def trim_outliers(self, values: List) -> List:
        """
        Given a list of floats, remove the top 10% and bottom 10% of the values,
        if the list is over 100 values long.

        Args:
            values: A list of floats

        Returns:
            The middle 80% of values
        """
        if len(values) < 100:
            return values

        sorted_data = sorted(values)
        num_values = len(sorted_data)
        remove_count = math.floor(num_values * 0.1)

        trimmed_data = sorted_data[remove_count:-remove_count]
        return trimmed_data

    def stats_helper(self, metric: List[float]):
        result = {"mean": 0, "median": 0, "stdev": 0, "min": 0, "max": 0, "one_percent": 0}
        try:
            metric = self.trim_outliers(metric)
            result["mean"] = round(statistics.mean(metric), 2)
            result["median"] = round(statistics.median(metric), 2)
            result["stdev"] = round(statistics.stdev(metric), 2)
            result["min"] = round(min(metric), 2)
            result["max"] = round(max(metric), 2)
            result["one_percent"] = round(self.get_one_percent_high(metric), 2)
        except Exception as e:
            logger.warn(f"Unable to calculate frametime stats: {e}")
        return result

    def calc_stats(self) -> None:
        self.app_stats = self.stats_helper(self.app_frametime_samples)
        self.physics_stats = self.stats_helper(self.physics_frametime_samples)
        self.gpu_stats = self.stats_helper(self.gpu_frametime_samples)
        self.renderer_stats = self.stats_helper(self.renderer_frametime_samples)

        # Multi-GPU support
        self.per_gpu_stats = []
        for gpu_samples in self.per_gpu_frametime_samples:
            if gpu_samples:
                self.per_gpu_stats.append(self.stats_helper(gpu_samples))
            else:
                self.per_gpu_stats.append({"mean": 0, "median": 0, "stdev": 0, "min": 0, "max": 0, "one_percent": 0})
