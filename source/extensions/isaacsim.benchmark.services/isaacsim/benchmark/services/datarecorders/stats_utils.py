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

"""Statistics utilities for benchmark samples."""

import math
import statistics
from dataclasses import dataclass


@dataclass
class Stats:
    """Statistical summary of a sample set.

    Args:
        mean: Mean value.
        median: Median value.
        stdev: Standard deviation.
        min: Minimum value.
        max: Maximum value.
        p99: 99th percentile value.
    """

    mean: float
    median: float
    stdev: float
    min: float
    max: float
    p99: float

    @classmethod
    def from_samples(cls, samples: list[float], trim_outliers: bool = True) -> "Stats":
        """Calculate statistics from samples.

        Args:
            samples: List of sample values.
            trim_outliers: If True and len(samples) >= 100, remove top/bottom 10%.

        Returns:
            Statistics object with calculated values.

        Example:

        .. code-block:: python

            stats = Stats.from_samples(samples)
        """
        if not samples:
            return cls(0.0, 0.0, 0.0, 0.0, 0.0, 0.0)

        working_samples = samples
        if trim_outliers and len(samples) >= 100:
            working_samples = cls._trim_outliers(samples)

        return cls(
            mean=round(statistics.mean(working_samples), 2),
            median=round(statistics.median(working_samples), 2),
            stdev=round(statistics.stdev(working_samples), 2) if len(working_samples) > 1 else 0.0,
            min=round(min(working_samples), 2),
            max=round(max(working_samples), 2),
            p99=round(cls._percentile(sorted(working_samples), 0.99), 2),
        )

    @staticmethod
    def _trim_outliers(samples: list[float]) -> list[float]:
        """Remove top and bottom 10% of samples.

        Args:
            samples: List of sample values.

        Returns:
            Middle 80% of samples.
        """
        sorted_samples = sorted(samples)
        trim_count = len(sorted_samples) // 10
        return sorted_samples[trim_count:-trim_count] if trim_count > 0 else sorted_samples

    @staticmethod
    def _percentile(sorted_samples: list[float], p: float) -> float:
        """Calculate percentile from sorted samples.

        Args:
            sorted_samples: Pre-sorted list of values.
            p: Percentile as fraction from 0.0 to 1.0.

        Returns:
            Value at percentile p.
        """
        if not sorted_samples:
            return 0.0

        k = (len(sorted_samples) - 1) * p
        f = math.floor(k)
        c = math.ceil(k)

        if f == c:
            return sorted_samples[int(k)]

        return sorted_samples[f] * (c - k) + sorted_samples[c] * (k - f)

    def to_dict(self) -> dict:
        """Convert to dictionary format.

        Returns:
            Dictionary of statistic values.

        Example:

        .. code-block:: python

            stats_dict = stats.to_dict()
        """
        return {
            "mean": self.mean,
            "median": self.median,
            "stdev": self.stdev,
            "min": self.min,
            "max": self.max,
            "one_percent": self.p99,
        }
