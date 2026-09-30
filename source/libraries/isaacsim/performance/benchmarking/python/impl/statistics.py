# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

"""Compute statistics from preserved raw benchmark samples."""

from __future__ import annotations

import math
import statistics


def percentile(samples: list[float], probability: float) -> float:
    """Return a linearly interpolated percentile without deleting outliers.

    Args:
        samples: Raw sample values.
        probability: Percentile probability between zero and one.

    Returns:
        Interpolated percentile value.

    """
    if not samples:
        raise ValueError("percentiles require at least one sample")
    if not 0.0 <= probability <= 1.0:
        raise ValueError("percentile probability must be between zero and one")
    ordered = sorted(samples)
    position = (len(ordered) - 1) * probability
    lower = math.floor(position)
    upper = math.ceil(position)
    if lower == upper:
        return ordered[lower]
    weight = position - lower
    return ordered[lower] * (1.0 - weight) + ordered[upper] * weight


def summarize_samples(samples: list[float]) -> dict[str, float | int]:
    """Report the complete required summary over preserved raw samples.

    Args:
        samples: Raw sample values.

    Returns:
        Summary statistics, or an empty mapping when no samples exist.

    """
    if not samples:
        return {}
    return {
        "count": len(samples),
        "mean": statistics.fmean(samples),
        "median": statistics.median(samples),
        "standard_deviation": statistics.pstdev(samples),
        "minimum": min(samples),
        "maximum": max(samples),
        "p50": percentile(samples, 0.50),
        "p95": percentile(samples, 0.95),
        "p99": percentile(samples, 0.99),
    }
