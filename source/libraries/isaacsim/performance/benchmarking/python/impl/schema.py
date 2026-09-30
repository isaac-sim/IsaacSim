# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

"""Serialize the single benchmark result document."""

from __future__ import annotations

from collections import defaultdict
from datetime import datetime, timezone
from typing import Any

from isaacsim.performance.benchmarking.impl.model import _BenchmarkResult
from isaacsim.performance.benchmarking.impl.statistics import summarize_samples

RESULT_SCHEMA = "isaacsim.performance.benchmarking.result.v2"


def _metric_documents(result: _BenchmarkResult) -> dict[str, Any]:
    """Build metric documents from independent raw samples.

    Args:
        result: Raw benchmark result.

    Returns:
        Metric documents keyed by metric name.

    """
    raw_values: dict[str, list[tuple[int, float]]] = defaultdict(list)
    for sample in result.samples:
        raw_values["duration_ms"].append((sample.index, sample.duration_ms))
        raw_values["total_duration_ms"].append((sample.index, sample.total_duration_ms))
        raw_values["throughput_per_s"].append((sample.index, sample.throughput_per_s))
        for name, value in sample.metrics.items():
            raw_values[name].append((sample.index, value))
    documents: dict[str, Any] = {}
    for name, values in sorted(raw_values.items()):
        numeric_values = [value for _, value in values]
        documents[name] = {
            "unit": result.metric_units[name],
            "samples": [{"sample": sample, "value": value} for sample, value in values],
            "statistics": summarize_samples(numeric_values),
        }
    return documents


def _region_documents(result: _BenchmarkResult) -> dict[str, Any]:
    """Group region aggregates by their explicit hierarchical name.

    Args:
        result: Raw benchmark result.

    Returns:
        Region documents keyed by measurement name.

    """
    grouped: dict[str, list[dict[str, Any]]] = defaultdict(list)
    for observation in result.regions:
        grouped[observation.name].append(
            {
                "phase": observation.phase,
                "sample": observation.sample,
                "count": observation.count,
                "total_ms": observation.total_ms,
                "mean_ms": observation.total_ms / observation.count,
                "minimum_ms": observation.minimum_ms,
                "maximum_ms": observation.maximum_ms,
            }
        )
    documents: dict[str, Any] = {}
    for name, observations in sorted(grouped.items()):
        measured = [entry for entry in observations if entry["sample"] is not None]
        documents[name] = {
            "unit": "ms",
            "samples": observations,
            "statistics": {
                "total_ms": summarize_samples([entry["total_ms"] for entry in measured]),
                "mean_ms": summarize_samples([entry["mean_ms"] for entry in measured]),
            },
        }
    return documents


def result_document(result: _BenchmarkResult) -> dict[str, Any]:
    """Convert an internal result to the public JSON representation.

    Args:
        result: Raw benchmark result.

    Returns:
        Versioned public result document.

    """
    definition = result.definition
    batch_sizes = sorted({sample.batch_size for sample in result.samples})
    return {
        "schema": RESULT_SCHEMA,
        "generated_at": datetime.now(timezone.utc).isoformat(),
        "run_id": result.run_id,
        "benchmark": {
            "id": definition.identifier,
            "component": definition.component,
            "scope": definition.scope,
            "parameters": result.parameters,
            "resources": list(definition.resources),
            "duration": definition.duration,
            "metric_profiles": list(definition.metric_profiles),
            "seed": result.seed,
            "completion": definition.completion,
        },
        "sampling": {
            "warmup_samples": definition.warmup_samples,
            "measured_samples": definition.measured_samples,
            "completed_samples": len(result.samples),
            "batch_sizes": batch_sizes,
            "calibrated_batching": definition.calibrated_batching,
        },
        "status": {"valid": result.valid, "errors": result.errors},
        "lifecycle": {**result.lifecycle_ms, "validation": result.validation},
        "metrics": _metric_documents(result),
        "regions": _region_documents(result),
        "environment": result.environment,
    }
