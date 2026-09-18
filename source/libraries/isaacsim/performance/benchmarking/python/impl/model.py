# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

"""Private data structures shared by benchmark authoring and execution."""

from __future__ import annotations

from collections.abc import Callable
from dataclasses import dataclass, field
from typing import Any


@dataclass(frozen=True)
class _BenchmarkDefinition:
    """Static metadata captured by a benchmark decorator."""

    identifier: str
    component: str
    scope: str
    target: Callable[..., Any] | type[Any]
    parameters: dict[str, Any]
    resources: tuple[str, ...]
    duration: str
    metric_profiles: tuple[str, ...]
    seed: int
    completion: str
    simulated_time: float | str | None
    warmup_samples: int
    measured_samples: int
    calibrated_batching: bool


@dataclass
class _RegionObservation:
    """Aggregated observations for one named region in one sample or phase."""

    name: str
    phase: str
    sample: int | None
    count: int
    total_ms: float
    minimum_ms: float
    maximum_ms: float


@dataclass
class _ActiveMeasurement:
    """Context-local state used by ``benchmark.measure`` and ``record``."""

    phase: str = "inactive"
    sample: int | None = None
    retain: bool = False
    region_values: dict[str, list[float]] = field(default_factory=dict)
    recorded_values: dict[str, tuple[list[float], str]] = field(default_factory=dict)


@dataclass
class _SampleResult:
    """Raw values retained for one independent measurement sample."""

    index: int
    batch_size: int
    duration_ms: float
    total_duration_ms: float
    throughput_per_s: float
    metrics: dict[str, float] = field(default_factory=dict)


@dataclass
class _BenchmarkResult:
    """Complete result assembled by the benchmark runner."""

    run_id: str
    definition: _BenchmarkDefinition
    parameters: dict[str, Any]
    seed: int
    valid: bool = False
    errors: list[str] = field(default_factory=list)
    samples: list[_SampleResult] = field(default_factory=list)
    metric_units: dict[str, str] = field(default_factory=dict)
    regions: list[_RegionObservation] = field(default_factory=list)
    lifecycle_ms: dict[str, float] = field(default_factory=dict)
    validation: dict[str, Any] = field(default_factory=lambda: {"performed": False, "passed": False})
    environment: dict[str, Any] = field(default_factory=dict)
