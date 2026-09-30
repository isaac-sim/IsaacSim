# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

"""Tests for the intentionally small benchmark authoring facade."""

from __future__ import annotations

from typing import Any

import pytest
from isaacsim.performance.benchmarking import benchmark
from isaacsim.performance.benchmarking.impl.authoring import measurement_context


def test_function_decorator_preserves_callable_and_defaults() -> None:
    """Verify that a function decorator preserves the callable and its defaults."""

    @benchmark.subsystem("tests.authoring.function", warmup_samples=0, measured_samples=2)
    def work(count: int = 3) -> int:
        return count

    assert work() == 3
    definition = work.__benchmark_definition__
    assert definition.target is work
    assert definition.parameters == {"count": 3}
    assert definition.warmup_samples == 0
    assert definition.measured_samples == 2


def test_class_decorator_uses_constructor_defaults_and_fixed_lifecycle() -> None:
    """Verify that a class decorator captures defaults and uses the fixed lifecycle."""

    @benchmark.workflow("tests.authoring.class", warmup_frames=1, measured_frames=2)
    class Stateful:
        def __init__(self, value: int = 4) -> None:
            self.value = value

        def run(self) -> None:
            self.value += 1

    instance = Stateful()
    instance.run()
    assert instance.value == 5
    definition = Stateful.__benchmark_definition__
    assert definition.parameters == {"value": 4}
    assert definition.target is Stateful


def test_class_requires_run_and_parameter_defaults() -> None:
    """Reject stateful benchmarks without a run method or parameter defaults."""
    with pytest.raises(TypeError, match=r"require run\(\)"):

        @benchmark.subsystem("tests.authoring.no-run")
        class MissingRun:
            pass

    with pytest.raises(TypeError, match="requires a default"):

        @benchmark.operation("tests.authoring.no-default")
        def missing_default(value: int) -> None:
            del value


def test_measure_and_record_are_noops_outside_a_run() -> None:
    """Keep measurement helpers harmless outside a benchmark run."""
    with benchmark.measure("invalid name with spaces"):
        pass
    benchmark.record("invalid name with spaces", float("nan"), "")


def test_measure_aggregates_nested_regions_and_record_batches_values() -> None:
    """Aggregate nested regions and repeated recorded values."""
    with measurement_context("measurement", 7, retain=True) as state:
        with benchmark.measure("outer"):
            with benchmark.measure("inner"):
                pass
            with benchmark.measure("inner"):
                pass
        benchmark.record("items", 3, "count")
        benchmark.record("items", 4, "count")
        with pytest.raises(ValueError, match="changed units"):
            benchmark.record("items", 5, "items")

    assert set(state.region_values) == {"outer", "outer/inner"}
    assert len(state.region_values["outer/inner"]) == 2
    assert state.recorded_values == {"items": ([3.0, 4.0], "count")}


@pytest.mark.parametrize(
    ("value", "error"),
    (
        (True, TypeError),
        ("0.1", TypeError),
        (-0.1, ValueError),
        (float("inf"), TypeError),
    ),
)
def test_simulated_time_parameter_default_must_be_valid(value: Any, error: type[Exception]) -> None:
    """Reject invalid simulated-time parameter defaults.

    Args:
        value: Invalid default value under test.
        error: Expected validation exception.

    """
    with pytest.raises(error):

        @benchmark.subsystem(
            "tests.authoring.invalid-simulated-time",
            metrics=("simulation",),
            simulated_time="dt",
        )
        def work(dt: Any = value) -> None:
            del dt
