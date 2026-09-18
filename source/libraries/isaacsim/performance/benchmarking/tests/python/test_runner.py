# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

"""Tests for benchmark lifecycle, sampling, validation, and metrics."""

from __future__ import annotations

import asyncio

import pytest
from isaacsim.performance.benchmarking import benchmark
from isaacsim.performance.benchmarking.impl import profiles, runner
from isaacsim.performance.benchmarking.impl.runner import run_definition
from isaacsim.performance.benchmarking.impl.schema import result_document


def test_stateful_lifecycle_has_no_hidden_invocation_and_validates_once() -> None:
    """Run each stateful lifecycle boundary the documented number of times."""
    events: list[str] = []

    @benchmark.subsystem("tests.runner.lifecycle", warmup_samples=2, measured_samples=3)
    class Stateful:
        def setup(self) -> None:
            events.append("setup")

        def prepare_sample(self) -> None:
            events.append("prepare")

        def run(self) -> None:
            events.append("run")

        def validate(self) -> bool:
            events.append("validate")
            return True

        def teardown(self) -> None:
            events.append("teardown")

    result = asyncio.run(run_definition(Stateful.__benchmark_definition__, run_id="lifecycle"))
    assert result.valid
    assert events.count("run") == 5
    assert events.count("prepare") == 5
    assert events.count("validate") == 1
    assert events[0] == "setup"
    assert events[-1] == "teardown"


def test_partial_setup_failure_still_tears_down() -> None:
    """Run teardown after a partial setup failure."""
    events: list[str] = []

    @benchmark.subsystem("tests.runner.setup-failure", warmup_samples=0, measured_samples=1)
    class Stateful:
        def setup(self) -> None:
            events.append("setup")
            raise RuntimeError("partial setup")

        def run(self) -> None:
            events.append("run")

        def teardown(self) -> None:
            events.append("teardown")

    result = asyncio.run(run_definition(Stateful.__benchmark_definition__))
    assert not result.valid
    assert events == ["setup", "teardown"]
    assert "partial setup" in result.errors[0]


def test_cancellation_during_measurement_still_tears_down() -> None:
    """Run teardown when measurement is canceled."""
    events: list[str] = []

    @benchmark.subsystem("tests.runner.canceled", warmup_samples=0, measured_samples=1)
    class Stateful:
        def setup(self) -> None:
            events.append("setup")

        async def run(self) -> None:
            events.append("run")
            raise asyncio.CancelledError

        def teardown(self) -> None:
            events.append("teardown")

    with pytest.raises(asyncio.CancelledError):
        asyncio.run(run_definition(Stateful.__benchmark_definition__))

    assert events == ["setup", "run", "teardown"]


def test_async_lifecycle_uses_one_event_loop() -> None:
    """Use one event loop throughout an asynchronous lifecycle."""
    loops: list[asyncio.AbstractEventLoop] = []

    @benchmark.workflow("tests.runner.async", warmup_frames=1, measured_frames=2)
    class AsyncStateful:
        async def setup(self) -> None:
            loops.append(asyncio.get_running_loop())

        async def run(self) -> None:
            loops.append(asyncio.get_running_loop())

        async def validate(self) -> bool:
            loops.append(asyncio.get_running_loop())
            return True

        async def teardown(self) -> None:
            loops.append(asyncio.get_running_loop())

    result = asyncio.run(run_definition(AsyncStateful.__benchmark_definition__))
    assert result.valid
    assert len(loops) == 6
    assert len({id(loop) for loop in loops}) == 1


def test_explicit_regions_and_custom_metrics_are_retained_per_sample() -> None:
    """Retain explicit regions and custom metrics for each measured sample."""
    value = 0

    @benchmark.workflow("tests.runner.regions", warmup_frames=1, measured_frames=2)
    def frame() -> None:
        nonlocal value
        value += 1
        with benchmark.measure("step"):
            with benchmark.measure("readback"):
                pass
        benchmark.record("height", value, "m")

    result = asyncio.run(run_definition(frame.__benchmark_definition__))
    document = result_document(result)
    assert result.valid
    assert [sample["value"] for sample in document["metrics"]["height"]["samples"]] == [2.0, 3.0]
    assert len(document["regions"]["step"]["samples"]) == 2
    assert len(document["regions"]["step/readback"]["samples"]) == 2


def test_operation_batch_averages_repeated_custom_metrics(monkeypatch: pytest.MonkeyPatch) -> None:
    """Average repeated custom metrics over an operation batch.

    Args:
        monkeypatch: Temporary calibration modifier.

    """
    invocation = 0

    async def batch_three(*args: object, **kwargs: object) -> int:
        del args, kwargs
        await asyncio.sleep(0)
        return 3

    monkeypatch.setattr(runner, "_calibrate_batch_size", batch_three)

    @benchmark.operation("tests.runner.batched-record", warmup_samples=0, measured_samples=1)
    def work() -> None:
        nonlocal invocation
        invocation += 1
        benchmark.record("invocation", invocation, "count")

    result = asyncio.run(run_definition(work.__benchmark_definition__))

    assert result.valid
    assert result.samples[0].batch_size == 3
    assert result.samples[0].metrics["invocation"] == 2.0


def test_process_and_simulation_profiles_have_concrete_samples() -> None:
    """Collect concrete process and simulated-time samples."""

    @benchmark.subsystem(
        "tests.runner.profiles",
        warmup_samples=0,
        measured_samples=2,
        metrics=("process", "simulation"),
        simulated_time="dt",
    )
    def step(dt: float = 0.1) -> None:
        pass

    result = asyncio.run(run_definition(step.__benchmark_definition__))
    document = result_document(result)
    assert result.valid
    assert len(document["metrics"]["process_rss_mib"]["samples"]) == 2
    assert [sample["value"] for sample in document["metrics"]["simulated_time_s"]["samples"]] == [0.1, 0.1]


def test_effective_simulated_time_is_validated_before_construction() -> None:
    """Validate effective simulated time before constructing benchmark state."""
    events: list[str] = []

    @benchmark.subsystem(
        "tests.runner.invalid-effective-simulated-time",
        warmup_samples=0,
        measured_samples=1,
        metrics=("simulation",),
        simulated_time="dt",
    )
    class Stateful:
        def __init__(self, dt: float = 0.1) -> None:
            events.append("construct")
            self.dt = dt

        def run(self) -> None:
            pass

    with pytest.raises(ValueError, match="finite and non-negative"):
        asyncio.run(run_definition(Stateful.__benchmark_definition__, {"dt": -0.1}))

    assert events == []


def test_gpu_profile_has_concrete_samples_and_closes_nvml(monkeypatch: pytest.MonkeyPatch) -> None:
    """Collect GPU samples and close the NVML monitor.

    Args:
        monkeypatch: Temporary GPU-monitor modifier.

    """
    snapshots = iter(
        (
            profiles._GpuSnapshot((profiles._GpuDeviceSnapshot(0.2, 1000.0, 100.0, True),)),
            profiles._GpuSnapshot((profiles._GpuDeviceSnapshot(0.7, 1200.0, 150.0, True),)),
        )
    )

    class FakeMonitor:
        def __init__(self) -> None:
            self.closed = False

        def snapshot(self) -> profiles._GpuSnapshot:
            return next(snapshots)

        def close(self) -> None:
            self.closed = True

    monitor = FakeMonitor()
    monkeypatch.setattr(runner, "_NvmlMonitor", lambda: monitor)

    @benchmark.subsystem("tests.runner.gpu", warmup_samples=0, measured_samples=1, metrics=("gpu",))
    def step() -> None:
        pass

    result = asyncio.run(run_definition(step.__benchmark_definition__))
    document = result_document(result)

    assert result.valid
    assert document["metrics"]["device_gpu_utilization"]["unit"] == "ratio"
    assert document["metrics"]["device_gpu_utilization"]["samples"][0]["value"] == 0.7
    assert document["metrics"]["device_vram_used_mib"]["samples"][0]["value"] == 1200.0
    assert document["metrics"]["process_vram_used_mib"]["samples"][0]["value"] == 150.0
    assert monitor.closed


def test_unavailable_requested_gpu_metrics_are_invalid_and_teardown_runs(monkeypatch: pytest.MonkeyPatch) -> None:
    """Invalidate unavailable requested GPU metrics and still run teardown.

    Args:
        monkeypatch: Temporary GPU-monitor modifier.

    """
    events: list[str] = []

    @benchmark.subsystem("tests.runner.gpu-unavailable", warmup_samples=0, measured_samples=1, metrics=("gpu",))
    class Stateful:
        def setup(self) -> None:
            events.append("setup")

        def run(self) -> None:
            events.append("run")

        def teardown(self) -> None:
            events.append("teardown")

    def unavailable_monitor() -> None:
        raise RuntimeError("NVML driver unavailable")

    monkeypatch.setattr(runner, "_NvmlMonitor", unavailable_monitor)

    result = asyncio.run(run_definition(Stateful.__benchmark_definition__))

    assert not result.valid
    assert events == ["setup", "teardown"]
    assert result.errors == ["measurement: RuntimeError: NVML driver unavailable"]


def test_validation_false_and_measurement_failure_are_invalid() -> None:
    """Mark false validation and measurement failures as invalid results."""

    @benchmark.subsystem("tests.runner.invalid", warmup_samples=0, measured_samples=1)
    class Invalid:
        def run(self) -> None:
            pass

        def validate(self) -> bool:
            return False

    result = asyncio.run(run_definition(Invalid.__benchmark_definition__))
    assert not result.valid
    assert result.validation == {"performed": True, "passed": False}
    assert result.errors == ["validation: validate() returned False"]
