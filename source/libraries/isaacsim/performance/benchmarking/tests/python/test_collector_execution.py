# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

"""Verify resource integration preserves execution and result contracts."""

from __future__ import annotations

import asyncio
import json
from contextlib import nullcontext
from pathlib import Path
from types import SimpleNamespace
from typing import Any

import pytest
from isaacsim.performance.benchmarking import benchmark
from isaacsim.performance.benchmarking.impl import reporting, runner
from isaacsim.performance.benchmarking.impl.profiles import _ProcessSnapshot


@pytest.fixture
def resource_provider(monkeypatch: pytest.MonkeyPatch) -> type:
    """Replace external observation I/O while retaining the real runner.

    Args:
        monkeypatch: Temporary attribute and environment overrides.

    Returns:
        Provider class with configurable observations and recorded lifecycle calls.

    """

    class ResourceProvider:
        instances: list[Any] = []
        spend: Any = staticmethod(lambda: None)
        fail_close = False

        def __init__(self, metric_profiles: tuple[str, ...], run_id: str, *, backend: str, capture: bool) -> None:
            self.profiles = metric_profiles
            self.provenance = {"backend": backend}
            self.capture = capture
            self.capture_document = {"schema": "isaacsim.performance.benchmarking.collector.v1", "run_id": run_id}
            self.calls: list[str] = []
            self.boundaries = 0
            self.inputs: list[dict[str, Any]] = []
            type(self).instances.append(self)

        def start(self) -> None:
            self.calls.append("start")

        def begin_measurement(self) -> None:
            self.calls.append("begin")

        def end_measurement(self) -> None:
            self.calls.append("end")

        def snapshot(self) -> SimpleNamespace:
            self.spend()
            self.boundaries += 1
            return SimpleNamespace(
                process=_ProcessSnapshot(rss_mib=40.0),
                gpu=None,
                raw={"timestamp": float(self.boundaries)},
            )

        def range(self, name: str) -> Any:
            del name
            return nullcontext()

        def record_sample(
            self,
            index: int,
            batch: int,
            elapsed: float,
            before: Any,
            after: Any,
            *,
            cpu_before: float,
            cpu_after: float,
        ) -> None:
            self.inputs.append({"index": index, "batch_size": batch, "elapsed_seconds": elapsed})

        def close(self) -> None:
            self.calls.append("close")
            self.capture_document = {**self.capture_document, "samples": self.inputs} if self.capture else None
            if self.fail_close:
                raise RuntimeError("provider cleanup failed")

    monkeypatch.setattr(runner, "_ResourceCollector", ResourceProvider, raising=False)
    return ResourceProvider


@pytest.mark.parametrize("asynchronous", [False, True])
def test_public_execution_forwards_backend_and_preserves_custom_values(
    resource_provider: type, asynchronous: bool
) -> None:
    """Both public entry points preserve repeated custom observations.

    Args:
        resource_provider: Fixture supplying an injected observation provider and its recorded calls.
        asynchronous: Whether to exercise the asynchronous public entry point.

    """

    @benchmark.subsystem("tests.collector.forwarding", metrics=("process",), warmup_samples=0, measured_samples=1)
    def work() -> None:
        benchmark.record("height", 4, "m")
        benchmark.record("height", 8, "m")

    options = {"resource_backend": "builtin", "capture_collector": False, "save": False}
    document = asyncio.run(benchmark.run_async(work, **options)) if asynchronous else benchmark.run(work, **options)
    assert document["status"]["valid"]
    assert document["metrics"]["height"]["samples"][0]["value"] == 6.0
    assert document["environment"]["collector"]["backend"] == "builtin"
    assert resource_provider.instances[0].calls == ["start", "begin", "end", "close"]


def test_resource_acquisition_is_outside_sample_clock(resource_provider: type, monkeypatch: pytest.MonkeyPatch) -> None:
    """Slow resource reads do not alter measured workload duration.

    Args:
        resource_provider: Fixture supplying an injected observation provider and its recorded calls.
        monkeypatch: Temporary attribute and environment overrides.

    """
    current = 0
    cpu_seconds = 0.0

    def spend() -> None:
        nonlocal current, cpu_seconds
        current += 100_000_000
        cpu_seconds += 0.1

    resource_provider.spend = staticmethod(spend)
    monkeypatch.setattr(runner.time, "process_time", lambda: cpu_seconds)

    @benchmark.subsystem("tests.collector.clock", metrics=("process",), warmup_samples=0, measured_samples=1)
    def work() -> None:
        nonlocal current, cpu_seconds
        current += 5_000_000
        cpu_seconds += 0.002

    result = asyncio.run(
        runner.run_definition(work.__benchmark_definition__, clock_ns=lambda: current, resource_backend="ovperf")
    )
    assert result.valid
    assert result.samples[0].duration_ms == 5.0
    assert result.samples[0].metrics["process_cpu_time_s"] == pytest.approx(0.002)
    assert result.samples[0].metrics["process_cpu_utilization"] == pytest.approx(0.4)


def test_measurement_failure_keeps_primary_error_and_closes_once(resource_provider: type) -> None:
    """Provider cleanup cannot replace the original workload error.

    Args:
        resource_provider: Fixture supplying an injected observation provider and its recorded calls.

    """
    events: list[str] = []
    resource_provider.fail_close = True

    @benchmark.subsystem("tests.collector.failure", metrics=("process",), warmup_samples=0, measured_samples=1)
    class Work:
        def run(self) -> None:
            raise ValueError("workload failed")

        def teardown(self) -> None:
            events.append("teardown")

    result = asyncio.run(runner.run_definition(Work.__benchmark_definition__, resource_backend="ovperf"))
    assert not result.valid
    assert "workload failed" in result.errors[0]
    assert any("provider cleanup failed" in error for error in result.errors)
    assert resource_provider.instances[0].calls.count("close") == 1
    assert events == ["teardown"]


def test_cancelled_workload_closes_provider_and_tears_down(resource_provider: type) -> None:
    """Async cancellation propagates after owned resources are released.

    Args:
        resource_provider: Fixture supplying an injected observation provider and its recorded calls.

    """
    events: list[str] = []
    resource_provider.fail_close = True

    @benchmark.subsystem("tests.collector.cancel", metrics=("process",), warmup_samples=0, measured_samples=1)
    class Work:
        async def run(self) -> None:
            raise asyncio.CancelledError

        def teardown(self) -> None:
            events.append("teardown")

    with pytest.raises(asyncio.CancelledError):
        asyncio.run(runner.run_definition(Work.__benchmark_definition__, resource_backend="ovperf"))
    assert resource_provider.instances[0].calls.count("close") == 1
    assert events == ["teardown"]


def test_capture_writes_referenced_sidecar_without_changing_result_shape(
    resource_provider: type, tmp_path: Path
) -> None:
    """Optional native observations remain outside the result.v3 top level.

    Args:
        resource_provider: Fixture supplying an injected observation provider and its recorded calls.
        tmp_path: Isolated directory for result and collector artifacts.

    """

    @benchmark.subsystem("tests.collector.capture", metrics=("process",), warmup_samples=0, measured_samples=1)
    def work() -> None:
        pass

    path = tmp_path / "result.json"
    document = benchmark.run(work, output=path, resource_backend="ovperf", capture_collector=True)
    artifact = document["environment"]["collector"]["artifact"]
    assert Path(artifact).name == artifact
    sidecar = json.loads((tmp_path / artifact).read_text())
    assert sidecar["run_id"] == document["run_id"]
    assert sidecar["samples"][0]["index"] == 0
    assert json.loads(path.read_text()) == document
    assert set(document) == {
        "schema",
        "generated_at",
        "run_id",
        "benchmark",
        "sampling",
        "status",
        "lifecycle",
        "metrics",
        "regions",
        "environment",
    }
    assert sorted(p.name for p in tmp_path.iterdir()) == sorted(["result.json", artifact])


@pytest.mark.parametrize("asynchronous", [False, True])
def test_capture_cannot_silently_discard_requested_data(tmp_path: Path, asynchronous: bool) -> None:
    """Reject capture without persistence before running user code.

    Args:
        tmp_path: Isolated directory for result and collector artifacts.
        asynchronous: Whether to exercise the asynchronous public entry point.

    """
    calls: list[str] = []

    @benchmark.subsystem("tests.collector.no-save", warmup_samples=0, measured_samples=1)
    def work() -> None:
        calls.append("run")

    with pytest.raises(ValueError, match="capture_collector.*save"):
        if asynchronous:
            asyncio.run(benchmark.run_async(work, save=False, capture_collector=True))
        else:
            benchmark.run(work, save=False, capture_collector=True)
    assert not calls
    assert not list(tmp_path.iterdir())


def test_failed_result_replacement_preserves_previous_capture(
    resource_provider: type,
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    """Keep the previous result and collector artifact readable after a failed overwrite.

    Args:
        resource_provider: Fixture supplying an injected observation provider and its recorded calls.
        tmp_path: Isolated directory for result and collector artifacts.
        monkeypatch: Temporary attribute and environment overrides.

    """

    @benchmark.subsystem("tests.collector.atomic", metrics=("process",), warmup_samples=0, measured_samples=1)
    def work() -> None:
        pass

    path = tmp_path / "result.json"
    previous = benchmark.run(work, output=path, resource_backend="ovperf", capture_collector=True)
    previous_artifact = tmp_path / previous["environment"]["collector"]["artifact"]
    previous_capture = previous_artifact.read_bytes()
    replace = reporting.os.replace

    def fail_result(source: Any, destination: Any) -> None:
        if Path(destination) == path:
            raise PermissionError("Result cannot be replaced")
        replace(source, destination)

    monkeypatch.setattr(reporting.os, "replace", fail_result)
    with pytest.raises(PermissionError, match="Result cannot be replaced"):
        benchmark.run(work, output=path, resource_backend="ovperf", capture_collector=True)
    assert json.loads(path.read_text()) == previous
    assert previous_artifact.read_bytes() == previous_capture
    assert set(tmp_path.iterdir()) == {path, previous_artifact}


@pytest.mark.parametrize("backend", ["", "unknown", 1, None])
def test_invalid_backend_is_rejected_before_user_code(backend: Any) -> None:
    """Validate runtime provider selection at the public boundary.

    Args:
        backend: Resource backend value under test.

    """

    @benchmark.subsystem("tests.collector.bad-backend", warmup_samples=0, measured_samples=1)
    def work() -> None:
        pytest.fail("Invalid backend must not execute user code")

    with pytest.raises((ValueError, TypeError), match="resource_backend"):
        benchmark.run(work, save=False, resource_backend=backend)
