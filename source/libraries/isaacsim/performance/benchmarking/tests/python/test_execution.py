# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

"""Tests for public in-process benchmark execution and reporting."""

from __future__ import annotations

import asyncio
import json
from pathlib import Path

import pytest
from isaacsim.performance.benchmarking import benchmark
from isaacsim.performance.benchmarking.impl import reporting
from isaacsim.performance.benchmarking.impl.schema import RESULT_SCHEMA


def test_run_executes_decorated_target_and_writes_result(tmp_path: Path, capsys: pytest.CaptureFixture[str]) -> None:
    """Run a benchmark directly from its decorated target.

    Args:
        tmp_path: Temporary output directory.
        capsys: Captured standard streams.

    """

    @benchmark.subsystem("tests.execution.output", warmup_samples=0, measured_samples=1)
    def work(value: int = 2) -> None:
        benchmark.record("value", value, "count")

    output = tmp_path / "output" / "result.json"
    document = benchmark.run(work, {"value": 4}, seed=7, output=output)

    stdout = capsys.readouterr().out
    assert stdout.startswith("tests.execution.output: valid")
    assert f"Result: {output}" in stdout
    assert document["schema"] == RESULT_SCHEMA
    assert document["benchmark"]["parameters"] == {"value": 4}
    assert document["benchmark"]["seed"] == 7
    assert document["metrics"]["value"]["samples"][0]["value"] == 4.0
    assert json.loads(output.read_text(encoding="utf-8")) == document
    assert [path.name for path in output.parent.iterdir()] == ["result.json"]


def test_run_uses_configured_output_directory_and_unique_names(tmp_path: Path, monkeypatch: pytest.MonkeyPatch) -> None:
    """Place automatic results beneath the configured environment root.

    Args:
        tmp_path: Temporary output directory.
        monkeypatch: Temporary attribute and environment modifier.

    """

    @benchmark.subsystem("tests.execution.configured", warmup_samples=0, measured_samples=1)
    def work() -> None:
        pass

    root = tmp_path / "results"
    monkeypatch.setenv("ISAACSIM_BENCHMARK_OUTPUT_DIR", str(root))

    benchmark.run(work)
    benchmark.run(work)

    files = sorted((root / "tests.execution.configured").glob("*.json"))
    assert len(files) == 2
    assert files[0].name != files[1].name


def test_run_falls_back_to_system_temporary_directory(tmp_path: Path, monkeypatch: pytest.MonkeyPatch) -> None:
    """Keep default output away from the application's working directory.

    Args:
        tmp_path: Temporary output directory.
        monkeypatch: Temporary attribute and environment modifier.

    """

    @benchmark.subsystem("tests.execution.temporary", warmup_samples=0, measured_samples=1)
    def work() -> None:
        pass

    monkeypatch.delenv("ISAACSIM_BENCHMARK_OUTPUT_DIR", raising=False)
    monkeypatch.setattr(reporting.tempfile, "gettempdir", lambda: str(tmp_path))

    benchmark.run(work)

    assert len(list((tmp_path / "isaacsim-benchmarks" / "tests.execution.temporary").glob("*.json"))) == 1


def test_run_can_skip_result_persistence(tmp_path: Path, monkeypatch: pytest.MonkeyPatch) -> None:
    """Return a result without creating the configured output directory.

    Args:
        tmp_path: Temporary output directory.
        monkeypatch: Temporary attribute and environment modifier.

    """

    @benchmark.subsystem("tests.execution.no-save", warmup_samples=0, measured_samples=1)
    def work() -> None:
        pass

    monkeypatch.setenv("ISAACSIM_BENCHMARK_OUTPUT_DIR", str(tmp_path / "results"))

    document = benchmark.run(work, save=False)

    assert document["status"]["valid"]
    assert not (tmp_path / "results").exists()


def test_run_rejects_output_when_saving_is_disabled(tmp_path: Path) -> None:
    """Reject contradictory output arguments.

    Args:
        tmp_path: Temporary output directory.

    """

    @benchmark.subsystem("tests.execution.invalid-output", warmup_samples=0, measured_samples=1)
    def work() -> None:
        pass

    with pytest.raises(ValueError, match="cannot combine output"):
        benchmark.run(work, output=tmp_path / "result.json", save=False)

    with pytest.raises(TypeError, match="save must be a bool"):
        benchmark.run(work, save=0)


def test_run_requires_a_decorated_target() -> None:
    """Reject ordinary callables before starting an event loop."""

    def work() -> None:
        pass

    with pytest.raises(TypeError, match="benchmark-decorated"):
        benchmark.run(work)


def test_run_async_supports_applications_with_a_running_event_loop() -> None:
    """Expose async execution and direct running-loop callers toward it."""

    @benchmark.subsystem("tests.execution.async", warmup_samples=0, measured_samples=1)
    async def work() -> None:
        await asyncio.sleep(0)

    async def application() -> dict[str, object]:
        with pytest.raises(RuntimeError, match=r"await benchmark\.run_async\(\)"):
            benchmark.run(work, save=False)
        return await benchmark.run_async(work, save=False)

    document = asyncio.run(application())

    assert document["status"]["valid"]
