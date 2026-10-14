# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

"""Tests for the benchmark resource collector adapter."""

from __future__ import annotations

import asyncio
import os
import time
from contextlib import AbstractContextManager
from types import SimpleNamespace
from typing import Any
from unittest import mock

import pytest
from isaacsim.performance.benchmarking import benchmark
from isaacsim.performance.benchmarking.impl import collector, profiles, runner

_MIB = 1024 * 1024
_DOMAIN = "isaacsim.performance.benchmarking"


def _observation(value: float, *, available: bool = True, fresh: bool = True) -> dict[str, Any]:
    """Create one native snapshot observation.

    Args:
        value: Value stored in the fake observation.
        available: Whether the observation reports an available value.
        fresh: Whether the observation belongs to the current snapshot request.

    Returns:
        Fake observation with value, freshness, and availability metadata.

    """
    return {
        "value": value,
        "unit": "B",
        "timestamp": 101.0,
        "age_seconds": 0.0,
        "available": available,
        "fresh": fresh,
    }


def _snapshot_document(metrics: dict[str, dict[str, Any]], timestamp: float = 101.0) -> dict[str, Any]:
    """Create a native fresh-snapshot document.

    Args:
        metrics: Fake observations keyed by metric name.
        timestamp: Snapshot timestamp in monotonic seconds.

    Returns:
        Fresh-snapshot document containing the supplied observations.

    """
    return {
        "schema": "ovperf_collector.snapshot.v1",
        "mode": "fresh",
        "timestamp": timestamp,
        "metrics": metrics,
        "selection": {},
    }


class _FakeRange(AbstractContextManager):
    """Record native range entry and exit.

    Args:
        events: Shared list receiving lifecycle and range events.
        domain: Domain label recorded with each range event.
        name: Domain or range label.

    """

    def __init__(self, events: list[Any], domain: str, name: str) -> None:
        self._events = events
        self._domain = domain
        self._name = name

    def __enter__(self) -> _FakeRange:
        self._events.append(("range_enter", self._domain, self._name))
        return self

    def __exit__(self, *exc: object) -> None:
        self._events.append(("range_exit", self._domain, self._name))


class _FakeDomain:
    """Provide the released native domain interface.

    Args:
        native: Fake native API supplying observations and recording calls.
        name: Domain or range label.

    """

    def __init__(self, native: _FakeNative, name: str) -> None:
        self._native = native
        self.name = name
        native.events.append(("domain", name))

    def range(self, name: str) -> _FakeRange:
        """Create a range in this domain.

        Args:
            name: Domain or range label.

        Returns:
            Range context manager recording entry and exit in this domain.

        """
        return _FakeRange(self._native.events, self.name, name)


class _FakeNative:
    """Model the released ovperf-collector 0.3 Python lifecycle.

    Args:
        snapshots: Snapshot documents to return in order, or ``None`` for an empty queue.

    """

    API_SUCCESS = 0
    API_ERROR = 1
    API_ERROR_ALREADY_RUNNING = 3
    SUPPORTED = True
    __version__ = "0.3.1"

    def __init__(self, snapshots: list[dict[str, Any]] | None = None) -> None:
        self.events: list[Any] = []
        self.snapshots = list(snapshots or [])
        self.initialized = False
        self.collecting = False
        self.init_result = self.API_SUCCESS
        self.start_result = self.API_SUCCESS
        self.stop_result = self.API_SUCCESS
        self.shutdown_result = self.API_SUCCESS
        self.run_document = {"schema": "ovperf_collector.run.v1", "series": []}
        self.last_error = "fake native error"
        self.hardware = {"gpus": [{"pci_bus_id": "00000000:01:00.0"}], "capability": {"gpu_discovery": "ready"}}

    def hardware_info(self) -> dict[str, Any]:
        """Return initialization-time GPU discovery without collecting samples.

        Returns:
            Configured GPU inventory and discovery capability.

        """
        self.events.append("hardware_info")
        return self.hardware

    def Domain(self, name: str) -> _FakeDomain:  # noqa: N802 - Match the released API.
        """Create an NVTX domain.

        Args:
            name: Domain or range label.

        Returns:
            Domain that records range events through this fake API.

        """
        return _FakeDomain(self, name)

    def get_version(self) -> tuple[int, int, int]:
        """Return the native C version.

        Returns:
            Native version tuple reported by the test double.

        """
        return (0, 3, 1)

    def get_last_error(self) -> str:
        """Return the last native error.

        Returns:
            Configured native error message.

        """
        return self.last_error

    def is_initialized(self) -> bool:
        """Return whether a native initialization owner exists.

        Returns:
            Whether the fake API currently has an initialization owner.

        """
        return self.initialized

    def is_collecting(self) -> bool:
        """Return whether a native collection owner exists.

        Returns:
            Whether the fake API currently has a collection owner.

        """
        return self.collecting

    def init(self, **kwargs: Any) -> int:
        """Acquire an initialization reference.

        Args:
            **kwargs: Initialization options to record for assertions.

        Returns:
            Configured initialization status.

        """
        self.events.append(("init", kwargs))
        self.initialized = True
        return self.init_result

    def shutdown(self) -> int:
        """Release an initialization reference.

        Returns:
            Configured shutdown status.

        """
        self.events.append("shutdown")
        self.initialized = False
        return self.shutdown_result

    def start_collection(self) -> int:
        """Acquire a collection reference.

        Returns:
            Configured collection-start status.

        """
        self.events.append("start_collection")
        self.collecting = True
        return self.start_result

    def stop_collection(self) -> int:
        """Release a collection reference.

        Returns:
            Configured collection-stop status.

        """
        self.events.append("stop_collection")
        self.collecting = False
        return self.stop_result

    def snapshot(self, *, mode: str = "latest") -> dict[str, Any]:
        """Return the next native snapshot document.

        Args:
            mode: Snapshot mode to record for assertions.

        Returns:
            Next queued snapshot document.

        """
        self.events.append(("snapshot", mode))
        if not self.snapshots:
            raise AssertionError("Unexpected snapshot")
        return self.snapshots.pop(0)

    def run(self) -> dict[str, Any]:
        """Return the retained native run document.

        Returns:
            Configured native run document.

        """
        self.events.append("run")
        return self.run_document


def _native_collector(
    monkeypatch: pytest.MonkeyPatch,
    native: _FakeNative,
    metric_profiles: tuple[str, ...] = ("process",),
    *,
    capture: bool = False,
    run_id: str = "run-7",
) -> collector._ResourceCollector:
    """Create a collector with a supported injected native API.

    Args:
        monkeypatch: Temporary attribute and environment overrides.
        native: Fake native API supplying observations and recording calls.
        metric_profiles: Resource profiles requested by the test collector.
        capture: Whether the test enables periodic capture.
        run_id: Run identifier retained in the collector artifact.

    Returns:
        Unstarted collector using the supplied fake native API.

    """
    monkeypatch.setattr(collector.platform, "system", lambda: "Linux")
    monkeypatch.setattr(collector.platform, "machine", lambda: "x86_64")
    monkeypatch.setattr(collector, "_load_ovperf", lambda: native)
    return collector._ResourceCollector(metric_profiles, run_id, backend="ovperf", capture=capture)


@pytest.mark.parametrize("backend", ["builtin", "auto"])
def test_empty_builtin_profiles_do_not_load_provider_resources(monkeypatch: pytest.MonkeyPatch, backend: str) -> None:
    """Keep empty builtin and unsupported-platform runs inert.

    Args:
        monkeypatch: Temporary attribute and environment overrides.
        backend: Resource backend value under test.

    """
    monkeypatch.setattr(collector, "_native_platform_supported", lambda: False)
    monkeypatch.setattr(collector, "_load_ovperf", mock.Mock(side_effect=AssertionError("native load")))
    monkeypatch.setattr(profiles, "process_snapshot", mock.Mock(side_effect=AssertionError("process provider")))
    monkeypatch.setattr(profiles, "_NvmlMonitor", mock.Mock(side_effect=AssertionError("GPU provider")))

    resource_collector = collector._ResourceCollector((), "empty", backend=backend)
    resource_collector.start()
    snapshot = resource_collector.snapshot()
    resource_collector.close()

    assert snapshot == collector._ResourceSnapshot(process=None, gpu=None, raw=None)


@pytest.mark.parametrize("backend", ["auto", "ovperf"])
def test_timing_only_execution_traces_without_collection(monkeypatch: pytest.MonkeyPatch, backend: str) -> None:
    """Trace default metrics without resource queries or a periodic sampler.

    Args:
        monkeypatch: Temporary attribute and environment overrides.
        backend: Resource backend value under test.

    """
    native = _FakeNative()
    monkeypatch.setattr(collector, "_native_platform_supported", lambda: True)
    monkeypatch.setattr(collector, "_load_ovperf", lambda: native)

    @benchmark.subsystem("tests.collector.timing-only", resources=("gpu",), warmup_samples=1, measured_samples=2)
    def work() -> None:
        native.events.append("work")

    document = benchmark.run(work, resource_backend=backend, save=False)

    assert document["status"]["valid"], document["status"]
    assert [event for event in native.events if isinstance(event, tuple) and event[0] == "range_enter"] == [
        ("range_enter", _DOMAIN, "MEASURE"),
        ("range_enter", _DOMAIN, "sample/0"),
        ("range_enter", _DOMAIN, "sample/1"),
    ]
    assert native.events[0] == ("init", {"component": _DOMAIN, "gpu": False, "gpm": False, "metrics": {"*": False}})
    assert native.events.count("work") == 3
    assert not any(isinstance(event, tuple) and event[0] == "snapshot" for event in native.events)
    assert "start_collection" not in native.events
    assert native.events[-1] == "shutdown"
    assert not native.initialized and not native.collecting


@pytest.mark.parametrize(("backend", "capture"), [("builtin", False), ("ovperf", False), ("ovperf", True)])
def test_cpu_metrics_exclude_resource_io(monkeypatch: pytest.MonkeyPatch, backend: str, capture: bool) -> None:
    """Exclude 5 ms resource reads from a 1 ms workload and its captured CPU boundaries.

    Args:
        monkeypatch: Temporary attribute and environment overrides.
        backend: Resource backend value under test.
        capture: Whether the test enables periodic capture.

    """
    wall_ns = 0
    cpu_seconds = 0.0

    def read_memory() -> float:
        nonlocal wall_ns, cpu_seconds
        wall_ns += 5_000_000
        cpu_seconds += 0.005
        return 8.0

    native = _FakeNative()

    def snapshot(*, mode: str) -> dict[str, Any]:
        assert mode == "fresh"
        return _snapshot_document({"process.memory.rss_bytes": _observation(read_memory() * _MIB)})

    monkeypatch.setattr(time, "process_time", lambda: cpu_seconds)
    monkeypatch.setattr(profiles, "_resident_memory_mib", read_memory)
    monkeypatch.setattr(collector, "_native_platform_supported", lambda: True)
    monkeypatch.setattr(collector, "_load_ovperf", lambda: native)
    monkeypatch.setattr(native, "snapshot", snapshot)

    @benchmark.subsystem("tests.collector.cpu-boundary", metrics=("process",), warmup_samples=0, measured_samples=2)
    def work() -> None:
        nonlocal wall_ns, cpu_seconds
        wall_ns += 1_000_000
        cpu_seconds += 0.001

    result = asyncio.run(
        runner.run_definition(
            work.__benchmark_definition__,
            clock_ns=lambda: wall_ns,
            resource_backend=backend,
            capture_collector=capture,
        )
    )
    assert result.valid, result.errors
    for sample in result.samples:
        assert sample.duration_ms == 1.0
        assert sample.metrics["process_cpu_time_s"] == pytest.approx(0.001)
        assert sample.metrics["process_cpu_utilization"] == pytest.approx(1.0)
    if capture:
        samples = result.collector_capture["samples"]
        assert [sample["before"]["process_cpu_seconds"] for sample in samples] == pytest.approx([0.005, 0.016])
        assert [sample["after"]["process_cpu_seconds"] for sample in samples] == pytest.approx([0.006, 0.017])


def test_builtin_backend_uses_existing_process_and_gpu_providers(monkeypatch: pytest.MonkeyPatch) -> None:
    """Select builtin providers when requested explicitly.

    Args:
        monkeypatch: Temporary attribute and environment overrides.

    """
    process = profiles._ProcessSnapshot(rss_mib=64.0)
    gpu = profiles._GpuSnapshot((profiles._GpuDeviceSnapshot(0.5, 100.0, 10.0, True),))
    monitor = SimpleNamespace(snapshot=mock.Mock(return_value=gpu), close=mock.Mock())
    monkeypatch.setattr(profiles, "process_snapshot", mock.Mock(return_value=process))
    monkeypatch.setattr(profiles, "_NvmlMonitor", mock.Mock(return_value=monitor))
    monkeypatch.setattr(collector, "_load_ovperf", mock.Mock(side_effect=AssertionError("native load")))

    resource_collector = collector._ResourceCollector(("process", "gpu"), "builtin", backend="builtin")
    resource_collector.start()
    snapshot = resource_collector.snapshot()
    resource_collector.close()
    resource_collector.close()

    assert snapshot == collector._ResourceSnapshot(process=process, gpu=gpu, raw=None)
    monitor.close.assert_called_once_with()
    assert resource_collector.provenance["backend"] == "builtin"


def test_capture_requires_native_backend() -> None:
    """Reject capture when the caller explicitly selects builtin providers."""
    with pytest.raises(ValueError, match="capture.*ovperf"):
        collector._ResourceCollector(("process",), "builtin-capture", backend="builtin", capture=True)


def test_auto_selects_builtin_on_an_unsupported_platform(monkeypatch: pytest.MonkeyPatch) -> None:
    """Select builtin providers on an unsupported native platform.

    Args:
        monkeypatch: Temporary attribute and environment overrides.

    """
    process = profiles._ProcessSnapshot(rss_mib=2.0)
    monkeypatch.setattr(collector.platform, "system", lambda: "Windows")
    monkeypatch.setattr(collector.platform, "machine", lambda: "AMD64")
    monkeypatch.setattr(profiles, "process_snapshot", lambda: process)
    monkeypatch.setattr(collector, "_load_ovperf", mock.Mock(side_effect=AssertionError("native load")))

    resource_collector = collector._ResourceCollector(("process",), "auto", backend="auto")
    resource_collector.start()

    assert resource_collector.snapshot().process == process
    assert resource_collector.provenance["backend"] == "builtin"


def test_explicit_native_backend_rejects_unsupported_platform(monkeypatch: pytest.MonkeyPatch) -> None:
    """Fail clearly when native collection is requested on another platform.

    Args:
        monkeypatch: Temporary attribute and environment overrides.

    """
    monkeypatch.setattr(collector.platform, "system", lambda: "Darwin")
    monkeypatch.setattr(collector.platform, "machine", lambda: "arm64")

    resource_collector = collector._ResourceCollector(("process",), "unsupported", backend="ovperf")

    with pytest.raises(RuntimeError, match="Linux x86_64"):
        resource_collector.start()


def test_auto_uses_builtin_when_ovperf_package_is_absent_on_supported_platform(
    monkeypatch: pytest.MonkeyPatch, caplog: pytest.LogCaptureFixture, capsys: pytest.CaptureFixture[str]
) -> None:
    """An absent optional top-level package quietly selects builtin observations."""
    monkeypatch.setattr(collector.platform, "system", lambda: "Linux")
    monkeypatch.setattr(collector.platform, "machine", lambda: "x86_64")
    monkeypatch.setattr(
        collector,
        "_load_ovperf",
        mock.Mock(side_effect=ModuleNotFoundError("No module named 'ovperf_collector'", name="ovperf_collector")),
    )
    process = profiles._ProcessSnapshot(rss_mib=2.0)
    monkeypatch.setattr(profiles, "process_snapshot", lambda: process)

    resource_collector = collector._ResourceCollector(("process",), "missing", backend="auto")
    resource_collector.start()

    assert resource_collector.snapshot().process == process
    assert resource_collector.provenance["backend"] == "builtin"
    assert not [record for record in caplog.records if record.levelno >= 30]
    assert capsys.readouterr() == ("", "")


@pytest.mark.parametrize(
    "error",
    [
        ImportError("installed package import failed"),
        ModuleNotFoundError("No module named 'native_dependency'", name="native_dependency"),
    ],
)
def test_auto_fails_when_installed_native_import_fails_on_supported_platform(
    monkeypatch: pytest.MonkeyPatch, error: ImportError
) -> None:
    """Installed package defects remain fatal in auto mode."""
    monkeypatch.setattr(collector.platform, "system", lambda: "Linux")
    monkeypatch.setattr(collector.platform, "machine", lambda: "x86_64")
    monkeypatch.setattr(collector, "_load_ovperf", mock.Mock(side_effect=error))

    resource_collector = collector._ResourceCollector(("process",), "broken", backend="auto")

    with pytest.raises(RuntimeError, match="ovperf-collector.*0.3.1"):
        resource_collector.start()


@pytest.mark.parametrize("backend,capture", [("ovperf", False), ("auto", True)])
def test_explicit_or_capture_requires_installed_native_package(
    monkeypatch: pytest.MonkeyPatch, backend: str, capture: bool
) -> None:
    """Explicit native intent remains strict when the package is absent."""
    monkeypatch.setattr(collector.platform, "system", lambda: "Linux")
    monkeypatch.setattr(collector.platform, "machine", lambda: "x86_64")
    monkeypatch.setattr(
        collector,
        "_load_ovperf",
        mock.Mock(side_effect=ModuleNotFoundError("No module named 'ovperf_collector'", name="ovperf_collector")),
    )

    resource_collector = collector._ResourceCollector(("process",), "missing", backend=backend, capture=capture)

    with pytest.raises(RuntimeError, match="ovperf-collector.*0.3.1"):
        resource_collector.start()


@pytest.mark.parametrize(
    ("wrapper_version", "native_version", "expected"),
    [("0.2.5", (0, 3, 1), "Python wrapper.*0.3.1"), ("0.3.1", (0, 2, 5), "native library.*0.3.1")],
)
def test_native_backend_requires_matching_released_versions(
    monkeypatch: pytest.MonkeyPatch,
    wrapper_version: str,
    native_version: tuple[int, int, int],
    expected: str,
) -> None:
    """Reject stale Python wrappers and native shared libraries.

    Args:
        monkeypatch: Temporary attribute and environment overrides.
        wrapper_version: Python wrapper version reported by the fake API.
        native_version: Native library version reported by the fake API.
        expected: Pattern expected in the version mismatch error.

    """
    native = _FakeNative()
    native.__version__ = wrapper_version
    native.get_version = mock.Mock(return_value=native_version)
    resource_collector = _native_collector(monkeypatch, native)

    with pytest.raises(RuntimeError, match=expected):
        resource_collector.start()


def test_native_start_initializes_only_the_requested_metrics(monkeypatch: pytest.MonkeyPatch) -> None:
    """Prepare exact snapshot sources without starting periodic collection.

    Args:
        monkeypatch: Temporary attribute and environment overrides.

    """
    native = _FakeNative()
    resource_collector = _native_collector(monkeypatch, native, ("process", "gpu"))

    resource_collector.start()

    assert native.events == [
        (
            "init",
            {
                "component": _DOMAIN,
                "gpu": True,
                "gpm": False,
                "metrics": {
                    "*": False,
                    "process.memory.rss_bytes": True,
                    "gpu.nvml_utilization_ratio": True,
                    "gpu.memory.used_bytes": True,
                    "gpu.memory.reserved_bytes": True,
                    "gpu.process.memory_bytes": True,
                },
            },
        ),
        "hardware_info",
        ("domain", _DOMAIN),
    ]
    assert not native.collecting
    resource_collector.close()


def test_native_snapshot_does_not_read_cpu_clock(monkeypatch: pytest.MonkeyPatch) -> None:
    """Leave workload CPU boundaries to the runner.

    Args:
        monkeypatch: Temporary attribute and environment overrides.

    """
    native = _FakeNative([_snapshot_document({"process.memory.rss_bytes": _observation(8 * _MIB)})])
    resource_collector = _native_collector(monkeypatch, native)

    def read_cpu_time() -> float:
        native.events.append("process_time")
        return 4.25

    clock = mock.Mock(side_effect=read_cpu_time)
    monkeypatch.setattr(time, "process_time", clock)
    resource_collector.start()
    native.events.clear()

    snapshot = resource_collector.snapshot()

    clock.assert_not_called()
    assert native.events == [("snapshot", "fresh")]
    assert snapshot.process == profiles._ProcessSnapshot(rss_mib=8.0)
    assert snapshot.raw is not None


@pytest.mark.parametrize("rss_available", [False, None])
def test_native_snapshot_preserves_unknown_rss(monkeypatch: pytest.MonkeyPatch, rss_available: bool | None) -> None:
    """Keep missing and unavailable RSS observations unknown.

    Args:
        monkeypatch: Temporary attribute and environment overrides.
        rss_available: Availability flag, or ``None`` to omit the RSS observation.

    """
    metrics = {}
    if rss_available is not None:
        metrics["process.memory.rss_bytes"] = _observation(0.0, available=rss_available)
    native = _FakeNative([_snapshot_document(metrics)])
    resource_collector = _native_collector(monkeypatch, native)
    resource_collector.start()

    assert resource_collector.snapshot().process.rss_mib is None


def test_native_process_snapshots_allow_one_missing_rss_endpoint(monkeypatch: pytest.MonkeyPatch) -> None:
    """Keep a sample usable when only one RSS boundary is available.

    Args:
        monkeypatch: Temporary attribute and environment overrides.

    """
    native = _FakeNative(
        [
            _snapshot_document({"process.memory.rss_bytes": _observation(0.0, available=False)}),
            _snapshot_document({"process.memory.rss_bytes": _observation(12 * _MIB)}),
        ]
    )
    resource_collector = _native_collector(monkeypatch, native)
    resource_collector.start()

    before = resource_collector.snapshot()
    after = resource_collector.snapshot()

    assert profiles.process_metrics(before.process, after.process, 1.0, cpu_seconds=0.5)["process_rss_mib"] == 12.0


def test_native_snapshot_maps_exact_gpu_observations(monkeypatch: pytest.MonkeyPatch) -> None:
    """Map NVML ratios, v1 device bytes, and current-process bytes without changing units.

    Args:
        monkeypatch: Temporary attribute and environment overrides.

    """
    bus = "00000000:01:00.0"
    current_pid = os.getpid()
    native = _FakeNative(
        [
            _snapshot_document(
                {
                    f"gpu.{bus}.nvml_utilization_ratio": _observation(0.375),
                    f"gpu.{bus}.memory.used_bytes": _observation(900 * _MIB),
                    f"gpu.{bus}.memory.reserved_bytes": _observation(100 * _MIB),
                    f"gpu.{bus}.process.pid{current_pid}.memory_bytes": _observation(40 * _MIB),
                    f"gpu.{bus}.process.pid{current_pid}.active_count": _observation(1),
                }
            )
        ]
    )
    resource_collector = _native_collector(monkeypatch, native, ("gpu",))
    resource_collector.start()

    snapshot = resource_collector.snapshot()

    assert snapshot.gpu == profiles._GpuSnapshot((profiles._GpuDeviceSnapshot(0.375, 1000.0, 40.0, True),))


@pytest.mark.parametrize("missing_suffix", ["nvml_utilization_ratio", "memory.used_bytes", "memory.reserved_bytes"])
def test_native_snapshot_rejects_missing_required_gpu_observations(
    monkeypatch: pytest.MonkeyPatch, missing_suffix: str
) -> None:
    """Reject a device boundary whose required NVML observation is not current.

    Args:
        monkeypatch: Temporary attribute and environment overrides.
        missing_suffix: GPU metric suffix whose observation is made stale.

    """
    bus = "00000000:01:00.0"
    metrics = {
        f"gpu.{bus}.nvml_utilization_ratio": _observation(0.25),
        f"gpu.{bus}.memory.used_bytes": _observation(900 * _MIB),
        f"gpu.{bus}.memory.reserved_bytes": _observation(100 * _MIB),
    }
    metrics[f"gpu.{bus}.{missing_suffix}"]["fresh"] = False
    native = _FakeNative([_snapshot_document(metrics)])
    resource_collector = _native_collector(monkeypatch, native, ("gpu",))
    resource_collector.start()

    with pytest.raises(RuntimeError, match=f"{missing_suffix}.*fresh"):
        resource_collector.snapshot()


def test_native_snapshot_never_infers_process_gpu_state_from_stale_pid_rows(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    """Represent absent current PID data as unknown instead of a fabricated zero.

    Args:
        monkeypatch: Temporary attribute and environment overrides.

    """
    bus = "00000000:01:00.0"
    current_pid = os.getpid()
    native = _FakeNative(
        [
            _snapshot_document(
                {
                    f"gpu.{bus}.nvml_utilization_ratio": _observation(0.25),
                    f"gpu.{bus}.memory.used_bytes": _observation(900 * _MIB),
                    f"gpu.{bus}.memory.reserved_bytes": _observation(100 * _MIB),
                    f"gpu.{bus}.process.pid{current_pid}.memory_bytes": _observation(20 * _MIB, fresh=False),
                    f"gpu.{bus}.process.pid99999.memory_bytes": _observation(50 * _MIB),
                }
            )
        ]
    )
    resource_collector = _native_collector(monkeypatch, native, ("gpu",))
    resource_collector.start()

    device = resource_collector.snapshot().gpu.devices[0]

    assert device.process_vram_used_mib is None
    assert device.process_active is None


def test_native_snapshot_keeps_absent_current_pid_data_unknown(monkeypatch: pytest.MonkeyPatch) -> None:
    """Do not turn an absent initial process roster into zero process memory.

    Args:
        monkeypatch: Temporary attribute and environment overrides.

    """
    bus = "00000000:01:00.0"
    native = _FakeNative(
        [
            _snapshot_document(
                {
                    f"gpu.{bus}.nvml_utilization_ratio": _observation(0.25),
                    f"gpu.{bus}.memory.used_bytes": _observation(900 * _MIB),
                    f"gpu.{bus}.memory.reserved_bytes": _observation(100 * _MIB),
                }
            )
        ]
    )
    resource_collector = _native_collector(monkeypatch, native, ("gpu",))
    resource_collector.start()

    device = resource_collector.snapshot().gpu.devices[0]

    assert device.process_vram_used_mib is None
    assert device.process_active is None


@pytest.mark.parametrize(
    ("activity", "memory", "expected_activity"),
    [(0, 0, False), (1, 0, True), (1, None, True), (None, None, None)],
)
def test_native_gpu_activity_is_independent_of_memory(
    monkeypatch: pytest.MonkeyPatch, activity: int | None, memory: int | None, expected_activity: bool | None
) -> None:
    """Separate known inactivity, zero VRAM, and unavailable PID observations.

    Args:
        monkeypatch: Temporary attribute and environment overrides.
        activity: Process activity count, or ``None`` for an unavailable observation.
        memory: Process memory in bytes, or ``None`` for an unavailable observation.
        expected_activity: Expected activity flag, or ``None`` when unknown.

    """
    prefix = "gpu.00000000:01:00.0."
    metrics = {
        prefix + "nvml_utilization_ratio": _observation(0.5),
        prefix + "memory.used_bytes": _observation(900 * _MIB),
        prefix + "memory.reserved_bytes": _observation(100 * _MIB),
        prefix + f"process.pid{os.getpid()}.active_count": _observation(activity or 0, available=activity is not None),
        prefix + f"process.pid{os.getpid()}.memory_bytes": _observation(memory or 0, available=memory is not None),
    }
    native = _FakeNative([_snapshot_document(metrics)])
    resource_collector = _native_collector(monkeypatch, native, ("gpu",))
    try:
        resource_collector.start()
        device = resource_collector.snapshot().gpu.devices[0]
        assert device.process_active is expected_activity
        assert device.process_vram_used_mib == memory
    finally:
        resource_collector.close()


@pytest.mark.parametrize("discovery", ["empty", "failed", "partial"])
def test_native_gpu_discovery_failure_precedes_warmup(monkeypatch: pytest.MonkeyPatch, discovery: str) -> None:
    """Fail requested GPU metrics before warmup and release native initialization.

    Args:
        monkeypatch: Temporary attribute and environment overrides.
        discovery: GPU discovery state reported by the fake API.

    """
    native = _FakeNative()
    native.hardware = {
        "gpus": [{"pci_bus_id": "gpu0"}] if discovery == "partial" else [],
        "capability": {"gpu_discovery": discovery, "gpu_discovery_error": "inventory unavailable"},
    }
    monkeypatch.setattr(collector, "_native_platform_supported", lambda: True)
    monkeypatch.setattr(collector, "_load_ovperf", lambda: native)

    @benchmark.subsystem("tests.collector.discovery", metrics=("gpu",), warmup_samples=1, measured_samples=1)
    def work() -> None:
        native.events.append("work")

    document = benchmark.run(work, resource_backend="ovperf", save=False)

    assert not document["status"]["valid"]
    assert any(f"GPU discovery {discovery}" in error for error in document["status"]["errors"])
    assert "work" not in native.events
    assert "start_collection" not in native.events
    assert native.events[-1] == "shutdown"
    assert not native.initialized


def test_native_ranges_expose_one_measurement_and_each_sample(monkeypatch: pytest.MonkeyPatch) -> None:
    """Place MEASURE and sample ranges in the benchmark domain without a duplicate session.

    Args:
        monkeypatch: Temporary attribute and environment overrides.

    """
    native = _FakeNative()
    resource_collector = _native_collector(monkeypatch, native)
    resource_collector.start()
    native.events.clear()

    resource_collector.begin_measurement()
    with resource_collector.range("sample/2"):
        native.events.append("work")
    resource_collector.end_measurement()

    assert native.events == [
        ("range_enter", _DOMAIN, "MEASURE"),
        ("range_enter", _DOMAIN, "sample/2"),
        "work",
        ("range_exit", _DOMAIN, "sample/2"),
        ("range_exit", _DOMAIN, "MEASURE"),
    ]


def test_start_rejects_existing_initialization_without_releasing_its_owner(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    """Do not join or shut down an initialization owned by another component.

    Args:
        monkeypatch: Temporary attribute and environment overrides.

    """
    native = _FakeNative()
    native.initialized = True
    resource_collector = _native_collector(monkeypatch, native)

    with pytest.raises(RuntimeError, match="already initialized"):
        resource_collector.start()

    assert native.events == []


def test_start_balances_reference_acquired_during_initialization_race(monkeypatch: pytest.MonkeyPatch) -> None:
    """Release only the joined reference when initialization loses a race.

    Args:
        monkeypatch: Temporary attribute and environment overrides.

    """
    native = _FakeNative()
    native.init_result = native.API_ERROR_ALREADY_RUNNING
    resource_collector = _native_collector(monkeypatch, native)

    with pytest.raises(RuntimeError, match="initialization owner"):
        resource_collector.start()

    assert [event if isinstance(event, str) else event[0] for event in native.events] == ["init", "shutdown"]


def test_capture_balances_reference_acquired_during_collection_race(monkeypatch: pytest.MonkeyPatch) -> None:
    """Release a joined collection reference and retain initialization ownership for close.

    Args:
        monkeypatch: Temporary attribute and environment overrides.

    """
    native = _FakeNative()
    native.start_result = native.API_ERROR_ALREADY_RUNNING
    resource_collector = _native_collector(monkeypatch, native, capture=True)
    resource_collector.start()

    with pytest.raises(RuntimeError, match="collection owner"):
        resource_collector.begin_measurement()
    resource_collector.close()

    event_names = [event if isinstance(event, str) else event[0] for event in native.events]
    assert event_names == ["init", "domain", "start_collection", "stop_collection", "shutdown"]


def test_capture_rejects_existing_collection_without_stopping_its_owner(monkeypatch: pytest.MonkeyPatch) -> None:
    """Do not acquire or stop collection when another owner is already active.

    Args:
        monkeypatch: Temporary attribute and environment overrides.

    """
    native = _FakeNative()
    resource_collector = _native_collector(monkeypatch, native, capture=True)
    resource_collector.start()
    native.collecting = True

    with pytest.raises(RuntimeError, match="already collecting"):
        resource_collector.begin_measurement()
    resource_collector.close()

    event_names = [event if isinstance(event, str) else event[0] for event in native.events]
    assert "start_collection" not in event_names
    assert "stop_collection" not in event_names
    assert event_names[-1] == "shutdown"


def test_capture_rejects_owner_joining_after_measurement_begins(monkeypatch: pytest.MonkeyPatch) -> None:
    """Do not export or stop a shared run when another component joins mid-measurement.

    Args:
        monkeypatch: Temporary attribute and environment overrides.

    """

    class SharedNative(_FakeNative):
        def __init__(self) -> None:
            super().__init__()
            self.initialization_references = 0
            self.collection_references = 0

        def init(self, **kwargs: Any) -> int:
            result = super().init(**kwargs)
            self.initialization_references += 1
            return result if self.initialization_references == 1 else self.API_ERROR_ALREADY_RUNNING

        def start_collection(self) -> int:
            result = super().start_collection()
            self.collection_references += 1
            return result if self.collection_references == 1 else self.API_ERROR_ALREADY_RUNNING

        def stop_collection(self) -> int:
            self.events.append("stop_collection")
            self.collection_references -= 1
            self.collecting = self.collection_references > 0
            return self.API_SUCCESS

        def shutdown(self) -> int:
            self.events.append("shutdown")
            self.initialization_references -= 1
            self.initialized = self.initialization_references > 0
            if not self.initialized:
                self.collection_references = 0
                self.collecting = False
            return self.API_SUCCESS

    native = SharedNative()
    resource_collector = _native_collector(monkeypatch, native, capture=True)
    resource_collector.start()
    resource_collector.begin_measurement()
    assert native.init() == native.API_ERROR_ALREADY_RUNNING
    assert native.start_collection() == native.API_ERROR_ALREADY_RUNNING

    with pytest.raises(RuntimeError, match="another owner"):
        resource_collector.end_measurement()
    resource_collector.close()

    assert resource_collector.capture_document is None
    assert "run" not in native.events
    assert native.initialization_references == 1
    assert native.collection_references == 1


def test_capture_stops_exports_and_shuts_down_in_order(monkeypatch: pytest.MonkeyPatch) -> None:
    """Capture explicit boundaries and export retained native data before shutdown.

    Args:
        monkeypatch: Temporary attribute and environment overrides.

    """
    before_raw = _snapshot_document({"process.memory.rss_bytes": _observation(10 * _MIB)}, timestamp=100.0)
    after_raw = _snapshot_document({"process.memory.rss_bytes": _observation(12 * _MIB)}, timestamp=102.0)
    native = _FakeNative([before_raw, after_raw])
    resource_collector = _native_collector(monkeypatch, native, capture=True)
    resource_collector.start()

    resource_collector.begin_measurement()
    before = resource_collector.snapshot()
    after = resource_collector.snapshot()
    resource_collector.record_sample(3, 4, 0.25, before, after, cpu_before=5.0, cpu_after=5.5)
    resource_collector.end_measurement()
    resource_collector.close()
    resource_collector.close()

    event_names = [event if isinstance(event, str) else event[0] for event in native.events]
    assert event_names[-4:] == ["stop_collection", "range_exit", "run", "shutdown"]
    assert resource_collector.capture_document == {
        "schema": "isaacsim.performance.benchmarking.collector.v1",
        "run_id": "run-7",
        "clocks": {
            "snapshot_timestamp": "absolute CLOCK_MONOTONIC seconds",
            "native_run_t": "seconds relative to ovperf-collector initialization",
            "process_cpu_seconds": "cumulative process CPU seconds from time.process_time()",
        },
        "samples": [
            {
                "index": 3,
                "batch_size": 4,
                "elapsed_seconds": 0.25,
                "before": {"snapshot": before_raw, "process_cpu_seconds": 5.0},
                "after": {"snapshot": after_raw, "process_cpu_seconds": 5.5},
            }
        ],
        "run": native.run_document,
    }


def test_close_attempts_collection_release_and_shutdown_after_capture_error(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    """Attempt every owned release when capture export fails.

    Args:
        monkeypatch: Temporary attribute and environment overrides.

    """
    native = _FakeNative()
    native.run = mock.Mock(side_effect=RuntimeError("export failed"))
    resource_collector = _native_collector(monkeypatch, native, capture=True)
    resource_collector.start()
    resource_collector.begin_measurement()

    with pytest.raises(RuntimeError, match="export failed"):
        resource_collector.end_measurement()
    resource_collector.close()

    event_names = [event if isinstance(event, str) else event[0] for event in native.events]
    assert "stop_collection" in event_names
    assert event_names[-1] == "shutdown"


@pytest.mark.parametrize("operation", ["stop_collection", "shutdown"])
def test_native_release_failure_can_be_retried_without_repeating_capture(
    monkeypatch: pytest.MonkeyPatch, operation: str
) -> None:
    """Retain failed release ownership while completing each other cleanup step once.

    Args:
        monkeypatch: Temporary attribute and environment overrides.
        operation: Native release operation configured to fail once.

    """
    native = _FakeNative()
    resource_collector = _native_collector(monkeypatch, native, capture=True)
    resource_collector.start()
    resource_collector.begin_measurement()

    if operation == "stop_collection":
        native.stop_result = native.API_ERROR
        with pytest.raises(RuntimeError, match="stop_collection"):
            resource_collector.end_measurement()
        native.stop_result = native.API_SUCCESS
        resource_collector.end_measurement()
    else:
        native.shutdown_result = native.API_ERROR
        with pytest.raises(RuntimeError, match="shutdown"):
            resource_collector.close()
        native.shutdown_result = native.API_SUCCESS
    resource_collector.close()
    resource_collector.close()

    event_names = [event if isinstance(event, str) else event[0] for event in native.events]
    assert event_names.count("stop_collection") == (2 if operation == "stop_collection" else 1)
    assert event_names.count("shutdown") == (2 if operation == "shutdown" else 1)
    assert event_names.count("range_exit") == 1
    assert event_names.count("run") == 1
    assert not native.initialized
    assert not native.collecting


def test_serial_native_collectors_return_all_ownership_to_zero(monkeypatch: pytest.MonkeyPatch) -> None:
    """Allow serial benchmark runs after each collector balances its references.

    Args:
        monkeypatch: Temporary attribute and environment overrides.

    """
    native = _FakeNative()

    for run_id in ("first", "second"):
        resource_collector = _native_collector(monkeypatch, native, capture=True, run_id=run_id)
        resource_collector.start()
        resource_collector.begin_measurement()
        resource_collector.end_measurement()
        resource_collector.close()

        assert not native.initialized
        assert not native.collecting

    assert native.events.count("shutdown") == 2
    assert native.events.count("stop_collection") == 2
