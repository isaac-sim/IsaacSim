# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

"""Tests for Kit-independent process and GPU metric providers."""

from __future__ import annotations

from types import SimpleNamespace
from unittest import mock

import pytest
from isaacsim.performance.benchmarking.impl import profiles

_MIB = 1024 * 1024


class _FakeNvmlError(Exception):
    """NVML error used by the test double."""


def _fake_nvml() -> SimpleNamespace:
    """Create an NVML module double with two devices.

    Returns:
        NVML-compatible test double.

    """
    current_pid = 42
    initialization = mock.Mock()
    shutdown = mock.Mock()

    def utilization(handle: int) -> SimpleNamespace:
        return SimpleNamespace(gpu=25 + handle * 50)

    def memory(handle: int) -> SimpleNamespace:
        return SimpleNamespace(used=(1000 + handle * 500) * _MIB)

    def compute_processes(handle: int) -> list[SimpleNamespace]:
        if handle == 0:
            return [SimpleNamespace(pid=current_pid, usedGpuMemory=120 * _MIB)]
        return [SimpleNamespace(pid=100, usedGpuMemory=80 * _MIB)]

    def graphics_processes(handle: int) -> list[SimpleNamespace]:
        if handle == 0:
            return [SimpleNamespace(pid=current_pid, usedGpuMemory=120 * _MIB)]
        return []

    return SimpleNamespace(
        NVMLError=_FakeNvmlError,
        NVML_VALUE_NOT_AVAILABLE=(1 << 64) - 1,
        nvmlInit=initialization,
        nvmlShutdown=shutdown,
        nvmlDeviceGetCount=lambda: 2,
        nvmlDeviceGetHandleByIndex=lambda index: index,
        nvmlDeviceGetUtilizationRates=utilization,
        nvmlDeviceGetMemoryInfo=memory,
        nvmlDeviceGetComputeRunningProcesses=compute_processes,
        nvmlDeviceGetGraphicsRunningProcesses=graphics_processes,
    )


def test_nvml_monitor_collects_device_and_process_counters(monkeypatch: pytest.MonkeyPatch) -> None:
    """Collect device counters and avoid double-counting graphics and compute memory.

    Args:
        monkeypatch: Temporary process identifier modifier.

    """
    nvml = _fake_nvml()
    monkeypatch.setattr(profiles.os, "getpid", lambda: 42)

    monitor = profiles._NvmlMonitor(nvml)
    snapshot = monitor.snapshot()
    monitor.close()

    assert snapshot.devices == (
        profiles._GpuDeviceSnapshot(
            utilization=0.25,
            vram_used_mib=1000.0,
            process_vram_used_mib=120.0,
            process_active=True,
        ),
        profiles._GpuDeviceSnapshot(
            utilization=0.75,
            vram_used_mib=1500.0,
            process_vram_used_mib=0.0,
            process_active=False,
        ),
    )
    nvml.nvmlInit.assert_called_once_with()
    nvml.nvmlShutdown.assert_called_once_with()


def test_gpu_metrics_use_devices_running_the_current_process() -> None:
    """Exclude unrelated visible devices when the process device is known."""
    before = profiles._GpuSnapshot(
        devices=(
            profiles._GpuDeviceSnapshot(0.2, 1000.0, 100.0, True),
            profiles._GpuDeviceSnapshot(0.9, 8000.0, 0.0, False),
        )
    )
    after = profiles._GpuSnapshot(
        devices=(
            profiles._GpuDeviceSnapshot(0.6, 1100.0, 150.0, True),
            profiles._GpuDeviceSnapshot(1.0, 9000.0, 0.0, False),
        )
    )

    assert profiles.gpu_metrics(before, after) == {
        "device_gpu_utilization": 0.6,
        "device_vram_used_mib": 1100.0,
        "process_vram_used_mib": 150.0,
    }


def test_gpu_metrics_omit_incomplete_process_vram() -> None:
    """Do not report a partial total when one selected device lacks process data."""
    before = profiles._GpuSnapshot(
        devices=(
            profiles._GpuDeviceSnapshot(0.2, 1000.0, 100.0, True),
            profiles._GpuDeviceSnapshot(0.3, 2000.0, None, True),
        )
    )
    after = profiles._GpuSnapshot(
        devices=(
            profiles._GpuDeviceSnapshot(0.4, 1100.0, 150.0, True),
            profiles._GpuDeviceSnapshot(0.5, 2100.0, None, True),
        )
    )

    metrics = profiles.gpu_metrics(before, after)

    assert metrics["device_gpu_utilization"] == 0.5
    assert metrics["device_vram_used_mib"] == 3200.0
    assert "process_vram_used_mib" not in metrics


@pytest.mark.parametrize("known_first", [False, True])
def test_gpu_metrics_require_both_process_memory_boundaries(known_first: bool) -> None:
    """Do not turn an unavailable active allocation into the other boundary's zero.

    Args:
        known_first: Whether the known memory observation precedes the unavailable one.

    """
    inactive = profiles._GpuSnapshot(devices=(profiles._GpuDeviceSnapshot(0.2, 1000.0, 0.0, False),))
    unavailable = profiles._GpuSnapshot(devices=(profiles._GpuDeviceSnapshot(0.4, 1100.0, None, True),))
    before, after = (inactive, unavailable) if known_first else (unavailable, inactive)

    metrics = profiles.gpu_metrics(before, after)

    assert metrics["device_gpu_utilization"] == 0.4
    assert "process_vram_used_mib" not in metrics


def test_nvml_monitor_reports_driver_initialization_failure() -> None:
    """Turn an unavailable NVML driver into a clear benchmark-facing error."""
    nvml = _fake_nvml()
    nvml.nvmlInit.side_effect = _FakeNvmlError("Driver Not Loaded")

    with pytest.raises(RuntimeError, match="NVML initialization failed: Driver Not Loaded"):
        profiles._NvmlMonitor(nvml)

    nvml.nvmlShutdown.assert_not_called()


def test_process_metrics_omit_unavailable_resident_memory() -> None:
    """Do not turn unsupported resident-memory collection into a zero sample."""
    before = profiles._ProcessSnapshot(rss_mib=None)
    after = profiles._ProcessSnapshot(rss_mib=None)

    metrics = profiles.process_metrics(before, after, elapsed_seconds=1.0, cpu_seconds=0.5)

    assert metrics["process_cpu_time_s"] == 0.5
    assert "process_rss_mib" not in metrics


def test_resident_memory_is_unavailable_when_host_query_fails(monkeypatch: pytest.MonkeyPatch) -> None:
    """Represent a missing host RSS interface explicitly.

    Args:
        monkeypatch: Temporary host-query modifier.

    """

    def unavailable() -> float:
        raise FileNotFoundError

    monkeypatch.setattr(profiles.os, "name", "posix")
    monkeypatch.setattr(profiles, "_procfs_resident_memory_mib", unavailable)

    assert profiles._resident_memory_mib() is None


def test_gpu_metrics_do_not_attribute_unrelated_devices_to_an_inactive_process() -> None:
    """Report known zero process VRAM without adopting another workload's device KPIs."""
    snapshot = profiles._GpuSnapshot((profiles._GpuDeviceSnapshot(0.9, 8000.0, 0.0, False),))

    assert profiles.gpu_metrics(snapshot, snapshot) == {"process_vram_used_mib": 0.0}


@pytest.mark.parametrize("other_active", [False, True])
def test_gpu_metrics_reject_unknown_device_attribution(other_active: bool) -> None:
    """Do not report a complete workload aggregate while any device's PID observation is unknown.

    Args:
        other_active: Whether another device has confirmed process activity.

    """
    before = profiles._GpuSnapshot(
        (
            profiles._GpuDeviceSnapshot(0.1, 100.0, 1.0 if other_active else 0.0, other_active),
            profiles._GpuDeviceSnapshot(0.9, 8000.0, None, None),
        )
    )
    after = profiles._GpuSnapshot(
        (
            profiles._GpuDeviceSnapshot(0.2, 200.0, 2.0 if other_active else 0.0, other_active),
            profiles._GpuDeviceSnapshot(1.0, 9000.0, 0.0, False),
        )
    )

    with pytest.raises(RuntimeError, match="GPU process activity is unavailable"):
        profiles.gpu_metrics(before, after)


@pytest.mark.parametrize("failure", ["query", "missing", "sentinel"])
def test_builtin_gpu_memory_preserves_incomplete_observations(monkeypatch: pytest.MonkeyPatch, failure: str) -> None:
    """Keep observed activity separate from memory that a second query cannot confirm.

    Args:
        monkeypatch: Temporary attribute and environment overrides.
        failure: Simulated NVML query failure, missing query, or unavailable memory sentinel.

    """
    nvml = _fake_nvml()
    monkeypatch.setattr(profiles.os, "getpid", lambda: 42)
    if failure == "query":
        nvml.nvmlDeviceGetGraphicsRunningProcesses = mock.Mock(side_effect=_FakeNvmlError("query failed"))
    elif failure == "missing":
        del nvml.nvmlDeviceGetGraphicsRunningProcesses
    else:
        nvml.nvmlDeviceGetGraphicsRunningProcesses = lambda handle: [
            SimpleNamespace(pid=42, usedGpuMemory=nvml.NVML_VALUE_NOT_AVAILABLE)
        ]
    monitor = profiles._NvmlMonitor(nvml)
    try:
        snapshot = monitor.snapshot()
        assert snapshot.devices[0].process_active is True
        assert snapshot.devices[0].process_vram_used_mib is None
        if failure != "sentinel":
            assert snapshot.devices[1].process_active is None
            assert snapshot.devices[1].process_vram_used_mib is None
    finally:
        monitor.close()
