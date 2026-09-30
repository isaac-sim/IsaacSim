# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

"""Provide concrete metric providers that work without Kit."""

from __future__ import annotations

import importlib
import os
import time
from dataclasses import dataclass
from typing import Any

_MIB = 1024.0 * 1024.0


def _procfs_resident_memory_mib() -> float:
    """Return current resident memory from procfs.

    Returns:
        Resident memory in mebibytes.

    """
    with open("/proc/self/statm", encoding="utf-8") as statm:
        resident_pages = int(statm.read().split()[1])
    return resident_pages * os.sysconf("SC_PAGE_SIZE") / _MIB


def _windows_resident_memory_mib() -> float:
    """Return current resident memory from the Windows process API.

    Returns:
        Resident memory in mebibytes.

    """
    import ctypes
    from ctypes import wintypes

    class ProcessMemoryCounters(ctypes.Structure):
        """Native process memory counters returned by GetProcessMemoryInfo."""

        _fields_ = [
            ("cb", wintypes.DWORD),
            ("PageFaultCount", wintypes.DWORD),
            ("PeakWorkingSetSize", ctypes.c_size_t),
            ("WorkingSetSize", ctypes.c_size_t),
            ("QuotaPeakPagedPoolUsage", ctypes.c_size_t),
            ("QuotaPagedPoolUsage", ctypes.c_size_t),
            ("QuotaPeakNonPagedPoolUsage", ctypes.c_size_t),
            ("QuotaNonPagedPoolUsage", ctypes.c_size_t),
            ("PagefileUsage", ctypes.c_size_t),
            ("PeakPagefileUsage", ctypes.c_size_t),
        ]

    counters = ProcessMemoryCounters()
    counters.cb = ctypes.sizeof(counters)
    get_current_process = ctypes.windll.kernel32.GetCurrentProcess
    get_current_process.restype = wintypes.HANDLE
    get_process_memory_info = ctypes.windll.psapi.GetProcessMemoryInfo
    get_process_memory_info.argtypes = [wintypes.HANDLE, ctypes.POINTER(ProcessMemoryCounters), wintypes.DWORD]
    get_process_memory_info.restype = wintypes.BOOL
    process = get_current_process()
    succeeded = get_process_memory_info(process, ctypes.byref(counters), counters.cb)
    if not succeeded:
        raise OSError(ctypes.get_last_error(), "GetProcessMemoryInfo failed")
    return float(counters.WorkingSetSize) / _MIB


def _resident_memory_mib() -> float | None:
    """Return current resident memory when the host provides a supported API.

    Returns:
        Resident memory in mebibytes, or ``None`` when unavailable.

    """
    try:
        if os.name == "nt":
            return _windows_resident_memory_mib()
        return _procfs_resident_memory_mib()
    except (AttributeError, FileNotFoundError, IndexError, OSError, ValueError):
        return None


@dataclass(frozen=True)
class _ProcessSnapshot:
    """Low-overhead process counters captured outside the measured timer."""

    cpu_seconds: float
    rss_mib: float | None


def process_snapshot() -> _ProcessSnapshot:
    """Capture process counters used by the process metric profile.

    Returns:
        Current process snapshot.

    """
    return _ProcessSnapshot(cpu_seconds=time.process_time(), rss_mib=_resident_memory_mib())


def process_metrics(before: _ProcessSnapshot, after: _ProcessSnapshot, elapsed_seconds: float) -> dict[str, float]:
    """Derive process metrics for one measured sample.

    Args:
        before: Process snapshot captured before the sample.
        after: Process snapshot captured after the sample.
        elapsed_seconds: Measured wall-clock duration.

    Returns:
        Available process metrics keyed by metric name.

    """
    cpu_seconds = max(0.0, after.cpu_seconds - before.cpu_seconds)
    metrics = {
        "process_cpu_time_s": cpu_seconds,
        "process_cpu_utilization": cpu_seconds / elapsed_seconds if elapsed_seconds > 0.0 else 0.0,
    }
    rss_values = [value for value in (before.rss_mib, after.rss_mib) if value is not None]
    if rss_values:
        metrics["process_rss_mib"] = max(rss_values)
    return metrics


@dataclass(frozen=True)
class _GpuDeviceSnapshot:
    """NVML counters for one device captured outside the measured timer."""

    utilization: float
    vram_used_mib: float
    process_vram_used_mib: float | None
    process_active: bool


@dataclass(frozen=True)
class _GpuSnapshot:
    """NVML counters for every visible device at one sample boundary."""

    devices: tuple[_GpuDeviceSnapshot, ...]


class _NvmlMonitor:
    """Own one NVML session used to collect GPU benchmark metrics.

    Args:
        nvml: NVML-compatible module, or ``None`` to import it.

    """

    def __init__(self, nvml: Any | None = None) -> None:
        if nvml is None:
            try:
                nvml = importlib.import_module("pynvml")
            except ImportError as error:
                raise RuntimeError("GPU metrics require the nvidia-ml-py package") from error

        self._nvml = nvml
        self._initialized = False
        try:
            self._nvml.nvmlInit()
            self._initialized = True
            self._handles = tuple(
                self._nvml.nvmlDeviceGetHandleByIndex(index) for index in range(self._nvml.nvmlDeviceGetCount())
            )
        except self._nvml.NVMLError as error:
            self._shutdown_after_initialization_failure()
            raise RuntimeError(f"NVML initialization failed: {error}") from error
        if not self._handles:
            self.close()
            raise RuntimeError("NVML did not report any NVIDIA GPUs")

    def close(self) -> None:
        """Release the monitor's NVML session."""
        if not self._initialized:
            return
        self._initialized = False
        try:
            self._nvml.nvmlShutdown()
        except self._nvml.NVMLError as error:
            raise RuntimeError(f"NVML shutdown failed: {error}") from error

    def snapshot(self) -> _GpuSnapshot:
        """Capture GPU utilization and VRAM counters for every visible device.

        Returns:
            Current counters for every visible device.

        """
        devices: list[_GpuDeviceSnapshot] = []
        for index, handle in enumerate(self._handles):
            try:
                utilization = self._nvml.nvmlDeviceGetUtilizationRates(handle)
                memory = self._nvml.nvmlDeviceGetMemoryInfo(handle)
            except self._nvml.NVMLError as error:
                raise RuntimeError(f"NVML failed to query GPU {index}: {error}") from error
            process_active, process_vram_used_mib = self._process_vram_used_mib(handle)
            devices.append(
                _GpuDeviceSnapshot(
                    utilization=float(utilization.gpu) / 100.0,
                    vram_used_mib=float(memory.used) / _MIB,
                    process_vram_used_mib=process_vram_used_mib,
                    process_active=process_active,
                )
            )
        return _GpuSnapshot(devices=tuple(devices))

    def _process_vram_used_mib(self, handle: Any) -> tuple[bool, float | None]:
        """Return whether this process uses a device and its VRAM when available.

        Args:
            handle: NVML device handle.

        Returns:
            Process activity and VRAM usage in mebibytes, if available.

        """
        process_active = False
        process_queries_succeeded = False
        process_query_failed = False
        used_memory_values: list[float] = []
        for query_name in ("nvmlDeviceGetComputeRunningProcesses", "nvmlDeviceGetGraphicsRunningProcesses"):
            query = getattr(self._nvml, query_name, None)
            if query is None:
                continue
            try:
                processes = query(handle)
            except self._nvml.NVMLError:
                process_query_failed = True
                continue
            process_queries_succeeded = True
            for process in processes:
                if process.pid != os.getpid():
                    continue
                process_active = True
                used_memory = getattr(process, "usedGpuMemory", None)
                if used_memory is None or used_memory == getattr(self._nvml, "NVML_VALUE_NOT_AVAILABLE", None):
                    continue
                used_memory_values.append(float(used_memory) / _MIB)

        if used_memory_values:
            # Compute and graphics queries can both report the same process allocation.
            return process_active, max(used_memory_values)
        available_zero = process_queries_succeeded and not process_query_failed and not process_active
        return process_active, 0.0 if available_zero else None

    def _shutdown_after_initialization_failure(self) -> None:
        """Release NVML without masking the initialization error."""
        if not self._initialized:
            return
        self._initialized = False
        try:
            self._nvml.nvmlShutdown()
        except self._nvml.NVMLError:
            pass


def gpu_metrics(before: _GpuSnapshot, after: _GpuSnapshot) -> dict[str, float]:
    """Derive aggregate GPU metrics for one measured sample.

    Args:
        before: GPU snapshot captured before the sample.
        after: GPU snapshot captured after the sample.

    Returns:
        Available GPU metrics keyed by metric name.

    """
    if len(before.devices) != len(after.devices):
        raise RuntimeError("NVML device count changed during benchmark measurement")

    selected_indices = [
        index
        for index, (before_device, after_device) in enumerate(zip(before.devices, after.devices))
        if before_device.process_active or after_device.process_active
    ]
    if not selected_indices:
        selected_indices = list(range(len(before.devices)))

    metrics = {
        "device_gpu_utilization": max(
            max(before.devices[index].utilization, after.devices[index].utilization) for index in selected_indices
        ),
        "device_vram_used_mib": sum(
            max(before.devices[index].vram_used_mib, after.devices[index].vram_used_mib) for index in selected_indices
        ),
    }
    process_vram_used_mib = 0.0
    process_vram_available = True
    for index in selected_indices:
        values = [
            value
            for value in (
                before.devices[index].process_vram_used_mib,
                after.devices[index].process_vram_used_mib,
            )
            if value is not None
        ]
        if not values:
            process_vram_available = False
            break
        process_vram_used_mib += max(values)
    if process_vram_available:
        metrics["process_vram_used_mib"] = process_vram_used_mib
    return metrics
