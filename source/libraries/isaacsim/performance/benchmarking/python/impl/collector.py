# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

"""Adapt resource metric providers to the benchmark runner."""

from __future__ import annotations

import importlib
import math
import os
import platform
import re
from contextlib import AbstractContextManager, nullcontext
from dataclasses import dataclass
from typing import Any

from isaacsim.performance.benchmarking.impl import profiles

_BACKENDS = ("auto", "ovperf", "builtin")
_DOMAIN = "isaacsim.performance.benchmarking"
_MIB = 1024.0 * 1024.0
_OVPERF_VERSION = "0.3.1"
_GPU_SUFFIXES = (
    "nvml_utilization_ratio",
    "memory.used_bytes",
    "memory.reserved_bytes",
)
_GPU_METRIC_PATTERN = re.compile(r"^gpu\.(.+)\.(" + "|".join(map(re.escape, _GPU_SUFFIXES)) + r")$")


def _validate_resource_options(backend: str, capture: bool) -> None:
    """Validate provider options shared by the public API and collector.

    Args:
        backend: Requested resource provider.
        capture: Whether to retain native collection data.

    Raises:
        ValueError: If the backend is invalid or capture is combined with builtin providers.
        TypeError: If capture is not Boolean.

    """
    if not isinstance(backend, str) or backend not in _BACKENDS:
        raise ValueError(f"resource_backend must be one of {_BACKENDS}, not {backend!r}")
    if not isinstance(capture, bool):
        raise TypeError("capture_collector must be a bool")
    if capture and backend == "builtin":
        raise ValueError("capture_collector requires the ovperf resource_backend")


@dataclass(slots=True)
class _CollectorState:
    """Track run progress, owned native references, and capture export.

    Initialization and collection ownership are independent of run progress:
    startup can fail after acquiring a reference that cleanup must still release.
    """

    started: bool = False
    #: Provider startup completed.
    closed: bool = False
    #: Close completed with no owned native references remaining.

    init_owned: bool = False
    #: This run acquired an initialization reference that still needs release.
    collection_owned: bool = False
    #: This run acquired a collection reference that still needs release.

    capture_started: bool = False
    #: Periodic collection started for this measured phase.
    capture_export_attempted: bool = False
    #: Capture export was attempted, whether it succeeded or failed.


@dataclass(frozen=True)
class _ResourceSnapshot:
    """Resource observations captured at one measured boundary."""

    process: profiles._ProcessSnapshot | None
    gpu: profiles._GpuSnapshot | None
    raw: dict[str, Any] | None


def _load_ovperf() -> Any:
    """Load the released native collector package.

    Returns:
        Imported ovperf-collector Python API.

    """
    return importlib.import_module("ovperf_collector")


def _native_platform_supported() -> bool:
    """Return whether the released wheel supports this host platform.

    Returns:
        Whether the host is Linux x86_64.

    """
    return platform.system() == "Linux" and platform.machine().lower() == "x86_64"


def _native_error(native: Any, operation: str, status: int) -> RuntimeError:
    """Create a benchmark-facing native lifecycle error.

    Args:
        native: Loaded collector API supplying error details.
        operation: Name of the failed native operation.
        status: Native error status returned by the operation.

    Returns:
        Exception describing the failed operation and available native error details.

    """
    detail = native.get_last_error()
    suffix = f": {detail}" if detail else ""
    return RuntimeError(f"ovperf-collector {operation} failed with status {status}{suffix}")


def _current_observation(metrics: dict[str, Any], name: str) -> dict[str, Any] | None:
    """Return one available observation acquired by the current fresh request.

    Args:
        metrics: Observations from a native snapshot, keyed by metric name.
        name: Metric to retrieve.

    Returns:
        Observation with a finite numeric value, or ``None`` if it is missing,
        unavailable, stale, or has an invalid value.

    """
    observation = metrics.get(name)
    if not isinstance(observation, dict):
        return None
    if not observation.get("available") or not observation.get("fresh"):
        return None
    value = observation.get("value")
    if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value):
        return None
    return observation


class _ResourceCollector:
    """Collect process and GPU boundary observations for one benchmark run.

    Args:
        metric_profiles: Metric profile names requested by the benchmark.
        run_id: Identifier stored in an optional collector artifact.
        backend: Resource provider selection.
        capture: Whether to retain native collection data for the measured phase.

    Raises:
        ValueError: If an option is invalid or capture is combined with builtin providers.
        TypeError: If capture is not Boolean.
    """

    def __init__(
        self,
        metric_profiles: tuple[str, ...],
        run_id: str,
        *,
        backend: str = "auto",
        capture: bool = False,
    ) -> None:
        _validate_resource_options(backend, capture)

        self._state = _CollectorState()
        self._capture = capture
        self._capture_samples: list[dict[str, Any]] = []
        self._domain: Any | None = None
        self._gpu_monitor: profiles._NvmlMonitor | None = None
        self._measure_range: AbstractContextManager[Any] | None = None
        self._native: Any | None = None
        self._requested_backend = backend
        self._requested_gpu = "gpu" in metric_profiles
        self._requested_process = "process" in metric_profiles
        self._run_id = run_id
        self._mode = "none"
        self.capture_document: dict[str, Any] | None = None
        self.provenance: dict[str, Any] = {"backend": "none"}

    def start(self) -> None:
        """Prepare tracing and the resource providers requested by this run."""
        if self._state.started:
            return

        native_supported = _native_platform_supported()
        use_native = self._requested_backend == "ovperf" or (self._requested_backend == "auto" and native_supported)
        if self._capture and not native_supported:
            raise RuntimeError("Resource capture requires ovperf-collector on Linux x86_64")
        if self._requested_backend == "ovperf" and not native_supported:
            raise RuntimeError(f"ovperf-collector {_OVPERF_VERSION} is supported only on Linux x86_64")

        if not use_native:
            if not (self._requested_process or self._requested_gpu or self._capture):
                self._state.started = True
                return
            self._start_builtin()
            return
        self._start_native()

    def begin_measurement(self) -> None:
        """Begin native MEASURE tracing and optional sample recording."""
        if self._measure_range is not None:
            raise RuntimeError("Resource measurement is already active")
        if self._mode == "ovperf" and self._capture:
            native = self._native_api()
            if native.is_collecting():
                raise RuntimeError("ovperf-collector is already collecting for another owner")
            status = native.start_collection()
            self._state.collection_owned = status in (
                native.API_SUCCESS,
                native.API_ERROR_ALREADY_RUNNING,
            )
            if status == native.API_ERROR_ALREADY_RUNNING:
                release_error = self._release_collection()
                message = "ovperf-collector collection owner appeared during start"
                if release_error is not None:
                    message = f"{message}; {release_error}"
                raise RuntimeError(message)
            if status != native.API_SUCCESS:
                raise _native_error(native, "start_collection", status)
            self._state.capture_started = True

        self._measure_range = self.range("MEASURE")
        self._measure_range.__enter__()

    def end_measurement(self) -> None:
        """End native MEASURE tracing and preserve optional capture data."""
        first_error: BaseException | None = None
        if self._state.collection_owned:
            first_error = self._release_collection()
        if self._measure_range is not None:
            measure_range = self._measure_range
            self._measure_range = None
            try:
                measure_range.__exit__(None, None, None)
            except BaseException as error:
                first_error = first_error or error
        if self._state.capture_started and not self._state.capture_export_attempted and first_error is None:
            try:
                self._preserve_capture()
            except BaseException as error:
                first_error = error
        if first_error is not None:
            raise first_error

    def snapshot(self) -> _ResourceSnapshot:
        """Capture a fresh resource boundary outside the benchmark stopwatch.

        Returns:
            Boundary observations for the requested profiles and optional native capture.
            Unrequested profiles are ``None``; raw native data is ``None`` for builtin providers
            or when no resource queries are needed.

        Raises:
            RuntimeError: If the provider cannot supply a snapshot or required GPU observations.

        """
        if self._mode == "none" or not (self._requested_process or self._requested_gpu or self._capture):
            return _ResourceSnapshot(process=None, gpu=None, raw=None)
        if self._mode == "builtin":
            process = profiles.process_snapshot() if self._requested_process else None
            gpu = self._gpu_monitor.snapshot() if self._gpu_monitor is not None else None
            return _ResourceSnapshot(process=process, gpu=gpu, raw=None)

        native = self._native_api()
        document = native.snapshot(mode="fresh")
        metrics = document.get("metrics")
        if not isinstance(metrics, dict):
            raise RuntimeError("ovperf-collector fresh snapshot did not contain a metrics object")
        process = self._process_snapshot(metrics) if self._requested_process else None
        gpu = self._gpu_snapshot(metrics) if self._requested_gpu else None
        return _ResourceSnapshot(process=process, gpu=gpu, raw=document)

    def range(self, name: str) -> AbstractContextManager[Any]:
        """Create a native profiling range or a no-op context manager.

        Args:
            name: Label displayed in the profiling timeline.

        Returns:
            Profiling context manager, or a no-op context manager without a native domain.

        """
        if self._domain is None:
            return nullcontext()
        return self._domain.range(name)

    def record_sample(
        self,
        index: int,
        batch_size: int,
        elapsed_seconds: float,
        before: _ResourceSnapshot,
        after: _ResourceSnapshot,
        *,
        cpu_before: float,
        cpu_after: float,
    ) -> None:
        """Retain boundary records for one measured sample when capture is enabled.

        Args:
            index: Zero-based measured sample index.
            batch_size: Number of workload invocations in the sample.
            elapsed_seconds: Workload wall-clock duration in seconds.
            before: Resource observations captured before the workload stopwatch.
            after: Resource observations captured after the workload stopwatch.
            cpu_before: Cumulative process CPU seconds read before the workload stopwatch.
            cpu_after: Cumulative process CPU seconds read after the workload stopwatch.

        """
        if not self._capture:
            return
        self._capture_samples.append(
            {
                "index": index,
                "batch_size": batch_size,
                "elapsed_seconds": elapsed_seconds,
                "before": {
                    "snapshot": before.raw,
                    "process_cpu_seconds": cpu_before,
                },
                "after": {
                    "snapshot": after.raw,
                    "process_cpu_seconds": cpu_after,
                },
            }
        )

    def close(self) -> None:
        """Release every provider reference acquired by this collector."""
        if self._state.closed:
            return
        first_error: BaseException | None = None
        try:
            self.end_measurement()
        except BaseException as error:
            first_error = error
        if self._gpu_monitor is not None:
            monitor = self._gpu_monitor
            self._gpu_monitor = None
            try:
                monitor.close()
            except BaseException as error:
                first_error = first_error or error
        if self._state.init_owned:
            release_error = self._release_initialization()
            first_error = first_error or release_error
        self._state.closed = not self._state.collection_owned and not self._state.init_owned
        if first_error is not None:
            raise first_error

    def _start_builtin(self) -> None:
        """Prepare built-in providers for an unsupported host or an explicit selection."""
        self._gpu_monitor = profiles._NvmlMonitor() if self._requested_gpu else None
        self._mode = "builtin"
        self._state.started = True
        self.provenance = {"backend": "builtin"}

    def _start_native(self) -> None:
        """Prepare the released native collector with exclusive ownership."""
        try:
            native = _load_ovperf()
        except ModuleNotFoundError as error:
            if self._requested_backend == "auto" and not self._capture and error.name == "ovperf_collector":
                self._start_builtin()
                return
            raise RuntimeError(
                f"The ovperf backend requires ovperf-collector {_OVPERF_VERSION} on Linux x86_64"
            ) from error
        except (ImportError, OSError) as error:
            raise RuntimeError(
                f"The ovperf backend requires ovperf-collector {_OVPERF_VERSION} on Linux x86_64"
            ) from error
        if not native.SUPPORTED:
            raise RuntimeError("ovperf-collector native snapshots are unavailable on this Linux x86_64 host")
        if native.__version__ != _OVPERF_VERSION:
            raise RuntimeError(f"ovperf-collector Python wrapper must be {_OVPERF_VERSION}, found {native.__version__}")
        native_version = native.get_version()
        if native_version != tuple(map(int, _OVPERF_VERSION.split("."))):
            raise RuntimeError(f"ovperf-collector native library must be {_OVPERF_VERSION}, found {native_version}")
        if native.is_initialized():
            raise RuntimeError("ovperf-collector is already initialized by another owner")

        metric_selection: dict[str, bool] = {"*": False}
        if self._requested_process:
            metric_selection["process.memory.rss_bytes"] = True
        if self._requested_gpu:
            metric_selection.update(
                {
                    "gpu.nvml_utilization_ratio": True,
                    "gpu.memory.used_bytes": True,
                    "gpu.memory.reserved_bytes": True,
                    "gpu.process.memory_bytes": True,
                }
            )
        status = native.init(
            component=_DOMAIN,
            gpu=self._requested_gpu,
            gpm=False,
            metrics=metric_selection,
        )
        self._native = native
        self._state.init_owned = status in (native.API_SUCCESS, native.API_ERROR_ALREADY_RUNNING)
        if status == native.API_ERROR_ALREADY_RUNNING:
            release_error = self._release_initialization()
            message = "ovperf-collector initialization owner appeared during init"
            if release_error is not None:
                message = f"{message}; {release_error}"
            raise RuntimeError(message)
        if status != native.API_SUCCESS:
            raise _native_error(native, "init", status)

        if self._requested_gpu:
            hardware = native.hardware_info()
            capability = hardware.get("capability", {})
            discovery = capability.get("gpu_discovery", "unknown")
            if discovery != "ready" or not hardware.get("gpus"):
                detail = capability.get("gpu_discovery_error") or "no complete NVIDIA GPU inventory"
                raise RuntimeError(f"ovperf-collector GPU discovery {discovery}: {detail}")

        self._domain = native.Domain(_DOMAIN)
        self._mode = "ovperf"
        self._state.started = True
        self.provenance = {
            "backend": "ovperf",
            "wrapper_version": native.__version__,
            "native_version": ".".join(str(part) for part in native_version),
        }

    def _process_snapshot(self, metrics: dict[str, Any]) -> profiles._ProcessSnapshot:
        """Translate native observations to a process memory snapshot.

        Args:
            metrics: Observations from a native snapshot, keyed by metric name.

        Returns:
            Process snapshot with RSS in mebibytes, or ``None`` for RSS when unavailable.

        """
        rss = _current_observation(metrics, "process.memory.rss_bytes")
        rss_mib = float(rss["value"]) / _MIB if rss is not None else None
        return profiles._ProcessSnapshot(rss_mib=rss_mib)

    def _gpu_snapshot(self, metrics: dict[str, Any]) -> profiles._GpuSnapshot:
        """Translate native observations to GPU boundary snapshots.

        Args:
            metrics: Observations from a native snapshot, keyed by metric name.

        Returns:
            Device snapshots with independent process activity and memory availability.
            Either process field is ``None`` when its observation is unavailable.

        Raises:
            RuntimeError: If no devices or required fresh device observations are reported.

        """
        device_names = {match.group(1) for name in metrics if (match := _GPU_METRIC_PATTERN.match(name)) is not None}
        if not device_names:
            raise RuntimeError("ovperf-collector fresh snapshot did not report any GPU devices")

        devices: list[profiles._GpuDeviceSnapshot] = []
        current_pid = os.getpid()
        for device_name in sorted(device_names):
            required: dict[str, dict[str, Any]] = {}
            for suffix in _GPU_SUFFIXES:
                observation = _current_observation(metrics, f"gpu.{device_name}.{suffix}")
                if observation is None:
                    raise RuntimeError(
                        f"ovperf-collector GPU observation {suffix!r} is missing, unavailable, or not fresh"
                    )
                required[suffix] = observation

            process = _current_observation(
                metrics,
                f"gpu.{device_name}.process.pid{current_pid}.memory_bytes",
            )
            activity = _current_observation(metrics, f"gpu.{device_name}.process.pid{current_pid}.active_count")
            devices.append(
                profiles._GpuDeviceSnapshot(
                    utilization=float(required["nvml_utilization_ratio"]["value"]),
                    vram_used_mib=(
                        float(required["memory.used_bytes"]["value"])
                        + float(required["memory.reserved_bytes"]["value"])
                    )
                    / _MIB,
                    process_vram_used_mib=float(process["value"]) / _MIB if process is not None else None,
                    process_active=bool(activity["value"]) if activity is not None else None,
                )
            )
        return profiles._GpuSnapshot(devices=tuple(devices))

    def _release_collection(self) -> BaseException | None:
        """Release this collector's native collection reference.

        Returns:
            ``None`` after a successful release, or an error with ownership retained for retry.

        """
        native = self._native_api()
        status = native.stop_collection()
        if status != native.API_SUCCESS:
            return _native_error(native, "stop_collection", status)
        self._state.collection_owned = False
        return None

    def _release_initialization(self) -> BaseException | None:
        """Release this collector's native initialization reference.

        Returns:
            ``None`` after a successful release, or an error with ownership retained for retry.

        """
        native = self._native_api()
        status = native.shutdown()
        if status != native.API_SUCCESS:
            return _native_error(native, "shutdown", status)
        self._state.init_owned = False
        return None

    def _preserve_capture(self) -> None:
        """Retain the collector artifact before native shutdown clears the run."""
        native = self._native_api()
        self._state.capture_export_attempted = True
        if native.is_collecting():
            raise RuntimeError("Cannot capture ovperf-collector while another owner is still collecting")
        self.capture_document = {
            "schema": "isaacsim.performance.benchmarking.collector.v1",
            "run_id": self._run_id,
            "clocks": {
                "snapshot_timestamp": "absolute CLOCK_MONOTONIC seconds",
                "native_run_t": "seconds relative to ovperf-collector initialization",
                "process_cpu_seconds": "cumulative process CPU seconds from time.process_time()",
            },
            "samples": self._capture_samples,
            "run": native.run(),
        }

    def _native_api(self) -> Any:
        """Return the loaded native collector API.

        Returns:
            Loaded ovperf-collector Python API.

        Raises:
            RuntimeError: If the native API has not been loaded.

        """
        if self._native is None:
            raise RuntimeError("ovperf-collector native API is not loaded")
        return self._native
