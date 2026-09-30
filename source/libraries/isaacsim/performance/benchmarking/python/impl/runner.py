# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

"""One lifecycle runner for function and stateful class benchmarks."""

from __future__ import annotations

import inspect
import random
import statistics
import sys
import time
import uuid
from collections.abc import Callable
from typing import Any

from isaacsim.performance.benchmarking.impl.authoring import (
    _validate_simulated_time,
    bind_parameters,
    measurement_context,
)
from isaacsim.performance.benchmarking.impl.environment import collect_environment
from isaacsim.performance.benchmarking.impl.model import (
    _ActiveMeasurement,
    _BenchmarkDefinition,
    _BenchmarkResult,
    _RegionObservation,
    _SampleResult,
)
from isaacsim.performance.benchmarking.impl.profiles import (
    _NvmlMonitor,
    gpu_metrics,
    process_metrics,
    process_snapshot,
)

_MINIMUM_OPERATION_BATCH_SECONDS = 0.05
_MAXIMUM_OPERATION_BATCH_SIZE = 1 << 20
_BUILTIN_METRIC_UNITS = {
    "duration_ms": "ms",
    "total_duration_ms": "ms",
    "throughput_per_s": "operations/s",
    "process_cpu_time_s": "s",
    "process_cpu_utilization": "ratio",
    "process_rss_mib": "MiB",
    "device_gpu_utilization": "ratio",
    "device_vram_used_mib": "MiB",
    "process_vram_used_mib": "MiB",
    "simulated_time_s": "s",
    "real_time_factor": "ratio",
    "simulation_step_latency_ms": "ms",
    "simulation_steps_per_s": "steps/s",
}


async def _await_result(value: Any) -> Any:
    """Await a lifecycle return value when needed.

    Args:
        value: Synchronous value or awaitable.

    Returns:
        Resolved lifecycle return value.

    """
    if inspect.isawaitable(value):
        return await value
    return value


def _error_message(phase: str, error: BaseException) -> str:
    """Format a stable lifecycle failure.

    Args:
        phase: Lifecycle phase that failed.
        error: Exception raised by the phase.

    Returns:
        Stable human-readable failure message.

    """
    return f"{phase}: {type(error).__name__}: {error}"


def _retain_regions(result: _BenchmarkResult, state: _ActiveMeasurement) -> None:
    """Move aggregate region observations from an active boundary into the result.

    Args:
        result: Result receiving the observations.
        state: Completed measurement boundary.

    """
    for name, values in state.region_values.items():
        result.regions.append(
            _RegionObservation(
                name=name,
                phase=state.phase,
                sample=state.sample,
                count=len(values),
                total_ms=sum(values),
                minimum_ms=min(values),
                maximum_ms=max(values),
            )
        )


def _seed_random_generators(seed: int) -> None:
    """Seed Python and an already-loaded NumPy module without adding a dependency.

    Args:
        seed: Deterministic random seed.

    """
    random.seed(seed)
    numpy = sys.modules.get("numpy")
    if numpy is not None and hasattr(numpy, "random"):
        numpy.random.seed(seed)


class _Execution:
    """Adapt a decorated function or class to the fixed lifecycle.

    Args:
        definition: Benchmark definition to execute.
        parameters: Effective benchmark parameters.

    """

    def __init__(self, definition: _BenchmarkDefinition, parameters: dict[str, Any]) -> None:
        self._definition = definition
        self._parameters = parameters
        self._instance = definition.target(**parameters) if isinstance(definition.target, type) else None

    @property
    def stateful(self) -> bool:
        """Return whether this execution owns a class instance."""
        return self._instance is not None

    def hook(self, name: str) -> Callable[[], Any] | None:
        """Return an optional bound lifecycle method.

        Args:
            name: Lifecycle method name.

        Returns:
            Bound lifecycle method, or ``None`` when unavailable.

        """
        if self._instance is None:
            return None
        hook = getattr(self._instance, name, None)
        return hook if callable(hook) else None

    async def run(self) -> None:
        """Execute one complete unit of measured work."""
        if self._instance is None:
            value = self._definition.target(**self._parameters)
        else:
            value = self._instance.run()
        await _await_result(value)


async def _call_hook(hook: Callable[[], Any] | None) -> Any:
    """Call one optional lifecycle hook.

    Args:
        hook: Lifecycle hook, if defined.

    Returns:
        Resolved hook return value, or ``None`` when no hook is defined.

    """
    if hook is None:
        return None
    return await _await_result(hook())


async def _run_batch(execution: _Execution, batch_size: int) -> None:
    """Run a calibrated batch serially on the runner event loop.

    Args:
        execution: Adapted benchmark execution.
        batch_size: Number of operations to run.

    """
    for _ in range(batch_size):
        await execution.run()


async def _calibrate_batch_size(execution: _Execution, clock_ns: Callable[[], int]) -> int:
    """Choose an operation batch large enough for stable host timing.

    Args:
        execution: Adapted benchmark execution.
        clock_ns: Monotonic nanosecond clock.

    Returns:
        Calibrated operations per measured sample.

    """
    batch_size = 1
    while True:
        await _call_hook(execution.hook("prepare_sample"))
        with measurement_context("calibration", None, retain=False):
            started_ns = clock_ns()
            await _run_batch(execution, batch_size)
            elapsed_seconds = (clock_ns() - started_ns) / 1_000_000_000.0
        if elapsed_seconds >= _MINIMUM_OPERATION_BATCH_SECONDS or batch_size >= _MAXIMUM_OPERATION_BATCH_SIZE:
            return batch_size
        batch_size *= 2


def _resolve_simulated_time(definition: _BenchmarkDefinition, parameters: dict[str, Any]) -> float | None:
    """Resolve simulated time from static metadata or an effective parameter.

    Args:
        definition: Benchmark definition containing simulated-time metadata.
        parameters: Effective benchmark parameters.

    Returns:
        Simulated seconds per operation, or ``None`` when not configured.

    """
    value = definition.simulated_time
    if isinstance(value, str):
        value = parameters[value]
    return _validate_simulated_time(value) if value is not None else None


async def _setup_execution(execution: _Execution, result: _BenchmarkResult, clock_ns: Callable[[], int]) -> bool:
    """Run setup and retain its lifecycle measurements.

    Args:
        execution: Adapted benchmark execution.
        result: Result receiving lifecycle data.
        clock_ns: Monotonic nanosecond clock.

    Returns:
        Whether setup completed successfully.

    """
    started_ns = clock_ns()
    state = None
    try:
        with measurement_context("setup", None, retain=True) as state:
            await _call_hook(execution.hook("setup"))
        return True
    except Exception as error:
        result.errors.append(_error_message("setup", error))
        return False
    finally:
        if state is not None:
            _retain_regions(result, state)
        result.lifecycle_ms["setup_ms"] = (clock_ns() - started_ns) / 1_000_000.0


async def _measure_execution(
    execution: _Execution,
    result: _BenchmarkResult,
    simulated_time: float | None,
    clock_ns: Callable[[], int],
) -> None:
    """Calibrate, warm up, and collect all measured samples.

    Args:
        execution: Adapted benchmark execution.
        result: Result receiving measured samples.
        simulated_time: Simulated seconds per operation, if configured.
        clock_ns: Monotonic nanosecond clock.

    """
    definition = result.definition
    gpu_monitor = None
    try:
        gpu_monitor = _NvmlMonitor() if "gpu" in definition.metric_profiles else None
        batch_size = await _calibrate_batch_size(execution, clock_ns) if definition.calibrated_batching else 1
        for _ in range(definition.warmup_samples):
            await _call_hook(execution.hook("prepare_sample"))
            with measurement_context("warmup", None, retain=False):
                await _run_batch(execution, batch_size)

        for sample_index in range(definition.measured_samples):
            await _call_hook(execution.hook("prepare_sample"))
            before_process = process_snapshot() if "process" in definition.metric_profiles else None
            before_gpu = gpu_monitor.snapshot() if gpu_monitor is not None else None
            with measurement_context("measurement", sample_index, retain=True) as state:
                started_ns = clock_ns()
                await _run_batch(execution, batch_size)
                elapsed_seconds = (clock_ns() - started_ns) / 1_000_000_000.0
            after_process = process_snapshot() if before_process is not None else None
            after_gpu = gpu_monitor.snapshot() if before_gpu is not None and gpu_monitor is not None else None
            _retain_regions(result, state)
            elapsed_ms = elapsed_seconds * 1000.0
            metrics: dict[str, float] = {}
            for name, (values, unit) in state.recorded_values.items():
                previous_unit = result.metric_units.get(name)
                if previous_unit is not None and previous_unit != unit:
                    raise ValueError(f"Benchmark metric {name!r} changed units from {previous_unit!r} to {unit!r}")
                if name in _BUILTIN_METRIC_UNITS:
                    raise ValueError(f"Benchmark metric {name!r} is reserved by the runner")
                result.metric_units[name] = unit
                metrics[name] = statistics.fmean(values)
            if before_process is not None and after_process is not None:
                metrics.update(process_metrics(before_process, after_process, elapsed_seconds))
            if before_gpu is not None and after_gpu is not None:
                metrics.update(gpu_metrics(before_gpu, after_gpu))
            if "simulation" in definition.metric_profiles and simulated_time is not None:
                completed_simulated_time = simulated_time * batch_size
                metrics.update(
                    {
                        "simulated_time_s": completed_simulated_time,
                        "real_time_factor": (
                            completed_simulated_time / elapsed_seconds if elapsed_seconds > 0.0 else 0.0
                        ),
                        "simulation_step_latency_ms": elapsed_ms / batch_size,
                        "simulation_steps_per_s": batch_size / elapsed_seconds if elapsed_seconds > 0.0 else 0.0,
                    }
                )
            result.samples.append(
                _SampleResult(
                    index=sample_index,
                    batch_size=batch_size,
                    duration_ms=elapsed_ms / batch_size,
                    total_duration_ms=elapsed_ms,
                    throughput_per_s=batch_size / elapsed_seconds if elapsed_seconds > 0.0 else 0.0,
                    metrics=metrics,
                )
            )
    except Exception as error:
        result.errors.append(_error_message("measurement", error))
    finally:
        if gpu_monitor is not None:
            try:
                gpu_monitor.close()
            except Exception as error:
                result.errors.append(_error_message("measurement", error))


async def _validate_execution(execution: _Execution, result: _BenchmarkResult, clock_ns: Callable[[], int]) -> None:
    """Run the optional validation hook once.

    Args:
        execution: Adapted benchmark execution.
        result: Result receiving validation data.
        clock_ns: Monotonic nanosecond clock.

    """
    validation_hook = execution.hook("validate")
    if validation_hook is None:
        result.validation = {"performed": False, "passed": True}
        return

    started_ns = clock_ns()
    state = None
    try:
        with measurement_context("validation", None, retain=True) as state:
            validation_value = await _call_hook(validation_hook)
        if not isinstance(validation_value, bool):
            raise TypeError("validate() must return bool")
        result.validation = {"performed": True, "passed": validation_value}
        if not validation_value:
            result.errors.append("validation: validate() returned False")
    except Exception as error:
        result.validation = {"performed": True, "passed": False, "reason": str(error)}
        result.errors.append(_error_message("validation", error))
    finally:
        if state is not None:
            _retain_regions(result, state)
        result.lifecycle_ms["validation_ms"] = (clock_ns() - started_ns) / 1_000_000.0


async def _teardown_execution(execution: _Execution, result: _BenchmarkResult, clock_ns: Callable[[], int]) -> None:
    """Run teardown and retain its lifecycle measurements.

    Args:
        execution: Adapted benchmark execution.
        result: Result receiving lifecycle data.
        clock_ns: Monotonic nanosecond clock.

    """
    teardown_hook = execution.hook("teardown")
    if teardown_hook is None:
        return

    started_ns = clock_ns()
    state = None
    try:
        with measurement_context("teardown", None, retain=True) as state:
            await _call_hook(teardown_hook)
    except Exception as error:
        result.errors.append(_error_message("teardown", error))
    finally:
        if state is not None:
            _retain_regions(result, state)
        result.lifecycle_ms["teardown_ms"] = (clock_ns() - started_ns) / 1_000_000.0


async def run_definition(
    definition: _BenchmarkDefinition,
    parameter_overrides: dict[str, Any] | None = None,
    *,
    seed: int | None = None,
    run_id: str | None = None,
    clock_ns: Callable[[], int] | None = None,
) -> _BenchmarkResult:
    """Run a decorated benchmark using the fixed lifecycle and return raw results.

    Args:
        definition: Benchmark definition to execute.
        parameter_overrides: Parameter values for this run.
        seed: Random seed override.
        run_id: Stable identifier override for this run.
        clock_ns: Monotonic nanosecond clock override.

    Returns:
        Raw benchmark result.

    """
    selected_clock = clock_ns or time.perf_counter_ns
    selected_seed = definition.seed if seed is None else seed
    parameters = bind_parameters(definition, parameter_overrides or {})
    simulated_time = _resolve_simulated_time(definition, parameters)
    result = _BenchmarkResult(
        run_id=run_id or str(uuid.uuid4()),
        definition=definition,
        parameters=parameters,
        seed=selected_seed,
    )
    result.metric_units.update(_BUILTIN_METRIC_UNITS)
    _seed_random_generators(selected_seed)

    try:
        execution = _Execution(definition, parameters)
    except Exception as error:
        result.errors.append(_error_message("construct", error))
        result.environment = collect_environment()
        return result

    try:
        setup_succeeded = await _setup_execution(execution, result, selected_clock)
        if setup_succeeded:
            await _measure_execution(execution, result, simulated_time, selected_clock)
        if setup_succeeded and not result.errors:
            await _validate_execution(execution, result, selected_clock)
    finally:
        if execution.stateful:
            await _teardown_execution(execution, result, selected_clock)

    result.valid = not result.errors and len(result.samples) == definition.measured_samples
    result.environment = collect_environment()
    return result


__all__: list[str] = []
