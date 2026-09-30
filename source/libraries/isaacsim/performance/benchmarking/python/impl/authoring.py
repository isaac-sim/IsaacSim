# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

"""Small public facade for declaring and instrumenting Python benchmarks."""

from __future__ import annotations

import asyncio
import contextvars
import inspect
import json
import math
import re
import time
from collections.abc import Callable, Iterator
from contextlib import contextmanager
from pathlib import Path
from typing import Any, TypeVar

from isaacsim.performance.benchmarking.impl.model import _ActiveMeasurement, _BenchmarkDefinition

_Target = TypeVar("_Target", bound=Callable[..., Any] | type[Any])
_IDENTIFIER = re.compile(r"^[a-z0-9]+(?:[._-][a-z0-9]+)*$")
_MEASUREMENT_NAME = re.compile(r"^[A-Za-z0-9]+(?:[._/-][A-Za-z0-9]+)*$")
_ACTIVE_MEASUREMENT: contextvars.ContextVar[_ActiveMeasurement | None] = contextvars.ContextVar(
    "isaacsim_benchmark_measurement", default=None
)
_REGION_STACK: contextvars.ContextVar[tuple[str, ...]] = contextvars.ContextVar(
    "isaacsim_benchmark_region_stack", default=()
)
_DURATIONS = {"short", "medium", "long"}
_COMPLETION = {"synchronous", "awaitable", "fetched", "fenced", "presented"}
_METRIC_PROFILES = {"gpu", "process", "simulation"}
_RESOURCES = {"cpu", "gpu"}


def _validate_simulated_time(value: Any) -> float:
    """Return a validated simulated-time value.

    Args:
        value: Value to validate.

    Returns:
        Validated value in seconds.

    """
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise TypeError("simulated_time must resolve to a number")
    numeric_value = float(value)
    if not math.isfinite(numeric_value) or numeric_value < 0.0:
        raise ValueError("simulated_time must be finite and non-negative")
    return numeric_value


def _infer_component(identifier: str) -> str:
    """Infer the owning component from a normalized benchmark identifier.

    Args:
        identifier: Normalized benchmark identifier.

    Returns:
        Inferred component identifier.

    """
    segments = identifier.split(".")
    if segments[0] == "isaacsim" and len(segments) >= 3:
        return ".".join(segments[:3])
    if segments[0] == "benchmarks" and len(segments) >= 2:
        return ".".join(segments[:2])
    return segments[0]


def _parameter_defaults(target: Callable[..., Any] | type[Any]) -> dict[str, Any]:
    """Extract JSON-compatible defaults from a function or class constructor.

    Args:
        target: Benchmark function or class.

    Returns:
        Declared parameter defaults keyed by parameter name.

    """
    defaults: dict[str, Any] = {}
    for parameter in inspect.signature(target).parameters.values():
        if parameter.kind in (parameter.VAR_POSITIONAL, parameter.VAR_KEYWORD, parameter.POSITIONAL_ONLY):
            raise TypeError("Benchmark parameters must be named and cannot use *args or **kwargs")
        if parameter.default is inspect.Parameter.empty:
            raise TypeError(f"Benchmark parameter {parameter.name!r} requires a default value")
        defaults[parameter.name] = parameter.default
    try:
        json.dumps(defaults, sort_keys=True, allow_nan=False)
    except (TypeError, ValueError) as error:
        raise TypeError("Benchmark parameter defaults must be JSON-compatible") from error
    return defaults


def _validate_lifecycle(target: type[Any]) -> None:
    """Validate the fixed lifecycle of a stateful benchmark class.

    Args:
        target: Stateful benchmark class to validate.

    """
    run = getattr(target, "run", None)
    if not callable(run):
        raise TypeError("Stateful benchmark classes require run()")
    for name in ("setup", "prepare_sample", "run", "validate", "teardown"):
        method = getattr(target, name, None)
        if method is None:
            continue
        parameters = tuple(inspect.signature(method).parameters.values())
        if len(parameters) != 1 or parameters[0].name != "self":
            raise TypeError(f"Benchmark lifecycle method {name}() accepts only self")


def bind_parameters(definition: _BenchmarkDefinition, overrides: dict[str, Any]) -> dict[str, Any]:
    """Merge parameter overrides while preserving declared parameter types.

    Args:
        definition: Benchmark definition containing declared defaults.
        overrides: Parameter values supplied for this run.

    Returns:
        Effective benchmark parameters.

    """
    unknown = set(overrides) - set(definition.parameters)
    if unknown:
        raise ValueError(f"Unsupported benchmark parameters: {sorted(unknown)}")
    effective = {**definition.parameters, **overrides}
    for name, value in overrides.items():
        default = definition.parameters[name]
        if default is None:
            continue
        expected = type(default)
        compatible = isinstance(value, expected)
        if expected is int and isinstance(value, bool):
            compatible = False
        if expected is float and isinstance(value, int) and not isinstance(value, bool):
            compatible = True
        if not compatible:
            raise ValueError(
                f"Invalid type for benchmark parameter {name!r}: "
                f"expected {expected.__name__}, received {type(value).__name__}"
            )
    try:
        inspect.signature(definition.target).bind(**effective)
        json.dumps(effective, sort_keys=True, allow_nan=False)
    except (TypeError, ValueError) as error:
        raise ValueError(f"Invalid parameters for benchmark {definition.identifier}: {error}") from error
    return effective


@contextmanager
def measurement_context(phase: str, sample: int | None, *, retain: bool) -> Iterator[_ActiveMeasurement]:
    """Activate explicit measurements for one runner-owned lifecycle boundary.

    Args:
        phase: Lifecycle phase containing the measurement.
        sample: Sample index, or ``None`` outside sampled execution.
        retain: Whether to retain metrics recorded in this context.

    Yields:
        Active measurement state for the lifecycle boundary.

    """
    state = _ActiveMeasurement(phase=phase, sample=sample, retain=retain)
    token = _ACTIVE_MEASUREMENT.set(state)
    stack_token = _REGION_STACK.set(())
    try:
        yield state
    finally:
        _REGION_STACK.reset(stack_token)
        _ACTIVE_MEASUREMENT.reset(token)


class BenchmarkAuthoring:
    """Facade for declaring, running, and instrumenting benchmarks.

    Use the module-level :data:`benchmark` instance rather than constructing
    this class directly.

    Example:

    .. code-block:: python

        >>> from isaacsim.performance.benchmarking import benchmark

        >>> @benchmark.operation(
        ...     "isaacsim.example.increment",
        ...     warmup_samples=0,
        ...     measured_samples=1,
        ... )
        ... def increment(value: int = 1) -> int:
        ...     return value + 1
        >>> result = benchmark.run(increment, save=False)  # doctest: +NO_CHECK
        >>> result["status"]["valid"]  # doctest: +NO_CHECK
        True

    """

    def operation(
        self,
        id: str,
        *,
        component: str | None = None,
        metrics: tuple[str, ...] = (),
        resources: tuple[str, ...] = ("cpu",),
        duration: str = "short",
        warmup_samples: int = 5,
        measured_samples: int = 30,
        seed: int = 0,
        completion: str = "synchronous",
        simulated_time: float | str | None = None,
    ) -> Callable[[_Target], _Target]:
        """Declare a low-level operation benchmark with calibrated batching.

        Args:
            id: Stable, normalized identifier for the benchmark.
            component: Owning component identifier inferred from ``id`` when omitted.
            metrics: Optional ``gpu``, ``process``, or ``simulation`` metric profiles.
            resources: Resources exercised by the benchmark.
            duration: Expected ``short``, ``medium``, or ``long`` duration class.
            warmup_samples: Number of calibrated batches to execute before measurement.
            measured_samples: Number of independent calibrated batches to retain.
            seed: Random seed applied before benchmark execution.
            completion: Completion boundary described by the measured operation.
            simulated_time: Simulated seconds per operation or the name of a benchmark parameter that supplies it.

        Returns:
            Decorator that records the benchmark definition on a function or class.

        Raises:
            TypeError: If sample counts, the seed, simulated time, or target parameters have invalid types.
            ValueError: If an identifier, option, sample count, metric profile, or simulated time is invalid.

        Example:

        .. code-block:: python

            >>> @benchmark.operation(
            ...     "isaacsim.example.noop",
            ...     warmup_samples=0,
            ...     measured_samples=1,
            ... )
            ... def noop() -> None:
            ...     pass

        """
        return self._decorate(
            id,
            scope="operation",
            component=component,
            metrics=metrics,
            resources=resources,
            duration=duration,
            warmup_samples=warmup_samples,
            measured_samples=measured_samples,
            seed=seed,
            completion=completion,
            simulated_time=simulated_time,
            calibrated_batching=True,
        )

    def subsystem(
        self,
        id: str,
        *,
        component: str | None = None,
        metrics: tuple[str, ...] = (),
        resources: tuple[str, ...] = ("cpu",),
        duration: str = "short",
        warmup_samples: int = 5,
        measured_samples: int = 30,
        seed: int = 0,
        completion: str = "synchronous",
        simulated_time: float | str | None = None,
    ) -> Callable[[_Target], _Target]:
        """Declare a subsystem benchmark without automatic batching.

        Args:
            id: Stable, normalized identifier for the benchmark.
            component: Owning component identifier inferred from ``id`` when omitted.
            metrics: Optional ``gpu``, ``process``, or ``simulation`` metric profiles.
            resources: Resources exercised by the benchmark.
            duration: Expected ``short``, ``medium``, or ``long`` duration class.
            warmup_samples: Number of individual runs to execute before measurement.
            measured_samples: Number of independent runs to retain.
            seed: Random seed applied before benchmark execution.
            completion: Completion boundary described by the measured subsystem.
            simulated_time: Simulated seconds per run or the name of a benchmark parameter that supplies it.

        Returns:
            Decorator that records the benchmark definition on a function or class.

        Raises:
            TypeError: If sample counts, the seed, simulated time, or target parameters have invalid types.
            ValueError: If an identifier, option, sample count, metric profile, or simulated time is invalid.

        Example:

        .. code-block:: python

            >>> @benchmark.subsystem(
            ...     "isaacsim.example.pipeline",
            ...     warmup_samples=0,
            ...     measured_samples=1,
            ... )
            ... def run_pipeline() -> None:
            ...     pass

        """
        return self._decorate(
            id,
            scope="subsystem",
            component=component,
            metrics=metrics,
            resources=resources,
            duration=duration,
            warmup_samples=warmup_samples,
            measured_samples=measured_samples,
            seed=seed,
            completion=completion,
            simulated_time=simulated_time,
            calibrated_batching=False,
        )

    def workflow(
        self,
        id: str,
        *,
        component: str | None = None,
        metrics: tuple[str, ...] = (),
        resources: tuple[str, ...] = ("cpu",),
        duration: str = "medium",
        warmup_frames: int = 5,
        measured_frames: int = 10,
        seed: int = 0,
        completion: str = "synchronous",
        simulated_time: float | str | None = None,
    ) -> Callable[[_Target], _Target]:
        """Declare an end-to-end workflow benchmark where each run is one frame.

        Args:
            id: Stable, normalized identifier for the benchmark.
            component: Owning component identifier inferred from ``id`` when omitted.
            metrics: Optional ``gpu``, ``process``, or ``simulation`` metric profiles.
            resources: Resources exercised by the benchmark.
            duration: Expected ``short``, ``medium``, or ``long`` duration class.
            warmup_frames: Number of frames to execute before measurement.
            measured_frames: Number of independent frames to retain.
            seed: Random seed applied before benchmark execution.
            completion: Completion boundary described by each frame.
            simulated_time: Simulated seconds per frame or the name of a benchmark parameter that supplies it.

        Returns:
            Decorator that records the benchmark definition on a function or class.

        Raises:
            TypeError: If frame counts, the seed, simulated time, or target parameters have invalid types.
            ValueError: If an identifier, option, frame count, metric profile, or simulated time is invalid.

        Example:

        .. code-block:: python

            >>> @benchmark.workflow(
            ...     "isaacsim.example.frame",
            ...     warmup_frames=0,
            ...     measured_frames=1,
            ... )
            ... def render_frame() -> None:
            ...     pass

        """
        return self._decorate(
            id,
            scope="workflow",
            component=component,
            metrics=metrics,
            resources=resources,
            duration=duration,
            warmup_samples=warmup_frames,
            measured_samples=measured_frames,
            seed=seed,
            completion=completion,
            simulated_time=simulated_time,
            calibrated_batching=False,
        )

    def _decorate(
        self,
        id: str,
        *,
        scope: str,
        component: str | None,
        metrics: tuple[str, ...],
        resources: tuple[str, ...],
        duration: str,
        warmup_samples: int,
        measured_samples: int,
        seed: int,
        completion: str,
        simulated_time: float | str | None,
        calibrated_batching: bool,
    ) -> Callable[[_Target], _Target]:
        """Build a scope-specific decorator.

        Args:
            id: Stable normalized benchmark identifier.
            scope: Benchmark scope.
            component: Owning component identifier.
            metrics: Requested metric profiles.
            resources: Resources exercised by the benchmark.
            duration: Expected duration class.
            warmup_samples: Number of warmup samples.
            measured_samples: Number of retained samples.
            seed: Default random seed.
            completion: Measured completion boundary.
            simulated_time: Simulated time value or parameter name.
            calibrated_batching: Whether to calibrate operation batches.

        Returns:
            Decorator that records a benchmark definition on its target.

        """
        if not isinstance(id, str) or not _IDENTIFIER.fullmatch(id):
            raise ValueError(f"Benchmark id must be a normalized lowercase identifier: {id}")
        if component is not None and (not isinstance(component, str) or not component):
            raise ValueError("Benchmark component must be a non-empty string")
        if duration not in _DURATIONS:
            raise ValueError(f"Unsupported benchmark duration: {duration}")
        if completion not in _COMPLETION:
            raise ValueError(f"Unsupported completion description: {completion}")
        if (
            isinstance(warmup_samples, bool)
            or not isinstance(warmup_samples, int)
            or isinstance(measured_samples, bool)
            or not isinstance(measured_samples, int)
        ):
            raise TypeError("Benchmark sample counts must be integers")
        if warmup_samples < 0 or measured_samples < 1:
            raise ValueError("Benchmark sample counts require warmup >= 0 and measured >= 1")
        if isinstance(seed, bool) or not isinstance(seed, int):
            raise TypeError("Benchmark seed must be an integer")
        selected_resources = tuple(dict.fromkeys(resources))
        if not selected_resources or set(selected_resources) - _RESOURCES:
            raise ValueError("Benchmark resources must contain cpu, gpu, or both")
        selected_metrics = tuple(dict.fromkeys(metrics))
        unsupported_metrics = set(selected_metrics) - _METRIC_PROFILES
        if unsupported_metrics:
            raise ValueError(f"Unsupported metric profiles: {sorted(unsupported_metrics)}")
        if "simulation" in selected_metrics and simulated_time is None:
            raise ValueError("The simulation metric profile requires simulated_time")
        if isinstance(simulated_time, (int, float)):
            _validate_simulated_time(simulated_time)
        elif simulated_time is not None and not isinstance(simulated_time, str):
            raise TypeError("simulated_time must be a number or parameter name")

        def decorate(target: _Target) -> _Target:
            if isinstance(target, type):
                _validate_lifecycle(target)
            defaults = _parameter_defaults(target)
            if isinstance(simulated_time, str):
                if simulated_time not in defaults:
                    raise ValueError(f"Simulation-time parameter {simulated_time!r} is unavailable for {id}")
                _validate_simulated_time(defaults[simulated_time])
            definition = _BenchmarkDefinition(
                identifier=id,
                component=component or _infer_component(id),
                scope=scope,
                target=target,
                parameters=defaults,
                resources=selected_resources,
                duration=duration,
                metric_profiles=selected_metrics,
                seed=seed,
                completion=completion,
                simulated_time=simulated_time,
                warmup_samples=warmup_samples,
                measured_samples=measured_samples,
                calibrated_batching=calibrated_batching,
            )
            setattr(target, "__benchmark_definition__", definition)
            return target

        return decorate

    def run(
        self,
        target: _Target,
        parameters: dict[str, Any] | None = None,
        *,
        seed: int | None = None,
        output: str | Path | None = None,
        save: bool = True,
    ) -> dict[str, Any]:
        """Execute the complete benchmark lifecycle and return its result.

        Args:
            target: Function or class decorated by this facade.
            parameters: Overrides for declared parameter defaults.
            seed: Random seed override.
            output: Explicit result file. When omitted, use the configured or temporary output root.
            save: Whether to persist the returned result.

        Returns:
            Structured benchmark result document.

        Raises:
            TypeError: If ``target`` is not decorated or ``save`` is not Boolean.
            ValueError: If ``output`` is provided while result persistence is disabled.
            RuntimeError: If called while an event loop is already running.

        Example:

        .. code-block:: python

            >>> result = benchmark.run(increment, save=False)  # doctest: +NO_CHECK
            >>> result["benchmark"]["id"]  # doctest: +NO_CHECK
            'isaacsim.example.increment'

        """
        definition = self._validate_run_request(target, output, save)
        try:
            asyncio.get_running_loop()
        except RuntimeError:
            pass
        else:
            raise RuntimeError(
                "benchmark.run() cannot be called from a running event loop; use await benchmark.run_async() instead"
            )
        return asyncio.run(self._run_definition(definition, parameters, seed=seed, output=output, save=save))

    async def run_async(
        self,
        target: _Target,
        parameters: dict[str, Any] | None = None,
        *,
        seed: int | None = None,
        output: str | Path | None = None,
        save: bool = True,
    ) -> dict[str, Any]:
        """Execute a benchmark from an application that owns an event loop.

        Args:
            target: Function or class decorated by this facade.
            parameters: Overrides for declared parameter defaults.
            seed: Random seed override.
            output: Explicit result file. When omitted, use the configured or temporary output root.
            save: Whether to persist the returned result.

        Returns:
            Structured benchmark result document.

        Raises:
            TypeError: If ``target`` is not decorated or ``save`` is not Boolean.
            ValueError: If ``output`` is provided while result persistence is disabled.

        Example:

        .. code-block:: python

            >>> result = await benchmark.run_async(increment, save=False)  # doctest: +SKIP
            >>> print(result["status"]["valid"])  # doctest: +SKIP
            True

        """
        definition = self._validate_run_request(target, output, save)
        return await self._run_definition(definition, parameters, seed=seed, output=output, save=save)

    @staticmethod
    def _validate_run_request(target: _Target, output: str | Path | None, save: bool) -> _BenchmarkDefinition:
        """Validate public execution arguments that do not require the runner.

        Args:
            target: Decorated benchmark target.
            output: Explicit output path, if requested.
            save: Whether to persist the result.

        Returns:
            Benchmark definition attached to the target.

        """
        definition = getattr(target, "__benchmark_definition__", None)
        if not isinstance(definition, _BenchmarkDefinition):
            raise TypeError("benchmark execution requires a benchmark-decorated function or class")
        if not isinstance(save, bool):
            raise TypeError("benchmark execution save must be a bool")
        if not save and output is not None:
            raise ValueError("benchmark execution cannot combine output with save=False")
        return definition

    @staticmethod
    async def _run_definition(
        definition: _BenchmarkDefinition,
        parameters: dict[str, Any] | None,
        *,
        seed: int | None,
        output: str | Path | None,
        save: bool,
    ) -> dict[str, Any]:
        """Run and report one validated benchmark request.

        Args:
            definition: Validated benchmark definition.
            parameters: Parameter overrides for this run.
            seed: Random seed override.
            output: Explicit output path, if requested.
            save: Whether to persist the result.

        Returns:
            Structured benchmark result document.

        """
        from isaacsim.performance.benchmarking.impl.reporting import format_summary, resolve_output_path, write_result
        from isaacsim.performance.benchmarking.impl.runner import run_definition
        from isaacsim.performance.benchmarking.impl.schema import result_document

        result = await run_definition(definition, parameters, seed=seed)
        output_path = resolve_output_path(result, output) if save else None
        document = write_result(result, output_path) if output_path is not None else result_document(result)
        print(format_summary(document))
        if output_path is not None:
            print(f"Result: {output_path}")
        return document

    @contextmanager
    def measure(self, name: str) -> Iterator[None]:
        """Measure a named block when called inside an active benchmark run.

        Calls outside an active retained sample are no-ops. Nested measurement
        names are joined with ``/`` in the result document.

        Args:
            name: Region name containing letters, digits, dots, underscores, hyphens, or slashes.

        Yields:
            Control to the measured block.

        Raises:
            ValueError: If ``name`` is invalid during an active retained sample.

        Example:

        .. code-block:: python

            >>> with benchmark.measure("update/physics"):
            ...     pass

        """
        state = _ACTIVE_MEASUREMENT.get()
        if state is None or not state.retain:
            yield
            return
        if not _MEASUREMENT_NAME.fullmatch(name):
            raise ValueError(f"Invalid benchmark measurement name: {name}")
        stack = _REGION_STACK.get()
        path = "/".join((*stack, name))
        token = _REGION_STACK.set((*stack, name))
        started_ns = time.perf_counter_ns()
        try:
            yield
        finally:
            elapsed_ms = (time.perf_counter_ns() - started_ns) / 1_000_000.0
            _REGION_STACK.reset(token)
            state.region_values.setdefault(path, []).append(elapsed_ms)

    def record(self, name: str, value: float, unit: str) -> None:
        """Record one numeric value for the active parent sample.

        Calls outside an active measured sample are no-ops. Repeated calls with
        the same name are averaged for that sample and must use the same unit.

        Args:
            name: Metric name containing letters, digits, dots, underscores, hyphens, or slashes.
            value: Finite numeric value to record.
            unit: Nonempty unit label for the value.

        Raises:
            TypeError: If ``value`` is not numeric.
            ValueError: If the name, value, or unit is invalid, or the unit changes within a sample.

        Example:

        .. code-block:: python

            >>> benchmark.record("active_bodies", 24, "count")

        """
        state = _ACTIVE_MEASUREMENT.get()
        if state is None or not state.retain or state.sample is None:
            return
        if not _MEASUREMENT_NAME.fullmatch(name):
            raise ValueError(f"Invalid benchmark metric name: {name}")
        if not unit or not unit.strip():
            raise ValueError("Benchmark metric units cannot be empty")
        if isinstance(value, bool) or not isinstance(value, (int, float)):
            raise TypeError("Benchmark metric values must be numeric")
        numeric_value = float(value)
        if not math.isfinite(numeric_value):
            raise ValueError("Benchmark metric values must be finite")
        recorded = state.recorded_values.get(name)
        if recorded is not None:
            values, previous_unit = recorded
            if previous_unit != unit:
                raise ValueError(f"Benchmark metric {name!r} changed units from {previous_unit!r} to {unit!r}")
            values.append(numeric_value)
            return
        state.recorded_values[name] = ([numeric_value], unit)


benchmark = BenchmarkAuthoring()

__all__ = ["benchmark"]
