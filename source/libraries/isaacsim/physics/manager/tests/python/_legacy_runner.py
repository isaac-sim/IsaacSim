# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
# http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Run scenario tests across registered backends and device configurations.

This module centralizes scenario instantiation, backend lifecycle management,
CPU/GPU selection, and explicit expected failures for unfinished scenarios.
"""

from __future__ import annotations

import os
import sys
from collections.abc import Callable, Sequence
from functools import partial
from types import TracebackType
from typing import TypeVar

import pytest

# Make tests/python/tensors/ importable regardless of pytest's CWD.
_TENSORS_DIR = os.path.dirname(os.path.abspath(__file__))
if _TENSORS_DIR not in sys.path:
    sys.path.append(_TENSORS_DIR)

from _scenario import DeviceParams, RunnerInMemory, ScenarioBase, SimulationEntities  # noqa: E402

TestClass = TypeVar("TestClass", bound=type[object])
_ScenarioOutcome = tuple[BaseException, TracebackType | None] | None
_SCENARIO_BATCHES: dict[tuple[str, type[ScenarioBase]], tuple[type[ScenarioBase], ...]] = {}
_SCENARIO_BATCH_RESULTS: dict[
    tuple[str, str, bool, bool, int | None, tuple[type[ScenarioBase], ...]],
    dict[type[ScenarioBase], _ScenarioOutcome],
] = {}


def cpu_device() -> DeviceParams:
    """Select CPU simulation and tensor execution.

    Returns:
        Device parameters for an all-CPU scenario.

    """
    return DeviceParams(use_gpu_sim=False, use_gpu_pipeline=False)


def gpu_device() -> DeviceParams:
    """Select GPU simulation and tensor execution.

    Returns:
        Device parameters for an all-GPU scenario.

    """
    return DeviceParams(use_gpu_sim=True, use_gpu_pipeline=True)


def register_scenario_batches(
    engine: str,
    batches: Sequence[Sequence[type[ScenarioBase]]],
) -> None:
    """Register compatible scenarios that can share one backend lifecycle.

    Args:
        engine: Registered physics engine name.
        batches: Groups of scenarios with identical stages and warmup needs.

    Raises:
        ValueError: If a batch is too small or registers one scenario more than once.

    """
    for scenarios in batches:
        batch = tuple(scenarios)
        if len(batch) < 2:
            raise ValueError("A scenario batch must contain at least two scenarios")
        for scenario_cls in batch:
            key = (engine, scenario_cls)
            existing = _SCENARIO_BATCHES.get(key)
            if existing is not None and existing != batch:
                raise ValueError(f"Scenario is registered in multiple {engine} batches: {scenario_cls.__name__}")
            _SCENARIO_BATCHES[key] = batch


def _capture_scenario_outcome(operation: Callable[[], object]) -> _ScenarioOutcome:
    """Run one scenario operation and retain a later-reportable outcome.

    Args:
        operation: Zero-argument callable to execute.

    Returns:
        Captured exception and traceback, or None after success.

    """
    try:
        operation()
    except (KeyboardInterrupt, SystemExit):
        raise
    except BaseException as error:
        return error, error.__traceback__
    return None


def _run_scenario_check(
    test_case: object,
    scenario_cls: type[ScenarioBase],
    device: DeviceParams,
    simulation: SimulationEntities,
) -> None:
    """Initialize and check one scenario against an already-warmed simulation.

    Args:
        test_case: Test instance supplied to the scenario.
        scenario_cls: Scenario class whose assertions should run.
        device: Simulation and tensor device selection.
        simulation: Entity-view factory for the attached backend.

    """
    scenario = scenario_cls(test_case, device)
    scenario.on_start(simulation)
    scenario.check_cell()


def _run_scenario_batch(
    test_case: object,
    scenario_classes: tuple[type[ScenarioBase], ...],
    engine: str,
    device: DeviceParams,
    frontend: str,
) -> dict[type[ScenarioBase], _ScenarioOutcome]:
    """Run compatible scenario checks through one attached backend.

    Args:
        test_case: Test instance supplied to the scenarios.
        scenario_classes: Scenario classes sharing a stage shape and warmup.
        engine: Registered physics engine name.
        device: Simulation and tensor device selection.
        frontend: Tensor frontend name.

    Returns:
        Captured outcome for every scenario class.

    """
    outcomes: dict[type[ScenarioBase], _ScenarioOutcome] = {}
    first_class = scenario_classes[0]
    first_scenario = first_class(test_case, device)
    runner = RunnerInMemory(first_scenario, engine=engine, frontend=frontend)

    start_outcome = _capture_scenario_outcome(runner.start)
    if start_outcome is not None:
        _capture_scenario_outcome(runner.stop)
        return dict.fromkeys(scenario_classes, start_outcome)

    try:
        outcomes[first_class] = _capture_scenario_outcome(runner.simulate)
        for scenario_cls in scenario_classes[1:]:
            outcomes[scenario_cls] = _capture_scenario_outcome(
                partial(
                    _run_scenario_check,
                    test_case,
                    scenario_cls,
                    device,
                    first_scenario.sim,
                )
            )
    finally:
        stop_outcome = _capture_scenario_outcome(runner.stop)

    if stop_outcome is not None:
        first_success = next((scenario_cls for scenario_cls, outcome in outcomes.items() if outcome is None), None)
        if first_success is not None:
            outcomes[first_success] = stop_outcome
    return outcomes


def run_scenario(
    test_case: object,
    scenario_cls: type[ScenarioBase],
    engine: str,
    device: DeviceParams,
    frontend: str = "warp",
) -> None:
    """Run a scenario against one engine, device, and tensor frontend.

    CPU and GPU variants can run in the same pytest process. GPU test methods
    use :data:`gpu_only` to skip only when no CUDA device is available.

    Args:
        test_case: Test instance supplied to the scenario.
        scenario_cls: Scenario class to instantiate and run.
        engine: Registered physics engine name.
        device: Simulation and tensor device selection.
        frontend: Tensor frontend name.

    """
    batch = _SCENARIO_BATCHES.get((engine, scenario_cls))
    if batch is not None:
        batch_key = (
            engine,
            frontend,
            device.use_gpu_sim,
            device.use_gpu_pipeline,
            device.num_workers,
            batch,
        )
        outcomes = _SCENARIO_BATCH_RESULTS.get(batch_key)
        if outcomes is None:
            outcomes = _run_scenario_batch(test_case, batch, engine, device, frontend)
            _SCENARIO_BATCH_RESULTS[batch_key] = outcomes
        outcome = outcomes[scenario_cls]
        if outcome is None:
            return
        error, traceback = outcome
        raise error.with_traceback(traceback)

    scenario = scenario_cls(test_case, device)
    runner = RunnerInMemory(scenario, engine=engine, frontend=frontend)
    exc_info = None
    try:
        runner.start()
        runner.simulate()
    except Exception:
        exc_info = sys.exc_info()
    finally:
        try:
            runner.stop()
        except Exception:
            if exc_info is None:
                exc_info = sys.exc_info()
    if exc_info is None:
        return

    exc = exc_info[1]
    raise exc.with_traceback(exc_info[2])


# `@gpu_only` skips when no CUDA device is visible. ovphysx 0.5+ allows
# per-instance device pick, so CPU and GPU variants coexist in a single
# pytest invocation — the only reason a GPU variant is still skipped is
# a CPU-only host (no CUDA).
def _cuda_available() -> bool:
    # Gate on warp, the test frontend and a hard dependency, rather than
    # torch. torch may be absent on a GPU host (e.g. a misconfigured CI
    # container); keying off it would mask a real GPU as CPU-only and
    # silently skip every GPU variant.
    try:
        import warp as wp
    except ImportError:
        return False
    try:
        return wp.get_cuda_device_count() > 0
    except Exception:
        return False


gpu_only = pytest.mark.skipif(not _cuda_available(), reason="no CUDA device available")


def unimplemented_placeholder(test_class: TestClass) -> TestClass:
    """Mark every placeholder test in a class as an expected failure.

    These methods contain no runnable scenario yet. Reporting them as skipped
    would make an unfinished scenario look like an engine limitation. Replace
    each generated expected failure with a real scenario when that test lands.

    Args:
        test_class: Class whose test methods are unimplemented placeholders.

    Returns:
        The decorated test class.

    """
    for name, method in list(vars(test_class).items()):
        if not name.startswith("test") or not callable(method):
            continue

        def fail_unported_test(self: object, test_name: str = name) -> None:
            """Fail one unported test with its original qualified name.

            Args:
                self: Placeholder test instance.
                test_name: Original test method name.

            """
            pytest.fail(f"{test_class.__name__}.{test_name} is an unported test placeholder")

        fail_unported_test.__name__ = method.__name__
        fail_unported_test.__doc__ = method.__doc__
        setattr(test_class, name, pytest.mark.xfail(reason="unported test placeholder")(fail_unported_test))
    return test_class
