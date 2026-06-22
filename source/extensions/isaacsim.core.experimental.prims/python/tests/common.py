# SPDX-FileCopyrightText: Copyright (c) 2021-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

"""Common test utilities."""

from __future__ import annotations

import os
from collections.abc import Callable
from typing import Any, Literal

import carb
import numpy as np
import omni.kit.test
import warp as wp
from isaacsim.core.simulation_manager import SimulationManager


def cprint(message: str) -> None:
    """Print a message when verbose test logging is enabled.

    Args:
        message: Message to print when verbose test logging is enabled.
    """
    if os.environ.get("ISAACSIM_TEST_VERBOSE", "0").lower() in ["1", "true", "yes"]:
        print(message)


# simple decorator to skip test if default engine is not in supported engines
def requires_engines(supported_engines: list[Literal["physx", "newton"]] = ["physx", "newton"]) -> Callable:
    """Requires engines.

    Args:
        supported_engines: Physics engines supported by the decorated test.

    Returns:
        Decorator that skips tests when the active engine is unsupported.
    """

    def decorator(func: Callable) -> Callable:
        async def wrapper(self: Any, *args: Any, **kwargs: Any) -> Any:
            default_engine = SimulationManager.get_default_engine()
            if default_engine and default_engine.lower() not in supported_engines:
                cprint(f"  |-- Skipping test: engine '{default_engine}' not in supported engines {supported_engines}")
                return None
            return await func(self, *args, **kwargs)

        return wrapper

    return decorator


def parametrize(
    *,
    devices: list[Literal["cpu", "cuda"]] = ["cpu", "cuda"],
    backends: list[Literal["usd", "usdrt", "fabric", "tensor"]] = ["usd", "usdrt", "fabric", "tensor"],
    instances: list[Literal["one", "many"]] = ["one", "many"],
    operations: list[Literal["wrap", "create"]] = ["wrap", "create"],
    supported_engines: list[Literal["physx", "newton"]] = ["physx", "newton"],
    prim_class: type,
    prim_class_kwargs: dict = {},
    populate_stage_func: Callable[[int, Literal["wrap", "create"]], None],
    populate_stage_func_kwargs: dict = {},
    max_num_prims: int = 5,
) -> Callable:
    """Parametrize.

    Args:
        devices: Devices to include in the generated test cases.
        backends: Backends to include in the generated test cases.
        instances: Instance modes to include in the generated test cases.
        operations: Stage population operations to include in the generated test cases.
        supported_engines: Physics engines supported by the decorated test.
        prim_class: Prim wrapper class used by generated test cases.
        prim_class_kwargs: Keyword arguments passed to the prim wrapper class.
        populate_stage_func: Callable used to populate the test stage.
        populate_stage_func_kwargs: Keyword arguments passed to the stage population callable.
        max_num_prims: Maximum number of prims to create for a test case.

    Returns:
        Decorator that runs a test over the requested prim configurations.
    """

    def decorator(func: Callable) -> Callable:
        async def wrapper(self: Any) -> None:
            # Switch to the default engine if specified in settings
            default_engine = SimulationManager.get_default_engine()
            if default_engine:
                if default_engine.lower() not in supported_engines:
                    cprint(
                        f"  |-- Skipping test: engine '{default_engine}' not in supported engines {supported_engines}"
                    )
                    return
                current_engine = SimulationManager.get_active_physics_engine()
                if current_engine != default_engine.lower():
                    success = SimulationManager.switch_physics_engine(default_engine)
                    if not success:
                        carb.log_warn(f"Failed to switch to {default_engine} engine")
                    await omni.kit.app.get_app().next_update_async()
                cprint(f"  |-- Testing with engine: {default_engine}")
            current_engine = SimulationManager.get_active_physics_engine()
            for device in devices:
                for backend in backends:
                    for instance in instances:
                        for operation in operations:
                            assert backend in ["usd", "usdrt", "fabric", "tensor"], f"Invalid backend: {backend}"
                            assert instance in ["one", "many"], f"Invalid instance: {instance}"
                            assert operation in ["wrap", "create"], f"Invalid operation: {operation}"
                            cprint(
                                f"      |-- engine: {current_engine} |-- device: {device}, backend: {backend}, instance: {instance}, operation: {operation}"
                            )
                            # populate stage
                            await populate_stage_func(max_num_prims, operation, **populate_stage_func_kwargs)
                            # configure simulation manager
                            SimulationManager.set_physics_sim_device(device)
                            # parametrize test
                            if operation == "wrap":
                                paths = "/World/A_0" if instance == "one" else "/World/A_.*"
                            elif operation == "create":
                                paths = (
                                    "/World/A_0"
                                    if instance == "one"
                                    else [f"/World/A_{i}" for i in range(max_num_prims)]
                                )
                            prim = prim_class(paths, **prim_class_kwargs)
                            num_prims = 1 if instance == "one" else max_num_prims
                            # call test method according to backend
                            if backend == "tensor":
                                omni.timeline.get_timeline_interface().play()
                                await omni.kit.app.get_app().next_update_async()
                                await func(self, prim=prim, num_prims=num_prims, device=device, backend=backend)
                                omni.timeline.get_timeline_interface().stop()
                            elif backend in ["usd", "usdrt", "fabric"]:
                                await func(self, prim=prim, num_prims=num_prims, device=device, backend=backend)

        return wrapper

    return decorator


def check_array(
    a: wp.array | list[wp.array],
    shape: list[int] | None = None,
    dtype: type | None = None,
    device: str | wp.Device | None = None,
) -> None:
    """Check array.

    Args:
        a: Value to check.
        shape: Expected array shape.
        dtype: Expected data type.
        device: Device under test.
    """
    for i, x in enumerate(a if isinstance(a, (list, tuple)) else [a]):
        assert isinstance(x, wp.array), f"[{i}]: {repr(x)} ({type(x)}) is not a Warp array"
        if shape is not None:
            assert tuple(x.shape) == tuple(shape), f"[{i}]: Unexpected shape: expected {shape}, got {x.shape}"
        if dtype is not None:
            assert x.dtype == dtype, f"[{i}]: Unexpected dtype: expected {dtype}, got {x.dtype}"
        if device is not None:
            assert x.device == wp.get_device(device), f"[{i}]: Unexpected device: expected {device}, got {x.device}"


def check_lists(
    a: list, b: list, *, check_value: bool = True, check_type: bool = True, predicate: callable = None
) -> None:
    """Check lists.

    Args:
        a: Value to check.
        b: Expected value to compare against.
        check_value: Value passed by the caller.
        check_type: Value passed by the caller.
        predicate: Optional transform applied before comparing values.
    """
    assert len(a) == len(b), f"Unexpected length: expected {len(a)}, got {len(b)}"
    for x, y in zip(a, b):
        if check_value:
            if predicate is not None:
                x = predicate(x)
                y = predicate(y)
            assert x == y, f"Unexpected value: expected {x}, got {y}"
        if check_type:
            assert type(x) == type(y), f"Unexpected type: expected {type(x)}, got {type(y)}"


def check_equal(
    a: wp.array | np.ndarray | list[wp.array] | list[np.ndarray],
    b: wp.array | np.ndarray | list[wp.array] | list[np.ndarray],
    *,
    given: list | None = None,
) -> None:
    """Check equal.

    Args:
        a: Value to check.
        b: Expected value to compare against.
        given: Original input values used to produce the compared values.
    """
    msg = ""
    a = a if isinstance(a, (list, tuple)) else [a]
    b = b if isinstance(b, (list, tuple)) else [b]
    assert len(a) == len(b), f"Unexpected (check) length: {len(a)} != {len(b)}"
    for i, (x, y) in enumerate(zip(a, b)):
        if isinstance(x, wp.array):
            x = x.numpy()
        if isinstance(y, wp.array):
            y = y.numpy()
        if given is not None:
            msg = f"\nGiven ({type(given[i])})\n{given[i]}"
        assert x.shape == y.shape, f"[{i}]: Unexpected shape: expected {x.shape}, got {y.shape}{msg}"
        assert (x == y).all(), f"[{i}]: Unexpected value:\nExpected\n{x}\nGot\n{y}{msg}"


def check_allclose(
    a: wp.array | np.ndarray | list[wp.array] | list[np.ndarray],
    b: wp.array | np.ndarray | list[wp.array] | list[np.ndarray],
    *,
    rtol: float = 1e-03,
    atol: float = 1e-05,
    given: list | None = None,
) -> None:
    """Check allclose.

    Args:
        a: Value to check.
        b: Expected value to compare against.
        rtol: Relative tolerance for value comparisons.
        atol: Absolute tolerance for value comparisons.
        given: Original input values used to produce the compared values.
    """
    msg = ""
    a = a if isinstance(a, (list, tuple)) else [a]
    b = b if isinstance(b, (list, tuple)) else [b]
    assert len(a) == len(b), f"Unexpected (check) length: {len(a)} != {len(b)}"
    for i, (x, y) in enumerate(zip(a, b)):
        if isinstance(x, wp.array):
            x = x.numpy()
        if isinstance(y, wp.array):
            y = y.numpy()
        if given is not None:
            msg = f"\nGiven ({type(given[i])})\n{given[i]}"
        assert x.shape == y.shape, f"[{i}]: Unexpected shape: expected {x.shape}, got {y.shape}{msg}"
        assert np.allclose(
            x, y, rtol=rtol, atol=atol
        ), f"[{i}]: Unexpected value (within tolerance):\nExpected\n{x}\nGot\n{y}{msg}"


def draw_sample(
    *,
    shape: tuple,
    dtype: type,
    types: list = [list, np.ndarray, wp.array],
    low: int | float = 0.0,
    high: int | float = 1.0,
    normalized: bool = False,
    transform: callable = None,
) -> list:
    """Draw sample.

    Args:
        shape: Expected array shape.
        dtype: Expected data type.
        types: Container types to generate.
        low: Lower bound for generated samples.
        high: Upper bound for generated samples.
        normalized: Whether to normalize generated samples.
        transform: Optional transform applied to generated samples.

    Returns:
        Generated samples for each requested container type.
    """
    samples = []
    for _type in types:
        # sample according to dtype
        if dtype is wp.bool:
            sample = np.random.choice([True, False], size=shape).astype(np.bool_)
            basic_type = bool(sample.flatten()[0])
        elif dtype is wp.int32:
            sample = np.random.randint(low=low, high=high, size=shape).astype(np.int32)
            basic_type = int(sample.flatten()[0])
        elif dtype is wp.uint32:
            sample = np.random.randint(low=low, high=high, size=shape).astype(np.uint32)
            basic_type = int(sample.flatten()[0])
        elif dtype is wp.float32:
            sample = np.random.uniform(low=low, high=high, size=shape).astype(np.float32)
            if normalized:
                sample = (sample / np.linalg.norm(sample, axis=-1, keepdims=True)).astype(np.float32)
            basic_type = float(sample.flatten()[0])
        else:
            raise ValueError(f"Invalid dtype: {dtype}")
        # apply transform if provided
        if transform:
            sample = transform(sample)
        # create single sample and broadcasted sample
        if sample.ndim == 2:
            single_sample = sample[[0]]
        elif sample.ndim == 3:
            single_sample = sample[[0], [0]]
        else:
            raise ValueError(f"Unsupported dimensionality: {sample.ndim}")
        single_sample_broadcasted = np.broadcast_to(single_sample, shape)
        single_sample_broadcasted_basic = np.broadcast_to(single_sample.flatten()[:1], shape)
        # create samples
        if _type is list:
            samples.append((single_sample.reshape((shape[-1],)).tolist(), single_sample_broadcasted))
            samples.append((single_sample.reshape((1, shape[-1])).tolist(), single_sample_broadcasted))
            samples.append((sample.tolist(), sample))
        elif _type is np.ndarray:
            samples.append((single_sample.reshape((shape[-1],)).copy(), single_sample_broadcasted))
            samples.append((single_sample.reshape((1, shape[-1])).copy(), single_sample_broadcasted))
            samples.append((sample.copy(), sample))
        elif _type is wp.array:
            samples.append((wp.array(single_sample.reshape((shape[-1],)).copy()), single_sample_broadcasted))
            samples.append((wp.array(single_sample.reshape((1, shape[-1])).copy()), single_sample_broadcasted))
            samples.append((wp.array(sample.copy()), sample))
        else:
            raise ValueError(f"Invalid type: {_type}")
    if shape[-1] == 1:
        samples.append((basic_type, single_sample_broadcasted_basic))
    return samples


def draw_choice(*, shape: tuple, choices: list) -> list:
    """Draw choice.

    Args:
        shape: Expected array shape.
        choices: Value passed by the caller.

    Returns:
        Requested value.
    """
    sample = np.random.choice(np.array(choices, dtype=object).flatten(), size=shape)
    # create single sample and broadcasted sample
    if sample.ndim == 1:
        single_sample = sample[[0]]
    elif sample.ndim == 2:
        single_sample = sample[[0]]
    else:
        raise ValueError(f"Unsupported dimensionality: {sample.ndim}")
    single_sample_broadcasted = np.broadcast_to(single_sample, shape)
    # create samples
    samples = [
        (single_sample.flatten().tolist(), single_sample_broadcasted.tolist()),
        (single_sample.tolist(), single_sample_broadcasted.tolist()),
        (sample.tolist(), sample.tolist()),
    ]
    return samples


def draw_indices(*, count: int, step: int = 2, types: list = [list, np.ndarray, wp.array, None]) -> list:
    """Draw indices.

    Args:
        count: Expected number of contact records.
        step: Value passed by the caller.
        types: Container types to generate.

    Returns:
        Requested value.
    """
    indices = list(range(0, count, step))
    indices_list = []
    for _type in types:
        if _type is list:
            indices_list.append((indices[:], len(indices)))
        elif _type is np.ndarray:
            indices_list.append((np.array(indices), len(indices)))
        elif _type is wp.array:
            indices_list.append((wp.array(indices, dtype=wp.int32, device="cpu"), len(indices)))
        elif _type is None:
            indices_list.append((None, count))
    return indices_list
