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

"""Benchmark per-joint, grouped, and USD-built articulation actuators.

The three cases use the same Franka Panda asset, the same seven arm joints, and
the same per-joint PD gains:

* ``python_baseline``: seven ``ActuatorConfig`` objects, each targeting one joint.
* ``python_grouped``: one ``ActuatorConfig`` targeting all seven joints.
* ``usd_model_builder``: seven USD actuator prims parsed and grouped by Newton's
  ``ModelBuilder``.

Construction and steady-state ``step_actuators()`` timings are reported
separately. The step benchmark intentionally excludes physics solving and
rendering so it measures the actuator bridge code directly.

Run from the Isaac Sim release directory, for example::

    ./python.sh ../../../source/standalone_examples/benchmarks/benchmark_articulation_actuator_grouping.py \
        --num-robots 32 --steps 1000 --trials 7 --output /tmp/actuator_grouping.json
"""

from __future__ import annotations

import argparse
import gc
import json
import math
import statistics
import time
from collections.abc import Callable
from pathlib import Path
from typing import Any

parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument("--num-robots", type=int, default=32, help="Number of Franka instances to control.")
parser.add_argument("--steps", type=int, default=1000, help="Actuator steps per timing trial.")
parser.add_argument("--trials", type=int, default=7, help="Number of steady-state timing trials.")
parser.add_argument("--warmup-steps", type=int, default=50, help="Untimed actuator steps before measurement.")
parser.add_argument(
    "--construction-repeats",
    type=int,
    default=10,
    help="Number of measured construction repetitions per case.",
)
parser.add_argument(
    "--device",
    default=None,
    help="Device for both physics state and actuator work. Defaults to Warp's current device.",
)
parser.add_argument("--output", type=Path, default=None, help="Optional JSON output path.")
args, _ = parser.parse_known_args()

for name in ("num_robots", "steps", "trials", "warmup_steps", "construction_repeats"):
    if getattr(args, name) < 1:
        raise ValueError(f"--{name.replace('_', '-')} must be positive.")

from isaacsim import SimulationApp

simulation_app = SimulationApp({"headless": True})

import isaacsim.core.experimental.utils.stage as stage_utils
import omni.kit.app
import omni.timeline
import warp as wp
from isaacsim.core.experimental.actuators import (
    ActuatorConfig,
    ArticulationActuators,
    PDControlConfig,
    add_actuator,
)
from isaacsim.core.simulation_manager import SimulationManager
from isaacsim.storage.native import get_assets_root_path_async
from newton.actuators import DrivePD
from pxr import Gf, UsdGeom

FRANKA_USD_REL_PATH = "Isaac/Robots_Multiphysics/FrankaRobotics/FrankaPanda/franka/franka.usda"
FRANKA_BASE_PATH = "/World/Franka"
FRANKA_PATH_EXPRESSION = f"{FRANKA_BASE_PATH}_.*"
ARM_JOINTS = [f"panda_joint{index}" for index in range(1, 8)]
KP_GAINS = [67.0, 66.0, 65.0, 64.0, 63.0, 62.0, 61.0]
KD_GAINS = [8.0] * len(ARM_JOINTS)
TARGET_POSITIONS = [0.8, -1.3, 0.0, -2.87, 0.0, 2.0, 0.75]
STEP_DT = 1.0 / 60.0

Factory = Callable[[], ArticulationActuators]


async def setup_stage(num_robots: int) -> None:
    """Create a stage containing repeated references to the existing Franka asset.

    Args:
        num_robots: Number of Franka references to add.

    Example:

    .. code-block:: python

        >>> simulation_app.run_coroutine(setup_stage(num_robots=4))
    """
    await stage_utils.create_new_stage_async()
    stage_utils.define_prim("/World", "Xform")
    stage_utils.define_prim("/World/PhysicsScene", "PhysicsScene")

    assets_root = await get_assets_root_path_async()
    franka_usd = f"{assets_root}/{FRANKA_USD_REL_PATH}"
    grid_width = math.ceil(math.sqrt(num_robots))
    for robot_index in range(num_robots):
        prim = stage_utils.add_reference_to_stage(franka_usd, f"{FRANKA_BASE_PATH}_{robot_index}")
        xformable = UsdGeom.Xformable(prim)
        translate_ops = [op for op in xformable.GetOrderedXformOps() if op.GetOpType() == UsdGeom.XformOp.TypeTranslate]
        translate_op = translate_ops[0] if translate_ops else xformable.AddTranslateOp()
        translate_op.Set(Gf.Vec3d(2.0 * (robot_index % grid_width), 2.0 * (robot_index // grid_width), 0.0))
    await omni.kit.app.get_app().next_update_async()


def author_usd_actuators() -> None:
    """Author seven scalar PD actuator recipes on the first Franka template.

    Example:

    .. code-block:: python

        >>> author_usd_actuators()
    """
    first_robot_path = f"{FRANKA_BASE_PATH}_0"
    for joint_name, kp, kd in zip(ARM_JOINTS, KP_GAINS, KD_GAINS, strict=True):
        add_actuator(
            first_robot_path,
            target_names=joint_name,
            name=f"{joint_name}_benchmark_actuator",
            controller=PDControlConfig(kp=kp, kd=kd),
        )


def build_baseline_configs(num_robots: int, device: wp.DeviceLike) -> list[tuple[ActuatorConfig, list[str]]]:
    """Build one Python actuator configuration per joint.

    Args:
        num_robots: Number of articulation instances.
        device: Device for component arrays.

    Returns:
        Seven per-joint actuator groups.

    Example:

    .. code-block:: python

        >>> configs = build_baseline_configs(num_robots=4, device=wp.get_device("cpu"))
        >>> len(configs)
        7
    """
    return [
        (
            ActuatorConfig(
                controller=DrivePD(
                    kp=wp.full(num_robots, kp, dtype=wp.float32, device=device),
                    kd=wp.full(num_robots, kd, dtype=wp.float32, device=device),
                )
            ),
            [joint_name],
        )
        for joint_name, kp, kd in zip(ARM_JOINTS, KP_GAINS, KD_GAINS, strict=True)
    ]


def build_grouped_config(num_robots: int, device: wp.DeviceLike) -> list[tuple[ActuatorConfig, list[str]]]:
    """Build one Python actuator configuration for all arm joints.

    Args:
        num_robots: Number of articulation instances.
        device: Device for component arrays.

    Returns:
        One seven-joint actuator group with robot-major component arrays.

    Example:

    .. code-block:: python

        >>> configs = build_grouped_config(num_robots=4, device=wp.get_device("cpu"))
        >>> len(configs)
        1
    """
    config = ActuatorConfig(
        controller=DrivePD(
            kp=wp.array(KP_GAINS * num_robots, dtype=wp.float32, device=device),
            kd=wp.array(KD_GAINS * num_robots, dtype=wp.float32, device=device),
        )
    )
    return [(config, ARM_JOINTS)]


def synchronize(device: wp.DeviceLike) -> None:
    """Wait for all queued Warp work on a device.

    Args:
        device: Device whose work must complete.

    Example:

    .. code-block:: python

        >>> synchronize(wp.get_device("cpu"))
    """
    wp.synchronize_device(device)


def summarize(samples: list[float]) -> dict[str, float]:
    """Summarize timing samples.

    Args:
        samples: Timing values in a common unit.

    Returns:
        Basic distribution statistics in the input unit.

    Example:

    .. code-block:: python

        >>> summarize([1.0, 2.0, 3.0])["mean"]
        2.0
    """
    ordered = sorted(samples)
    percentile_index = min(len(ordered) - 1, round(0.95 * (len(ordered) - 1)))
    return {
        "median": statistics.median(samples),
        "mean": statistics.fmean(samples),
        "min": ordered[0],
        "p95": ordered[percentile_index],
        "stdev": statistics.stdev(samples) if len(samples) > 1 else 0.0,
    }


def benchmark_construction(factory: Factory, device: wp.DeviceLike, repeats: int) -> dict[str, float]:
    """Measure warmed-up wrapper construction latency.

    Args:
        factory: Function that creates a fresh actuator wrapper.
        device: Device used by the wrapper.
        repeats: Number of measured constructions.

    Returns:
        Construction-time statistics in milliseconds.

    Example:

    .. code-block:: python

        >>> stats = benchmark_construction(factory, device, repeats=10)
        >>> stats["mean"] >= 0.0
        True
    """
    warmup = factory()
    synchronize(device)
    warmup.close()
    del warmup
    gc.collect()

    samples_ms: list[float] = []
    for _ in range(repeats):
        synchronize(device)
        start_ns = time.perf_counter_ns()
        actuated = factory()
        synchronize(device)
        samples_ms.append((time.perf_counter_ns() - start_ns) / 1.0e6)
        actuated.close()
        del actuated
        gc.collect()
    return summarize(samples_ms)


def benchmark_steps(
    factory: Factory,
    device: wp.DeviceLike,
    *,
    warmup_steps: int,
    steps: int,
    trials: int,
) -> tuple[dict[str, float], int, int]:
    """Measure steady-state ``step_actuators`` throughput.

    Args:
        factory: Function that creates a fresh actuator wrapper.
        device: Device used by the wrapper.
        warmup_steps: Number of untimed calls used to warm kernels and caches.
        steps: Calls measured in each trial.
        trials: Number of timing trials.

    Returns:
        Per-step statistics in microseconds, actuator batch count, and physical motor count.

    Example:

    .. code-block:: python

        >>> stats, batches, motors = benchmark_steps(
        ...     factory, device, warmup_steps=50, steps=1000, trials=7
        ... )
        >>> stats["mean"] >= 0.0 and batches > 0 and motors > 0
        True
    """
    actuated = factory()
    try:
        arm_indices = actuated.articulation.get_dof_indices(ARM_JOINTS)
        actuated.articulation.set_dof_position_targets(TARGET_POSITIONS, dof_indices=arm_indices)

        for _ in range(warmup_steps):
            actuated.step_actuators(STEP_DT)
        synchronize(device)

        samples_us: list[float] = []
        for _ in range(trials):
            synchronize(device)
            start_ns = time.perf_counter_ns()
            for _ in range(steps):
                actuated.step_actuators(STEP_DT)
            synchronize(device)
            elapsed_ns = time.perf_counter_ns() - start_ns
            samples_us.append(elapsed_ns / steps / 1.0e3)

        actuator_batches = len(actuated.actuators)
        motor_count = sum(actuator.num_actuators for actuator in actuated.actuators)
        return summarize(samples_us), actuator_batches, motor_count
    finally:
        actuated.close()


def print_results(results: dict[str, Any]) -> None:
    """Print a compact timing table and relative step speedups.

    Args:
        results: Complete benchmark result dictionary.

    Example:

    .. code-block:: python

        >>> print_results(results)
    """
    print("\nActuator grouping benchmark")
    print(
        f"Actuator device: {results['actuator_device']} | Physics device: {results['physics_device']} | "
        f"Robots: {results['num_robots']} | "
        f"Steps/trial: {results['steps']} | Trials: {results['trials']}"
    )
    print("\nCase                 Batches  Motors  Construct median (ms)  Step median (us)  Step p95 (us)")
    print("-" * 93)
    for name, case in results["cases"].items():
        print(
            f"{name:<20} {case['actuator_batches']:>7} {case['motor_count']:>7} "
            f"{case['construction_ms']['median']:>22.3f} "
            f"{case['step_us']['median']:>17.3f} {case['step_us']['p95']:>14.3f}"
        )

    print("\nMedian step speedup relative to python_baseline:")
    for name, speedup in results["step_speedup_vs_baseline"].items():
        print(f"  {name}: {speedup:.2f}x")


def main() -> None:
    """Set up the shared Franka stage and execute all benchmark cases.

    Example:

    .. code-block:: python

        >>> main()
    """
    device = wp.get_device(args.device)
    SimulationManager.set_physics_sim_device(str(device))
    SimulationManager.set_physics_dt(STEP_DT)
    simulation_app.run_coroutine(setup_stage(args.num_robots))
    author_usd_actuators()

    timeline = omni.timeline.get_timeline_interface()
    timeline.play()
    simulation_app.update()

    factories: dict[str, Factory] = {
        "python_baseline": lambda: ArticulationActuators.from_actuators(
            FRANKA_PATH_EXPRESSION,
            build_baseline_configs(args.num_robots, device),
            auto_step_pre_physics=False,
            device=device,
        ),
        "python_grouped": lambda: ArticulationActuators.from_actuators(
            FRANKA_PATH_EXPRESSION,
            build_grouped_config(args.num_robots, device),
            auto_step_pre_physics=False,
            device=device,
        ),
        "usd_model_builder": lambda: ArticulationActuators(
            FRANKA_PATH_EXPRESSION,
            auto_step_pre_physics=False,
            device=device,
        ),
    }

    cases: dict[str, Any] = {}
    for name, factory in factories.items():
        print(f"Benchmarking {name}...")
        construction_ms = benchmark_construction(factory, device, args.construction_repeats)
        step_us, actuator_batches, motor_count = benchmark_steps(
            factory,
            device,
            warmup_steps=args.warmup_steps,
            steps=args.steps,
            trials=args.trials,
        )
        expected_motor_count = args.num_robots * len(ARM_JOINTS)
        if motor_count != expected_motor_count:
            raise RuntimeError(f"{name} built {motor_count} motors; expected {expected_motor_count}.")
        cases[name] = {
            "actuator_batches": actuator_batches,
            "motor_count": motor_count,
            "construction_ms": construction_ms,
            "step_us": step_us,
        }

    baseline_step_us = cases["python_baseline"]["step_us"]["median"]
    speedups = {name: baseline_step_us / case["step_us"]["median"] for name, case in cases.items()}
    results = {
        "actuator_device": str(device),
        "physics_device": str(SimulationManager.get_physics_sim_device()),
        "num_robots": args.num_robots,
        "steps": args.steps,
        "trials": args.trials,
        "warmup_steps": args.warmup_steps,
        "construction_repeats": args.construction_repeats,
        "cases": cases,
        "step_speedup_vs_baseline": speedups,
    }
    print_results(results)

    if args.output is not None:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(json.dumps(results, indent=2) + "\n", encoding="utf-8")
        print(f"\nWrote JSON results to {args.output}")

    timeline.stop()
    simulation_app.update()


try:
    main()
finally:
    simulation_app.close()
