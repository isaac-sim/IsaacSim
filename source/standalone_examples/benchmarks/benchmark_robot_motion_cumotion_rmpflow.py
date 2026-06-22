# SPDX-FileCopyrightText: Copyright (c) 2024-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

"""Benchmark cuMotion RMPflow (Franka + WorldBinding + RmpFlowController) under timeline / physics steps."""

from __future__ import annotations

import argparse
import time
from collections.abc import Callable
from dataclasses import dataclass

parser = argparse.ArgumentParser()
parser.add_argument(
    "--num-frames", type=int, default=600, help="Number of app update frames to run in the benchmark phase"
)
parser.add_argument(
    "--backend-type",
    default="OmniPerfKPIFile",
    choices=["LocalLogMetrics", "JSONFileMetrics", "OsmoKPIFile", "OmniPerfKPIFile"],
    help="Benchmarking backend, defaults",
)

args, unknown = parser.parse_known_args()

n_frames = args.num_frames

from isaacsim import SimulationApp

simulation_app = SimulationApp({"headless": True})  #  "max_gpu_count": n_gpus

import numpy as np
import omni.timeline
from isaacsim.core.simulation_manager import SimulationEvent, SimulationManager
from isaacsim.core.utils.extensions import enable_extension

enable_extension("isaacsim.benchmark.services")
enable_extension("isaacsim.robot_motion.cumotion.examples")

import cProfile
import pstats

from isaacsim.benchmark.services import BaseIsaacBenchmark
from isaacsim.benchmark.services.metrics.measurements import SingleMeasurement
from isaacsim.robot_motion.cumotion.examples.rmp_flow.scenario import FrankaRmpFlowExample

profiler = cProfile.Profile()

benchmark = BaseIsaacBenchmark(
    benchmark_name="benchmark_robot_motion_cumotion_rmpflow",
    workflow_metadata={
        "metadata": [
            {"name": "num_frames", "data": n_frames},
        ]
    },
    backend_type=args.backend_type,
)


@dataclass
class _PhysicsStepTimingRecorder:
    """Fixed-size buffer + index: O(1) per physics callback, no list growth."""

    buffer: np.ndarray
    idx: int = 0
    enabled: bool = False


def _make_physics_pre_step(
    scenario: FrankaRmpFlowExample, recorder: _PhysicsStepTimingRecorder
) -> Callable[[float, object], None]:
    """Build callback for SimulationManager.register_callback(PHYSICS_PRE_STEP)."""

    def on_physics_pre_step(step_dt: float, context: object) -> None:
        if scenario._articulation is not None and not scenario._articulation.is_physics_tensor_entity_valid():
            return
        t0 = time.perf_counter()
        scenario.update(step_dt)
        if recorder.enabled and recorder.idx < recorder.buffer.shape[0]:
            recorder.buffer[recorder.idx] = (time.perf_counter() - t0) * 1000.0
            recorder.idx += 1

    return on_physics_pre_step


def _store_physics_step_timing_custom_measurements(phase_name: str, samples: np.ndarray) -> None:
    """Emit SingleMeasurement rows so the Summary Report includes them (DictMeasurement is not rendered)."""
    prefix = "physics_pre_step scenario_update"
    if samples.size == 0:
        benchmark.store_custom_measurement(
            phase_name,
            SingleMeasurement(name=f"{prefix} samples", value=0, unit=""),
        )
        return
    benchmark.store_custom_measurement(
        phase_name,
        SingleMeasurement(name=f"{prefix} samples", value=int(samples.size), unit=""),
    )
    benchmark.store_custom_measurement(
        phase_name,
        SingleMeasurement(name=f"{prefix} mean", value=float(np.mean(samples)), unit="ms"),
    )
    benchmark.store_custom_measurement(
        phase_name,
        SingleMeasurement(name=f"{prefix} p95", value=float(np.percentile(samples, 95)), unit="ms"),
    )
    benchmark.store_custom_measurement(
        phase_name,
        SingleMeasurement(name=f"{prefix} max", value=float(np.max(samples)), unit="ms"),
    )


physics_timing = _PhysicsStepTimingRecorder(buffer=np.empty(n_frames, dtype=np.float64))

print("Loading phase...")
benchmark.set_phase("loading", start_recording_frametime=False, start_recording_runtime=True)

scenario = FrankaRmpFlowExample()
simulation_app.run_coroutine(scenario.load())

simulation_app.update()

if SimulationManager.get_physics_simulation_view() is None:
    SimulationManager.initialize_physics()

timeline = omni.timeline.get_timeline_interface()
timeline.play()
for _ in range(5):
    simulation_app.update()
timeline.stop()
simulation_app.update()

benchmark.store_measurements()

timeline.play()
physics_callback_id = SimulationManager.register_callback(
    _make_physics_pre_step(scenario, physics_timing),
    event=SimulationEvent.PHYSICS_PRE_STEP,
    order=0,
)

print("Benchmark phase...")
profiler.enable()
benchmark.set_phase("benchmark", start_recording_frametime=True, start_recording_runtime=True, warmup_frames=15)
physics_timing.idx = 0
physics_timing.enabled = True
for _ in range(n_frames):
    simulation_app.update()
physics_timing.enabled = False

benchmark.store_measurements()

samples = physics_timing.buffer[: physics_timing.idx]
_store_physics_step_timing_custom_measurements("benchmark", samples)
if physics_timing.idx != n_frames:
    print(
        f"Warning: expected {n_frames} physics pre-step samples, recorded {physics_timing.idx} "
        "(check articulation validity / physics substeps)."
    )
if samples.size:
    print(
        "physics_pre_step scenario_update (ms): mean="
        f"{float(np.mean(samples)):.4f} p95={float(np.percentile(samples, 95)):.4f} "
        f"max={float(np.max(samples)):.4f} (n={samples.size})"
    )
else:
    print("physics_pre_step scenario_update: no samples recorded")

profiler.disable()

print("Stopping benchmark...")
SimulationManager.deregister_callback(physics_callback_id)

for i in range(10):
    simulation_app.update()
print("Stopping timeline...")
timeline.stop()
for i in range(10):
    simulation_app.update()

print("Stopping benchmark...")
benchmark.stop()
print("Closing simulation app...")

# print out the stats according to longest tottime and cumtime:
print("*" * 100)
print("cumulative time:")
stats = pstats.Stats(profiler)
stats.sort_stats("cumtime").print_stats(10)
print("*" * 100)
print("total time:")
stats.sort_stats("tottime").print_stats(10)
print("*" * 100)

simulation_app.close()
print("Benchmark completed")
