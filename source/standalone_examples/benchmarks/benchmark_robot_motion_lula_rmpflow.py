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

"""Benchmark legacy Lula RMPflow (Franka + isaacsim.core.api.World + motion_generation) under timeline / physics steps."""

from __future__ import annotations

import argparse
import time
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
from isaacsim.core.experimental.utils import stage as stage_utils
from isaacsim.core.simulation_manager import SimulationEvent, SimulationManager
from isaacsim.core.utils.extensions import enable_extension

enable_extension("isaacsim.benchmark.services")
enable_extension("isaacsim.robot_motion.motion_generation.examples")

import cProfile
import pstats

from isaacsim.benchmark.services import BaseIsaacBenchmark
from isaacsim.benchmark.services.metrics.measurements import SingleMeasurement
from isaacsim.robot_motion.motion_generation.examples.rmp_flow.scenario import FrankaRmpFlowExample

profiler = cProfile.Profile()


def _load_franka_rmpflow_world(simulation_app) -> FrankaRmpFlowExample:
    """Load the legacy Lula Franka example without ``UIBuilder._load_world_async``.

    The deprecated UI builds ``World`` before ``create_new_stage()``; opening a stage invalidates that
    ``World``, so headless runs fail with ``World.instance()`` being ``None``. This path creates the
    stage and assets first, then constructs ``World`` and registers prims — same end state as the UI
    after a successful load, without modifying the extension.

    ``SimulationApp.run_coroutine`` must not be used for long ``async`` loaders: monopolizing Kit's
    asyncio loop triggers Python 3.12 ``RuntimeError: Cannot enter into task ... while another task``
    spam from unrelated extensions. Standalone scripts also skip automatic ``SimulationContext``
    setup when ``ISAAC_LAUNCHED_FROM_TERMINAL`` is set, so we mirror ``initialize_simulation_context_async``
    with synchronous ``World`` APIs and ``simulation_app.update()`` flushes.
    """
    import carb.eventdispatcher
    import omni.usd
    from isaacsim.core.api.world import World
    from isaacsim.core.prims import SingleXFormPrim as XFormPrim
    from isaacsim.core.utils.stage import create_new_stage, get_current_stage
    from isaacsim.core.utils.viewports import set_camera_view
    from pxr import Sdf, UsdLux

    prev_world = World.instance()
    if prev_world is not None:
        prev_world.clear_all_callbacks()
        prev_world.clear_instance()
    for _ in range(2):
        simulation_app.update()

    create_new_stage()
    sphere_light = UsdLux.SphereLight.Define(get_current_stage(), Sdf.Path("/World/SphereLight"))
    sphere_light.CreateRadiusAttr(2)
    sphere_light.CreateIntensityAttr(100000)
    XFormPrim(str(sphere_light.GetPath())).set_world_pose([6.5, 0, 12])
    set_camera_view(eye=[2, 1.5, 2], target=[0, 0, 0], camera_prim_path="/OmniverseKit_Persp")

    scenario = FrankaRmpFlowExample()
    loaded_objects = scenario.load_example_assets()

    world = World(physics_dt=1 / 60.0, rendering_dt=1 / 60.0)
    for loaded_object in loaded_objects:
        world.scene.add(loaded_object)

    # Same ordering as ``initialize_simulation_context_async``, without ``await`` (see module docstring).
    if world.is_playing():
        world.stop()
    simulation_app.update()

    world._init_stage(
        physics_dt=1 / 60.0,
        rendering_dt=1 / 60.0,
        stage_units_in_meters=1.0,
        physics_prim_path="/physicsScene",
        sim_params=None,
        set_defaults=True,
        backend="numpy",
        device=None,
    )
    simulation_app.update()

    world._stage_open_callback = carb.eventdispatcher.get_eventdispatcher().observe_event(
        event_name=omni.usd.get_context().stage_event_name(omni.usd.StageEventType.OPENED),
        on_event=world._stage_open_callback_fn,
        observer_name="isaacsim.core.api.SimulationContext._stage_open_callback",
    )
    simulation_app.update()
    world._setup_default_callback_fns()
    simulation_app.update()

    world.reset()
    simulation_app.update()
    simulation_app.update()
    world.pause()
    simulation_app.update()

    scenario.setup()
    return scenario


benchmark = BaseIsaacBenchmark(
    benchmark_name="benchmark_robot_motion_lula_rmpflow",
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


def _make_physics_pre_step(scenario: FrankaRmpFlowExample, recorder: _PhysicsStepTimingRecorder):
    """Build callback for SimulationManager.register_callback(PHYSICS_PRE_STEP)."""

    def on_physics_pre_step(step_dt: float, context: object) -> None:
        # Legacy ``isaacsim.core.prims.SingleArticulation`` uses ``handles_initialized`` (not experimental
        # ``is_physics_tensor_entity_valid``). PHYSICS_PRE_STEP can fire before tensor handles exist.
        art = scenario._articulation
        if art is None or scenario._rmpflow is None:
            return
        if not art.handles_initialized:
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

scenario = _load_franka_rmpflow_world(simulation_app)
stage_utils.set_stage_up_axis("Z")
stage_utils.set_stage_units(meters_per_unit=1.0)

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

print("RMPflow setup phase...")
benchmark.set_phase("rmpflow_setup", start_recording_frametime=False, start_recording_runtime=True)
# RMPflow is initialized at the end of ``_load_franka_rmpflow_world()``.
benchmark.store_measurements()

timeline.play()
# Ensure PhysX articulation handles exist before registering pre-step callbacks (World.reset does not
# always attach tensors until the timeline has stepped at least once with physics enabled).
for _ in range(10):
    simulation_app.update()
    if scenario._articulation is not None and scenario._articulation.handles_initialized:
        break
else:
    if scenario._articulation is not None and not scenario._articulation.handles_initialized:
        scenario._articulation.initialize()

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

# Drop Python references to scenario/world prims before ``UsdContext.close_stage()`` runs inside
# ``simulation_app.close()``. Otherwise the stage refcount is still > 1 at shutdown and omni.usd
# logs ``Unexpected reference count of 2 for UsdStage ... while being closed``. Harmless, but
# tearing down symmetrically with the startup path keeps shutdown clean.
import gc

import omni.usd
from isaacsim.core.api.world import World

_world = World.instance()
if _world is not None:
    _world.clear_all_callbacks()
    _world.scene.clear()
    World.clear_instance()
del scenario
gc.collect()
omni.usd.get_context().close_stage()

simulation_app.close()
print("Benchmark completed")
