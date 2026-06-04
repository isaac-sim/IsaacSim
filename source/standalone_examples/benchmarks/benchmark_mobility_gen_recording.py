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

"""Headless benchmark for MobilityGen trajectory recording performance.

Runs the recording loop (physics step → scenario.step → write_state_dict_common)
for N steps and reports per-phase timing via isaacsim.benchmark.services for
consistent metric reporting compatible with nightly and PerfLab pipelines.

Example usage (from the Isaac Sim build directory):

    cd _build/linux-x86_64/release

    # Plain timing run — default RandomAccelerationScenario
    ./python.sh ../../../source/standalone_examples/benchmarks/benchmark_mobility_gen_recording.py \\
        --num-steps 200

    # Benchmark a different scenario
    ./python.sh ../../../source/standalone_examples/benchmarks/benchmark_mobility_gen_recording.py \\
        --scenario random_path_following --num-steps 200

    ./python.sh ../../../source/standalone_examples/benchmarks/benchmark_mobility_gen_recording.py \\
        --scenario keyboard_teleop --num-steps 200
"""

import argparse
import os
import tempfile
import time

DEFAULT_OMAP_PATH = os.path.join(
    os.path.dirname(os.path.abspath(__file__)),
    "data",
    "mobility_gen",
    "warehouse_multiple_shelves",
    "map.yaml",
)

parser = argparse.ArgumentParser(description="MobilityGen recording benchmark")
parser.add_argument(
    "--omap",
    type=str,
    default=DEFAULT_OMAP_PATH,
    help="Path to the occupancy map YAML file.",
)
parser.add_argument(
    "--scene-url",
    type=str,
    default=None,
    help=(
        "URL or path to the warehouse USD stage.  " "Defaults to warehouse_multiple_shelves from the Isaac assets root."
    ),
)
parser.add_argument(
    "--scenario",
    type=str,
    default="random_acceleration",
    choices=["random_acceleration", "random_path_following", "keyboard_teleop"],
    help="Scenario to benchmark.",
)
parser.add_argument(
    "--num-steps",
    type=int,
    default=200,
    help="Number of physics steps to run for benchmarking.",
)
parser.add_argument(
    "--warmup-steps",
    type=int,
    default=20,
    help="Number of warm-up steps to run before timing begins.",
)
parser.add_argument(
    "--output-dir",
    type=str,
    default=None,
    help="Directory to write recorded data.  Defaults to a temporary directory.",
)
parser.add_argument(
    "--async-write",
    action=argparse.BooleanOptionalAction,
    default=True,
    help="Use async background thread for npz writes (--async-write / --no-async-write).",
)
parser.add_argument(
    "--backend-type",
    default="OmniPerfKPIFile",
    choices=["LocalLogMetrics", "JSONFileMetrics", "OsmoKPIFile", "OmniPerfKPIFile"],
    help="Benchmarking backend for isaacsim.benchmark.services metric reporting.",
)

args, _unknown = parser.parse_known_args()

# ─────────────────────────────────────────────────────────────────────────────
# Launch Isaac Sim
# ─────────────────────────────────────────────────────────────────────────────
from isaacsim import SimulationApp

simulation_app = SimulationApp(launch_config={"headless": True, "multi_gpu": False})

import carb
import isaacsim.core.experimental.utils.app as app_utils
from isaacsim.core.experimental.objects import GroundPlane
from isaacsim.core.experimental.utils.stage import get_current_stage, open_stage
from isaacsim.core.simulation_manager import SimulationManager
from isaacsim.storage.native import get_assets_root_path
from pxr import UsdGeom

app_utils.enable_extension("isaacsim.replicator.experimental.mobility_gen")
app_utils.enable_extension("isaacsim.replicator.mobility_gen.examples")
app_utils.enable_extension("isaacsim.benchmark.services")

simulation_app.update()

from isaacsim.benchmark.services import BaseIsaacBenchmark
from isaacsim.benchmark.services.datarecorders import (
    MeasurementData,
    MeasurementDataRecorder,
)
from isaacsim.benchmark.services.metrics import measurements
from isaacsim.benchmark.services.utils import wait_until_stage_is_fully_loaded
from isaacsim.replicator.experimental.mobility_gen import (
    Config,
    MobilityGenWriter,
    OccupancyMap,
    save_sensor_overrides,
)
from isaacsim.replicator.mobility_gen.examples import (
    CarterRobot,
    KeyboardTeleoperationScenario,
    RandomAccelerationScenario,
    RandomPathFollowingScenario,
)


# ─────────────────────────────────────────────────────────────────────────────
# Custom benchmark recorder
# ─────────────────────────────────────────────────────────────────────────────
class MobilityGenRecorder(MeasurementDataRecorder):
    """Records per-phase step timings from the MobilityGen recording loop."""

    def __init__(self, context=None, **kwargs):
        self._results = None

    def set_results(self, results: dict):
        self._results = results

    def start_collecting(self):
        pass

    def stop_collecting(self):
        pass

    def get_data(self) -> MeasurementData:
        if not self._results:
            return MeasurementData()

        d = self._results
        N = d["num_steps"]
        t_total = d["t_total"]

        ms_list = [
            measurements.SingleMeasurement(name="Step Rate", value=round(N / t_total, 2), unit="steps/s"),
            measurements.SingleMeasurement(name="Mean Step Time", value=round(t_total * 1000 / N, 3), unit="ms"),
            measurements.SingleMeasurement(
                name="SimulationManager.step ms/step", value=round(d["t_sim_step"] * 1000 / N, 3), unit="ms"
            ),
            measurements.SingleMeasurement(
                name="simulation_app.update ms/step", value=round(d["t_app_update"] * 1000 / N, 3), unit="ms"
            ),
            measurements.SingleMeasurement(
                name="scenario.step ms/step", value=round(d["t_scenario_step"] * 1000 / N, 3), unit="ms"
            ),
            measurements.SingleMeasurement(
                name="state_dict_common ms/step", value=round(d["t_state_dict"] * 1000 / N, 3), unit="ms"
            ),
            measurements.SingleMeasurement(
                name="write_state_dict_common ms/step", value=round(d["t_write"] * 1000 / N, 3), unit="ms"
            ),
            measurements.SingleMeasurement(name="Resets", value=d["n_resets"], unit="count"),
        ]

        if d.get("is_random_accel"):
            ms_list.extend(
                [
                    measurements.SingleMeasurement(
                        name="update_state ms/step", value=round(d["t_update_state"] * 1000 / N, 3), unit="ms"
                    ),
                    measurements.SingleMeasurement(
                        name="robot.write_action ms/step", value=round(d["t_write_action"] * 1000 / N, 3), unit="ms"
                    ),
                ]
            )

        return MeasurementData(measurements=ms_list)


# ─────────────────────────────────────────────────────────────────────────────
# Resolve paths
# ─────────────────────────────────────────────────────────────────────────────
omap_path = os.path.expanduser(args.omap)
scene_url = args.scene_url or (
    get_assets_root_path() + "/Isaac/Environments/Simple_Warehouse/warehouse_multiple_shelves.usd"
)

output_dir = args.output_dir or tempfile.mkdtemp(prefix="mobility_gen_bench_")
recording_path = os.path.join(output_dir, "recording")

SCENARIO_CLASSES = {
    "random_acceleration": RandomAccelerationScenario,
    "random_path_following": RandomPathFollowingScenario,
    "keyboard_teleop": KeyboardTeleoperationScenario,
}
ScenarioClass = SCENARIO_CLASSES[args.scenario]

carb.log_warn(f"[MobilityGen Bench] Scene URL : {scene_url}")
carb.log_warn(f"[MobilityGen Bench] Omap path : {omap_path}")
carb.log_warn(f"[MobilityGen Bench] Output dir: {output_dir}")
carb.log_warn(f"[MobilityGen Bench] Scenario  : {ScenarioClass.__name__}")

# ─────────────────────────────────────────────────────────────────────────────
# Set up benchmark
# ─────────────────────────────────────────────────────────────────────────────
benchmark = BaseIsaacBenchmark(
    benchmark_name="benchmark_mobility_gen_recording",
    workflow_metadata={
        "metadata": [
            {"name": "scenario", "data": ScenarioClass.__name__},
            {"name": "num_steps", "data": args.num_steps},
            {"name": "warmup_steps", "data": args.warmup_steps},
            {"name": "async_write", "data": args.async_write},
            {"name": "physics_dt_ms", "data": round(CarterRobot.physics_dt * 1000, 2)},
        ]
    },
    backend_type=args.backend_type,
)

# Append the custom recorder so store_measurements() picks it up.
mobility_gen_recorder = MobilityGenRecorder()
benchmark.recorders.append(mobility_gen_recorder)

benchmark.set_phase("loading", start_recording_frametime=False, start_recording_runtime=True)

# ─────────────────────────────────────────────────────────────────────────────
# Load stage and build scenario
# ─────────────────────────────────────────────────────────────────────────────
open_stage(scene_url)
# Pump the Kit update loop until frametimes stabilize so that async USD/material
# loading completes before physics is initialized.  A single update() is not
# sufficient for a complex scene like the warehouse and causes SimulationManager.step()
# to hang on the first physics step.
wait_until_stage_is_fully_loaded()

occupancy_map = OccupancyMap.from_ros_yaml(omap_path)

SimulationManager.setup_simulation(dt=CarterRobot.physics_dt)

ground_plane = GroundPlane("/World/ground_plane", templates=None)
stage = get_current_stage()
for mesh_path in ground_plane.meshes.paths:
    UsdGeom.Imageable(stage.GetPrimAtPath(mesh_path)).MakeInvisible()

robot = CarterRobot.build("/World/robot")
scenario = ScenarioClass.from_robot_occupancy_map(robot, occupancy_map)

SimulationManager.initialize_physics()
simulation_app.update()

scenario.reset()

config = Config(
    scenario_type=ScenarioClass.__name__,
    robot_type=CarterRobot.__name__,
    scene_usd=scene_url,
)
writer = MobilityGenWriter(recording_path, async_write=args.async_write)
writer.write_config(config)
writer.write_occupancy_map(scenario.occupancy_map)
save_sensor_overrides(robot.prim_path, recording_path)

# ─────────────────────────────────────────────────────────────────────────────
# Warm-up
# ─────────────────────────────────────────────────────────────────────────────
step_size = CarterRobot.physics_dt
carb.log_warn(f"[MobilityGen Bench] Warming up ({args.warmup_steps} steps)...")
for _ in range(args.warmup_steps):
    SimulationManager.step(steps=1)
    simulation_app.update()
    is_alive = scenario.step(step_size)
    if not is_alive:
        scenario.reset()

benchmark.store_measurements()

# ─────────────────────────────────────────────────────────────────────────────
# Benchmark loop — emit Tracy trigger then run timed steps
# ─────────────────────────────────────────────────────────────────────────────
carb.log_warn("Starting phase: benchmark")

benchmark.set_phase("benchmark")

N = args.num_steps
t_sim_step = 0.0
t_app_update = 0.0
t_scenario_step = 0.0
# Sub-timings for RandomAccelerationScenario (inlined step)
t_update_state = 0.0
t_write_action = 0.0
t_state_dict = 0.0
t_write = 0.0
t_total = 0.0
n_resets = 0

carb.log_warn(f"[MobilityGen Bench] Running {N} benchmark steps...")

# For RandomAccelerationScenario we inline the step to get sub-phase timing.
is_random_accel = isinstance(scenario, RandomAccelerationScenario)
if is_random_accel:
    import numpy as _np

robot = scenario.robot

for step_idx in range(N):
    t0 = time.perf_counter()

    # ── SimulationManager.step ────────────────────────────────────────────
    t1 = time.perf_counter()
    SimulationManager.step(steps=1)
    t_sim_step += time.perf_counter() - t1

    # ── simulation_app.update ─────────────────────────────────────────────
    t1 = time.perf_counter()
    simulation_app.update()
    t_app_update += time.perf_counter() - t1

    # ── scenario.step (or inlined for RandomAcceleration) ─────────────────
    t1 = time.perf_counter()
    if is_random_accel:
        # Inlined step for sub-phase granularity
        t2 = time.perf_counter()
        scenario.update_state()
        t_update_state += time.perf_counter() - t2

        current_action = robot.action.get_value()
        linear_velocity = (
            current_action[0] + step_size * _np.random.randn(1) * robot.random_action_linear_acceleration_std
        )
        angular_velocity = (
            current_action[1] + step_size * _np.random.randn(1) * robot.random_action_angular_acceleration_std
        )
        linear_velocity = _np.clip(linear_velocity, *robot.random_action_linear_velocity_range)[0]
        angular_velocity = _np.clip(angular_velocity, *robot.random_action_angular_velocity_range)[0]
        robot.action.set_value(_np.array([linear_velocity, angular_velocity]))

        t2 = time.perf_counter()
        robot.write_action(step_size)
        t_write_action += time.perf_counter() - t2

        pose = robot.get_pose_2d()
        if not scenario.collision_occupancy_map.check_world_point_in_bounds(pose):
            scenario.is_alive = False
        elif not scenario.collision_occupancy_map.check_world_point_in_freespace(pose):
            scenario.is_alive = False
        is_alive = scenario.is_alive
    else:
        is_alive = scenario.step(step_size)

    t_scenario_step += time.perf_counter() - t1

    if not is_alive:
        scenario.reset()
        n_resets += 1

    # ── Build state dict ───────────────────────────────────────────────────
    t1 = time.perf_counter()
    state_dict = scenario.state_dict_common()
    t_state_dict += time.perf_counter() - t1

    # ── Write to disk ──────────────────────────────────────────────────────
    t1 = time.perf_counter()
    writer.write_state_dict_common(state_dict, step=step_idx)
    t_write += time.perf_counter() - t1

    t_total += time.perf_counter() - t0


# ─────────────────────────────────────────────────────────────────────────────
# Report results
# ─────────────────────────────────────────────────────────────────────────────
def _ms(t):
    return t * 1000.0 / N


accounted = t_sim_step + t_app_update + t_scenario_step + t_state_dict + t_write

carb.log_warn("=" * 65)
carb.log_warn(
    f"[MobilityGen Bench] Results over {N} steps"
    f" ({ScenarioClass.__name__}, physics dt={step_size*1000:.1f} ms, async_write={args.async_write}):"
)
carb.log_warn(f"  {'Total loop time':<32}: {t_total*1000:>8.1f} ms total  {_ms(t_total):>8.3f} ms/step")
carb.log_warn(
    f"  {'SimulationManager.step':<32}: {t_sim_step*1000:>8.1f} ms  ({100*t_sim_step/t_total:>5.1f}%)  {_ms(t_sim_step):>8.3f} ms/step"
)
carb.log_warn(
    f"  {'simulation_app.update':<32}: {t_app_update*1000:>8.1f} ms  ({100*t_app_update/t_total:>5.1f}%)  {_ms(t_app_update):>8.3f} ms/step"
)
carb.log_warn(
    f"  {'scenario.step':<32}: {t_scenario_step*1000:>8.1f} ms  ({100*t_scenario_step/t_total:>5.1f}%)  {_ms(t_scenario_step):>8.3f} ms/step"
)
if is_random_accel:
    carb.log_warn(
        f"    {'update_state':<30}: {t_update_state*1000:>8.1f} ms  ({100*t_update_state/t_total:>5.1f}%)  {_ms(t_update_state):>8.3f} ms/step"
    )
    carb.log_warn(
        f"    {'robot.write_action':<30}: {t_write_action*1000:>8.1f} ms  ({100*t_write_action/t_total:>5.1f}%)  {_ms(t_write_action):>8.3f} ms/step"
    )
carb.log_warn(
    f"  {'state_dict_common':<32}: {t_state_dict*1000:>8.1f} ms  ({100*t_state_dict/t_total:>5.1f}%)  {_ms(t_state_dict):>8.3f} ms/step"
)
carb.log_warn(
    f"  {'write_state_dict_common':<32}: {t_write*1000:>8.1f} ms  ({100*t_write/t_total:>5.1f}%)  {_ms(t_write):>8.3f} ms/step"
)
carb.log_warn(f"  ---")
carb.log_warn(f"  {'Accounted overhead':<32}: {accounted*1000:>8.1f} ms  ({100*accounted/t_total:>5.1f}%)")
carb.log_warn(f"  {'Resets during benchmark':<32}: {n_resets}")
carb.log_warn(f"  {'Effective step rate':<32}: {N/t_total:>8.1f} steps/s")
carb.log_warn("=" * 65)

mobility_gen_recorder.set_results(
    {
        "num_steps": N,
        "t_total": t_total,
        "t_sim_step": t_sim_step,
        "t_app_update": t_app_update,
        "t_scenario_step": t_scenario_step,
        "t_state_dict": t_state_dict,
        "t_write": t_write,
        "n_resets": n_resets,
        "is_random_accel": is_random_accel,
        "t_update_state": t_update_state,
        "t_write_action": t_write_action,
    }
)

benchmark.store_measurements()
benchmark.stop()

writer.close()
simulation_app.close()
