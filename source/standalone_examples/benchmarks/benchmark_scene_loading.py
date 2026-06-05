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

"""Benchmark scene loading times in Isaac Sim."""

import argparse
import time

parser = argparse.ArgumentParser()
parser.add_argument("--num-frames", type=int, default=600, help="Number of frames to run benchmark for")
parser.add_argument(
    "--duration", type=int, default=None, help="Optional - duration in minutes (wall-clock time), overrides frame count"
)
parser.add_argument("--env-url", default=None, required=True, help="Path to the environment url - required")
parser.add_argument(
    "--camera-position", type=float, nargs=3, default=None, help="Set perspective position <x> <y> <z> - optional"
)
parser.add_argument(
    "--camera-target", type=float, nargs=3, default=None, help="Set perspective target <x> <y> <z> - optional"
)
parser.add_argument("--gpu-frametime", action="store_true", help="Enable GPU frametime measurement")
parser.add_argument("--non-headless", action="store_false", help="Run with GUI - nonheadless mode")
parser.add_argument("--num-gpus", type=int, default=None, help="Number of GPUs on machine.")
parser.add_argument(
    "--backend-type",
    default="OmniPerfKPIFile",
    choices=["LocalLogMetrics", "JSONFileMetrics", "OsmoKPIFile", "OmniPerfKPIFile"],
    help="Benchmarking backend, defaults",
)

args, unknown = parser.parse_known_args()

n_frames = args.num_frames
duration = args.duration
env_url = args.env_url
cam_pos = args.camera_position
cam_target = args.camera_target
gpu_frametime = args.gpu_frametime
headless = args.non_headless
n_gpu = args.num_gpus

# Both cam_pos and cam_target should be specified if used
if (cam_pos and not cam_target) or (cam_target and not cam_pos):
    parser.error("Both --camera-position and --camera-target must be specified together.")

from isaacsim import SimulationApp

simulation_app = SimulationApp({"headless": headless, "max_gpu_count": n_gpu})

import omni
import omni.kit.test
from isaacsim.core.utils.extensions import enable_extension
from isaacsim.core.utils.viewports import set_camera_view

enable_extension("isaacsim.ros2.bridge")
omni.kit.app.get_app().update()

enable_extension("isaacsim.benchmark.services")
from isaacsim.benchmark.services import DEFAULT_RECORDERS, BaseIsaacBenchmark

# Create the benchmark
# Define recorders to use, use default set, other combinations, or custom data recorders
recorders = DEFAULT_RECORDERS + ["gpu_frametime"] if gpu_frametime else DEFAULT_RECORDERS
benchmark = BaseIsaacBenchmark(
    benchmark_name="benchmark_scene_loading",
    workflow_metadata={"metadata": [{"name": "env_url", "data": env_url}, {"name": "duration", "data": duration}]},
    backend_type=args.backend_type,
    recorders=recorders,
)

# Track scene loading time
benchmark.set_phase("loading", start_recording_frametime=False, start_recording_runtime=True)
benchmark.fully_load_stage(benchmark.assets_root_path + env_url)
benchmark.store_measurements()

timeline = omni.timeline.get_timeline_interface()
timeline.play()

benchmark.set_phase("benchmark", warmup_frames=15)

if cam_pos is not None:
    set_camera_view(eye=cam_pos, target=cam_target, camera_prim_path="/OmniverseKit_Persp")

if duration is not None:
    start_time = time.time()
    while time.time() - start_time < (duration * 60):
        omni.kit.app.get_app().update()

else:
    for _ in range(1, n_frames):
        omni.kit.app.get_app().update()

benchmark.store_measurements()
benchmark.stop()

timeline.stop()
simulation_app.close()
