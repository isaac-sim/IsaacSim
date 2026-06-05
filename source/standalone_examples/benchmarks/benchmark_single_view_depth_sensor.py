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

"""Benchmark single-view depth sensor performance in Isaac Sim."""

import argparse

parser = argparse.ArgumentParser()
parser.add_argument("--num-cameras", type=int, default=1, help="Number of cameras")
parser.add_argument(
    "--resolution", nargs=2, type=int, default=[1280, 720], help="Camera resolution as [width, height] px"
)
parser.add_argument("--num-gpus", type=int, default=None, help="Number of GPUs on machine.")
parser.add_argument("--num-frames", type=int, default=600, help="Number of frames to run benchmark for")
parser.add_argument("--gpu-frametime", action="store_true", help="Enable GPU frametime measurement")
parser.add_argument("--non-headless", action="store_false", help="Run with GUI - nonheadless mode")
parser.add_argument("--viewport-updates", action="store_false", help="Enable viewport updates when headless")
parser.add_argument(
    "--backend-type",
    default="OmniPerfKPIFile",
    choices=["LocalLogMetrics", "JSONFileMetrics", "OsmoKPIFile", "OmniPerfKPIFile"],
    help="Benchmarking backend, defaults",
)

args, unknown = parser.parse_known_args()

n_camera = args.num_cameras
resolution = args.resolution
n_gpu = args.num_gpus
n_frames = args.num_frames
gpu_frametime = args.gpu_frametime
headless = args.non_headless
viewport_updates = args.viewport_updates

from isaacsim import SimulationApp

simulation_app = SimulationApp(
    {"headless": headless, "max_gpu_count": n_gpu, "disable_viewport_updates": viewport_updates}
)

import carb
import omni
from isaacsim.core.experimental.utils.stage import is_stage_loading
from isaacsim.core.utils.extensions import enable_extension
from isaacsim.core.utils.rotations import euler_angles_to_quat
from isaacsim.sensors.camera import SingleViewDepthSensor

enable_extension("isaacsim.benchmark.services")
from isaacsim.benchmark.services import DEFAULT_RECORDERS, BaseIsaacBenchmark

# Create the benchmark
# Define recorders to use, use default set, other combinations, or custom data recorders
recorders = DEFAULT_RECORDERS + ["gpu_frametime"] if gpu_frametime else DEFAULT_RECORDERS
benchmark = BaseIsaacBenchmark(
    benchmark_name="benchmark_single_view_depth_sensor",
    workflow_metadata={
        "metadata": [
            {"name": "num_cameras", "data": n_camera},
            {"name": "width", "data": resolution[0]},
            {"name": "height", "data": resolution[1]},
            {"name": "num_gpus", "data": carb.settings.get_settings().get("/renderer/multiGpu/currentGpuCount")},
        ]
    },
    backend_type=args.backend_type,
    recorders=recorders,
)
benchmark.set_phase("loading", start_recording_frametime=False, start_recording_runtime=True)

scene_path = "/Isaac/Environments/Simple_Warehouse/full_warehouse.usd"
benchmark.fully_load_stage(benchmark.assets_root_path + scene_path)

# make sure scene is loaded in all viewports
while is_stage_loading():
    print("asset still loading, waiting to finish")
    omni.kit.app.get_app().update()
omni.kit.app.get_app().update()

timeline = omni.timeline.get_timeline_interface()
for i in range(n_camera):
    sensor = SingleViewDepthSensor(
        name=f"depth_sensor_{i}",
        prim_path=f"/World/depth_sensor_{i}",
        position=[-8, 13, 2.0],
        orientation=euler_angles_to_quat([90, 0, 90 + i * 360 / n_camera], degrees=True),
        resolution=(resolution[0], resolution[1]),
    )
    sensor.initialize()
    sensor.attach_annotator("DepthSensorDistance")

omni.kit.app.get_app().update()
benchmark.store_measurements()
# perform benchmark
timeline.play()
benchmark.set_phase("benchmark", warmup_frames=15)
for _ in range(1, n_frames):
    omni.kit.app.get_app().update()
benchmark.store_measurements()
benchmark.stop()

timeline.stop()
simulation_app.close()
