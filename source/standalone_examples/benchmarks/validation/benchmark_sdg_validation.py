# SPDX-FileCopyrightText: Copyright (c) 2024-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

import argparse
from pathlib import Path

parser = argparse.ArgumentParser()
parser.add_argument("--num-frames", type=int, default=600, help="Number of frames to capture")
parser.add_argument("--num-cameras", type=int, default=2, help="Number of cameras")
parser.add_argument("--num-gpus", type=int, default=None, help="Number of GPUs on machine.")
parser.add_argument("--resolution", nargs=2, type=int, default=[1280, 720], help="Camera resolution")
parser.add_argument(
    "--asset-count", type=int, default=10, help="Number of assets of each type (cube, cone, cylinder, sphere, torus)"
)
parser.add_argument("--non-headless", action="store_false", help="Run in non-headless mode")
parser.add_argument(
    "--backend-type",
    default="OmniPerfKPIFile",
    choices=["LocalLogMetrics", "JSONFileMetrics", "OsmoKPIFile", "OmniPerfKPIFile"],
    help="Benchmarking backend, defaults",
)

parser.add_argument("--env-url", default=None, help="Path to the environment url, default None")
parser.add_argument("--gpu-frametime", action="store_true", help="Enable GPU frametime measurement")

parser.add_argument(
    "--golden-dir",
    default="standalone_examples/benchmarks/validation/golden_data",
    help="Directory holding golden images - relative to the current working directory",
)
parser.add_argument(
    "--output-dir",
    default="standalone_examples/benchmarks/validation/captures",
    help="Directory holding output images from current run - relative to the current working directory",
)
parser.add_argument("--tolerance", type=int, default=5, help="Tolerance for mean difference in image comparison")
parser.add_argument(
    "--blur-kernel",
    type=int,
    default=0,
    help="Apply Gaussian blur with this kernel size before comparison (0=disabled)",
)
parser.add_argument(
    "--regenerate-golden",
    action="store_true",
    help="Regenerate golden images from current run. WARNING: This will overwrite existing golden images.",
)

args, unknown = parser.parse_known_args()

num_frames = args.num_frames
num_cameras = args.num_cameras
width, height = args.resolution[0], args.resolution[1]
asset_count = args.asset_count
annotators_str = "rgb"
headless = args.non_headless
n_gpu = args.num_gpus
env_url = args.env_url
gpu_frametime = args.gpu_frametime

print(f"[SDG Benchmark] Running SDG Benchmark with:")
print(f"\tnum_frames: {num_frames}")
print(f"\tnum_cameras: {num_cameras}")
print(f"\tresolution: {width}x{height}")
print(f"\tasset_count: {asset_count}")
print(f"\tannotators: {annotators_str}")
print(f"\theadless: {headless}")
print(f"\tenv_url: {env_url}")

import os
import shutil
import time

from isaacsim import SimulationApp

simulation_app = SimulationApp({"headless": headless, "max_gpu_count": n_gpu})

REPLICATOR_GLOBAL_SEED = 11

import carb
import omni.kit.app
import omni.replicator.core as rep
import omni.usd
from isaacsim.core.utils.extensions import enable_extension
from isaacsim.storage.native import get_assets_root_path
from omni.kit.viewport.utility import get_active_viewport

enable_extension("isaacsim.benchmark.services")

from isaacsim.benchmark.services import BaseIsaacBenchmark
from isaacsim.benchmark.services.validation import Validator

# Create the benchmark
benchmark = BaseIsaacBenchmark(
    benchmark_name="benchmark_sdg",
    workflow_metadata={
        "metadata": [
            {"name": "num_frames", "data": num_frames},
            {"name": "num_cameras", "data": num_cameras},
            {"name": "width", "data": width},
            {"name": "height", "data": height},
            {"name": "asset_count", "data": asset_count},
            {"name": "annotators", "data": annotators_str},
            {"name": "num_gpus", "data": carb.settings.get_settings().get("/renderer/multiGpu/currentGpuCount")},
        ]
    },
    backend_type=args.backend_type,
    gpu_frametime=gpu_frametime,
)

benchmark.set_phase("loading", start_recording_frametime=False, start_recording_runtime=True)

if env_url is not None:
    env_path = env_url if env_url.startswith("omniverse://") else get_assets_root_path() + env_url
    print(f"[SDG Benchmark] Loading stage from path: {env_path}")
    omni.usd.get_context().open_stage(env_path)
else:
    print(f"[SDG Benchmark] Loading a new empty stage..")
    omni.usd.get_context().new_stage()

rep.set_global_seed(REPLICATOR_GLOBAL_SEED)
rep.create.light(rotation=(315, 0, 0), intensity=2000, light_type="distant")
rep.create.light(intensity=400, light_type="dome")
cubes = rep.create.cube(count=asset_count, semantics=[("class", "cube")])
cones = rep.create.cone(count=asset_count, semantics=[("class", "cone")])
cylinders = rep.create.cylinder(count=asset_count, semantics=[("class", "cylinder")])
spheres = rep.create.sphere(count=asset_count, semantics=[("class", "sphere")])
tori = rep.create.torus(count=asset_count, semantics=[("class", "torus")])


cameras = []
for i in range(num_cameras):
    cameras.append(rep.create.camera(name=f"cam_{i}"))
render_products = []
for i, cam in enumerate(cameras):
    render_products.append(rep.create.render_product(cam, (width, height), name=f"rp_{i}"))

# Set rgb annotator
annot = rep.AnnotatorRegistry.get_annotator("rgb")
for rp in render_products:
    annot.attach(rp)

assets = rep.create.group([cubes, cones, cylinders, spheres, tori])
cameras = rep.create.group(cameras)

with rep.trigger.on_frame():
    with assets:
        rep.modify.pose(
            position=rep.distribution.uniform((-3, -3, -3), (3, 3, 3)),
            rotation=rep.distribution.uniform((0, 0, 0), (360, 360, 360)),
            scale=rep.distribution.uniform(0.1, 1),
        )
        rep.randomizer.color(rep.distribution.uniform((0, 0, 0), (1, 1, 1)))
    with cameras:
        rep.modify.pose(
            position=rep.distribution.uniform((5, 5, 5), (10, 10, 10)),
            look_at=(0, 0, 0),
        )

rep.orchestrator.preview()
# Run for a few frames to ensure everything is loaded
for _ in range(10):
    omni.kit.app.get_app().update()
benchmark.store_measurements()

print("[SDG Benchmark] Starting SDG..")
benchmark.set_phase("benchmark")
rep.orchestrator.run_until_complete(num_frames=num_frames)
benchmark.store_measurements()
omni.kit.app.get_app().update()
benchmark.stop()


# Validator setup
stage = omni.usd.get_context().get_stage()

validator = Validator.from_cli_args(args)
passed = validator.run(stage, benchmark_name=benchmark.benchmark_name)

simulation_app.close()
