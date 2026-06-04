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

"""Workflow benchmark: synthetic data capture with Replicator ``CosmosWriter``."""

import argparse
import os
import shutil
import tempfile

parser = argparse.ArgumentParser()
parser.add_argument("--num-frames", type=int, default=60, help="Frames to capture")
parser.add_argument("--num-gpus", type=int, default=None, help="Number of GPUs on machine.")
parser.add_argument("--resolution", nargs=2, type=int, default=[1280, 720], help="Render product resolution")
parser.add_argument("--gpu-frametime", action="store_true", help="Enable GPU frametime measurement")
parser.add_argument("--non-headless", action="store_true", help="Run with GUI")
parser.add_argument(
    "--viewport-updates",
    dest="disable_viewport_updates",
    action="store_false",
    default=True,
    help="Enable viewport updates when headless",
)
parser.add_argument(
    "--backend-type",
    default="OmniPerfKPIFile",
    choices=["LocalLogMetrics", "JSONFileMetrics", "OsmoKPIFile", "OmniPerfKPIFile"],
    help="Benchmarking backend",
)
parser.add_argument(
    "--output-dir",
    default=None,
    help="CosmosWriter disk backend output directory (default: temp dir under cwd)",
)
parser.add_argument("--delete-data-when-done", action="store_true", help="Remove output directory after benchmark")

args, unknown = parser.parse_known_args()

num_frames = args.num_frames
width, height = args.resolution[0], args.resolution[1]
n_gpu = args.num_gpus
gpu_frametime = args.gpu_frametime
headless = not args.non_headless
disable_viewport_updates = args.disable_viewport_updates

from isaacsim import SimulationApp

simulation_app = SimulationApp(
    {"headless": headless, "max_gpu_count": n_gpu, "disable_viewport_updates": disable_viewport_updates}
)

import carb
import omni.kit.app
import omni.replicator.core as rep
import omni.timeline
import omni.usd
from isaacsim.core.utils.extensions import enable_extension

enable_extension("isaacsim.benchmark.services")
from isaacsim.benchmark.services import DEFAULT_RECORDERS, BaseIsaacBenchmark

recorders = list(DEFAULT_RECORDERS) + (["gpu_frametime"] if gpu_frametime else [])
out_dir = args.output_dir or os.path.join(tempfile.gettempdir(), "isaac_workflow_cosmos_writer_out")
os.makedirs(out_dir, exist_ok=True)

benchmark = BaseIsaacBenchmark(
    benchmark_name="benchmark_workflow_sdg_cosmos_writer",
    workflow_metadata={
        "metadata": [
            {"name": "num_frames", "data": num_frames},
            {"name": "width", "data": width},
            {"name": "height", "data": height},
            {"name": "output_dir", "data": out_dir},
            {"name": "num_gpus", "data": carb.settings.get_settings().get("/renderer/multiGpu/currentGpuCount")},
        ]
    },
    backend_type=args.backend_type,
    recorders=recorders,
)

benchmark.set_phase("loading", start_recording_frametime=False, start_recording_runtime=True)

carb.settings.get_settings().set("rtx/post/dlss/execMode", 2)
carb.settings.get_settings().set_bool("/app/omni.graph.scriptnode/opt_in", True)

rep.orchestrator.set_capture_on_play(False)
rep.settings.set_stage_up_axis("Z")
rep.settings.set_stage_meters_per_unit(1.0)

omni.usd.get_context().new_stage()
rep.functional.create.dome_light(intensity=500)

plane = rep.functional.create.plane(position=(0, 0, 0), scale=(10, 10, 1), semantics={"class": "plane"})
rep.functional.physics.apply_collider(plane)

sphere = rep.functional.create.sphere(position=(0, 0, 3), semantics={"class": "sphere"})
rep.functional.physics.apply_collider(sphere)
rep.functional.physics.apply_rigid_body(sphere)

cube = rep.functional.create.cube(position=(1, 1, 2), scale=0.5, semantics={"class": "cube"})
rep.functional.physics.apply_collider(cube)
rep.functional.physics.apply_rigid_body(cube)

camera = rep.functional.create.camera(position=(5, 5, 3), look_at=(0, 0, 0))
rp = rep.create.render_product(camera, (width, height))

segmentation_mapping = {
    "plane": [0, 0, 255, 255],
    "cube": [255, 0, 0, 255],
    "sphere": [0, 255, 0, 255],
}
backend = rep.backends.get("DiskBackend")
backend.initialize(output_dir=out_dir)
cosmos_writer = rep.WriterRegistry.get("CosmosWriter")
cosmos_writer.initialize(backend=backend, segmentation_mapping=segmentation_mapping)
cosmos_writer.attach(rp)

timeline = omni.timeline.get_timeline_interface()
timeline.play()
omni.kit.app.get_app().update()

benchmark.store_measurements()
benchmark.set_phase("benchmark")

for _ in range(num_frames):
    omni.kit.app.get_app().update()
    rep.orchestrator.step(delta_time=0.0, pause_timeline=False)

benchmark.store_measurements()

timeline.pause()
rep.orchestrator.wait_until_complete()
cosmos_writer.detach()
rp.destroy()

benchmark.stop()

if args.delete_data_when_done and os.path.isdir(out_dir):
    shutil.rmtree(out_dir, ignore_errors=True)

simulation_app.close()
