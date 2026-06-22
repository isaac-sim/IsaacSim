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

"""SDG workflow: Replicator with rigid-body props, drop-style randomization, and heavy functional randomization."""

import argparse
import os
import shutil
import time
from typing import Any

VALID_ANNOTATORS = {
    "rgb",
    "bounding_box_2d_tight_fast",
    "bounding_box_2d_loose_fast",
    "semantic_segmentation",
    "instance_id_segmentation_fast",
    "instance_segmentation_fast",
    "distance_to_camera",
    "distance_to_image_plane",
    "bounding_box_3d_fast",
    "occlusion",
    "normals",
    "motion_vectors",
    "camera_params",
    "pointcloud",
    "skeleton_data",
}

parser = argparse.ArgumentParser()
parser.add_argument("--num-frames", type=int, default=600, help="Number of frames to capture")
parser.add_argument("--num-cameras", type=int, default=1, help="Number of cameras")
parser.add_argument("--num-gpus", type=int, default=None, help="Number of GPUs on machine.")
parser.add_argument("--resolution", nargs=2, type=int, default=[1280, 720], help="Camera resolution")
parser.add_argument(
    "--rt-subframes", type=int, default=-1, help="Number of RT subframes to render per step (-1: use renderer default)"
)
parser.add_argument(
    "--asset-count", type=int, default=8, help="Number of assets of each primitive type (smaller default for physics)"
)
parser.add_argument(
    "--annotators",
    nargs="+",
    default=["rgb", "distance_to_image_plane"],
    choices=list(VALID_ANNOTATORS) + ["all"],
    help="Annotators for BasicWriter",
)
parser.add_argument("--delete-data-when-done", action="store_true", help="Delete local data after benchmarking")
parser.add_argument("--print-results", action="store_true", help="Print results in terminal")
parser.add_argument("--non-headless", action="store_true", help="Run in non-headless mode")
parser.add_argument(
    "--backend-type",
    default="OmniPerfKPIFile",
    choices=["LocalLogMetrics", "JSONFileMetrics", "OsmoKPIFile", "OmniPerfKPIFile"],
    help="Benchmarking backend",
)
parser.add_argument("--skip-write", action="store_true", help="Skip writing annotator data to disk")
parser.add_argument("--gpu-frametime", action="store_true", help="Enable GPU frametime measurement")
parser.add_argument(
    "--viewport-updates",
    dest="disable_viewport_updates",
    action="store_false",
    default=True,
    help="Enable viewport updates when headless",
)
parser.add_argument("--app-frametime", action="store_true", help="Enable Kit app frametime measurement")
parser.add_argument(
    "--with-og-randomization",
    action="store_true",
    help="Use OmniGraph-based randomization instead of functional API",
)
parser.add_argument(
    "--wait-for-render",
    action="store_true",
    help="Wait for render to complete before capturing frame",
)
parser.add_argument(
    "--no-replicator-write-to-fabric",
    action="store_true",
    help="Disable Replicator write-to-fabric mode",
)
parser.add_argument(
    "--randomize-drop-height",
    action="store_true",
    help="Each frame randomize vertical spawn for rigid bodies (drop-style)",
)

args, unknown = parser.parse_known_args()

num_frames = args.num_frames
num_cameras = args.num_cameras
width, height = args.resolution[0], args.resolution[1]
asset_count = args.asset_count
annotators_str = ", ".join(args.annotators)
delete_data_when_done = args.delete_data_when_done
print_results = args.print_results
headless = not args.non_headless
n_gpu = args.num_gpus
skip_write = args.skip_write
gpu_frametime = args.gpu_frametime
disable_viewport_updates = args.disable_viewport_updates
rt_subframes = args.rt_subframes
app_frametime = args.app_frametime
with_functional_randomization = not args.with_og_randomization
wait_for_render = args.wait_for_render
replicator_write_to_fabric = not args.no_replicator_write_to_fabric
randomize_drop_height = args.randomize_drop_height

if "all" in args.annotators:
    annotators_kwargs = dict.fromkeys(VALID_ANNOTATORS, True)
else:
    annotators_kwargs = {annotator: True for annotator in args.annotators if annotator in VALID_ANNOTATORS}

from isaacsim import SimulationApp

simulation_app = SimulationApp(
    {"headless": headless, "max_gpu_count": n_gpu, "disable_viewport_updates": disable_viewport_updates}
)

REPLICATOR_GLOBAL_SEED = 42

import carb
import omni.kit.app
import omni.replicator.core as rep
import omni.timeline
import omni.usd
from isaacsim.core.utils.extensions import enable_extension
from omni.replicator.core import Writer

enable_extension("isaacsim.benchmark.services")
from isaacsim.benchmark.services import DEFAULT_RECORDERS, BaseIsaacBenchmark
from isaacsim.benchmark.services.datarecorders import (
    MeasurementData,
    MeasurementDataRecorder,
    MeasurementDataRecorderRegistry,
)
from isaacsim.benchmark.services.metrics import measurements


class FPSWriter(Writer):
    """Lightweight writer that tracks replicator step speed (FPS)."""

    def __init__(self, annotators: list[str] | None = None) -> None:
        self._last_frame_time = None
        self._times_per_frame = []
        self.annotators = annotators if annotators else ["rgb"]

    def write(self, data: dict[str, Any]) -> None:
        """Record the time between consecutive Replicator frames."""
        if self._last_frame_time is None:
            self._last_frame_time = time.time()
            return

        time_per_frame = time.time() - self._last_frame_time
        self._times_per_frame.append(time_per_frame)
        self._last_frame_time = time.time()


@MeasurementDataRecorderRegistry.register("replicator_fps")
class ReplicatorFPSRecorder(MeasurementDataRecorder):
    """Bridge between FPSWriter and benchmark metrics."""

    def __init__(self, context: Any = None) -> None:
        self.context = context
        self._fps_writer = None

    def set_fps_writer(self, fps_writer: FPSWriter) -> None:
        """Attach an FPS writer for metric extraction."""
        self._fps_writer = fps_writer

    def start_collecting(self) -> None:
        """Start collecting FPS measurements."""

    def stop_collecting(self) -> None:
        """Stop collecting FPS measurements."""

    def get_data(self) -> MeasurementData:
        """Return the collected FPS measurements."""
        if not self._fps_writer or not self._fps_writer._times_per_frame:
            return MeasurementData()

        times = self._fps_writer._times_per_frame
        avg_time = sum(times) / len(times)
        avg_fps = 1.0 / avg_time if avg_time > 0 else 0.0
        min_time = min(times)
        max_time = max(times)

        return MeasurementData(
            measurements=[
                measurements.SingleMeasurement(name="Mean FPS", value=round(avg_fps, 2), unit="FPS"),
                measurements.SingleMeasurement(
                    name="Replicator Images/s", value=round(avg_fps * num_cameras, 2), unit="img/s"
                ),
                measurements.SingleMeasurement(
                    name="Mean Replicator Frametime", value=round(avg_time * 1000, 2), unit="ms"
                ),
                measurements.SingleMeasurement(
                    name="Min Replicator Frametime", value=round(min_time * 1000, 2), unit="ms"
                ),
                measurements.SingleMeasurement(
                    name="Max Replicator Frametime", value=round(max_time * 1000, 2), unit="ms"
                ),
                measurements.SingleMeasurement(name="Replicator Total Frames", value=len(times) + 1, unit="frames"),
            ]
        )


recorders = DEFAULT_RECORDERS.copy()
if gpu_frametime:
    recorders.append("gpu_frametime")
if not app_frametime and "app_frametime" in recorders:
    recorders.remove("app_frametime")
recorders.append("replicator_fps")

benchmark = BaseIsaacBenchmark(
    benchmark_name="benchmark_workflow_sdg_physics_randomization",
    workflow_metadata={
        "metadata": [
            {"name": "num_frames", "data": num_frames},
            {"name": "num_cameras", "data": num_cameras},
            {"name": "width", "data": width},
            {"name": "height", "data": height},
            {"name": "asset_count", "data": asset_count},
            {"name": "annotators", "data": annotators_str},
            {"name": "num_gpus", "data": carb.settings.get_settings().get("/renderer/multiGpu/currentGpuCount")},
            {"name": "skip_write", "data": skip_write},
            {"name": "with_functional_randomization", "data": with_functional_randomization},
            {"name": "wait_for_render", "data": wait_for_render},
            {"name": "replicator_write_to_fabric", "data": replicator_write_to_fabric},
            {"name": "randomize_drop_height", "data": randomize_drop_height},
        ]
    },
    backend_type=args.backend_type,
    recorders=recorders,
)

benchmark.set_phase("loading", start_recording_frametime=False, start_recording_runtime=True)

omni.usd.get_context().new_stage()
rep.functional.create.xform(name="World")
rep.functional.create.distant_light(intensity=2200, parent="/World", name="DistantLight")
rep.functional.create.dome_light(intensity=500, parent="/World", name="DomeLight")

carb.settings.get_settings().set("/exts/omni.replicator.core/enableWriteToFabric", replicator_write_to_fabric)
rep.set_global_seed(REPLICATOR_GLOBAL_SEED)
rng = rep.rng.ReplicatorRNG(seed=REPLICATOR_GLOBAL_SEED)

cubes = rep.functional.create_batch.cube(count=asset_count, parent="/World", name="Cube", semantics={"class": "cube"})
cones = rep.functional.create_batch.cone(count=asset_count, parent="/World", name="Cone", semantics={"class": "cone"})
cylinders = rep.functional.create_batch.cylinder(
    count=asset_count, parent="/World", name="Cylinder", semantics={"class": "cylinder"}
)
spheres = rep.functional.create_batch.sphere(
    count=asset_count, parent="/World", name="Sphere", semantics={"class": "sphere"}
)
tori = rep.functional.create_batch.torus(count=asset_count, parent="/World", name="Torus", semantics={"class": "torus"})
assets = cubes + cones + cylinders + spheres + tori

for prim in assets:
    rep.functional.physics.apply_collider(prim)
    rep.functional.physics.apply_rigid_body(prim)

cameras = rep.functional.create_batch.camera(count=num_cameras, parent="/World", name="Camera")
render_products = []
for i, cam in enumerate(cameras):
    render_products.append(rep.create.render_product(cam, (width, height), name=f"rp_{i}"))

replicator_fps_recorder = None
for recorder in benchmark.recorders:
    if isinstance(recorder, ReplicatorFPSRecorder):
        replicator_fps_recorder = recorder
        break

if replicator_fps_recorder:
    fps_writer_instance = FPSWriter(annotators=list(annotators_kwargs.keys()))
    fps_writer_instance.attach(render_products)
    replicator_fps_recorder.set_fps_writer(fps_writer_instance)

output_directory = None
if not skip_write:
    writer = rep.writers.get("BasicWriter")
    output_directory = (
        os.getcwd() + f"/_out_sdg_physics_{num_frames}f_{num_cameras}c_{asset_count}a_{len(annotators_kwargs)}ann"
    )
    print(f"[workflow_sdg_physics] Output directory: {output_directory}")
    aw = {k.replace("_fast", ""): v for k, v in annotators_kwargs.items()}
    writer.initialize(output_dir=output_directory, **aw)
    writer.attach(render_products)

if with_functional_randomization:
    random_poses = rng.generator.uniform((-4, -4, 2), (4, 4, 8), size=(len(assets), 3))
    random_rotations = rng.generator.uniform((0, 0, 0), (360, 360, 360), size=(len(assets), 3))
    random_scales = rng.generator.uniform(0.15, 1.0, size=len(assets))
    rep.functional.modify.pose(
        prims=assets, position_value=random_poses, rotation_value=random_rotations, scale_value=random_scales
    )
    rep.functional.randomizer.display_color(assets, rng=rng)
    random_camera_poses = rng.generator.uniform((5, 5, 4), (12, 12, 10), size=(len(cameras), 3))
    rep.functional.modify.pose(
        prims=cameras, position_value=random_camera_poses, look_at_value=(0, 0, 1), look_at_up_axis=(0, 0, 1)
    )
else:
    with rep.trigger.on_frame():
        assets_group = rep.create.group([prim.GetPath() for prim in cubes + cones + cylinders + spheres + tori])
        cameras_group = rep.create.group([cam.GetPath() for cam in cameras])
        with assets_group:
            rep.modify.pose(
                position=rep.distribution.uniform((-4, -4, 2), (4, 4, 8)),
                rotation=rep.distribution.uniform((0, 0, 0), (360, 360, 360)),
                scale=rep.distribution.uniform(0.15, 1.0),
            )
            rep.randomizer.color(rep.distribution.uniform((0, 0, 0), (1, 1, 1)))
        with cameras_group:
            rep.modify.pose(
                position=rep.distribution.uniform((5, 5, 4), (12, 12, 10)),
                look_at=(0, 0, 1),
            )
    rep.orchestrator.preview()

timeline = omni.timeline.get_timeline_interface()
timeline.play()

for _ in range(10):
    omni.kit.app.get_app().update()
benchmark.store_measurements()

benchmark.set_phase("benchmark")
start_time = time.time()
for _i in range(num_frames):
    if with_functional_randomization:
        random_poses = rng.generator.uniform((-4, -4, 2), (4, 4, 8), size=(len(assets), 3))
        random_rotations = rng.generator.uniform((0, 0, 0), (360, 360, 360), size=(len(assets), 3))
        random_scales = rng.generator.uniform(0.15, 1.0, size=len(assets))
        if randomize_drop_height:
            random_poses[:, 2] = rng.generator.uniform(3.0, 12.0, size=len(assets))
        rep.functional.modify.pose(
            prims=assets, position_value=random_poses, rotation_value=random_rotations, scale_value=random_scales
        )
        rep.functional.randomizer.display_color(assets, rng=rng)
        random_camera_poses = rng.generator.uniform((5, 5, 4), (12, 12, 10), size=(len(cameras), 3))
        rep.functional.modify.pose(
            prims=cameras, position_value=random_camera_poses, look_at_value=(0, 0, 1), look_at_up_axis=(0, 0, 1)
        )
    rep.orchestrator.step(rt_subframes=rt_subframes, wait_for_render=wait_for_render)
    omni.kit.app.get_app().update()

end_time = time.time()
benchmark.store_measurements()

if not skip_write:
    rep.orchestrator.wait_until_complete()
if delete_data_when_done and output_directory and os.path.isdir(output_directory):
    shutil.rmtree(output_directory)

if print_results:
    duration = end_time - start_time
    print(f"[workflow_sdg_physics] duration: {duration}s avg FPS: {num_frames / duration:.2f}")

timeline.stop()
benchmark.stop()
simulation_app.close()
