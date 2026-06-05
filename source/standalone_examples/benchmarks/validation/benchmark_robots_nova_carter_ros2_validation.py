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

"""Validate Nova Carter ROS2 benchmark results against golden data."""

import argparse
import os

_VALIDATION_DIR = os.path.dirname(os.path.realpath(__file__))
_DEFAULT_GOLDEN_DIR = os.path.join(_VALIDATION_DIR, "golden_data")
_DEFAULT_CAPTURES_DIR = os.path.join(_VALIDATION_DIR, "captures")

parser = argparse.ArgumentParser()
parser.add_argument("--num-robots", type=int, default=1, help="Number of robots")
parser.add_argument(
    "--enable-3d-lidar", type=int, default=0, choices=range(0, 1 + 1), help="Number of 3D lidars to enable, per robot."
)
parser.add_argument(
    "--enable-2d-lidar", type=int, default=0, choices=range(0, 2 + 1), help="Number of 2D lidars to enable, per robot."
)
parser.add_argument(
    "--enable-hawks",
    type=int,
    default=4,
    choices=range(0, 4 + 1),
    help="Number of Hawk camera stereo pairs to enable, per robot.",
)
parser.add_argument("--num-gpus", type=int, default=None, help="Number of GPUs on machine.")
parser.add_argument("--num-frames", type=int, default=600, help="Number of frames to run benchmark for")
parser.add_argument("--gpu-frametime", action="store_true", help="Enable GPU frametime measurement")
parser.add_argument("--non-headless", action="store_false", help="Run with GUI - nonheadless mode")
parser.add_argument(
    "--backend-type",
    default="OmniPerfKPIFile",
    choices=["LocalLogMetrics", "JSONFileMetrics", "OsmoKPIFile", "OmniPerfKPIFile"],
    help="Benchmarking backend, defaults",
)

parser.add_argument(
    "--golden-dir",
    default=_DEFAULT_GOLDEN_DIR,
    help="Directory holding golden images (default: validation/golden_data next to this script)",
)
parser.add_argument(
    "--output-dir",
    default=_DEFAULT_CAPTURES_DIR,
    help="Directory for captured images (default: validation/captures next to this script)",
)
parser.add_argument("--tolerance", type=int, default=10, help="Tolerance for mean difference in image comparison")
parser.add_argument(
    "--blur-kernel",
    type=int,
    default=3,
    help="Apply Gaussian blur with this kernel size before comparison (0=disabled)",
)
parser.add_argument(
    "--regenerate-golden",
    action="store_true",
    help="Regenerate golden images from current run. WARNING: This will overwrite existing golden images.",
)
parser.add_argument(
    "--exit-on-fail",
    action="store_true",
    help="Return non-zero exit code when image validation fails",
)
parser.add_argument(
    "--use-timestamp-matching",
    action="store_true",
    help="Encode simulation time in captured filenames and validate by matching timestamps to golden.",
)
parser.add_argument(
    "--timestamp-tolerance",
    type=float,
    default=0.01,
    help="Max simulation-time difference (seconds) when matching captured to golden (default: 0.01)",
)

parser.add_argument(
    "--async-render-handshake", action="store_true", help="Run with async rendering and handshake enabled"
)
parser.add_argument(
    "--tick-rate", type=float, default=0.0, help="Tick rate for camera sensors (Hz). 0.0 means default rate."
)

args, unknown = parser.parse_known_args()

n_robot = args.num_robots
enable_3d_lidar = args.enable_3d_lidar
enable_2d_lidar = args.enable_2d_lidar
enable_hawks = args.enable_hawks
n_gpu = args.num_gpus
n_frames = args.num_frames
gpu_frametime = args.gpu_frametime
headless = args.non_headless
async_render_handshake = args.async_render_handshake
tick_rate = args.tick_rate

extra_args = []
if async_render_handshake:
    async_render_handshake_args = [
        "--/app/asyncRendering=true",
        "--/app/omni.usd/asyncHandshake=true",
        "--/omni/replicator/asyncRendering=true",
    ]
    extra_args.extend(async_render_handshake_args)


import shutil
from datetime import datetime

import numpy as np
from isaacsim import SimulationApp

simulation_app = SimulationApp({"headless": headless, "max_gpu_count": n_gpu, "extra_args": extra_args})

import carb
import omni
import omni.graph.core as og
import omni.kit.test
from isaacsim.core.api import PhysicsContext
from isaacsim.core.utils.extensions import enable_extension
from isaacsim.core.utils.stage import get_current_stage
from isaacsim.core.utils.viewports import set_camera_view
from isaacsim.robot.wheeled_robots.robots import WheeledRobot
from pxr import Gf, Usd, UsdGeom

enable_extension("isaacsim.benchmark.services")

import omni.replicator.core as rep
from isaacsim.benchmark.services import DEFAULT_RECORDERS, BaseIsaacBenchmark
from isaacsim.benchmark.services.validation import Validator
from omni.replicator.core import functional as F
from omni.replicator.core.scripts.writers_default.basicwriter import BasicWriter


class TimestampedBasicWriter(BasicWriter):
    """BasicWriter subclass that uses the timeline simulation time as the
    filename instead of an incrementing frame counter."""

    def write(self, data):
        self._ref_time_sec = omni.timeline.get_timeline_interface().get_current_time()
        super().write(data)

    def _write_rgb(self, anno_rp_data, output_path):
        file_path = f"{output_path}rgb_{self._ref_time_sec:.6f}.{self._image_output_format}"
        self._backend.schedule(F.write_image, data=anno_rp_data["data"], path=file_path)


rep.WriterRegistry.register(TimestampedBasicWriter)

# Create the benchmark
recorders = DEFAULT_RECORDERS + ["gpu_frametime"] if gpu_frametime else DEFAULT_RECORDERS
benchmark = BaseIsaacBenchmark(
    benchmark_name="benchmark_robots_nova_carter_ros2",
    workflow_metadata={
        "metadata": [
            {"name": "num_hawks", "data": enable_hawks},
            {"name": "num_2d_lidars", "data": enable_2d_lidar},
            {"name": "num_3d_lidars", "data": enable_3d_lidar},
            {"name": "num_robots", "data": n_robot},
            {"name": "num_gpus", "data": carb.settings.get_settings().get("/renderer/multiGpu/currentGpuCount")},
        ]
    },
    backend_type=args.backend_type,
    recorders=recorders,
)


benchmark.set_phase("loading", start_recording_frametime=False, start_recording_runtime=True)

enable_extension("isaacsim.ros2.bridge")

omni.kit.app.get_app().update()

robot_path = "/Isaac/Samples/ROS2/Robots/Nova_Carter_ROS.usd"
scene_path = "/Isaac/Environments/Simple_Warehouse/full_warehouse.usd"

benchmark.fully_load_stage(benchmark.assets_root_path + scene_path)

# NOTE: Modify endtimecode to prevent step skipping errors
with Usd.EditContext(get_current_stage(), get_current_stage().GetRootLayer()):
    get_current_stage().SetEndTimeCode(1000000.0)

stage = omni.usd.get_context().get_stage()
PhysicsContext(physics_dt=1.0 / 60.0)
set_camera_view(eye=[-6, -15.5, 6.5], target=[-6, 10.5, -1], camera_prim_path="/OmniverseKit_Persp")

lidars_2d = ["/front_2d_lidar_render_product", "/back_2d_lidar_render_product"]
hawk_actiongraphs = ["/front_hawk", "/left_hawk", "/right_hawk", "/back_hawk"]

robots = []
for i in range(n_robot):
    robot_prim_path = "/Robots/Robot_" + str(i)
    robot_usd_path = benchmark.assets_root_path + robot_path
    # position the robot robot
    MAX_IN_LINE = 10
    robot_position = np.array([-2 * (i % MAX_IN_LINE), -2 * np.floor(i / MAX_IN_LINE), 0])
    current_robot = WheeledRobot(
        prim_path=robot_prim_path,
        wheel_dof_names=["joint_wheel_left", "joint_wheel_right"],
        create_robot=True,
        usd_path=robot_usd_path,
        position=robot_position,
    )

    omni.kit.app.get_app().update()
    omni.kit.app.get_app().update()

    for i in range(len(lidars_2d)):
        if i < enable_2d_lidar:
            og.Controller.attribute(robot_prim_path + "/ros_lidars" + lidars_2d[i] + ".inputs:enabled").set(True)
        else:
            og.Controller.attribute(robot_prim_path + "/ros_lidars" + lidars_2d[i] + ".inputs:enabled").set(False)

    if enable_3d_lidar > 0:
        og.Controller.attribute(robot_prim_path + "/ros_lidars/front_3d_lidar_render_product.inputs:enabled").set(True)
    else:
        og.Controller.attribute(robot_prim_path + "/ros_lidars/front_3d_lidar_render_product.inputs:enabled").set(False)

    for i in range(len(hawk_actiongraphs)):
        if i < enable_hawks:
            og.Controller.attribute(
                robot_prim_path + hawk_actiongraphs[i] + "/left_camera_render_product" + ".inputs:enabled"
            ).set(True)
            og.Controller.attribute(
                robot_prim_path + hawk_actiongraphs[i] + "/right_camera_render_product" + ".inputs:enabled"
            ).set(True)
        else:
            og.Controller.attribute(
                robot_prim_path + hawk_actiongraphs[i] + "/left_camera_render_product" + ".inputs:enabled"
            ).set(False)
            og.Controller.attribute(
                robot_prim_path + hawk_actiongraphs[i] + "/right_camera_render_product" + ".inputs:enabled"
            ).set(False)

    robots.append(current_robot)

if tick_rate > 0:
    for robot_idx in range(n_robot):
        robot_prim_path = "/Robots/Robot_" + str(robot_idx)
        robot_prim = stage.GetPrimAtPath(robot_prim_path)
        for prim in Usd.PrimRange(robot_prim):
            if prim.IsA(UsdGeom.Camera):
                prim.ApplyAPI("OmniSensorAPI")
                prim.GetAttribute("omni:sensor:tickRate").Set(tick_rate)

# Set this to true so that we always publish regardless of subscribers
carb.settings.get_settings().set_bool("/exts/isaacsim.ros2.bridge/publish_without_verification", True)


timeline = omni.timeline.get_timeline_interface()
timeline.play()
omni.kit.app.get_app().update()

robot_initial_poses = []
for robot in robots:
    robot.initialize()
    pos, orient = robot.get_world_pose()
    robot_initial_poses.append((pos, orient))

omni.kit.app.get_app().update()
omni.kit.app.get_app().update()

ROTATION_HZ = 1.0
ROTATION_DEG_PER_SEC = ROTATION_HZ * 360.0
PHYSICS_DT = 1.0 / 60.0

benchmark.store_measurements()
# perform benchmark
benchmark.set_phase("benchmark", warmup_frames=15)

# Setup image validator
validator = Validator.from_cli_args(args, auto_cleanup=False)
validator.build_render_product_map(stage)

# Convert paths to absolute
golden_dir_abs = os.path.abspath(args.golden_dir) if not os.path.isabs(args.golden_dir) else args.golden_dir
output_dir_abs = os.path.abspath(args.output_dir) if not os.path.isabs(args.output_dir) else args.output_dir

# Create run directory for all frames
timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
if args.regenerate_golden:
    run_dir = os.path.join(golden_dir_abs, benchmark.benchmark_name)
    if os.path.isdir(run_dir):
        shutil.rmtree(run_dir)
    os.makedirs(run_dir, exist_ok=True)
else:
    run_dir = os.path.join(output_dir_abs, f"{benchmark.benchmark_name}_{timestamp}")
    os.makedirs(run_dir, exist_ok=True)

print(f"\n{'='*80}")
print("CAPTURING FRAMES")
print(f"{'='*80}")
print(f"Run directory: {run_dir}\n")

# Create one writer per render product so each gets its own RationalTimeSyncGate.
# A single writer attached to all RPs blocks under multitick because the sync gate
# waits for every annotator to fire at the same rational time, which never happens
# when cameras tick at different rates.
# Use the camera prim path from render_product_map directly as the output subdirectory
# so that the directory tree matches the golden data layout without any post-hoc rename.
writer_type = "TimestampedBasicWriter" if args.use_timestamp_matching else "BasicWriter"
writers = {}
for rp_path, camera_path in validator.render_product_map.items():
    w = rep.WriterRegistry.get(writer_type)
    w.initialize(output_dir=os.path.join(run_dir, camera_path, "rgb"), rgb=True)
    w.attach([rp_path])
    writers[rp_path] = w

frame_timestamps = []
for frame_idx in range(0, n_frames):
    angle_deg = ROTATION_DEG_PER_SEC * (frame_idx + 1) * PHYSICS_DT
    yaw_rotation = Gf.Rotation(Gf.Vec3d(0, 0, 1), angle_deg)
    for robot, (init_pos, init_orient) in zip(robots, robot_initial_poses):
        init_quat = Gf.Quatd(float(init_orient[0]), Gf.Vec3d(*[float(x) for x in init_orient[1:]]))
        new_quat = yaw_rotation.GetQuat() * init_quat
        new_orient = np.array([new_quat.GetReal(), *new_quat.GetImaginary()])
        robot.set_world_pose(position=init_pos, orientation=new_orient)
    rep.orchestrator.step(wait_for_render=True)
    # omni.kit.app.get_app().update()
    if args.use_timestamp_matching:
        frame_timestamps.append(timeline.get_current_time())
print(f"*** DONE ***")

# WAR Disable timeline + replicator sync
rep.orchestrator.set_capture_on_play(False)
rep.orchestrator.stop()
rep.orchestrator.wait_until_complete()
timeline.stop()
print(f"*** AFTER wait_until_complete ***")
total_written = sum(w.num_written or 0 for w in writers.values())
print(f"CAPTURED {total_written} frames total across {len(writers)} writers")
for rp_path, w in writers.items():
    print(f"  {rp_path}: {w.num_written} frames")
for w in writers.values():
    w.detach()
print(f"*** AFTER DETACH ***")

# When not using TimestampedBasicWriter, fall back to post-hoc timestamp renaming
if args.use_timestamp_matching and frame_timestamps and writer_type == "BasicWriter":
    validator.rename_captured_frames_to_timestamps(run_dir, frame_timestamps)

benchmark.store_measurements()
benchmark.stop()

validate = True
if validate:
    if args.regenerate_golden:
        print(f"\n{'='*80}")
        print("GOLDEN IMAGES REGENERATED")
        print(f"{'='*80}")
        print(f"Location: {run_dir}")
        print(f"Total frames: {n_frames-1}")
        print(f"{'='*80}")
        overall_validation_passed = True
    else:
        if args.use_timestamp_matching:
            validation_results = validator.validate_frames_by_timestamp(
                captured_run_dir=run_dir,
                golden_benchmark_dir=os.path.join(golden_dir_abs, benchmark.benchmark_name),
                time_tolerance_sec=args.timestamp_tolerance,
            )
        else:
            validation_results = validator.validate_frames(
                captured_run_dir=run_dir,
                golden_benchmark_dir=os.path.join(golden_dir_abs, benchmark.benchmark_name),
            )

        total_frames = validation_results["total"]
        passed_count = validation_results["passed"]
        failed_count = validation_results["failed"]

        print(f"\n{'='*80}")
        print("FINAL VALIDATION SUMMARY")
        print(f"{'='*80}")
        print(f"Total Frames: {total_frames}")
        print(f"Passed: {passed_count}")
        print(f"Failed: {failed_count}")

        overall_validation_passed = failed_count == 0

        if overall_validation_passed:
            print(f"\nOVERALL RESULT: PASS")
        else:
            print(f"\nOVERALL RESULT: FAIL")

        print(f"{'='*80}")

timeline.stop()
simulation_app.close()

# Exit with appropriate code if validation failed and exit_on_fail is set
if args.exit_on_fail and not overall_validation_passed:
    exit(1)
