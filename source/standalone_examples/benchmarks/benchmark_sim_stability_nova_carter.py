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

"""Benchmark Nova Carter ROS rig for simulation RTF stability (windowed sim vs wall clock).

Uses the same Nova_Carter_ROS sensor setup as benchmark_robots_nova_carter_ros2.py but
drives the robot with wheel articulation. RTF is measured by isaacsim.benchmark.services
``rtf_stability`` / ``app_frametime`` recorders.
"""

import argparse

parser = argparse.ArgumentParser()
parser.add_argument("--num-robots", type=int, default=1, help="Number of robots")
parser.add_argument(
    "--enable-3d-lidar", type=int, default=0, choices=range(0, 1 + 1), help="Number of 3D lidars to enable, per robot."
)
parser.add_argument(
    "--enable-2d-lidar",
    type=int,
    default=2,
    choices=range(0, 2 + 1),
    help="Number of 2D lidars to enable, per robot (default 2 for customer-style workload).",
)
parser.add_argument(
    "--enable-hawks",
    type=int,
    default=2,
    choices=range(0, 4 + 1),
    help="Number of Hawk camera stereo pairs to enable, per robot (default 2).",
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
parser.add_argument(
    "--async-render-handshake", action="store_true", help="Run with async rendering and handshake enabled"
)
parser.add_argument(
    "--tick-rate",
    type=float,
    default=10.0,
    help="Tick rate for camera sensors (Hz). 0.0 means default rate.",
)
parser.add_argument(
    "--rtf-window-ms",
    type=float,
    default=100.0,
    help="Wall-clock window size (ms) for each RTF stability sample (carb setting).",
)
parser.add_argument(
    "--rtf-export-samples",
    action="store_true",
    help="If set, emit ListMeasurement of all windowed RTF samples (larger KPI output).",
)
parser.add_argument(
    "--physics-dt",
    type=float,
    default=1.0 / 60.0,
    help="Physics simulation timestep in seconds (SimulationManager.setup_simulation dt). Default: 1/60 s.",
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
viewport_updates = args.viewport_updates
async_render_handshake = args.async_render_handshake
tick_rate = args.tick_rate
rtf_window_ms = args.rtf_window_ms
rtf_export_samples = args.rtf_export_samples
physics_dt = args.physics_dt
if physics_dt <= 0.0:
    raise SystemExit("--physics-dt must be positive")

extra_args = []
if async_render_handshake:
    async_render_handshake_args = [
        "--/app/asyncRendering=true",
        "--/app/omni.usd/asyncHandshake=true",
        "--/omni/replicator/asyncRendering=true",
    ]
    extra_args.extend(async_render_handshake_args)

import numpy as np
from isaacsim import SimulationApp

simulation_app = SimulationApp(
    {
        "headless": headless,
        "max_gpu_count": n_gpu,
        "disable_viewport_updates": viewport_updates,
        "extra_args": extra_args,
    }
)

import carb
import omni
import omni.graph.core as og
import omni.kit.test
from isaacsim.core.experimental.utils.stage import get_current_stage
from isaacsim.core.rendering_manager import RenderingManager
from isaacsim.core.simulation_manager import SimulationManager
from isaacsim.core.utils.extensions import enable_extension
from isaacsim.core.utils.types import ArticulationAction
from isaacsim.core.utils.viewports import set_camera_view
from isaacsim.robot.wheeled_robots.robots import WheeledRobot
from pxr import Usd, UsdGeom

carb.settings.get_settings().set_float(
    "/exts/isaacsim.benchmark.services/rtf_stability/window_wall_ms", float(rtf_window_ms)
)
carb.settings.get_settings().set_bool(
    "/exts/isaacsim.benchmark.services/rtf_stability/export_window_samples", bool(rtf_export_samples)
)

enable_extension("isaacsim.benchmark.services")

from isaacsim.benchmark.services import DEFAULT_RECORDERS, BaseIsaacBenchmark

recorders = list(DEFAULT_RECORDERS) + ["rtf_stability", "physics_step_interval"]
if gpu_frametime:
    recorders.append("gpu_frametime")

benchmark = BaseIsaacBenchmark(
    benchmark_name="benchmark_sim_stability_nova_carter",
    workflow_metadata={
        "metadata": [
            {"name": "num_hawks", "data": enable_hawks},
            {"name": "num_2d_lidars", "data": enable_2d_lidar},
            {"name": "num_3d_lidars", "data": enable_3d_lidar},
            {"name": "num_robots", "data": n_robot},
            {"name": "tick_rate_hz", "data": tick_rate},
            {"name": "rtf_window_wall_ms", "data": rtf_window_ms},
            {"name": "rtf_export_window_samples", "data": rtf_export_samples},
            {"name": "physics_dt_s", "data": physics_dt},
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

with Usd.EditContext(get_current_stage(), get_current_stage().GetRootLayer()):
    get_current_stage().SetEndTimeCode(1000000.0)

stage = omni.usd.get_context().get_stage()
SimulationManager.setup_simulation(dt=physics_dt)
RenderingManager.set_dt(1.0 / 60.0)
set_camera_view(eye=[-6, -15.5, 6.5], target=[-6, 10.5, -1], camera_prim_path="/OmniverseKit_Persp")

lidars_2d = ["/front_2d_lidar_render_product", "/back_2d_lidar_render_product"]
hawk_actiongraphs = ["/front_hawk", "/left_hawk", "/right_hawk", "/back_hawk"]

robots = []
for robot_idx in range(n_robot):
    robot_prim_path = "/Robots/Robot_" + str(robot_idx)
    robot_usd_path = benchmark.assets_root_path + robot_path
    MAX_IN_LINE = 10
    robot_position = np.array([-2 * (robot_idx % MAX_IN_LINE), -2 * np.floor(robot_idx / MAX_IN_LINE), 0])
    current_robot = WheeledRobot(
        prim_path=robot_prim_path,
        wheel_dof_names=["joint_wheel_left", "joint_wheel_right"],
        create_robot=True,
        usd_path=robot_usd_path,
        position=robot_position,
    )

    omni.kit.app.get_app().update()
    omni.kit.app.get_app().update()

    for lid_idx in range(len(lidars_2d)):
        if lid_idx < enable_2d_lidar:
            og.Controller.attribute(robot_prim_path + "/ros_lidars" + lidars_2d[lid_idx] + ".inputs:enabled").set(True)
        else:
            og.Controller.attribute(robot_prim_path + "/ros_lidars" + lidars_2d[lid_idx] + ".inputs:enabled").set(False)

    if enable_3d_lidar > 0:
        og.Controller.attribute(robot_prim_path + "/ros_lidars/front_3d_lidar_render_product.inputs:enabled").set(True)
    else:
        og.Controller.attribute(robot_prim_path + "/ros_lidars/front_3d_lidar_render_product.inputs:enabled").set(False)

    for hawk_idx in range(len(hawk_actiongraphs)):
        if hawk_idx < enable_hawks:
            og.Controller.attribute(
                robot_prim_path + hawk_actiongraphs[hawk_idx] + "/left_camera_render_product" + ".inputs:enabled"
            ).set(True)
            og.Controller.attribute(
                robot_prim_path + hawk_actiongraphs[hawk_idx] + "/right_camera_render_product" + ".inputs:enabled"
            ).set(True)
        else:
            og.Controller.attribute(
                robot_prim_path + hawk_actiongraphs[hawk_idx] + "/left_camera_render_product" + ".inputs:enabled"
            ).set(False)
            og.Controller.attribute(
                robot_prim_path + hawk_actiongraphs[hawk_idx] + "/right_camera_render_product" + ".inputs:enabled"
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

carb.settings.get_settings().set_bool("/exts/isaacsim.ros2.bridge/publish_without_verification", True)

timeline = omni.timeline.get_timeline_interface()
timeline.play()
omni.kit.app.get_app().update()

for robot in robots:
    robot.initialize()
    robot.apply_wheel_actions(
        ArticulationAction(joint_positions=None, joint_efforts=None, joint_velocities=5 * np.array([0, 1]))
    )

omni.kit.app.get_app().update()
omni.kit.app.get_app().update()


benchmark.store_measurements()
benchmark.set_phase("benchmark", warmup_frames=15)

for _ in range(1, n_frames):
    omni.kit.app.get_app().update()

benchmark.store_measurements()
benchmark.stop()

timeline.stop()
simulation_app.close()
