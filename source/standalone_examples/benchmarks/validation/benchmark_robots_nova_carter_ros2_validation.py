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

args, unknown = parser.parse_known_args()

n_robot = args.num_robots
enable_3d_lidar = args.enable_3d_lidar
enable_2d_lidar = args.enable_2d_lidar
enable_hawks = args.enable_hawks
n_gpu = args.num_gpus
n_frames = args.num_frames
gpu_frametime = args.gpu_frametime
headless = args.non_headless


import os
from pathlib import Path

import numpy as np
from isaacsim import SimulationApp

simulation_app = SimulationApp({"headless": headless, "max_gpu_count": n_gpu})

import carb
import omni
import omni.graph.core as og
import omni.kit.test
import omni.replicator.core as rep
from isaacsim.core.api import PhysicsContext
from isaacsim.core.utils.extensions import enable_extension
from isaacsim.core.utils.stage import get_current_stage
from isaacsim.core.utils.viewports import set_camera_view
from isaacsim.robot.wheeled_robots.robots import WheeledRobot
from pxr import Usd

enable_extension("isaacsim.benchmark.services")

from isaacsim.benchmark.services import BaseIsaacBenchmark
from isaacsim.benchmark.services.validation import Validator

# Create the benchmark
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
    gpu_frametime=gpu_frametime,
)


# Generate Twist message
def move_cmd_msg(x, y, z, ax, ay, az):
    msg = Twist()
    msg.linear.x = x
    msg.linear.y = y
    msg.linear.z = z
    msg.angular.x = ax
    msg.angular.y = ay
    msg.angular.z = az
    return msg


benchmark.set_phase("loading", start_recording_frametime=False, start_recording_runtime=True)

enable_extension("isaacsim.ros2.bridge")
import rclpy
from geometry_msgs.msg import Twist

omni.kit.app.get_app().update()

# Create publisher for move commands
rclpy.init()
node = rclpy.create_node("cmd_vel_publisher")
cmd_vel_pub = node.create_publisher(Twist, "cmd_vel", 1)

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

# Set this to true so that we always publish regardless of subscribers
carb.settings.get_settings().set_bool("/exts/isaacsim.ros2.bridge/publish_without_verification", True)

timeline = omni.timeline.get_timeline_interface()
timeline.play()
omni.kit.app.get_app().update()

for robot in robots:
    robot.initialize()
    # start the robot rotating in place so not to run into each
    move_cmd = move_cmd_msg(0.0, 0.0, 0.0, 0.0, 0.0, 1.0)
    cmd_vel_pub.publish(move_cmd)

omni.kit.app.get_app().update()
omni.kit.app.get_app().update()

benchmark.store_measurements()
# perform benchmark
benchmark.set_phase("benchmark")

for _ in range(1, n_frames):
    omni.kit.app.get_app().update()

benchmark.store_measurements()

node.destroy_node()
rclpy.shutdown()
benchmark.stop()

# validate benchmark
validator = Validator.from_cli_args(args)
passed = validator.run(stage, benchmark_name=benchmark.benchmark_name)

timeline.stop()
simulation_app.close()
