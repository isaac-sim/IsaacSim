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

"""Benchmark EvoBot robot simulation performance."""

import argparse

parser = argparse.ArgumentParser()
parser.add_argument("--num-robots", nargs="+", type=int, default=[1, 10, 20], help="Number of robots per phase")
parser.add_argument("--num-gpus", type=int, default=None, help="Number of GPUs on machine.")
parser.add_argument("--num-frames", type=int, default=600, help="Number of frames to run benchmark for")
parser.add_argument(
    "--backend-type",
    default="OmniPerfKPIFile",
    choices=["LocalLogMetrics", "JSONFileMetrics", "OsmoKPIFile", "OmniPerfKPIFile"],
    help="Benchmarking backend, defaults",
)

args, unknown = parser.parse_known_args()

n_robot = args.num_robots
n_gpu = args.num_gpus
n_frames = args.num_frames

import numpy as np
from isaacsim import SimulationApp

simulation_app = SimulationApp({"headless": True, "max_gpu_count": n_gpu})

import carb
import omni
import omni.kit.test
from isaacsim.core.api import PhysicsContext
from isaacsim.core.utils.extensions import enable_extension
from isaacsim.core.utils.types import ArticulationAction
from isaacsim.robot.wheeled_robots.robots import WheeledRobot

enable_extension("isaacsim.benchmark.services")
from isaacsim.benchmark.services import BaseIsaacBenchmark

# Create the benchmark
benchmark = BaseIsaacBenchmark(
    benchmark_name="benchmark_robots_evobot",
    workflow_metadata={
        "metadata": [
            {"name": "num_robots", "data": n_robot},
            {"name": "num_gpus", "data": carb.settings.get_settings().get("/renderer/multiGpu/currentGpuCount")},
        ]
    },
    backend_type=args.backend_type,
)
robot_path = "/Isaac/Robots/Fraunhofer/Evobot/evobot.usd"
scene_path = "/Isaac/Environments/Simple_Warehouse/full_warehouse.usd"
benchmark.fully_load_stage(benchmark.assets_root_path + scene_path)
stage = omni.usd.get_context().get_stage()
PhysicsContext(physics_dt=1.0 / 60.0)
timeline = omni.timeline.get_timeline_interface()

for num_robot in n_robot:
    benchmark.set_phase(f"loading_{num_robot}_robots", start_recording_frametime=False, start_recording_runtime=True)

    robots = []
    for i in range(int(num_robot)):
        robot_prim_path = "/Robots/Robot_" + str(i)
        robot_usd_path = benchmark.assets_root_path + robot_path
        # position the robot
        MAX_IN_LINE = 10
        robot_position = np.array([-2 * (i % MAX_IN_LINE), -2 * np.floor(i / MAX_IN_LINE), 0])
        current_robot = WheeledRobot(
            prim_path=robot_prim_path,
            wheel_dof_names=["left_wheel_joint", "right_wheel_joint"],
            create_robot=True,
            usd_path=robot_usd_path,
            position=robot_position,
        )

        omni.kit.app.get_app().update()
        omni.kit.app.get_app().update()

        robots.append(current_robot)

    timeline.play()
    omni.kit.app.get_app().update()

    for robot in robots:
        robot.initialize()
        # start the robot rotating in place
        robot.apply_wheel_actions(
            ArticulationAction(joint_positions=None, joint_efforts=None, joint_velocities=5 * np.array([1, -1]))
        )

    omni.kit.app.get_app().update()
    omni.kit.app.get_app().update()

    benchmark.store_measurements()
    # Perform benchmark
    benchmark.set_phase(f"benchmark_{num_robot}_robots")

    for _ in range(1, n_frames):
        omni.kit.app.get_app().update()

    benchmark.store_measurements()
    timeline.stop()

benchmark.stop()
simulation_app.close()
