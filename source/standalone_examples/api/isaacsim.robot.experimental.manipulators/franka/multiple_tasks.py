# SPDX-FileCopyrightText: Copyright (c) 2021-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

"""Demonstrate multi-robot Franka pick-and-place operations."""

from __future__ import annotations

"""
This example demonstrates how to run two Franka robots performing pick-and-place operations.

The example serves to illustrate the following concepts:
- How to set up multiple Franka pick-and-place tasks with unique robot and cube prims.
- How to configure physics simulation device (CPU or CUDA).
- How to use different inverse kinematics methods for robot control.
- How to drive multiple pick-and-place instances in a single simulation loop.

The source code is organized into 3 main sections:
1. Command-line argument parsing and SimulationApp launch (common to all standalone examples).
2. Scene setup and initialization for each robot.
3. Example logic and simulation loop.
"""

# 1. --------------------------------------------------------------------

# Parse any command-line arguments specific to the standalone application (only known arguments).
import argparse

import numpy as np

parser = argparse.ArgumentParser()
parser.add_argument("--test", action="store_true", help="Run in test mode (exit after task completes)")
parser.add_argument("--device", type=str, choices=["cpu", "cuda"], default="cpu", help="Simulation device")
parser.add_argument(
    "--ik-method",
    type=str,
    choices=["singular-value-decomposition", "pseudoinverse", "transpose", "damped-least-squares"],
    default="damped-least-squares",
    help="Differential inverse kinematics method",
)
args, _ = parser.parse_known_args()

# Launch the `SimulationApp` (see DEFAULT_LAUNCHER_CONFIG for available configuration):
# https://docs.isaacsim.omniverse.nvidia.com/latest/py/source/extensions/isaacsim.simulation_app/docs/index.html
from isaacsim import SimulationApp

simulation_app = SimulationApp({"headless": False})
import omni.kit.app

omni.kit.app.get_app().get_extension_manager().set_extension_enabled_immediate(
    "isaacsim.robot.experimental.manipulators.examples", True
)

# Any Omniverse level imports must occur after the `SimulationApp` class is instantiated (because APIs are provided
# by the extension/runtime plugin system, it must be loaded before they will be available to import).
import omni.timeline
from isaacsim.core.simulation_manager import SimulationManager
from isaacsim.robot.experimental.manipulators.examples.franka import FrankaPickPlace

# 2. --------------------------------------------------------------------

# Setup scene programmatically:
# - Configure physics simulation device (CPU or CUDA)
SimulationManager.set_physics_sim_device(args.device)
simulation_app.update()

# - Create multiple Franka pick-and-place tasks (two robots, each with its own cube and target)
num_of_tasks = 2
pick_place_tasks = []
for i in range(num_of_tasks):
    offset = np.array([0.0, (i * 2) - 3, 0.0])
    robot_path = f"/World/robot_{i}"
    cube_path = f"/World/Cube_{i}"
    task = FrankaPickPlace(robot_name=f"Robot {i}")
    task.setup_scene(offset=offset, robot_path=robot_path, cube_path=cube_path)
    pick_place_tasks.append(task)

# 3. --------------------------------------------------------------------

# Start timeline for physics simulation.
omni.timeline.get_timeline_interface().play()
simulation_app.update()  # - Allow physics to initialize

# Initialize task state tracking (per-robot reset and completion).
reset_needed = True
task_completed = [False] * num_of_tasks

print("Starting multi-robot pick-and-place execution")
# Run simulation loop until all tasks complete or application shutdown.
while simulation_app.is_running():
    # - Check if simulation is running
    if SimulationManager.is_simulating():
        # - Reset all tasks on first iteration
        if reset_needed:
            for i in range(num_of_tasks):
                pick_place_tasks[i].reset()
            reset_needed = False

        # - Execute one step of pick-and-place for each robot that is not yet done
        for i in range(num_of_tasks):
            if not task_completed[i]:
                pick_place_tasks[i].forward(args.ik_method)

    # - Check completion for each task and print when done
    for i in range(num_of_tasks):
        if pick_place_tasks[i].is_done() and not task_completed[i]:
            print(f"Robot {i}: done picking and placing")
            task_completed[i] = True

    if args.test and all(task_completed):
        break

    # - Update simulation
    simulation_app.update()

# Close the `SimulationApp`.
simulation_app.close()
