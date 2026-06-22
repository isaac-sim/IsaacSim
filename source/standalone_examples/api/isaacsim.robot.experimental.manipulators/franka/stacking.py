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

"""This example demonstrates how to perform a stacking operation using the Franka robot arm.

The example serves to illustrate the following concepts:
- How to set up a Franka robot and multiple cubes for a stacking task.
- How to execute a multi-phase stacking sequence (pick cubes and stack them) in a simulation loop.

The source code is organized into 3 main sections:
1. Command-line argument parsing and SimulationApp launch (common to all standalone examples).
2. Scene setup and initialization.
3. Example logic and simulation loop.
"""

from __future__ import annotations

# Parse any command-line arguments specific to the standalone application (only known arguments).
import argparse

# 1. --------------------------------------------------------------------


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
from isaacsim.robot.experimental.manipulators.examples.franka.stacking import Stacking

# 2. --------------------------------------------------------------------

# Setup scene programmatically:
# - Configure physics simulation device (CPU or CUDA)
SimulationManager.set_physics_sim_device(args.device)
simulation_app.update()

# - Create and initialize the stacking task (robot, ground, lights, and cubes)
stacking = Stacking()
stacking.setup_scene()

# 3. --------------------------------------------------------------------

# Start timeline for physics simulation.
omni.timeline.get_timeline_interface().play()
simulation_app.update()  # - Allow physics to initialize

# Initialize task state tracking.
reset_needed = True
task_completed = False

print("Starting stacking execution")
# Run simulation loop until task completion or application shutdown.
while simulation_app.is_running():
    # - Check if simulation is running and task is not yet completed
    if SimulationManager.is_simulating() and not task_completed:
        # - Reset the task on first iteration
        if reset_needed:
            stacking.reset()
            reset_needed = False

        # - Execute one step of the stacking operation using the specified IK method
        stacking.forward(args.ik_method)

    # - Check if task is completed and print completion message
    if stacking.is_done() and not task_completed:
        print("Done stacking")
        task_completed = True
        if args.test:
            break

    # - Update simulation
    simulation_app.update()

# Close the `SimulationApp`.
simulation_app.close()
