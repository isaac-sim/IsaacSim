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

"""This example demonstrates how to run a UR10 follow-target task.

The example serves to illustrate the following concepts:
- How to set up a UR10 robot for following a target pose with inverse kinematics.
- How to configure physics simulation device (CPU or CUDA).
- How to use different inverse kinematics methods for robot control.
- How to run a continuous follow-target loop (move the target cube in the viewport to drive the robot).

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
parser.add_argument("--test", action="store_true", help="Run in test mode (exit after N frames)")
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
from isaacsim.robot.experimental.manipulators.examples.universal_robots import UR10FollowTarget

# 2. --------------------------------------------------------------------

# Setup scene programmatically:
# - Configure physics simulation device (CPU or CUDA)
SimulationManager.set_physics_sim_device(args.device)
simulation_app.update()

# - Create and initialize the UR10 follow-target task
follow_target = UR10FollowTarget()
follow_target.setup_scene()

print("UR10 Follow Target Example")
print("=" * 40)
print("\nInstructions:")
print("- The robot will try to follow the red target cube")
print("- You can move the target cube in the viewport by selecting it")
print("- Press Ctrl+C to exit")
print("=" * 40)

# 3. --------------------------------------------------------------------

# Start timeline for physics simulation.
omni.timeline.get_timeline_interface().play()
simulation_app.update()  # - Allow physics to initialize

# Initialize task state tracking.
reset_needed = True
frame_count = 0

# Run simulation loop until application shutdown.
while simulation_app.is_running():
    # - Check if simulation is running
    if SimulationManager.is_simulating():
        # - Reset the robot on first iteration
        if reset_needed:
            follow_target.reset_robot()
            print("Robot reset to default pose")
            reset_needed = False

        # - Move robot towards target using the specified IK method
        follow_target.move_to_target(ik_method=args.ik_method)
        frame_count += 1

    if args.test and frame_count >= 10:
        break

    # - Update simulation
    simulation_app.update()

# Close the `SimulationApp`.
simulation_app.close()
