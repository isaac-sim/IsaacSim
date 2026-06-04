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

"""Demonstrate UR robot follow-target using experimental IK solver."""

from __future__ import annotations

import argparse

parser = argparse.ArgumentParser()
parser.add_argument("--test", default=False, action="store_true", help="Run in test mode")
parser.add_argument("--device", type=str, choices=["cpu", "cuda"], default="cpu", help="Simulation device")
parser.add_argument(
    "--ik_method",
    type=str,
    default="damped-least-squares",
    choices=["damped-least-squares", "pseudoinverse", "transpose", "singular-value-decomposition"],
    help="Inverse kinematics method",
)

args, _ = parser.parse_known_args()

from isaacsim import SimulationApp

simulation_app = SimulationApp(
    {
        "headless": False,
        "extra_args": [
            "--enable",
            "isaacsim.robot.manipulators.examples",
            "--enable",
            "isaacsim.robot.experimental.manipulators.examples",
        ],
    }
)

import omni.timeline
from isaacsim.core.simulation_manager import SimulationManager
from isaacsim.robot.experimental.manipulators.examples.universal_robots import UR10FollowTarget


def main():
    """Main function to run the UR10 follow target example."""
    # Set physics simulation device
    SimulationManager.set_physics_sim_device(args.device)
    simulation_app.update()

    # Create the follow target task
    follow_target = UR10FollowTarget()

    # Set up the scene
    follow_target.setup_scene(
        # target_position=[0.4, 0.2, 0.3]  # Initial target position
    )

    print("UR10 Follow Target Example")
    print("=" * 40)
    print("\nInstructions:")
    print("- The robot will try to follow the red target cube")
    print("- You can move the target cube in the viewport by selecting it")
    print("- Press Ctrl+C to exit")
    print("=" * 40)

    # Play the simulation
    omni.timeline.get_timeline_interface().play()
    simulation_app.update()

    # Initialize variables
    reset_needed = True

    # Main simulation loop
    frame_count = 0
    while simulation_app.is_running():
        if SimulationManager.is_simulating():
            if reset_needed:
                # Reset robot to default pose
                follow_target.reset_robot()
                print("Robot reset to default pose")
                reset_needed = False

            # Move robot towards target using IK
            follow_target.move_to_target(ik_method=args.ik_method)

        # Update simulation
        simulation_app.update()
        frame_count += 1
        if args.test and frame_count >= 10:
            break


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\nExiting...")
    except Exception as e:
        print(f"Error: {e}")
    finally:
        simulation_app.close()
