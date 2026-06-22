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

"""Demonstrate Franka robot pick-and-place with differential IK."""

from __future__ import annotations

import argparse

parser = argparse.ArgumentParser()
parser.add_argument("--test", default=False, action="store_true", help="Run in test mode")
parser.add_argument("--device", type=str, choices=["cpu", "cuda"], default="cpu", help="Simulation device")
parser.add_argument(
    "--ik-method",
    type=str,
    choices=["singular-value-decomposition", "pseudoinverse", "transpose", "damped-least-squares"],
    default="damped-least-squares",
    help="Differential inverse kinematics method",
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
from isaacsim.robot.experimental.manipulators.examples.franka import FrankaPickPlace


def main() -> None:
    """Run Franka pick-and-place simulation loop."""
    print("Starting Simple Franka Pick-and-Place Demo")
    SimulationManager.set_physics_sim_device(args.device)
    simulation_app.update()

    pick_place = FrankaPickPlace()
    pick_place.setup_scene()

    # Play the simulation.
    omni.timeline.get_timeline_interface().play()
    simulation_app.update()

    reset_needed = True
    task_completed = False

    print("Starting pick-and-place execution")
    frame_count = 0
    while simulation_app.is_running():
        if SimulationManager.is_simulating() and not task_completed:
            if reset_needed:
                pick_place.reset()
                reset_needed = False

            # Execute one step of the pick-and-place operation
            pick_place.forward(args.ik_method)

        if pick_place.is_done() and not task_completed:
            print("done picking and placing")
            task_completed = True

        simulation_app.update()
        frame_count += 1
        if args.test and frame_count >= 10:
            break


if __name__ == "__main__":
    try:
        main()
    except Exception as e:
        print(f"Error: {e}")
    finally:
        simulation_app.close()
