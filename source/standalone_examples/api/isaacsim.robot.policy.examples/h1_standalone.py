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

"""Demonstrate H1 humanoid robot simulation with policy control."""

from isaacsim import SimulationApp

simulation_app = SimulationApp({"headless": False})

import argparse

import carb
import omni.timeline
from isaacsim.core.deprecation_manager import import_module
from isaacsim.core.experimental.utils.stage import define_prim
from isaacsim.core.rendering_manager import RenderingManager
from isaacsim.core.simulation_manager import SimulationManager
from isaacsim.core.simulation_manager.impl.isaac_events import IsaacEvents
from isaacsim.robot.policy.examples.robots import H1FlatTerrainPolicy
from isaacsim.storage.native import get_assets_root_path

torch = import_module("torch")

parser = argparse.ArgumentParser(description="Define the number of robots.")
parser.add_argument("--num-robots", type=int, default=1, help="Number of robots (default: 1)")
parser.add_argument(
    "--env-url",
    default="/Isaac/Environments/Grid/default_environment.usd",
    required=False,
    help="Path to the environment url",
)
parser.add_argument("--device", type=str, choices=["cpu", "cuda"], default="cpu", help="Simulation device")
parser.add_argument("--test", default=False, action="store_true", help="Run in test mode")

args, unknown = parser.parse_known_args()
print(f"Number of robots: {args.num_robots}")
print(f"Using device: {args.device}")

first_step = True
reset_needed = False
robots = []


# initialize robot on first step, run robot advance
def on_physics_step(step_size: float, context: object) -> None:
    """Handle physics step for robot initialization, reset, and control."""
    global first_step
    global reset_needed
    if first_step:
        for robot in robots:
            robot.initialize()
        first_step = False
    elif reset_needed:
        reset_needed = False
        first_step = True
    else:
        for robot in robots:
            robot.forward(step_size, base_command)


assets_root_path = get_assets_root_path()
if assets_root_path is None:
    carb.log_error("Could not find Isaac Sim assets folder")

# spawn scene
prim = define_prim("/World/Ground", "Xform")
asset_path = assets_root_path + args.env_url
prim.GetReferences().AddReference(asset_path)

# spawn physics scene
# TODO: physics scene should be created by simulation manager
define_prim("/World/PhysicsScene", "PhysicsScene")

# set rendering manager
RenderingManager.set_dt(8.0 / 200.0)

# spawn simulation manager
SimulationManager.set_physics_sim_device(args.device)
SimulationManager.set_physics_dt(1.0 / 200.0)

# robot command
base_command = torch.zeros(3, device=args.device)

# spawn robot
for i in range(0, args.num_robots):
    h1 = H1FlatTerrainPolicy(
        prim_path="/World/H1_" + str(i),
        usd_path=assets_root_path + "/Isaac/Robots/Unitree/H1/h1.usd",
        position=[0, i, 1.05],
    )

    robots.append(h1)

_physics_callback_id = SimulationManager.register_callback(on_physics_step, IsaacEvents.POST_PHYSICS_STEP)

omni.timeline.get_timeline_interface().play()
simulation_app.update()

i = 0
while simulation_app.is_running():
    simulation_app.update()

    if SimulationManager.is_simulating():
        if args.test and i > 10:
            break
        if i >= 0 and i < 80:
            # forward
            base_command = torch.tensor([0.5, 0, 0], device=args.device)
        elif i >= 80 and i < 130:
            # rotate
            base_command = torch.tensor([0.5, 0, 0.5], device=args.device)
        elif i >= 130 and i < 200:
            # side ways
            base_command = torch.tensor([0, 0, 0.5], device=args.device)
        elif i == 200:
            i = 0
        i += 1
    else:
        reset_needed = True
simulation_app.close()
