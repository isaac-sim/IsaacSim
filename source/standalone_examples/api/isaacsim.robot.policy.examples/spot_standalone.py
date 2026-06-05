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

"""Demonstrate Spot robot simulation with policy control."""

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
from isaacsim.robot.policy.examples.robots import SpotFlatTerrainPolicy
from isaacsim.storage.native import get_assets_root_path

torch = import_module("torch")


first_step = True
reset_needed = False

parser = argparse.ArgumentParser(description="Select simulation device.")
parser.add_argument("--test", default=False, action="store_true", help="Run in test mode")
parser.add_argument("--device", type=str, choices=["cpu", "cuda"], default="cpu", help="Simulation device")

args, unknown = parser.parse_known_args()
print(f"Using device: {args.device}")


# initialize robot on first step, run robot advance
def on_physics_step(step_size, context) -> None:
    """Handle physics step for Spot initialization, reset, and control."""
    global first_step
    global reset_needed
    if first_step:
        spot.initialize()
        first_step = False
    elif reset_needed:
        reset_needed = False
        first_step = True
    else:
        spot.forward(step_size, base_command)


# spawn world
assets_root_path = get_assets_root_path()
if assets_root_path is None:
    carb.log_error("Could not find Isaac Sim assets folder")

# spawn warehouse scene
prim = define_prim("/World/Ground", "Xform")
asset_path = assets_root_path + "/Isaac/Environments/Grid/default_environment.usd"
prim.GetReferences().AddReference(asset_path)

# spawn physics scene
# TODO: physics scene should be created by simulation manager
define_prim("/World/PhysicsScene", "PhysicsScene")

# set rendering manager
RenderingManager.set_dt(8.0 / 200.0)

# spawn simulation manager
SimulationManager.set_physics_sim_device(args.device)
SimulationManager.set_physics_dt(1.0 / 200.0)

# spawn robot
spot = SpotFlatTerrainPolicy(
    prim_path="/World/Spot",
    position=[0, 0, 0.8],
)
# robot command
base_command = torch.zeros(3, device=args.device)

# register physics callback
_physics_callback_id = SimulationManager.register_callback(on_physics_step, IsaacEvents.POST_PHYSICS_STEP)

# play simulation
omni.timeline.get_timeline_interface().play()
simulation_app.update()

i = 0
while simulation_app.is_running():
    simulation_app.update()
    if SimulationManager.is_simulating():
        if i >= 0 and i < 80:
            # forward
            base_command = torch.tensor([2, 0, 0], device=args.device)
        elif i >= 80 and i < 130:
            # rotate
            base_command = torch.tensor([1, 0, 2], device=args.device)
        elif i >= 130 and i < 200:
            # side ways
            base_command = torch.tensor([0, 1, 0], device=args.device)
        elif i == 200:
            i = 0
            if args.test is True:
                print("Reached: ", spot.robot.get_world_poses()[0])
                break
        i += 1
    else:
        reset_needed = True
simulation_app.close()
