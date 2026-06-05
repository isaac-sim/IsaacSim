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

"""Demonstrate adding and controlling multiple Franka robots."""

from isaacsim import SimulationApp

simulation_app = SimulationApp({"headless": False})

import argparse
import sys

import carb
import numpy as np
from isaacsim.core.api import World
from isaacsim.core.api.robots import Robot
from isaacsim.core.utils.stage import add_reference_to_stage, get_stage_units
from isaacsim.core.utils.types import ArticulationAction
from isaacsim.storage.native import get_assets_root_path

parser = argparse.ArgumentParser()
parser.add_argument("--test", default=False, action="store_true", help="Run in test mode")
args, unknown = parser.parse_known_args()

assets_root_path = get_assets_root_path()
if assets_root_path is None:
    carb.log_error("Could not find Isaac Sim assets folder")
    simulation_app.close()
    sys.exit()

my_world = World(stage_units_in_meters=1.0)
my_world.scene.add_default_ground_plane()

asset_path = assets_root_path + "/Isaac/Robots/FrankaRobotics/FrankaPanda/franka.usd"
robot1 = add_reference_to_stage(usd_path=asset_path, prim_path="/World/Franka_1")
robot1.GetVariantSet("Gripper").SetVariantSelection("AlternateFinger")
robot1.GetVariantSet("Mesh").SetVariantSelection("Quality")
robot2 = add_reference_to_stage(usd_path=asset_path, prim_path="/World/Franka_2")
robot2.GetVariantSet("Gripper").SetVariantSelection("AlternateFinger")
robot2.GetVariantSet("Mesh").SetVariantSelection("Quality")
articulated_system_1 = my_world.scene.add(Robot(prim_path="/World/Franka_1", name="my_franka_1"))
articulated_system_2 = my_world.scene.add(Robot(prim_path="/World/Franka_2", name="my_franka_2"))

for i in range(5):
    print("resetting...")
    my_world.reset()
    articulated_system_1.set_world_pose(position=np.array([0.0, 2.0, 0.0]) / get_stage_units())
    articulated_system_2.set_world_pose(position=np.array([0.0, -2.0, 0.0]) / get_stage_units())
    articulated_system_1.set_joint_positions(np.array([1.5, 1.5, 1.5, 1.5, 1.5, 1.5, 1.5, 1.5, 1.5]))
    for j in range(500):
        my_world.step(render=True)
        if j == 100:
            articulated_system_2.get_articulation_controller().apply_action(
                ArticulationAction(joint_positions=np.array([1.5, 1.5, 1.5, 1.5, 1.5, 1.5, 1.5, 1.5, 1.5]))
            )
        if j == 400:
            print("Franka 1's joint positions are: ", articulated_system_1.get_joint_positions())
            print("Franka 2's joint positions are: ", articulated_system_2.get_joint_positions())
    if args.test is True:
        break
simulation_app.close()
