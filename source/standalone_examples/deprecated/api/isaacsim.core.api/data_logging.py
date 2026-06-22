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

"""Demonstrate logging and saving robot simulation data."""

import sys
from typing import Any

from isaacsim import SimulationApp

simulation_app = SimulationApp({"headless": False})

import carb
from isaacsim.core.api import World
from isaacsim.core.api.robots import Robot
from isaacsim.core.utils.stage import add_reference_to_stage
from isaacsim.storage.native import get_assets_root_path

assets_root_path = get_assets_root_path()
if assets_root_path is None:
    carb.log_error("Could not find Isaac Sim assets folder")
    simulation_app.close()
    sys.exit()

my_world = World(stage_units_in_meters=1.0)
my_world.scene.add_default_ground_plane()

asset_path = assets_root_path + "/Isaac/Robots/FrankaRobotics/FrankaPanda/franka.usd"
robot = add_reference_to_stage(usd_path=asset_path, prim_path="/World/Franka_1")
robot.GetVariantSet("Gripper").SetVariantSelection("AlternateFinger")
robot.GetVariantSet("Mesh").SetVariantSelection("Quality")
articulated_system_1 = my_world.scene.add(Robot(prim_path="/World/Franka_1", name="my_franka_1"))


my_world.reset()
data_logger = my_world.get_data_logger()


def frame_logging_func(tasks: Any, scene: Any) -> dict[str, list[float]]:
    """Return joint position data for the current simulation frame."""
    return {
        "joint_positions": scene.get_object("my_franka_1").get_joint_positions().tolist(),
        "applied_joint_positions": scene.get_object("my_franka_1").get_applied_action().joint_positions.tolist(),
    }


data_logger.add_data_frame_logging_func(frame_logging_func)
data_logger.start()
for j in range(100):
    my_world.step(render=True)

data_logger.save(log_path="./isaac_sim_data.json")
data_logger.reset()

data_logger.load(log_path="./isaac_sim_data.json")
print(data_logger.get_data_frame(data_frame_index=2))
simulation_app.close()
