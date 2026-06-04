# SPDX-FileCopyrightText: Copyright (c) 2025-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

"""Provides functionality for building and loading mobility generation scenarios with robots, occupancy maps, and camera configurations."""

import os

import isaacsim.core.api.objects as objects
import isaacsim.core.utils.prims as prim_utils
from isaacsim.core.utils.stage import get_current_stage, open_stage
from isaacsim.core.utils.viewports import set_active_viewport_camera

from .occupancy_map import OccupancyMap
from .reader import MobilityGenReader
from .robot import ROBOTS
from .scenario import SCENARIOS, MobilityGenScenario
from .utils.global_utils import new_world


def load_scenario(path: str) -> MobilityGenScenario:
    """Load a mobility generation scenario from the specified directory.

    Sets up the complete simulation environment including the USD stage, robot, occupancy map,
    and camera configuration based on the scenario configuration file.

    Args:
        path: Directory path containing the scenario files (config, stage, occupancy map).

    Returns:
        The loaded and configured mobility generation scenario.
    """
    reader = MobilityGenReader(path)
    config = reader.read_config()
    robot_type = ROBOTS.get(config.robot_type)
    scenario_type = SCENARIOS.get(config.scenario_type)
    if os.path.exists(os.path.join(path, "stage.usdz")):
        open_stage(os.path.join(path, "stage.usdz"))
    else:
        open_stage(os.path.join(path, "stage.usd"))

    stage = get_current_stage()

    robot_prim_path = "/World/robot"

    if stage.GetPrimAtPath(path).IsValid():
        prim_utils.delete_prim(robot_prim_path)

    objects.GroundPlane("/World/ground_plane", visible=False)
    new_world(physics_dt=robot_type.physics_dt)
    occupancy_map = reader.read_occupancy_map()
    robot = robot_type.build(robot_prim_path)
    chase_camera_path = robot.build_chase_camera()
    set_active_viewport_camera(chase_camera_path)
    robot_type = ROBOTS.get(config.robot_type)
    occupancy_map = OccupancyMap.from_ros_yaml(ros_yaml_path=os.path.join(path, "occupancy_map", "map.yaml"))
    scenario = scenario_type.from_robot_occupancy_map(robot, occupancy_map)
    return scenario
