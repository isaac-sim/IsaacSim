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

"""Build utilities for loading MobilityGen scenarios from recorded data."""

from __future__ import annotations

import os
import tempfile

from pxr import UsdGeom

from .occupancy_map import OccupancyMap
from .reader import MobilityGenReader
from .robot import ROBOTS
from .scenario import SCENARIOS, MobilityGenScenario

_SDG_PATHS = (
    "/Render/PostProcess/SDGPipeline",
    "/Render/PostRender/SDGPipeline",
    "/Render/Simulation/SDGPipeline",
)


def load_scenario(path: str) -> MobilityGenScenario:
    """Load a MobilityGen scenario from a recorded data directory.

    Args:
        path: The path to the recorded data directory.

    Returns:
        The loaded MobilityGen scenario.
    """
    from isaacsim.core.experimental.objects import GroundPlane
    from isaacsim.core.experimental.utils.stage import delete_prim, get_current_stage, open_stage
    from isaacsim.core.rendering_manager import ViewportManager
    from isaacsim.core.simulation_manager import SimulationManager
    from pxr import Usd as _Usd

    reader = MobilityGenReader(path)
    config = reader.read_config()
    robot_type = ROBOTS.get(config.robot_type)
    scenario_type = SCENARIOS.get(config.scenario_type)
    if os.path.exists(os.path.join(path, "stage.usdz")):
        open_stage(os.path.join(path, "stage.usdz"))
    else:
        # Strip any SDGPipeline prims that UI sessions may have baked into the
        # recording stage.  Opening such a stage in Kit would recreate the stale
        # OmniGraph nodes and crash during headless replay.  We strip them from a
        # temp copy so the original recording file is never modified.
        _src = os.path.join(path, "stage.usd")
        _disk_stage = _Usd.Stage.Open(_src)
        _needs_strip = any(_disk_stage.GetPrimAtPath(p).IsValid() for p in _SDG_PATHS)
        if _needs_strip:
            for p in _SDG_PATHS:
                if _disk_stage.GetPrimAtPath(p).IsValid():
                    _disk_stage.RemovePrim(p)
            _tmp = os.path.join(tempfile.mkdtemp(), "stage.usd")
            _disk_stage.Export(_tmp)
            del _disk_stage
            open_stage(_tmp)
        else:
            del _disk_stage
            open_stage(_src)

    stage = get_current_stage()
    robot_prim_path = "/World/robot"

    if stage.GetPrimAtPath(robot_prim_path).IsValid():
        delete_prim(robot_prim_path)

    SimulationManager.setup_simulation(dt=robot_type.physics_dt)
    ground_plane = GroundPlane("/World/ground_plane", templates=None)
    # Hide the render mesh to prevent z-fighting with the warehouse USD floor.
    # Develop uses GroundPlane(visible=False); experimental API has no such parameter.
    for mesh_path in ground_plane.meshes.paths:
        UsdGeom.Imageable(stage.GetPrimAtPath(mesh_path)).MakeInvisible()
    robot = robot_type.build(robot_prim_path)
    chase_camera_path = robot.build_chase_camera()
    if ViewportManager.get_viewport_api() is not None:
        ViewportManager.set_camera(chase_camera_path)
    occupancy_map = OccupancyMap.from_ros_yaml(ros_yaml_path=os.path.join(path, "occupancy_map", "map.yaml"))
    scenario = scenario_type.from_robot_occupancy_map(robot, occupancy_map)
    return scenario
