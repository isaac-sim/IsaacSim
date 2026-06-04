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

"""Common test utilities and helpers."""

from __future__ import annotations

from dataclasses import dataclass, field

import carb
import isaacsim.core.experimental.utils.stage as stage_utils
import numpy as np
import omni.kit.app
import omni.timeline
from isaacsim.core.simulation_manager import SimulationManager
from isaacsim.storage.native import get_assets_root_path_async

EARTH_GRAVITY = 9.81
MOON_GRAVITY = 1.62
CM_GRAVITY = 981.0

GRAVITY_TOLERANCE = 0.1
ANGLE_TOLERANCE_DEG = 0.1
ANGULAR_VEL_TOLERANCE = 0.2
ORIENTATION_TOLERANCE = 1e-4
SMALL_TOLERANCE = 0.01


async def step_simulation(seconds: float) -> None:
    """Step the simulation forward by the given number of seconds."""
    dt = SimulationManager.get_physics_dt()
    steps = max(1, int(round(seconds / dt)))
    for _ in range(steps):
        await omni.kit.app.get_app().next_update_async()


async def reset_timeline(timeline=None, *, steps: int = 2) -> None:
    """Stop and restart the timeline."""
    if timeline is None:
        timeline = omni.timeline.get_timeline_interface()
    timeline.stop()
    await omni.kit.app.get_app().next_update_async()
    timeline.play()
    for _ in range(steps):
        await omni.kit.app.get_app().next_update_async()


@dataclass
class AntConfig:
    """Configuration data for ant robot used in sensor tests."""

    leg_paths: list[str] = field(default_factory=lambda: ["/Ant/Arm_{:02d}/Lower_Arm".format(i + 1) for i in range(4)])
    sphere_path: str = "/Ant/Sphere"
    sensor_offsets: list[np.ndarray] = field(default_factory=lambda: [np.array([[40.0, 0.0, 0.0]]) for _ in range(4)])
    # IMU sensor offsets (at origin for each sensor location)
    imu_sensor_offsets: list[np.ndarray] = field(
        default_factory=lambda: [np.array([[0.0, 0.0, 0.0]]) for _ in range(5)]
    )
    # IMU sensor orientations (identity quaternions, wxyz)
    sensor_quatd: list[np.ndarray] = field(default_factory=lambda: [np.array([[1.0, 0.0, 0.0, 0.0]]) for _ in range(5)])
    colors: list[tuple[float, float, float, float]] = field(
        default_factory=lambda: [(1, 0, 0, 1), (0, 1, 0, 1), (0, 0, 1, 1), (1, 1, 0, 1)]
    )
    shoulder_joints: list[str] = field(
        default_factory=lambda: ["/Ant/Arm_{:02d}/Upper_Arm/shoulder_joint".format(i + 1) for i in range(4)]
    )
    lower_joints: list[str] = field(default_factory=list)

    def __post_init__(self):
        """Initialize computed fields after dataclass creation."""
        if not self.lower_joints:
            self.lower_joints = ["{}/lower_arm_joint".format(path) for path in self.leg_paths]


async def setup_ant_scene(physics_rate: float = 60.0) -> AntConfig:
    """Load the ant USD scene and return configuration data.

    Args:
        physics_rate: Physics simulation rate in Hz.

    Returns:
        AntConfig with paths and sensor configuration for the ant robot.
    """
    assets_root_path = await get_assets_root_path_async()
    if assets_root_path is None:
        carb.log_error("Could not find Isaac Sim assets folder")
        raise RuntimeError("Could not find Isaac Sim assets folder")

    await stage_utils.open_stage_async(assets_root_path + "/Isaac/Robots/IsaacSim/Ant/ant_colored.usd")
    await omni.kit.app.get_app().next_update_async()

    stage_utils.set_stage_units(meters_per_unit=1.0)
    SimulationManager.setup_simulation(dt=1.0 / physics_rate)

    return AntConfig()
