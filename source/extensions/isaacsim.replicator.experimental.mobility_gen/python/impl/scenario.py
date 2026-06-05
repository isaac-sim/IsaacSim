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

"""MobilityGen scenario base class and registry."""

from abc import ABC, abstractmethod

from PIL import Image

from .common import Module
from .occupancy_map import OccupancyMap
from .robot import MobilityGenRobot
from .utils.registry import Registry


class MobilityGenScenario(Module, ABC):
    """Abstract base class for MobilityGen scenarios.

    Args:
        robot: The robot for this scenario.
        occupancy_map: The occupancy map for this scenario.
    """

    def __init__(self, robot: MobilityGenRobot, occupancy_map: OccupancyMap) -> None:
        self.robot = robot
        self.occupancy_map = occupancy_map
        self.buffered_occupancy_map = occupancy_map.buffered_meters(self.robot.occupancy_map_radius)

    @classmethod
    def from_robot_occupancy_map(cls, robot: MobilityGenRobot, occupancy_map: OccupancyMap) -> "MobilityGenScenario":
        """Create a scenario from a robot and occupancy map.

        Args:
            robot: The robot for this scenario.
            occupancy_map: The occupancy map for this scenario.

        Returns:
            The created scenario.
        """
        return cls(robot, occupancy_map)

    @abstractmethod
    def reset(self) -> None:
        """Reset the scenario to its initial state."""
        ...

    @abstractmethod
    def step(self, step_size: float) -> bool:
        """Step the scenario forward by one timestep.

        Args:
            step_size: The physics timestep size in seconds.

        Returns:
            True if the episode is still active, False if it has ended.
        """
        ...

    def get_visualization_image(self) -> Image.Image:
        """Get a visualization image of the current scenario state.

        Returns:
            A PIL image representing the current state.
        """
        image = self.occupancy_map.ros_image()
        return image


SCENARIOS = Registry[MobilityGenScenario]()
