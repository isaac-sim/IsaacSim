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

"""Implements base functionality for mobility generation scenarios with robot navigation in occupancy map environments."""

from typing import NoReturn

from PIL import Image

from .common import Module
from .occupancy_map import OccupancyMap
from .robot import MobilityGenRobot
from .utils.registry import Registry


class MobilityGenScenario(Module):
    """Base class for mobility generation scenarios that simulate robot movement in occupancy map environments.

    This class provides the foundation for creating scenarios where robots navigate through environments
    represented by occupancy maps. It manages the robot instance, occupancy map data, and creates a buffered
    version of the occupancy map based on the robot's occupancy map radius for collision detection and
    path planning.

    Subclasses must implement the abstract methods `reset()` and `step()` to define specific scenario
    behavior and movement logic. The class also provides visualization capabilities through the
    `get_visualization_image()` method.

    Args:
        robot: The robot instance that will navigate in the scenario.
        occupancy_map: The occupancy map representing the environment layout and obstacles.
    """

    def __init__(self, robot: MobilityGenRobot, occupancy_map: OccupancyMap) -> None:
        self.robot = robot
        self.occupancy_map = occupancy_map
        self.buffered_occupancy_map = occupancy_map.buffered_meters(self.robot.occupancy_map_radius)

    @classmethod
    def from_robot_occupancy_map(cls, robot: MobilityGenRobot, occupancy_map: OccupancyMap) -> "MobilityGenScenario":
        """Create a MobilityGenScenario instance from a robot and occupancy map.

        Args:
            robot: The mobility generation robot.
            occupancy_map: The occupancy map for the scenario.

        Returns:
            A new MobilityGenScenario instance.
        """
        return cls(robot, occupancy_map)

    def reset(self) -> NoReturn:
        """Reset the scenario to its initial state."""
        raise NotImplementedError

    def step(self, step_size: float) -> bool:
        """Advances the scenario by one step.

        Args:
            step_size: The size of the step to take.

        Returns:
            True if the step was successful.
        """
        raise NotImplementedError

    def get_visualization_image(self) -> Image:
        """Get the visualization image of the occupancy map.

        Returns:
            The ROS image representation of the occupancy map.
        """
        image = self.occupancy_map.ros_image()
        return image


SCENARIOS = Registry[MobilityGenScenario]()
