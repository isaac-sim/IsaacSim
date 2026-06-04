# SPDX-FileCopyrightText: Copyright (c) 2022-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

"""These classes make it easy to monitor obstacles and automatically suppress and unsuppress them.

as needed.

Obstacle monitors use the logical state monitoring framework to handle this functionality in the
background. This module provides an ObstacleMonitorContext base class to make adding obstacle
monitors tooling straightforward.

Example usage (adapted from standalone_examples/api/isaacsim.cortex.framework/behaviors/ur10/bin_stacking_behavior.py):

from isaacsim.cortex.framework.obstacle_monitor_context import ObstacleMonitor, ObstacleMonitorContext

class FlipStationObstacleMonitor(ObstacleMonitor):
    def __init__(self, context):
        # This monitor will enable and disable the "flip_station_sphere" which is an object added to
        # the world.
        super().__init__(context, [context.world.scene.get_object("flip_station_sphere")])

    def is_obstacle_required(self):
        # The specifics aren't important. See bin_stacking_behavior.py for the exact implementation.
        ...
        return is_required

class NavigationObstacleMonitor(ObstacleMonitor):
    def __init__(self, context):
        # This monitor will enable and disable three obstacles related to navigation, each of which
        # are stored in the world object.
        obstacles = [
            context.world.scene.get_object("navigation_dome_obs"),
            context.world.scene.get_object("navigation_barrier_obs"),
            context.world.scene.get_object("navigation_flip_station_obs"),
        ]
        super().__init__(context, obstacles)

    def is_obstacle_required(self):
        # The specifics aren't important. See bin_stacking_behavior.py for the exact implementation.
        ...
        return is_required


class BinStackingContext(ObstacleMonitorContext):
    def __init__(self, robot):
        super().__init__()

        ...

        # Create an add the obstacle monitors.
        self.flip_station_obs_monitor = FlipStationObstacleMonitor(self)
        self.navigation_obs_monitor = NavigationObstacleMonitor(self)
        self.add_obstacle_monitors([self.flip_station_obs_monitor, self.navigation_obs_monitor])

        ...


        # Bin stacking logical state
        self.bins = []
        self.active_bin = None
        self.stacked_bins = []

        # It's important to add logical state monitors using the API rather than directly setting
        # the self.monitors member to a separate list because it already contains a monitor function
        # to handle the obstacle monitors.
        self.add_monitors(
            [
                BinStackingContext.monitor_bins,
                BinStackingContext.monitor_active_bin,
                BinStackingContext.monitor_active_bin_grasp_T,
                BinStackingContext.monitor_active_bin_grasp_reached,
                self.diagnostics_monitor.monitor,
            ]


    def reset(self):
        super().reset()  # Reset the obstacle monitors

        # Reset bin stacking logical state
        self.bins.clear()
        self.active_bin = None
        self.stacked_bins.clear()
"""

from __future__ import annotations

from abc import ABC, abstractmethod
from collections.abc import Sequence
from typing import Optional

from isaacsim.cortex.framework.df import DfLogicalState
from isaacsim.cortex.framework.motion_commander import CortexObstacleType, MotionCommander


class ObstacleMonitor(ABC):
    """An obstacle monitor is a logical state monitor that handles monitoring obstacles and.

    automatically suppressing or unsuppressing them as needed by user defined conditions.

    Obstacles all have an autotoggle feature which can be activated and deactivated using
    activate_autotoggle() and deactivate_autotoggle(). Then autotoggle is activated, the obstacle
    monitor activates and automatically toggles the obstacle, enabling and disabling it as specified
    by the is_obstacles_required() method.

    Deriving classes must implement the is_obstacle_required() method to define when the obstacle is
    enabled (True) or disabled (False) when autotoggle is active.

    Args:
        obstacles: The list of obstacle this monitor will be monitoring.
        motion_commander: Optional motion commander which will be used to enable and disable the
            obstacles. If one isn't supplied on construction, it should be supplied by a call to
            set_motion_commander().
    """

    def __init__(
        self, obstacles: Sequence[CortexObstacleType], motion_commander: Optional[MotionCommander] = None
    ) -> None:
        self.obstacles = obstacles
        self.motion_commander = motion_commander
        self.is_obstacles_enabled = True

    def set_motion_commander(self, motion_commander: MotionCommander) -> None:
        """Set the motion commander which will be used to enable and disable obstacles.

        Args:
            motion_commander: The motion commander to be used.
        """
        self.motion_commander = motion_commander

    @abstractmethod
    def is_obstacle_required(self) -> bool:
        """This is the main API method deriving classes should override.

        It should specify whether this obstacle monitor's obstacles are needed at any given time.
        It will be queried only when autotoggle is active.

        Returns:
            True if the obstacle is needed (and should be enabled), False if it's not needed
            (and should be disabled).
        """
        raise NotImplementedError()

    def reset(self) -> None:
        """Reset this obstacle monitor back to its initial state."""
        self.is_autotoggle_active = False
        self.disable_obstacles()

    def activate_autotoggle(self) -> None:
        """Turn on autotoggle. Starts the obstacle monitor automatically enabling or disabling the.

        obstacle per the boolean return of is_obstacle_required().
        """
        self.is_autotoggle_active = True

    def deactivate_autotoggle(self) -> None:
        """Turn off autotoggle. Stops the obstacle monitor's activity. Disables all monitored.

        obstacles if they're currently enabled.
        """
        self.is_autotoggle_active = False
        self.disable_obstacles_if_needed()

    def enable_obstacles(self) -> None:
        """Enable the collection of all obstacles."""
        for obs in self.obstacles:
            self.motion_commander.enable_obstacle(obs)
        self.is_obstacles_enabled = True

    def enable_obstacles_if_needed(self) -> None:
        """Enable the collection of all obstacles if they aren't already enabled."""
        if not self.is_obstacles_enabled:
            self.enable_obstacles()

    def disable_obstacles(self) -> None:
        """Disable the collection of all obstacles."""
        for obs in self.obstacles:
            self.motion_commander.disable_obstacle(obs)
        self.is_obstacles_enabled = False

    def disable_obstacles_if_needed(self) -> None:
        """Disable the collection of all obstacles if they aren't already disabled."""
        if self.is_obstacles_enabled:
            self.disable_obstacles()

    def step(self) -> None:
        """Step this obstacle monitor.

        If autotoggle is active, enables the obstacles if they're required and disables them if
        they're not.

        If autotoggle is deactivated, disables the obstacles.
        """
        if self.is_autotoggle_active:
            if self.is_obstacle_required():
                self.enable_obstacles_if_needed()
            else:
                self.disable_obstacles_if_needed()
        else:
            self.disable_obstacles_if_needed()


class ObstacleMonitorContext(DfLogicalState):
    """Base class for an context object with obstacle monitors.

    Simplifies adding obstacle monitors as logical state monitors. Any obstacle monitor added is
    both tracked and automatically added as a logical state monitor. Provides a reset() base
    implementation which resets all obstacle monitors.

    Args:
        motion_commander: The motion commander used to enable and disable obstacles.
    """

    def __init__(self, motion_commander: MotionCommander) -> None:
        super().__init__()
        self.motion_commander = motion_commander
        self.obstacle_monitors = []
        self.add_monitor(ObstacleMonitorContext._monitor_obstacles)

    def reset(self) -> None:
        """Reset all obstacle monitors. Deriving classes overriding this method should call.

        super().reset() to ensure the obstacle monitors are reset as well.
        """
        for obs_monitor in self.obstacle_monitors:
            obs_monitor.reset()

    def add_obstacle_monitors(self, obstacle_monitors: Sequence[ObstacleMonitor]) -> None:
        """Add a sequence of obstacle monitors. Their monitor methods will be automatically added.

        as logical state monitors.

        The obstacle monitors' monitor methods will be called in the order provided.

        Args:
            obstacle_monitors: The sequence of obstacle monitors to add.
        """
        for obs_monitor in obstacle_monitors:
            obs_monitor.set_motion_commander(self.motion_commander)
        self.obstacle_monitors.extend(obstacle_monitors)

    def _monitor_obstacles(self) -> None:
        """An internal monitor method which calls each obstacle monitor's monitor function. This.

        method is added as a logical state monitor before any obstacle monitors are added. Obstacle
        monitors can be added any any point and all of their monitor methods will correctly be
        called from this monitor method.
        """
        for obs_monitor in self.obstacle_monitors:
            obs_monitor.step()
