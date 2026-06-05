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

"""A commander implements an abstraction of a policy controlling a subset of joints. Each commander.

defines its own command API accessed by the decision layer. This abstract base class defines the
methods used by the cortex framework behind the scenes for processing, resetting, etc. the
commanders.
"""

from __future__ import annotations

from abc import ABC, abstractmethod
from collections.abc import Sequence
from typing import Any

from isaacsim.core.api.articulations import ArticulationSubset
from isaacsim.core.utils.types import ArticulationAction


class Commander(ABC):
    """Abstract base class of a commander.

    A commander governs the control of a particular subset of joints. Users implement behavior by
    sending commands to the commander using a custom command API defined by the deriving class.  The
    abstract base class API includes only methods needed for this commander to be registered with a
    ControlledArticulation object, including methods for processing commands, resetting the
    commander, and accessing the latest action.

    Often, a deriving class would implement a set_command(self, command) method where command is a
    custom command type providing all the information needed for commanding the behavior. But we
    place no framework restrictions on the nature of the command API used by any given deriving
    class.

    This API is meant to model standard command APIs of robotic system. Often commands are sent
    through some pub-sub messaging system such as ROS or ZeroMQ then processed within a real-time
    control loop. These real-time loops often process any queued message once per cycle. In
    simulation, we have synchronicity where commands might be set by the decision layer and then
    processed in the same step of the loop runner, so we can simplify implementations by assuming
    there will only be one command set per cycle (no queuing necessary). But we still sparate out
    the command API calls (such as set_command(command)) from the processing of the commands to
    follow the broader processing model.

    In particular, this command API supports both discrete commands and continuous streams of
    commands.

    All commanders command a subset of the robot's joints which is specified on
    construction.

    Args:
        articulation_subset: The subset of joints being controlled by this commander.
    """

    def __init__(self, articulation_subset: ArticulationSubset) -> None:
        self.articulation_subset = articulation_subset
        self.latest_command = None

    @property
    def num_controlled_joints(self) -> int:
        """Returns the number of controlled joints as defined by the articulation subset."""
        return self.articulation_subset.num_joints

    @property
    def controlled_joints(self) -> Sequence[str]:
        """Returns the names of the controlled joints."""
        return self.articulation_subset.joint_names

    @property
    def latest_action(self) -> ArticulationAction:
        """Returns the latest applied action."""
        return self.articulation_subset.get_applied_action()

    @property
    def command(self) -> Any:
        """Returns the latest received command.

        The type of this command is defined by the deriving class.
        """
        return self.latest_command

    def send(self, command: Any) -> None:
        """Send a command to this commander. The command is cached off in the member.

        latest_command.

        The type of the command is defined by the deriving class.

        Args:
            command: The command to send.
        """
        self.latest_command = command

    def clear(self) -> None:
        """Clear the latest command. Sets latest_command to None."""
        self.latest_command = None

    @abstractmethod
    def step(self, dt: float) -> None:
        """Steps the commander to process the latest command.

        Override this method to define how the underlying policy is processed.

        Args:
            dt: The time step for this step.
        """
        raise NotImplementedError()

    def reset(self) -> None:
        """Reset the commander. By default it does nothing.

        This method doesn't handle resetting the command.
        """

    def post_reset(self) -> None:
        """Clear the command and reset the commander. This method is called automatically at the.

        right time by the CortexWorld after the simulation is reset (hence the post_ prefix).
        """
        self.clear()
        self.reset()
