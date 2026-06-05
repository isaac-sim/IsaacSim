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

"""Simple decider network that prints left, right, or middle based on end-effector position."""

from __future__ import annotations

from typing import Any

from isaacsim.cortex.framework.df import DfAction, DfDecider, DfDecision, DfNetwork
from isaacsim.cortex.framework.dfb import DfRobotApiContext


class Context(DfRobotApiContext):
    """Context that monitors end-effector y-position and lateral state.

    Args:
        robot: The robot API instance.
    """

    def __init__(self, robot: Any) -> None:
        super().__init__(robot)

        self.reset()
        self.add_monitors([Context.monitor_y, Context.monitor_is_left, Context.monitor_is_middle])

    def reset(self) -> None:
        """Reset the monitored state variables."""
        self.y = None
        self.is_left = None
        self.is_middle = None

    def monitor_y(self) -> None:
        """Update the y-position of the end-effector."""
        self.y = self.robot.arm.get_fk_p()[1]

    def monitor_is_left(self) -> None:
        """Update whether the end-effector is on the left side."""
        self.is_left = self.y < 0

    def monitor_is_middle(self) -> None:
        """Update whether the end-effector is in the middle region."""
        self.is_middle = self.y > -0.15 and self.y < 0.15


class PrintAction(DfAction):
    """Action that prints a message on entry.

    Args:
        msg: The message to print.
    """

    def __init__(self, msg: str | None = None) -> None:
        super().__init__()
        self.msg = msg

    def __str__(self) -> str:
        """Return the message as a string representation.

        Returns:
            The message string.
        """
        return self.msg

    def enter(self) -> None:
        """Print the message or parameters on entry."""
        if self.params is not None:
            self.msg = self.params
            print(self.params)
        else:
            print(self.msg)


class Dispatch(DfDecider):
    """Top-level decider that dispatches print actions based on end-effector position."""

    def __init__(self) -> None:
        super().__init__()
        self.add_child("print_left", PrintAction("<left>"))
        self.add_child("print_right", PrintAction("<right>"))
        self.add_child("print", PrintAction(""))

    def decide(self) -> Any:
        """Decide which print action to run based on lateral position.

        Returns:
            A DfDecision directing to print_left, print_right, or print.
        """
        if self.context.is_middle:
            return DfDecision("print", "<middle>")  # Send parameters down to generic print.

        if self.context.is_left:
            return DfDecision("print_left")
        else:
            return DfDecision("print_right")


def make_decider_network(robot: Any) -> Any:
    """Create the simple decider network for the given robot.

    Args:
        robot: The robot API instance.

    Returns:
        A configured DfNetwork for the simple decider behavior.
    """
    return DfNetwork(Dispatch(), context=Context(robot))
