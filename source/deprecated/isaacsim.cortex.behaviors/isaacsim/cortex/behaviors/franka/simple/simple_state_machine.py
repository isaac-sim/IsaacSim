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

"""Simple state machine that moves the end-effector between two target positions."""

from __future__ import annotations

from typing import Any

import numpy as np
from isaacsim.cortex.framework.df import DfNetwork, DfState, DfStateMachineDecider, DfStateSequence
from isaacsim.cortex.framework.dfb import DfBasicContext


class ReachState(DfState):
    """State that moves the end-effector to a target position and exits on arrival.

    Args:
        target_p: The 3D target position.
    """

    def __init__(self, target_p: np.ndarray) -> None:
        self.target_p = target_p

    def __str__(self) -> str:
        """Return a string representation including the target position.

        Returns:
            A string representation of this state with the target position.
        """
        return f"{super().__str__()}({self.target_p})"

    def enter(self) -> None:
        """Send the end-effector to the target position."""
        self.context.robot.arm.send_end_effector(target_position=self.target_p)

    def step(self) -> Any:
        """Continue until the end-effector reaches the target.

        Returns:
            Self to keep moving, or None when the target is reached.
        """
        if np.linalg.norm(self.target_p - self.context.robot.arm.get_fk_p()) < 0.01:
            return None
        return self


def make_decider_network(robot: Any) -> Any:
    """Create the simple state machine decider network for the given robot.

    Args:
        robot: The robot API instance.

    Returns:
        A configured DfNetwork for the simple state machine behavior.
    """
    p1 = np.array([0.2, -0.2, 0.01])
    p2 = np.array([0.6, 0.3, 0.6])
    root = DfStateMachineDecider(DfStateSequence([ReachState(p1), ReachState(p2)], loop=True))
    return DfNetwork(root, context=DfBasicContext(robot))
