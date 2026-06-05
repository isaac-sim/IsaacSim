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

"""Simple example of constructing a running a state machine. This state machine will loop choosing.

a target on the ground away from obstacles and pecking at it.

In general this will loop successfully forever as long as the world is static. However, if the user
moves a block (obstacle) to overlap with a chosen target, the end-effector will avoid the block and
be unable to reach its target, thereby stalling.

This sort of reactivity is more natural to program using decider networks as demonstrated in
peck_decider_network.py, where the system constantly monitors the target and triggers the system to
choose a new one if the target becomes blocked.
"""

from __future__ import annotations

from typing import Any

import isaacsim.cortex.framework.math_util as math_util
import numpy as np
from isaacsim.cortex.framework.df import DfNetwork, DfState, DfStateMachineDecider, DfStateSequence, DfTimedDeciderState
from isaacsim.cortex.framework.dfb import DfBasicContext, DfCloseGripper, DfLift
from isaacsim.cortex.framework.motion_commander import ApproachParams, PosePq


def sample_target_p() -> np.ndarray:
    """Sample a random target position on the ground plane.

    Returns:
        A 3D numpy array representing a random target position.
    """
    min_x = 0.3
    max_x = 0.7
    min_y = -0.4
    max_y = 0.4

    pt = np.zeros(3)
    pt[0] = (max_x - min_x) * np.random.random_sample() + min_x
    pt[1] = (max_y - min_y) * np.random.random_sample() + min_y
    pt[2] = 0.01

    return pt


def make_target_rotation(target_p: np.ndarray) -> np.ndarray:
    """Compute a downward-facing rotation quaternion oriented toward the target.

    Args:
        target_p: The 3D target position.

    Returns:
        A quaternion representing the target rotation.
    """
    return math_util.matrix_to_quat(
        math_util.make_rotation_matrix(az_dominant=np.array([0.0, 0.0, -1.0]), ax_suggestion=-target_p)
    )


class PeckState(DfState):
    """State that samples a target, sends the end-effector to peck, and waits for arrival."""

    def is_near_obs(self, p: np.ndarray) -> bool:
        """Check whether a point is within proximity of any registered obstacle.

        Args:
            p: The 3D point to check.

        Returns:
            True if the point is near an obstacle, False otherwise.
        """
        for _, obs in self.context.robot.registered_obstacles.items():
            obs_p, _ = obs.get_world_pose()
            if np.linalg.norm(obs_p - p) < 0.2:
                return True
        return False

    def sample_target_p_away_from_obs(self) -> np.ndarray:
        """Sample a random target position that is not near any obstacle.

        Returns:
            A 3D numpy array representing a target position away from all obstacles.
        """
        target_p = sample_target_p()
        while self.is_near_obs(target_p):
            target_p = sample_target_p()
        return target_p

    def enter(self) -> None:
        """Sample a target and send the end-effector toward it."""
        # On entry, sample a target.
        target_p = self.sample_target_p_away_from_obs()
        target_q = make_target_rotation(target_p)
        self.target = PosePq(target_p, target_q)
        approach_params = ApproachParams(direction=np.array([0.0, 0.0, -0.1]), std_dev=0.04)
        self.context.robot.arm.send_end_effector(self.target, approach_params=approach_params)

    def step(self) -> Any:
        """Continue until the end-effector reaches the target.

        Returns:
            Self to keep moving, or None when the target is reached.
        """
        target_dist = np.linalg.norm(self.context.robot.arm.get_fk_p() - self.target.p)
        if target_dist < 0.01:
            return None  # Exit
        return self  # Keep going


def make_decider_network(robot: Any) -> Any:
    """Create the peck state machine decider network for the given robot.

    Args:
        robot: The robot API instance.

    Returns:
        A configured DfNetwork for the peck state machine behavior.
    """
    # Build a state machine decider from a sequencial state machine. The sequence will be
    #
    #   1. close gripper,
    #   2. peck at target,
    #   3. lift the end-effector.
    #
    # It's set to loop, so it will simply peck repeatedly until the behavior is replaced. Note that
    # PeckState chooses its target on entry.
    root = DfStateMachineDecider(
        DfStateSequence(
            [DfCloseGripper(), PeckState(), DfTimedDeciderState(DfLift(height=0.05), activity_duration=0.25)], loop=True
        )
    )
    return DfNetwork(root, context=DfBasicContext(robot))
