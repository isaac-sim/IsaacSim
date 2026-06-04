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

"""Peck behavior implemented as a decider network with context monitors."""

from __future__ import annotations

from typing import Any

import isaacsim.cortex.framework.math_util as math_util
import numpy as np
from isaacsim.cortex.framework.df import (
    DfAction,
    DfDecider,
    DfDecision,
    DfNetwork,
    DfState,
    DfStateMachineDecider,
    DfStateSequence,
    DfTimedDeciderState,
    DfWriteContextState,
)
from isaacsim.cortex.framework.dfb import DfLift, DfRobotApiContext
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


class PeckContext(DfRobotApiContext):
    """Context for the peck behavior with obstacle-aware target sampling.

    Args:
        robot: The robot API instance.
    """

    def __init__(self, robot: Any) -> None:
        super().__init__(robot)
        self.robot = robot
        self.reset()
        self.add_monitors([PeckContext.monitor_active_target_p])

    def reset(self) -> None:
        """Reset the context state."""
        self.is_done = True
        self.active_target_p = None

    def monitor_active_target_p(self) -> None:
        """Mark the task as done if the active target is near an obstacle."""
        if self.active_target_p is not None and self.is_near_obs(self.active_target_p):
            self.is_done = True

    def set_is_done(self) -> None:
        """Mark the current peck task as done."""
        self.is_done = True

    def is_near_obs(self, p: np.ndarray) -> bool:
        """Check whether a point is within proximity of any registered obstacle.

        Args:
            p: The 3D point to check.

        Returns:
            True if the point is near an obstacle, False otherwise.
        """
        for _, obs in self.robot.registered_obstacles.items():
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

    def choose_next_target(self) -> None:
        """Choose the next peck target away from obstacles."""
        self.active_target_p = self.sample_target_p_away_from_obs()


class PeckState(DfState):
    """State that sends the end-effector to peck at the active target."""

    def enter(self) -> None:
        """Compute the peck target pose and send the end-effector command."""
        target_p = self.context.active_target_p
        target_q = make_target_rotation(target_p)
        self.target = PosePq(target_p, target_q)
        approach_params = ApproachParams(direction=np.array([0.0, 0.0, -0.1]), std_dev=0.04)
        self.context.robot.arm.send_end_effector(self.target, approach_params=approach_params)

    def step(self) -> Any:
        """Continue until the end-effector reaches the target.

        Returns:
            Self to keep moving, or None when the target is reached.
        """
        # Send the command each cycle so exponential smoothing will converge.
        target_dist = np.linalg.norm(self.context.robot.arm.get_fk_p() - self.target.p)
        if target_dist < 0.01:
            return None  # Exit
        return self  # Keep going


class ChooseTarget(DfAction):
    """Action that chooses the next peck target."""

    def step(self) -> None:
        """Mark the task as not done and choose the next target."""
        self.context.is_done = False
        self.context.choose_next_target()


class CloseGripper(DfAction):
    """Action that closes the gripper."""

    def enter(self) -> None:
        """Close the gripper."""
        self.context.robot.gripper.close()


class Dispatch(DfDecider):
    """The top-level decider.

    If the current peck task is done, then it will choose a target.  Otherwise, it executes the peck
    behavior. The peck behavior is a sequential state machine which 1. closes the gripper, 2. pecks,
    3. lifts the end-effector slightly, 4. writes to the context that it's done.

    This behavior by itself is equivalent to the state machine variant in peck_state_machine.py.
    However, the context is also continually monitoring the situation and if it sees that its
    current target is blocked, it'll set the context.is_done flag to True triggering this Dispatch
    decider to choose a new target.
    """

    def __init__(self) -> None:
        super().__init__()

        self.add_child("choose_target", ChooseTarget())
        self.add_child(
            "peck",
            DfStateMachineDecider(
                DfStateSequence(
                    [
                        CloseGripper(),
                        PeckState(),
                        DfTimedDeciderState(DfLift(height=0.05), activity_duration=0.25),
                        DfWriteContextState(lambda context: context.set_is_done()),
                    ]
                )
            ),
        )

    def decide(self) -> Any:
        """Decide to choose a target if done, otherwise continue pecking.

        Returns:
            A DfDecision directing to choose_target or peck.
        """
        if self.context.is_done:
            return DfDecision("choose_target")
        else:
            return DfDecision("peck")


def make_decider_network(robot: Any) -> Any:
    """Create the peck decider network for the given robot.

    Args:
        robot: The robot API instance.

    Returns:
        A configured DfNetwork for the peck decider behavior.
    """
    return DfNetwork(Dispatch(), context=PeckContext(robot))
