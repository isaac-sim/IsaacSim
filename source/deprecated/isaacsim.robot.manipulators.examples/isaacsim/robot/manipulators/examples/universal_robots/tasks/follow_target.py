# SPDX-FileCopyrightText: Copyright (c) 2021-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

"""Follow-target task definition for Universal Robots arms."""

from __future__ import annotations

import isaacsim.core.api.tasks as tasks
import numpy as np
from isaacsim.core.utils.prims import is_prim_path_valid
from isaacsim.core.utils.rotations import euler_angles_to_quat
from isaacsim.core.utils.string import find_unique_string_name
from isaacsim.robot.manipulators.examples.universal_robots import UR10


class FollowTarget(tasks.FollowTarget):
    """UR10 robot follow target task.

    Args:
        name: Task name.
        target_prim_path: USD path for target.
        target_name: Target name.
        target_position: Initial target position.
        target_orientation: Initial target orientation.
        offset: Task offset.
        ur10_prim_path: USD path for UR10 robot.
        ur10_robot_name: Robot name.
        attach_gripper: Whether to attach gripper.
    """

    def __init__(
        self,
        name: str = "ur10_follow_target",
        target_prim_path: str | None = None,
        target_name: str | None = None,
        target_position: np.ndarray | None = None,
        target_orientation: np.ndarray | None = None,
        offset: np.ndarray | None = None,
        ur10_prim_path: str | None = None,
        ur10_robot_name: str | None = None,
        attach_gripper: bool = False,
    ) -> None:
        if target_orientation is None:
            target_orientation = euler_angles_to_quat(np.array([0, np.pi / 2.0, 0]))
        tasks.FollowTarget.__init__(
            self,
            name=name,
            target_prim_path=target_prim_path,
            target_name=target_name,
            target_position=target_position,
            target_orientation=target_orientation,
            offset=offset,
        )
        self._ur10_prim_path = ur10_prim_path
        self._ur10_robot_name = ur10_robot_name
        self._attach_gripper = attach_gripper
        return

    def set_robot(self) -> UR10:
        """Create and configure the UR10 robot.

        Returns:
            Configured UR10 robot instance.
        """
        if self._ur10_prim_path is None:
            self._ur10_prim_path = find_unique_string_name(
                initial_name="/World/UR10", is_unique_fn=lambda x: not is_prim_path_valid(x)
            )
        if self._ur10_robot_name is None:
            self._ur10_robot_name = find_unique_string_name(
                initial_name="my_ur10", is_unique_fn=lambda x: not self.scene.object_exists(x)
            )
        self._ur10_robot = UR10(
            prim_path=self._ur10_prim_path, name=self._ur10_robot_name, attach_gripper=self._attach_gripper
        )
        self._ur10_robot.set_joints_default_state(
            positions=np.array([-np.pi / 2, -np.pi / 2, -np.pi / 2, -np.pi / 2, np.pi / 2, 0])
        )
        return self._ur10_robot
