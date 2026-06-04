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

"""Follow-target task definition for the Franka robot."""

from __future__ import annotations

import isaacsim.core.api.tasks as tasks
import numpy as np
from isaacsim.core.utils.prims import is_prim_path_valid
from isaacsim.core.utils.string import find_unique_string_name
from isaacsim.robot.manipulators.examples.franka import Franka


class FollowTarget(tasks.FollowTarget):
    """Franka robot follow target task.

    Args:
        name: Task name.
        target_prim_path: USD path for target.
        target_name: Target name.
        target_position: Initial target position.
        target_orientation: Initial target orientation.
        offset: Task offset.
        franka_prim_path: USD path for Franka robot.
        franka_robot_name: Robot name.
    """

    def __init__(
        self,
        name: str = "franka_follow_target",
        target_prim_path: str | None = None,
        target_name: str | None = None,
        target_position: np.ndarray | None = None,
        target_orientation: np.ndarray | None = None,
        offset: np.ndarray | None = None,
        franka_prim_path: str | None = None,
        franka_robot_name: str | None = None,
    ) -> None:
        tasks.FollowTarget.__init__(
            self,
            name=name,
            target_prim_path=target_prim_path,
            target_name=target_name,
            target_position=target_position,
            target_orientation=target_orientation,
            offset=offset,
        )
        self._franka_prim_path = franka_prim_path
        self._franka_robot_name = franka_robot_name
        return

    def set_robot(self) -> Franka:
        """Create and configure the Franka robot.

        Returns:
            Configured Franka robot instance.
        """
        if self._franka_prim_path is None:
            self._franka_prim_path = find_unique_string_name(
                initial_name="/World/Franka", is_unique_fn=lambda x: not is_prim_path_valid(x)
            )
        if self._franka_robot_name is None:
            self._franka_robot_name = find_unique_string_name(
                initial_name="my_franka", is_unique_fn=lambda x: not self.scene.object_exists(x)
            )
        return Franka(prim_path=self._franka_prim_path, name=self._franka_robot_name)
