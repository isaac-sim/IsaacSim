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

"""Stacking task definition for Universal Robots arms."""

from __future__ import annotations

import numpy as np
from isaacsim.core.api.tasks import Stacking as BaseStacking
from isaacsim.core.utils.prims import is_prim_path_valid
from isaacsim.core.utils.stage import get_stage_units
from isaacsim.core.utils.string import find_unique_string_name
from isaacsim.robot.manipulators.examples.universal_robots import UR10


class Stacking(BaseStacking):
    """UR10 robot stacking task.

    Args:
        name: Task name.
        target_position: Stack target position.
        cube_size: Size of each cube.
        offset: Task offset.
    """

    def __init__(
        self,
        name: str = "ur10_stacking",
        target_position: np.ndarray | None = None,
        cube_size: np.ndarray | None = None,
        offset: np.ndarray | None = None,
    ) -> None:
        if target_position is None:
            target_position = np.array([0.7, 0.7, 0]) / get_stage_units()
        BaseStacking.__init__(
            self,
            name=name,
            cube_initial_positions=np.array([[0.3, 0.3, 0.3], [0.3, -0.3, 0.3]]) / get_stage_units(),
            cube_initial_orientations=None,
            stack_target_position=target_position,
            cube_size=cube_size,
            offset=offset,
        )
        return

    def set_robot(self) -> UR10:
        """Create and configure the UR10 robot.

        Returns:
            Configured UR10 robot instance.
        """
        ur10_prim_path = find_unique_string_name(
            initial_name="/World/ur10", is_unique_fn=lambda x: not is_prim_path_valid(x)
        )
        ur10_robot_name = find_unique_string_name(
            initial_name="my_ur10", is_unique_fn=lambda x: not self.scene.object_exists(x)
        )
        self._ur10_robot = UR10(prim_path=ur10_prim_path, name=ur10_robot_name, attach_gripper=True)
        self._ur10_robot.set_joints_default_state(
            positions=np.array([-np.pi / 2, -np.pi / 2, -np.pi / 2, -np.pi / 2, np.pi / 2, 0])
        )
        return self._ur10_robot

    def pre_step(self, time_step_index: int, simulation_time: float) -> None:
        """Called before each physics step to update gripper.

        Args:
            time_step_index: Current simulation step index.
            simulation_time: Current simulation time.
        """
        BaseStacking.pre_step(self, time_step_index=time_step_index, simulation_time=simulation_time)
        self._ur10_robot.gripper.update()
        return
