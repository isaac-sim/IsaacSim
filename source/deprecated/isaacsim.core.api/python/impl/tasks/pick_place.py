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

"""Abstract base class for pick and place tasks involving cube manipulation with a robot."""

from __future__ import annotations

from abc import ABC, abstractmethod

import numpy as np
from isaacsim.core.api.objects import DynamicCuboid
from isaacsim.core.api.scenes.scene import Scene
from isaacsim.core.api.tasks import BaseTask
from isaacsim.core.utils.prims import is_prim_path_valid
from isaacsim.core.utils.stage import get_stage_units
from isaacsim.core.utils.string import find_unique_string_name


class PickPlace(ABC, BaseTask):
    """Abstract task for picking and placing a cube with a robot.

    Args:
        name: Task name identifier.
        cube_initial_position: Initial cube position.
        cube_initial_orientation: Initial cube orientation.
        target_position: Target position for placing.
        cube_size: Size of the cube.
        offset: Offset for all task objects.

    """

    def __init__(
        self,
        name: str,
        cube_initial_position: np.ndarray | None = None,
        cube_initial_orientation: np.ndarray | None = None,
        target_position: np.ndarray | None = None,
        cube_size: np.ndarray | None = None,
        offset: np.ndarray | None = None,
    ) -> None:
        BaseTask.__init__(self, name=name, offset=offset)
        self._robot = None
        self._target_cube = None
        self._cube = None
        self._cube_initial_position = cube_initial_position
        self._cube_initial_orientation = cube_initial_orientation
        self._target_position = target_position
        self._cube_size = cube_size
        if self._cube_size is None:
            self._cube_size = np.array([0.0515, 0.0515, 0.0515]) / get_stage_units()
        if self._cube_initial_position is None:
            self._cube_initial_position = np.array([0.3, 0.3, 0.3]) / get_stage_units()
        if self._cube_initial_orientation is None:
            self._cube_initial_orientation = np.array([1, 0, 0, 0])
        if self._target_position is None:
            self._target_position = np.array([-0.3, -0.3, 0]) / get_stage_units()
            self._target_position[2] = self._cube_size[2] / 2.0
        self._target_position = self._target_position + self._offset
        return

    def set_up_scene(self, scene: Scene) -> None:
        """Set up the scene with cube and robot.

        Args:
            scene: The scene to populate.

        """
        super().set_up_scene(scene)
        scene.add_default_ground_plane()
        cube_prim_path = find_unique_string_name(
            initial_name="/World/Cube", is_unique_fn=lambda x: not is_prim_path_valid(x)
        )
        cube_name = find_unique_string_name(initial_name="cube", is_unique_fn=lambda x: not self.scene.object_exists(x))
        self._cube = scene.add(
            DynamicCuboid(
                name=cube_name,
                position=self._cube_initial_position,
                orientation=self._cube_initial_orientation,
                prim_path=cube_prim_path,
                scale=self._cube_size,
                size=1.0,
                color=np.array([0, 0, 1]),
            )
        )
        self._task_objects[self._cube.name] = self._cube
        self._robot = self.set_robot()
        scene.add(self._robot)
        self._task_objects[self._robot.name] = self._robot
        self._move_task_objects_to_their_frame()
        return

    @abstractmethod
    def set_robot(self) -> None:
        """Create and configure the robot for the task."""
        raise NotImplementedError

    def set_params(
        self,
        cube_position: np.ndarray | None = None,
        cube_orientation: np.ndarray | None = None,
        target_position: np.ndarray | None = None,
    ) -> None:
        """Set task parameters for cube position, orientation, and target position.

        Args:
            cube_position: New position for the cube.
            cube_orientation: New orientation for the cube.
            target_position: New target position for placing.

        """
        if target_position is not None:
            self._target_position = target_position
        if cube_position is not None or cube_orientation is not None:
            self._cube.set_local_pose(translation=cube_position, orientation=cube_orientation)
        return

    def get_params(self) -> dict:
        """Get current task parameters including cube and robot states.

        Returns:
            Dictionary containing task parameters with values and modifiability flags.

        """
        params_representation = {}
        position, orientation = self._cube.get_local_pose()
        params_representation["cube_position"] = {"value": position, "modifiable": True}
        params_representation["cube_orientation"] = {"value": orientation, "modifiable": True}
        params_representation["target_position"] = {"value": self._target_position, "modifiable": True}
        params_representation["cube_name"] = {"value": self._cube.name, "modifiable": False}
        params_representation["robot_name"] = {"value": self._robot.name, "modifiable": False}
        return params_representation

    def get_observations(self) -> dict:
        """Get current task observations.

        Returns:
            Dictionary with cube and robot observations.

        """
        joints_state = self._robot.get_joints_state()
        cube_position, cube_orientation = self._cube.get_local_pose()
        robot_obs = {
            "joint_positions": joints_state.positions,
        }
        if hasattr(self._robot, "end_effector") and self._robot.end_effector is not None:
            end_effector_position, _ = self._robot.end_effector.get_local_pose()
            robot_obs["end_effector_position"] = end_effector_position
        return {
            self._cube.name: {
                "position": cube_position,
                "orientation": cube_orientation,
                "target_position": self._target_position,
            },
            self._robot.name: robot_obs,
        }

    def pre_step(self, time_step_index: int, simulation_time: float) -> None:
        """Called before each physics step.

        Args:
            time_step_index: Current simulation step index.
            simulation_time: Current simulation time.

        """
        return

    def post_reset(self) -> None:
        """Reset the robot gripper to open position after task reset."""
        from isaacsim.robot.manipulators.grippers.parallel_gripper import ParallelGripper

        if isinstance(self._robot.gripper, ParallelGripper):
            self._robot.gripper.set_joint_positions(self._robot.gripper.joint_opened_positions)
        return

    def calculate_metrics(self) -> dict:
        """Calculate task metrics.

        Returns:
            Dictionary containing calculated task metrics.

        """
        raise NotImplementedError

    def is_done(self) -> bool:
        """Check if task is complete.

        Returns:
            Whether the task is complete.

        """
        raise NotImplementedError
