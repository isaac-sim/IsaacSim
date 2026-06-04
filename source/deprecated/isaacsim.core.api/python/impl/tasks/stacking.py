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

"""Provides an abstract base class for stacking multiple cubes with a robot."""

from __future__ import annotations

from abc import ABC, abstractmethod

import numpy as np
from isaacsim.core.api.objects import DynamicCuboid
from isaacsim.core.api.scenes.scene import Scene
from isaacsim.core.api.tasks import BaseTask
from isaacsim.core.utils.prims import is_prim_path_valid
from isaacsim.core.utils.stage import get_stage_units
from isaacsim.core.utils.string import find_unique_string_name


class Stacking(ABC, BaseTask):
    """Abstract task for stacking multiple cubes with a robot.

    Args:
        name: Task name identifier.
        cube_initial_positions: Initial positions for all cubes.
        cube_initial_orientations: Initial orientations for cubes.
        stack_target_position: Position to stack cubes at.
        cube_size: Size of each cube.
        offset: Offset for all task objects.

    """

    def __init__(
        self,
        name: str,
        cube_initial_positions: np.ndarray,
        cube_initial_orientations: np.ndarray | None = None,
        stack_target_position: np.ndarray | None = None,
        cube_size: np.ndarray | None = None,
        offset: np.ndarray | None = None,
    ) -> None:
        BaseTask.__init__(self, name=name, offset=offset)
        self._robot = None
        self._num_of_cubes = cube_initial_positions.shape[0]
        self._cube_initial_positions = cube_initial_positions
        self._cube_initial_orientations = cube_initial_orientations
        if self._cube_initial_orientations is None:
            self._cube_initial_orientations = [None] * self._num_of_cubes
        self._stack_target_position = stack_target_position
        self._cube_size = cube_size
        if self._cube_size is None:
            self._cube_size = np.array([0.0515, 0.0515, 0.0515]) / get_stage_units()
        if stack_target_position is None:
            self._stack_target_position = np.array([-0.3, -0.3, 0]) / get_stage_units()
        self._stack_target_position = self._stack_target_position + self._offset
        self._cubes = []
        return

    def set_up_scene(self, scene: Scene) -> None:
        """Set up the scene with cubes and robot.

        Args:
            scene: The scene to populate.

        """
        super().set_up_scene(scene)
        scene.add_default_ground_plane()
        for i in range(self._num_of_cubes):
            color = np.random.uniform(size=(3,))
            cube_prim_path = find_unique_string_name(
                initial_name="/World/Cube", is_unique_fn=lambda x: not is_prim_path_valid(x)
            )
            cube_name = find_unique_string_name(
                initial_name="cube", is_unique_fn=lambda x: not self.scene.object_exists(x)
            )
            self._cubes.append(
                scene.add(
                    DynamicCuboid(
                        name=cube_name,
                        position=self._cube_initial_positions[i],
                        orientation=self._cube_initial_orientations[i],
                        prim_path=cube_prim_path,
                        scale=self._cube_size,
                        size=1.0,
                        color=color,
                    )
                )
            )
            self._task_objects[self._cubes[-1].name] = self._cubes[-1]
        self._robot = self.set_robot()
        scene.add(self._robot)
        self._task_objects[self._robot.name] = self._robot
        self._move_task_objects_to_their_frame()
        return

    @abstractmethod
    def set_robot(self) -> None:
        """Create and return the robot for this task.

        Raises:
            NotImplementedError: Must be implemented by subclass.

        """
        raise NotImplementedError

    def set_params(
        self,
        cube_name: str | None = None,
        cube_position: np.ndarray | None = None,
        cube_orientation: np.ndarray | None = None,
        stack_target_position: np.ndarray | None = None,
    ) -> None:
        """Set task parameters.

        Args:
            cube_name: Name of cube to modify.
            cube_position: New position for cube.
            cube_orientation: New orientation for cube.
            stack_target_position: New stack target position.

        """
        if stack_target_position is not None:
            self._stack_target_position = stack_target_position
        if cube_name is not None:
            self._task_objects[cube_name].set_local_pose(translation=cube_position, orientation=cube_orientation)
        return

    def get_params(self) -> dict:
        """Get task parameters.

        Returns:
            Dictionary of task parameters.

        """
        params_representation = {}
        params_representation["stack_target_position"] = {"value": self._stack_target_position, "modifiable": True}
        params_representation["robot_name"] = {"value": self._robot.name, "modifiable": False}
        return params_representation

    def get_observations(self) -> dict:
        """Get current task observations.

        Returns:
            Dictionary with cube and robot observations.

        """
        joints_state = self._robot.get_joints_state()
        end_effector_position, _ = self._robot.end_effector.get_local_pose()
        observations = {
            self._robot.name: {
                "joint_positions": joints_state.positions,
                "end_effector_position": end_effector_position,
            }
        }
        for i in range(self._num_of_cubes):
            cube_position, cube_orientation = self._cubes[i].get_local_pose()
            observations[self._cubes[i].name] = {
                "position": cube_position,
                "orientation": cube_orientation,
                "target_position": np.array(
                    [
                        self._stack_target_position[0],
                        self._stack_target_position[1],
                        (self._cube_size[2] * i) + self._cube_size[2] / 2.0,
                    ]
                ),
            }
        return observations

    def pre_step(self, time_step_index: int, simulation_time: float) -> None:
        """Called before each physics step.

        Args:
            time_step_index: Current simulation step index.
            simulation_time: Current simulation time.

        """
        return

    def post_reset(self) -> None:
        """Called after world reset to open gripper."""
        from isaacsim.robot.manipulators.grippers.parallel_gripper import ParallelGripper

        if isinstance(self._robot.gripper, ParallelGripper):
            self._robot.gripper.set_joint_positions(self._robot.gripper.joint_opened_positions)
        return

    def get_cube_names(self) -> list[str]:
        """Get the names of all cubes in the task.

        Returns:
            List of cube names.

        """
        cube_names = []
        for i in range(self._num_of_cubes):
            cube_names.append(self._cubes[i].name)
        return cube_names

    def calculate_metrics(self) -> dict:
        """Calculate task metrics.

        Raises:
            NotImplementedError: Must be implemented by subclass.

        Returns:
            Dictionary of task metrics.

        """
        raise NotImplementedError

    def is_done(self) -> bool:
        """Check if task is complete.

        Raises:
            NotImplementedError: Must be implemented by subclass.

        Returns:
            True if task is done, False otherwise.

        """
        raise NotImplementedError
