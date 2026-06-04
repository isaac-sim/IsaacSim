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

"""Abstract task implementation for robot end effector target following scenarios."""

from __future__ import annotations

from abc import ABC, abstractmethod
from collections import OrderedDict

import numpy as np
from isaacsim.core.api.objects import VisualCuboid
from isaacsim.core.api.scenes.scene import Scene
from isaacsim.core.api.tasks import BaseTask
from isaacsim.core.prims import SingleXFormPrim
from isaacsim.core.utils.prims import is_prim_path_valid
from isaacsim.core.utils.rotations import euler_angles_to_quat
from isaacsim.core.utils.stage import get_stage_units
from isaacsim.core.utils.string import find_unique_string_name


class FollowTarget(ABC, BaseTask):
    """Abstract task for following a target with a robot end effector.

    Args:
        name: Task name identifier.
        target_prim_path: USD path for the target prim.
        target_name: Name for the target object.
        target_position: Initial target position.
        target_orientation: Initial target orientation.
        offset: Offset for all task objects.

    """

    def __init__(
        self,
        name: str,
        target_prim_path: str | None = None,
        target_name: str | None = None,
        target_position: np.ndarray | None = None,
        target_orientation: np.ndarray | None = None,
        offset: np.ndarray | None = None,
    ) -> None:
        BaseTask.__init__(self, name=name, offset=offset)
        self._robot = None
        self._target_name = target_name
        self._target = None
        self._target_prim_path = target_prim_path
        self._target_position = target_position
        self._target_orientation = target_orientation
        self._target_visual_material = None
        self._obstacle_cubes = OrderedDict()
        if self._target_position is None:
            self._target_position = np.array([0, 0.1, 0.7]) / get_stage_units()
        return

    def set_up_scene(self, scene: Scene) -> None:
        """Set up the scene with target and robot.

        Args:
            scene: The scene to populate.

        """
        super().set_up_scene(scene)
        scene.add_default_ground_plane()
        if self._target_orientation is None:
            self._target_orientation = euler_angles_to_quat(np.array([-np.pi, 0, np.pi]))
        if self._target_prim_path is None:
            self._target_prim_path = find_unique_string_name(
                initial_name="/World/TargetCube", is_unique_fn=lambda x: not is_prim_path_valid(x)
            )
        if self._target_name is None:
            self._target_name = find_unique_string_name(
                initial_name="target", is_unique_fn=lambda x: not self.scene.object_exists(x)
            )
        self.set_params(
            target_prim_path=self._target_prim_path,
            target_position=self._target_position,
            target_orientation=self._target_orientation,
            target_name=self._target_name,
        )
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
        target_prim_path: str | None = None,
        target_name: str | None = None,
        target_position: np.ndarray | None = None,
        target_orientation: np.ndarray | None = None,
    ) -> None:
        """Set task parameters including target pose.

        Args:
            target_prim_path: USD path for target.
            target_name: Name for target object.
            target_position: Target position.
            target_orientation: Target orientation.

        """
        if target_prim_path is not None:
            if self._target is not None:
                del self._task_objects[self._target.name]
            if is_prim_path_valid(target_prim_path):
                self._target = self.scene.add(
                    SingleXFormPrim(
                        prim_path=target_prim_path,
                        position=target_position,
                        orientation=target_orientation,
                        name=target_name,
                    )
                )
            else:
                self._target = self.scene.add(
                    VisualCuboid(
                        name=target_name,
                        prim_path=target_prim_path,
                        position=target_position,
                        orientation=target_orientation,
                        color=np.array([1, 0, 0]),
                        size=1.0,
                        scale=np.array([0.03, 0.03, 0.03]) / get_stage_units(),
                    )
                )
            self._task_objects[self._target.name] = self._target
            self._target_visual_material = self._target.get_applied_visual_material()
            if self._target_visual_material is not None:
                if hasattr(self._target_visual_material, "set_color"):
                    self._target_visual_material.set_color(np.array([1, 0, 0]))
        else:
            if self._target is None:
                raise RuntimeError("Cannot update target pose before set_up_scene() has been called.")
            self._target.set_local_pose(translation=target_position, orientation=target_orientation)
        return

    def get_params(self) -> dict:
        """Get task parameters.

        Returns:
            Dictionary of task parameters.

        """
        params_representation = {}
        params_representation["target_prim_path"] = {"value": self._target.prim_path, "modifiable": True}
        params_representation["target_name"] = {"value": self._target.name, "modifiable": True}
        position, orientation = self._target.get_local_pose()
        params_representation["target_position"] = {"value": position, "modifiable": True}
        params_representation["target_orientation"] = {"value": orientation, "modifiable": True}
        params_representation["robot_name"] = {"value": self._robot.name, "modifiable": False}
        return params_representation

    def get_observations(self) -> dict:
        """Get current task observations.

        Returns:
            Dictionary with robot and target observations.

        """
        joints_state = self._robot.get_joints_state()
        target_position, target_orientation = self._target.get_local_pose()

        # The target cannot be below the ground plane; use a copy to avoid mutating the prim's data
        clamped_position = np.array(target_position)
        if clamped_position[2] <= (0.035 / get_stage_units()):
            clamped_position[2] = 0.035 / get_stage_units()

        return {
            self._robot.name: {
                "joint_positions": np.array(joints_state.positions),
                "joint_velocities": np.array(joints_state.velocities),
            },
            self._target.name: {"position": clamped_position, "orientation": np.array(target_orientation)},
        }

    def calculate_metrics(self) -> dict:
        """Calculate task metrics.

        Raises:
            NotImplementedError: Must be implemented by subclass.

        Returns:
            Dictionary containing calculated task metrics.

        """
        raise NotImplementedError

    def is_done(self) -> bool:
        """Check if task is complete.

        Raises:
            NotImplementedError: Must be implemented by subclass.

        Returns:
            Whether the task is complete.

        """
        raise NotImplementedError

    def target_reached(self) -> bool:
        """Check if the end effector has reached the target.

        Returns:
            True if target is reached, False otherwise.

        """
        end_effector_position, _ = self._robot.end_effector.get_world_pose()
        target_position, _ = self._target.get_world_pose()
        if np.mean(np.abs(np.array(end_effector_position) - np.array(target_position))) < (0.035 / get_stage_units()):
            return True
        else:
            return False

    def pre_step(self, time_step_index: int, simulation_time: float) -> None:
        """Called before each physics step to update target visual.

        Args:
            time_step_index: Current simulation step index.
            simulation_time: Current simulation time.

        """
        if self._target_visual_material is not None:
            if hasattr(self._target_visual_material, "set_color"):
                if self.target_reached():
                    self._target_visual_material.set_color(color=np.array([0, 1.0, 0]))
                else:
                    self._target_visual_material.set_color(color=np.array([1.0, 0, 0]))

        return

    def post_reset(self) -> None:
        """Called after world reset."""
        return

    def add_obstacle(self, position: np.ndarray = None) -> None:
        """Add an obstacle cube to the scene.

        Args:
            position: Position for the obstacle.

        Returns:
            The created obstacle cube object.

        """
        # TODO: move to task frame if there is one
        cube_prim_path = find_unique_string_name(
            initial_name="/World/ObstacleCube", is_unique_fn=lambda x: not is_prim_path_valid(x)
        )
        cube_name = find_unique_string_name(initial_name="cube", is_unique_fn=lambda x: not self.scene.object_exists(x))
        if position is None:
            position = np.array([0.1, 0.1, 0.7]) / get_stage_units()
        cube = self.scene.add(
            VisualCuboid(
                name=cube_name,
                position=position + self._offset,
                prim_path=cube_prim_path,
                size=0.1 / get_stage_units(),
                color=np.array([0, 0, 1.0]),
            )
        )
        self._obstacle_cubes[cube.name] = cube
        return cube

    def remove_obstacle(self, name: str | None = None) -> None:
        """Remove an obstacle from the scene.

        Args:
            name: Name of obstacle to remove. Defaults to last added.

        """
        if name is not None:
            self.scene.remove_object(name)
            del self._obstacle_cubes[name]
        else:
            if not self._obstacle_cubes:
                raise IndexError("No obstacles to remove.")
            obstacle_to_delete = list(self._obstacle_cubes.keys())[-1]
            self.scene.remove_object(obstacle_to_delete)
            del self._obstacle_cubes[obstacle_to_delete]
        return

    def get_obstacle_to_delete(self) -> object:
        """Get the last obstacle that would be deleted.

        Returns:
            The obstacle object to be deleted.

        """
        if not self._obstacle_cubes:
            raise IndexError("No obstacles exist.")
        obstacle_to_delete = list(self._obstacle_cubes.keys())[-1]
        return self.scene.get_object(obstacle_to_delete)

    def obstacles_exist(self) -> bool:
        """Check if any obstacles exist in the scene.

        Returns:
            True if obstacles exist, False otherwise.

        """
        if len(self._obstacle_cubes) > 0:
            return True
        else:
            return False

    def cleanup(self) -> None:
        """Remove all obstacles from the scene."""
        obstacles_to_delete = list(self._obstacle_cubes.keys())
        for obstacle_to_delete in obstacles_to_delete:
            self.scene.remove_object(obstacle_to_delete)
            del self._obstacle_cubes[obstacle_to_delete]
        return
