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

"""Base class for creating and managing simulation tasks in Isaac Sim environments."""

from __future__ import annotations

import numpy as np
from isaacsim.core.api.scenes.scene import Scene
from isaacsim.core.api.simulation_context import SimulationContext
from isaacsim.core.utils.stage import get_current_stage, get_stage_units


class BaseTask(object):
    """This class provides a way to set up a task in a scene and modularize adding objects to stage,.

    getting observations needed for the behavioral layer, calculating metrics needed about the task,
    calling certain things pre-stepping, creating multiple tasks at the same time and much more.

    Checkout the required tutorials at https://docs.isaacsim.omniverse.nvidia.com/latest/index.html

    Args:
        name: needs to be unique if added to the World.
        offset: offset applied to all assets of the task.

    Raises:
        RuntimeError: If the current USD stage or USD stage meters-to-unit conversion factor is not valid.
    """

    def __init__(self, name: str, offset: np.ndarray | None = None) -> None:
        self._scene = None
        self._name = name
        self._offset = offset
        self._task_objects = {}
        if self._offset is None:
            self._offset = np.array([0.0, 0.0, 0.0])

        if not get_current_stage():
            raise RuntimeError(
                f"Cannot create task '{self._name}' because no USD stage is currently open. "
                "Create or open a stage before constructing a task."
            )
        stage_units = get_stage_units()
        if not stage_units or stage_units <= 0.0:
            raise RuntimeError(f"Invalid USD stage meters-to-unit conversion factor: {stage_units}")

        self._device = None
        if SimulationContext.instance() is not None:
            self._device = SimulationContext.instance().device
        return

    @property
    def device(self) -> str:
        """Device used for simulation computations.

        Returns:
            The simulation device instance.

        """
        return self._device

    @property
    def scene(self) -> Scene:
        """Scene instance associated with this task.

        Returns:
            The scene instance associated with this task.

        """
        return self._scene

    @property
    def name(self) -> str:
        """Name of the task.

        Returns:
            The task name.

        """
        return self._name

    def set_up_scene(self, scene: Scene) -> None:
        """Adding assets to the stage as well as adding the encapsulated objects such as SingleXFormPrim..etc.

               to the task_objects happens here.

        Args:
            scene: The scene to set up with task assets.

        """
        self._scene = scene
        return

    def _move_task_objects_to_their_frame(self) -> None:
        """Move all registered task objects to their final position using the task offset."""
        # if self._task_path:
        # TODO: assumption all task objects are under the same parent
        # Specifying a task path has many limitations atm
        # SingleXFormPrim(prim_path=self._task_path, position=self._offset)
        # for object_name, task_object in self._task_objects.items():
        #     new_prim_path = self._task_path + "/" + task_object.prim_path.split("/")[-1]
        #     task_object.change_prim_path(new_prim_path)
        #     current_position, current_orientation = task_object.get_world_pose()
        for object_name, task_object in self._task_objects.items():
            current_position, current_orientation = task_object.get_world_pose()
            task_object.set_world_pose(position=current_position + self._offset, orientation=current_orientation)
            task_object.set_default_state(position=current_position + self._offset, orientation=current_orientation)
        return

    def get_task_objects(self) -> dict:
        """Get all objects registered with the task.

        Returns:
            Dictionary of task objects keyed by name.

        """
        return self._task_objects

    def get_observations(self) -> dict:
        """Returns current observations from the objects needed for the behavioral layer.

        Raises:
            NotImplementedError: Must be implemented by subclass.

        Returns:
            Dictionary containing task-specific observations.

        """
        raise NotImplementedError

    def calculate_metrics(self) -> dict:
        """Calculate and return task metrics.

        Raises:
            NotImplementedError: Must be implemented by subclass.

        Returns:
            Dictionary containing calculated task metrics.

        """
        raise NotImplementedError

    def is_done(self) -> bool:
        """Returns True of the task is done.

        Raises:
            NotImplementedError: Must be implemented by subclass.

        Returns:
            True if the task is complete, False otherwise.

        """
        raise NotImplementedError

    def pre_step(self, time_step_index: int, simulation_time: float) -> None:
        """Called before stepping the physics simulation.

        Args:
            time_step_index: Current physics step index.
            simulation_time: Current simulation time in seconds.

        """
        return

    def post_reset(self) -> None:
        """Calls while doing a .reset() on the world."""
        return

    def get_description(self) -> str:
        """Get a description of the task.

        Returns:
            A string describing the task.

        """
        return ""

    def cleanup(self) -> None:
        """Called before calling a reset() on the world to removed temporary objects that were added during.

        simulation for instance.
        """
        return

    def set_params(self, *args: object, **kwargs: object) -> None:
        """Changes the modifiable parameters of the task.

        Args:
            *args: Variable length argument list.
            **kwargs: Additional keyword arguments for task parameters.

        Raises:
            NotImplementedError: Must be implemented by subclass.

        """
        raise NotImplementedError

    def get_params(self) -> dict:
        """Gets the parameters of the task.

               This is defined differently for each task in order to access the task's objects and values.
               Note that this is different from get_observations.
               Things like the robot name, block name..etc can be defined here for faster retrieval.
               should have the form of params_representation["param_name"] = {"value": param_value, "modifiable": bool}.

        Raises:
            NotImplementedError: Must be implemented by subclass.

        Returns:
            Defined parameters of the task.

        """
        raise NotImplementedError
