# SPDX-FileCopyrightText: Copyright (c) 2018-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

"""Abstract base class for creating interactive Isaac Sim examples and samples.

.. deprecated::
    This module is deprecated. Use :mod:`isaacsim.examples.base.base_sample_experimental`
    (``BaseSample`` from ``isaacsim.examples.base``) which uses the new experimental APIs
    (``SimulationManager``, ``app_utils``, ``stage_utils``) instead of the legacy ``World`` API.
"""

from __future__ import annotations

import gc
import warnings
from abc import abstractmethod

from isaacsim.core.api import World
from isaacsim.core.api.scenes.scene import Scene
from isaacsim.core.utils.stage import create_new_stage_async, update_stage_async
from isaacsim.core.utils.viewports import set_camera_view


class BaseSample(object):
    """Abstract base class for creating interactive Isaac Sim examples and samples.

    This class provides a standardized framework for building interactive demonstrations and examples in Isaac Sim.
    It manages the simulation world lifecycle, handles common operations like loading, resetting, and clearing scenes,
    and defines abstract methods that subclasses must implement to create specific sample content.

    The class automatically manages world settings including physics timestep, stage units, and rendering timestep.
    It provides async methods for world operations and integrates with the Isaac Sim task system for physics callbacks.
    Subclasses need to implement scene setup, post-load initialization, pre/post reset handling, and cleanup logic.

    Key lifecycle methods that subclasses must implement:
    - setup_scene(): Configure the world with assets and tasks
    - setup_post_load(): Initialize variables after first world reset
    - setup_pre_reset(): Prepare for world reset (remove callbacks, reset controllers)
    - setup_post_reset(): Handle post-reset operations
    - setup_post_clear(): Clean up after clearing the world

    The class handles camera positioning, physics callback management, and ensures proper cleanup during extension
    hot reloading scenarios.
    """

    def __init__(self) -> None:
        warnings.warn(
            "BaseSample from isaacsim.examples.interactive.base_sample is deprecated. "
            "Use BaseSample from isaacsim.examples.base (base_sample_experimental) instead.",
            DeprecationWarning,
            stacklevel=2,
        )
        self._world = None
        self._current_tasks = None
        self._world_settings = {"physics_dt": 1.0 / 60.0, "stage_units_in_meters": 1.0, "rendering_dt": 1.0 / 60.0}
        # self._logging_info = ""
        return

    def get_world(self) -> World | None:
        """The current World instance.

        Returns:
            The World instance or None if not initialized.
        """
        return self._world

    def set_world_settings(
        self, physics_dt: float = None, stage_units_in_meters: float = None, rendering_dt: float = None
    ) -> None:
        """Updates the world settings configuration.

        Args:
            physics_dt: Physics timestep in seconds.
            stage_units_in_meters: Stage units conversion factor to meters.
            rendering_dt: Rendering timestep in seconds.
        """
        if physics_dt is not None:
            self._world_settings["physics_dt"] = physics_dt
        if stage_units_in_meters is not None:
            self._world_settings["stage_units_in_meters"] = stage_units_in_meters
        if rendering_dt is not None:
            self._world_settings["rendering_dt"] = rendering_dt
        return

    async def load_world_async(self) -> None:
        """Function called when clicking load buttton."""
        await create_new_stage_async()
        self._world = World(**self._world_settings)
        await self._world.initialize_simulation_context_async()
        self.setup_scene()
        set_camera_view(eye=[1.5, 1.5, 1.5], target=[0.01, 0.01, 0.01], camera_prim_path="/OmniverseKit_Persp")
        self._current_tasks = self._world.get_current_tasks()
        await self._world.reset_async()
        await self._world.pause_async()
        await self.setup_post_load()
        if len(self._current_tasks) > 0:
            self._world.add_physics_callback("tasks_step", self._world.step_async)
        return

    async def reset_async(self) -> None:
        """Function called when clicking reset buttton."""
        if self._world.is_tasks_scene_built() and len(self._current_tasks) > 0:
            if self._world.physics_callback_exists("tasks_step"):
                self._world.remove_physics_callback("tasks_step")
        await self._world.play_async()
        await update_stage_async()
        await self.setup_pre_reset()
        await self._world.reset_async()
        await self._world.pause_async()
        await self.setup_post_reset()
        if self._world.is_tasks_scene_built() and len(self._current_tasks) > 0:
            self._world.add_physics_callback("tasks_step", self._world.step_async)
        return

    @abstractmethod
    def setup_scene(self, scene: Scene) -> None:
        """Used to setup anything in the world, adding tasks happen here for instance.

        Args:
            scene: The scene to set up with sample assets.
        """
        return

    @abstractmethod
    async def setup_post_load(self) -> None:
        """Called after first reset of the world when pressing load, intializing provate variables happen here."""
        return

    @abstractmethod
    async def setup_pre_reset(self) -> None:
        """Called in reset button before resetting the world to remove a physics callback for instance or a controller reset."""
        return

    @abstractmethod
    async def setup_post_reset(self) -> None:
        """Called in reset button after resetting the world which includes one step with rendering."""
        return

    @abstractmethod
    async def setup_post_clear(self) -> None:
        """Called after clicking clear button or after creating a new stage and clearing the instance of the world with its callbacks."""
        return

    # def log_info(self, info):
    #     self._logging_info += str(info) + "\n"
    #     return

    def _world_cleanup(self) -> None:
        """Cleans up the world instance by stopping simulation and clearing callbacks."""
        if self._world is not None:
            self._world.stop()
            self._world.clear_all_callbacks()
        self._current_tasks = None
        self.world_cleanup()
        return

    def world_cleanup(self) -> None:
        """Function called when extension shutdowns and starts again, (hot reloading feature)."""
        return

    async def clear_async(self) -> None:
        """Function called when clicking clear button."""
        if self._world is not None:
            # Ensure the simulation is fully stopped and the app processes at least one update
            # before we start tearing down callbacks and/or closing the stage.
            await self._world.stop_async()
            await update_stage_async()
            self._world_cleanup()
            self._world.clear_instance()
            self._world = None
            gc.collect()
        # Create a fresh stage after cleaning up the existing World/callbacks.
        # Creating a new stage first can invalidate the World/SimulationContext while callbacks
        # are still live, which risks use-after-free during stage teardown.
        await create_new_stage_async()
        await self.setup_post_clear()
        return
