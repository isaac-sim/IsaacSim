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

"""Base class for creating Isaac Sim simulation samples with structured lifecycle management."""

import gc
from abc import abstractmethod

import carb
import isaacsim.core.experimental.utils.app as app_utils
import isaacsim.core.experimental.utils.stage as stage_utils
import omni.kit.app
import omni.physics.core
from isaacsim.core.rendering_manager import RenderingManager, ViewportManager
from isaacsim.core.simulation_manager import PhysicsScene, PhysxScene, SimulationManager


class BaseSample(object):
    """Base class for Isaac Sim example samples with simulation and rendering management."""

    def __init__(self) -> None:
        self._physics_sim_interface = omni.physics.core.get_physics_simulation_interface()
        self._world_settings = {
            "physics_dt": 1.0 / 60.0,
            "stage_units_in_meters": 1.0,
            "rendering_dt": 1.0 / 60.0,
            "device": "cpu",
        }
        self._logging_info = ""

    def set_world_settings(
        self,
        physics_dt: float | None = None,
        stage_units_in_meters: float | None = None,
        rendering_dt: float | None = None,
        device: str | None = None,
    ) -> None:
        """Updates the world settings with the provided values.

        Args:
            physics_dt: Physics simulation timestep in seconds.
            stage_units_in_meters: Number of meters per stage unit.
            rendering_dt: Rendering timestep in seconds.
            device: Physics simulation device (``"cpu"`` or ``"cuda"``).
        """
        if physics_dt is not None:
            self._world_settings["physics_dt"] = physics_dt
        if stage_units_in_meters is not None:
            self._world_settings["stage_units_in_meters"] = stage_units_in_meters
        if rendering_dt is not None:
            self._world_settings["rendering_dt"] = rendering_dt
        if device is not None:
            self._world_settings["device"] = device

    async def load_world_async(self) -> None:
        """Function called when clicking load button."""
        await stage_utils.create_new_stage_async()

        # Set up stage properties
        stage_utils.set_stage_up_axis("Z")
        stage_utils.set_stage_units(meters_per_unit=self._world_settings["stage_units_in_meters"])
        self.setup_scene()
        ViewportManager.set_camera_view(eye=[1.5, 1.5, 1.5], target=[0.01, 0.01, 0.01], camera="/OmniverseKit_Persp")

        await omni.kit.app.get_app().next_update_async()

        SimulationManager.setup_simulation(
            dt=self._world_settings["physics_dt"],
            device=self._world_settings["device"],
        )
        RenderingManager.set_dt(dt=self._world_settings["rendering_dt"])
        await omni.kit.app.get_app().next_update_async()

        app_utils.play()
        await omni.kit.app.get_app().next_update_async()

        await self.setup_post_load()

    async def reset_async(self) -> None:
        """Function called when clicking reset button."""
        await self.setup_pre_reset()

        app_utils.stop()
        await omni.kit.app.get_app().next_update_async()

        self._reapply_physics_device()

        app_utils.play()
        await omni.kit.app.get_app().next_update_async()

        await self.setup_post_reset()

    def _reapply_physics_device(self) -> None:
        """Re-apply physics device settings on physics scene prims before play.

        In complex scenes with multiple USD references (e.g. robo_party), physics scene
        prims can become stale after stop(), causing get_physics_scene_paths() to return
        empty. When that happens, we create/configure a PhysxScene at the default path
        so that initialize_physics() finds it in the cache and skips recreating it with
        GPU defaults.
        """
        device = self._world_settings.get("device", "cpu")
        is_cpu = device == "cpu"

        scene_paths = PhysicsScene.get_physics_scene_paths()
        if not scene_paths:
            scene_paths = ["/PhysicsScene"]

        for scene_path in scene_paths:
            physx_scene = PhysxScene(scene_path)
            physx_scene.set_dt(self._world_settings["physics_dt"])
            physx_scene.set_enabled_gpu_dynamics(not is_cpu)
            physx_scene.set_broadphase_type("MBP" if is_cpu else "GPU")

        carb.settings.get_settings().set_bool("/physics/suppressReadback", not is_cpu)

    @abstractmethod
    def setup_scene(self) -> None:
        """Used to setup anything in the world, adding tasks happen here for instance."""

    @abstractmethod
    async def setup_post_load(self) -> None:
        """Called after first reset of the world when pressing load,.

        intializing private variables happen here.
        """

    @abstractmethod
    async def setup_pre_reset(self) -> None:
        """Called in reset button before resetting the world.

        to remove a physics callback for instance or a controller reset.
        """

    @abstractmethod
    async def setup_post_reset(self) -> None:
        """Called in reset button after resetting the world which includes one step with rendering."""

    @abstractmethod
    async def setup_post_clear(self) -> None:
        """Called after clicking clear button.

        or after creating a new stage and clearing the instance of the world with its callbacks.
        """

    def log_info(self, info: str) -> None:
        """Appends information to the logging string.

        Args:
            info: Information to log.
        """
        self._logging_info += str(info) + "\n"

    def _physics_cleanup(self) -> None:
        """Cleanup physics resources."""
        if app_utils.is_playing():
            app_utils.stop()
        self.physics_cleanup()

    def physics_cleanup(self) -> None:
        """Function called when extension shutdowns and starts again, (hot reloading feature)."""

    async def clear_async(self) -> None:
        """Function called when clicking clear button."""
        await stage_utils.create_new_stage_async()
        self._physics_cleanup()
        gc.collect()
        await self.setup_post_clear()
