# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

from __future__ import annotations

import isaacsim.core.experimental.utils.stage as stage_utils
from isaacsim.core.simulation_manager import IsaacEvents, SimulationManager
from isaacsim.examples.base.base_sample_experimental import BaseSample
from isaacsim.storage.native import get_assets_root_path


class ExampleScenario(BaseSample):
    """Simulation scenario for {{title}}.

    Implements the scene lifecycle: setup, load, reset, and clear.
    Add your simulation assets and physics logic here.
    """

    def __init__(self) -> None:
        super().__init__()
        self._physics_callback_id = None

    def setup_scene(self) -> None:
        """Set up the scene with a ground plane and any initial assets.

        This is called when the user clicks Load World. Add USD references,
        create prims, and configure the environment here.
        """
        stage_utils.add_reference_to_stage(
            usd_path=get_assets_root_path() + "/Isaac/Environments/Grid/default_environment.usd",
            path="/World/ground",
        )

    async def setup_post_load(self):
        """Called after the world is loaded and simulation is playing.

        Register physics callbacks, initialize controllers, or set up
        any state that depends on the simulation being active.
        """
        self._physics_callback_id = SimulationManager.register_callback(
            self.on_physics_step, event=IsaacEvents.POST_PHYSICS_STEP
        )

    def on_physics_step(self, step_size: float, context: object) -> None:
        """Called on every physics step while the simulation is running.

        Args:
            step_size: The physics simulation step size in seconds.
            context: The physics simulation context.
        """
        pass

    async def setup_pre_reset(self):
        """Called before the world is reset. Clean up callbacks here."""
        if self._physics_callback_id is not None:
            SimulationManager.deregister_callback(self._physics_callback_id)
            self._physics_callback_id = None

    async def setup_post_reset(self):
        """Called after the world is reset. Re-register callbacks here."""
        self._physics_callback_id = SimulationManager.register_callback(
            self.on_physics_step, event=IsaacEvents.POST_PHYSICS_STEP
        )

    async def setup_post_clear(self):
        """Called after the stage is cleared."""
        if self._physics_callback_id is not None:
            SimulationManager.deregister_callback(self._physics_callback_id)
            self._physics_callback_id = None

    def physics_cleanup(self) -> None:
        """Called on extension shutdown or hot-reload."""
        if self._physics_callback_id is not None:
            SimulationManager.deregister_callback(self._physics_callback_id)
            self._physics_callback_id = None
