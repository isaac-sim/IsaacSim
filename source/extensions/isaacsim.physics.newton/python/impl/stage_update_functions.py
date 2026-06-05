# SPDX-FileCopyrightText: Copyright (c) 2024-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

"""Newton stage update functions conforming to omni.physics.core.StageUpdateFns interface."""

from __future__ import annotations

from typing import TYPE_CHECKING

import carb

if TYPE_CHECKING:
    from .newton_stage import NewtonStage


class NewtonStageUpdateFunctions:
    """Implementation of stage update functions for Newton physics backend.

    This class provides all the required methods for the StageUpdateFns interface
    defined in omni.physics.core, allowing Newton to work with the unified physics
    stage update interface.

    Args:
        newton_stage: NewtonStage instance that manages the Newton simulation.
    """

    def __init__(self, newton_stage: NewtonStage) -> None:
        self.newton_stage = newton_stage
        self.is_physics_loaded = False
        self.is_paused_state = False
        self.update_count = 0

    def start_simulation(self) -> bool:
        """Called when simulation needs to start.

        This initializes the Newton simulation. Physics stepping is handled separately
        by simulate() calls.

        Returns:
            True if start was successful.
        """
        return True

    def on_attach(self, stage_id: int) -> bool:
        """Called when a stage gets attached.

        Does not load physics, just sets the stage internally.

        Args:
            stage_id: Stage Id that should be attached.

        Returns:
            True if attachment was successful.
        """
        try:
            self.newton_stage.stage_id = stage_id  # type: ignore[assignment]
            self.newton_stage.on_attach(stage_id, meters_per_unit=1.0)
            return True
        except Exception as e:
            carb.log_error(f"[Newton] on_attach failed: {e}")
            return False

    def on_detach(self) -> bool:
        """Called when stage gets detached.

        Returns:
            True if detachment was successful.
        """
        try:
            self.newton_stage.on_detach()
            self.is_physics_loaded = False
            self.is_paused_state = False
            return True
        except Exception as e:
            carb.log_error(f"[Newton] on_detach failed: {e}")
            return False

    def on_update(self, time: float, delta_time: float, physics_enabled: bool) -> bool:
        """Called on stage update.

        Args:
            time: Current time in seconds.
            delta_time: Elapsed time from previous update in seconds.
            physics_enabled: Enable physics update.

        Returns:
            True if update was successful.
        """
        try:
            if physics_enabled and not self.is_paused_state:
                self.newton_stage.on_update(None, delta_time)
                self.update_count += 1
            return True
        except Exception as e:
            carb.log_error(f"[Newton] on_update failed: {e}")
            return False

    def on_resume(self, time: float) -> bool:
        """Called when timeline play is requested.

        Args:
            time: Current time in seconds.

        Returns:
            True if resume was successful.
        """
        try:
            self.is_paused_state = False
            self.newton_stage.playing = True
            return True
        except Exception as e:
            carb.log_error(f"[Newton] on_resume failed: {e}")
            return False

    def on_pause(self) -> bool:
        """Called when timeline gets paused.

        Returns:
            True if pause was successful.
        """
        try:
            self.is_paused_state = True
            self.newton_stage.playing = False
            self.newton_stage.graph = None
            return True
        except Exception as e:
            carb.log_error(f"[Newton] on_pause failed: {e}")
            return False

    def on_reset(self) -> bool:
        """Called when timeline is stopped.

        Returns:
            True if reset was successful.
        """
        try:
            self.update_count = 0
            self.is_paused_state = False
            self.newton_stage.playing = False
            self.newton_stage.init()
            self.newton_stage.sim_time = 0.0  # type: ignore[assignment]
            return True
        except Exception as e:
            carb.log_error(f"[Newton] on_reset failed: {e}")
            return False

    def force_load_physics_from_usd(self) -> bool:
        """Called when a force load from USD is requested.

        This marks the physics as needing initialization. The actual initialization
        happens lazily in step_sim() to avoid repeated expensive initializations
        during rapid stage transitions.

        Returns:
            True if the request was accepted.
        """
        try:
            self.newton_stage.initialized = False
            self.is_physics_loaded = False
            return True
        except Exception as e:
            carb.log_error(f"[Newton] force_load_physics_from_usd failed: {e}")
            return False

    def release_physics_objects(self) -> bool:
        """Called when a release of physics objects is requested.

        Returns:
            True if release was successful.
        """
        try:
            self.newton_stage.init()
            self.is_physics_loaded = False
            return True
        except Exception as e:
            carb.log_error(f"[Newton] release_physics_objects failed: {e}")
            return False

    def handle_raycast(self, origin: carb.Float3 | None, direction: carb.Float3 | None, has_input: bool) -> bool:
        """Called when a raycast request is executed.

        Args:
            origin: Start position of the raycast.
            direction: Direction of the raycast.
            has_input: Whether the input control is set or reset.

        Returns:
            True if raycast was handled.
        """
        return True

    def reset_simulation(self) -> bool:
        """Called when a reset of physics simulation is requested.

        This will release all physics objects and reset the simulation,
        while keeping the stage attached.

        Returns:
            True if reset was successful.
        """
        try:
            self.release_physics_objects()
            self.on_reset()
            return True
        except Exception as e:
            carb.log_error(f"[Newton] reset_simulation failed: {e}")
            return False
