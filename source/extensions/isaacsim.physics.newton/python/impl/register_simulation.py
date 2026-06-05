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

"""Newton simulation registration with unified physics interface."""

from __future__ import annotations

from typing import TYPE_CHECKING, Any

import carb

from .simulation_functions import NewtonSimulationFunctions
from .stage_update_functions import NewtonStageUpdateFunctions

if TYPE_CHECKING:
    from .newton_stage import NewtonStage


class NewtonSimulationRegistry:
    """Registry for Newton physics simulation with unified physics interface.

    This class handles creating a Simulation object with Newton-specific implementations
    and registering it with the omni.physics.core interface, allowing Newton to work
    seamlessly alongside PhysX using the same unified API.
    """

    def __init__(self) -> None:
        self.simulation_id: int | None = None
        self.newton_stage: NewtonStage | None = None
        self.simulation: Any = None
        self.sim_fns: Any = None
        self.stage_update_fns: Any = None

    def register_newton(self, newton_stage: NewtonStage) -> int | None:
        """Register Newton as a physics simulation backend.

        This creates wrapper classes for Newton that implement the required
        simulation_fns and stage_update_fns interfaces, creates a Simulation
        object, assigns all the function pointers, and registers it with the
        physics interface.

        Args:
            newton_stage: NewtonStage instance that manages the Newton simulation.

        Returns:
            Simulation ID assigned by the physics interface, or None if registration failed.
        """
        try:
            from omni.physics.core import Simulation, get_physics_interface, k_invalid_simulation_id

            self.newton_stage = newton_stage

            # Create simulation functions wrapper
            self.sim_fns = NewtonSimulationFunctions(newton_stage)
            self.stage_update_fns = NewtonStageUpdateFunctions(newton_stage)

            # Store reference in newton_stage so it can call the callbacks
            newton_stage.simulation_functions = self.sim_fns  # type: ignore[attr-defined]

            # Create Simulation object
            self.simulation = Simulation()

            # Assign all simulation_fns methods
            self.simulation.simulation_fns.initialize = self.sim_fns.initialize
            self.simulation.simulation_fns.close = self.sim_fns.close
            self.simulation.simulation_fns.get_attached_stage = self.sim_fns.get_attached_stage
            self.simulation.simulation_fns.simulate = self.sim_fns.simulate
            self.simulation.simulation_fns.fetch_results = self.sim_fns.fetch_results
            self.simulation.simulation_fns.check_results = self.sim_fns.check_results
            self.simulation.simulation_fns.flush_changes = self.sim_fns.flush_changes
            self.simulation.simulation_fns.pause_change_tracking = self.sim_fns.pause_change_tracking
            self.simulation.simulation_fns.is_change_tracking_paused = self.sim_fns.is_change_tracking_paused
            self.simulation.simulation_fns.subscribe_physics_contact_report_events = (
                self.sim_fns.subscribe_physics_contact_report_events
            )
            self.simulation.simulation_fns.unsubscribe_physics_contact_report_events = (
                self.sim_fns.unsubscribe_physics_contact_report_events
            )
            self.simulation.simulation_fns.get_simulation_time_steps_per_second = (
                self.sim_fns.get_simulation_time_steps_per_second
            )
            self.simulation.simulation_fns.get_simulation_timestamp = self.sim_fns.get_simulation_timestamp
            self.simulation.simulation_fns.get_simulation_step_count = self.sim_fns.get_simulation_step_count
            self.simulation.simulation_fns.subscribe_physics_on_step_events = (
                self.sim_fns.subscribe_physics_on_step_events
            )
            self.simulation.simulation_fns.unsubscribe_physics_on_step_events = (
                self.sim_fns.unsubscribe_physics_on_step_events
            )
            self.simulation.simulation_fns.is_capable_of_simulating = self.sim_fns.is_capable_of_simulating

            # Assign all stage_update_fns methods
            self.simulation.stage_update_fns.start_simulation = self.stage_update_fns.start_simulation
            self.simulation.stage_update_fns.on_attach = self.stage_update_fns.on_attach
            self.simulation.stage_update_fns.on_detach = self.stage_update_fns.on_detach
            self.simulation.stage_update_fns.on_update = self.stage_update_fns.on_update
            self.simulation.stage_update_fns.on_resume = self.stage_update_fns.on_resume
            self.simulation.stage_update_fns.on_pause = self.stage_update_fns.on_pause
            self.simulation.stage_update_fns.on_reset = self.stage_update_fns.on_reset
            self.simulation.stage_update_fns.force_load_physics_from_usd = (
                self.stage_update_fns.force_load_physics_from_usd
            )
            self.simulation.stage_update_fns.release_physics_objects = self.stage_update_fns.release_physics_objects
            self.simulation.stage_update_fns.handle_raycast = self.stage_update_fns.handle_raycast
            self.simulation.stage_update_fns.reset_simulation = self.stage_update_fns.reset_simulation

            # Register with physics interface
            physics = get_physics_interface()
            self.simulation_id = physics.register_simulation(self.simulation, "Newton")

            if self.simulation_id == k_invalid_simulation_id:
                carb.log_error("[newton] Failed to register simulation with physics interface")
                return None

            # Store simulation_id for context callbacks
            self.sim_fns.simulation_id = self.simulation_id

            carb.log_info(f"[Newton] Successfully registered with physics interface (ID: {self.simulation_id})")
            return self.simulation_id

        except Exception as e:
            carb.log_error(f"[Newton] Failed to register simulation: {e}")
            import traceback

            traceback.print_exc()
            return None

    def unregister_newton(self) -> None:
        """Unregister Newton simulation from physics interface."""
        if self.simulation_id is not None:
            try:
                from omni.physics.core import get_physics_interface

                physics = get_physics_interface()
                physics.unregister_simulation(self.simulation_id)
                carb.log_info(f"[Newton] Unregistered simulation (ID: {self.simulation_id})")
                self.simulation_id = None
            except Exception as e:
                carb.log_error(f"[Newton] Failed to unregister simulation: {e}")

    def is_registered(self) -> bool:
        """Check if Newton is currently registered.

        Returns:
            True if Newton is registered with a valid simulation ID.
        """
        return self.simulation_id is not None

    def get_simulation_id(self) -> int | None:
        """Get the simulation ID assigned by the physics interface.

        Returns:
            Simulation ID, or None if not registered.
        """
        return self.simulation_id
