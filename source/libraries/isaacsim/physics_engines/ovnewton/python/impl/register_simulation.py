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

"""Register Newton simulation callbacks with the physics registration API."""

from __future__ import annotations

from isaacsim.common.logging import Logger
from isaacsim.physics.registration import (
    Simulation,
    SimulationId,
    k_invalid_simulation_id,
    register_simulation,
    unregister_simulation,
)

from .newton_config import NewtonConfig
from .newton_stage import NewtonStage
from .simulation_functions import NewtonSimulationFunctions
from .tensors.entity_factories import DEFAULT_SIMULATION_NAME

_log = Logger("isaacsim.physics_engines.ovnewton.impl")


class NewtonSimulationRegistry:
    """Manage a Newton stage and its registered simulation callbacks.

    Args:
        config: Newton backend configuration. Defaults are used when omitted.
        simulation_name: Name used to register the simulation.

    """

    def __init__(self, config: NewtonConfig | None = None, simulation_name: str | None = None) -> None:
        self.stage = NewtonStage(config)
        # Naming a simulation is what lets several coexist: entity factories resolve their stage by an exact
        # name match, so distinct names keep them apart. The default is the name the tensor factories are
        # registered under at import, so the two agree without relying on a case-insensitive match.
        self.simulation_name = simulation_name or DEFAULT_SIMULATION_NAME
        # Set when this registry registered the tensor factories itself. Teardown keys on it rather than on
        # the name, so a simulation never removes registrations another owner made.
        self.owns_tensor_factories = False
        self.sim_fns = NewtonSimulationFunctions(self.stage)
        self.simulation: Simulation | None = None
        self.simulation_id: SimulationId | None = None

    def register(self) -> SimulationId:
        """Register the Newton simulation callbacks.

        Returns:
            The assigned simulation identifier.

        Raises:
            RuntimeError: If the registration API rejects the simulation.

        """
        self.simulation = Simulation()

        # SimulationFns
        s = self.simulation.simulation_fns
        s.initialize = self.sim_fns.initialize
        s.close = self.sim_fns.close
        s.get_attached_stage = self.sim_fns.get_attached_stage
        s.has_attached_stage = self.sim_fns.has_attached_stage
        s.simulate_async = self.sim_fns.simulate_async
        s.simulate = self.sim_fns.simulate
        s.fetch_results = self.sim_fns.fetch_results
        s.check_results = self.sim_fns.check_results
        s.flush_changes = self.sim_fns.flush_changes
        s.pause_change_tracking = self.sim_fns.pause_change_tracking
        s.is_change_tracking_paused = self.sim_fns.is_change_tracking_paused
        s.subscribe_physics_contact_report_events = self.sim_fns.subscribe_physics_contact_report_events
        s.unsubscribe_physics_contact_report_events = self.sim_fns.unsubscribe_physics_contact_report_events
        s.get_simulation_time_steps_per_second = self.sim_fns.get_simulation_time_steps_per_second
        s.get_simulation_timestamp = self.sim_fns.get_simulation_timestamp
        s.get_simulation_step_count = self.sim_fns.get_simulation_step_count
        s.subscribe_physics_on_step_events = self.sim_fns.subscribe_physics_on_step_events
        s.unsubscribe_physics_on_step_events = self.sim_fns.unsubscribe_physics_on_step_events
        s.is_capable_of_simulating = self.sim_fns.is_capable_of_simulating

        sim_id = register_simulation(self.simulation, self.simulation_name)

        if sim_id == k_invalid_simulation_id:
            raise RuntimeError("Newton: register_simulation returned k_invalid_simulation_id")

        self.simulation_id = sim_id
        self.sim_fns.simulation_id = sim_id
        # Tell the stage its own SimulationId so `simulate_async` can
        # populate `PhysicsStepContext.simulation_id` when invoking
        # pre/post step subscribers — the only way a consumer
        # subscribed across multiple engines can tell whose callback
        # is firing.
        try:
            self.stage._simulation_id = int(sim_id)
        except (TypeError, AttributeError):  # pragma: no cover - defensive
            self.stage._simulation_id = 0
        _log.info(f"Registered Newton backend (id={sim_id})")
        return sim_id

    def unregister(self) -> None:
        """Unregister the simulation if it is registered."""
        if self.simulation_id is None:
            return
        unregister_simulation(self.simulation_id)
        _log.info(f"Unregistered Newton backend (id={self.simulation_id})")
        self.simulation_id = None
        self.simulation = None


# Module-level convenience handles -------------------------------------------

# Global registry keyed by the SimulationId's integer payload. The Newton entity
# factories look up the engine's stage by this int; callers that hold a
# `SimulationId` object can pass either the object or `sim_id.id`
# interchangeably (see `_resolve_key`).
_active: dict[int, NewtonSimulationRegistry] = {}


def _resolve_key(simulation_id: SimulationId | int) -> int:
    """Convert a simulation identifier to its integer registry key.

    Args:
        simulation_id: Simulation identifier object or integer payload.

    Returns:
        Integer payload used as the active-registry key.

    """
    if hasattr(simulation_id, "id"):
        return int(simulation_id.id)
    return int(simulation_id)


def register(config: NewtonConfig | None = None, simulation_name: str | None = None) -> SimulationId:
    """Create and register a Newton simulation.

    Args:
        config: Newton backend configuration.
        simulation_name: Name to register the simulation and its tensor factories under. Pass a distinct name
            per simulation to run several at once; omit it for a single simulation under the default name.

    Returns:
        The assigned simulation identifier. Pass it to :func:`unregister`.

    Raises:
        RuntimeError: If the physics registration API rejects the simulation.

    """
    reg = NewtonSimulationRegistry(config, simulation_name)
    sim_id = reg.register()
    _active[_resolve_key(sim_id)] = reg
    from .tensors import register_with_umbrella

    reg.owns_tensor_factories = register_with_umbrella(reg.simulation_name)
    return sim_id


def unregister(simulation_id: SimulationId | int) -> None:
    """Unregister a Newton simulation.

    Args:
        simulation_id: Registered simulation identifier or its integer payload.

    """
    reg = _active.pop(_resolve_key(simulation_id), None)
    if reg is None:
        return
    if reg.owns_tensor_factories:
        from .tensors import unregister_from_umbrella

        unregister_from_umbrella(reg.simulation_name)
    reg.unregister()


def get_registry(simulation_id: SimulationId | int) -> NewtonSimulationRegistry | None:
    """Look up an active Newton registry by simulation identifier.

    Args:
        simulation_id: Registered simulation identifier or its integer payload.

    Returns:
        Active registry for the identifier, or None when the simulation is not registered.

    """
    return _active.get(_resolve_key(simulation_id))
