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


"""Provide utils functionality."""

from typing import Any

import isaacsim.physics.registration as registration


class MockSimulator:
    """Mock physics simulation functionality."""

    def __init__(self) -> None:
        self._simulation_id = None
        self._physics_subscription_id = 0
        self._physics_subscriptions = {"pre": {}, "post": {}}

    def check_results(self) -> bool:
        """Check results.

        Returns:
            The resulting value.

        """
        return True

    def close(self) -> bool:
        """Handle close.

        Returns:
            The resulting value.

        """
        return True

    def fetch_results(self) -> None:
        """Handle fetch results."""

    def flush_changes(self) -> None:
        """Handle flush changes."""

    def get_attached_stage(self) -> int:
        """Get attached stage.

        Returns:
            The resulting value.

        """
        return 0

    def get_simulation_step_count(self) -> int:
        """Get simulation step count.

        Returns:
            The resulting value.

        """
        return 0

    def get_simulation_time_steps_per_second(self, stage_id: Any, scene_path: Any) -> int:
        """Get simulation time steps per second.

        Args:
            stage_id: Identifier of the USD stage.
            scene_path: Path to the physics scene.

        Returns:
            The resulting value.

        """
        return 0

    def get_simulation_timestamp(self) -> int:
        """Get simulation timestamp.

        Returns:
            The resulting value.

        """
        return 0

    def initialize(self, ovstage_instance: Any, usd_stage_id: Any) -> bool:
        """Initialize.

        Args:
            ovstage_instance: OVStage instance under test.
            usd_stage_id: Identifier of the USD stage.

        Returns:
            The resulting value.

        """
        return True

    def is_capable_of_simulating(self, schema_names: Any) -> tuple:
        """Handle is capable of simulating.

        Args:
            schema_names: Names of schemas expected on the prim.

        Returns:
            Whether capable of simulating.

        """
        return (True, [True] * len(schema_names))

    def is_change_tracking_paused(self) -> bool:
        """Handle is change tracking paused.

        Returns:
            Whether change tracking paused.

        """
        return False

    def pause_change_tracking(self, pause: Any) -> None:
        """Handle pause change tracking.

        Args:
            pause: Whether to pause the simulation.

        """

    def set_capability_check_enabled(self, enabled: Any) -> None:
        """Set capability check enabled.

        Args:
            enabled: Whether the feature is enabled.

        """

    def set_supported_capability(self, schema_name: Any, is_supported: Any) -> None:
        """Set supported capability.

        Args:
            schema_name: Name of the schema to inspect.
            is_supported: Whether the backend supports the operation.

        """

    def simulate(self, elapsed_time: Any, current_time: Any) -> None:
        """Handle simulate.

        Args:
            elapsed_time: Elapsed simulation time.
            current_time: Current simulation time.

        """
        self.simulate_async(elapsed_time, current_time)
        self.fetch_results()

    def simulate_async(self, elapsed_time: Any, current_time: Any) -> None:
        """Handle simulate async.

        Args:
            elapsed_time: Elapsed simulation time.
            current_time: Current simulation time.

        """
        context = registration.PhysicsStepContext()
        context.simulation_id = self._simulation_id
        # trigger pre-step callbacks
        callbacks = sorted(self._physics_subscriptions["pre"].values(), key=lambda x: x[0])
        for callback in callbacks:
            callback[1](elapsed_time, context)
        # trigger post-step callbacks
        callbacks = sorted(self._physics_subscriptions["post"].values(), key=lambda x: x[0])
        for callback in callbacks:
            callback[1](elapsed_time, context)

    def subscribe_physics_contact_report_events(self, on_event: Any) -> int:
        """Handle subscribe physics contact report events.

        Args:
            on_event: Callback invoked for a physics event.

        Returns:
            The resulting value.

        """
        return registration.k_invalid_subscription_id

    def subscribe_physics_on_step_events(self, pre_step: Any, order: Any, on_update: Any) -> Any:
        """Handle subscribe physics on step events.

        Args:
            pre_step: Whether to subscribe to pre-step events.
            order: Expected callback invocation order.
            on_update: Callback invoked for an update.

        Returns:
            The resulting value.

        """
        self._physics_subscription_id += 1
        subscription_type = "pre" if pre_step else "post"
        self._physics_subscriptions[subscription_type][self._physics_subscription_id] = (order, on_update)
        return self._physics_subscription_id

    def unsubscribe_physics_contact_report_events(self, subscription_id: Any) -> None:
        """Handle unsubscribe physics contact report events.

        Args:
            subscription_id: Identifier of the event subscription.

        """

    def unsubscribe_physics_on_step_events(self, subscription_id: Any) -> None:
        """Handle unsubscribe physics on step events.

        Args:
            subscription_id: Identifier of the event subscription.

        """
        for subscription_type in ["pre", "post"]:
            if subscription_id in self._physics_subscriptions[subscription_type]:
                del self._physics_subscriptions[subscription_type][subscription_id]
                break


def setup_simulation_fns(simulation: Any, simulator: Any) -> None:
    """Wire all simulation function pointers to the simulator.

    Args:
        simulation: Simulation registration to exercise.
        simulator: Mock simulator backing the registration.

    """
    simulation.simulation_fns.check_results = simulator.check_results
    simulation.simulation_fns.close = simulator.close
    simulation.simulation_fns.fetch_results = simulator.fetch_results
    simulation.simulation_fns.flush_changes = simulator.flush_changes
    simulation.simulation_fns.get_attached_stage = simulator.get_attached_stage
    simulation.simulation_fns.get_simulation_step_count = simulator.get_simulation_step_count
    simulation.simulation_fns.get_simulation_time_steps_per_second = simulator.get_simulation_time_steps_per_second
    simulation.simulation_fns.get_simulation_timestamp = simulator.get_simulation_timestamp
    simulation.simulation_fns.initialize = simulator.initialize
    simulation.simulation_fns.is_capable_of_simulating = simulator.is_capable_of_simulating
    simulation.simulation_fns.is_change_tracking_paused = simulator.is_change_tracking_paused
    simulation.simulation_fns.pause_change_tracking = simulator.pause_change_tracking
    simulation.simulation_fns.simulate = simulator.simulate
    simulation.simulation_fns.simulate_async = simulator.simulate_async
    simulation.simulation_fns.subscribe_physics_contact_report_events = (
        simulator.subscribe_physics_contact_report_events
    )
    simulation.simulation_fns.subscribe_physics_on_step_events = simulator.subscribe_physics_on_step_events
    simulation.simulation_fns.unsubscribe_physics_contact_report_events = (
        simulator.unsubscribe_physics_contact_report_events
    )
    simulation.simulation_fns.unsubscribe_physics_on_step_events = simulator.unsubscribe_physics_on_step_events
