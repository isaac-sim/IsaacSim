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

"""Newton simulation functions conforming to omni.physics.core.SimulationFns interface."""

from __future__ import annotations

from collections.abc import Callable
from typing import TYPE_CHECKING

import carb
from pxr import Plug, Usd

if TYPE_CHECKING:
    from .newton_stage import NewtonStage


class NewtonSimulationFunctions:
    """Implementation of simulation functions for Newton physics backend.

    This class provides all the required methods for the SimulationFns interface
    defined in omni.physics.core, allowing Newton to work with the unified physics
    simulation interface.

    Args:
        newton_stage: NewtonStage instance that manages the Newton simulation.
    """

    def __init__(self, newton_stage: NewtonStage) -> None:
        self.newton_stage = newton_stage
        self.contact_callbacks: dict[int, Callable] = {}
        self._next_contact_cb_id = 0
        self.step_callbacks: dict[int, tuple[bool, int, Callable]] = {}
        self._next_step_cb_id = 0
        self.change_tracking_paused = False
        self.simulation_id = 0  # Will be set after registration with physics interface

    def initialize(self, stage_id: int) -> bool:
        """Initialize physics simulation with a USD stage.

        This will run the physics parser and populate the simulation with
        the corresponding simulation objects. Previous stage will be closed.

        Args:
            stage_id: USD stageId (can be retrieved from a stagePtr).

        Returns:
            True if stage was successfully initialized.
        """
        try:
            self.newton_stage.stage_id = stage_id  # type: ignore[assignment]
            self.newton_stage.on_attach(stage_id, meters_per_unit=1.0)
            return True
        except Exception as e:
            carb.log_error(f"[Newton] Failed to initialize stage: {e}")
            return False

    def close(self) -> None:
        """Close the simulation, removing all objects from the simulation."""
        try:
            self.newton_stage.on_detach()
            self.newton_stage.stage_id = None
        except Exception as e:
            carb.log_error(f"[Newton] Failed to close simulation: {e}")

    def get_attached_stage(self) -> int:
        """Get the currently attached USD stage.

        Returns:
            USD stageId, 0 means no stage is attached.
        """
        return (
            self.newton_stage.stage_id if hasattr(self.newton_stage, "stage_id") and self.newton_stage.stage_id else 0
        )

    def simulate(self, elapsed_time: float, current_time: float) -> None:
        """Execute physics simulation.

        The simulation will simulate the exact elapsedTime passed. No substepping will happen.
        It is the caller's responsibility to provide reasonable elapsedTime.
        In general it is recommended to use fixed size time steps with a maximum of 1/60 of a second.

        Args:
            elapsed_time: Simulation time in seconds.
            current_time: Current time, might be used for time sampled transformations to apply.
        """
        try:
            # Import here to avoid circular dependency
            from omni.physics.core import PhysicsStepContext

            # Call pre-step callbacks
            context = PhysicsStepContext()
            context.scene_path = self.get_attached_stage()
            context.simulation_id = self.simulation_id

            sorted_callbacks = sorted(
                self.step_callbacks.values(),
                key=lambda x: x[1],
            )

            # Call pre-step callbacks (sorted by order)
            for pre_step, order, callback in sorted_callbacks:
                if pre_step:
                    try:
                        callback(elapsed_time, context)
                    except Exception as e:
                        carb.log_error(f"[Newton] Pre-step callback error: {e}")

            # Step Newton simulation
            self.newton_stage.step_sim(elapsed_time)

            # Call post-step callbacks (sorted by order)
            for pre_step, order, callback in sorted_callbacks:
                if not pre_step:
                    try:
                        callback(elapsed_time, context)
                    except Exception as e:
                        carb.log_error(f"[Newton] Post-step callback error: {e}")
        except Exception as e:
            carb.log_error(f"[Newton] Simulation step error: {e}")

    def start_simulation(self) -> None:
        """Start simulation.

        This method is called at the beginning of simulation to allow the physics engine
        to store initial transformations. It does NOT take a physics step - stepping is
        handled by simulate() calls.
        """
        # PhysX uses this to store initial transformations for reset
        # Newton doesn't need to do anything here - initialization happens lazily in step_sim

    def fetch_results(self) -> None:
        """Fetch simulation results.

        Writing out simulation results based on physics settings.
        This is a blocking call that waits until the simulation is finished.
        """
        try:
            from omni.physics.core import ContactDataVector, ContactEventHeaderVector, FrictionAnchorsDataVector

            # Create empty contact data structures
            headers = ContactEventHeaderVector()
            contacts = ContactDataVector()
            friction = FrictionAnchorsDataVector()

            for callback in self.contact_callbacks.values():
                try:
                    callback(headers, contacts, friction)
                except Exception as e:
                    carb.log_error(f"[Newton] Contact callback error: {e}")
        except Exception as e:
            carb.log_error(f"[Newton] Fetch results error: {e}")

    def check_results(self) -> bool:
        """Check if simulation finished.

        Returns:
            True if simulation finished.
        """
        return self.newton_stage.initialized

    def flush_changes(self) -> None:
        """Flush changes to force physics to process buffered changes.

        Changes to physics get buffered. In some cases flushing changes is required
        if ordering is important.
        """
        # Newton initializes synchronously, so this is essentially a no-op
        # but we ensure initialization is complete
        if not self.newton_stage.initialized:
            carb.log_warn("[Newton] flush_changes called but Newton not initialized yet")

    def pause_change_tracking(self, pause: bool) -> None:
        """Pause change tracking for physics listener.

        Args:
            pause: Pause or resume the change tracking.
        """
        self.change_tracking_paused = pause

    def is_change_tracking_paused(self) -> bool:
        """Check if fabric change tracking for physics listener is paused.

        Returns:
            True if change tracking is paused.
        """
        return self.change_tracking_paused

    def subscribe_physics_contact_report_events(self, on_event: Callable) -> int:
        """Subscribe to physics simulation contact report events.

        The contact buffer data are available for one simulation step.

        Args:
            on_event: The callback function to be called on contact report.

        Returns:
            Subscription Id for release.
        """
        self._next_contact_cb_id += 1
        self.contact_callbacks[self._next_contact_cb_id] = on_event
        return self._next_contact_cb_id

    def unsubscribe_physics_contact_report_events(self, subscription_id: int) -> None:
        """Unsubscribe from physics contact report events.

        Args:
            subscription_id: Subscription ID returned from subscribe.
        """
        self.contact_callbacks.pop(subscription_id, None)

    def get_simulation_time_steps_per_second(self, stage_id: int, scene_path: int) -> int:
        """Get physics simulation time steps per second.

        Args:
            stage_id: Stage id.
            scene_path: Returns the time steps for given scene. If 0 is passed,
                returns the first found scene stepping.

        Returns:
            Current time steps per second.
        """
        if hasattr(self.newton_stage, "physics_frequency"):
            return int(self.newton_stage.physics_frequency)  # type: ignore[has-type]
        return (
            int(self.newton_stage.cfg.physics_frequency) if hasattr(self.newton_stage.cfg, "physics_frequency") else 60
        )

    def get_simulation_timestamp(self) -> int:
        """Get physics simulation timestamp.

        Timestamp will increase with every simulation step.

        Returns:
            Current timestamp.
        """
        return getattr(self.newton_stage, "simulation_timestamp", 0)

    def get_simulation_step_count(self) -> int:
        """Get the number of physics steps performed in the active simulation.

        The step count resets to 0 when a new simulation starts.

        Returns:
            Number of steps since the currently active simulation started,
            or 0 if there is no active simulation.
        """
        return getattr(self.newton_stage, "simulation_step_count", 0)

    def subscribe_physics_on_step_events(self, pre_step: bool, order: int, on_update: Callable) -> int:
        """Subscribe to physics pre/post step events.

        Subscriptions cannot be changed in the onUpdate callback.

        Args:
            pre_step: Whether to execute this callback right before the physics step event.
                If False, the callback will be executed right after the physics step event.
            order: An integer value used to order the callbacks. 0 means highest priority,
                1 is less priority, and so on.
            on_update: The callback function to be called on update.

        Returns:
            Subscription Id for release, returns kInvalidSubscriptionId if failed.
        """
        self._next_step_cb_id += 1
        self.step_callbacks[self._next_step_cb_id] = (pre_step, order, on_update)
        return self._next_step_cb_id

    def unsubscribe_physics_on_step_events(self, subscription_id: int) -> None:
        """Unsubscribe from physics step events.

        Args:
            subscription_id: Subscription ID returned from subscribe.
        """
        self.step_callbacks.pop(subscription_id, None)

    def is_capable_of_simulating(self, schema_names: list[str]) -> tuple[bool, list[bool]]:
        """Check if simulation is capable of simulating given schema types.

        Args:
            schema_names: List of schema names to check

        Returns:
            A tuple of (success, capabilities) where success is True if the operation was successful
                   and capabilities is a list of booleans indicating support for each schema.
        """
        capabilities = []
        for schema_name in schema_names:
            is_capable = False
            # Find the TfType for this schema name
            tf_type = Usd.SchemaRegistry.GetAPITypeFromSchemaTypeName(schema_name)
            if tf_type.isUnknown:
                tf_type = Usd.SchemaRegistry.GetConcreteTypeFromSchemaTypeName(schema_name)

            if not tf_type.isUnknown:
                # Get the plugin that declares this type
                plugin = Plug.Registry().GetPluginForType(tf_type)
                if plugin:
                    plugin_name = plugin.name
                    # Set isCapable to true if the schema is from known plugins
                    is_capable = plugin_name == "newton" or plugin_name == "mjcPhysics" or plugin_name == "usdPhysics"
            capabilities.append(is_capable)

        return (True, capabilities)
