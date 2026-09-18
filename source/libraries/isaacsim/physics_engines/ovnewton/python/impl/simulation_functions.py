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

"""Implement Newton callbacks for ``isaacsim.physics.registration.Simulation``.

Each method matches a simulation-function slot and forwards the request to the
attached :class:`NewtonStage`.
"""

from __future__ import annotations

from collections.abc import Callable
from typing import Any

from isaacsim.common.logging import Logger
from isaacsim.physics.registration import SimulationId

from .newton_stage import NewtonStage

_log = Logger("isaacsim.physics_engines.ovnewton.impl")


class NewtonSimulationFunctions:
    """Forward registered simulation callbacks to a Newton stage.

    Args:
        stage: Newton stage that owns simulation state.

    """

    def __init__(self, stage: NewtonStage) -> None:
        self._stage = stage
        # Filled in after the registration API assigns an identifier.
        self.simulation_id: SimulationId | int = 0

    # SimulationFns slots ------------------------------------------------

    def initialize(self, ovstage: Any | None, usd_identifier: str | None) -> bool:
        """Initialize Newton from a caller-owned ovstage.

        Dual-input contract matching ovphysx: the native ``ovstage`` handle is the
        physics source. ``usd_identifier`` is only republished through
        :meth:`get_attached_stage`. The Python Stage must previously have been
        registered via :func:`isaacsim.physics_engines.ovstage.get_native_handle`.

        Args:
            ovstage: Native address of a caller-owned ovstage instance, or 0/None for an empty scene.
            usd_identifier: Numeric stage identifier string, or None when no identifier is supplied.

        Returns:
            True when the Newton stage accepts the initialization, otherwise False.

        """
        # `as_native_handle` / `lookup_stage` are pure Python and need no runtime
        # discovery; `NewtonStage.initialize` calls `ovstage.setup()` itself before
        # importing ovnewton, where a missing bundled runtime degrades gracefully.
        from isaacsim.physics_engines.ovstage import as_native_handle, lookup_stage

        try:
            stage_id = int(usd_identifier) if usd_identifier else 0
        except (TypeError, ValueError):
            _log.error(f"Newton initialize rejected a non-numeric usd_identifier: {usd_identifier!r}")
            return False

        try:
            handle = as_native_handle(ovstage)
        except TypeError as exc:
            _log.error(f"Newton initialize rejected an unusable ovstage handle: {exc}")
            return False

        ovstage_obj = None
        if handle:
            ovstage_obj = lookup_stage(handle)
            if ovstage_obj is None:
                _log.error(
                    f"ovstage handle {handle:#x} is not registered; the owning Python Stage must be passed "
                    "through isaacsim.physics_engines.ovstage.get_native_handle() and kept alive."
                )
                return False
        return self._stage.initialize(ovstage_obj, stage_id)

    def close(self) -> bool:
        """Detach the Newton simulation.

        Returns:
            Always True because the ovstage remains caller-owned.

        """
        self._stage.close()
        return True

    def get_attached_stage(self) -> int:
        """Get the attached USD stage identifier.

        Returns:
            Attached stage identifier, or zero when no stage is attached.

        """
        return self._stage.get_attached_stage()

    def has_attached_stage(self) -> bool:
        """Report whether Newton still borrows a caller-owned ovstage.

        Returns:
            True when a borrowed ovstage is retained (including after a failed
            model build that left ``_initialized`` false for retry), otherwise False.

        """
        return self._stage.has_attached_stage()

    def simulate_async(self, elapsed_time: float, current_time: float) -> None:
        """Advance a simulation step through the asynchronous callback slot.

        Args:
            elapsed_time: Duration of the simulation step in seconds.
            current_time: Current simulation time supplied by the manager.

        """
        self._stage.simulate_async(elapsed_time, current_time)

    def simulate(self, elapsed_time: float, current_time: float) -> None:
        """Advance a synchronous simulation step.

        Args:
            elapsed_time: Duration of the simulation step in seconds.
            current_time: Current simulation time supplied by the manager.

        """
        self._stage.simulate(elapsed_time, current_time)

    def fetch_results(self) -> None:
        """Dispatch the simulation callback's compatibility contact report."""
        self._stage.fetch_results()

    def check_results(self) -> bool:
        """Check whether synchronous results are ready.

        Returns:
            Always True because Newton completes the step synchronously.

        """
        return self._stage.check_results()

    def flush_changes(self) -> None:
        """Forward a request to flush pending simulation changes."""
        self._stage.flush_changes()

    def pause_change_tracking(self, pause: bool) -> None:
        """Set whether change tracking is paused.

        Args:
            pause: True to pause change tracking, or False to resume it.

        """
        self._stage.pause_change_tracking(pause)

    def is_change_tracking_paused(self) -> bool:
        """Check whether change tracking is paused.

        Returns:
            True when the Newton stage has paused change tracking.

        """
        return self._stage.is_change_tracking_paused()

    # Contact-report subscriptions --------------------------------------

    def subscribe_physics_contact_report_events(self, callback: Callable[[Any, Any, Any], Any]) -> int:
        """Subscribe to physics contact-report events.

        Args:
            callback: Function invoked with contact headers, contact data, and friction-anchor data.

        Returns:
            Identifier used to remove the subscription.

        """
        return self._stage.subscribe_contact_report(callback)

    def unsubscribe_physics_contact_report_events(self, subscription_id: int) -> None:
        """Remove a physics contact-report subscription.

        Args:
            subscription_id: Identifier returned when the callback was subscribed.

        """
        self._stage.unsubscribe_contact_report(subscription_id)

    # Time / step queries ------------------------------------------------

    def get_simulation_time_steps_per_second(self, stage_id: int, scene_path: int) -> int:
        """Get the configured simulation frequency.

        Args:
            stage_id: Stage identifier supplied by the manager.
            scene_path: Encoded physics-scene path supplied by the manager.

        Returns:
            Configured number of simulation steps per second.

        """
        return int(self._stage.config.default_steps_per_second)

    def get_simulation_timestamp(self) -> int:
        """Get the current simulation timestamp.

        Returns:
            Timestamp incremented after each initialized step callback that does not encounter a solver error.

        """
        return self._stage.timestamp

    def get_simulation_step_count(self) -> int:
        """Get the completed simulation-step count.

        Returns:
            Number of initialized step callbacks that did not encounter a solver error since attachment or reset.

        """
        return self._stage.step_count

    # Step subscriptions -------------------------------------------------

    def subscribe_physics_on_step_events(
        self, pre_step: bool, order: int, callback: Callable[[float, Any], Any]
    ) -> int:
        """Subscribe to physics-step events.

        Args:
            pre_step: True to receive the event before the solver step, or False to receive it afterward.
            order: Ordering value within the selected event phase.
            callback: Function invoked with the step duration and physics-step context.

        Returns:
            Identifier used to remove the subscription.

        """
        return self._stage.subscribe_step_event(pre_step, order, callback)

    def unsubscribe_physics_on_step_events(self, subscription_id: int) -> None:
        """Remove a physics-step subscription.

        Args:
            subscription_id: Identifier returned when the callback was subscribed.

        """
        self._stage.unsubscribe_step_event(subscription_id)

    # Capability check ---------------------------------------------------

    def is_capable_of_simulating(self, schema_names: list[str]) -> tuple[bool, list[bool]]:
        """Check support for requested USD physics schemas.

        Args:
            schema_names: Schema type names to query.

        Returns:
            A successful-query flag and one support flag per requested schema.

        """
        return self._stage.is_capable_of_simulating(schema_names)
