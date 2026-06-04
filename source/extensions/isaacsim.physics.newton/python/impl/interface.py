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

"""Newton physics interface for simulation control."""

from __future__ import annotations

from collections.abc import Callable
from typing import TYPE_CHECKING

import carb
import omni.timeline

if TYPE_CHECKING:
    from .newton_stage import NewtonStage


class NewtonPhysicsInterface:
    """Interface for controlling Newton physics simulation.

    Provides methods for simulation control, stepping, and event subscription.

    Args:
        newtonStage: NewtonStage instance that manages the simulation.
    """

    def __init__(self, newtonStage: NewtonStage) -> None:
        self.newtonStage = newtonStage
        self.timeline = omni.timeline.get_timeline_interface()

    @carb.profiler.profile
    def force_load_physics_from_usd(self, device: str | None = None) -> None:
        """Force load physics from USD.

        Args:
            device: Device to use for simulation.
        """
        self.newtonStage.initialize_newton(device)

    def flush_changes(self) -> None:
        """Flush any pending changes to ensure Newton is fully initialized.

        This is called before creating simulation views to ensure Newton's state
        is synchronized, similar to PhysX's flush_changes().
        """
        if not self.newtonStage.initialized:
            carb.log_warn("[Newton] flush_changes called but Newton not initialized yet")

    @carb.profiler.profile
    def start_simulation(self) -> None:
        """Start the physics simulation."""
        self.update_simulation(self.newtonStage.sim_dt, 0.0)

    @carb.profiler.profile
    def update_simulation(self, elapsedStep: float, currentTime: float) -> None:
        """Update the simulation by stepping physics.

        Args:
            elapsedStep: Time elapsed since last update.
            currentTime: Current simulation time.
        """
        self.newtonStage.step_sim(self.newtonStage.sim_dt)

    @carb.profiler.profile
    def update_transformations(
        self,
        updateToFastCache: bool = True,
        updateToUsd: bool = False,
        updateVelocitiesToUsd: bool = True,
        outputVelocitiesLocalSpace: bool = False,
    ) -> None:
        """Update transformations from simulation to USD/Fabric.

        Args:
            updateToFastCache: Update to fast cache.
            updateToUsd: Update to USD.
            updateVelocitiesToUsd: Update velocities to USD.
            outputVelocitiesLocalSpace: Output velocities in local space.
        """

    @carb.profiler.profile
    def update(self, elapsedStep: float, currentTime: float) -> None:
        """Update callback for stage update events.

        Args:
            elapsedStep: Time elapsed since last update.
            currentTime: Current simulation time.
        """

    @carb.profiler.profile
    def fetch_results(self) -> None:
        """Fetch simulation results after stepping."""
        return

    def subscribe_physics_step_events(self, callback: Callable) -> None:
        """Subscribe to physics step events.

        Args:
            callback: Callback function to be called on each physics step.
        """
        self.newtonStage.physics_callbacks.append(callback)

    def unsubscribe_physics_step_events(self, callback: Callable) -> None:
        """Unsubscribe from physics step events.

        Args:
            callback: Callback function to remove.
        """
        self.newtonStage.physics_callbacks.remove(callback)

    def simulate(self, dt: float, currentTime: float) -> None:
        """Step the simulation by the given time delta.

        Args:
            dt: Time delta for this step.
            currentTime: Current simulation time.
        """
        self.newtonStage.step_sim(dt)
