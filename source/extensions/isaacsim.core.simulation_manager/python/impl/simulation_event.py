# SPDX-FileCopyrightText: Copyright (c) 2021-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

"""Defines simulation event types as an enumeration for tracking various stages of simulation lifecycle and physics steps."""

from enum import Enum


class SimulationEvent(Enum):
    """Simulation event types."""

    # Physics-related events

    PHYSICS_PRE_STEP = "isaacsim.simulation.physics_pre_step"
    """Event triggered before a physics step is performed."""

    PHYSICS_POST_STEP = "isaacsim.simulation.physics_post_step"
    """Event triggered after a physics step is performed."""

    # Simulation lifecycle events

    SIMULATION_SETUP = "isaacsim.simulation.simulation_setup"
    """Event triggered when the simulation setup is complete.

    The simulation setup is complete once 1) the application has been played and 2) a warm-up step has been performed.
    The warm-up step ensures that:

    * The physics engine creates and initializes its internal state for the attached stage.
    * An entry point to the physics engine is created through ``omni.physics.tensors``.

    At that point, the simulation is ready to advance.
    """

    SIMULATION_STARTED = "isaacsim.simulation.started"
    """Event triggered when the simulation setup is complete and the simulation is ready to advance."""

    SIMULATION_PAUSED = "isaacsim.simulation.paused"
    """Event triggered when the simulation is paused."""

    SIMULATION_RESUMED = "isaacsim.simulation.resumed"
    """Event triggered when the simulation is resumed.

    The simulation is resumed when the application is played again after being paused.
    """

    SIMULATION_STOPPED = "isaacsim.simulation.stopped"
    """Event triggered when the simulation is stopped."""

    # USD-related events

    PRIM_DELETED = "isaacsim.simulation.prim_deleted"
    """Event triggered when a prim is deleted from the stage."""
