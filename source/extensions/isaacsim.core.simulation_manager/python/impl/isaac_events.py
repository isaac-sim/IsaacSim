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

"""Defines deprecated Isaac events enumeration for simulation lifecycle management."""

from enum import Enum


class IsaacEvents(Enum):
    """Isaac events types.

    .. deprecated:: 1.7.0

        Use :py:class:`~isaacsim.core.simulation_manager.SimulationEvent` enum instead.
    """

    PHYSICS_WARMUP = "isaac.physics_warmup"
    """Physics warm-up.

    .. deprecated:: 1.7.0

        Use ``SimulationEvent``'s :py:attr:`~isaacsim.core.simulation_manager.SimulationEvent.SIMULATION_SETUP` instead.
    """

    SIMULATION_VIEW_CREATED = "isaac.simulation_view_created"
    """Simulation view created.

    .. deprecated:: 1.7.0

        Use ``SimulationEvent``'s :py:attr:`~isaacsim.core.simulation_manager.SimulationEvent.SIMULATION_SETUP` instead.
    """

    PHYSICS_READY = "isaac.physics_ready"
    """Physics ready.

    .. deprecated:: 1.7.0

        Use ``SimulationEvent``'s :py:attr:`~isaacsim.core.simulation_manager.SimulationEvent.SIMULATION_STARTED` instead.
    """

    POST_RESET = "isaac.post_reset"
    """Post reset.

    .. deprecated:: 1.7.0

        |br| No replacement is provided, as the core experimental API does not use such event.
    """

    PRIM_DELETION = "isaac.prim_deletion"
    """Prim deletion.

    .. deprecated:: 1.7.0

        Use ``SimulationEvent``'s :py:attr:`~isaacsim.core.simulation_manager.SimulationEvent.PRIM_DELETED` instead.
    """

    PRE_PHYSICS_STEP = "isaac.pre_physics_step"
    """Pre physics step.

    .. deprecated:: 1.7.0

        Use ``SimulationEvent``'s :py:attr:`~isaacsim.core.simulation_manager.SimulationEvent.PHYSICS_PRE_STEP` instead.
    """

    POST_PHYSICS_STEP = "isaac.post_physics_step"
    """Post physics step.

    .. deprecated:: 1.7.0

        Use ``SimulationEvent``'s :py:attr:`~isaacsim.core.simulation_manager.SimulationEvent.PHYSICS_POST_STEP` instead.
    """

    TIMELINE_STOP = "isaac.timeline_stop"
    """Timeline stop.

    .. deprecated:: 1.7.0

        Use ``SimulationEvent``'s :py:attr:`~isaacsim.core.simulation_manager.SimulationEvent.SIMULATION_STOPPED` instead.
    """
