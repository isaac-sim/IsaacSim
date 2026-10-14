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

"""Provide the Newton physics simulator backend.

Use :func:`register` to publish Newton simulation callbacks through
``isaacsim.physics.registration``. Applications can then select and drive the
registered simulation through the physics manager. Registration does not
require the Kit application lifecycle, and Newton, Warp, and USD are loaded
only when their functionality is needed.

The registration supplies simulation lifecycle, stepping, subscription, time,
and capability callbacks. Scene-query, interaction, and benchmark callbacks
remain unavailable.
"""

from .newton_config import NewtonConfig, NewtonSolverOutputConfig
from .newton_stage import NewtonStage
from .register_simulation import (
    NewtonSimulationRegistry,
    get_registry,
    register,
    unregister,
)
from .solver_config import SolverConfig, SolverType

__all__ = [
    "NewtonConfig",
    "NewtonSolverOutputConfig",
    "NewtonStage",
    "SolverConfig",
    "SolverType",
    "NewtonSimulationRegistry",
    "register",
    "unregister",
    "get_registry",
]
