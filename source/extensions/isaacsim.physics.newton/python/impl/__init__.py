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

"""Newton physics implementation module providing core classes and interfaces for physics simulation."""

from .fabric import FabricManager
from .interface import NewtonPhysicsInterface
from .newton_config import NewtonConfig
from .newton_stage import NewtonStage
from .property_query import (
    NewtonPropertyQueryArticulationLink,
    NewtonPropertyQueryArticulationResponse,
    NewtonPropertyQueryInterface,
    get_newton_property_query_interface,
)
from .solver_config import (
    MuJoCoSolverConfig,
    XPBDSolverConfig,
)

__all__ = [
    "NewtonStage",
    "NewtonPhysicsInterface",
    "FabricManager",
    "NewtonConfig",
    "XPBDSolverConfig",
    "MuJoCoSolverConfig",
    "NewtonPropertyQueryInterface",
    "NewtonPropertyQueryArticulationLink",
    "NewtonPropertyQueryArticulationResponse",
    "get_newton_property_query_interface",
]
