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

"""Isaac Sim Newton Physics Extension.

This extension integrates the Newton physics engine into Isaac Sim, providing
high-performance GPU-accelerated physics simulation with support for multiple
integrators (XPBD, MuJoCo, Featherstone, Semi-Implicit).

Submodules:
    - tensors: Tensor-based interface for Newton physics (see isaacsim.physics.newton.tensors)
"""

from .impl.collision_config import CollisionConfig, HydroelasticConfig
from .impl.extension import NewtonSimExtension as NewtonSimExtension
from .impl.extension import (
    acquire_physics_interface,
    acquire_stage,
    configure_newton,
    get_active_physics_engine,
    get_available_physics_engines,
    get_newton_config,
)
from .impl.newton_config import NewtonConfig, NewtonSolverOutputConfig
from .impl.newton_stage import NewtonStage
from .impl.solver_config import (
    MuJoCoSolverConfig,
    VBDSolverConfig,
    XPBDSolverConfig,
)
from .impl.utils import (
    get_newton_solver,
    get_newton_solver_to_physics_scene_object,
    newton_solver_to_api_schema,
    switch_newton_solver,
)

__all__ = [
    # Public API
    "acquire_physics_interface",
    "acquire_stage",
    "configure_newton",
    "get_newton_config",
    "get_active_physics_engine",
    "get_available_physics_engines",
    # Configuration classes
    "NewtonConfig",
    "NewtonSolverOutputConfig",
    "CollisionConfig",
    "HydroelasticConfig",
    "NewtonStage",
    "XPBDSolverConfig",
    "MuJoCoSolverConfig",
    # utilities
    "newton_solver_to_api_schema",
    "get_newton_solver_to_physics_scene_object",
    "switch_newton_solver",
    "get_newton_solver",
]
