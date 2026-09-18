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

"""Provide the Python and Warp Newton tensor backend.

Importing this module registers Newton view factories with the process-wide
``isaacsim.physics.manager.impl.tensors`` registry. Articulation views expose
dense degree-of-freedom state and configuration, root and link state, inertial
properties, dynamics matrices, force application, and structural metadata.
Degree-of-freedom operations are registered only when their index maps are
rectangular; affected operations remain absent for variable-width selections.

Rigid-body views expose pose, velocity, inertial properties, and force
application. Rigid-contact views expose raw contact records, aggregate sensor
forces, sensor-by-filter forces, and sensor metadata. Operations without a
faithful Newton implementation are marked unsupported, and unsupported entity
families produce empty views without registered operations.

The adapters read the attached ``pxr.Usd.Stage`` and Newton model through
``NewtonStage`` and use Warp for tensor dispatch.
"""

from __future__ import annotations

# Re-exports for callers that need the underlying Python/Warp view classes.
from .articulation_view import NewtonArticulationView
from .backend import (
    ArticulationSet,
    NewtonSimView,
    RigidBodySet,
    RigidContactSet,
)
from .entity_factories import (
    DEFAULT_SIMULATION_NAME,
    ENGINE_NAME,
    register_with_umbrella,
    unregister_from_umbrella,
)
from .rigid_body_view import NewtonRigidBodyView
from .rigid_contact_view import NewtonRigidContactView
from .utils import find_matching_paths

# Register Newton view factories on import.
register_with_umbrella()

__all__ = [
    "ArticulationSet",
    "NewtonArticulationView",
    "NewtonRigidBodyView",
    "NewtonRigidContactView",
    "NewtonSimView",
    "RigidBodySet",
    "RigidContactSet",
    "find_matching_paths",
    "register_with_umbrella",
    "unregister_from_umbrella",
    "DEFAULT_SIMULATION_NAME",
    "ENGINE_NAME",
]
