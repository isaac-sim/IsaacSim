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

"""Isaac Sim Newton Tensors Extension.

This extension provides a tensor-based interface for Newton physics simulation in Isaac Sim.
It supports multiple tensor frameworks (NumPy, PyTorch, Warp) and provides views for
articulations, rigid bodies, and contact sensors.
"""

from . import kernels
from .articulation_view import NewtonArticulationView
from .backend import ArticulationSet, NewtonSimView, RigidBodySet, RigidContactSet
from .rigid_body_view import NewtonRigidBodyView
from .rigid_contact_view import NewtonRigidContactView
from .tensor_api import NewtonSimulationView, create_simulation_view
from .utils import find_matching_paths

__all__ = [
    "create_simulation_view",
    "NewtonSimulationView",
    "NewtonSimView",
    "ArticulationSet",
    "RigidBodySet",
    "RigidContactSet",
    "NewtonArticulationView",
    "NewtonRigidBodyView",
    "NewtonRigidContactView",
    "find_matching_paths",
    "kernels",
]
