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

"""Define solver-specific configuration for the Newton backend.

Plain Python types only — no Newton imports here so the module is safe to
import even when the ``newton`` PyPI package is not installed.
"""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum


class SolverType(str, Enum):
    """Enumerate available Newton solver configuration values."""

    XPBD = "xpbd"
    MUJOCO = "mujoco"


@dataclass
class SolverConfig:
    """Store solver-specific tuning requested by an integration.

    These values do not affect Newton stage construction or stepping. The stage constructs an XPBD solver with its
    internal iteration setting.

    Args:
        solver_type: Requested Newton solver family.
        iterations: Requested number of position-correction iterations per substep.
        substeps: Requested number of solver substeps per physics step.
        friction_coefficient: Requested contact-friction coefficient.
        restitution_coefficient: Requested contact-restitution coefficient.

    """

    solver_type: SolverType = SolverType.XPBD
    #: Requested Newton solver family.

    iterations: int = 8
    #: Requested number of position-correction iterations per substep.

    substeps: int = 4
    #: Requested number of solver substeps per physics step.

    friction_coefficient: float = 0.5
    #: Requested contact-friction coefficient.

    restitution_coefficient: float = 0.0
    #: Requested contact-restitution coefficient.
