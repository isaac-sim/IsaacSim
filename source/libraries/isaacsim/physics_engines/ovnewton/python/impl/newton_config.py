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

"""Define configuration dataclasses for the Newton backend.

The configuration uses plain Python types and does not read process-global
settings.
"""

from __future__ import annotations

from dataclasses import dataclass, field

from .solver_config import SolverConfig


@dataclass
class NewtonConfig:
    """Configure the Newton physics backend.

    Only ``default_steps_per_second`` and ``device`` affect backend behavior. The remaining integration options do not
    affect stage construction or stepping.

    Args:
        use_cuda_graph: Whether an integration should request CUDA-graph capture.
        auto_switch_on_startup: Whether a carrier application should request Newton activation after registration.
        default_steps_per_second: Simulation frequency reported through the registration callbacks.
        solver: Requested solver settings, or None to create the default settings.
        device: Warp device used while ``ovnewton`` finalizes the Newton model, or None to use Newton's process default.

    """

    use_cuda_graph: bool = True
    #: Request CUDA-graph capture when an integration supplies graph-based stepping.

    auto_switch_on_startup: bool = True
    #: Request backend activation when a carrier application honors automatic switching.

    default_steps_per_second: int = 60
    #: Simulation frequency reported through the registration callbacks.

    solver: SolverConfig | None = field(default=None)
    #: Requested solver settings; these do not affect stage construction or stepping.

    device: str | None = None
    #: Warp device active while ``ovnewton`` builds the Newton model, or None to preserve Newton's process default.

    def __post_init__(self) -> None:
        """Populate the default solver configuration."""
        if self.solver is None:
            self.solver = SolverConfig()
