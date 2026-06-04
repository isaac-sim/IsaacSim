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

"""Solver configuration classes for Newton physics engine."""

from dataclasses import dataclass
from typing import Literal


@dataclass
class NewtonSolverConfig:
    """Base configuration for Newton solvers."""

    solver_type: str = "None"
    """Type of solver to use: 'xpbd', 'mujoco', 'featherstone', 'semiImplicit'."""


# Note: num_substeps moved to NewtonConfig (simulation-level parameter)


@dataclass
class XPBDSolverConfig(NewtonSolverConfig):
    """Configuration for XPBD (Extended Position-Based Dynamics) solver.

    An implicit integrator using eXtended Position-Based Dynamics (XPBD) for rigid and soft body simulation.

    References:
        - Miles Macklin, Matthias Müller, and Nuttapong Chentanez. 2016. XPBD: position-based simulation of compliant
          constrained dynamics. In Proceedings of the 9th International Conference on Motion in Games (MIG '16).
          Association for Computing Machinery, New York, NY, USA, 49-54. https://doi.org/10.1145/2994258.2994272
        - Matthias Müller, Miles Macklin, Nuttapong Chentanez, Stefan Jeschke, and Tae-Yong Kim. 2020. Detailed rigid
          body simulation with extended position based dynamics. In Proceedings of the ACM SIGGRAPH/Eurographics
          Symposium on Computer Animation (SCA '20). Eurographics Association, Goslar, DEU,
          Article 10, 1-12. https://doi.org/10.1111/cgf.14105
    """

    solver_type: Literal["xpbd"] = "xpbd"
    """Type of solver to use: 'xpbd', 'mujoco', 'featherstone', 'semiImplicit'."""

    iterations: int = 2
    """Number of solver iterations."""

    soft_body_relaxation: float = 0.9
    """Relaxation parameter for soft body simulation."""

    soft_contact_relaxation: float = 0.9
    """Relaxation parameter for soft contact simulation."""

    joint_linear_relaxation: float = 0.7
    """Relaxation parameter for joint linear simulation."""

    joint_angular_relaxation: float = 0.4
    """Relaxation parameter for joint angular simulation."""

    joint_linear_compliance: float = 0.0
    """Compliance parameter for joint linear simulation."""

    joint_angular_compliance: float = 0.0
    """Compliance parameter for joint angular simulation."""

    rigid_contact_relaxation: float = 0.8
    """Relaxation parameter for rigid contact simulation."""

    rigid_contact_con_weighting: bool = True
    """Whether to use contact constraint weighting for rigid contact simulation."""

    angular_damping: float = 0.0
    """Angular damping parameter for rigid contact simulation."""

    enable_restitution: bool = False
    """Whether to enable restitution for rigid contact simulation."""


@dataclass
class MuJoCoSolverConfig(NewtonSolverConfig):
    """Configuration for MuJoCo Warp solver-related parameters.

    These parameters are used to configure the MuJoCo Warp solver. For more information, see the
    `MuJoCo Warp documentation`_.

    .. _MuJoCo Warp documentation: https://github.com/google-deepmind/mujoco_warp
    """

    solver_type: Literal["mujoco"] = "mujoco"
    """Type of solver to use: 'xpbd', 'mujoco', 'featherstone', 'semiImplicit'."""

    njmax: int = 1200
    """Number of constraints per environment (world)."""

    nconmax: int | None = 200
    """Number of contact points per environment (world)."""

    iterations: int = 100
    """Number of solver iterations."""

    ls_iterations: int = 15
    """Number of line search iterations for the solver."""

    solver: str = "newton"
    """Solver type. Can be "cg" or "newton", or their corresponding MuJoCo integer constants."""

    integrator: str = "implicitfast"
    """Integrator type. Can be "euler", "rk4", or "implicit", or their corresponding MuJoCo integer constants."""

    cone: str = "elliptic"
    """The type of contact friction cone. Can be "pyramidal" or "elliptic"."""

    impratio: float = 1.0
    """Frictional-to-normal constraint impedance ratio."""

    use_mujoco_cpu: bool = False
    """Whether to use the pure MuJoCo backend instead of `mujoco_warp`."""

    disable_contacts: bool = False
    """Whether to disable contact computation in MuJoCo."""

    update_data_interval: int = 1
    """Frequency (in simulation steps) at which to update the MuJoCo Data object from the Newton state."""

    save_to_mjcf: str | None = None
    """Optional path to save the generated MJCF model file."""

    ls_parallel: bool = True
    """Whether to use parallel line search."""

    use_mujoco_contacts: bool = False
    """Whether to use MuJoCo's contact computation."""

    tolerance: float = 1e-6
    """Solver tolerance for early termination of the iterative solver."""

    ls_tolerance: float = 0.001
    """Solver tolerance for early termination of the line search."""

    include_sites: bool = False
    """If True, Newton shapes marked with ShapeFlags.SITE are exported as MuJoCo sites."""
