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

"""Main configuration for Newton simulation in Isaac Sim."""

from dataclasses import dataclass, field

from .solver_config import MuJoCoSolverConfig, NewtonSolverConfig


@dataclass
class NewtonConfig:
    """Configuration for Newton physics simulation in Isaac Sim.

    This configuration follows IsaacLab's pattern of separating simulation-level
    parameters from solver-specific parameters.
    """

    # ========== Simulation-Level Settings ==========

    num_substeps: int = 1
    """Number of substeps to use for the solver."""

    debug_mode: bool = False
    """Whether to enable debug mode for the solver."""

    use_cuda_graph: bool = True
    """Whether to use CUDA graph capture for performance optimization.

    When enabled, the simulation loop is captured as a CUDA graph for faster execution.
    Highly recommended for production use.
    """

    time_step_app: bool = True
    """Whether the application should drive simulation time stepping.

    If True, the stage update node calls step_sim(). If False, external code must step.
    """

    physics_frequency: float = 600.0
    """Physics simulation frequency in Hz."""

    update_fabric: bool = True
    """Whether to synchronize Newton state to USD Fabric each frame."""

    disable_physx_fabric_tracker: bool = True
    """Whether to pause PhysX fabric change tracking (if PhysX is loaded)."""

    # ========== USD Parsing Settings ==========

    collapse_fixed_joints: bool = False
    """Whether to merge bodies connected by fixed joints during USD parsing.

    When enabled, reduces body count and improves performance.
    """

    fix_missing_xform_ops: bool = True
    """Whether to add missing identity xform operations to geometry prims to suppress USD warnings."""

    # ========== Physics Material Defaults ==========

    contact_ke: float = 1.0e4
    """Default contact stiffness (spring constant)."""

    contact_kd: float = 1.0e2
    """Default contact damping coefficient."""

    contact_kf: float = 1.0e1
    """Default contact friction force coefficient."""

    contact_mu: float = 1.0
    """Default friction coefficient (Coulomb friction)."""

    contact_ka: float = 0.5
    """Default contact adhesion coefficient."""

    restitution: float = 0.0
    """Default coefficient of restitution (bounciness). 0 = no bounce, 1 = perfectly elastic."""

    contact_margin: float = 0.01
    """Contact margin for rigid body collision detection."""

    soft_contact_margin: float = 0.01
    """Contact margin for soft body collision detection."""

    # ========== Joint Defaults ==========

    joint_limit_ke: float = 1.0e2
    """Default joint limit stiffness."""

    joint_limit_kd: float = 1.0e0
    """Default joint limit damping."""

    armature: float = 0.1
    """Default joint armature (rotor inertia)."""

    joint_damping: float = 1.0
    """Default joint damping coefficient."""

    pd_scale: float = 1.0
    """Scaling factor for PD controller gains when parsing USD joint drives."""

    # ========== Solver Configuration ==========

    solver_cfg: NewtonSolverConfig = field(default_factory=MuJoCoSolverConfig)
    """Solver-specific configuration."""
