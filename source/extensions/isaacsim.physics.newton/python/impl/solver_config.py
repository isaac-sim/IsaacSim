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


# WARNING: VBD interface is still in flux. When enable VBD in the future, double check the VBD solver interface with the existing parameters here
@dataclass
class VBDSolverConfig(NewtonSolverConfig):
    """Configuration for VBD (Vertex Block Descent) solver."""

    solver_type: Literal["vbd"] = "vbd"
    """Type of solver to use: 'xpbd', 'mujoco', 'featherstone', 'semiImplicit', 'vbd'."""

    # Common parameters
    iterations: int = 10
    """Number of VBD iterations per step."""

    friction_epsilon: float = 1e-2
    """Threshold to smooth small relative velocities in friction computation (used for both particle and rigid body contacts)."""

    integrate_with_external_rigid_solver: bool = False
    """Indicator for coupled rigid body-cloth simulation. When set to `True`, the solver assumes rigid bodies are integrated by an external solver (one-way coupling)."""

    # Particle parameters
    particle_enable_self_contact: bool = False
    """Whether to enable self-contact detection for particles."""

    particle_self_contact_radius: float = 0.2
    """The radius used for self-contact detection."""

    particle_self_contact_margin: float = 0.2
    """The margin used for self-contact detection."""

    particle_conservative_bound_relaxation: float = 0.85
    """Relaxation factor for conservative penetration-free projection"""

    particle_vertex_contact_buffer_size: int = 32
    """Preallocation size for each vertex's vertex-triangle collision buffer."""

    particle_edge_contact_buffer_size: int = 64
    """Preallocation size for edge's edge-edge collision buffer."""

    particle_collision_detection_interval: int = 0
    """Controls how frequently particle self-contact detection is applied during the simulation."""

    particle_edge_parallel_epsilon: float = 1e-5
    """Threshold to detect near-parallel edges in edge-edge collision handling."""

    particle_enable_tile_solve: bool = True
    """Whether to accelerate the particle solver using tile API."""

    particle_topological_contact_filter_threshold: int = 2
    """Maximum topological distance (measured in rings) under which candidate self-contacts are discarded."""

    particle_rest_shape_contact_exclusion_radius: float = 0.0
    """Additional world-space distance threshold for filtering topologically close primitives."""

    particle_external_vertex_contact_filtering_map: dict | None = None
    """Optional dictionary used to exclude additional vertex-triangle pairs during contact generation."""

    particle_external_edge_contact_filtering_map: dict | None = None
    """Optional dictionary used to exclude additional edge-edge pairs during contact generation."""

    # Rigid body parameters — AVBD hyperparameters
    rigid_avbd_alpha: float = 0.95
    """Used as the default alpha for joints and body-body contacts."""

    rigid_avbd_joint_alpha: float | None = None
    """Joint-specific alpha override."""

    rigid_avbd_contact_alpha: float | None = None
    """Body-body contact alpha override."""

    rigid_avbd_beta: float = 0.0
    """Penalty ramp rate per AVBD iteration."""

    rigid_avbd_linear_beta: float | None = None
    """Linear beta override for linear constraints (meters)."""

    rigid_avbd_angular_beta: float | None = None
    """Angular beta override for angular constraints (radians)."""

    rigid_avbd_gamma: float = 0.999
    """Per-step decay factor for penalty k and persisted hard-mode lambda."""

    # Rigid body - contacts
    rigid_contact_hard: bool = True
    """Whether body-body rigid contacts use hard mode or soft mode."""

    rigid_contact_history: bool = False
    """Whether to persist body-body contact state across steps from the collision pipeline."""

    rigid_contact_stick_motion_eps: float = 1.0e-4
    """Tangential contact residual threshold for marking hard body-body contacts as sticking."""

    rigid_contact_stick_freeze_translation_eps: float = 1.0e-4
    """World-space translation threshold for the body-level deadzone snap on dynamic-dynamic sticking contacts."""

    rigid_contact_stick_freeze_angular_eps: float = 1.0e-4
    """Angular threshold [rad] for the body-level deadzone snap on dynamic-dynamic sticking contacts."""

    rigid_contact_k_start: float = 1.0e2
    """Body-body and body-particle contact penalty seed for AVBD ramping."""

    rigid_body_contact_buffer_size: int = 64
    """Max body-body contacts per rigid body for per-body contact lists."""

    rigid_body_particle_contact_buffer_size: int = 256
    """Max body-particle contacts tracked per rigid body."""

    # Rigid body - joints
    rigid_joint_linear_ke: float = 1.0e5
    """Penalty stiffness ceiling for non-cable structural linear joint slots."""

    rigid_joint_angular_ke: float = 1.0e5
    """Penalty stiffness ceiling for non-cable structural angular joint slots."""

    rigid_joint_linear_k_start: float = 1.0e2
    """Linear penalty seed for AVBD ramping."""

    rigid_joint_angular_k_start: float = 1.0e1
    """Angular penalty seed for AVBD ramping."""

    rigid_joint_linear_kd: float = 0.0
    """Rayleigh damping coefficient for non-cable linear joint constraints."""

    rigid_joint_angular_kd: float = 0.0
    """Rayleigh damping coefficient for non-cable angular joint constraints."""


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
    """Type of solver to use: 'xpbd', 'mujoco', 'featherstone', 'semiImplicit', 'vbd'."""

    iterations: int = 2
    """Number of solver iterations."""


@dataclass
class MuJoCoSolverConfig(NewtonSolverConfig):
    """Configuration for MuJoCo Warp solver-related parameters.

    These parameters are used to configure the MuJoCo Warp solver. For more information, see the
    `MuJoCo Warp documentation`_.

    .. _MuJoCo Warp documentation: https://github.com/google-deepmind/mujoco_warp
    """

    solver_type: Literal["mujoco"] = "mujoco"
    """Type of solver to use: 'xpbd', 'mujoco', 'featherstone', 'semiImplicit', 'vbd'."""

    njmax: int = 1200
    """Number of constraints per environment (world)."""

    nconmax: int | None = 200
    """Number of contact points per environment (world)."""

    cone: Literal["pyramidal", "elliptic"] | None = None
    """Contact friction cone override. If None, use the value authored on the physics scene."""

    use_mujoco_cpu: bool = False
    """Whether to use the pure MuJoCo backend instead of `mujoco_warp`."""

    disable_contacts: bool = False
    """Whether to disable contact computation in MuJoCo."""

    update_data_interval: int = 1
    """Frequency (in simulation steps) at which to update the MuJoCo Data object from the Newton state."""

    save_to_mjcf: str | None = None
    """Optional path to save the generated MJCF model file."""

    use_mujoco_contacts: bool = False
    """Whether to use MuJoCo's contact computation."""

    include_sites: bool = False
    """If True, Newton shapes marked with ShapeFlags.SITE are exported as MuJoCo sites."""
