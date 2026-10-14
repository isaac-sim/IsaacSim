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

"""Contact-free differentiable Newton bridge using Featherstone under ``wp.Tape``.

Returned positions, velocities, and applied torques retain a torch autograd path to
the parameter batch.

Trajectories must be fixed-base or otherwise contact-free.

Known approximations and limitations:

- ``SolverFeatherstone`` does not apply ``Model.joint_friction``; joint friction is
  applied by a custom kernel as a smooth Coulomb torque ``-friction * tanh(qd * eps)``
  through ``Control.joint_f``, matching the torque convention of the other SysID
  bridges. ``eps`` is requested at :data:`FRICTION_INV_VELOCITY_EPS` but capped per
  DOF so the torque's zero-velocity slope stays inside the explicit integrator's
  stability region; a capped knee softens Coulomb friction toward viscous damping
  at low speed and is reported.
- Feedforward modes are ``"none"``, ``"gravity"``, and ``"inverse_dynamics"``.
  Feedforward is evaluated at the measured trajectory, remains theta-independent,
  and requires a fixed base.
- ``JOINT_ARMATURE`` is rejected: armature only enters the dynamics through the
  Cholesky factorization, whose Warp adjoint is a stub, so its gradient is silently
  zero. Identify armature with the MuJoCo Newton bridge or a derivative-free backend.
- Every taped substep owns a distinct Featherstone factorization workspace, so
  the joint-space mass matrix is rebuilt at the current configuration without
  overwriting primal values needed by earlier reverse-pass steps.
- Held joints have their drive gains capped to the explicit-integrator stability
  region for ``sub_dt``; identified joints are not modified.
- Mapped-joint PD, feedforward, friction, and effort limiting are evaluated in
  one differentiable kernel. The reported torque is the torque applied to the
  plant. ``effort_clamp`` supports ``"none"`` and ``"max_effort"``.

Row 0 is the measured initial state; command row ``k`` produces state row ``k + 1``.

CUDA graph capture replays forward, backward, and adjoint-zeroing launches for a
rollout shape. The first rollout remains uncaptured to compile kernels and allocate
solver buffers; capture failures fall back to the uncaptured path.
"""

from __future__ import annotations

import gc
import importlib
import itertools
import math
import time
import traceback
from dataclasses import dataclass, field, replace
from typing import TYPE_CHECKING, Any

import carb
import numpy as np
import torch

from .actuator_compatibility import ACTUATOR_RUNTIME_EXPLICIT
from .backend_capabilities import NEWTON_FEATHERSTONE_CAPABILITIES
from .env_bridge import SysIdEnvironmentBridgeError
from .feedforward import (
    FF_CACHE_MAX_ENTRIES,
    FeedforwardCacheEntry,
    compute_feedforward,
    feedforward_content_hash,
    filtered_acceleration,
)
from .inertia_param import inertia_matrix_to_log_cholesky
from .newton_sysid_bridge import (
    NewtonCoreModules,
    _load_newton_core_modules,
    newton_core_modules_available,
)
from .parameter_apply_torch import (
    ThetaDecodeBaselines,
    decode_theta_batch_to_tensors,
    validate_differentiable_entries,
)
from .parameter_types import (
    GLOBAL_LINK_INDEX,
    SysIdParameterEntry,
    SysIdParameterType,
    default_bounds_for_type,
)
from .rollout_result import SysIdRolloutResult
from .run_spec import NEWTON_SOLVER_FEATHERSTONE_DIFF, NewtonSimulationRunSpec
from .trajectory_csv import TrajectoryDataset

if TYPE_CHECKING:
    from .usd_parameter_io import JointUsdSnapshot

# Requested inverse velocity scale of the smooth Coulomb friction torque
# `-friction * tanh(qd * eps)`. The MuJoCo bridge uses `tanh(qd * 100)`, but
# only to reconstruct its reported torque channel after physics: there friction
# is a solver constraint, so the value is never integrated or differentiated.
# Here the same term is a force on an explicit integrator under a Warp tape, so
# the kernel lowers it per DOF whenever the integrator cannot carry the
# resulting stiffness.
FRICTION_INV_VELOCITY_EPS = 100.0

# `-f * tanh(qd * eps)` saturates in the forward rollout but linearizes to an
# effective damping of `f * eps` at `qd = 0`, and that linearization is what the
# reverse pass propagates. Symplectic Euler stays stable while the total
# velocity-proportional slope satisfies
# `(passive + kd + f*eps) * sub_dt / inertia < 2`, where `passive` is the
# model's own `joint_damping`; budget half of that so the adjoint decays rather
# than merely staying bounded, which also leaves room for the effective-inertia
# estimate to be imperfect.
FRICTION_SLOPE_STABILITY_RATIO = 1.0

#: Share of the slope budget held for friction when the substep count is
#: raised to fit it. Splitting the budget evenly keeps the authored drive
#: damping and the Coulomb knee equally representable.
FRICTION_BUDGET_SHARE = 0.5

#: Upper bound on the substep count the bridge raises to on its own. Taped
#: memory and rollout time grow linearly with it, and identifying inertia
#: drives the requested count without bound, so past this the budget falls
#: back to flooring the inertia instead of making every candidate slower.
MAX_AUTO_FEATHERSTONE_SUBSTEPS = 16

# Guards the in-kernel division by the candidate friction. Below this the
# capped scale would exceed the requested one anyway, so the cap never binds
# and the value only keeps the division finite.
_FRICTION_SLOPE_MIN_DENOM = 1.0e-6

# Trajectory poses sampled when measuring the articulated inertia. The mass
# matrix varies with configuration and each sample costs a solver step per DOF,
# so this trades probe time for how much of the pose range the bound covers.
_INERTIA_PROBE_SAMPLES = 4

# Half-range used when a mapped joint declares no finite limit, so the probe
# still samples poses away from rest.
_INERTIA_PROBE_FALLBACK_RANGE = 1.0

#: Cartesian axis each centre-of-mass offset type is added along.
_COM_OFFSET_TYPES: dict[SysIdParameterType, int] = {
    SysIdParameterType.LINK_COM_OFFSET_X: 0,
    SysIdParameterType.LINK_COM_OFFSET_Y: 1,
    SysIdParameterType.LINK_COM_OFFSET_Z: 2,
}

_FALLBACK_HINT = "set simulation.newton.solver='mujoco' or use a derivative-free backend (CMA-ES, Bayesian)"

# Avoid driver failures from very large CUDA graphs. The environment variable
# ISAACSIM_SYSID_CAPTURE_MAX_LAUNCHES overrides this conservative default.
_CAPTURE_MAX_LAUNCHES_DEFAULT = 400_000


def _capture_max_launches() -> int:
    import os

    try:
        return int(os.environ.get("ISAACSIM_SYSID_CAPTURE_MAX_LAUNCHES", _CAPTURE_MAX_LAUNCHES_DEFAULT))
    except ValueError:
        return _CAPTURE_MAX_LAUNCHES_DEFAULT


_DIFF_SUPPORTED_PARAMETERS = NEWTON_FEATHERSTONE_CAPABILITIES.parameter_types

_KERNEL_CACHE: dict[int, tuple[Any, Any, Any]] = {}


def newton_differentiable_modules_available() -> tuple[bool, str]:
    """Return whether the optional modules needed by the Featherstone bridge are importable.

    Returns:
        Result produced by the operation.
    """
    return newton_core_modules_available()


def _load_diff_kernels(wp) -> tuple[Any, Any, Any]:  # noqa: ANN001
    """Build the mapped-torque, state-gather, and target-scatter Warp kernels.

    Args:
        wp: Value supplied for ``wp``.

    Returns:
        Result produced by the operation.
    """  # noqa: DOC106, DOC107
    cached = _KERNEL_CACHE.get(id(wp))
    if cached is not None:
        return cached

    min_friction_denom = wp.constant(_FRICTION_SLOPE_MIN_DENOM)

    @wp.kernel
    def mapped_joint_torque(  # noqa: ANN202
        joint_q: wp.array(dtype=wp.float32),
        joint_qd: wp.array(dtype=wp.float32),
        coord_index_map: wp.array(dtype=wp.int32),
        dof_index_map: wp.array(dtype=wp.int32),
        target_pos_stage: wp.array2d(dtype=wp.float32),
        target_vel_stage: wp.array2d(dtype=wp.float32),
        target_step: int,
        stiffness: wp.array(dtype=wp.float32),
        damping: wp.array(dtype=wp.float32),
        friction: wp.array(dtype=wp.float32),
        inv_velocity_eps: float,
        friction_slope_budget: wp.array(dtype=wp.float32),
        feedforward: wp.array2d(dtype=wp.float32),
        feedforward_step: int,
        effort_limit: wp.array(dtype=wp.float32),
        clamp_effort: int,
        output_step: int,
        joint_f: wp.array(dtype=wp.float32),
        sim_tau: wp.array2d(dtype=wp.float32),
    ):
        tid = wp.tid()
        coord = coord_index_map[tid]
        dof = dof_index_map[tid]
        velocity = joint_qd[dof]
        # `-f * tanh(qd * eps)` saturates at `f`, but its slope at `qd = 0` is
        # `f * eps`, an effective damping the explicit integrator has to carry
        # on every substep. Spend what the budget leaves after the candidate's
        # own damping, so the total slope stays inside the stability region for
        # any candidate. Capping here rather than on the host is what keeps the
        # tape's friction adjoint consistent with the torque actually applied.
        slope_headroom = wp.max(friction_slope_budget[tid] - damping[tid], 0.0)
        capped_eps = slope_headroom / wp.max(friction[tid], min_friction_denom)
        eps = wp.min(inv_velocity_eps, capped_eps)
        torque = (
            stiffness[tid] * (target_pos_stage[target_step, dof] - joint_q[coord])
            + damping[tid] * (target_vel_stage[target_step, dof] - velocity)
            + feedforward[feedforward_step, tid]
            - friction[tid] * wp.tanh(velocity * eps)
        )
        if clamp_effort != 0:
            torque = wp.clamp(torque, -effort_limit[tid], effort_limit[tid])
        joint_f[dof] = torque
        if output_step >= 0:
            sim_tau[output_step, tid] = torque

    @wp.kernel
    def gather_joint_state(  # noqa: ANN202
        joint_q: wp.array(dtype=wp.float32),
        joint_qd: wp.array(dtype=wp.float32),
        coord_index_map: wp.array(dtype=wp.int32),
        dof_index_map: wp.array(dtype=wp.int32),
        step: int,
        sim_q: wp.array2d(dtype=wp.float32),
        sim_qd: wp.array2d(dtype=wp.float32),
    ):
        tid = wp.tid()
        sim_q[step, tid] = joint_q[coord_index_map[tid]]
        sim_qd[step, tid] = joint_qd[dof_index_map[tid]]

    @wp.kernel
    def scatter_joint_targets(  # noqa: ANN202
        target_pos_stage: wp.array2d(dtype=wp.float32),
        target_vel_stage: wp.array2d(dtype=wp.float32),
        step: int,
        joint_target_pos: wp.array(dtype=wp.float32),
        joint_target_vel: wp.array(dtype=wp.float32),
    ):
        tid = wp.tid()
        joint_target_pos[tid] = target_pos_stage[step, tid]
        joint_target_vel[tid] = target_vel_stage[step, tid]

    _KERNEL_CACHE[id(wp)] = (
        mapped_joint_torque,
        gather_joint_state,
        scatter_joint_targets,
    )
    return mapped_joint_torque, gather_joint_state, scatter_joint_targets


@dataclass
class _DiffRolloutContext:
    """Replicated differentiable model plus reusable rollout buffers for one world count."""

    model: Any
    solver_cls: Any
    solvers: list[Any]
    world_count: int
    num_dof: int
    substeps: int
    coords_per_world: int
    dofs_per_world: int
    bodies_per_world: int
    coord_map_np: np.ndarray  # (world_count * num_dof,) into joint_q
    dof_map_np: np.ndarray  # (world_count * num_dof,) into joint_qd / per-DOF arrays
    #: Per-DOF command defaults, including held joints outside the trajectory.
    default_joint_target: np.ndarray
    #: All actuated DOF indices across worlds (mapped + held) for diagnostics.
    all_actuated_dofs_np: np.ndarray
    coord_index_map: Any  # warp int32 twin of `coord_map_np`
    dof_index_map: Any
    stiffness_wp: Any  # (world_count * num_dof,) float32, requires_grad
    damping_wp: Any  # (world_count * num_dof,) float32, requires_grad
    friction_wp: Any  # (world_count * num_dof,) float32, requires_grad
    #: (world_count * num_dof,) float32 total velocity-proportional slope each
    #: DOF may carry. Constant on the tape; the kernel divides it by the
    #: candidate friction so that dependency stays recorded.
    friction_slope_budget_wp: Any
    effort_limit_wp: Any  # (world_count * num_dof,) float32
    clamp_effort: bool
    default_joint_q: np.ndarray
    default_joint_qd: np.ndarray
    default_target_ke: np.ndarray
    default_target_kd: np.ndarray
    baselines: ThetaDecodeBaselines
    link_to_body: dict[int, int]
    states: list[Any]
    controls: list[Any]
    #: Actuated DOFs NOT covered by the trajectory (held at their default pose).
    held_dofs_np: np.ndarray = field(default_factory=lambda: np.zeros(0, dtype=np.int64))
    sim_q: Any = None  # warp (num_steps, world_count * num_dof) float32, requires_grad
    sim_qd: Any = None
    sim_tau: Any = None
    final_joint_f: Any = None
    sim_steps: int = 0
    held_gain_cap_logged: bool = False
    #: Lowest articulated inertia per mapped DOF (world-0 order) the identified
    #: inertials can reach. Kept so a budget resize does not have to re-probe.
    probed_inertia_np: np.ndarray | None = None
    #: Inertia the budget is built from: the probe, floored only where the
    #: substep cap cannot carry the authored damping at the probed value.
    mapped_inertia_np: np.ndarray | None = None
    #: Model ``joint_damping`` per mapped DOF (world-0 order). Featherstone
    #: applies it explicitly alongside the drive, so it spends the same budget.
    passive_damping_np: np.ndarray | None = None
    #: Substep count and floored DOFs last reported, so mixed-rate chunks do
    #: not repeat the same warning on every rollout.
    budget_sizing_logged: tuple | None = None
    #: Factor pulling the coupled modes inside the per-DOF scalar bound.
    #: Independent of the timestep, so it survives a budget resize.
    mapped_coupling: float = 1.0
    #: Inertial corners the probe last minimized over. Changing which inertial
    #: parameters are identified changes the bound, so it re-probes.
    probed_corners: tuple[_WorstCaseInertials, ...] | None = None
    #: Allowed total velocity-proportional slope per mapped DOF.
    friction_slope_budget_np: np.ndarray | None = None
    #: Command sample period the slope budget was last sized for. The budget
    #: scales with ``sub_dt``, so a trajectory at a different rate must resize.
    friction_budget_dt: float = 0.0
    #: Theta-independent controller feedforward; constant on the tape.
    gravity_ff_active: bool = False
    gravity_ff_wp: Any = None
    gravity_ff_np: np.ndarray | None = None
    #: Feedforward cache entry currently uploaded to ``gravity_ff_wp``.
    gravity_ff_source: tuple | None = None
    #: Measured velocity targets used by feedforward controller modes.
    velocity_targets_np: np.ndarray | None = None
    #: Persistent step-indexed drive-target staging buffers.
    target_pos_stage: Any = None
    target_vel_stage: Any = None
    #: Capture records keyed by rollout shape and persistent buffer identity.
    capture_cache: dict[tuple, "_CaptureRecord"] = field(default_factory=dict)
    capture_failed: bool = False
    capture_budget_logged: bool = False
    plain_max_substeps: int = 0
    plain_launches_per_substep: float = 0.0
    plain_forward_s: dict[tuple, float] = field(default_factory=dict)
    plain_backward_s: dict[tuple, float] = field(default_factory=dict)
    #: Fingerprint of the target contents currently staged on the device.
    commands_fingerprint: tuple | None = None


@dataclass
class _CaptureRecord:
    """Forward, backward, and adjoint-zeroing graphs for one rollout shape."""

    tape: Any
    graph_forward: Any
    graph_backward: Any
    graph_zero: Any
    seed_q: Any
    seed_qd: Any
    seed_tau: Any
    replay_logged: bool = False


class NewtonDifferentiableSysIdBridge:
    """Differentiable contact-free Newton bridge conforming to ``SysIdEnvironmentBridge``.

    Rollouts return torch tensors whose autograd graph reaches ``theta_per_env``.
    Structural failures raise instead of substituting a non-differentiable path.

    Args:
        robot_prim_path: Constructor value for ``robot_prim_path``.
        stage_path: Constructor value for ``stage_path``.
        stage: Constructor value for ``stage``.
        newton_config: Constructor value for ``newton_config``.
        joint_baselines: Constructor value for ``joint_baselines``.
        joint_paths: Constructor value for ``joint_paths``.
        link_paths: Constructor value for ``link_paths``.
        baseline_link_inertia_lc: Constructor value for ``baseline_link_inertia_lc``.
        physics_dt: Constructor value for ``physics_dt``.
        num_joints: Constructor value for ``num_joints``.
        num_links: Constructor value for ``num_links``.
        robot_builder: Constructor value for ``robot_builder``.
        actuator_runtime: Constructor value for ``actuator_runtime``.
        **_kwargs: Constructor value for ``_kwargs``.
    """

    supports_arbitrary_candidate_batch = True
    requires_serial_backward = True

    def __init__(
        self,
        *,
        robot_prim_path: str,
        stage_path: str = "",
        stage: Any = None,
        newton_config: NewtonSimulationRunSpec | None = None,
        joint_baselines: list[JointUsdSnapshot | None] | None = None,
        joint_paths: list[str] | None = None,
        link_paths: list[str] | None = None,
        baseline_link_inertia_lc: dict[int, np.ndarray] | None = None,
        physics_dt: float = 1.0 / 60.0,
        num_joints: int = 0,
        num_links: int = 0,
        robot_builder: Any = None,
        actuator_runtime: str = ACTUATOR_RUNTIME_EXPLICIT,
        **_kwargs: Any,
    ) -> None:
        self.robot_prim_path = robot_prim_path
        self.stage_path = stage_path
        self.stage = stage
        self.newton_config = newton_config or NewtonSimulationRunSpec(solver=NEWTON_SOLVER_FEATHERSTONE_DIFF)
        self.actuator_runtime = str(actuator_runtime).lower()
        if self.actuator_runtime != ACTUATOR_RUNTIME_EXPLICIT:
            raise SysIdEnvironmentBridgeError(
                "Differentiable Featherstone implements an explicit mapped-joint PD pipeline; "
                f"got actuator_runtime={actuator_runtime!r}."
            )
        if str(self.newton_config.solver) != NEWTON_SOLVER_FEATHERSTONE_DIFF:
            raise SysIdEnvironmentBridgeError(
                "NewtonDifferentiableSysIdBridge requires "
                f"simulation.newton.solver='{NEWTON_SOLVER_FEATHERSTONE_DIFF}', got {self.newton_config.solver!r}."
            )
        if str(self.newton_config.actuator_source) != "drive_defaults":
            raise SysIdEnvironmentBridgeError(
                "Differentiable Featherstone uses mapped-joint drive baselines and requires "
                "simulation.newton.actuator_source='drive_defaults'."
            )
        if str(self.newton_config.controller) != "pd":
            raise SysIdEnvironmentBridgeError(
                "Differentiable Featherstone implements PD control and requires " "simulation.newton.controller='pd'."
            )
        self._physics_dt = max(float(physics_dt), 1e-9)
        self._modules: NewtonCoreModules = _load_newton_core_modules()
        self._device = torch.device(self.newton_config.device or "cpu")
        self._joint_baselines = list(joint_baselines or [])
        self._joint_paths = [str(path) for path in (joint_paths or [])]
        self._link_paths = [str(path) for path in (link_paths or [])]
        self._baseline_link_inertia_lc = dict(baseline_link_inertia_lc or {})
        self._num_joints = int(num_joints or len(self._joint_baselines) or len(self._joint_paths) or 0)
        self._num_links = int(num_links or len(self._link_paths) or 0)
        self._substeps = max(1, int(getattr(self.newton_config, "featherstone_substeps", 4)))
        self._effort_clamp = str(getattr(self.newton_config, "effort_clamp", "max_effort") or "").strip().lower()
        if self._effort_clamp not in ("none", "max_effort"):
            raise SysIdEnvironmentBridgeError(
                f"Differentiable Featherstone rollouts support effort_clamp='none' or 'max_effort'; "
                f"got {self._effort_clamp!r}."
            )
        mode = str(getattr(self.newton_config, "feedforward", "none") or "none").strip().lower()
        if mode not in ("none", "gravity", "inverse_dynamics"):
            raise SysIdEnvironmentBridgeError(
                f"Unknown simulation.newton.feedforward mode '{mode}'; "
                "expected 'none', 'gravity', or 'inverse_dynamics'."
            )
        self._feedforward_mode = mode
        self._capture_enabled = bool(getattr(self.newton_config, "cuda_graph_capture", True))
        carb.log_warn(f"SysId: differentiable bridge feedforward mode = '{mode}'.")
        self._trajectory: TrajectoryDataset | None = None
        self._ff_cache: dict[int, FeedforwardCacheEntry] = {}
        self._diagnostics_logged = False
        self._robot_builder = robot_builder
        self._ctx_cache: dict[int, _DiffRolloutContext] = {}
        self._featherstone_kernels = self._import_featherstone_kernels()
        self._backward_pending = False

    @property
    def device(self) -> torch.device:  # noqa: D102
        return self._device

    @property
    def actuator_metadata(self) -> dict[str, Any]:
        """Newton run-report metadata for the differentiable bridge."""
        return {
            "solver": NEWTON_SOLVER_FEATHERSTONE_DIFF,
            "actuator_runtime": self.actuator_runtime,
            "featherstone_substeps": self._substeps,
            "featherstone_substeps_effective": max(
                (ctx.substeps for ctx in self._ctx_cache.values()), default=self._substeps
            ),
            "contact_free": True,
            "feedforward": self._feedforward_mode,
            "effort_clamp": self._effort_clamp,
            "cuda_graph_capture": self._capture_enabled,
            "supported_parameters": sorted(ptype.value for ptype in _DIFF_SUPPORTED_PARAMETERS),
        }

    def set_trajectory(self, trajectory: TrajectoryDataset) -> None:  # noqa: D102
        self._trajectory = trajectory
        if self._num_joints <= 0:
            self._num_joints = int(trajectory.num_joints)

    def validate_parameter_entries(self, entries: list[SysIdParameterEntry]) -> None:
        """Reject parameter types the differentiable rollout cannot produce gradients for.

        Args:
            entries: Parameter entries handled by the operation.
        """
        unsupported = validate_differentiable_entries(entries)
        if unsupported:
            armature_note = ""
            if SysIdParameterType.JOINT_ARMATURE.value in unsupported:
                armature_note = " (armature has no adjoint path through the Featherstone Cholesky solve)"
            raise SysIdEnvironmentBridgeError(
                f"The differentiable Newton bridge supports {sorted(p.value for p in _DIFF_SUPPORTED_PARAMETERS)}; "
                f"unsupported: {unsupported}{armature_note}. To identify them, {_FALLBACK_HINT}."
            )

    async def run_rollout_async(  # noqa: D102
        self,
        theta_per_env: torch.Tensor,
        param_entries: list[SysIdParameterEntry],
        commands: torch.Tensor,
        num_steps: int,
    ) -> SysIdRolloutResult:
        if self._trajectory is None:
            raise SysIdEnvironmentBridgeError("Differentiable Newton bridge trajectory is not configured.")
        self.validate_parameter_entries(param_entries)

        rows = theta_per_env.to(device=self._device, dtype=torch.float32)
        num_dof = int(commands.shape[1])
        world_count = int(rows.shape[0])
        num_steps = int(min(num_steps, commands.shape[0]))
        if world_count < 1 or num_dof < 1 or num_steps < 1:
            raise SysIdEnvironmentBridgeError("Differentiable rollout requires at least one world, DOF, and step.")
        requires_backward = bool(rows.requires_grad and torch.is_grad_enabled())
        if self._backward_pending:
            raise SysIdEnvironmentBridgeError(
                "A differentiable Featherstone rollout is still awaiting backward(). "
                "Backpropagate or discard that rollout before starting another one."
            )
        if requires_backward:
            self._acquire_backward_lease()
        try:
            # The friction slope budget scales with sub_dt, so the context has
            # to be sized against the period this rollout will actually use.
            dt = self._rollout_dt(num_steps)
            ctx = self._get_context(world_count, num_dof, num_steps, dt, param_entries)
            entries = self._remap_entries_for_newton(param_entries, ctx.link_to_body)
            decoded = decode_theta_batch_to_tensors(rows, entries, ctx.baselines)
            _require_representable_friction(ctx, decoded.kd, decoded.friction, dt)

            q0, dq0 = self._initial_state(num_dof)

            commands_np = commands[:num_steps].detach().cpu().numpy().astype(np.float32)
            self._ensure_gravity_feedforward(ctx, num_steps, commands_np)

            # Yield once before the synchronous taped rollout; never inside it
            # because Kit renderer kernels must not be recorded on this tape.
            await self._next_update_async()
            positions, velocities, torques = _NewtonDiffRollout.apply(
                decoded.ke,
                decoded.kd,
                decoded.friction,
                decoded.mass,
                decoded.com,
                decoded.inertia,
                self,
                ctx,
                q0,
                dq0,
                commands_np,
                num_steps,
                float(dt),
            )
            await self._next_update_async()
        except BaseException:
            if requires_backward:
                self._release_backward_lease()
            raise

        if self._feedforward_mode != "none" and not self._diagnostics_logged:
            self._diagnostics_logged = True
            ff_means = (
                [round(float(v), 2) for v in ctx.gravity_ff_np[:num_steps].mean(axis=0)]
                if ctx.gravity_ff_np is not None
                else None
            )
            ke_now = decoded.ke[0].detach().cpu().numpy()
            kd_now = decoded.kd[0].detach().cpu().numpy()
            carb.log_warn(
                f"SysId: rollout torque channel: ff_active={ctx.gravity_ff_active} steps={num_steps} "
                f"|tau|mean={float(torques.detach().abs().mean()):.2f} ff_means={ff_means} "
                f"ke={[round(float(v), 1) for v in ke_now]} kd={[round(float(v), 1) for v in kd_now]} mapping="
                + ", ".join(getattr(self, "_joint_mapping_summary", []))
            )
        return SysIdRolloutResult(positions=positions, velocities=velocities, torques=torques)

    def cleanup(self, remove_clones: bool = True) -> None:
        """Release cached differentiable rollout state.

        Args:
            remove_clones: Ignored because Newton rollouts do not create USD clone environments.
        """
        _ = remove_clones
        self._trajectory = None
        self._ctx_cache = {}
        self._ff_cache = {}
        self._backward_pending = False

    def _acquire_backward_lease(self) -> None:
        """Prevent a rollout from overwriting buffers needed by a pending backward pass."""
        if self._backward_pending:
            raise SysIdEnvironmentBridgeError(
                "A differentiable Featherstone rollout is still awaiting backward(). "
                "Backpropagate or discard that rollout before starting another one."
            )
        self._backward_pending = True

    def _release_backward_lease(self) -> None:
        """Release the shared rollout buffers after their backward pass completes."""
        self._backward_pending = False

    # ------------------------------------------------------------------
    # Context construction
    # ------------------------------------------------------------------

    def _get_context(
        self,
        world_count: int,
        num_dof: int,
        num_steps: int,
        dt: float,
        param_entries: Sequence[SysIdParameterEntry],
    ) -> _DiffRolloutContext:
        """Return the rollout context for a world count, building it if needed.

        Contexts are cached per world count and reused across chunks, so the
        timestep-dependent arrays are re-sized here rather than at construction.

        Args:
            world_count: Candidates evaluated in parallel.
            num_dof: Identified joints per world.
            num_steps: Command samples this rollout will integrate.
            dt: Command sample period this rollout will use.
            param_entries: Parameters being identified, which set how low the
                candidate inertials the budget must stay valid for can go.

        Returns:
            Context sized for this rollout.

        Raises:
            SysIdEnvironmentBridgeError: If the trajectory's DOF count does not
                match the model's actuated joint count.
        """
        ctx = self._ctx_cache.get(world_count)
        if ctx is None:
            ctx = self._build_context(world_count, num_dof, param_entries)
            self._ctx_cache[world_count] = ctx
        if ctx.num_dof != num_dof:
            raise SysIdEnvironmentBridgeError(
                f"Trajectory has {num_dof} DOFs but the Newton model exposes {ctx.num_dof} actuated joints per world."
            )
        # The context is cached per world count, but the slope budget scales
        # with sub_dt and multiple-shooting re-enters with chunks recorded at
        # different rates. Resize in place so captured graphs keep their
        # pointer into the same array.
        self._resize_friction_budget(ctx, dt, param_entries)
        self._ensure_rollout_buffers(ctx, num_steps)
        return ctx

    def _resize_friction_budget(
        self,
        ctx: _DiffRolloutContext,
        dt: float,
        param_entries: Sequence[SysIdParameterEntry],
    ) -> None:
        """Re-sizes the substep count and friction slope budget for a rollout.

        The substep count is raised, up to
        :data:`MAX_AUTO_FEATHERSTONE_SUBSTEPS`, until the authored drive damping
        plus the passive joint damping leave friction
        :data:`FRICTION_BUDGET_SHARE` of the stability budget at the lowest
        inertia the identified parameters can reach. Only where the cap still
        falls short is the inertia floored, see :func:`_integrable_inertia`.

        Args:
            ctx: Rollout context whose budget is being resized.
            dt: Command sample period of the rollout about to run.
            param_entries: Parameters being identified this rollout.
        """
        entries = self._remap_entries_for_newton(param_entries, ctx.link_to_body)
        corners = _candidate_inertial_corners(entries, ctx.bodies_per_world)
        probed = ctx.probed_corners is not None and _same_corners(ctx.probed_corners, corners)
        if probed and math.isclose(ctx.friction_budget_dt, dt, rel_tol=1e-9, abs_tol=1e-12):
            return
        if not probed:
            ctx.probed_inertia_np, ctx.mapped_coupling = _articulated_axis_inertia(
                self._modules.warp,
                ctx.model,
                ctx.solver_cls,
                ctx.dof_map_np[: ctx.num_dof],
                ctx.coord_map_np[: ctx.num_dof],
                ctx.default_joint_q,
                corners,
                ctx.bodies_per_world,
            )
            ctx.probed_corners = corners
        damping = ctx.baselines.damping.detach().cpu().numpy()[: ctx.num_dof]
        ctx.substeps = _integrable_substeps(
            ctx.probed_inertia_np,
            damping,
            ctx.passive_damping_np,
            ctx.mapped_coupling,
            self._substeps,
            dt,
        )
        ctx.mapped_inertia_np = _integrable_inertia(
            ctx.probed_inertia_np,
            damping,
            ctx.passive_damping_np,
            ctx.mapped_coupling,
            ctx.substeps,
            dt,
        )
        sizing = (ctx.substeps, tuple(np.flatnonzero(ctx.mapped_inertia_np > ctx.probed_inertia_np).tolist()))
        if sizing != ctx.budget_sizing_logged:
            ctx.budget_sizing_logged = sizing
            _log_budget_sizing(
                ctx.substeps,
                self._substeps,
                ctx.probed_inertia_np,
                ctx.mapped_inertia_np,
                damping,
                ctx.passive_damping_np,
                dt,
                ctx.dof_map_np[: ctx.num_dof],
            )
        budget = _friction_slope_budget(
            ctx.mapped_inertia_np, ctx.mapped_coupling, ctx.substeps, dt, ctx.passive_damping_np
        )
        ctx.friction_slope_budget_np = budget
        ctx.friction_budget_dt = float(dt)
        ctx.friction_slope_budget_wp.assign(np.tile(budget, ctx.world_count).astype(np.float32))
        _warn_softened_friction_knee(
            budget,
            ctx.mapped_inertia_np,
            ctx.baselines.friction.detach().cpu().numpy(),
            ctx.baselines.damping.detach().cpu().numpy(),
            self._commanded_peak_velocity(),
            ctx.mapped_coupling,
            ctx.substeps,
            dt,
            ctx.dof_map_np[: ctx.num_dof],
            ctx.passive_damping_np,
        )

    def _commanded_peak_velocity(self) -> float:
        """Return the largest joint speed the configured trajectory reaches.

        Used only to decide how loudly to report a softened friction knee: a
        knee above this speed means the trajectory never leaves the linear part
        of the tanh, so Coulomb friction was refit as viscous damping.

        Returns:
            Peak absolute joint velocity, or 0.0 when it cannot be determined.
        """
        trajectory = self._trajectory
        if trajectory is None:
            return 0.0
        velocities = getattr(trajectory, "velocities", None)
        if velocities is None:
            return 0.0
        values = np.asarray(velocities, dtype=np.float64)
        if values.size == 0 or not np.all(np.isfinite(values)):
            return 0.0
        return float(np.abs(values).max())

    def _build_context(
        self,
        world_count: int,
        num_dof: int,
        param_entries: Sequence[SysIdParameterEntry],
    ) -> _DiffRolloutContext:
        newton = self._modules.newton
        wp = self._modules.warp
        robot = self._get_robot_builder()
        builder_cls = getattr(newton, "ModelBuilder", None)
        if builder_cls is None:
            raise SysIdEnvironmentBridgeError("Newton ModelBuilder is unavailable.")
        scene = builder_cls()
        replicate = getattr(scene, "replicate", None)
        if not callable(replicate):
            raise SysIdEnvironmentBridgeError("Newton ModelBuilder.replicate is unavailable.")
        replicate(robot, world_count=world_count)
        model = scene.finalize(self.newton_config.device or None, requires_grad=True)

        joint_count = int(model.joint_count)
        if joint_count < 1 or joint_count % world_count != 0:
            raise SysIdEnvironmentBridgeError(
                f"Newton model joint count {joint_count} is not divisible by world count {world_count}."
            )
        joints_per_world = joint_count // world_count
        coords_per_world = int(model.joint_coord_count) // world_count
        dofs_per_world = int(model.joint_dof_count) // world_count
        bodies_per_world = int(model.body_count) // world_count

        coord0, dof0 = self._actuated_joint_maps(model, joints_per_world, num_dof)
        world_index = np.repeat(np.arange(world_count, dtype=np.int64), num_dof)
        coord_map_np = (np.tile(coord0, world_count) + world_index * coords_per_world).astype(np.int32)
        dof_map_np = (np.tile(dof0, world_count) + world_index * dofs_per_world).astype(np.int32)

        solvers = importlib.import_module("newton.solvers")
        solver_cls = getattr(solvers, "SolverFeatherstone", None)
        if solver_cls is None:
            raise SysIdEnvironmentBridgeError("newton.solvers.SolverFeatherstone is unavailable.")
        baselines, link_to_body = self._build_decode_baselines(model, dof0, bodies_per_world)

        # Armature is fixed because Featherstone's factorization has no armature
        # adjoint. A snapshot-derived floor stabilizes held joints.
        actuated_world0 = list(getattr(self, "_all_actuated_world0", []))
        if getattr(model, "joint_armature", None) is not None and self._joint_baselines:
            armature_full = model.joint_armature.numpy().copy()
            snapshot_values = [
                float(getattr(snapshot, "armature", 0.0) or 0.0)
                for snapshot in self._joint_baselines
                if snapshot is not None
            ]
            armature_floor = max(snapshot_values) if snapshot_values else 0.0
            mapped_dofs = {int(value) for value in dof0}
            for idx in range(num_dof):
                snapshot = self._joint_baselines[idx] if idx < len(self._joint_baselines) else None
                value = float(getattr(snapshot, "armature", 0.0) or 0.0) if snapshot is not None else 0.0
                if value > 0.0:
                    for world in range(world_count):
                        dof = int(dof0[idx]) + world * dofs_per_world
                        armature_full[dof] = max(armature_full[dof], value)
            if armature_floor > 0.0:
                for _coord, dof in actuated_world0:
                    if int(dof) in mapped_dofs:
                        continue
                    for world in range(world_count):
                        index = int(dof) + world * dofs_per_world
                        armature_full[index] = max(armature_full[index], armature_floor)
            model.joint_armature.assign(armature_full)

        # Unmapped actuated joints hold their imported configuration.
        default_joint_q = model.joint_q.numpy().copy()
        if model.joint_target_q is not None:
            default_joint_target = model.joint_target_q.numpy().copy()
        else:
            default_joint_target = np.zeros(int(model.joint_dof_count), dtype=np.float32)
        for coord, dof in getattr(self, "_all_actuated_world0", []):
            for world in range(world_count):
                default_joint_target[dof + world * dofs_per_world] = default_joint_q[coord + world * coords_per_world]

        all_actuated_dofs_np = np.asarray(
            [int(dof) + world * dofs_per_world for world in range(world_count) for _coord, dof in actuated_world0],
            dtype=np.int64,
        )
        mapped_dof_set = {int(value) for value in dof_map_np}
        held_dofs_np = np.asarray(
            [int(dof) for dof in all_actuated_dofs_np if int(dof) not in mapped_dof_set],
            dtype=np.int64,
        )
        effort_limits = np.full(num_dof, np.finfo(np.float32).max, dtype=np.float32)
        if self._effort_clamp == "max_effort":
            for idx in range(num_dof):
                snapshot = self._joint_baselines[idx] if idx < len(self._joint_baselines) else None
                max_force = (
                    float(getattr(snapshot, "max_force", float("inf"))) if snapshot is not None else float("inf")
                )
                if math.isfinite(max_force):
                    effort_limits[idx] = max(max_force, 0.0)
        effort_limits = np.tile(effort_limits, world_count)

        # Smooth friction is stiff in the adjoint even where it saturates in the
        # forward pass, so the kernel caps its slope against what this model and
        # sub_dt can integrate. Probe the inertia that cap is measured against
        # here, once, because it costs a solver step per DOF per sample.
        corners = _candidate_inertial_corners(
            self._remap_entries_for_newton(param_entries, link_to_body), bodies_per_world
        )
        mapped_inertia, mapped_coupling = _articulated_axis_inertia(
            wp,
            model,
            solver_cls,
            dof0,
            coord0,
            default_joint_q,
            corners,
            bodies_per_world,
        )
        joint_damping = getattr(model, "joint_damping", None)
        passive_damping = (
            np.abs(np.asarray(joint_damping.numpy(), dtype=np.float64)[dof0])
            if joint_damping is not None
            else np.zeros(num_dof, dtype=np.float64)
        )

        ctx = _DiffRolloutContext(
            model=model,
            solver_cls=solver_cls,
            solvers=[],
            world_count=world_count,
            num_dof=num_dof,
            substeps=self._substeps,
            coords_per_world=coords_per_world,
            dofs_per_world=dofs_per_world,
            bodies_per_world=bodies_per_world,
            coord_map_np=coord_map_np,
            dof_map_np=dof_map_np,
            coord_index_map=wp.array(coord_map_np, dtype=wp.int32, device=model.device),
            dof_index_map=wp.array(dof_map_np, dtype=wp.int32, device=model.device),
            stiffness_wp=wp.zeros(
                world_count * num_dof,
                dtype=wp.float32,
                device=model.device,
                requires_grad=True,
            ),
            damping_wp=wp.zeros(
                world_count * num_dof,
                dtype=wp.float32,
                device=model.device,
                requires_grad=True,
            ),
            friction_wp=wp.zeros(
                world_count * num_dof,
                dtype=wp.float32,
                device=model.device,
                requires_grad=True,
            ),
            friction_slope_budget_wp=wp.zeros(world_count * num_dof, dtype=wp.float32, device=model.device),
            probed_inertia_np=mapped_inertia,
            mapped_inertia_np=mapped_inertia,
            passive_damping_np=passive_damping,
            mapped_coupling=mapped_coupling,
            probed_corners=corners,
            # Left unset so the first `_resize_friction_budget` call sizes the
            # substeps and budget instead of duplicating that here.
            friction_budget_dt=0.0,
            effort_limit_wp=wp.array(effort_limits, dtype=wp.float32, device=model.device),
            clamp_effort=self._effort_clamp == "max_effort",
            default_joint_target=default_joint_target,
            all_actuated_dofs_np=all_actuated_dofs_np,
            held_dofs_np=held_dofs_np,
            default_joint_q=default_joint_q,
            default_joint_qd=model.joint_qd.numpy().copy(),
            default_target_ke=model.joint_target_ke.numpy().copy(),
            default_target_kd=model.joint_target_kd.numpy().copy(),
            baselines=baselines,
            link_to_body=link_to_body,
            states=[],
            controls=[],
        )
        return ctx

    def _actuated_joint_maps(self, model: Any, joints_per_world: int, num_dof: int) -> tuple[np.ndarray, np.ndarray]:
        """Map trajectory DOF order to world-0 joint coordinate/DOF indices.

        Args:
            model: Value supplied for ``model``.
            joints_per_world: Value supplied for ``joints_per_world``.
            num_dof: Value supplied for ``num_dof``.

        Returns:
            Result produced by the operation.
        """  # noqa: DOC107
        joint_q_start = model.joint_q_start.numpy()
        joint_qd_start = model.joint_qd_start.numpy()
        labels = [str(label) for label in getattr(model, "joint_label", [])]

        actuated: list[tuple[int, int, int]] = []  # (joint index, coord index, dof index)
        for joint in range(joints_per_world):
            coord_span = int(joint_q_start[joint + 1] - joint_q_start[joint])
            dof_span = int(joint_qd_start[joint + 1] - joint_qd_start[joint])
            if dof_span == 0:
                continue  # fixed joint
            if coord_span == 7 and dof_span == 6:
                continue  # floating base (free joint): tolerated, not actuated
            if coord_span == 1 and dof_span == 1:
                actuated.append((joint, int(joint_q_start[joint]), int(joint_qd_start[joint])))
                continue
            raise SysIdEnvironmentBridgeError(
                f"Joint {joint} has {dof_span} DOFs; the differentiable bridge supports 1-DOF "
                f"joints plus an optional floating base. To simulate this robot, {_FALLBACK_HINT}."
            )
        if len(actuated) < num_dof:
            raise SysIdEnvironmentBridgeError(
                f"Newton model exposes {len(actuated)} actuated joints per world but the trajectory "
                f"has {num_dof} DOFs (check robot import and joint ordering)."
            )

        # Subset mappings require unique paths; import-order guesses are unsafe.
        ordered = actuated[:num_dof]
        if self._joint_paths:
            if len(self._joint_paths) < num_dof:
                raise SysIdEnvironmentBridgeError(
                    f"The trajectory has {num_dof} DOFs but only {len(self._joint_paths)} joint paths were supplied."
                )
            if len(labels) < joints_per_world:
                raise SysIdEnvironmentBridgeError(
                    "Newton did not expose enough joint labels to resolve the supplied trajectory joint paths."
                )
            matched: list[tuple[int, int, int]] = []
            for path in self._joint_paths[:num_dof]:
                candidates = []
                for joint, coord, dof in actuated:
                    label = labels[joint]
                    if label == path or label.endswith(path) or path.rsplit("/", 1)[-1] == label.rsplit("/", 1)[-1]:
                        candidates.append((joint, coord, dof))
                if len(candidates) != 1:
                    raise SysIdEnvironmentBridgeError(
                        f"Cannot uniquely map trajectory joint path '{path}' to a Newton joint "
                        f"({len(candidates)} matches). Check the imported joint labels."
                    )
                matched.append(candidates[0])
            if len({item[0] for item in matched}) != num_dof:
                raise SysIdEnvironmentBridgeError("Trajectory joint paths map to duplicate Newton joints.")
            ordered = matched
        elif len(actuated) != num_dof:
            raise SysIdEnvironmentBridgeError(
                f"Newton exposes {len(actuated)} actuated joints but the trajectory has {num_dof} DOFs. "
                "Supply joint paths to map the trajectory subset explicitly."
            )
        coord0 = np.asarray([item[1] for item in ordered], dtype=np.int32)
        dof0 = np.asarray([item[2] for item in ordered], dtype=np.int32)
        self._all_actuated_world0 = [(int(c), int(d)) for _j, c, d in actuated]
        self._joint_mapping_summary = [
            f"tele[{i}]->{labels[j].rsplit('/', 1)[-1] if j < len(labels) else j}(dof {d})"
            for i, (j, _c, d) in enumerate(ordered)
        ]
        return coord0, dof0

    def _build_decode_baselines(
        self,
        model,  # noqa: ANN001
        dof0: np.ndarray,
        bodies_per_world: int,
    ) -> tuple[ThetaDecodeBaselines, dict[int, int]]:
        """Read parameter baselines from the finalized model (world 0) and USD snapshots.

        Args:
            model: Value supplied for ``model``.
            dof0: Value supplied for ``dof0``.
            bodies_per_world: Value supplied for ``bodies_per_world``.

        Returns:
            Result produced by the operation.
        """  # noqa: DOC107
        num_dof = int(dof0.shape[0])
        model_ke = model.joint_target_ke.numpy()
        model_kd = model.joint_target_kd.numpy()
        model_friction = model.joint_friction.numpy() if getattr(model, "joint_friction", None) is not None else None

        stiffness = np.ones(num_dof, dtype=np.float32)
        damping = np.ones(num_dof, dtype=np.float32)
        friction = np.zeros(num_dof, dtype=np.float32)
        for idx in range(num_dof):
            snapshot = self._joint_baselines[idx] if idx < len(self._joint_baselines) else None
            dof = int(dof0[idx])
            if snapshot is not None:
                stiffness[idx] = max(float(snapshot.stiffness), 1e-6)
                damping[idx] = max(float(snapshot.damping), 1e-6)
                friction[idx] = max(float(snapshot.friction or snapshot.dynamic_friction), 0.0)
            else:
                stiffness[idx] = max(float(model_ke[dof]), 1e-6)
                damping[idx] = max(float(model_kd[dof]), 1e-6)
                if model_friction is not None:
                    friction[idx] = max(float(model_friction[dof]), 0.0)

        mass = model.body_mass.numpy()[:bodies_per_world].astype(np.float64)
        com = np.asarray(model.body_com.numpy())[:bodies_per_world].reshape(bodies_per_world, 3).astype(np.float64)
        inertia = (
            np.asarray(model.body_inertia.numpy())[:bodies_per_world].reshape(bodies_per_world, 3, 3).astype(np.float64)
        )
        inertia_lc = np.zeros((bodies_per_world, 6), dtype=np.float64)
        for body in range(bodies_per_world):
            try:
                inertia_lc[body] = inertia_matrix_to_log_cholesky(inertia[body])
            except ValueError:
                # Zero/degenerate inertia (e.g. massless fixed base): leave the neutral
                # encoding; such bodies must not carry inertia parameters anyway.
                inertia_lc[body] = 0.0

        baselines = ThetaDecodeBaselines(
            stiffness=torch.as_tensor(stiffness, device=self._device),
            damping=torch.as_tensor(damping, device=self._device),
            friction=torch.as_tensor(friction, device=self._device),
            link_mass=torch.as_tensor(mass, device=self._device, dtype=torch.float32),
            link_com=torch.as_tensor(com, device=self._device, dtype=torch.float32),
            link_inertia=torch.as_tensor(inertia, device=self._device, dtype=torch.float32),
            link_inertia_lc=torch.as_tensor(inertia_lc, device=self._device, dtype=torch.float32),
        )

        link_to_body: dict[int, int] = {}
        labels = [str(label) for label in getattr(model, "body_label", [])][:bodies_per_world]
        for link_index, path in enumerate(self._link_paths):
            for body, label in enumerate(labels):
                if label == path or label.endswith(path) or path.endswith(label.rsplit("/", 1)[-1]):
                    link_to_body[link_index] = body
                    break
        return baselines, link_to_body

    def _remap_entries_for_newton(
        self,
        param_entries: Sequence[SysIdParameterEntry],
        link_to_body: dict[int, int],
    ) -> list[SysIdParameterEntry]:
        """Translate SysID link indices into Newton world-0 body indices.

        Args:
            param_entries: Parameters in SysID link order.
            link_to_body: SysID link index to Newton world-0 body index.

        Returns:
            Entries whose link parameters index Newton bodies.

        Raises:
            SysIdEnvironmentBridgeError: If a link parameter names a link with
                no matching Newton body.
        """
        link_types = (
            SysIdParameterType.LINK_MASS,
            SysIdParameterType.LINK_COM_OFFSET_X,
            SysIdParameterType.LINK_COM_OFFSET_Y,
            SysIdParameterType.LINK_COM_OFFSET_Z,
            SysIdParameterType.LINK_INERTIA_LOG_CHOLESKY,
        )
        remapped: list[SysIdParameterEntry] = []
        for entry in param_entries:
            if entry.param_type not in link_types or entry.link_index == GLOBAL_LINK_INDEX:
                remapped.append(entry)
                continue
            body = link_to_body.get(entry.link_index)
            if body is None:
                path = (
                    self._link_paths[entry.link_index]
                    if 0 <= entry.link_index < len(self._link_paths)
                    else f"link {entry.link_index}"
                )
                raise SysIdEnvironmentBridgeError(
                    f"Cannot map SysID link '{path}' to a Newton body for parameter "
                    f"{entry.param_type.value}; check the robot import."
                )
            remapped.append(replace(entry, link_index=body))
        return remapped

    def _ensure_rollout_buffers(self, ctx: _DiffRolloutContext, num_steps: int) -> None:
        """Grow the per-substep rollout buffers, reclaiming memory on allocation failure.

        A longer rollout (e.g. the post-solve confidence stage's full-trajectory
        nominal rollout) can require several GiB of fresh per-substep states while
        earlier stages still pin captured graphs and other contexts' buffers; on a
        16 GB card the growth itself is what dies. One reclaim-and-retry pass frees
        everything droppable (all capture caches, other contexts' state lists) and
        resumes the partial growth, so the stage degrades to uncaptured rollouts
        instead of aborting.

        Args:
            ctx: Value supplied for ``ctx``.
            num_steps: Value supplied for ``num_steps``.
        """
        try:
            self._grow_rollout_buffers(ctx, num_steps)
        except Exception as exc:  # noqa: BLE001 - allocation failure mode is driver-specific
            carb.log_warn(
                f"SysId: rollout buffer allocation failed for steps={num_steps} x substeps={ctx.substeps} "
                f"(worlds={ctx.world_count}): {exc}. Releasing captured graphs and cached rollout buffers, "
                "then retrying once."
            )
            self._reclaim_rollout_memory(keep_ctx=ctx)
            try:
                self._grow_rollout_buffers(ctx, num_steps)
            except Exception as retry_exc:
                raise SysIdEnvironmentBridgeError(
                    f"Could not allocate differentiable rollout buffers for {num_steps} steps x "
                    f"{ctx.substeps} substeps (worlds={ctx.world_count}) even after releasing captured "
                    f"CUDA graphs and cached buffers: {retry_exc}"
                ) from retry_exc

    def _grow_rollout_buffers(self, ctx: _DiffRolloutContext, num_steps: int) -> None:
        wp = self._modules.warp
        model = ctx.model
        total_substeps = max(0, num_steps - 1) * ctx.substeps
        while len(ctx.states) < total_substeps + 1:
            ctx.states.append(model.state())
        while len(ctx.controls) < total_substeps:
            control = model.control()
            if control.joint_target_q is None:
                control.joint_target_q = wp.zeros(
                    int(model.joint_dof_count),
                    dtype=wp.float32,
                    device=model.device,
                    requires_grad=True,
                )
            control.joint_target_qd.zero_()
            control.joint_f.zero_()
            ctx.controls.append(control)
        while len(ctx.solvers) < total_substeps:
            # A shared solver would overwrite M/J/H/L before the reverse pass.
            ctx.solvers.append(ctx.solver_cls(model, angular_damping=0.0, update_mass_matrix_interval=1))
        if ctx.sim_q is None or ctx.sim_steps < num_steps:
            width = ctx.world_count * ctx.num_dof
            ctx.sim_q = wp.zeros(
                (num_steps, width),
                dtype=wp.float32,
                device=model.device,
                requires_grad=True,
            )
            ctx.sim_qd = wp.zeros(
                (num_steps, width),
                dtype=wp.float32,
                device=model.device,
                requires_grad=True,
            )
            ctx.sim_tau = wp.zeros(
                (num_steps, width),
                dtype=wp.float32,
                device=model.device,
                requires_grad=True,
            )
            ctx.final_joint_f = wp.zeros(
                int(model.joint_dof_count),
                dtype=wp.float32,
                device=model.device,
                requires_grad=True,
            )
            ctx.sim_steps = num_steps
            ctx.capture_cache.clear()
            # Stable staging identities allow capture reuse across chunks.
            ctx.target_pos_stage = wp.zeros(
                (num_steps, int(model.joint_dof_count)),
                dtype=wp.float32,
                device=model.device,
            )
            ctx.target_vel_stage = wp.zeros(
                (num_steps, int(model.joint_dof_count)),
                dtype=wp.float32,
                device=model.device,
            )
            ctx.commands_fingerprint = None
            if self._feedforward_mode != "none":
                ctx.gravity_ff_wp = wp.zeros((num_steps, width), dtype=wp.float32, device=model.device)
                ctx.gravity_ff_source = None
        if ctx.gravity_ff_wp is None:
            # Keep the rollout kernel signature uniform without feedforward.
            ctx.gravity_ff_wp = wp.zeros(
                (1, ctx.world_count * ctx.num_dof),
                dtype=wp.float32,
                device=model.device,
            )

    def _ensure_gravity_feedforward(self, ctx: _DiffRolloutContext, num_steps: int, commands_np: np.ndarray) -> None:
        """Load the per-step controller feedforward, computing and caching per trajectory.

        Args:
            ctx: Value supplied for ``ctx``.
            num_steps: Value supplied for ``num_steps``.
            commands_np: Value supplied for ``commands_np``.
        """
        if self._feedforward_mode == "none":
            ctx.gravity_ff_active = False
            return
        entry = self._feedforward_cache_entry(ctx, num_steps, commands_np)
        if entry.feedforward is None:
            raise SysIdEnvironmentBridgeError(
                "The differentiable feedforward cache contains no torque data. "
                "Feedforward computation must succeed before a rollout can run."
            )
        rows = min(int(ctx.gravity_ff_wp.shape[0]), entry.steps)
        source = (entry.key, entry.content_hash, rows)
        if ctx.gravity_ff_source == source:
            return
        # Preserve the buffer identity referenced by captured graphs.
        buffer = np.zeros(
            (int(ctx.gravity_ff_wp.shape[0]), ctx.world_count * ctx.num_dof),
            dtype=np.float32,
        )
        buffer[:rows] = np.tile(entry.feedforward[:rows].astype(np.float32), (1, ctx.world_count))
        ctx.gravity_ff_wp.assign(buffer)
        ctx.gravity_ff_np = entry.feedforward
        ctx.velocity_targets_np = entry.velocity_targets
        ctx.gravity_ff_source = source
        ctx.gravity_ff_active = True

    def _feedforward_cache_entry(
        self, ctx: _DiffRolloutContext, num_steps: int, commands_np: np.ndarray
    ) -> FeedforwardCacheEntry:
        """Return the cached feedforward for the active trajectory, computing on miss.

        The optimizer alternates ``set_trajectory`` over the training chunks every
        iteration; identity-plus-content keying makes that alternation a cache hit
        instead of a full nominal-model recomputation per rollout.

        Args:
            ctx: Value supplied for ``ctx``.
            num_steps: Value supplied for ``num_steps``.
            commands_np: Value supplied for ``commands_np``.

        Returns:
            Result produced by the operation.
        """
        trajectory = self._trajectory
        key = id(trajectory)
        content_hash = feedforward_content_hash(trajectory)
        entry = self._ff_cache.get(key)
        if entry is not None and entry.content_hash == content_hash and entry.steps >= num_steps:
            self._ff_cache.pop(key)  # refresh LRU order
            self._ff_cache[key] = entry
            return entry
        entry = self._compute_feedforward_entry(ctx, num_steps, commands_np, key, content_hash)
        self._ff_cache.pop(key, None)
        while len(self._ff_cache) >= FF_CACHE_MAX_ENTRIES:
            self._ff_cache.pop(next(iter(self._ff_cache)))
        self._ff_cache[key] = entry
        return entry

    def _compute_feedforward_entry(
        self,
        ctx: _DiffRolloutContext,
        num_steps: int,
        commands_np: np.ndarray,
        key: int,
        content_hash: int,
    ) -> FeedforwardCacheEntry:
        """Compute the feedforward for the active trajectory and mode.

        Args:
            ctx: Value supplied for ``ctx``.
            num_steps: Value supplied for ``num_steps``.
            commands_np: Value supplied for ``commands_np``.
            key: Value supplied for ``key``.
            content_hash: Value supplied for ``content_hash``.

        Returns:
            Result produced by the operation.
        """
        # Floating-base feedforward would require unobserved base-wrench data.
        joint_type = ctx.model.joint_type.numpy()[: int(ctx.model.joint_count) // ctx.world_count]
        newton = self._modules.newton
        if any(int(jt) in (int(newton.JointType.FREE), int(newton.JointType.DISTANCE)) for jt in joint_type):
            raise SysIdEnvironmentBridgeError(
                f"simulation.newton.feedforward='{self._feedforward_mode}' requires a fixed-base robot; "
                "this articulation has a FREE/DISTANCE joint. Use feedforward='none' for floating-base robots."
            )
        # Match the controller's measured-state evaluation when telemetry exists.
        positions = None
        trajectory = self._trajectory
        if trajectory is not None and int(trajectory.positions.shape[0]) >= num_steps:
            positions = np.asarray(trajectory.positions[:num_steps, : ctx.num_dof], dtype=np.float64)
        if positions is None:
            positions = np.asarray(commands_np[:num_steps, : ctx.num_dof], dtype=np.float64)
        velocities = None
        if trajectory is not None and int(trajectory.velocities.shape[0]) >= num_steps:
            velocities = np.asarray(trajectory.velocities[:num_steps, : ctx.num_dof], dtype=np.float64)
        if velocities is None:
            velocities = np.zeros_like(positions)
        # Tracking-controller damping acts on measured velocity error.
        rates_for_ff = None
        accelerations = None
        if self._feedforward_mode == "inverse_dynamics":
            rates_for_ff = velocities
            times = None
            if trajectory is not None and int(np.asarray(trajectory.times).shape[0]) >= num_steps:
                times = np.asarray(trajectory.times[:num_steps], dtype=np.float64).reshape(-1)
            accelerations = _filtered_acceleration(velocities, times)
        try:
            feedforward = _compute_feedforward(ctx, self._modules.newton, positions, rates_for_ff, accelerations)
        except SysIdEnvironmentBridgeError:
            raise
        except Exception as exc:
            raise SysIdEnvironmentBridgeError(
                f"Could not compute the requested '{self._feedforward_mode}' feedforward for the "
                f"differentiable Featherstone rollout: {exc}"
            ) from exc
        carb.log_warn(
            f"SysId: differentiable bridge '{self._feedforward_mode}' feedforward active over "
            f"{num_steps} steps (|ff| max {float(np.abs(feedforward).max()):.2f} Nm)."
        )
        return FeedforwardCacheEntry(
            key=key,
            trajectory=trajectory,
            content_hash=content_hash,
            steps=num_steps,
            feedforward=feedforward,
            velocity_targets=velocities.astype(np.float32),
        )

    def _get_robot_builder(self):  # noqa: ANN202
        """Build (once) the single-world robot ModelBuilder.

        Returns:
            Result produced by the operation.
        """
        if self._robot_builder is not None:
            return self._robot_builder
        builder_cls = getattr(self._modules.newton, "ModelBuilder", None)
        if builder_cls is None:
            raise SysIdEnvironmentBridgeError("Newton ModelBuilder is unavailable.")
        source = self.stage if self.stage is not None else self.stage_path
        if not source:
            raise SysIdEnvironmentBridgeError("Differentiable Newton bridge requires a stage or stage path.")
        builder = builder_cls()
        add_usd = getattr(builder, "add_usd", None)
        if not callable(add_usd):
            raise SysIdEnvironmentBridgeError("Newton ModelBuilder.add_usd is unavailable for this robot.")
        add_usd(source, root_path=self.robot_prim_path or "/")
        self._robot_builder = builder
        return builder

    def _import_featherstone_kernels(self):  # noqa: ANN202
        try:
            return importlib.import_module("newton._src.solvers.featherstone.kernels")
        except Exception as exc:
            raise SysIdEnvironmentBridgeError(
                f"Newton Featherstone kernels are unavailable ({exc}); {_FALLBACK_HINT}."
            ) from exc

    def _capture_supported(self, ctx: _DiffRolloutContext) -> bool:
        """Whether CUDA graph capture applies to this context (CUDA-only, opt-out flag).

        Args:
            ctx: Value supplied for ``ctx``.

        Returns:
            Result produced by the operation.
        """
        if not self._capture_enabled or ctx.capture_failed:
            return False
        try:
            device = self._modules.warp.get_device(ctx.model.device)
        except Exception:
            return False
        return bool(getattr(device, "is_cuda", False))

    def release_rollout_memory(self) -> None:
        """Free captured CUDA graphs, tapes, and per-substep rollout buffers.

        Post-solve stages (validation replay, parameter confidence) run
        forward-only rollouts, often at a different world count (the confidence
        batch rolls baseline + all perturbations as parallel worlds). On a
        16 GB card the solve's captured graphs (~7 GiB per 800x32 shape) plus
        the grown per-substep state lists otherwise leave too little memory and
        the confidence stage dies on its first allocation. Buffers are rebuilt
        lazily by the next rollout; capture is disabled bridge-wide afterwards
        — including for contexts built later, such as the confidence batch's
        multi-world model — because gradient work is finished and re-capturing
        would re-pin the memory the later stages need.
        """
        self._capture_enabled = False
        self._reclaim_rollout_memory()

    def _reclaim_rollout_memory(self, keep_ctx: _DiffRolloutContext | None = None) -> None:
        """Drop every reclaimable rollout allocation (captured graphs, state lists).

        ``keep_ctx`` preserves that context's partially grown state/control
        lists so an out-of-memory retry can resume the growth instead of
        starting over. Capture is latched off on every context either way:
        after memory pressure, graph re-capture would immediately re-pin
        multi-GiB allocations.

        Args:
            keep_ctx: Value supplied for ``keep_ctx``.
        """
        for ctx in self._ctx_cache.values():
            ctx.capture_cache.clear()
            ctx.capture_failed = True
            ctx.plain_max_substeps = 0
            ctx.commands_fingerprint = None
            if ctx is keep_ctx:
                continue
            ctx.states.clear()
            ctx.controls.clear()
            ctx.solvers.clear()
            ctx.sim_q = None
            ctx.sim_qd = None
            ctx.sim_tau = None
            ctx.final_joint_f = None
            ctx.target_pos_stage = None
            ctx.target_vel_stage = None
            ctx.sim_steps = 0
        gc.collect()
        try:
            if torch.cuda.is_available():
                torch.cuda.empty_cache()
        except Exception:  # noqa: BLE001 - memory release is best-effort
            pass

    def _initial_state(self, num_dof: int) -> tuple[np.ndarray, np.ndarray]:
        assert self._trajectory is not None
        q = np.zeros(num_dof, dtype=np.float32)
        dq = np.zeros(num_dof, dtype=np.float32)
        if self._trajectory.positions.shape[0] > 0:
            q = np.asarray(self._trajectory.positions[0, :num_dof], dtype=np.float32)
        if self._trajectory.velocities.shape[0] > 0:
            dq = np.asarray(self._trajectory.velocities[0, :num_dof], dtype=np.float32)
        return q, dq

    def _rollout_dt(self, num_steps: int) -> float:
        if self._trajectory is None or num_steps < 2:
            return self._physics_dt
        times = np.asarray(self._trajectory.times[:num_steps], dtype=np.float64).reshape(-1)
        if times.shape[0] < 2:
            return self._physics_dt
        diffs = np.diff(times)
        diffs = diffs[np.isfinite(diffs) & (diffs > 0.0)]
        if diffs.size == 0:
            return self._physics_dt
        return float(np.median(diffs))

    @staticmethod
    async def _next_update_async() -> None:
        from .runtime import yield_control

        await yield_control()


def _filtered_acceleration(velocities: np.ndarray, times: np.ndarray | None) -> np.ndarray:
    return filtered_acceleration(velocities, times)


def _compute_feedforward(
    ctx: _DiffRolloutContext,
    newton_module,  # noqa: ANN001
    q_steps: np.ndarray,
    qd_steps: np.ndarray | None = None,
    qdd_steps: np.ndarray | None = None,
) -> np.ndarray:
    """Adapter over :func:`feedforward.compute_feedforward` using the rollout context.

    Args:
        ctx: Value supplied for ``ctx``.
        newton_module: Value supplied for ``newton_module``.
        q_steps: Value supplied for ``q_steps``.
        qd_steps: Value supplied for ``qd_steps``.
        qdd_steps: Value supplied for ``qdd_steps``.

    Returns:
        Result produced by the operation.
    """  # noqa: DOC107
    return compute_feedforward(
        newton_module,
        ctx.model,
        world_count=ctx.world_count,
        coords_per_world=ctx.coords_per_world,
        dofs_per_world=ctx.dofs_per_world,
        coord_map_world0=ctx.coord_map_np,
        dof_map_world0=ctx.dof_map_np,
        default_joint_q_world0=ctx.default_joint_q,
        baseline_mass=ctx.baselines.link_mass.detach().cpu().numpy(),
        baseline_com=ctx.baselines.link_com.detach().cpu().numpy(),
        baseline_inertia=ctx.baselines.link_inertia.detach().cpu().numpy(),
        q_steps=q_steps,
        qd_steps=qd_steps,
        qdd_steps=qdd_steps,
    )


def _inertia_probe_poses(
    model: Any,  # noqa: ANN401
    dof0: np.ndarray,
    coord0: np.ndarray,
    default_joint_q: np.ndarray,
) -> np.ndarray:
    """Return the configurations the articulated-inertia probe evaluates.

    The rest pose plus deterministic interior samples of each joint's limit
    range. Sampling all joints together rather than one at a time keeps the
    probe linear in DOF count; a serial chain's distal inertia is dominated by
    how folded the chain is, which joint-wide samples do capture.

    Args:
        model: Finalized Newton model supplying joint limits.
        dof0: World-0 DOF index per mapped DOF. Joint limits are DOF-indexed.
        coord0: World-0 coordinate index per mapped DOF.
        default_joint_q: Full rest coordinate vector.

    Returns:
        ``(num_samples, num_mapped)`` array of joint coordinates.
    """
    rest = np.asarray(default_joint_q, dtype=np.float64)[coord0]
    lower = _limit_array(model, "joint_limit_lower", dof0, rest - _INERTIA_PROBE_FALLBACK_RANGE)
    upper = _limit_array(model, "joint_limit_upper", dof0, rest + _INERTIA_PROBE_FALLBACK_RANGE)
    span = np.where(upper > lower, upper - lower, 0.0)
    centre = np.where(upper > lower, 0.5 * (lower + upper), rest)

    poses = [rest]
    # Fixed fractions of the range rather than random draws, so the bound is
    # reproducible across processes and runs.
    for fraction in np.linspace(-0.5, 0.5, num=_INERTIA_PROBE_SAMPLES):
        poses.append(centre + fraction * span)
    return np.asarray(poses, dtype=np.float64)


def _limit_array(
    model: Any,  # noqa: ANN401
    name: str,
    dof0: np.ndarray,
    fallback: np.ndarray,
) -> np.ndarray:
    """Read a per-DOF joint limit, substituting a range where it is unbounded.

    Args:
        model: Finalized Newton model.
        name: Model attribute holding the DOF-indexed limit array.
        dof0: World-0 DOF index per mapped DOF.
        fallback: Values to use where the limit is missing or not finite.

    Returns:
        Finite limit value per mapped DOF.
    """
    source = getattr(model, name, None)
    if source is None:
        return np.asarray(fallback, dtype=np.float64)
    values = np.asarray(source.numpy(), dtype=np.float64)
    fallback = np.asarray(fallback, dtype=np.float64)
    # Fall back per DOF: one index past the end of a short array must not throw
    # away the real limits of every other joint.
    in_range = dof0 < values.shape[0]
    picked = np.where(in_range, values[np.minimum(dof0, max(values.shape[0] - 1, 0))], np.inf)
    usable = in_range & np.isfinite(picked) & (np.abs(picked) < 1.0e6)
    return np.where(usable, picked, fallback)


def _articulated_axis_inertia(
    wp: Any,  # noqa: ANN401
    model: Any,  # noqa: ANN401
    solver_cls: Any,  # noqa: ANN401
    dof0: np.ndarray,
    coord0: np.ndarray,
    default_joint_q: np.ndarray,
    corners: tuple[_WorstCaseInertials, ...],
    bodies_per_world: int,
) -> tuple[np.ndarray, float]:
    """Return the effective inertia each mapped DOF feels for its own torque.

    Stability of an explicit integrator at joint ``i`` is governed by the
    *articulated* inertia ``1 / (H^-1)[i, i]``, not by the mass-matrix diagonal
    ``H[i, i]``. Those differ: coupling to the rest of the chain reduces what a
    joint feels, and ``1/(H^-1)[i, i] <= H[i, i]`` always. Summing link
    inertials onto the joint axis would compute the latter and overestimate the
    former -- unsafe in exactly the direction that matters, and worst for the
    distal joints of a serial arm.

    Measured rather than derived: a unit torque is applied at one DOF and the
    resulting acceleration differenced against a zero-torque step of the same
    state. Differencing cancels gravity, Coriolis, drive, and limit forces, so
    what remains is ``qdd = H^-1 tau``. The minimum over the sampled
    configurations is returned, because ``H`` depends on ``q`` and the smallest
    inertia gives the strictest bound. Those poses are drawn deterministically
    from the joint limits rather than from the loaded trajectory, so the bound
    is a property of the robot and cannot change between chunks of one solve.

    Args:
        wp: Imported ``warp`` module.
        model: Finalized Newton model.
        solver_cls: Featherstone solver class used for the probe.
        dof0: World-0 DOF index per mapped DOF.
        coord0: World-0 coordinate index per mapped DOF.
        default_joint_q: Rest coordinates the probe starts from. Taken from the
            context rather than the live model, whose coordinates hold
            whatever the previous rollout ended on.
        corners: Inertial corners to minimize over, so the bound stays valid
            for candidate mass, centre-of-mass, and inertia values.
        bodies_per_world: Body count of a single replicated world.

    Returns:
        Effective inertia per mapped DOF over the sampled configurations, and
        the derating that keeps the coupled modes inside the same bound.
    """
    num_mapped = int(dof0.shape[0])
    # The probe writes the model's inertials to reach each candidate corner and
    # steps throwaway states through it, so everything it touches has to be put
    # back before the rollout context reads its defaults.
    restore = {
        name: getattr(model, name).numpy().copy()
        for name in ("joint_q", "joint_qd", "body_q", "body_qd", "body_mass", "body_com", "body_inertia")
        if getattr(model, name, None) is not None
    }
    default_q = np.asarray(default_joint_q, dtype=np.float32).copy()
    tau = np.zeros(int(model.joint_dof_count), dtype=np.float32)
    # Small enough that the measured qd/dt is the instantaneous acceleration,
    # large enough to stay well clear of float32 cancellation.
    probe_dt = 1.0e-5

    def accelerations(
        solver: Any, states: tuple[Any, Any, Any], torque: np.ndarray, joint_q: np.ndarray
    ) -> np.ndarray:  # noqa: ANN401
        """Return the joint accelerations one torque produces at one pose.

        Args:
            solver: Solver built against the inertials being probed.
            states: Input state, output state, and control to step with.
            torque: Applied joint torque, indexed by model DOF.
            joint_q: Joint coordinates to evaluate at.

        Returns:
            Acceleration per model DOF, from rest.
        """
        state_in, state_out, control = states
        state_in.joint_q.assign(joint_q)
        state_in.joint_qd.zero_()
        state_out.joint_qd.zero_()
        control.joint_f.assign(torque)
        solver.step(state_in, state_out, control=control, contacts=None, dt=probe_dt)
        return np.asarray(state_out.joint_qd.numpy(), dtype=np.float64) / probe_dt

    inertia = np.full(num_mapped, np.inf, dtype=np.float64)
    coupling = 1.0
    try:
        for corner in corners:
            # Every corner starts from the baseline rather than the previous
            # corner, so the scalings never compound.
            for name in ("body_mass", "body_com", "body_inertia"):
                getattr(model, name).assign(restore[name])
            if not corner.identity():
                _apply_worst_case_inertials(model, corner, bodies_per_world)
            # The solver reads the inertials when it is built, so it has to be
            # rebuilt per corner; mutating the model under a live solver is
            # silently ignored. Built with the rollout's own settings so the
            # measured inertia describes the solver that has to integrate it.
            solver = solver_cls(model, angular_damping=0.0, update_mass_matrix_interval=1)
            states = (model.state(), model.state(), model.control())
            for sample in _inertia_probe_poses(model, dof0, coord0, default_q):
                joint_q = default_q.copy()
                joint_q[coord0] = np.asarray(sample, dtype=np.float32)
                baseline = accelerations(solver, states, np.zeros_like(tau), joint_q)
                # Column j is the mapped part of `H^-1 e_j`, so the assembled
                # matrix is the mapped block of the inverse mass matrix. Torque
                # only ever enters on mapped DOFs, which is what makes that
                # block the object governing the coupled modes.
                inverse_mass = np.zeros((num_mapped, num_mapped), dtype=np.float64)
                for index in range(num_mapped):
                    probe = tau.copy()
                    probe[int(dof0[index])] = 1.0
                    response = accelerations(solver, states, probe, joint_q)
                    inverse_mass[:, index] = response[dof0] - baseline[dof0]
                # The diagonal is (H^-1)[i, i]; non-positive means the probe
                # told us nothing here, though another sample may still.
                diagonal = np.diag(inverse_mass).copy()
                if not np.all(diagonal > 0.0):
                    continue
                inertia = np.minimum(inertia, 1.0 / diagonal)
                coupling = min(coupling, _coupling_derating(inverse_mass, diagonal))
    finally:
        for name, values in restore.items():
            getattr(model, name).assign(values)
    if not np.all(np.isfinite(inertia)):
        raise SysIdEnvironmentBridgeError(
            "Could not measure the effective inertia of identified joints "
            f"{np.flatnonzero(~np.isfinite(inertia)).tolist()}; the Featherstone model may be degenerate "
            "(zero mass or inertia on a driven link). Author link inertials or physxJoint:armature, "
            f"or {_FALLBACK_HINT}."
        )
    return inertia, coupling


@dataclass(frozen=True)
class _WorstCaseInertials:
    """Lowest inertials a candidate can reach, per body of world 0.

    The slope budget is measured once, but link mass, centre of mass, and
    inertia are themselves identifiable. A candidate lighter than the baseline
    has less articulated inertia than the budget was sized against, so the
    stability bound it was chosen to satisfy no longer holds. Probing these
    values instead of the baselines keeps the budget valid across the whole
    search space rather than only at its starting point.

    Attributes:
        mass_scale: Multiplier on each body's baseline mass.
        inertia_scale: Multiplier on each body's baseline inertia tensor.
        com_offset: ``(bodies, 3)`` Cartesian metres added to each baseline
            centre of mass, matching how ``LINK_COM_OFFSET_*`` is decoded.
    """

    mass_scale: np.ndarray
    inertia_scale: np.ndarray
    com_offset: np.ndarray

    def identity(self) -> bool:
        """Return whether this corner leaves the baseline inertials untouched.

        Returns:
            True when nothing would be changed.
        """
        return (
            bool(np.all(self.mass_scale == 1.0))
            and bool(np.all(self.inertia_scale == 1.0))
            and bool(np.all(self.com_offset == 0.0))
        )


def _candidate_inertial_corners(
    param_entries: Sequence[SysIdParameterEntry],
    bodies_per_world: int,
) -> tuple[_WorstCaseInertials, ...]:
    """Return the inertial corners the slope budget must stay valid at.

    ``LINK_MASS`` and ``LINK_INERTIA_LOG_CHOLESKY`` scale the mass matrix
    monotonically, so for those only the lower bound has to be probed.
    ``LINK_COM_OFFSET_*`` does not: a joint's articulated inertia is a Schur
    complement of ``H``, which is not monotone in the lever arm, and shortening
    an offset was measured to *raise* the value on a two-link chain. The
    optimizer adds each axis's offset to the baseline centre of mass, so every
    identified axis is enumerated at both of its signed bounds, in every sign
    combination across axes, and the baseline is always included because an
    offset of zero is inside the search space. All bodies sharing an axis move
    together; per-body sign mixes are not enumerated.

    The minimum of a non-monotone function over a box need not sit at a corner,
    so this is corner sampling rather than a proof. It is strictly tighter than
    probing the baseline alone, which is reachable and so can never be skipped.

    ``LINK_INERTIA_LOG_CHOLESKY`` has no informative lower bound: it replaces
    the Cholesky factor outright, and its default range reaches a log-diagonal
    of -3 with off-diagonals free, admitting inertia tensors arbitrarily close
    to singular. The tensor is driven to zero, leaving the joint on its
    armature; a joint with no armature then fails the probe's finiteness check,
    which is the honest outcome.

    Args:
        param_entries: Parameters being identified this run, with link
            indices already translated to Newton world-0 bodies.
        bodies_per_world: Body count of a single replicated world.

    Returns:
        Inertial corners to probe, always starting with the baseline.
    """
    ones = np.ones(bodies_per_world, dtype=np.float64)
    no_offset = np.zeros((bodies_per_world, 3), dtype=np.float64)
    baseline = _WorstCaseInertials(ones.copy(), ones.copy(), no_offset.copy())

    def bodies_of(entry: SysIdParameterEntry) -> slice | list[int]:
        if entry.link_index == GLOBAL_LINK_INDEX:
            return slice(None)
        return [entry.link_index] if 0 <= entry.link_index < bodies_per_world else []

    mass_floor = ones.copy()
    inertia_floor = ones.copy()
    com_low = no_offset.copy()
    com_high = no_offset.copy()
    com_axes: set[int] = set()
    for entry in param_entries:
        target = bodies_of(entry)
        if entry.param_type == SysIdParameterType.LINK_MASS:
            minimum, _maximum = default_bounds_for_type(entry.param_type)
            mass_floor[target] = np.minimum(mass_floor[target], max(minimum, 0.0))
        elif entry.param_type in _COM_OFFSET_TYPES:
            axis = _COM_OFFSET_TYPES[entry.param_type]
            minimum, maximum = default_bounds_for_type(entry.param_type)
            com_low[target, axis] = np.minimum(com_low[target, axis], minimum)
            com_high[target, axis] = np.maximum(com_high[target, axis], maximum)
            if not isinstance(target, list) or target:
                com_axes.add(axis)
        elif entry.param_type == SysIdParameterType.LINK_INERTIA_LOG_CHOLESKY:
            inertia_floor[target] = 0.0

    mass_options = [ones] if np.all(mass_floor >= 1.0) else [ones, mass_floor]
    inertia_options = [ones] if np.all(inertia_floor >= 1.0) else [ones, inertia_floor]
    # Both signs per axis: lengthening an offset can lower the Schur complement
    # even though it raises the joint's own diagonal term.
    com_options = [no_offset]
    axes = sorted(com_axes)
    for sides in itertools.product((com_low, com_high), repeat=len(axes)):
        offset = no_offset.copy()
        for axis, side in zip(axes, sides):
            offset[:, axis] = side[:, axis]
        com_options.append(offset)

    corners = [
        _WorstCaseInertials(mass.copy(), inertia.copy(), com.copy())
        for mass in mass_options
        for inertia in inertia_options
        for com in com_options
    ]
    # Keep the baseline first so the common friction-only run probes exactly
    # once and the ordering of the reported bound is stable.
    return (baseline, *(corner for corner in corners if not _same_worst_case(corner, baseline)))


def _same_worst_case(left: _WorstCaseInertials, right: _WorstCaseInertials) -> bool:
    """Return whether two worst-case inertial sets would probe identically.

    Args:
        left: Previously probed worst case.
        right: Worst case the current rollout requires.

    Returns:
        True when every scaling matches.
    """
    return (
        np.array_equal(left.mass_scale, right.mass_scale)
        and np.array_equal(left.inertia_scale, right.inertia_scale)
        and np.array_equal(left.com_offset, right.com_offset)
    )


def _same_corners(
    left: tuple[_WorstCaseInertials, ...],
    right: tuple[_WorstCaseInertials, ...],
) -> bool:
    """Return whether two corner sets would probe identically.

    Args:
        left: Corners previously probed.
        right: Corners the current rollout requires.

    Returns:
        True when both sets hold the same corners in the same order.
    """
    return len(left) == len(right) and all(_same_worst_case(a, b) for a, b in zip(left, right))


def _apply_worst_case_inertials(
    model: Any,  # noqa: ANN401
    worst_case: _WorstCaseInertials,
    bodies_per_world: int,
) -> None:
    """Scale the model's inertials to the worst case, across every world.

    Args:
        model: Finalized Newton model, modified in place.
        worst_case: Scalings to apply.
        bodies_per_world: Body count of a single replicated world.
    """
    world_count = max(int(model.body_mass.shape[0]) // max(bodies_per_world, 1), 1)
    mass = np.asarray(model.body_mass.numpy(), dtype=np.float64)
    com = np.asarray(model.body_com.numpy(), dtype=np.float64).reshape(-1, 3)
    inertia = np.asarray(model.body_inertia.numpy(), dtype=np.float64).reshape(-1, 3, 3)

    mass_scale = np.tile(worst_case.mass_scale, world_count)
    inertia_scale = np.tile(worst_case.inertia_scale, world_count)
    offset = np.tile(np.asarray(worst_case.com_offset, dtype=np.float64).reshape(-1, 3), (world_count, 1))

    model.body_mass.assign((mass * mass_scale).astype(np.float32))
    model.body_inertia.assign((inertia * inertia_scale[:, None, None]).astype(np.float32))
    model.body_com.assign(np.ascontiguousarray(com + offset, dtype=np.float32))


def _require_representable_friction(
    ctx: _DiffRolloutContext,
    damping: torch.Tensor,
    friction: torch.Tensor,
    dt: float,
) -> None:
    """Reject candidates whose drive damping leaves no room for friction.

    The kernel spends the slope budget on the candidate's drive damping first
    and gives friction whatever is left. When damping alone reaches the budget
    there is nothing left, so ``eps`` floors at zero, ``tanh(0)`` makes the
    friction torque identically zero, and the reported friction gradient is
    exactly zero. The optimizer then sees a parameter with no influence on the
    loss, leaves it at its starting value, and the run reports it as
    identified. That is a silently wrong answer rather than a degraded one, so
    it has to fail instead of warn -- a joint this far outside the integrator's
    stability region cannot represent Coulomb friction at this ``sub_dt`` no
    matter what the optimizer proposes.

    Args:
        ctx: Rollout context holding the slope budget and substep count.
        damping: ``(rows, num_dof)`` candidate drive damping.
        friction: ``(rows, num_dof)`` candidate friction magnitude.
        dt: Command sample period.

    Raises:
        SysIdEnvironmentBridgeError: If any candidate would have its friction
            deleted rather than merely softened.
    """
    budget = ctx.friction_slope_budget_np
    if budget is None or not ctx.num_dof:
        return
    allowed = torch.as_tensor(budget, dtype=damping.dtype, device=damping.device)
    # Only a joint that is actually meant to carry friction is affected; where
    # friction is zero the floored scale changes nothing.
    starved = (damping.abs() >= allowed) & (friction.abs() > 0.0)
    if not bool(starved.any()):
        return

    per_dof = starved.any(dim=0)
    dofs = [int(ctx.dof_map_np[index]) for index in torch.nonzero(per_dof).flatten().tolist()]
    worst = int(torch.nonzero(per_dof).flatten()[0].item())
    inertia = ctx.mapped_inertia_np
    passive = float(ctx.passive_damping_np[worst]) if ctx.passive_damping_np is not None else 0.0
    substeps = ctx.substeps
    needed = _substeps_for_full_friction(
        float(budget[worst]),
        float(friction[:, worst].abs().max()),
        float(damping[:, worst].abs().max()),
        float(inertia[worst]) if inertia is not None else 0.0,
        ctx.mapped_coupling,
        dt,
        passive,
    )
    raise SysIdEnvironmentBridgeError(
        f"Differentiable Newton cannot represent joint friction on identified DOF(s) {dofs}: the "
        f"candidate drive damping ({float(damping[:, worst].abs().max()):g}) already reaches the whole "
        f"velocity-slope budget ({float(budget[worst]):g}, after passive joint damping {passive:g}) that "
        f"this model and sub_dt ({dt / max(1, substeps) * 1000:.3g} ms, featherstone_substeps={substeps}) "
        "can integrate, leaving none for friction. Its friction torque and gradient would both be zero, so the "
        f"parameter would be reported as identified without ever affecting the loss. Raise "
        f"featherstone_substeps to at least {needed}, author physxJoint:armature (reflected rotor "
        f"inertia) on the robot, lower the damping upper bound, or {_FALLBACK_HINT}."
    )


def _substeps_for_full_friction(
    budget: float,
    friction: float,
    damping: float,
    inertia: float,
    coupling: float,
    dt: float,
    passive: float = 0.0,
) -> int:
    """Return the substeps that would carry the requested friction scale.

    Inverts the whole chain the budget is built from -- articulated inertia,
    coupled-mode derating, and ``sub_dt`` -- rather than rescaling the current
    ``eps``, so the answer accounts for the derating the kernel actually
    applies. Omitting ``coupling`` here understates the count by ``1/coupling``,
    which on a long chain is a factor of three or more.

    Args:
        budget: Current slope budget for the joint.
        friction: Friction magnitude to make room for.
        damping: Drive damping competing for the same budget.
        inertia: Articulated inertia of the joint.
        coupling: Coupled-mode derating the budget carries.
        dt: Command sample period.
        passive: Model ``joint_damping`` spending the same budget.

    Returns:
        Substep count, at least 1.
    """
    required_slope = FRICTION_INV_VELOCITY_EPS * abs(friction) + abs(damping) + abs(passive)
    carrying_capacity = FRICTION_SLOPE_STABILITY_RATIO * max(coupling, 1e-12) * abs(inertia)
    if carrying_capacity <= 0.0:
        return 0
    return max(int(math.ceil(dt * required_slope / carrying_capacity)), 1)


def _coupling_derating(inverse_mass: np.ndarray, diagonal: np.ndarray) -> float:
    """Return the factor that pulls the coupled modes inside the scalar bound.

    Budgeting each DOF against its own articulated inertia bounds only the
    diagonal of ``H^-1 D``, which is not what the integrator propagates: the
    rollout evolves the coupled modes, whose growth is set by that matrix's
    eigenvalues. Normalizing by the articulated inertia makes every diagonal
    entry of the symmetrized matrix exactly 1, so its trace equals the DOF
    count and its largest eigenvalue can approach that. Three or more
    identified joints at saturating friction therefore breach the stability
    limit of 2 while every joint is still inside its own scalar budget.

    ``lambda_max(H^-1 D)`` is monotone in ``D`` (``H^-1`` is positive definite,
    so ``A^(1/2) (D' - D) A^(1/2)`` stays positive semi-definite), which makes
    the all-DOF-saturated corner the worst case and this one derating
    sufficient for every candidate under it. The factor is independent of
    ``sub_dt``, because scaling ``D`` scales the eigenvalues in proportion.

    Two limits are worth being explicit about. Only the mapped block is
    derated, while held joints add their own damping to the same ``D`` and
    couple through the same ``H``; :func:`_cap_held_joint_gains` bounds that
    contribution, but against armature rather than articulated inertia, and
    only for held joints that have armature. And the articulated inertia this
    normalizes by is a minimum over a sampled pose set, so a pose between
    samples can leave the derating slightly optimistic. Treat the result as a
    close bound rather than a proof, with :func:`_warn_nonfinite_rollout` as
    the backstop that names the binding joint when it is not enough.

    Args:
        inverse_mass: Mapped block of the inverse joint-space mass matrix.
        diagonal: Its diagonal, the reciprocal of the articulated inertia.

    Returns:
        Factor in ``(0, 1]`` to multiply the per-DOF slope budget by.
    """
    # The probe measures H^-1 column by column, so round-off leaves it slightly
    # asymmetric while `eigvalsh` assumes symmetry.
    symmetric = 0.5 * (inverse_mass + inverse_mass.T)
    scale = np.sqrt(1.0 / diagonal)
    worst = float(np.linalg.eigvalsh(symmetric * np.outer(scale, scale)).max())
    # Unit diagonal puts the largest eigenvalue at or above 1; at exactly 1 the
    # modes are uncoupled and the scalar budget already holds.
    return 1.0 / worst if worst > 1.0 else 1.0


def _damping_substeps_required(
    inertia: np.ndarray,
    damping: np.ndarray,
    passive: np.ndarray | None,
    coupling: float,
    dt: float,
) -> np.ndarray:
    """Return, per mapped DOF, the substeps that leave friction its budget share.

    Args:
        inertia: Lowest reachable articulated inertia per mapped DOF.
        damping: Authored drive damping per mapped DOF.
        passive: Model ``joint_damping`` per mapped DOF, if any.
        coupling: Coupled-mode derating the budget carries.
        dt: Command sample period.

    Returns:
        Fractional substep count per mapped DOF; ``inf`` where the inertia is
        not positive.
    """
    slope = np.abs(np.asarray(damping, dtype=np.float64))
    if passive is not None:
        slope = slope + np.abs(np.asarray(passive, dtype=np.float64))
    share = min(max(FRICTION_BUDGET_SHARE, 0.0), 1.0 - 1e-6)
    capacity = (
        FRICTION_SLOPE_STABILITY_RATIO * max(coupling, 1e-12) * (1.0 - share) * np.asarray(inertia, dtype=np.float64)
    )
    with np.errstate(divide="ignore", invalid="ignore"):
        return np.where(capacity > 0.0, dt * slope / np.maximum(capacity, 1e-300), np.inf)


def _integrable_substeps(
    inertia: np.ndarray,
    damping: np.ndarray,
    passive: np.ndarray | None,
    coupling: float,
    configured: int,
    dt: float,
) -> int:
    """Return the substep count a rollout at ``dt`` should integrate with.

    The probe reports the smallest articulated inertia the identified mass,
    COM, and inertia bounds can reach, and those bounds routinely reach
    inertias at which the authored drive damping alone exceeds the explicit
    integrator's stability region at the configured ``sub_dt``. Raising the
    inertia the budget is sized against would admit a total slope past that
    region, so the substep count is raised first, until the drive and passive
    damping leave friction :data:`FRICTION_BUDGET_SHARE` of the budget at that
    inertia. It never drops below the configured count and is capped at
    :data:`MAX_AUTO_FEATHERSTONE_SUBSTEPS` unless the configured count is
    already higher; :func:`_integrable_inertia` covers what the cap cannot.

    Args:
        inertia: Lowest reachable articulated inertia per mapped DOF.
        damping: Authored drive damping per mapped DOF.
        passive: Model ``joint_damping`` per mapped DOF, if any.
        coupling: Coupled-mode derating the budget carries.
        configured: ``featherstone_substeps`` from the run spec.
        dt: Command sample period.

    Returns:
        Substep count, at least ``configured``.
    """
    configured = max(1, int(configured))
    required = _damping_substeps_required(inertia, damping, passive, coupling, dt)
    if required.size == 0:
        return configured
    worst = float(np.max(required))
    needed = int(math.ceil(worst)) if math.isfinite(worst) else MAX_AUTO_FEATHERSTONE_SUBSTEPS
    return max(configured, min(needed, MAX_AUTO_FEATHERSTONE_SUBSTEPS))


def _integrable_inertia(
    probed: np.ndarray,
    damping: np.ndarray,
    passive: np.ndarray | None,
    coupling: float,
    substeps: int,
    dt: float,
) -> np.ndarray:
    """Floor the probed inertia where the substep count still cannot carry it.

    ``LINK_INERTIA_LOG_CHOLESKY`` admits near-singular inertia tensors, so the
    probe can report inertias at which no affordable substep count integrates
    even the authored damping. Sizing the budget there spends it all on
    damping and makes friction unrepresentable for every candidate, including
    the baseline. Past :func:`_integrable_substeps`, the floor is therefore the
    inertia at which the drive and passive damping still leave friction
    :data:`FRICTION_BUDGET_SHARE` of the budget at this ``sub_dt``.

    This is not a stability guarantee below the floor: a candidate that really
    reaches a lower inertia can still exceed the limit once friction is added,
    and is rejected only when its rollout or adjoint turns non-finite. Where the
    substep count suffices the floor is at or below the probe and changes
    nothing.

    Args:
        probed: Lowest reachable articulated inertia per mapped DOF.
        damping: Authored drive damping per mapped DOF.
        passive: Model ``joint_damping`` per mapped DOF, if any.
        coupling: Coupled-mode derating the budget carries.
        substeps: Substep count the rollout will use.
        dt: Command sample period.

    Returns:
        Inertia per mapped DOF to build the budget from.
    """
    slope = np.abs(np.asarray(damping, dtype=np.float64))
    if passive is not None:
        slope = slope + np.abs(np.asarray(passive, dtype=np.float64))
    share = min(max(FRICTION_BUDGET_SHARE, 0.0), 1.0 - 1e-6)
    sub_dt = max(dt / max(1, substeps), 1e-9)
    floor = slope * sub_dt / (FRICTION_SLOPE_STABILITY_RATIO * max(coupling, 1e-12) * (1.0 - share))
    return np.maximum(np.asarray(probed, dtype=np.float64), floor)


def _log_budget_sizing(
    substeps: int,
    configured: int,
    probed: np.ndarray,
    mapped: np.ndarray,
    damping: np.ndarray,
    passive: np.ndarray | None,
    dt: float,
    dof_labels: np.ndarray,
) -> None:
    """Report a raised substep count and any inertia the cap forced a floor on.

    Args:
        substeps: Substep count the rollout will use.
        configured: ``featherstone_substeps`` from the run spec.
        probed: Lowest reachable articulated inertia per mapped DOF.
        mapped: Inertia the budget is built from.
        damping: Authored drive damping per mapped DOF.
        passive: Model ``joint_damping`` per mapped DOF, if any.
        dt: Command sample period.
        dof_labels: Model DOF index per mapped DOF, for diagnostics.
    """
    floored = mapped > probed
    if substeps <= configured and not np.any(floored):
        return
    with np.errstate(divide="ignore", invalid="ignore"):
        lift = np.where(probed > 0.0, mapped / probed, np.inf)
    worst = int(np.argmax(lift)) if np.any(floored) else int(np.argmin(probed))
    passive_worst = float(passive[worst]) if passive is not None else 0.0
    message = (
        f"SysId differentiable bridge integrates with featherstone_substeps={substeps} "
        f"(configured {configured}, sub_dt={dt / substeps * 1000:.3g} ms) so drive and passive joint damping "
        f"leave friction {FRICTION_BUDGET_SHARE:g} of the stability budget. Binding is dof "
        f"{int(dof_labels[worst])}: drive damping {abs(float(damping[worst])):g} plus passive {passive_worst:g} "
        f"against the lowest effective inertia the identified parameters reach, {float(probed[worst]):g} kg m^2"
    )
    if substeps > configured:
        message += ". Rollouts cost proportionally more time and taped memory"
    if np.any(floored):
        message += (
            f". The {max(substeps, MAX_AUTO_FEATHERSTONE_SUBSTEPS)}-substep limit cannot carry that, so the "
            f"budget on {int(np.count_nonzero(floored))} DOF(s) is sized at {float(mapped[worst]):g} kg m^2 instead: "
            "a candidate whose inertia really falls below it may diverge and fail the finite-gradient check. "
            "Set featherstone_substeps higher, or author physxJoint:armature (reflected rotor inertia) on the robot"
        )
    carb.log_warn(message + ".")


def _friction_slope_budget(
    inertia: np.ndarray,
    coupling: float,
    substeps: int,
    dt: float,
    passive: np.ndarray | None = None,
) -> np.ndarray:
    """Return the velocity-proportional slope the kernel may spend per mapped DOF.

    Symplectic Euler on a damping-dominated joint compounds
    ``|1 - slope * sub_dt / inertia|`` once per substep, so the rollout is
    stable while that ratio stays under 2. The total slope allowed by
    :data:`FRICTION_SLOPE_STABILITY_RATIO` is shared by the model's passive
    ``joint_damping``, which Featherstone applies outside the kernel, and the
    drive damping plus friction linearization the kernel applies. This returns
    what remains after the passive share.

    The budget deliberately depends only on the model and the timestep, never
    on the candidate. The kernel divides it by the candidate friction to get a
    per-step ``eps``, which keeps that dependency on the tape and the reported
    friction gradient consistent with the torque actually applied.

    Args:
        inertia: Effective inertia per mapped DOF.
        coupling: Derating that keeps the coupled modes inside the same bound.
        substeps: Symplectic-Euler substeps per command sample.
        dt: Command sample period.
        passive: Model ``joint_damping`` per mapped DOF, if any.

    Returns:
        Allowed kernel velocity-proportional slope per mapped DOF.
    """
    sub_dt = max(dt / max(1, substeps), 1e-9)
    total = FRICTION_SLOPE_STABILITY_RATIO * coupling * np.asarray(inertia, dtype=np.float64) / sub_dt
    if passive is None:
        return total
    return np.maximum(total - np.abs(np.asarray(passive, dtype=np.float64)), 0.0)


def _warn_softened_friction_knee(
    budget: np.ndarray,
    inertia: np.ndarray,
    baseline_friction: np.ndarray,
    baseline_damping: np.ndarray,
    peak_velocity: float,
    coupling: float,
    substeps: int,
    dt: float,
    dof_labels: np.ndarray,
    passive: np.ndarray | None = None,
) -> None:
    """Warn when the stability budget softens the Coulomb knee at the baseline.

    Reported against the baseline candidate, which is the model the user
    authored; the optimizer exploring upward softens the knee further, and the
    kernel keeps that stable on its own.

    Args:
        budget: Allowed total slope per mapped DOF.
        inertia: Effective inertia per mapped DOF.
        baseline_friction: Baseline friction torque magnitude per mapped DOF.
        baseline_damping: Baseline drive damping per mapped DOF.
        peak_velocity: Largest joint speed the trajectory commands.
        coupling: Coupled-mode derating the budget carries.
        substeps: Symplectic-Euler substeps per command sample.
        dt: Command sample period.
        dof_labels: Model DOF index per mapped DOF, for diagnostics.
        passive: Model ``joint_damping`` per mapped DOF, already taken out of
            ``budget``.
    """
    friction = np.abs(np.asarray(baseline_friction, dtype=np.float64))
    damping = np.abs(np.asarray(baseline_damping, dtype=np.float64))
    headroom = np.maximum(budget - damping, 0.0)
    with np.errstate(divide="ignore", invalid="ignore"):
        eps = np.where(friction > 0.0, headroom / np.maximum(friction, 1e-12), np.inf)
    eps = np.minimum(eps, FRICTION_INV_VELOCITY_EPS)

    softened = eps < FRICTION_INV_VELOCITY_EPS - 1e-9
    if not np.any(softened):
        return
    worst = int(np.argmin(np.where(softened, eps, np.inf)))
    knee = 1.0 / max(float(eps[worst]), 1e-12)
    needed = _substeps_for_full_friction(
        float(budget[worst]),
        friction[worst],
        damping[worst],
        float(inertia[worst]),
        coupling,
        dt,
        float(passive[worst]) if passive is not None else 0.0,
    )
    message = (
        f"SysId differentiable bridge softened the smooth-friction knee on "
        f"{int(np.count_nonzero(softened))} of {int(eps.shape[0])} identified DOF(s) to keep the taped "
        f"reverse pass stable (sub_dt={dt / max(1, substeps) * 1000:.3g} ms, currently "
        f"featherstone_substeps={substeps}). Worst is dof {int(dof_labels[worst])}: at the authored "
        f"friction {friction[worst]:g} and damping {damping[worst]:g} against an effective inertia of "
        f"{inertia[worst]:g} kg m^2, the velocity scale falls from {FRICTION_INV_VELOCITY_EPS:g} to "
        f"{eps[worst]:g}, so friction behaves viscously below |qd| ~ {knee:.3g} rad/s"
    )
    if peak_velocity > 0.0:
        message += f" against a commanded peak of {peak_velocity:.3g} rad/s"
    message += (
        f". Identified friction will be biased low. Raise featherstone_substeps to about {needed}, or "
        "author physxJoint:armature (reflected rotor inertia) on the robot, to recover the Coulomb knee."
    )
    # A knee the trajectory never reaches means friction was refit as viscous
    # damping, which biases the result the user will act on.
    degraded = peak_velocity <= 0.0 or knee >= peak_velocity
    (carb.log_warn if degraded else carb.log_info)(message)


def _cap_held_joint_gains(ctx: _DiffRolloutContext, ke_full: np.ndarray, kd_full: np.ndarray, dt: float) -> None:
    """Clamp held-joint drive gains into the explicit-integrator stability region.

    Actuated joints outside the trajectory keep their imported drive gains, which
    on real robots are routinely far stiffer than symplectic Euler can integrate at
    ``sub_dt`` (e.g. Franka gripper drives author ``ke = 1e6``, giving
    ``omega * sub_dt approx 4.7`` against the stability bound of 2). The FORWARD
    rollout still looks fine because joint limits saturate the unstable mode, but
    the tape's reverse pass propagates the unsaturated linearization, so the
    adjoint grows exponentially per substep and long-horizon gradients turn NaN.
    Capping held joints at ``omega * sub_dt <= 1`` and ``kd * sub_dt / m <= 0.5``
    (4x margin) removes the unstable mode at the source. Identified (mapped)
    joints are never modified; their stability is the user's parameter bounds'
    responsibility and is diagnosed by :func:`_warn_nonfinite_rollout`.

    Args:
        ctx: Value supplied for ``ctx``.
        ke_full: Value supplied for ``ke_full``.
        kd_full: Value supplied for ``kd_full``.
        dt: Value supplied for ``dt``.
    """
    held = ctx.held_dofs_np
    if held.size == 0:
        return
    armature_arr = getattr(ctx.model, "joint_armature", None)
    if armature_arr is None:
        return
    sub_dt = max(dt / max(1, ctx.substeps), 1e-9)
    m_eff = armature_arr.numpy()[held]
    capable = m_eff > 0.0
    ke_cap = 0.25 * m_eff / (sub_dt * sub_dt)
    kd_cap = 0.5 * m_eff / sub_dt
    ke_held = ke_full[held]
    kd_held = kd_full[held]
    capped = capable & ((ke_held > ke_cap) | (kd_held > kd_cap))
    if not np.any(capped):
        return
    ke_full[held] = np.where(capable, np.minimum(ke_held, ke_cap), ke_held)
    kd_full[held] = np.where(capable, np.minimum(kd_held, kd_cap), kd_held)
    if not ctx.held_gain_cap_logged:
        ctx.held_gain_cap_logged = True
        examples = [
            f"dof {int(dof)}: ke {ke_held[i]:g}->{ke_full[held][i]:g}, kd {kd_held[i]:g}->{kd_full[held][i]:g}"
            for i, dof in enumerate(held)
            if capped[i]
        ][:4]
        carb.log_info(
            f"SysId differentiable bridge capped {int(np.count_nonzero(capped))} held-joint drive gain(s) to the "
            f"explicit-integrator stability region (sub_dt={sub_dt * 1000.0:.3g} ms): {'; '.join(examples)}. "
            "Held joints are not identified; this only affects how firmly they hold their default pose."
        )


def _warn_nonfinite_rollout(ctx: _DiffRolloutContext, dt: float, stage: str = "states") -> None:
    """Explain an exploded rollout or adjoint with a stability estimate.

    Symplectic Euler on a damping-dominated joint is stable only for
    ``sub_dt < 2 * effective_inertia / kd_total``. Mapped joints are driven by
    the bridge's own kernel, whose ``kd_total`` includes the smooth-friction
    linearization ``friction * eps`` alongside the drive damping; held joints
    keep their solver drives against joint armature.

    Args:
        ctx: Value supplied for ``ctx``.
        dt: Value supplied for ``dt``.
        stage: Which pass produced the non-finite values, for the message.
    """
    sub_dt = dt / max(1, ctx.substeps)
    message = [
        f"Differentiable Newton rollout produced non-finite {stage} (sub_dt={sub_dt * 1000:.2f} ms, "
        f"substeps={ctx.substeps})."
    ]

    def describe(label: str, kd_total: float, inertia: float, coupling: float, extra: str = "") -> bool:
        """Report a joint whose damping exceeds what this sub_dt can integrate.

        Args:
            label: Joint category named in the message.
            kd_total: Total velocity-proportional slope on the joint.
            inertia: Effective inertia resisting that slope.
            coupling: Derating for this category's coupled modes. Identified
                joints share a measured mass matrix; held joints are bounded
                against their own armature, so they carry no derating.
            extra: Parenthetical breakdown appended to the damping figure.

        Returns:
            Whether the joint was reported as the binding constraint.
        """
        if kd_total <= 0.0:
            return False
        if inertia <= 0.0:
            message.append(
                f"Worst {label} joint has effective damping {kd_total:g}{extra} with zero effective inertia: "
                "the explicit integrator cannot be stabilized by substeps alone. Author physxJoint:armature "
                "(reflected rotor inertia, e.g. 0.05-0.2 for industrial arms) on the robot's joints, "
                "then size featherstone_substeps so sub_dt < 2*armature/damping."
            )
            return True
        # The coupled modes breach the limit before any single joint does, so
        # the same derating the budget carries applies to this threshold.
        stable_sub_dt = 2.0 * coupling * inertia / kd_total
        # Above this the joint's damping is already inside the stability
        # region, so naming it would send the user after the wrong parameter.
        if stable_sub_dt > sub_dt:
            return False
        message.append(
            f"Worst {label} joint has effective damping {kd_total:g}{extra} against effective inertia "
            f"{inertia:g}; stability needs sub_dt < {stable_sub_dt * 1000:.2f} ms "
            f"(featherstone_substeps >= {int(math.ceil(dt / stable_sub_dt))}), or raise that joint's "
            "armature (reflected rotor inertia) in the USD."
        )
        return True

    explained = False
    try:
        # Mapped joints: the bridge kernel supplies PD plus smooth friction, so
        # the solver's own drive gains on these DOFs were zeroed at write time.
        inertia = ctx.mapped_inertia_np
        budget = ctx.friction_slope_budget_np
        if inertia is not None and budget is not None and ctx.num_dof:
            kd_kernel = np.abs(ctx.damping_wp.numpy().reshape(ctx.world_count, ctx.num_dof)).max(axis=0)
            friction = np.abs(ctx.friction_wp.numpy().reshape(ctx.world_count, ctx.num_dof)).max(axis=0)
            # Mirror the kernel's cap so the reported slope is the one that was
            # actually applied, not the uncapped `friction * eps`.
            with np.errstate(divide="ignore", invalid="ignore"):
                capped = np.maximum(budget - kd_kernel, 0.0) / np.maximum(friction, _FRICTION_SLOPE_MIN_DENOM)
            eps = np.minimum(FRICTION_INV_VELOCITY_EPS, capped)
            passive = ctx.passive_damping_np if ctx.passive_damping_np is not None else np.zeros_like(kd_kernel)
            kd_total = passive + kd_kernel + friction * eps
            with np.errstate(divide="ignore", invalid="ignore"):
                ratio = np.where(inertia > 0.0, kd_total / np.maximum(inertia, 1e-12), np.inf)
            worst = int(np.argmax(ratio))
            explained |= describe(
                "identified",
                float(kd_total[worst]),
                float(inertia[worst]),
                max(ctx.mapped_coupling, 1e-12),
                f" (passive {passive[worst]:g} plus drive {kd_kernel[worst]:g} plus friction "
                f"{friction[worst]:g} * eps {eps[worst]:g})",
            )
    except Exception:  # diagnostics must never mask the original problem
        carb.log_verbose(f"SysId: identified-joint stability estimate failed.\n{traceback.format_exc()}")

    try:
        # Held joints keep Newton's USD-import drive conversion.
        dofs = ctx.held_dofs_np
        if dofs.size:
            kd = ctx.model.joint_target_kd.numpy()[dofs]
            armature = ctx.model.joint_armature.numpy()[dofs]
            with np.errstate(divide="ignore", invalid="ignore"):
                ratio = np.where(armature > 0.0, kd / np.maximum(armature, 1e-12), np.inf)
            worst = int(np.argmax(ratio))
            explained |= describe("held", float(kd[worst]), float(armature[worst]), 1.0)
    except Exception:  # diagnostics must never mask the original problem
        carb.log_verbose(f"SysId: held-joint stability estimate failed.\n{traceback.format_exc()}")

    if not explained:
        # Every damping term is inside its stability limit, so the divergence
        # came from a term this estimate does not cover -- most often drive
        # stiffness against a command rate too coarse for it.
        message.append(
            "Damping on identified and held joints is within its stability limit at this sub_dt, so "
            "suspect drive stiffness or the command sample rate: raise featherstone_substeps, lower the "
            f"identified stiffness bounds, or {_FALLBACK_HINT}."
        )
    carb.log_warn("SysId: " + " ".join(message))


def _require_finite_array(values: np.ndarray, description: str) -> np.ndarray:
    """Return an optimizer-facing array after verifying it is finite.

    Args:
        values: Array to validate.
        description: Array description used in validation errors.

    Returns:
        Validated input array.

    Raises:
        SysIdEnvironmentBridgeError: If the array contains a non-finite value.
    """
    if not np.all(np.isfinite(values)):
        raise SysIdEnvironmentBridgeError(f"Differentiable Newton {description} contains non-finite values.")
    return values


def _record_rollout_launches(
    bridge: NewtonDifferentiableSysIdBridge,
    ctx: _DiffRolloutContext,
    num_steps: int,
    sub_dt: float,
) -> None:
    """Dispatch the full rollout launch sequence (taped and/or graph-captured).

    Each substep owns a distinct Featherstone solver workspace. Its spatial
    inertia, COM transforms, and mass matrix are rebuilt on tape at that
    substep's configuration, preserving both standard Featherstone forward
    dynamics and the primal values needed by the reverse pass. Per-substep
    targets are copied from persistent staging arrays so command and feedforward
    contents are not baked into captured graphs.

    Args:
        bridge: Value supplied for ``bridge``.
        ctx: Value supplied for ``ctx``.
        num_steps: Value supplied for ``num_steps``.
        sub_dt: Value supplied for ``sub_dt``.
    """
    wp = bridge._modules.warp
    model = ctx.model
    torque_kernel, gather_kernel, target_kernel = _load_diff_kernels(wp)
    aux = bridge._featherstone_kernels
    world_count, num_dof = ctx.world_count, ctx.num_dof
    joint_dof_count = int(model.joint_dof_count)

    wp.launch(
        gather_kernel,
        dim=world_count * num_dof,
        inputs=[
            ctx.states[0].joint_q,
            ctx.states[0].joint_qd,
            ctx.coord_index_map,
            ctx.dof_index_map,
            0,
        ],
        outputs=[ctx.sim_q, ctx.sim_qd],
        device=model.device,
    )
    # Note: `solver.step` writes each state's body_q twice on the tape
    # (step k-1's post-step FK, then step k's pre-step eval_rigid_fk
    # recomputing the same poses). This is adjoint-safe: warp's
    # adj_array_store consumes AND zeroes the stored adjoint, so only
    # the last writer propagates it.
    for k in range((num_steps - 1) * ctx.substeps):
        state_in = ctx.states[k]
        state_out = ctx.states[k + 1]
        control = ctx.controls[k]
        solver = ctx.solvers[k]
        state_in.clear_forces()
        wp.launch(
            aux.compute_spatial_inertia,
            int(model.body_count),
            inputs=[model.body_inertia, model.body_mass],
            outputs=[solver.body_I_m],
            device=model.device,
        )
        wp.launch(
            aux.compute_com_transforms,
            int(model.body_count),
            inputs=[model.body_com],
            outputs=[solver.body_X_com],
            device=model.device,
        )
        # Target staging is constant; Warp consumes the Control-store adjoint.
        wp.launch(
            target_kernel,
            dim=joint_dof_count,
            inputs=[
                ctx.target_pos_stage,
                ctx.target_vel_stage,
                k // ctx.substeps,
            ],
            outputs=[control.joint_target_q, control.joint_target_qd],
            device=model.device,
        )
        wp.launch(
            torque_kernel,
            dim=world_count * num_dof,
            inputs=[
                state_in.joint_q,
                state_in.joint_qd,
                ctx.coord_index_map,
                ctx.dof_index_map,
                ctx.target_pos_stage,
                ctx.target_vel_stage,
                k // ctx.substeps,
                ctx.stiffness_wp,
                ctx.damping_wp,
                ctx.friction_wp,
                FRICTION_INV_VELOCITY_EPS,
                ctx.friction_slope_budget_wp,
                ctx.gravity_ff_wp,
                (k // ctx.substeps) if ctx.gravity_ff_active else 0,
                ctx.effort_limit_wp,
                1 if ctx.clamp_effort else 0,
                (k // ctx.substeps) if k % ctx.substeps == 0 else -1,
            ],
            outputs=[control.joint_f, ctx.sim_tau],
            device=model.device,
        )
        solver.step(state_in, state_out, control, None, sub_dt)
        if (k + 1) % ctx.substeps == 0:
            wp.launch(
                gather_kernel,
                dim=world_count * num_dof,
                inputs=[
                    state_out.joint_q,
                    state_out.joint_qd,
                    ctx.coord_index_map,
                    ctx.dof_index_map,
                    (k + 1) // ctx.substeps,
                ],
                outputs=[ctx.sim_q, ctx.sim_qd],
                device=model.device,
            )
    final_state = ctx.states[(num_steps - 1) * ctx.substeps]
    wp.launch(
        torque_kernel,
        dim=world_count * num_dof,
        inputs=[
            final_state.joint_q,
            final_state.joint_qd,
            ctx.coord_index_map,
            ctx.dof_index_map,
            ctx.target_pos_stage,
            ctx.target_vel_stage,
            num_steps - 1,
            ctx.stiffness_wp,
            ctx.damping_wp,
            ctx.friction_wp,
            FRICTION_INV_VELOCITY_EPS,
            ctx.friction_slope_budget_wp,
            ctx.gravity_ff_wp,
            (num_steps - 1) if ctx.gravity_ff_active else 0,
            ctx.effort_limit_wp,
            1 if ctx.clamp_effort else 0,
            num_steps - 1,
        ],
        outputs=[ctx.final_joint_f, ctx.sim_tau],
        device=model.device,
    )


def _capture_rollout_graphs(
    bridge: NewtonDifferentiableSysIdBridge,
    ctx: _DiffRolloutContext,
    num_steps: int,
    sub_dt: float,
    capture_key: tuple,
) -> _CaptureRecord | None:
    """Capture forward, backward, and adjoint-zeroing graphs for one rollout shape.

    A prior uncaptured rollout compiles kernels and allocates solver buffers so
    stream capture remains allocation-free. Failures latch the uncaptured path.

    Args:
        bridge: Value supplied for ``bridge``.
        ctx: Value supplied for ``ctx``.
        num_steps: Value supplied for ``num_steps``.
        sub_dt: Value supplied for ``sub_dt``.
        capture_key: Value supplied for ``capture_key``.

    Returns:
        Result produced by the operation.
    """
    wp = bridge._modules.warp
    device = ctx.model.device
    start = time.perf_counter()
    try:
        tape = wp.Tape()
        with wp.ScopedCapture(device) as forward_capture:
            with tape:
                _record_rollout_launches(bridge, ctx, num_steps, sub_dt)
        seed_q = wp.zeros(ctx.sim_q.shape, dtype=wp.float32, device=device)
        seed_qd = wp.zeros(ctx.sim_qd.shape, dtype=wp.float32, device=device)
        seed_tau = wp.zeros(ctx.sim_tau.shape, dtype=wp.float32, device=device)
        with wp.ScopedCapture(device) as backward_capture:
            wp.copy(ctx.sim_q.grad, seed_q)
            wp.copy(ctx.sim_qd.grad, seed_qd)
            wp.copy(ctx.sim_tau.grad, seed_tau)
            tape.backward()
        with wp.ScopedCapture(device) as zero_capture:
            tape.zero()
        record = _CaptureRecord(
            tape=tape,
            graph_forward=forward_capture.graph,
            graph_backward=backward_capture.graph,
            graph_zero=zero_capture.graph,
            seed_q=seed_q,
            seed_qd=seed_qd,
            seed_tau=seed_tau,
        )
        # Prime graph executables inside the fallback handler. Zero seeds make
        # the backward/zero launches harmless, and forward replay is idempotent.
        wp.capture_launch(record.graph_backward)
        wp.capture_launch(record.graph_zero)
        wp.capture_launch(record.graph_forward)
        wp.synchronize_device(device)
    except Exception as exc:  # noqa: BLE001 - any capture failure must fall back
        ctx.capture_failed = True
        carb.log_warn(
            f"SysId: CUDA graph capture failed for the differentiable rollout "
            f"(steps={num_steps}, substeps={ctx.substeps}, worlds={ctx.world_count}): {exc}. "
            "Falling back to uncaptured rollouts for this model."
        )
        return None
    # Graphs hold one node per captured launch; bound the per-model cache so
    # alternating rollout shapes cannot accumulate unbounded GPU graph memory.
    while len(ctx.capture_cache) >= 4:
        ctx.capture_cache.pop(next(iter(ctx.capture_cache)))
    ctx.capture_cache[capture_key] = record
    carb.log_info(
        f"SysId: captured differentiable rollout into CUDA graphs in {time.perf_counter() - start:.1f} s "
        f"(steps={num_steps}, substeps={ctx.substeps}, worlds={ctx.world_count}, "
        f"launches={len(tape.launches)})."
    )
    return record


class _NewtonDiffRollout(torch.autograd.Function):
    """Taped Featherstone rollout exposed as a torch autograd node.

    ``forward`` writes the decoded parameter values into the replicated model, records
    the whole rollout on a ``wp.Tape``, and returns cloned position/velocity tensors.
    ``backward`` seeds the gather buffers' adjoints from ``grad_outputs``, replays the
    tape, and reads the accumulated parameter adjoints back into torch tensors.
    """

    @staticmethod
    def forward(  # noqa: PLR0915 - the taped rollout is one deliberate sequence
        ctx_ag,  # noqa: ANN001
        ke: torch.Tensor,
        kd: torch.Tensor,
        friction: torch.Tensor,
        mass: torch.Tensor,
        com: torch.Tensor,
        inertia: torch.Tensor,
        bridge: NewtonDifferentiableSysIdBridge,
        ctx: _DiffRolloutContext,
        q0: np.ndarray,
        dq0: np.ndarray,
        commands: np.ndarray,
        num_steps: int,
        dt: float,
    ) -> tuple[torch.Tensor, torch.Tensor, torch.Tensor]:
        wp = bridge._modules.warp
        model = ctx.model
        world_count, num_dof = ctx.world_count, ctx.num_dof

        # Warp accumulates adjoints across backward passes.
        for arr in (
            ctx.stiffness_wp,
            ctx.damping_wp,
            model.body_mass,
            model.body_com,
            model.body_inertia,
            ctx.friction_wp,
            ctx.sim_q,
            ctx.sim_qd,
            ctx.sim_tau,
        ):
            if arr is not None and arr.grad is not None:
                arr.grad.zero_()

        ke_full = ctx.default_target_ke.copy()
        kd_full = ctx.default_target_kd.copy()
        # The custom kernel drives mapped joints; solver drives retain held joints.
        ke_full[ctx.dof_map_np] = 0.0
        kd_full[ctx.dof_map_np] = 0.0
        _cap_held_joint_gains(ctx, ke_full, kd_full, float(dt))
        model.joint_target_ke.assign(ke_full)
        model.joint_target_kd.assign(kd_full)
        ctx.stiffness_wp.assign(ke.detach().cpu().numpy().reshape(-1).astype(np.float32))
        ctx.damping_wp.assign(kd.detach().cpu().numpy().reshape(-1).astype(np.float32))
        ctx.friction_wp.assign(friction.detach().cpu().numpy().reshape(-1).astype(np.float32))
        model.body_mass.assign(mass.detach().cpu().numpy().reshape(-1).astype(np.float32))
        model.body_com.assign(np.ascontiguousarray(com.detach().cpu().numpy().reshape(-1, 3), dtype=np.float32))
        model.body_inertia.assign(
            np.ascontiguousarray(inertia.detach().cpu().numpy().reshape(-1, 3, 3), dtype=np.float32)
        )
        # Stage commands once per chunk; the tape scatters them per substep.
        # The final command row contributes only to the reported torque row.
        velocity_targets_active = ctx.gravity_ff_active and ctx.velocity_targets_np is not None
        # Include the feedforward source to disambiguate identical command chunks.
        fingerprint = (
            num_steps,
            hash(commands[: max(0, num_steps - 1)].tobytes()),
            velocity_targets_active,
            ctx.gravity_ff_source,
        )
        if ctx.commands_fingerprint != fingerprint:
            joint_dof_count = int(model.joint_dof_count)
            stage_rows = int(ctx.target_pos_stage.shape[0])
            drive_rows = max(0, num_steps - 1)
            pos_stage = np.broadcast_to(
                ctx.default_joint_target.astype(np.float32),
                (stage_rows, joint_dof_count),
            ).copy()
            pos_stage[:drive_rows][:, ctx.dof_map_np] = np.tile(commands[:drive_rows], (1, world_count))
            vel_stage = np.zeros((stage_rows, joint_dof_count), dtype=np.float32)
            if velocity_targets_active:
                vel_stage[:drive_rows][:, ctx.dof_map_np] = np.tile(
                    ctx.velocity_targets_np[:drive_rows], (1, world_count)
                )
            ctx.target_pos_stage.assign(pos_stage)
            ctx.target_vel_stage.assign(vel_stage)
            ctx.commands_fingerprint = fingerprint

        # Initial state is measured data, not an optimized tape input.
        q_full = ctx.default_joint_q.copy()
        q_full[ctx.coord_map_np] = np.tile(q0, world_count)
        qd_full = ctx.default_joint_qd.copy()
        qd_full[ctx.dof_map_np] = np.tile(dq0, world_count)
        ctx.states[0].joint_q.assign(q_full)
        ctx.states[0].joint_qd.assign(qd_full)

        # Capture begins only after a plain rollout has allocated solver buffers.
        sub_dt = float(dt) / ctx.substeps
        capture_key = (
            num_steps,
            float(dt),
            ctx.substeps,
            bool(ctx.gravity_ff_active),
            id(ctx.gravity_ff_wp),
            id(ctx.target_pos_stage),
            id(ctx.target_vel_stage),
        )
        total_substeps = (num_steps - 1) * ctx.substeps
        use_capture = num_steps >= 2 and bridge._capture_supported(ctx)
        record = ctx.capture_cache.get(capture_key) if use_capture else None
        if record is None and use_capture and total_substeps <= ctx.plain_max_substeps:
            estimated_launches = int(ctx.plain_launches_per_substep * total_substeps)
            budget = _capture_max_launches()
            if estimated_launches > budget:
                if not ctx.capture_budget_logged:
                    ctx.capture_budget_logged = True
                    carb.log_warn(
                        f"SysId: skipping CUDA graph capture (~{estimated_launches} launches per rollout "
                        f"exceeds the {budget} budget; very large graphs can fail driver instantiation). "
                        "Rollouts stay uncaptured. Shorter multiple-shooting chunks or fewer "
                        "featherstone_substeps re-enable capture; ISAACSIM_SYSID_CAPTURE_MAX_LAUNCHES "
                        "overrides the budget on "
                        "larger GPUs."
                    )
            else:
                record = _capture_rollout_graphs(bridge, ctx, num_steps, sub_dt, capture_key)

        tape = None
        forward_start = time.perf_counter()
        if record is not None:
            wp.capture_launch(record.graph_forward)
        else:
            tape = wp.Tape()
            with tape:
                _record_rollout_launches(bridge, ctx, num_steps, sub_dt)

        ctx_ag.tape = tape
        ctx_ag.capture_record = record
        ctx_ag.capture_key = capture_key
        ctx_ag.rollout_ctx = ctx
        ctx_ag.bridge = bridge
        ctx_ag.num_steps = num_steps
        ctx_ag.dt = float(dt)
        ctx_ag.out_device = ke.device
        ctx_ag.out_dtype = ke.dtype

        def read_output(arr) -> torch.Tensor:  # noqa: ANN001
            values = np.asarray(arr.numpy(), dtype=np.float32)[:num_steps]
            tensor = torch.as_tensor(values, device=ke.device).clone()
            return tensor.reshape(num_steps, world_count, num_dof).permute(1, 0, 2).contiguous()

        positions = read_output(ctx.sim_q)
        velocities = read_output(ctx.sim_qd)
        torques = read_output(ctx.sim_tau)
        forward_elapsed = time.perf_counter() - forward_start
        if record is not None:
            if not record.replay_logged:
                record.replay_logged = True
                plain_s = ctx.plain_forward_s.get(capture_key)
                before = f"{plain_s:.2f} s" if plain_s is not None else "n/a"
                carb.log_info(
                    f"SysId: CUDA graph replay active (steps={num_steps}, substeps={ctx.substeps}, "
                    f"worlds={world_count}): forward rollout {before} -> {forward_elapsed:.2f} s."
                )
        elif use_capture:
            ctx.plain_max_substeps = max(ctx.plain_max_substeps, total_substeps)
            if total_substeps > 0:
                ctx.plain_launches_per_substep = len(tape.launches) / total_substeps
            ctx.plain_forward_s[capture_key] = forward_elapsed
        if not (
            torch.all(torch.isfinite(positions))
            and torch.all(torch.isfinite(velocities))
            and torch.all(torch.isfinite(torques))
        ):
            _warn_nonfinite_rollout(ctx, float(dt))
            raise SysIdEnvironmentBridgeError(
                "Differentiable Newton rollout produced non-finite state or torque values; "
                "refusing to return invalid optimizer inputs."
            )
        return positions, velocities, torques

    @staticmethod
    def backward(  # noqa: ANN205
        ctx_ag,  # noqa: ANN001
        grad_positions: torch.Tensor,
        grad_velocities: torch.Tensor,
        grad_torques: torch.Tensor,
    ):
        bridge: NewtonDifferentiableSysIdBridge = ctx_ag.bridge
        try:
            return _NewtonDiffRollout._backward_impl(ctx_ag, grad_positions, grad_velocities, grad_torques)
        finally:
            bridge._release_backward_lease()

    @staticmethod
    def _backward_impl(  # noqa: ANN205
        ctx_ag,  # noqa: ANN001
        grad_positions: torch.Tensor,
        grad_velocities: torch.Tensor,
        grad_torques: torch.Tensor,
    ):
        ctx: _DiffRolloutContext = ctx_ag.rollout_ctx
        bridge: NewtonDifferentiableSysIdBridge = ctx_ag.bridge
        tape = ctx_ag.tape
        record: _CaptureRecord | None = ctx_ag.capture_record
        num_steps = ctx_ag.num_steps
        world_count, num_dof = ctx.world_count, ctx.num_dof
        width = world_count * num_dof

        def seed(arr, grad_tensor: torch.Tensor) -> None:  # noqa: ANN001
            adj = grad_tensor.permute(1, 0, 2).reshape(num_steps, width).detach().cpu().numpy()
            adj = _require_finite_array(adj.astype(np.float32), "backward output adjoint")
            full = np.zeros((int(arr.shape[0]), width), dtype=np.float32)
            full[:num_steps] = adj
            arr.assign(full)

        backward_start = time.perf_counter()
        if record is not None:
            # Captured backward copies staging seeds into output adjoints.
            seed(record.seed_q, grad_positions)
            seed(record.seed_qd, grad_velocities)
            seed(record.seed_tau, grad_torques)
            wp = bridge._modules.warp
            wp.capture_launch(record.graph_backward)
        else:
            seed(ctx.sim_q.grad, grad_positions)
            seed(ctx.sim_qd.grad, grad_velocities)
            seed(ctx.sim_tau.grad, grad_torques)
            tape.backward()

        def read_grad(arr, picker) -> torch.Tensor:  # noqa: ANN001
            values = picker(np.asarray(arr.grad.numpy(), dtype=np.float32))
            # A finite forward rollout can still carry an unstable linearization,
            # so the adjoint is where that mode surfaces. Give it the same
            # stability guidance the forward check emits before failing closed.
            if not np.all(np.isfinite(values)):
                _warn_nonfinite_rollout(ctx, float(ctx_ag.dt), stage="parameter gradients")
            values = _require_finite_array(values, "backward parameter gradient")
            tensor = torch.as_tensor(values, device=ctx_ag.out_device, dtype=ctx_ag.out_dtype).clone()
            return tensor

        grad_ke = read_grad(ctx.stiffness_wp, lambda g: g.reshape(world_count, num_dof))
        grad_kd = read_grad(ctx.damping_wp, lambda g: g.reshape(world_count, num_dof))
        grad_friction = read_grad(ctx.friction_wp, lambda g: g.reshape(world_count, num_dof))
        grad_mass = read_grad(ctx.model.body_mass, lambda g: g.reshape(world_count, ctx.bodies_per_world))
        grad_com = read_grad(
            ctx.model.body_com,
            lambda g: g.reshape(world_count, ctx.bodies_per_world, 3),
        )
        grad_inertia = read_grad(
            ctx.model.body_inertia,
            lambda g: g.reshape(world_count, ctx.bodies_per_world, 3, 3),
        )

        backward_elapsed = time.perf_counter() - backward_start
        if record is not None:
            bridge._modules.warp.capture_launch(record.graph_zero)
            plain_s = ctx.plain_backward_s.get(ctx_ag.capture_key)
            if plain_s is not None:
                before_total = plain_s + ctx.plain_forward_s.get(ctx_ag.capture_key, 0.0)
                ctx.plain_backward_s.pop(ctx_ag.capture_key, None)
                carb.log_info(
                    f"SysId: CUDA graph replay backward {plain_s:.2f} s -> {backward_elapsed:.2f} s "
                    f"(uncaptured forward+backward was {before_total:.2f} s per iteration)."
                )
        else:
            tape.zero()
            if ctx_ag.capture_key is not None:
                ctx.plain_backward_s[ctx_ag.capture_key] = backward_elapsed
        ctx_ag.tape = None
        return (
            grad_ke,
            grad_kd,
            grad_friction,
            grad_mass,
            grad_com,
            grad_inertia,
            None,
            None,
            None,
            None,
            None,
            None,
            None,
        )


__all__ = [
    "FRICTION_BUDGET_SHARE",
    "FRICTION_INV_VELOCITY_EPS",
    "FRICTION_SLOPE_STABILITY_RATIO",
    "NewtonDifferentiableSysIdBridge",
    "newton_differentiable_modules_available",
]
