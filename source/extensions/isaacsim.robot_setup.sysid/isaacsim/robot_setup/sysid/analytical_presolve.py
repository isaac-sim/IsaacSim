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

"""Analytical presolve helpers for fixed-base, contact-free SysId runs."""

from __future__ import annotations

import traceback
from collections.abc import Sequence
from dataclasses import dataclass, field, replace
from typing import TYPE_CHECKING

import carb
import numpy as np
import torch

from .inertia_param import log_cholesky_to_inertia_matrix
from .parameter_apply import decode_theta_to_apply_state
from .parameter_space import (
    JOINT_FRICTION_BASELINE_FLOOR,
    JOINT_GAIN_BASELINE_FLOOR,
    ParameterBaselineState,
)
from .parameter_types import GLOBAL_LINK_INDEX, SysIdParameterEntry, SysIdParameterType
from .regressor import (
    AnalyticalDynamicsContext,
    UnmappedSubtreeLink,
    finite_difference_acceleration,
    rnea_inverse_dynamics,
)
from .residual_config import ResidualWeightConfig
from .torque_semantics import (
    EFFORT_SEMANTICS_EXTERNAL,
    EFFORT_SEMANTICS_LINK_SIDE,
    resolve_effort_semantics,
)
from .trajectory_csv import TrajectoryDataset
from .trajectory_resampling import combine_sample_weights
from .usd_kinematics import (
    UsdJointEdge,
    child_to_parent_body_transform,
    matrix_to_spatial_motion_transform,
    usd_joint_local_transform,
)

if TYPE_CHECKING:
    from pxr import Usd

VELOCITY_DEADBAND = 1e-4
IDENTIFIABILITY_TOL = 1e-9
MAX_IDENTIFIABLE_CONDITION = 1e8
CORRELATION_WARN_THRESHOLD = 0.98


@dataclass
class AnalyticalPresolveResult:
    """Seed vector and diagnostics from torque-domain analytical presolve."""

    theta_seed: torch.Tensor
    identifiable: list[bool]
    #: Per-entry flag derived from regressor excitation, rank, and conditioning, independent of parameter bounds.
    rank: int
    condition: float
    column_norms: list[float]
    warnings: list[str] = field(default_factory=list)
    target_source: str = "unavailable"
    active_column_count: int = 0
    family_seeded_counts: dict[str, int] = field(default_factory=dict)
    #: Counts of in-bounds analytical seeds applied to `theta_seed`, grouped by parameter family.
    full_rank: int = 0
    full_condition: float = float("inf")
    gravity_rank: int = 0
    gravity_condition: float = float("inf")
    #: Per-entry flag: True when the parameter is structurally absent from the torque
    #: equation used for seeding (drive stiffness/damping under link-side measured
    #: torque), as opposed to unseeded for lack of excitation.
    torque_equation_excluded: list[bool] = field(default_factory=list)

    @property
    def seeded_count(self) -> int:  # noqa: D102
        return int(sum(self.family_seeded_counts.values()))


def build_fixed_base_context_from_usd(
    stage: Usd.Stage | None,
    link_paths: Sequence[str],
    joint_paths: Sequence[str],
    baseline: ParameterBaselineState,
) -> AnalyticalDynamicsContext | None:
    """Build a conservative fixed-base serial context from USD paths.

    The current RNEA context represents one moving link per optimized DOF. A
    fixed base link, when present, is treated as the parent of the first moving
    link and is not included in the returned inertial parameter rows.

    Args:
        stage: USD stage used by the operation.
        link_paths: Value supplied for ``link_paths``.
        joint_paths: Value supplied for ``joint_paths``.
        baseline: Baseline model parameters.

    Returns:
        Result produced by the operation.
    """
    if stage is None or not joint_paths:
        return None

    links = [str(path) for path in link_paths if str(path)]
    joints = [str(path) for path in joint_paths if str(path)]
    if not links or not joints:
        return None

    link_to_index = {path: idx for idx, path in enumerate(links)}
    child_paths: list[str] = []
    parent_paths: list[str | None] = []
    active_joint_paths: list[str] = []
    source_link_indices: list[int] = []
    motion_subspaces: list[np.ndarray] = []
    transforms = []
    warnings: list[str] = []

    for joint_path in joints:
        prim = stage.GetPrimAtPath(joint_path)
        if not prim or not prim.IsValid():
            return None

        parent_path, child_path = _joint_body_paths(prim)
        # body1 is the child frame used by the compact RNEA. Guessing it from
        # list position silently accepts reversed or incomplete USD joints.
        if child_path is None or child_path == parent_path:
            return None
        if child_path not in link_to_index:
            return None
        if parent_path in link_to_index and link_to_index[parent_path] >= link_to_index[child_path]:
            return None
        if child_path in child_paths:
            return None

        edge = _usd_joint_edge_from_prim(prim, len(child_paths), parent_path, child_path)
        if edge is None:
            return None

        child_paths.append(child_path)
        parent_paths.append(parent_path if parent_path in link_to_index else None)
        active_joint_paths.append(joint_path)
        source_link_indices.append(link_to_index[child_path])
        transforms.append(edge)
        subspace = _joint_motion_subspace(prim, edge)
        if subspace is None:
            return None
        motion_subspaces.append(subspace)

    child_to_context_index = {path: idx for idx, path in enumerate(child_paths)}
    parents = np.full(len(child_paths), -1, dtype=np.int64)
    for idx, parent_path in enumerate(parent_paths):
        if parent_path in child_to_context_index:
            parents[idx] = child_to_context_index[parent_path]
        if int(parents[idx]) >= idx:
            return None

    masses = np.ones(len(child_paths), dtype=np.float64)
    coms = np.zeros((len(child_paths), 3), dtype=np.float64)
    for ctx_idx, path in enumerate(child_paths):
        source_idx = link_to_index[path]
        masses[ctx_idx] = max(float(baseline.link_masses.get(source_idx, 1.0)), 1e-9)
        com = np.asarray(baseline.link_com.get(source_idx, np.zeros(3)), dtype=np.float64).reshape(-1)
        if com.shape[0] == 3 and np.all(np.isfinite(com)):
            coms[ctx_idx] = com

    def xup_fn(link_index: int, q_value: float) -> np.ndarray:
        edge = transforms[link_index]
        matrix = child_to_parent_body_transform(edge, q_value)
        spatial = matrix_to_spatial_motion_transform(matrix)
        if not np.all(np.isfinite(spatial)):
            raise FloatingPointError(f"nonfinite_xup_transform: joint={edge.joint_path}, link={edge.body1_path}")
        return spatial

    gravity = _gravity_in_base_frame(stage, transforms, parents)

    return AnalyticalDynamicsContext(
        parents=parents,
        motion_subspaces=np.asarray(motion_subspaces, dtype=np.float64),
        xup_fn=xup_fn,
        gravity=gravity,
        link_masses=masses,
        link_coms=coms,
        source_link_indices=np.asarray(source_link_indices, dtype=np.int64),
        link_paths=tuple(child_paths),
        joint_paths=tuple(active_joint_paths),
        warnings=tuple(warnings),
        unmapped_subtree_links=_collect_unmapped_subtree_links(stage, links, link_to_index, child_to_context_index),
    )


def _gravity_in_base_frame(stage: Usd.Stage, transforms: list[UsdJointEdge], parents: np.ndarray) -> np.ndarray | None:
    """Rotate world gravity into the fixed base body frame.

    Args:
        stage: Stage containing the fixed-base articulation.
        transforms: Joint transforms ordered like the analytical context links.
        parents: Parent link indices for the analytical context.

    Returns:
        Spatial gravity expressed in the fixed base frame, or ``None`` when no
        fixed base can be resolved.
    """
    from pxr import Gf, Usd, UsdGeom

    for index, parent_index in enumerate(parents):
        if int(parent_index) >= 0:
            continue
        base_path = transforms[index].body0_path
        if not base_path:
            continue
        base_prim = stage.GetPrimAtPath(base_path)
        if not base_prim or not base_prim.IsValid():
            continue
        world = UsdGeom.XformCache(Usd.TimeCode.Default()).GetLocalToWorldTransform(base_prim)
        gravity_linear = np.asarray(
            world.GetInverse().TransformDir(Gf.Vec3d(0.0, 0.0, -9.80665)),
            dtype=np.float64,
        )
        return np.concatenate((np.zeros(3, dtype=np.float64), gravity_linear))
    return None


def _collect_unmapped_subtree_links(
    stage: Usd.Stage,
    link_paths: list[str],
    link_to_index: dict[str, int],
    child_to_context_index: dict[str, int],
) -> tuple[UnmappedSubtreeLink, ...]:
    """Locate links absent from the compact context that hang below a mapped link.

    Unmapped joints (a gripper's fixed hand attachment, held finger joints) do
    not appear in the context, so their subtree's weight would otherwise vanish
    from the RNEA. Scans the stage below the links' common ancestor for *all*
    joint types — the mapped-joint list excludes fixed joints, which are exactly
    how end-effectors attach — and walks each unmapped link up to its nearest
    mapped ancestor, accumulating the rest-configuration transform.

    Args:
        stage: USD stage used by the operation.
        link_paths: Value supplied for ``link_paths``.
        link_to_index: Value supplied for ``link_to_index``.
        child_to_context_index: Value supplied for ``child_to_context_index``.

    Returns:
        Result produced by the operation.
    """
    from pxr import Gf, Sdf, Usd, UsdPhysics

    try:
        common = Sdf.Path(link_paths[0])
        for path in link_paths[1:]:
            common = common.GetCommonPrefix(Sdf.Path(path))
        root_prim = stage.GetPrimAtPath(common) if common and common != Sdf.Path.absoluteRootPath else None
        prims = Usd.PrimRange(root_prim, Usd.TraverseInstanceProxies()) if root_prim else stage.Traverse()
        child_to_parent: dict[str, tuple[str, Gf.Matrix4d]] = {}
        for prim in prims:
            if not prim.IsA(UsdPhysics.Joint):
                continue
            parent_path, child_path = _joint_body_paths(prim)
            if parent_path not in link_to_index or child_path not in link_to_index:
                continue
            if child_path in child_to_parent:
                continue
            joint = UsdPhysics.Joint(prim)
            rest = usd_joint_local_transform(joint, 1).GetInverse() * usd_joint_local_transform(joint, 0)
            child_to_parent[child_path] = (parent_path, rest)

        lumps: list[UnmappedSubtreeLink] = []
        for path in link_paths:
            if path in child_to_context_index:
                continue
            current = path
            to_ancestor = Gf.Matrix4d(1.0)
            visited = {current}
            while current in child_to_parent:
                parent_path, rest = child_to_parent[current]
                # Row-vector convention: appending on the right maps the original
                # link's coordinates one body further up the chain.
                to_ancestor = to_ancestor * rest
                if parent_path in visited:
                    break
                visited.add(parent_path)
                if parent_path in child_to_context_index:
                    arr = np.asarray(to_ancestor, dtype=np.float64).reshape(4, 4)
                    if not np.all(np.isfinite(arr)):
                        break
                    lumps.append(
                        UnmappedSubtreeLink(
                            context_link_index=child_to_context_index[parent_path],
                            source_link_index=link_to_index[path],
                            rotation=arr[:3, :3].T.copy(),
                            translation=arr[3, :3].copy(),
                        )
                    )
                    break
                current = parent_path
        return tuple(lumps)
    except Exception as exc:
        # Collecting optional lumped-link diagnostics must not abort presolve.
        carb.log_verbose(f"Unmapped subtree link collection failed: {exc}\n{traceback.format_exc()}")
        return ()


def assemble_joint_scale_torque_regressor(
    positions: np.ndarray,
    velocities: np.ndarray,
    commands: np.ndarray,
    param_entries: Sequence[SysIdParameterEntry],
    baseline: ParameterBaselineState,
    *,
    velocity_deadband: float = VELOCITY_DEADBAND,
) -> np.ndarray:
    """Return torque-domain columns for joint friction/damping/stiffness scale terms.

    Args:
        positions: Value supplied for ``positions``.
        velocities: Value supplied for ``velocities``.
        commands: Value supplied for ``commands``.
        param_entries: Value supplied for ``param_entries``.
        baseline: Baseline model parameters.
        velocity_deadband: Value supplied for ``velocity_deadband``.

    Returns:
        Result produced by the operation.
    """
    q = np.asarray(positions, dtype=np.float64)
    qd = np.asarray(velocities, dtype=np.float64)
    cmd = np.asarray(commands, dtype=np.float64)
    times, num_dof = qd.shape
    regressor = np.zeros((times * num_dof, len(param_entries)), dtype=np.float64)

    for p_idx, entry in enumerate(param_entries):
        if entry.dof_index < 0 or entry.dof_index >= num_dof:
            continue
        j = entry.dof_index
        rows = np.arange(times, dtype=np.int64) * num_dof + j
        if entry.param_type == SysIdParameterType.JOINT_FRICTION:
            baseline_value = _baseline_friction(baseline, j)
            signs = np.sign(qd[:, j])
            signs[np.abs(qd[:, j]) <= velocity_deadband] = 0.0
            regressor[rows, p_idx] = -baseline_value * signs
        elif entry.param_type == SysIdParameterType.JOINT_DAMPING:
            regressor[rows, p_idx] = -_baseline_damping(baseline, j) * qd[:, j]
        elif entry.param_type == SysIdParameterType.JOINT_STIFFNESS:
            regressor[rows, p_idx] = _baseline_stiffness(baseline, j) * (cmd[:, j] - q[:, j])

    return regressor


def assemble_link_side_friction_regressor(
    velocities: np.ndarray,
    param_entries: Sequence[SysIdParameterEntry],
    baseline: ParameterBaselineState,
    *,
    velocity_deadband: float = VELOCITY_DEADBAND,
) -> np.ndarray:
    """Return friction-scale columns for the link-side torque equation.

    Models ``tau_J = RNEA(q, qd, qdd) + f * sign(qd)``: joint friction is the only
    drive-side term that can appear alongside the load-side dynamics. Drive
    stiffness/damping columns are deliberately absent -- the drive PD output equals
    ``tau_J`` at the true parameters, so adding them would double-count the
    measurement (see :func:`run_analytical_presolve`).

    Args:
        velocities: Value supplied for ``velocities``.
        param_entries: Value supplied for ``param_entries``.
        baseline: Baseline model parameters.
        velocity_deadband: Value supplied for ``velocity_deadband``.

    Returns:
        Result produced by the operation.
    """
    qd = np.asarray(velocities, dtype=np.float64)
    times, num_dof = qd.shape
    regressor = np.zeros((times * num_dof, len(param_entries)), dtype=np.float64)

    for p_idx, entry in enumerate(param_entries):
        if entry.param_type != SysIdParameterType.JOINT_FRICTION:
            continue
        if entry.dof_index < 0 or entry.dof_index >= num_dof:
            continue
        j = entry.dof_index
        rows = np.arange(times, dtype=np.int64) * num_dof + j
        signs = np.sign(qd[:, j])
        signs[np.abs(qd[:, j]) <= velocity_deadband] = 0.0
        regressor[rows, p_idx] = _baseline_friction(baseline, j) * signs

    return regressor


def run_analytical_presolve(
    trajectory: TrajectoryDataset,
    param_entries: Sequence[SysIdParameterEntry],
    theta_initial: torch.Tensor,
    theta_min: torch.Tensor,
    theta_max: torch.Tensor,
    baseline: ParameterBaselineState,
    *,
    residual_config: ResidualWeightConfig | None = None,
    dynamics_context: AnalyticalDynamicsContext | None = None,
    max_steps: int | None = None,
    torque_semantics: str | None = None,
    accelerations: np.ndarray | None = None,
) -> AnalyticalPresolveResult:
    """Fit analytical torque-domain terms and return a seeded theta vector.

    The regression equation depends on what the effort telemetry channel measures
    (``torque_semantics``; read from the trajectory metadata when ``None``):

    - ``link_side`` (the ``sensor_msgs/JointState.effort`` convention on
      gravity-compensated robots such as the Franka): the measurement is the
      transmitted joint torque ``tau_J = M(q) qdd + C(q, qd) qd + g(q)``, so the
      regression is gravity-inclusive RNEA inertial columns plus load-side friction
      columns. Drive stiffness/damping are NOT seeded from this equation: the drive
      PD output equals ``tau_J`` at the true parameters, so adding PD columns on top
      of the RNEA columns makes the model predict twice the measurement and biases
      every seeded parameter. Drive gains are identified in the position domain by
      the rollout optimizer instead.
    - ``external`` (e.g. Franka ``tau_ext_hat_filtered``): the measurement carries
      no parameter information in a contact-free run, so measured torque is ignored
      and the presolve falls back to the no-torque path.

    Without usable measured torque, drive friction/stiffness/damping scales are
    seeded against baseline inverse-dynamics torque -- the torque the drive must
    supply to realize the recorded motion -- and inertial parameters are left
    unchanged.

    ``accelerations`` overrides the internal finite-difference acceleration
    estimate. Callers stitching multiple training chunks into one trajectory
    must pass per-chunk accelerations: a finite difference across a chunk
    boundary differentiates through the inter-chunk discontinuity.

    Args:
        trajectory: Telemetry trajectory used by the operation.
        param_entries: Value supplied for ``param_entries``.
        theta_initial: Value supplied for ``theta_initial``.
        theta_min: Value supplied for ``theta_min``.
        theta_max: Value supplied for ``theta_max``.
        baseline: Baseline model parameters.
        residual_config: Value supplied for ``residual_config``.
        dynamics_context: Value supplied for ``dynamics_context``.
        max_steps: Value supplied for ``max_steps``.
        torque_semantics: Value supplied for ``torque_semantics``.
        accelerations: Value supplied for ``accelerations``.

    Returns:
        Result produced by the operation.
    """
    entries = list(param_entries)
    theta_seed = theta_initial.detach().cpu().to(dtype=torch.float32).clone()
    warnings: list[str] = []
    identifiable = [False for _ in entries]
    family_seeded_counts: dict[str, int] = {}
    torque_equation_excluded = [False for _ in entries]

    if residual_config is not None and residual_config.contact_force_weight > 0.0:
        warnings.append("Analytical presolve skipped because contact force residuals are enabled.")
        return _empty_result(theta_seed, identifiable, warnings)

    steps = int(max_steps) if max_steps is not None else int(trajectory.positions.shape[0])
    steps = max(1, min(steps, int(trajectory.positions.shape[0])))
    q = np.asarray(trajectory.positions[:steps], dtype=np.float64)
    qd = np.asarray(trajectory.velocities[:steps], dtype=np.float64)
    cmd = np.asarray(trajectory.commands[:steps], dtype=np.float64)
    times = np.asarray(trajectory.times[:steps], dtype=np.float64)
    if accelerations is not None:
        qdd = np.asarray(accelerations[:steps], dtype=np.float64)
        if qdd.shape != qd.shape:
            raise ValueError(f"accelerations shape {qdd.shape} does not match velocities {qd.shape}.")
    else:
        qdd = finite_difference_acceleration(times, qd)

    semantics = _effort_semantics(trajectory, torque_semantics)
    use_measured = _has_measured_torque(trajectory, steps)
    if use_measured and semantics != EFFORT_SEMANTICS_LINK_SIDE:
        if semantics == EFFORT_SEMANTICS_EXTERNAL:
            warnings.append(
                "Analytical presolve ignored measured torque because the effort channel reports external "
                "torque (tau_ext), which carries no parameter information in a contact-free run."
            )
        else:
            warnings.append(
                f"Analytical presolve ignored measured torque because effort semantics '{semantics}' is not "
                f"recognized (expected '{EFFORT_SEMANTICS_LINK_SIDE}' or '{EFFORT_SEMANTICS_EXTERNAL}')."
            )
        use_measured = False
    if use_measured and dynamics_context is None:
        warnings.append(
            "Analytical presolve ignored measured link-side torque because no fixed-base dynamics context "
            "was available to model it."
        )
        use_measured = False

    inertial_columns = _inertial_column_indices(entries)
    num_dof = int(q.shape[1])
    inertial_regressor = np.zeros((steps * num_dof, len(entries)), dtype=np.float64)
    initial_inertial_tau = np.zeros((steps, num_dof), dtype=np.float64)
    inertial_ok = False
    full_rank = 0
    full_condition = float("inf")
    gravity_rank = 0
    gravity_condition = float("inf")

    # The RNEA evaluation is needed both to build inertial delta columns and, on the
    # link-side path, to predict the measured torque at the initial theta.
    if dynamics_context is not None and (inertial_columns or use_measured):
        inertial_regressor, initial_inertial_tau, inertial_ok = _assemble_inertial_delta_regressor(
            q,
            qd,
            qdd,
            entries,
            theta_seed,
            theta_min,
            theta_max,
            baseline,
            dynamics_context,
            inertial_columns,
            warnings,
        )
        if inertial_columns:
            full_rank, full_condition = _diagnostic_rank_condition(inertial_regressor, inertial_columns)
            gravity_regressor, _, _ = _assemble_inertial_delta_regressor(
                q,
                np.zeros_like(qd),
                np.zeros_like(qdd),
                entries,
                theta_seed,
                theta_min,
                theta_max,
                baseline,
                dynamics_context,
                inertial_columns,
                warnings=None,
            )
            gravity_rank, gravity_condition = _diagnostic_rank_condition(gravity_regressor, inertial_columns)
    elif inertial_columns:
        warnings.append(
            "Analytical presolve could not seed inertial parameters because no fixed-base dynamics context was available."
        )

    if use_measured and not inertial_ok:
        warnings.append(
            "Analytical presolve ignored measured link-side torque because inverse dynamics could not be "
            "evaluated at the initial parameters."
        )
        use_measured = False

    if use_measured:
        # Link-side equation: tau_J = RNEA(q, qd, qdd) + f * sign(qd). Drive stiffness/
        # damping columns are excluded -- the drive output already equals tau_J at the
        # true parameters, so summing the drive model with RNEA double-counts the
        # measurement (the combined model predicts ~2*tau_J and biases every seed).
        joint_regressor = assemble_link_side_friction_regressor(qd, entries, baseline)
        analytical_columns = [
            idx for idx, entry in enumerate(entries) if entry.param_type == SysIdParameterType.JOINT_FRICTION
        ]
        drive_gain_columns = [
            idx
            for idx, entry in enumerate(entries)
            if entry.param_type in (SysIdParameterType.JOINT_DAMPING, SysIdParameterType.JOINT_STIFFNESS)
        ]
        for idx in drive_gain_columns:
            torque_equation_excluded[idx] = True
        if drive_gain_columns:
            warnings.append(
                "Analytical presolve left drive stiffness/damping unchanged: link-side measured torque equals "
                "the drive output at the true parameters, so drive gains are not observable in the torque "
                "equation; they are identified in the position domain by the optimizer."
            )
    else:
        # Drive balance against baseline inverse-dynamics torque: the PD output
        # k*(cmd - q) - d*qd - f*sign(qd) must supply the torque the recorded motion
        # requires, which seeds the drive gain scales.
        joint_regressor = assemble_joint_scale_torque_regressor(q, qd, cmd, entries, baseline)
        analytical_columns = [
            idx
            for idx, entry in enumerate(entries)
            if entry.param_type
            in (
                SysIdParameterType.JOINT_FRICTION,
                SysIdParameterType.JOINT_DAMPING,
                SysIdParameterType.JOINT_STIFFNESS,
            )
        ]

    X = joint_regressor + inertial_regressor

    if inertial_columns and not use_measured:
        warnings.append(
            "Analytical presolve left inertial parameters unchanged because measured link-side torque "
            "telemetry is required."
        )
    seed_columns = list(analytical_columns)
    if use_measured:
        seed_columns.extend(inertial_columns)
    seed_columns = sorted(set(seed_columns))

    if not seed_columns:
        warnings.append("Analytical presolve found no supported parameters to seed.")
        return _empty_result(
            theta_seed,
            identifiable,
            warnings,
            X,
            target_source="unavailable",
            full_rank=full_rank,
            full_condition=full_condition,
            gravity_rank=gravity_rank,
            gravity_condition=gravity_condition,
            torque_equation_excluded=torque_equation_excluded,
        )

    if use_measured:
        target = np.asarray(trajectory.torques[:steps], dtype=np.float64)
        target_source = "measured_torque"
    else:
        target = _baseline_inverse_dynamics(trajectory, steps, baseline, dynamics_context, warnings, accelerations=qdd)
        if target is None:
            warnings.append(
                "Analytical presolve skipped because no usable torque target was available "
                "(no link-side measured torque and no baseline inverse dynamics)."
            )
            return _empty_result(
                theta_seed,
                identifiable,
                warnings,
                X,
                target_source="unavailable",
                full_rank=full_rank,
                full_condition=full_condition,
                gravity_rank=gravity_rank,
                gravity_condition=gravity_condition,
                torque_equation_excluded=torque_equation_excluded,
            )
        warnings.append("Analytical presolve used baseline inverse-dynamics torque as the regression target.")
        target_source = "baseline_inverse_dynamics"

    y = target.reshape(-1)
    if y.shape[0] != X.shape[0]:
        raise ValueError(f"Analytical target has {y.shape[0]} rows but the regressor has {X.shape[0]} rows.")

    row_scale = np.ones(X.shape[0], dtype=np.float64)
    combined_weights = combine_sample_weights(
        residual_config.sample_weights if residual_config is not None else None,
        getattr(trajectory, "residual_sample_weights", None),
        num_steps=steps,
    )
    if combined_weights is not None:
        # Sample weights are residual multipliers: ResidualEngine applies them
        # before squaring, so use w here (not sqrt(w)) to match its metric.
        sample_scale = np.clip(combined_weights, 0.0, None)
        row_scale = np.repeat(sample_scale, num_dof)[: X.shape[0]]
    X_for_fit = X * row_scale[:, None]

    column_norms = np.linalg.norm(X_for_fit, axis=0)
    max_norm = float(np.max(column_norms[seed_columns])) if seed_columns else 0.0
    norm_tol = max(IDENTIFIABILITY_TOL, max_norm * 1e-8)
    active = [idx for idx in seed_columns if column_norms[idx] > norm_tol]
    if not active:
        warnings.append("Analytical presolve skipped because selected analytical columns have no useful excitation.")
        return _empty_result(
            theta_seed,
            identifiable,
            warnings,
            X,
            target_source=target_source,
            full_rank=full_rank,
            full_condition=full_condition,
            gravity_rank=gravity_rank,
            gravity_condition=gravity_condition,
            torque_equation_excluded=torque_equation_excluded,
        )

    X_active = X_for_fit[:, active]
    rank, condition = _rank_and_condition(X_active)
    if rank < len(active):
        warnings.append("Analytical presolve found rank-deficient active columns; leaving analytical seed unchanged.")
        return AnalyticalPresolveResult(
            theta_seed=theta_seed,
            identifiable=identifiable,
            rank=rank,
            condition=condition,
            column_norms=column_norms.tolist(),
            warnings=warnings,
            target_source=target_source,
            active_column_count=len(active),
            family_seeded_counts=family_seeded_counts,
            full_rank=full_rank,
            full_condition=full_condition,
            gravity_rank=gravity_rank,
            gravity_condition=gravity_condition,
            torque_equation_excluded=torque_equation_excluded,
        )
    if not np.isfinite(condition) or condition > MAX_IDENTIFIABLE_CONDITION:
        warnings.append(
            "Analytical presolve found ill-conditioned active columns "
            f"(condition {condition:.3g} exceeds {MAX_IDENTIFIABLE_CONDITION:.3g}); "
            "leaving analytical seed unchanged."
        )
        return AnalyticalPresolveResult(
            theta_seed=theta_seed,
            identifiable=identifiable,
            rank=rank,
            condition=condition,
            column_norms=column_norms.tolist(),
            warnings=warnings,
            target_source=target_source,
            active_column_count=len(active),
            family_seeded_counts=family_seeded_counts,
            full_rank=full_rank,
            full_condition=full_condition,
            gravity_rank=gravity_rank,
            gravity_condition=gravity_condition,
            torque_equation_excluded=torque_equation_excluded,
        )

    _append_correlation_warnings(X_active, active, entries, warnings)
    theta_np = theta_seed.detach().cpu().numpy().astype(np.float64)
    initial_prediction = joint_regressor @ theta_np
    if use_measured:
        initial_prediction += initial_inertial_tau.reshape(-1)[: initial_prediction.shape[0]]
    residual_target = (y - initial_prediction) * row_scale
    solution_delta, *_ = np.linalg.lstsq(X_active, residual_target, rcond=None)
    if not np.all(np.isfinite(solution_delta)):
        warnings.append("Analytical presolve produced a non-finite least-squares solution; leaving seed unchanged.")
        return AnalyticalPresolveResult(
            theta_seed=theta_seed,
            identifiable=identifiable,
            rank=rank,
            condition=condition,
            column_norms=column_norms.tolist(),
            warnings=warnings,
            target_source=target_source,
            active_column_count=len(active),
            family_seeded_counts=family_seeded_counts,
            full_rank=full_rank,
            full_condition=full_condition,
            gravity_rank=gravity_rank,
            gravity_condition=gravity_condition,
            torque_equation_excluded=torque_equation_excluded,
        )

    # Identifiability is determined by the active regressor's rank and conditioning.
    # Parameter bounds only decide whether an analytical seed can be applied.
    for p_idx in active:
        identifiable[p_idx] = True

    mins = theta_min.detach().cpu().numpy().astype(np.float64)
    maxs = theta_max.detach().cpu().numpy().astype(np.float64)

    for local_idx, p_idx in enumerate(active):
        value = float(theta_np[p_idx] + solution_delta[local_idx])
        if value < mins[p_idx] or value > maxs[p_idx]:
            warnings.append(
                f"Analytical presolve left {entries[p_idx].short_type_label()} unchanged because "
                f"the unconstrained seed {value:.6g} is outside [{mins[p_idx]:.6g}, {maxs[p_idx]:.6g}]."
            )
            continue
        theta_seed[p_idx] = torch.tensor(value, dtype=theta_seed.dtype)
        family = _entry_family(entries[p_idx])
        family_seeded_counts[family] = family_seeded_counts.get(family, 0) + 1

    return AnalyticalPresolveResult(
        theta_seed=theta_seed,
        identifiable=identifiable,
        rank=rank,
        condition=condition,
        column_norms=column_norms.tolist(),
        warnings=warnings,
        target_source=target_source,
        active_column_count=len(active),
        family_seeded_counts=family_seeded_counts,
        full_rank=full_rank,
        full_condition=full_condition,
        gravity_rank=gravity_rank,
        gravity_condition=gravity_condition,
        torque_equation_excluded=torque_equation_excluded,
    )


def _inertial_column_indices(entries: Sequence[SysIdParameterEntry]) -> list[int]:
    inertial_types = {
        SysIdParameterType.LINK_MASS,
        SysIdParameterType.LINK_COM_OFFSET_X,
        SysIdParameterType.LINK_COM_OFFSET_Y,
        SysIdParameterType.LINK_COM_OFFSET_Z,
        SysIdParameterType.LINK_INERTIA_LOG_CHOLESKY,
    }
    return [idx for idx, entry in enumerate(entries) if entry.param_type in inertial_types]


def _has_measured_torque(trajectory: TrajectoryDataset, steps: int) -> bool:
    torques = getattr(trajectory, "torques", None)
    if torques is None:
        return False
    measured = np.asarray(torques[:steps], dtype=np.float64)
    return measured.size > 0 and np.all(np.isfinite(measured))


def effort_semantics(trajectory: TrajectoryDataset, override: str | None = None) -> str:
    """Resolve the effort channel semantics declared for a loaded trajectory.

    Public wrapper over :func:`_effort_semantics` for UI/reporting consumers
    (e.g. the Data panel's source summary).

    Args:
        trajectory: Telemetry trajectory used by the operation.
        override: Value supplied for ``override``.

    Returns:
        Result produced by the operation.
    """
    return _effort_semantics(trajectory, override)


def _effort_semantics(trajectory: TrajectoryDataset, override: str | None) -> str:
    """Resolve the effort channel semantics from an override or trajectory metadata.

    Args:
        trajectory: Telemetry trajectory used by the operation.
        override: Value supplied for ``override``.

    Returns:
        Result produced by the operation.
    """
    return resolve_effort_semantics(trajectory, override)


def _entry_family(entry: SysIdParameterEntry) -> str:
    if entry.param_type == SysIdParameterType.LINK_MASS:
        return "link_mass"
    if entry.param_type in (
        SysIdParameterType.LINK_COM_OFFSET_X,
        SysIdParameterType.LINK_COM_OFFSET_Y,
        SysIdParameterType.LINK_COM_OFFSET_Z,
    ):
        return "link_com"
    if entry.param_type == SysIdParameterType.LINK_INERTIA_LOG_CHOLESKY:
        return "link_inertia"
    return entry.param_type.value


def _theta_step(
    theta: np.ndarray,
    theta_min: torch.Tensor,
    theta_max: torch.Tensor,
    index: int,
) -> float:
    mins = theta_min.detach().cpu().numpy().astype(np.float64)
    maxs = theta_max.detach().cpu().numpy().astype(np.float64)
    span = max(float(maxs[index] - mins[index]), 1e-6)
    step = min(max(span * 1e-4, 1e-5), 1e-3)
    if theta[index] + step <= maxs[index]:
        return step
    if theta[index] - step >= mins[index]:
        return -step
    return 0.0


def _context_source_link_indices(context: AnalyticalDynamicsContext) -> np.ndarray:
    if context.source_link_indices is None:
        return np.arange(context.num_links, dtype=np.int64)
    values = np.asarray(context.source_link_indices, dtype=np.int64).reshape(-1)
    if values.shape[0] != context.num_links:
        return np.arange(context.num_links, dtype=np.int64)
    return values


def _entries_for_context(
    entries: Sequence[SysIdParameterEntry],
    source_link_indices: np.ndarray,
) -> list[SysIdParameterEntry]:
    source_to_context = {int(source_idx): ctx_idx for ctx_idx, source_idx in enumerate(source_link_indices)}
    unmapped = int(np.asarray(source_link_indices).reshape(-1).shape[0])
    remapped: list[SysIdParameterEntry] = []
    for entry in entries:
        if entry.param_type not in (
            SysIdParameterType.LINK_MASS,
            SysIdParameterType.LINK_COM_OFFSET_X,
            SysIdParameterType.LINK_COM_OFFSET_Y,
            SysIdParameterType.LINK_COM_OFFSET_Z,
            SysIdParameterType.LINK_INERTIA_LOG_CHOLESKY,
        ):
            remapped.append(entry)
            continue
        if entry.link_index == GLOBAL_LINK_INDEX:
            remapped.append(entry)
            continue
        ctx_idx = source_to_context.get(int(entry.link_index))
        remapped.append(replace(entry, link_index=(ctx_idx if ctx_idx is not None else unmapped)))
    return remapped


def _unmapped_inertial_parameter_warnings(
    entries: Sequence[SysIdParameterEntry],
    context: AnalyticalDynamicsContext,
) -> list[str]:
    source_indices = {int(value) for value in _context_source_link_indices(context)}
    warnings: list[str] = []
    for entry in entries:
        if entry.link_index == GLOBAL_LINK_INDEX:
            continue
        if entry.param_type not in (
            SysIdParameterType.LINK_MASS,
            SysIdParameterType.LINK_COM_OFFSET_X,
            SysIdParameterType.LINK_COM_OFFSET_Y,
            SysIdParameterType.LINK_COM_OFFSET_Z,
            SysIdParameterType.LINK_INERTIA_LOG_CHOLESKY,
        ):
            continue
        if int(entry.link_index) not in source_indices:
            warnings.append(
                "unmapped_inertial_parameter: Analytical presolve left "
                f"{entry.short_type_label()} for source link {entry.link_index} unchanged because that link "
                "is not represented by the compact moving-link RNEA context."
            )
    return warnings


def _validate_inertial_context_inputs(
    context: AnalyticalDynamicsContext,
    baseline: ParameterBaselineState,
    positions: np.ndarray,
    velocities: np.ndarray,
    accelerations: np.ndarray,
) -> list[str]:
    warnings = list(context.warnings or ())
    if (
        positions.shape[1] != context.num_dof
        or velocities.shape[1] != context.num_dof
        or accelerations.shape[1] != context.num_dof
    ):
        warnings.append(
            "rnea_context_dof_mismatch: Analytical presolve skipped inertial columns because trajectory DOFs "
            f"({positions.shape[1]}) do not match analytical context DOFs ({context.num_dof})."
        )
        return warnings
    if not (
        np.all(np.isfinite(positions))
        and np.all(np.isfinite(velocities))
        and np.all(np.isfinite(accelerations))
        and np.all(np.isfinite(context.parents))
        and np.all(np.isfinite(context.motion_subspaces))
    ):
        warnings.append(
            "rnea_nonfinite_input: Analytical presolve skipped inertial columns because inputs are non-finite."
        )
        return warnings
    for ctx_idx, source_idx in enumerate(_context_source_link_indices(context)):
        path = context.link_paths[ctx_idx] if ctx_idx < len(context.link_paths) else f"source_link_{source_idx}"
        try:
            mass = float(baseline.link_masses.get(int(source_idx), 1.0))
        except (TypeError, ValueError):
            mass = float("nan")
        if not np.isfinite(mass) or mass <= 0.0:
            warnings.append(f"invalid_inertia_baseline: link={path} field=mass value={mass}")
        try:
            com = np.asarray(baseline.link_com.get(int(source_idx), np.zeros(3)), dtype=np.float64).reshape(3)
        except (TypeError, ValueError):
            com = np.full(3, np.nan, dtype=np.float64)
        if not np.all(np.isfinite(com)):
            warnings.append(f"invalid_inertia_baseline: link={path} field=center_of_mass value={com.tolist()}")
        inertia_lc = baseline.link_inertia_lc.get(int(source_idx))
        if inertia_lc is not None:
            try:
                inertia = log_cholesky_to_inertia_matrix(inertia_lc)
                if not np.all(np.isfinite(inertia)) or np.any(np.linalg.eigvalsh(inertia) <= 0.0):
                    warnings.append(f"invalid_inertia_baseline: link={path} field=inertia value=not_positive_definite")
            except (TypeError, ValueError, np.linalg.LinAlgError) as exc:
                warnings.append(f"invalid_inertia_baseline: link={path} field=inertia error={exc}")
    if warnings:
        return warnings
    try:
        for idx in range(context.num_dof):
            sample_q = float(positions[0, idx]) if positions.shape[0] else 0.0
            transform = context.xup(idx, sample_q)
            if not np.all(np.isfinite(transform)):
                joint = context.joint_paths[idx] if idx < len(context.joint_paths) else idx
                warnings.append(f"nonfinite_xup_transform: joint={joint}")
    except (FloatingPointError, ValueError) as exc:
        warnings.append(str(exc))
    return warnings


def _origin_referenced_inertia(inertia_com: np.ndarray, mass: float, com: np.ndarray) -> np.ndarray:
    """Transport a CoM-referenced rotational inertia to the body-frame origin.

    The RNEA spatial inertia ``[[I, h x], [h x^T, m 1]]`` needs ``I`` about the
    body origin; captured baselines (and the theta decode) carry the inertia
    about the CoM, so the parallel-axis term is added here.

    Args:
        inertia_com: Value supplied for ``inertia_com``.
        mass: Value supplied for ``mass``.
        com: Value supplied for ``com``.

    Returns:
        Result produced by the operation.
    """
    return inertia_com + mass * (float(com @ com) * np.eye(3, dtype=np.float64) - np.outer(com, com))


def _inertial_params_for_theta(
    baseline: ParameterBaselineState,
    context: AnalyticalDynamicsContext,
    entries: Sequence[SysIdParameterEntry],
    theta: np.ndarray | torch.Tensor,
) -> np.ndarray:
    theta_tensor = torch.as_tensor(theta, dtype=torch.float32)
    source_link_indices = _context_source_link_indices(context)
    baseline_inertia_lc = {
        ctx_idx: np.asarray(baseline.link_inertia_lc[source_idx], dtype=np.float64).copy()
        for ctx_idx, source_idx in enumerate(source_link_indices)
        if source_idx in baseline.link_inertia_lc
    }
    context_entries = _entries_for_context(entries, source_link_indices)
    state = decode_theta_to_apply_state(
        num_dof=context.num_dof,
        num_links=context.num_links,
        param_entries=context_entries,
        theta_row=theta_tensor,
        baseline_link_inertia_lc=baseline_inertia_lc,
    )
    params = np.zeros((context.num_links, 10), dtype=np.float64)
    for ctx_idx, source_idx in enumerate(source_link_indices):
        baseline_mass = max(float(baseline.link_masses.get(source_idx, 1.0)), 1e-9)
        mass_scale = float(state.link_mass_scale[ctx_idx]) if ctx_idx < state.link_mass_scale.shape[0] else 1.0
        mass = max(baseline_mass * mass_scale, 1e-9)
        baseline_com = np.asarray(baseline.link_com.get(source_idx, np.zeros(3)), dtype=np.float64).reshape(3)
        com_delta = (
            np.asarray(state.link_com_delta[ctx_idx], dtype=np.float64).reshape(3)
            if ctx_idx < state.link_com_delta.shape[0]
            else np.zeros(3, dtype=np.float64)
        )
        com = baseline_com + com_delta
        inertia = None
        if ctx_idx in state.link_inertia_flat:
            inertia = np.asarray(state.link_inertia_flat[ctx_idx], dtype=np.float64).reshape(3, 3)
        else:
            inertia_lc = baseline.link_inertia_lc.get(source_idx)
            if inertia_lc is not None and np.all(np.isfinite(inertia_lc)):
                try:
                    inertia = log_cholesky_to_inertia_matrix(inertia_lc)
                except ValueError:
                    inertia = None
        if inertia is None or not np.all(np.isfinite(inertia)):
            inertia = np.eye(3, dtype=np.float64) * mass
        inertia = _origin_referenced_inertia(inertia, mass, com)
        params[ctx_idx, 0] = mass
        params[ctx_idx, 1:4] = mass * com
        params[ctx_idx, 4] = inertia[0, 0]
        params[ctx_idx, 5] = 0.5 * (inertia[0, 1] + inertia[1, 0])
        params[ctx_idx, 6] = 0.5 * (inertia[0, 2] + inertia[2, 0])
        params[ctx_idx, 7] = inertia[1, 1]
        params[ctx_idx, 8] = 0.5 * (inertia[1, 2] + inertia[2, 1])
        params[ctx_idx, 9] = inertia[2, 2]
    # Unmapped distal bodies (e.g. an attached gripper) carry no theta entries, so
    # their baseline composite is a theta-independent additive load. RNEA is linear
    # in the per-link `[m, h, I]` rows, so including it corrects the predicted torque
    # level (gravity on the proximal joints) while the finite-difference regressor
    # columns -- differences of two evaluations that both contain it -- are unchanged.
    _accumulate_unmapped_subtree_params(params, context, baseline)
    return params


def _inverse_dynamics_for_theta(
    positions: np.ndarray,
    velocities: np.ndarray,
    accelerations: np.ndarray,
    entries: Sequence[SysIdParameterEntry],
    theta: np.ndarray | torch.Tensor,
    baseline: ParameterBaselineState,
    context: AnalyticalDynamicsContext,
) -> np.ndarray:
    inertial = _inertial_params_for_theta(baseline, context, entries, theta)
    rows = []
    with np.errstate(invalid="raise", over="raise"):
        for t in range(positions.shape[0]):
            tau = rnea_inverse_dynamics(context, positions[t], velocities[t], accelerations[t], inertial)
            if not np.all(np.isfinite(tau)):
                raise FloatingPointError("RNEA returned non-finite torque.")
            rows.append(tau)
    return np.asarray(rows, dtype=np.float64)


def _assemble_inertial_delta_regressor(
    positions: np.ndarray,
    velocities: np.ndarray,
    accelerations: np.ndarray,
    entries: Sequence[SysIdParameterEntry],
    theta: torch.Tensor,
    theta_min: torch.Tensor,
    theta_max: torch.Tensor,
    baseline: ParameterBaselineState,
    context: AnalyticalDynamicsContext,
    inertial_columns: Sequence[int],
    warnings: list[str] | None,
) -> tuple[np.ndarray, np.ndarray, bool]:
    theta_np = theta.detach().cpu().numpy().astype(np.float64)
    regressor = np.zeros((positions.shape[0] * positions.shape[1], len(entries)), dtype=np.float64)
    initial_tau = np.zeros((positions.shape[0], positions.shape[1]), dtype=np.float64)
    validation_warnings = _validate_inertial_context_inputs(context, baseline, positions, velocities, accelerations)
    if validation_warnings:
        if warnings is not None:
            warnings.extend(validation_warnings)
        return regressor, initial_tau, False
    if warnings is not None:
        warnings.extend(_unmapped_inertial_parameter_warnings(entries, context))
    try:
        initial_tau = _inverse_dynamics_for_theta(
            positions, velocities, accelerations, entries, theta_np, baseline, context
        )
    except (FloatingPointError, ValueError, np.linalg.LinAlgError) as exc:
        if warnings is not None:
            warnings.append(
                "Analytical presolve skipped inertial columns because inverse dynamics produced "
                f"non-finite values at the initial theta: {exc}"
            )
        return regressor, initial_tau, False
    initial_flat = initial_tau.reshape(-1)
    for p_idx in inertial_columns:
        step = _theta_step(theta_np, theta_min, theta_max, p_idx)
        if step == 0.0:
            if warnings is not None:
                warnings.append(
                    f"Analytical presolve could not perturb {entries[p_idx].short_type_label()} within bounds."
                )
            continue
        trial = theta_np.copy()
        trial[p_idx] += step
        try:
            trial_tau = _inverse_dynamics_for_theta(
                positions, velocities, accelerations, entries, trial, baseline, context
            )
        except (FloatingPointError, ValueError, np.linalg.LinAlgError) as exc:
            if warnings is not None:
                warnings.append(
                    "Analytical presolve skipped "
                    f"{entries[p_idx].short_type_label()} because inverse dynamics produced non-finite values: {exc}"
                )
            continue
        regressor[:, p_idx] = (trial_tau.reshape(-1) - initial_flat) / step
    return regressor, initial_tau, True


def _diagnostic_rank_condition(regressor: np.ndarray, columns: Sequence[int]) -> tuple[int, float]:
    if not columns:
        return 0, float("inf")
    finite_regressor = np.where(np.isfinite(regressor), regressor, 0.0)
    column_norms = np.linalg.norm(finite_regressor, axis=0)
    max_norm = float(np.max(column_norms[list(columns)])) if columns else 0.0
    norm_tol = max(IDENTIFIABILITY_TOL, max_norm * 1e-8)
    active = [idx for idx in columns if column_norms[idx] > norm_tol]
    if not active:
        return 0, float("inf")
    return _rank_and_condition(finite_regressor[:, active])


def _joint_body_paths(prim: Usd.Prim) -> tuple[str | None, str | None]:
    paths: list[str | None] = []
    for rel_name in ("physics:body0", "physics:body1"):
        rel = prim.GetRelationship(rel_name)
        targets = rel.GetTargets() if rel else []
        paths.append(str(targets[0]) if targets else None)
    return paths[0], paths[1]


def _usd_joint_edge_from_prim(
    prim: Usd.Prim,
    joint_index: int,
    parent_path: str | None,
    child_path: str,
) -> UsdJointEdge | None:
    from pxr import UsdPhysics

    if prim.IsA(UsdPhysics.RevoluteJoint):
        joint_type = "revolute"
    elif prim.IsA(UsdPhysics.PrismaticJoint):
        joint_type = "prismatic"
    else:
        return None
    base_joint = UsdPhysics.Joint(prim)
    try:
        local0 = usd_joint_local_transform(base_joint, 0)
        local1 = usd_joint_local_transform(base_joint, 1)
    except Exception:
        return None
    return UsdJointEdge(
        joint_path=str(prim.GetPath()),
        joint_index=joint_index,
        joint_type=joint_type,
        axis=_joint_axis(prim),
        body0_path=str(parent_path or ""),
        body1_path=str(child_path),
        local0=local0,
        local1=local1,
    )


def _joint_motion_subspace(prim: Usd.Prim, edge: UsdJointEdge) -> np.ndarray | None:
    """Joint motion subspace expressed in the child *body* frame.

    The axis token names a direction in the joint frame, but the RNEA state and
    inertial parameters live in the child body frame, and USD joints author a
    child-side joint frame (``localRot1``/``localPos1``) that is generally
    rotated and offset from the body frame. The subspace therefore carries the
    axis through ``R1`` and, for revolute joints, picks up the ``p1 x axis``
    linear coupling of a rotation axis that does not pass through the body
    origin.

    Args:
        prim: Value supplied for ``prim``.
        edge: Value supplied for ``edge``.

    Returns:
        Result produced by the operation.
    """
    from pxr import UsdPhysics

    axis = _joint_axis(prim)
    axis_index = {"X": 0, "Y": 1, "Z": 2}.get(axis)
    if axis_index is None:
        return None
    axis_joint = np.zeros(3, dtype=np.float64)
    axis_joint[axis_index] = 1.0
    local1 = np.asarray(edge.local1, dtype=np.float64).reshape(4, 4)
    # Row-vector convention: column-convention joint->body rotation is the
    # transpose of the upper 3x3 block; the joint origin sits at the bottom row.
    rotation = local1[:3, :3].T
    origin = local1[3, :3]
    axis_body = rotation @ axis_joint
    subspace = np.zeros(6, dtype=np.float64)
    if prim.IsA(UsdPhysics.RevoluteJoint):
        subspace[:3] = axis_body
        subspace[3:] = np.cross(origin, axis_body)
    elif prim.IsA(UsdPhysics.PrismaticJoint):
        subspace[3:] = axis_body
    else:
        return None
    return subspace


def _joint_axis(prim: Usd.Prim) -> str:
    from pxr import UsdPhysics

    if prim.IsA(UsdPhysics.RevoluteJoint):
        value = UsdPhysics.RevoluteJoint(prim).GetAxisAttr().Get()
    elif prim.IsA(UsdPhysics.PrismaticJoint):
        value = UsdPhysics.PrismaticJoint(prim).GetAxisAttr().Get()
    else:
        value = None
    return str(value or "Z").upper()


def _baseline_friction(baseline: ParameterBaselineState, dof_index: int) -> float:
    value = baseline.joint_dynamic_friction.get(dof_index, 0.0) or baseline.joint_friction.get(dof_index, 0.0)
    return max(float(value), JOINT_FRICTION_BASELINE_FLOOR)


def _baseline_damping(baseline: ParameterBaselineState, dof_index: int) -> float:
    return max(float(baseline.joint_damping.get(dof_index, 1.0)), JOINT_GAIN_BASELINE_FLOOR)


def _baseline_stiffness(baseline: ParameterBaselineState, dof_index: int) -> float:
    return max(float(baseline.joint_stiffness.get(dof_index, 1.0)), JOINT_GAIN_BASELINE_FLOOR)


def _baseline_inverse_dynamics(
    trajectory: TrajectoryDataset,
    steps: int,
    baseline: ParameterBaselineState,
    context: AnalyticalDynamicsContext | None,
    warnings: list[str],
    *,
    accelerations: np.ndarray | None = None,
) -> np.ndarray | None:
    if context is None:
        return None
    inertial = _baseline_inertial_params(context, baseline, include_unmapped_subtrees=True)
    q = np.asarray(trajectory.positions[:steps], dtype=np.float64)
    qd = np.asarray(trajectory.velocities[:steps], dtype=np.float64)
    if accelerations is not None:
        qdd = np.asarray(accelerations[:steps], dtype=np.float64)
    else:
        qdd = finite_difference_acceleration(np.asarray(trajectory.times[:steps], dtype=np.float64), qd)
    if q.shape[1] != context.num_dof or qd.shape[1] != context.num_dof or qdd.shape[1] != context.num_dof:
        warnings.append(
            "Analytical presolve skipped baseline inverse dynamics because trajectory DOFs "
            f"({q.shape[1]}) do not match analytical context DOFs ({context.num_dof})."
        )
        return None
    validation_warnings = _validate_inertial_context_inputs(context, baseline, q, qd, qdd)
    if validation_warnings:
        warnings.extend(validation_warnings)
        return None
    rows = []
    try:
        with np.errstate(all="raise"):
            for t in range(steps):
                tau = rnea_inverse_dynamics(context, q[t], qd[t], qdd[t], inertial)
                if not np.all(np.isfinite(tau)):
                    warnings.append(
                        "Analytical presolve skipped baseline inverse dynamics because RNEA returned NaN/Inf."
                    )
                    return None
                rows.append(tau)
    except (FloatingPointError, ValueError) as exc:
        warnings.append(f"Analytical presolve skipped baseline inverse dynamics because RNEA failed: {exc}")
        return None
    return np.asarray(rows, dtype=np.float64)


def _baseline_inertial_params(
    context: AnalyticalDynamicsContext,
    baseline: ParameterBaselineState,
    *,
    include_unmapped_subtrees: bool = False,
) -> np.ndarray:
    """Per-context-link ``[m, h, I]`` rows from the captured baseline.

    With ``include_unmapped_subtrees`` each recorded unmapped distal body (e.g.
    an attached gripper) is composited rigidly into its mapped parent row:
    summed mass, mass-weighted CoM, and the rotated inertia transported to the
    parent origin via the parallel-axis theorem. Without it those bodies are
    absent, which understates gravity torque on the proximal joints.

    Args:
        context: Runtime context used by the operation.
        baseline: Baseline model parameters.
        include_unmapped_subtrees: Value supplied for ``include_unmapped_subtrees``.

    Returns:
        Result produced by the operation.
    """
    params = np.zeros((context.num_links, 10), dtype=np.float64)
    for ctx_idx, source_idx in enumerate(_context_source_link_indices(context)):
        mass = max(float(baseline.link_masses.get(source_idx, 1.0)), 1e-9)
        com = np.asarray(baseline.link_com.get(source_idx, np.zeros(3)), dtype=np.float64).reshape(3)
        inertia_lc = baseline.link_inertia_lc.get(source_idx)
        if inertia_lc is not None and np.all(np.isfinite(inertia_lc)):
            try:
                inertia = log_cholesky_to_inertia_matrix(inertia_lc)
            except ValueError:
                inertia = np.eye(3, dtype=np.float64) * mass
        else:
            inertia = np.eye(3, dtype=np.float64) * mass
        if not np.all(np.isfinite(inertia)):
            inertia = np.eye(3, dtype=np.float64) * mass
        inertia = _origin_referenced_inertia(inertia, mass, com)
        params[ctx_idx, 0] = mass
        params[ctx_idx, 1:4] = mass * com
        # Symmetrized off-diagonals, matching `_inertial_params_for_theta`.
        params[ctx_idx, 4] = inertia[0, 0]
        params[ctx_idx, 5] = 0.5 * (inertia[0, 1] + inertia[1, 0])
        params[ctx_idx, 6] = 0.5 * (inertia[0, 2] + inertia[2, 0])
        params[ctx_idx, 7] = inertia[1, 1]
        params[ctx_idx, 8] = 0.5 * (inertia[1, 2] + inertia[2, 1])
        params[ctx_idx, 9] = inertia[2, 2]
    if include_unmapped_subtrees:
        _accumulate_unmapped_subtree_params(params, context, baseline)
    return params


def _accumulate_unmapped_subtree_params(
    params: np.ndarray,
    context: AnalyticalDynamicsContext,
    baseline: ParameterBaselineState,
) -> None:
    """Composite each recorded unmapped distal body into its mapped parent row in place.

    Rigid-composite math: summed mass, mass-weighted CoM, and the rotated inertia
    transported to the parent origin via the parallel-axis theorem.

    Args:
        params: Value supplied for ``params``.
        context: Runtime context used by the operation.
        baseline: Baseline model parameters.
    """
    for lump in context.unmapped_subtree_links:
        ctx_idx = int(lump.context_link_index)
        if not (0 <= ctx_idx < context.num_links):
            continue
        source_idx = int(lump.source_link_index)
        mass = float(baseline.link_masses.get(source_idx, 0.0))
        if not np.isfinite(mass) or mass <= 0.0:
            continue
        rotation = np.asarray(lump.rotation, dtype=np.float64).reshape(3, 3)
        com_local = np.asarray(baseline.link_com.get(source_idx, np.zeros(3)), dtype=np.float64).reshape(3)
        com = rotation @ com_local + np.asarray(lump.translation, dtype=np.float64).reshape(3)
        if not np.all(np.isfinite(com)):
            continue
        inertia_lc = baseline.link_inertia_lc.get(source_idx)
        inertia = np.zeros((3, 3), dtype=np.float64)
        if inertia_lc is not None and np.all(np.isfinite(inertia_lc)):
            try:
                candidate = log_cholesky_to_inertia_matrix(inertia_lc)
                if np.all(np.isfinite(candidate)):
                    inertia = candidate
            except ValueError:
                pass
        shifted = _origin_referenced_inertia(rotation @ inertia @ rotation.T, mass, com)
        params[ctx_idx, 0] += mass
        params[ctx_idx, 1:4] += mass * com
        params[ctx_idx, 4] += shifted[0, 0]
        params[ctx_idx, 5] += 0.5 * (shifted[0, 1] + shifted[1, 0])
        params[ctx_idx, 6] += 0.5 * (shifted[0, 2] + shifted[2, 0])
        params[ctx_idx, 7] += shifted[1, 1]
        params[ctx_idx, 8] += 0.5 * (shifted[1, 2] + shifted[2, 1])
        params[ctx_idx, 9] += shifted[2, 2]


def _rank_and_condition(matrix: np.ndarray) -> tuple[int, float]:
    if matrix.size == 0:
        return 0, float("inf")
    if not np.all(np.isfinite(matrix)):
        return 0, float("inf")
    singular = np.linalg.svd(matrix, compute_uv=False)
    if singular.size == 0:
        return 0, float("inf")
    tol = max(matrix.shape) * np.finfo(np.float64).eps * float(singular[0])
    rank = int(np.sum(singular > tol))
    condition = float(singular[0] / singular[-1]) if singular[-1] > 0.0 else float("inf")
    return rank, condition


def _append_correlation_warnings(
    matrix: np.ndarray,
    active_indices: Sequence[int],
    entries: Sequence[SysIdParameterEntry],
    warnings: list[str],
) -> None:
    if matrix.shape[1] < 2:
        return
    norms = np.linalg.norm(matrix, axis=0)
    safe = np.maximum(norms, IDENTIFIABILITY_TOL)
    corr = np.abs((matrix.T @ matrix) / np.outer(safe, safe))
    for i in range(corr.shape[0]):
        for j in range(i + 1, corr.shape[1]):
            if corr[i, j] > CORRELATION_WARN_THRESHOLD:
                left = entries[active_indices[i]].short_type_label()
                right = entries[active_indices[j]].short_type_label()
                warnings.append(
                    f"Analytical presolve columns are highly correlated ({left}, {right}: {corr[i, j]:.3f})."
                )


def _empty_result(
    theta_seed: torch.Tensor,
    identifiable: list[bool],
    warnings: list[str],
    regressor: np.ndarray | None = None,
    *,
    target_source: str = "unavailable",
    active_column_count: int = 0,
    family_seeded_counts: dict[str, int] | None = None,
    full_rank: int = 0,
    full_condition: float = float("inf"),
    gravity_rank: int = 0,
    gravity_condition: float = float("inf"),
    torque_equation_excluded: list[bool] | None = None,
) -> AnalyticalPresolveResult:
    column_norms = np.linalg.norm(regressor, axis=0).tolist() if regressor is not None else []
    return AnalyticalPresolveResult(
        theta_seed=theta_seed,
        identifiable=identifiable,
        rank=0,
        condition=float("inf"),
        column_norms=column_norms,
        warnings=warnings,
        target_source=target_source,
        active_column_count=active_column_count,
        family_seeded_counts=family_seeded_counts or {},
        full_rank=full_rank,
        full_condition=full_condition,
        gravity_rank=gravity_rank,
        gravity_condition=gravity_condition,
        torque_equation_excluded=torque_equation_excluded or [],
    )
