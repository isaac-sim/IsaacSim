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

"""Pre-solve check report: telemetry quality plus per-parameter identifiability.

Combines the telemetry-quality diagnostics and the analytical identifiability
presolve into one report a user (or the UI's Check stage) reads *before* burning
a solve. The report is purely diagnostic: it never mutates the run configuration
and never raises — hard errors remain :mod:`preflight`'s job.
"""

from __future__ import annotations

import math
import time
from dataclasses import asdict, dataclass, field
from typing import Any

import carb
import numpy as np

from .analytical_presolve import (
    build_fixed_base_context_from_usd,
    run_analytical_presolve,
)
from .parameter_types import SysIdParameterEntry, SysIdParameterType
from .regressor import finite_difference_acceleration
from .telemetry_quality import TelemetryQualityReport, build_telemetry_quality_report
from .trajectory_csv import TrajectoryDataset
from .trajectory_segments import TrajectoryChunk

SYSID_CHECK_REPORT_SCHEMA_VERSION = 1

VERDICT_IDENTIFIABLE = "identifiable"
VERDICT_WEAK = "weak"
VERDICT_NOT_IDENTIFIABLE = "not_identifiable"
VERDICT_UNKNOWN = "unknown"

_WEAK_COLUMN_NORM_EPS = 1e-10

# Multiplier-style parameters apply value = theta * baseline; a zero captured
# baseline makes the rollout bit-exactly independent of theta for any theta.
_ZERO_BASELINE_EPS = 1e-9


@dataclass(frozen=True)
class SysIdCheckIssue:
    """One check finding (the gate never emits errors; hard stops stay in preflight)."""

    code: str
    message: str
    severity: str = "warning"

    def to_dict(self) -> dict[str, Any]:  # noqa: D102
        return asdict(self)


@dataclass
class ParameterIdentifiabilityVerdict:
    """Identifiability assessment for one selected parameter."""

    index: int
    name: str
    param_type: str
    verdict: str
    dof_index: int = -1
    column_norm: float | None = None
    notes: list[str] = field(default_factory=list)

    def to_dict(self) -> dict[str, Any]:  # noqa: D102
        return asdict(self)


@dataclass
class SysIdCheckReport:
    """Combined pre-solve diagnostics for one run configuration."""

    schema_version: int
    ok: bool
    telemetry_quality: dict[str, Any]
    presolve: dict[str, Any] | None
    parameter_verdicts: list[ParameterIdentifiabilityVerdict]
    issues: list[SysIdCheckIssue]

    #: Live object behind `telemetry_quality`, so later stages can reuse the
    #: assessment instead of repeating it. Deliberately absent from `to_dict()`.
    telemetry_quality_report: TelemetryQualityReport | None = None

    def to_dict(self) -> dict[str, Any]:  # noqa: D102
        return {
            "schema_version": self.schema_version,
            "ok": self.ok,
            "telemetry_quality": dict(self.telemetry_quality),
            "presolve": dict(self.presolve) if self.presolve is not None else None,
            "parameter_verdicts": [verdict.to_dict() for verdict in self.parameter_verdicts],
            "issues": [issue.to_dict() for issue in self.issues],
        }

    def summary_lines(self) -> list[str]:
        """Compact human-readable summary for logs and CLIs.

        Returns:
            Result produced by the operation.
        """
        lines: list[str] = []
        counts: dict[str, int] = {}
        for verdict in self.parameter_verdicts:
            counts[verdict.verdict] = counts.get(verdict.verdict, 0) + 1
        if self.parameter_verdicts:
            summary = ", ".join(f"{count} {name}" for name, count in sorted(counts.items()))
            lines.append(f"Identifiability: {summary} (of {len(self.parameter_verdicts)} selected parameters)")
            for verdict in self.parameter_verdicts:
                if verdict.verdict in (VERDICT_NOT_IDENTIFIABLE, VERDICT_WEAK):
                    note = f" ({verdict.notes[0]})" if verdict.notes else ""
                    lines.append(f"  {verdict.name}: {verdict.verdict}{note}")
        # Collapse repeated issue kinds (large robots can emit dozens of near-identical
        # per-link warnings); show the first message as the representative. Presolve
        # warnings embed their own kind prefix before the first colon, so group on it.
        by_kind: dict[tuple[str, str], list[SysIdCheckIssue]] = {}
        for issue in self.issues:
            kind = issue.code
            if issue.code == "presolve_warning":
                kind = f"presolve_{issue.message.split(':', 1)[0].split('(', 1)[0].strip()[:48]}"
            by_kind.setdefault((issue.severity, kind), []).append(issue)
        for (severity, kind), group in by_kind.items():
            if len(group) <= 3:
                lines.extend(f"[{severity}] {issue.message}" for issue in group)
            else:
                lines.append(f"[{severity}] {kind} (x{len(group)}): {group[0].message}")
        return lines


def build_sysid_check_report(
    spec,  # noqa: ANN001
    *,
    stage=None,  # noqa: ANN001
    trajectory: TrajectoryDataset,
    train_chunks: list[TrajectoryChunk] | None = None,
    validation_chunks: list[TrajectoryChunk] | None = None,
) -> SysIdCheckReport:
    """Build the combined telemetry-quality + identifiability check report.

    Args:
        spec: The run spec (``SysIdRunSpec``) describing the intended solve.
        stage: Optional USD stage; without it the identifiability presolve is
            skipped and all verdicts are ``unknown`` ("could not assess").
        trajectory: Loaded telemetry.
        train_chunks: Optional pre-built training chunks. All training slices are
            stacked for the presolve while acceleration is estimated independently
            inside each chunk.
        validation_chunks: Optional pre-built validation chunks.

    Returns:
        Result produced by the operation.
    """  # noqa: DOC107
    issues: list[SysIdCheckIssue] = []
    started = time.perf_counter()
    chunk_specs = [chunk.spec for chunk in (train_chunks or []) + (validation_chunks or [])]
    carb.log_info(f"SysId check: assessing telemetry quality ({int(trajectory.times.shape[0])} samples)")
    quality = build_telemetry_quality_report(trajectory, chunk_specs or (spec.telemetry.chunks or None))
    carb.log_info(f"SysId check: telemetry quality done in {time.perf_counter() - started:.2f} s")
    for issue in quality.issues:
        issues.append(SysIdCheckIssue(code=f"telemetry_{issue.code}", message=issue.message, severity=issue.severity))

    verdicts = _initial_verdicts(spec, trajectory)
    presolve_payload = None
    if stage is not None and verdicts:
        presolve_started = time.perf_counter()
        carb.log_info(f"SysId check: assessing identifiability of {len(verdicts)} parameters")
        presolve_payload = _run_identifiability_presolve(spec, stage, trajectory, train_chunks, verdicts, issues)
        carb.log_info(f"SysId check: identifiability done in {time.perf_counter() - presolve_started:.2f} s")
    elif verdicts:
        for verdict in verdicts:
            verdict.notes.append("No USD stage available; identifiability could not be assessed.")
        issues.append(
            SysIdCheckIssue(
                code="identifiability_unavailable",
                message="No USD stage was available; selected-parameter identifiability could not be assessed.",
            )
        )

    _cross_reference_weak_excitation(quality, verdicts)

    telemetry_error = any(issue.severity == "error" for issue in quality.issues)
    any_not_identifiable = any(verdict.verdict == VERDICT_NOT_IDENTIFIABLE for verdict in verdicts)
    any_unknown = any(verdict.verdict == VERDICT_UNKNOWN for verdict in verdicts)
    carb.log_info(f"SysId check: complete in {time.perf_counter() - started:.2f} s")
    return SysIdCheckReport(
        schema_version=SYSID_CHECK_REPORT_SCHEMA_VERSION,
        ok=not (telemetry_error or any_not_identifiable or any_unknown),
        telemetry_quality=quality.to_dict(),
        presolve=presolve_payload,
        parameter_verdicts=verdicts,
        issues=issues,
        telemetry_quality_report=quality,
    )


def _initial_verdicts(spec, trajectory: TrajectoryDataset) -> list[ParameterIdentifiabilityVerdict]:  # noqa: ANN001
    verdicts: list[ParameterIdentifiabilityVerdict] = []
    num_joints = int(trajectory.num_joints)
    for index, item in enumerate(spec.parameters.selected or []):
        entry = item.to_entry()
        try:
            name = entry.display_name(num_joints)
        except Exception:
            name = entry.param_type.value
        verdicts.append(
            ParameterIdentifiabilityVerdict(
                index=index,
                name=name,
                param_type=entry.param_type.value,
                verdict=VERDICT_UNKNOWN,
                dof_index=int(entry.dof_index),
            )
        )
    return verdicts


def _multiplier_baseline_values(entry: SysIdParameterEntry, baseline) -> tuple[str, list[float]] | None:  # noqa: ANN001
    """Return (baseline label, captured baseline values ``entry`` multiplies), or None if absolute-style.

    An empty value list means the baseline for this entry was never captured
    (e.g. an unreadable joint snapshot) — the caller must skip, not flag.

    Args:
        entry: Value supplied for ``entry``.
        baseline: Baseline model parameters.

    Returns:
        Result produced by the operation.
    """  # noqa: DOC107

    def _by_dof(mapping: dict[int, float]) -> list[float]:
        if entry.dof_index >= 0:
            return [mapping[entry.dof_index]] if entry.dof_index in mapping else []
        return list(mapping.values())

    def _by_link(mapping: dict[int, float]) -> list[float]:
        if entry.link_index >= 0:
            return [mapping[entry.link_index]] if entry.link_index in mapping else []
        return list(mapping.values())

    ptype = entry.param_type
    if ptype == SysIdParameterType.JOINT_FRICTION:
        # The friction multiplier scales the friction/static/dynamic trio, so a
        # joint is only a no-op when every member of the trio is zero.
        dofs = [entry.dof_index] if entry.dof_index >= 0 else sorted(baseline.joint_friction)
        values = [
            max(
                abs(baseline.joint_friction[dof]),
                abs(baseline.joint_static_friction.get(dof, 0.0)),
                abs(baseline.joint_dynamic_friction.get(dof, 0.0)),
            )
            for dof in dofs
            if dof in baseline.joint_friction
        ]
        return "joint friction", values
    if ptype == SysIdParameterType.JOINT_STIFFNESS:
        return "drive stiffness", _by_dof(baseline.joint_stiffness)
    if ptype == SysIdParameterType.JOINT_DAMPING:
        return "drive damping", _by_dof(baseline.joint_damping)
    if ptype == SysIdParameterType.LINK_MASS:
        return "link mass", _by_link(baseline.link_masses)
    if ptype == SysIdParameterType.JOINT_LIMIT_LOWER_SCALE:
        return "lower joint limit", _by_dof(baseline.joint_lower_limits)
    if ptype == SysIdParameterType.JOINT_LIMIT_UPPER_SCALE:
        return "upper joint limit", _by_dof(baseline.joint_upper_limits)
    return None


def apply_zero_baseline_verdicts(
    verdicts: list[ParameterIdentifiabilityVerdict],
    entries: list[SysIdParameterEntry],
    baseline,  # noqa: ANN001
    issues: list[SysIdCheckIssue] | None = None,
) -> None:
    """Mark multiplier-style parameters whose captured baseline is zero as not identifiable.

    Multiplier parameterizations apply ``value = theta * baseline``, so a baseline of 0
    (e.g. a USD with no authored joint friction) makes the rollout independent of theta
    and no optimizer can move the parameter. The analytical presolve cannot see this —
    its regressor columns describe the physical quantity, not the encoding — so this
    rule overrides its verdict. ``verdicts`` and ``entries`` are index-aligned
    (both follow ``spec.parameters.selected`` order); ``baseline`` is a
    ``ParameterBaselineState``.

    Args:
        verdicts: Value supplied for ``verdicts``.
        entries: Parameter entries handled by the operation.
        baseline: Baseline model parameters.
        issues: Validation issues accumulated by the operation.
    """  # noqa: DOC107
    for verdict, entry in zip(verdicts, entries):
        resolved = _multiplier_baseline_values(entry, baseline)
        if resolved is None:
            continue
        label, values = resolved
        if not values or any(abs(value) > _ZERO_BASELINE_EPS for value in values):
            continue
        verdict.verdict = VERDICT_NOT_IDENTIFIABLE
        note = (
            f"Zero baseline: this parameter is a multiplier on the authored {label}, which is 0, "
            f"so it has no effect on the rollout for any value. Author a nonzero {label} in the "
            "USD or use an absolute parameterization."
        )
        verdict.notes.append(note)
        if issues is not None:
            issues.append(SysIdCheckIssue(code="zero_baseline_parameter", message=f"{verdict.name}: {note}"))


def _stack_presolve_training_data(
    trajectory: TrajectoryDataset,
    train_chunks: list[TrajectoryChunk] | None,
    *,
    max_steps_per_chunk: int,
) -> tuple[TrajectoryDataset, np.ndarray | None]:
    """Stack training chunks without differentiating velocity across chunk boundaries.

    Args:
        trajectory: Fallback trajectory when explicit training chunks are absent.
        train_chunks: Explicit training chunks to stack.
        max_steps_per_chunk: Maximum samples retained from each chunk.

    Returns:
        Stacked training trajectory and independently computed acceleration rows,
        or the fallback trajectory and ``None`` when stacking is unnecessary.
    """
    chunks = list(train_chunks or [])
    if len(chunks) <= 1:
        return (chunks[0].trajectory if chunks else trajectory), None

    selected: list[tuple[TrajectoryDataset, int]] = []
    for chunk in chunks:
        chunk_trajectory = chunk.trajectory
        count = min(int(chunk_trajectory.positions.shape[0]), max(2, int(max_steps_per_chunk)))
        selected.append((chunk_trajectory, count))

    time_parts: list[np.ndarray] = []
    position_parts: list[np.ndarray] = []
    velocity_parts: list[np.ndarray] = []
    command_parts: list[np.ndarray] = []
    acceleration_parts: list[np.ndarray] = []
    previous_end = 0.0
    for index, (chunk_trajectory, count) in enumerate(selected):
        raw_times = np.asarray(chunk_trajectory.times[:count], dtype=np.float64).reshape(-1)
        local_times = raw_times - raw_times[0]
        if index:
            positive_steps = np.diff(local_times)
            positive_steps = positive_steps[positive_steps > 0.0]
            boundary_step = float(np.median(positive_steps)) if positive_steps.size else 1e-6
            local_times = local_times + previous_end + max(boundary_step, 1e-12)
        previous_end = float(local_times[-1])
        time_parts.append(local_times)
        position_parts.append(np.asarray(chunk_trajectory.positions[:count], dtype=np.float64))
        velocities = np.asarray(chunk_trajectory.velocities[:count], dtype=np.float64)
        velocity_parts.append(velocities)
        command_parts.append(np.asarray(chunk_trajectory.commands[:count], dtype=np.float64))
        acceleration_parts.append(finite_difference_acceleration(raw_times, velocities))

    def optional_parts(name: str) -> np.ndarray | None:
        values = [getattr(chunk_trajectory, name, None) for chunk_trajectory, _count in selected]
        if any(value is None for value in values):
            return None
        return np.concatenate(
            [np.asarray(value[:count], dtype=np.float64) for value, (_chunk, count) in zip(values, selected)],
            axis=0,
        )

    stacked = TrajectoryDataset(
        times=np.concatenate(time_parts, axis=0),
        positions=np.concatenate(position_parts, axis=0),
        velocities=np.concatenate(velocity_parts, axis=0),
        commands=np.concatenate(command_parts, axis=0),
        metadata=trajectory.metadata,
        torques=optional_parts("torques"),
        end_effector_poses=optional_parts("end_effector_poses"),
        contact_forces=optional_parts("contact_forces"),
        residual_sample_weights=optional_parts("residual_sample_weights"),
    )
    return stacked, np.concatenate(acceleration_parts, axis=0)


def _run_identifiability_presolve(
    spec,  # noqa: ANN001
    stage,  # noqa: ANN001
    trajectory: TrajectoryDataset,
    train_chunks: list[TrajectoryChunk] | None,
    verdicts: list[ParameterIdentifiabilityVerdict],
    issues: list[SysIdCheckIssue],
) -> dict[str, Any] | None:
    # Imported here: check_report is imported by run_controller (function-local),
    # and these helpers live in run_controller.
    from .run_controller import build_parameter_space, optimizer_vectors

    parameter_space = None
    entries = None
    try:
        parameter_space, link_paths, joint_paths = build_parameter_space(spec, stage, trajectory)
        entries, theta_initial, theta_min, theta_max = optimizer_vectors(spec)
        context = build_fixed_base_context_from_usd(
            stage,
            link_paths,
            joint_paths[: trajectory.num_joints],
            parameter_space.baseline,
        )
        max_steps_per_chunk = max(2, int(spec.solver.max_rollout_steps))
        presolve_trajectory, presolve_accelerations = _stack_presolve_training_data(
            trajectory,
            train_chunks,
            max_steps_per_chunk=max_steps_per_chunk,
        )
        presolve_max_steps = (
            int(presolve_trajectory.positions.shape[0])
            if train_chunks and len(train_chunks) > 1
            else max_steps_per_chunk
        )
        result = run_analytical_presolve(
            presolve_trajectory,
            entries,
            theta_initial,
            theta_min,
            theta_max,
            parameter_space.baseline,
            residual_config=spec.residuals.to_config(),
            dynamics_context=context,
            max_steps=presolve_max_steps,
            accelerations=presolve_accelerations,
        )
    except Exception as exc:
        issues.append(SysIdCheckIssue(code="presolve_failed", message=f"Identifiability presolve failed: {exc}"))
        for verdict in verdicts:
            verdict.notes.append("Identifiability presolve failed; could not assess.")
        if parameter_space is not None and entries is not None:
            apply_zero_baseline_verdicts(verdicts, entries, parameter_space.baseline, issues)
        return None

    assessable = result.target_source != "unavailable" and result.active_column_count > 0
    for index, verdict in enumerate(verdicts):
        column_norm = float(result.column_norms[index]) if index < len(result.column_norms) else None
        verdict.column_norm = column_norm if column_norm is not None and math.isfinite(column_norm) else None
        if not assessable:
            verdict.notes.append(
                "Identifiability could not be assessed (no torque telemetry or fixed-base dynamics context)."
            )
            continue
        excluded = (
            bool(result.torque_equation_excluded[index]) if index < len(result.torque_equation_excluded) else False
        )
        if excluded:
            # Not a lack of excitation: link-side measured torque equals the drive output
            # at the true parameters, so drive gains are structurally absent from the
            # presolve torque equation. The rollout optimizer identifies them in the
            # position domain; leave the verdict unknown rather than misreport them.
            verdict.notes.append(
                "Not assessable from link-side measured torque; identified in the position domain during the solve."
            )
            continue
        identifiable = bool(result.identifiable[index]) if index < len(result.identifiable) else False
        if identifiable:
            verdict.verdict = VERDICT_IDENTIFIABLE
        elif verdict.column_norm is not None and verdict.column_norm > _WEAK_COLUMN_NORM_EPS:
            verdict.verdict = VERDICT_WEAK
            verdict.notes.append("Weak regressor excitation for this parameter.")
        elif verdict.column_norm is not None:
            verdict.verdict = VERDICT_NOT_IDENTIFIABLE
            verdict.notes.append("The recorded motion carries no information about this parameter.")
        else:
            verdict.notes.append("No analytical regressor column for this parameter type.")
    # Runs last: a zero baseline is a structural dead end that overrides whatever
    # the analytical regressor concluded for the parameter.
    apply_zero_baseline_verdicts(verdicts, entries, parameter_space.baseline, issues)
    for warning in result.warnings:
        issues.append(SysIdCheckIssue(code="presolve_warning", message=str(warning)))
    feedforward = _recommend_feedforward_mode(
        spec,
        presolve_trajectory,
        context,
        parameter_space.baseline,
        issues,
        accelerations=presolve_accelerations,
    )
    payload = {
        "rank": int(result.rank),
        "condition": float(result.condition),
        "target_source": str(result.target_source),
        "active_column_count": int(result.active_column_count),
        "warnings": [str(warning) for warning in result.warnings],
    }
    if feedforward is not None:
        payload["feedforward"] = feedforward
    return payload


_FEEDFORWARD_MAX_STEPS = 400


def _recommend_feedforward_mode(
    spec,  # noqa: ANN001
    trajectory: TrajectoryDataset,
    context,  # noqa: ANN001
    baseline,  # noqa: ANN001
    issues: list[SysIdCheckIssue],
    *,
    accelerations: np.ndarray | None = None,
) -> dict[str, Any] | None:
    """Compare measured torque with candidate dynamics models without inferring controller architecture.

    Total link-side torque is determined by the observed motion and plant dynamics;
    it does not reveal how a controller decomposed feedback and feedforward. These
    residuals are therefore model-fit diagnostics only. The compact fixed-base RNEA
    context includes unmapped distal subtrees in their mapped parent links.

    Args:
        spec: SysID run specification.
        trajectory: Telemetry trajectory used by the operation.
        context: Runtime context used by the operation.
        baseline: Baseline model parameters.
        issues: Validation issues accumulated by the operation.
        accelerations: Optional precomputed accelerations that preserve chunk boundaries.

    Returns:
        Result produced by the operation.
    """  # noqa: DOC107
    from .analytical_presolve import _baseline_inertial_params, _effort_semantics
    from .regressor import finite_difference_acceleration, rnea_inverse_dynamics

    if context is None:
        return None
    torques = getattr(trajectory, "torques", None)
    if torques is None:
        return None
    if _effort_semantics(trajectory, None) != "link_side":
        return None
    steps = min(int(trajectory.positions.shape[0]), _FEEDFORWARD_MAX_STEPS)
    measured = np.asarray(torques[:steps], dtype=np.float64)
    if measured.size == 0 or not np.all(np.isfinite(measured)):
        return None
    q = np.asarray(trajectory.positions[:steps], dtype=np.float64)
    qd = np.asarray(trajectory.velocities[:steps], dtype=np.float64)
    if q.shape[1] != context.num_dof or measured.shape[1] != context.num_dof:
        return None
    times = np.asarray(trajectory.times[:steps], dtype=np.float64).reshape(-1)
    qdd = (
        np.asarray(accelerations[:steps], dtype=np.float64)
        if accelerations is not None
        else finite_difference_acceleration(times, qd)
    )
    inertial = _baseline_inertial_params(context, baseline, include_unmapped_subtrees=True)
    zeros = np.zeros_like(qd)
    residuals: dict[str, float] = {}
    try:
        for mode, mode_qd, mode_qdd in (
            ("none", None, None),
            ("gravity", zeros, zeros),
            ("inverse_dynamics", qd, qdd),
        ):
            if mode == "none":
                predicted = np.zeros_like(measured)
            else:
                rows = [rnea_inverse_dynamics(context, q[t], mode_qd[t], mode_qdd[t], inertial) for t in range(steps)]
                predicted = np.asarray(rows, dtype=np.float64)
            residuals[mode] = float(np.sqrt(np.mean((measured - predicted) ** 2)))
    except Exception as exc:
        issues.append(
            SysIdCheckIssue(
                code="feedforward_check_failed",
                message=f"Feedforward-mode check skipped: {exc}",
                severity="info",
            )
        )
        return None

    best_fit = min(residuals, key=residuals.get)
    configured = str(getattr(spec.simulation.newton, "feedforward", "") or "").strip().lower()
    if not configured:
        configured = str(getattr(spec.simulation.newton, "feedforward", "none") or "none")
    return {
        "residual_rms": residuals,
        "best_fit": best_fit,
        "recommended": None,
        "configured": configured,
        "note": (
            "Model-fit residuals cannot identify controller feedback/feedforward decomposition; "
            "configure feedforward from controller metadata."
        ),
    }


def _cross_reference_weak_excitation(quality, verdicts: list[ParameterIdentifiabilityVerdict]) -> None:  # noqa: ANN001
    weak_joints = {int(idx) for idx in quality.weak_excitation_joint_indices}
    if not weak_joints:
        return
    joint_types = ("joint_friction", "joint_stiffness", "joint_damping", "joint_armature", "joint_integral_gain")
    for verdict in verdicts:
        if verdict.param_type not in joint_types or verdict.dof_index < 0:
            continue
        if verdict.dof_index in weak_joints:
            if verdict.verdict == VERDICT_IDENTIFIABLE:
                verdict.verdict = VERDICT_WEAK
            verdict.notes.append("Telemetry shows weak excitation on this joint.")


__all__ = [
    "SYSID_CHECK_REPORT_SCHEMA_VERSION",
    "ParameterIdentifiabilityVerdict",
    "SysIdCheckIssue",
    "SysIdCheckReport",
    "apply_zero_baseline_verdicts",
    "build_sysid_check_report",
]
