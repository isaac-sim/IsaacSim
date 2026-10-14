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

"""Machine-readable tuning assessment for SysID solve reports."""

from __future__ import annotations

import math
from collections.abc import Iterable
from dataclasses import asdict, dataclass, field
from enum import Enum
from typing import Any

import numpy as np

from .parameter_types import SysIdParameterType

# 2: `validation_improvement` gained `unpaired_channels`, and each overall
#    channel gained `unpaired_chunks`.
TUNING_ASSESSMENT_SCHEMA_VERSION = 2

ASSESSMENT_PASS = "pass"
ASSESSMENT_WARN = "warn"
ASSESSMENT_FAIL = "fail"
ASSESSMENT_UNAVAILABLE = "unavailable"


class _NextParameterRecommendation(str, Enum):
    """Stable recommendation codes for selecting the next parameter set."""

    REDUCE_OR_TIGHTEN = "reduce_selected_parameter_families_or_tighten_bounds"
    ADD_VALIDATION = "add_independent_validation_chunks_before_broadening_parameters"
    RERUN_SMALLER_SET = "rerun_with_smaller_parameter_set"
    CONSIDER_ARMATURE = "consider_joint_armature_only_after_validation_improves"
    CONSIDER_INERTIA = "consider_link_mass_or_inertia_only_with_torque_and_rich_acceleration_data"
    VALIDATE_MORE_HOLDOUTS = "keep_current_parameter_set_and_validate_on_more_holdouts"


class _RecommendedAction(str, Enum):
    """Stable action codes emitted by tuning assessments."""

    RESOLVE_ERRORS = "do_not_save_as_final_tuned_usd_until_errors_are_resolved"
    INSPECT_OVERLAYS = "inspect_validation_overlay_pngs_for_timing_and_amplitude_match"
    REVIEW_BOUND_HITS = "review_bounds_and_physical_plausibility_for_bound_hit_parameters"
    REVIEW_LARGE_DELTAS = "review_large_parameter_deltas_for_coupling_or_bad_initial_values"
    RUN_ANOTHER_HOLDOUT = "run_next_holdout_or_repeated_trial_before_broadening_parameter_scope"


@dataclass(frozen=True)
class TuningAssessmentIssue:
    """One actionable issue found while reviewing a completed tuning run."""

    code: str
    severity: str
    message: str
    details: dict[str, Any] = field(default_factory=dict)

    def to_dict(self) -> dict[str, Any]:  # noqa: D102
        return asdict(self)


@dataclass
class TuningAssessmentReport:
    """Structured assessment intended for automated pipelines and CI review."""

    schema_version: int
    status: str
    checks: dict[str, dict[str, Any]]
    issues: list[dict[str, Any]]
    metrics: dict[str, Any]
    parameter_summary: dict[str, Any]
    recommended_next_parameters: list[str]
    recommended_actions: list[str]

    def to_dict(self) -> dict[str, Any]:  # noqa: D102
        return asdict(self)


def build_tuning_assessment_report(
    spec,  # noqa: ANN001
    prepared,  # noqa: ANN001
    result,  # noqa: ANN001
    *,
    telemetry_quality=None,  # noqa: ANN001
) -> TuningAssessmentReport:
    """Assess whether a solve is credible enough for automated tuning decisions.

    Args:
        spec: SysID run specification.
        prepared: Prepared SysID run state.
        result: SysID result being processed.
        telemetry_quality: Value supplied for ``telemetry_quality``.

    Returns:
        Result produced by the operation.
    """  # noqa: DOC106, DOC107
    issues: list[TuningAssessmentIssue] = []
    checks: dict[str, dict[str, Any]] = {}
    config = getattr(prepared, "config", None)
    selected = getattr(result, "selected_status", None)
    final = getattr(result, "final_status", None)
    validation_metrics = list(getattr(result, "validation_metrics", []) or [])

    parameter_summary = _parameter_summary(config, selected)
    train_norm = _training_normalized_cost(config, selected)
    # Only a genuine holdout can credibly bound generalization. The training-chunk
    # fallback is evaluated on the fitted data (tagged holdout=False), so it
    # must not feed the train-vs-validation gap -- otherwise the gap reads ~1.0 and
    # masquerades as a clean pass on data the model was fit to.
    holdout_metrics = [m for m in validation_metrics if _is_holdout_metric(m)]
    validation_norm = _mean_metric(holdout_metrics, "normalized_cost")
    validation_ratio = (
        float(validation_norm / max(train_norm, 1e-12)) if _finite(train_norm) and _finite(validation_norm) else None
    )

    _check_finite_solution(checks, issues, selected)
    _check_accepted_solution(checks, issues, final, selected, getattr(result, "selected_status_source", ""))
    _check_bounds(checks, issues, parameter_summary)
    _check_parameter_coupling(
        checks, issues, config, parameter_summary, getattr(prepared, "resampling_diagnostics", [])
    )
    _check_validation_holdout(checks, issues, prepared, result, validation_metrics)
    validation_comparison = dict(getattr(result, "validation_comparison", {}) or {})
    _check_validation_improvement(checks, issues, validation_comparison)
    _check_train_validation_gap(checks, issues, train_norm, validation_norm, validation_ratio)
    _check_excitation_coverage(checks, issues, prepared)
    _check_resampling(checks, issues, getattr(prepared, "resampling_diagnostics", []))
    _check_telemetry_quality(checks, issues, telemetry_quality)
    _check_analytical_presolve(checks, issues, getattr(prepared, "analytical_presolve", None))

    recommended_next_parameters = _recommended_next_parameters(config, issues, validation_metrics)
    recommended_actions = _recommended_actions(issues, validation_metrics, parameter_summary)
    status = _overall_status(issues)
    return TuningAssessmentReport(
        schema_version=TUNING_ASSESSMENT_SCHEMA_VERSION,
        status=status,
        checks=checks,
        issues=[issue.to_dict() for issue in issues],
        metrics={
            "training_normalized_cost": train_norm,
            "validation_normalized_cost": validation_norm,
            "validation_to_training_cost_ratio": validation_ratio,
            "validation_metric_count": len(validation_metrics),
            "validation_comparison": validation_comparison,
        },
        parameter_summary=parameter_summary,
        recommended_next_parameters=recommended_next_parameters,
        recommended_actions=recommended_actions,
    )


def _check_finite_solution(
    checks: dict[str, dict[str, Any]],
    issues: list[TuningAssessmentIssue],
    selected,  # noqa: ANN001
) -> None:
    cost = getattr(selected, "cost", None)
    theta = list(getattr(selected, "theta", []) or [])
    finite = _finite(cost) and all(_finite(value) for value in theta)
    checks["finite_solution"] = {
        "status": ASSESSMENT_PASS if finite else ASSESSMENT_FAIL,
        "cost": float(cost) if _finite(cost) else None,
        "parameter_count": len(theta),
    }
    if not finite:
        issues.append(
            TuningAssessmentIssue(
                "non_finite_solution",
                "error",
                "Selected optimizer status has non-finite cost or parameters.",
            )
        )


def _check_accepted_solution(
    checks: dict[str, dict[str, Any]],
    issues: list[TuningAssessmentIssue],
    final,  # noqa: ANN001
    selected,  # noqa: ANN001
    source: str,
) -> None:
    accepted = bool(getattr(selected, "accepted", False))
    status = ASSESSMENT_PASS if accepted and source != "initial_fallback" else ASSESSMENT_WARN
    checks["accepted_solution"] = {
        "status": status,
        "selected_status_source": source,
        "selected_accepted": accepted,
        "final_accepted": bool(getattr(final, "accepted", False)),
    }
    if status != ASSESSMENT_PASS:
        issues.append(
            TuningAssessmentIssue(
                "no_accepted_update",
                "warning",
                "The selected output did not come from an accepted optimizer update.",
                {"selected_status_source": source},
            )
        )


def _check_bounds(
    checks: dict[str, dict[str, Any]],
    issues: list[TuningAssessmentIssue],
    parameter_summary: dict[str, Any],
) -> None:
    bound_hits = list(parameter_summary.get("bound_hits", []))
    status = ASSESSMENT_PASS if not bound_hits else ASSESSMENT_WARN
    checks["bound_hits"] = {
        "status": status,
        "count": len(bound_hits),
        "parameters": bound_hits,
    }
    if bound_hits:
        issues.append(
            TuningAssessmentIssue(
                "parameter_bound_hits",
                "warning",
                "One or more parameters ended very close to an optimization bound.",
                {"parameters": bound_hits},
            )
        )


def _check_parameter_coupling(
    checks: dict[str, dict[str, Any]],
    issues: list[TuningAssessmentIssue],
    config,  # noqa: ANN001
    parameter_summary: dict[str, Any],
    resampling_diagnostics,  # noqa: ANN001
) -> None:
    families = set(parameter_summary.get("families", []))
    pair_warnings: list[str] = []
    if "joint_friction" in families and "joint_damping" in families:
        pair_warnings.append("joint_friction_with_joint_damping")
    if "joint_armature" in families and ("link_mass" in families or "link_inertia" in families):
        pair_warnings.append("joint_armature_with_link_inertia_or_mass")
    if "link_mass" in families and ("link_com" in families or "link_inertia" in families):
        pair_warnings.append("link_mass_with_com_or_inertia")
    if "contact" in families and len(families) > 1:
        pair_warnings.append("contact_with_non_contact_parameters")
    if "joint_stiffness" in families and any(
        _diagnostic_reason(item) in {"timestamp_jitter", "irregular_dt"} for item in (resampling_diagnostics or [])
    ):
        pair_warnings.append("joint_stiffness_with_timing_warnings")
    if len(parameter_summary.get("rows", [])) > max(4, 2 * _num_joints(config)):
        pair_warnings.append("large_parameter_set")

    checks["parameter_coupling"] = {
        "status": ASSESSMENT_PASS if not pair_warnings else ASSESSMENT_WARN,
        "families": sorted(families),
        "warnings": pair_warnings,
    }
    for code in pair_warnings:
        issues.append(
            TuningAssessmentIssue(
                f"coupling_{code}",
                "warning",
                _coupling_message(code),
                {"families": sorted(families)},
            )
        )


def _check_validation_holdout(
    checks: dict[str, dict[str, Any]],
    issues: list[TuningAssessmentIssue],
    prepared,  # noqa: ANN001
    result,  # noqa: ANN001
    validation_metrics,  # noqa: ANN001
) -> None:
    source = str(getattr(result, "validation_artifact_source", "configured_chunks"))
    chunk_count = len(getattr(prepared, "validation_chunks", []) or [])
    metric_count = len(validation_metrics)
    if chunk_count > 0 and source == "configured_chunks" and metric_count > 0:
        status = ASSESSMENT_PASS
        issue = None
    elif metric_count > 0:
        status = ASSESSMENT_WARN
        issue = TuningAssessmentIssue(
            "no_configured_holdout",
            "warning",
            "Validation metrics were produced, but not from the configured independent holdout chunks.",
            {
                "artifact_source": source,
                "configured_validation_chunks": chunk_count,
                "validation_metric_count": metric_count,
            },
        )
    else:
        status = ASSESSMENT_WARN
        issue = TuningAssessmentIssue(
            "no_validation_metrics_at_all",
            "warning",
            "No validation metrics were produced; tuning credibility is limited.",
            {
                "artifact_source": source,
                "configured_validation_chunks": chunk_count,
                "validation_metric_count": metric_count,
            },
        )
    checks["validation_holdout"] = {
        "status": status,
        "configured_validation_chunks": chunk_count,
        "validation_metric_count": metric_count,
        "artifact_source": source,
    }
    if issue is not None:
        issues.append(issue)


def _check_validation_improvement(
    checks: dict[str, dict[str, Any]],
    issues: list[TuningAssessmentIssue],
    comparison: dict[str, Any],
) -> None:
    """Warn when any available held-out channel regresses from authored values.

    Args:
        checks: Assessment checks updated with the comparison outcome.
        issues: Assessment issues updated when a regression is found.
        comparison: Paired untuned and tuned validation comparison.
    """
    if comparison.get("holdout") is False:
        checks["validation_improvement"] = {
            "status": ASSESSMENT_UNAVAILABLE,
            "reason": "paired comparison uses training-data replay rather than an independent holdout",
            "reason_code": "training_data_replay",
            "regressed_channels": [],
        }
        return
    if not comparison.get("available"):
        checks["validation_improvement"] = {
            "status": ASSESSMENT_UNAVAILABLE,
            "reason": comparison.get("reason", "paired validation is unavailable"),
            "reason_code": "comparison_unavailable",
            "regressed_channels": [],
        }
        return
    overall = dict(comparison.get("overall") or {})
    regressed = sorted(
        name for name, payload in overall.items() if isinstance(payload, dict) and payload.get("status") == "regressed"
    )
    # A channel whose RMSE is finite in only one pass diverged on at least one
    # chunk. That is a failure to surface, not a clean pass.
    unpaired = sorted(
        name for name, payload in overall.items() if isinstance(payload, dict) and payload.get("unpaired_chunks")
    )
    if regressed:
        status = ASSESSMENT_WARN
        reason = ""
        reason_code = "regressed_channels"
    elif unpaired:
        status = ASSESSMENT_WARN
        reason = "one or more channels diverged on a validation chunk in exactly one pass"
        reason_code = "unpaired_channels"
    else:
        status = ASSESSMENT_PASS
        reason = ""
        reason_code = ""
    checks["validation_improvement"] = {
        "status": status,
        "reason_code": reason_code,
        "reason": reason,
        "regressed_channels": regressed,
        "unpaired_channels": unpaired,
        "overall": overall,
    }
    if regressed:
        issues.append(
            TuningAssessmentIssue(
                "validation_rmse_regression",
                "warning",
                "Accepted parameters increase one or more validation RMSE channels relative to authored values.",
                {"channels": regressed},
            )
        )
    if unpaired:
        issues.append(
            TuningAssessmentIssue(
                "validation_rmse_unpaired",
                "warning",
                "One or more validation RMSE channels are non-finite in exactly one pass, so no verdict is possible.",
                {"channels": unpaired},
            )
        )


def _check_train_validation_gap(
    checks: dict[str, dict[str, Any]],
    issues: list[TuningAssessmentIssue],
    train_norm: float | None,
    validation_norm: float | None,
    ratio: float | None,
) -> None:
    if not _finite(train_norm) or not _finite(validation_norm) or ratio is None:
        checks["train_validation_gap"] = {
            "status": ASSESSMENT_UNAVAILABLE,
            "training_normalized_cost": train_norm,
            "validation_normalized_cost": validation_norm,
            "validation_to_training_cost_ratio": ratio,
        }
        issues.append(
            TuningAssessmentIssue(
                "train_validation_gap_unavailable",
                "warning",
                "Could not compare training and validation normalized costs.",
            )
        )
        return

    if ratio <= 3.0:
        status = ASSESSMENT_PASS
        severity = ""
    elif ratio <= 10.0:
        status = ASSESSMENT_WARN
        severity = "warning"
    else:
        status = ASSESSMENT_FAIL
        severity = "error"
    checks["train_validation_gap"] = {
        "status": status,
        "training_normalized_cost": train_norm,
        "validation_normalized_cost": validation_norm,
        "validation_to_training_cost_ratio": ratio,
    }
    if severity:
        issues.append(
            TuningAssessmentIssue(
                "validation_cost_exceeds_training",
                severity,
                "Validation normalized cost is much larger than training normalized cost.",
                {"ratio": ratio},
            )
        )


def _check_excitation_coverage(
    checks: dict[str, dict[str, Any]],
    issues: list[TuningAssessmentIssue],
    prepared,  # noqa: ANN001
) -> None:
    train = {chunk.spec.excitation or "custom" for chunk in (getattr(prepared, "train_chunks", []) or [])}
    validation = {chunk.spec.excitation or "custom" for chunk in (getattr(prepared, "validation_chunks", []) or [])}
    all_tags = train | validation
    only_custom = not all_tags or all_tags == {"custom"}
    status = ASSESSMENT_PASS if len(all_tags) >= 2 and validation and not only_custom else ASSESSMENT_WARN
    checks["excitation_coverage"] = {
        "status": status,
        "train_excitations": sorted(train),
        "validation_excitations": sorted(validation),
        "unique_excitation_count": len(all_tags),
    }
    if status != ASSESSMENT_PASS:
        issues.append(
            TuningAssessmentIssue(
                "limited_excitation_coverage",
                "warning",
                "Telemetry chunks do not show broad independent excitation coverage.",
                {"train_excitations": sorted(train), "validation_excitations": sorted(validation)},
            )
        )


def _check_resampling(
    checks: dict[str, dict[str, Any]],
    issues: list[TuningAssessmentIssue],
    diagnostics,  # noqa: ANN001
) -> None:
    rows = [_diagnostic_dict(item) for item in (diagnostics or [])]
    capped = [row for row in rows if bool(row.get("capped"))]
    jittered = [row for row in rows if row.get("reason") in {"timestamp_jitter", "irregular_dt"}]
    status = ASSESSMENT_PASS if not capped and not jittered else ASSESSMENT_WARN
    checks["resampling"] = {
        "status": status,
        "chunk_count": len(rows),
        "enabled_count": sum(1 for row in rows if bool(row.get("enabled"))),
        "capped_count": len(capped),
        "timing_warning_count": len(jittered),
    }
    if capped:
        issues.append(
            TuningAssessmentIssue(
                "resampling_capped",
                "warning",
                "One or more chunks were resampled on a coarser capped grid.",
                {"count": len(capped)},
            )
        )
    if jittered:
        issues.append(
            TuningAssessmentIssue(
                "telemetry_timing_irregular",
                "warning",
                "One or more chunks had irregular telemetry timing before resampling.",
                {"count": len(jittered)},
            )
        )


def _check_telemetry_quality(
    checks: dict[str, dict[str, Any]],
    issues: list[TuningAssessmentIssue],
    telemetry_quality,  # noqa: ANN001
) -> None:
    warning_count = int(getattr(telemetry_quality, "warning_count", 0) or 0) if telemetry_quality is not None else 0
    checks["telemetry_quality"] = {
        "status": ASSESSMENT_PASS if warning_count == 0 else ASSESSMENT_WARN,
        "warning_count": warning_count,
    }
    if warning_count:
        issues.append(
            TuningAssessmentIssue(
                "telemetry_quality_warnings",
                "warning",
                "Telemetry quality reported warnings that should be reviewed.",
                {"warning_count": warning_count},
            )
        )


def _check_analytical_presolve(
    checks: dict[str, dict[str, Any]],
    issues: list[TuningAssessmentIssue],
    analytical_presolve,  # noqa: ANN001
) -> None:
    if analytical_presolve is None:
        checks["analytical_presolve"] = {"status": ASSESSMENT_UNAVAILABLE, "enabled": False}
        return
    warnings = list(getattr(analytical_presolve, "warnings", []) or [])
    rank = int(getattr(analytical_presolve, "rank", 0) or 0)
    condition = float(getattr(analytical_presolve, "condition", float("inf")))
    full_condition = float(getattr(analytical_presolve, "full_condition", float("inf")))
    gravity_condition = float(getattr(analytical_presolve, "gravity_condition", float("inf")))
    status = (
        ASSESSMENT_PASS if not warnings and rank > 0 and _finite(condition) and condition < 1e8 else ASSESSMENT_WARN
    )
    checks["analytical_presolve"] = {
        "status": status,
        "enabled": True,
        "target_source": getattr(analytical_presolve, "target_source", "unavailable"),
        "active_column_count": int(getattr(analytical_presolve, "active_column_count", 0) or 0),
        "seeded_count": int(getattr(analytical_presolve, "seeded_count", 0) or 0),
        "family_seeded_counts": dict(getattr(analytical_presolve, "family_seeded_counts", {}) or {}),
        "rank": rank,
        "condition": condition if _finite(condition) else None,
        "full_rank": int(getattr(analytical_presolve, "full_rank", 0) or 0),
        "full_condition": full_condition if _finite(full_condition) else None,
        "gravity_rank": int(getattr(analytical_presolve, "gravity_rank", 0) or 0),
        "gravity_condition": gravity_condition if _finite(gravity_condition) else None,
        "warnings": warnings,
    }
    if status != ASSESSMENT_PASS:
        issues.append(
            TuningAssessmentIssue(
                "analytical_presolve_conditioning",
                "warning",
                "Analytical presolve reported weak conditioning, low rank, or warnings.",
                {"rank": rank, "condition": condition if _finite(condition) else None, "warnings": warnings},
            )
        )


def _parameter_summary(config, selected) -> dict[str, Any]:  # noqa: ANN001
    if config is None or selected is None:
        return {"count": 0, "families": [], "rows": [], "bound_hits": [], "large_delta_count": 0}
    theta = np.asarray(list(getattr(selected, "theta", []) or []), dtype=np.float64)
    theta_initial = _tensor_to_numpy(getattr(config, "theta_initial", None))
    theta_min = _tensor_to_numpy(getattr(config, "theta_min", None))
    theta_max = _tensor_to_numpy(getattr(config, "theta_max", None))
    rows = []
    bound_hits = []
    families = []
    large_delta_count = 0
    for idx, entry in enumerate(getattr(config, "param_entries", []) or []):
        if idx >= theta.shape[0]:
            continue
        family = _parameter_family(entry.param_type)
        families.append(family)
        initial = float(theta_initial[idx]) if idx < theta_initial.shape[0] else None
        min_v = float(theta_min[idx]) if idx < theta_min.shape[0] else None
        max_v = float(theta_max[idx]) if idx < theta_max.shape[0] else None
        final = float(theta[idx])
        span = max(abs((max_v or 0.0) - (min_v or 0.0)), 1e-12) if min_v is not None and max_v is not None else None
        delta = final - initial if initial is not None else None
        delta_fraction = abs(delta) / span if delta is not None and span is not None else None
        if delta_fraction is not None and delta_fraction > 0.5:
            large_delta_count += 1
        hit = ""
        if min_v is not None and span is not None and abs(final - min_v) <= max(1e-6, 1e-3 * span):
            hit = "min"
        elif max_v is not None and span is not None and abs(final - max_v) <= max(1e-6, 1e-3 * span):
            hit = "max"
        name = entry.display_name(config.trajectory.num_joints) if hasattr(entry, "display_name") else str(entry)
        row = {
            "index": idx,
            "name": name,
            "param_type": entry.param_type.value if hasattr(entry.param_type, "value") else str(entry.param_type),
            "family": family,
            "initial": initial,
            "final": final,
            "min": min_v,
            "max": max_v,
            "delta": delta,
            "delta_fraction_of_range": delta_fraction,
            "bound_hit": hit,
        }
        rows.append(row)
        if hit:
            bound_hits.append({"index": idx, "name": name, "bound": hit, "value": final})
    return {
        "count": len(rows),
        "families": sorted(set(families)),
        "rows": rows,
        "bound_hits": bound_hits,
        "large_delta_count": large_delta_count,
    }


def _training_normalized_cost(config: Any, selected: Any) -> float | None:
    """Return training cost in the same per-residual units as validation metrics.

    The rollout optimizer stores a single-trajectory status cost as a raw
    half-sum-of-squares, so that path is divided by its residual width here.
    Segmented training is different: the optimizer divides every segment by its
    residual width before taking the configured weighted mean. Its status cost is
    therefore already normalized and must not be divided a second time.

    Args:
        config: Optimizer configuration that defines the training trajectories.
        selected: Selected optimizer status containing the reported cost.

    Returns:
        Per-residual training cost, or ``None`` when it cannot be computed.
    """
    cost = getattr(selected, "cost", None)
    if not _finite(cost) or config is None:
        return None
    training_segments = list(getattr(config, "training_segments", []) or [])
    training_chunks = list(getattr(config, "training_chunks", []) or [])
    if training_segments or training_chunks:
        return float(cost)
    residual_dim = _residual_dim(getattr(config, "trajectory", None), config)
    if residual_dim < 1:
        return None
    return float(cost) / float(residual_dim)


def _residual_dim(trajectory, config) -> int:  # noqa: ANN001
    if trajectory is None:
        return 0
    residual_cfg = getattr(config, "residual_weight_config", None)
    steps = min(
        int(getattr(trajectory, "commands", np.zeros((0, 0))).shape[0]),
        max(1, int(getattr(config, "max_rollout_steps", 1))),
    )
    joints = int(getattr(trajectory, "positions", np.zeros((0, 0))).shape[1])
    dim = 0
    if residual_cfg is None or float(getattr(residual_cfg, "position_weight", 1.0)) > 0.0:
        dim += steps * joints
    if residual_cfg is None or float(getattr(residual_cfg, "velocity_weight", 1.0)) > 0.0:
        dim += steps * joints
    if residual_cfg is not None and float(getattr(residual_cfg, "torque_weight", 0.0)) > 0.0:
        dim += steps * _optional_width(getattr(trajectory, "torques", None))
    if residual_cfg is not None and float(getattr(residual_cfg, "end_effector_pose_weight", 0.0)) > 0.0:
        dim += steps * _optional_width(getattr(trajectory, "end_effector_poses", None))
    if residual_cfg is not None and float(getattr(residual_cfg, "contact_force_weight", 0.0)) > 0.0:
        dim += steps * _optional_width(getattr(trajectory, "contact_forces", None))
    return int(dim)


def _recommended_next_parameters(
    config: Any,
    issues: list[TuningAssessmentIssue],
    validation_metrics: Any,
) -> list[str]:
    issue_codes = {issue.code for issue in issues}
    families = set()
    if config is not None:
        families = {_parameter_family(entry.param_type) for entry in (getattr(config, "param_entries", []) or [])}
    recommendations = []
    if any(code.startswith("coupling_") for code in issue_codes) or "parameter_bound_hits" in issue_codes:
        recommendations.append(_NextParameterRecommendation.REDUCE_OR_TIGHTEN.value)
    if {"no_configured_holdout", "no_validation_metrics_at_all"} & issue_codes:
        recommendations.append(_NextParameterRecommendation.ADD_VALIDATION.value)
    if "validation_cost_exceeds_training" in issue_codes:
        recommendations.append(_NextParameterRecommendation.RERUN_SMALLER_SET.value)
    if families <= {"joint_friction", "joint_damping", "joint_stiffness"} and validation_metrics:
        recommendations.append(_NextParameterRecommendation.CONSIDER_ARMATURE.value)
    if "joint_armature" in families and not ({"link_mass", "link_inertia"} & families):
        recommendations.append(_NextParameterRecommendation.CONSIDER_INERTIA.value)
    if not recommendations:
        recommendations.append(_NextParameterRecommendation.VALIDATE_MORE_HOLDOUTS.value)
    return recommendations


def _recommended_actions(
    issues: list[TuningAssessmentIssue],
    validation_metrics: Any,
    parameter_summary: dict[str, Any],
) -> list[str]:
    actions = []
    severities = {issue.severity for issue in issues}
    if "error" in severities:
        actions.append(_RecommendedAction.RESOLVE_ERRORS.value)
    if validation_metrics:
        actions.append(_RecommendedAction.INSPECT_OVERLAYS.value)
    if parameter_summary.get("bound_hits"):
        actions.append(_RecommendedAction.REVIEW_BOUND_HITS.value)
    if parameter_summary.get("large_delta_count", 0) > 0:
        actions.append(_RecommendedAction.REVIEW_LARGE_DELTAS.value)
    if not actions:
        actions.append(_RecommendedAction.RUN_ANOTHER_HOLDOUT.value)
    return actions


def _overall_status(issues: Iterable[TuningAssessmentIssue]) -> str:
    severities = {issue.severity for issue in issues}
    if "error" in severities:
        return ASSESSMENT_FAIL
    if "warning" in severities:
        return ASSESSMENT_WARN
    return ASSESSMENT_PASS


def _parameter_family(param_type) -> str:  # noqa: ANN001
    value = param_type.value if hasattr(param_type, "value") else str(param_type)
    if value in {SysIdParameterType.JOINT_FRICTION.value}:
        return "joint_friction"
    if value in {SysIdParameterType.JOINT_DAMPING.value}:
        return "joint_damping"
    if value in {SysIdParameterType.JOINT_INTEGRAL_GAIN.value}:
        return "joint_integral_gain"
    if value in {SysIdParameterType.JOINT_STIFFNESS.value}:
        return "joint_stiffness"
    if value in {SysIdParameterType.JOINT_ARMATURE.value}:
        return "joint_armature"
    if value == SysIdParameterType.ACTUATOR_COMMAND_DELAY_SECONDS.value:
        return "command_delay"
    if value == SysIdParameterType.LINK_MASS.value:
        return "link_mass"
    if value in {
        SysIdParameterType.LINK_COM_OFFSET_X.value,
        SysIdParameterType.LINK_COM_OFFSET_Y.value,
        SysIdParameterType.LINK_COM_OFFSET_Z.value,
    }:
        return "link_com"
    if value == SysIdParameterType.LINK_INERTIA_LOG_CHOLESKY.value:
        return "link_inertia"
    if value in {SysIdParameterType.JOINT_LIMIT_LOWER_SCALE.value, SysIdParameterType.JOINT_LIMIT_UPPER_SCALE.value}:
        return "joint_limits"
    return value


def _coupling_message(code: str) -> str:
    return {
        "joint_friction_with_joint_damping": "Joint friction and damping are coupled; validate before broadening.",
        "joint_armature_with_link_inertia_or_mass": "Armature and inertial terms can explain the same acceleration error.",
        "link_mass_with_com_or_inertia": "Mass, COM, and inertia are strongly coupled without rich torque/gravity data.",
        "contact_with_non_contact_parameters": "Contact parameters should usually be tuned separately from free-space dynamics.",
        "joint_stiffness_with_timing_warnings": "Stiffness changes may be compensating for telemetry timing issues.",
        "large_parameter_set": "The selected parameter set is large relative to the apparent manipulator size.",
    }.get(code, "Selected parameter families may be coupled.")


def _diagnostic_dict(item) -> dict[str, Any]:  # noqa: ANN001
    if hasattr(item, "to_dict"):
        return item.to_dict()
    if isinstance(item, dict):
        return dict(item)
    return {}


def _diagnostic_reason(item) -> str:  # noqa: ANN001
    return str(_diagnostic_dict(item).get("reason", ""))


def _is_holdout_metric(metric) -> bool:  # noqa: ANN001
    """Return True unless a metric is explicitly flagged as a non-holdout fallback.

    Validation metrics derived from the training-chunk fallback carry
    ``extra["holdout"] = False`` because they are evaluated on the training
    trajectory; configured holdout chunks leave the flag unset (treated as True).

    Args:
        metric: Value supplied for ``metric``.

    Returns:
        Result produced by the operation.
    """  # noqa: DOC106, DOC107
    extra = getattr(metric, "extra", None)
    if extra is None and isinstance(metric, dict):
        extra = metric.get("extra")
    if not isinstance(extra, dict):
        return True
    return bool(extra.get("holdout", True))


def _mean_metric(metrics, name: str) -> float | None:  # noqa: ANN001
    values = []
    for metric in metrics or []:
        value = getattr(metric, name, None)
        if value is None and isinstance(metric, dict):
            value = metric.get(name)
        if _finite(value):
            values.append(float(value))
    if not values:
        return None
    return float(sum(values) / len(values))


def _tensor_to_numpy(value) -> np.ndarray:  # noqa: ANN001
    if value is None:
        return np.asarray([], dtype=np.float64)
    if hasattr(value, "detach"):
        return value.detach().cpu().numpy().astype(np.float64).reshape(-1)
    return np.asarray(value, dtype=np.float64).reshape(-1)


def _optional_width(value) -> int:  # noqa: ANN001
    if value is None:
        return 0
    arr = np.asarray(value)
    if arr.ndim < 2:
        return 1
    return int(arr.shape[1])


def _num_joints(config) -> int:  # noqa: ANN001
    trajectory = getattr(config, "trajectory", None) if config is not None else None
    positions = getattr(trajectory, "positions", None)
    if positions is None:
        return 1
    arr = np.asarray(positions)
    return int(arr.shape[1]) if arr.ndim >= 2 else 1


def _finite(value) -> bool:  # noqa: ANN001
    try:
        return math.isfinite(float(value))
    except (TypeError, ValueError):
        return False
