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

"""Lightweight warn-first schema validation for SysID JSON contracts."""

from __future__ import annotations

import math
from collections.abc import Iterable, Mapping
from dataclasses import asdict, dataclass
from typing import Any

from .errors import SysIdSchemaError
from .ingest.config_types import TrajectorySourceType
from .optimizer_config import OptimizerBackend
from .parameter_types import SysIdParameterType
from .run_spec import (
    NEWTON_SOLVER_FEATHERSTONE_DIFF,
    NEWTON_SOLVER_MUJOCO,
    SIMULATION_ENGINE_ISAAC_SIM,
    SIMULATION_ENGINE_NEWTON,
)
from .trajectory_segments import TELEMETRY_CHUNK_ROLES


@dataclass(frozen=True)
class SysIdSchemaIssue:
    """One schema validation issue surfaced as a warning in v1."""

    path: str
    message: str
    severity: str = "warning"

    def to_dict(self) -> dict[str, Any]:
        """Return a JSON-ready representation of this schema issue.

        Returns:
            Dataclass fields as a JSON-compatible mapping.
        """
        return asdict(self)


_RUN_SPEC_KEYS = {
    "schema_version",
    "telemetry_quality",
    "stage",
    "telemetry",
    "simulation",
    "parameters",
    "solver",
    "residuals",
    "outputs",
    "sampling",
}
_SIMULATION_KEYS = {
    "engine",
    "robot_prim_path",
    "source_env_path",
    "env_paths_root",
    "parallel_clones",
    "co_locate_clones",
    "fabric_clones",
    "offline_stepping",
    "physics_backend",
    "actuator_runtime",
    "explicit_actuator_policy",
    "newton",
}
_NEWTON_KEYS = {
    "device",
    "actuator_source",
    "controller",
    "effort_clamp",
    "actuator_dt_mode",
    "solver",
    "featherstone_substeps",
    "mujoco_substeps",
    "feedforward",
    "cuda_graph_capture",
}
_PARAMETERS_KEYS = {"space", "selected"}
_PARAMETER_KEYS = {
    "param_type",
    "dof_index",
    "link_index",
    "component_index",
    "category",
    "initial",
    "min",
    "max",
}
_TELEMETRY_KEYS = {
    "source_type",
    "source_path",
    "mapping_path",
    "chunk_manifest_path",
    "time_window",
    "chunks",
    "lerobot_episode_index",
    "auto_split",
    "auto_split_train_fraction",
    "auto_split_min_chunk_seconds",
    "command_alignment_seconds",
}
_CHUNK_KEYS = {"name", "role", "excitation", "start", "end", "weight"}
_SOLVER_KEYS = {
    "optimizer",
    "damping_initial",
    "epsilon",
    "max_iterations",
    "max_rollout_steps",
    "use_analytical_jacobian",
    "use_analytical_presolve",
    "cma_population_size",
    "cma_sigma",
    "cma_batch_size",
    "cma_seed",
    "bo_initial_samples",
    "bo_candidate_count",
    "bo_batch_size",
    "bo_seed",
    "gd_learning_rate",
    "gd_num_restarts",
    "gd_seed",
}
_RESIDUAL_KEYS = {
    "position_weight",
    "velocity_weight",
    "torque_weight",
    "end_effector_pose_weight",
    "contact_force_weight",
    "sample_weights",
    "end_effector_link_index",
}
_OUTPUTS_KEYS = {
    "apply_parameters_to_usd",
    "write_usd_provenance",
    "export_provenance_sidecar",
    "provenance_path",
    "remove_clones_after_run",
    "render_validation_animation",
    "validation_animation_path",
    "validation_animation_fps",
    "validation_animation_max_frames",
    "validation_relative_change_threshold",
    "validation_absolute_change_threshold",
    "compute_parameter_confidence",
    "parameter_confidence_rollout_budget",
}
_SAMPLING_KEYS = {
    "mode",
    "target_dt_mode",
    "command_interpolation",
    "measured_interpolation",
    "auto_dt_ratio_threshold",
    "auto_jitter_threshold",
    "max_resampled_steps",
}
_TOPIC_MAPPING_KEYS = {
    "time_topic",
    "time_field",
    "position_topic",
    "position_fields",
    "velocity_topic",
    "velocity_fields",
    "command_topic",
    "command_fields",
    "torque_topic",
    "torque_fields",
    "torque_semantics",
    "end_effector_pose_topic",
    "end_effector_pose_fields",
    "contact_force_topic",
    "contact_force_fields",
    "joint_names",
}

_CHECK_REPORT_REQUIRED_KEYS = {
    "schema_version",
    "ok",
    "telemetry_quality",
    "presolve",
    "parameter_verdicts",
    "issues",
}
_RUN_REPORT_REQUIRED_KEYS = {
    "report_schema_version",
    "run_spec",
    "schema_issues",
    "telemetry_quality",
    "backend",
    "simulation_engine",
    "newton_config",
    "robot_prim_path",
    "train_chunks",
    "untuned_validation_metrics",
    "tuned_validation_metrics",
    "validation_metrics",
    "untuned_validation_summary",
    "tuned_validation_summary",
    "validation_summary",
    "validation_comparison",
    "validation_artifacts",
    "resampling_diagnostics",
    "tuning_assessment",
    "parameters",
    "command_alignment_seconds",
    "actuator_command_delay_seconds",
    "parameters_written",
    "provenance_path",
    "final_status",
    "last_accepted_status",
    "selected_status",
    "parameter_confidence",
    "extra",
}
# Keys added by run-report version 4. A version-3 artifact predates them, so
# requiring them would reject a report the validator declares readable.
_RUN_REPORT_V4_ADDED_KEYS = {
    "untuned_validation_metrics",
    "tuned_validation_metrics",
    "untuned_validation_summary",
    "tuned_validation_summary",
    "validation_comparison",
}
_RUN_REPORT_REQUIRED_KEYS_BY_VERSION = {
    3: _RUN_REPORT_REQUIRED_KEYS - _RUN_REPORT_V4_ADDED_KEYS,
}


def validate_sysid_check_report_payload(payload: dict[str, Any]) -> list[SysIdSchemaIssue]:
    """Validate the stable top-level SysID check-report contract.

    Args:
        payload: Serialized check-report payload.

    Returns:
        Contract violations with JSON paths into the payload.
    """
    issues = _validate_report_envelope(
        payload,
        required_keys=_CHECK_REPORT_REQUIRED_KEYS,
        version_key="schema_version",
        version=1,
        report_name="check report",
    )
    if not isinstance(payload, dict):
        return issues
    if "ok" in payload and not isinstance(payload["ok"], bool):
        issues.append(SysIdSchemaIssue("$.ok", "ok must be a boolean.", "error"))
    verdicts = payload.get("parameter_verdicts")
    if verdicts is not None and not isinstance(verdicts, list):
        issues.append(SysIdSchemaIssue("$.parameter_verdicts", "parameter_verdicts must be an array.", "error"))
    return issues


def validate_sysid_run_report_payload(payload: dict[str, Any]) -> list[SysIdSchemaIssue]:
    """Validate the stable top-level SysID run-report version 4 contract.

    Args:
        payload: Serialized run-report payload.

    Returns:
        Contract violations with JSON paths into the payload.
    """
    return _validate_report_envelope(
        payload,
        required_keys=_RUN_REPORT_REQUIRED_KEYS,
        version_key="report_schema_version",
        version=4,
        report_name="run report",
        readable_versions=_RUN_REPORT_REQUIRED_KEYS_BY_VERSION,
    )


def _validate_report_envelope(
    payload: dict[str, Any],
    *,
    required_keys: set[str],
    version_key: str,
    version: int,
    report_name: str,
    readable_versions: Mapping[int, set[str]] | None = None,
) -> list[SysIdSchemaIssue]:
    issues: list[SysIdSchemaIssue] = []
    if not isinstance(payload, dict):
        return [SysIdSchemaIssue("$", f"SysID {report_name} must be a JSON object.", "error")]
    actual_version = payload.get(version_key)
    readable = readable_versions or {}
    # A superseded payload is held to the key set of its own version; requiring
    # the current keys would reject the very artifacts declared readable. A
    # malformed version value is not a dict key, so fall back to the current set.
    superseded = isinstance(actual_version, int) and actual_version != version and actual_version in readable
    expected_keys = readable[actual_version] if superseded else required_keys
    for key in sorted(expected_keys - payload.keys()):
        issues.append(SysIdSchemaIssue(f"$.{key}", f"{key} is required.", "error"))
    if actual_version is not None and actual_version != version:
        if superseded:
            # An artifact written by an older release is still readable; the
            # newer keys are simply absent. Reject only unknown versions.
            issues.append(
                SysIdSchemaIssue(
                    f"$.{version_key}",
                    f"{report_name} uses superseded {version_key} {actual_version!r}; "
                    f"fields added in version {version} are absent.",
                    "warning",
                )
            )
        else:
            issues.append(
                SysIdSchemaIssue(
                    f"$.{version_key}",
                    f"{version_key} must be {version}, got {actual_version!r}.",
                    "error",
                )
            )
    return issues


def validate_sysid_run_spec_payload(payload: dict[str, Any]) -> list[SysIdSchemaIssue]:
    """Validate a RunSpec-like payload.

    Args:
        payload: Serialized input payload.

    Returns:
        Ordered warnings and errors with JSON paths into the payload.
    """
    issues: list[SysIdSchemaIssue] = []
    if not isinstance(payload, dict):
        return [SysIdSchemaIssue("$", "SysID run spec must be a JSON object.", "error")]

    _warn_unknown_keys(payload, _RUN_SPEC_KEYS, "$", issues)
    if "schema_version" not in payload:
        issues.append(SysIdSchemaIssue("$.schema_version", "Missing schema_version; default will be assumed."))

    telemetry = _mapping(payload.get("telemetry"), "$.telemetry", issues)
    if telemetry is not None:
        _warn_unknown_keys(telemetry, _TELEMETRY_KEYS, "$.telemetry", issues)
        source_type = telemetry.get("source_type")
        if source_type is not None:
            _validate_enum(
                source_type,
                {item.value for item in TrajectorySourceType},
                "$.telemetry.source_type",
                issues,
            )
        fraction = telemetry.get("auto_split_train_fraction")
        if fraction is not None and (
            not isinstance(fraction, (int, float)) or not 0.0 < float(fraction) < 1.0 or math.isnan(float(fraction))
        ):
            issues.append(
                SysIdSchemaIssue(
                    "$.telemetry.auto_split_train_fraction",
                    "auto_split_train_fraction should be a number strictly between 0 and 1.",
                )
            )
        _validate_nonnegative_number(
            telemetry,
            "auto_split_min_chunk_seconds",
            "$.telemetry.auto_split_min_chunk_seconds",
            issues,
        )
        _validate_finite_number(
            telemetry,
            "command_alignment_seconds",
            "$.telemetry.command_alignment_seconds",
            issues,
        )
        chunks = telemetry.get("chunks")
        if chunks is not None:
            if not isinstance(chunks, list):
                issues.append(SysIdSchemaIssue("$.telemetry.chunks", "chunks must be an array.", "error"))
            else:
                for idx, chunk in enumerate(chunks):
                    chunk_path = f"$.telemetry.chunks[{idx}]"
                    if not isinstance(chunk, dict):
                        issues.append(SysIdSchemaIssue(chunk_path, "chunk must be an object.", "error"))
                        continue
                    _warn_unknown_keys(chunk, _CHUNK_KEYS, chunk_path, issues)
                    _validate_enum(
                        chunk.get("role", "train"),
                        set(TELEMETRY_CHUNK_ROLES),
                        f"{chunk_path}.role",
                        issues,
                    )
                    for key in ("start", "end"):
                        if key not in chunk:
                            issues.append(
                                SysIdSchemaIssue(
                                    f"{chunk_path}.{key}",
                                    f"{key} is required.",
                                    "error",
                                )
                            )
                        else:
                            _validate_finite_number(chunk, key, f"{chunk_path}.{key}", issues)
                    _validate_positive_number(chunk, "weight", f"{chunk_path}.weight", issues)
                    start = chunk.get("start")
                    end = chunk.get("end")
                    if (
                        isinstance(start, (int, float))
                        and not isinstance(start, bool)
                        and math.isfinite(float(start))
                        and isinstance(end, (int, float))
                        and not isinstance(end, bool)
                        and math.isfinite(float(end))
                        and float(end) <= float(start)
                    ):
                        issues.append(
                            SysIdSchemaIssue(
                                f"{chunk_path}.end",
                                "end must be strictly greater than start.",
                                "error",
                            )
                        )

    solver = _mapping(payload.get("solver"), "$.solver", issues)
    if solver is not None:
        _warn_unknown_keys(solver, _SOLVER_KEYS, "$.solver", issues)
        optimizer = solver.get("optimizer")
        if optimizer is not None:
            _validate_enum(
                optimizer,
                {item.value for item in OptimizerBackend},
                "$.solver.optimizer",
                issues,
            )
        _validate_positive_number(solver, "epsilon", "$.solver.epsilon", issues)
        _validate_positive_number(solver, "damping_initial", "$.solver.damping_initial", issues)
        _validate_positive_int(solver, "max_iterations", "$.solver.max_iterations", issues)
        _validate_int_min(solver, "max_rollout_steps", "$.solver.max_rollout_steps", issues, minimum=2)
        _validate_nonnegative_int(solver, "cma_seed", "$.solver.cma_seed", issues)
        _validate_nonnegative_int(solver, "bo_seed", "$.solver.bo_seed", issues)
        _validate_nonnegative_int(solver, "gd_seed", "$.solver.gd_seed", issues)

    residuals = _mapping(payload.get("residuals"), "$.residuals", issues)
    if residuals is not None:
        _warn_unknown_keys(residuals, _RESIDUAL_KEYS, "$.residuals", issues)
        for key in (
            "position_weight",
            "velocity_weight",
            "torque_weight",
            "end_effector_pose_weight",
            "contact_force_weight",
        ):
            _validate_nonnegative_number(residuals, key, f"$.residuals.{key}", issues)
        sample_weights = residuals.get("sample_weights")
        if sample_weights is not None:
            if not isinstance(sample_weights, list):
                issues.append(SysIdSchemaIssue("$.residuals.sample_weights", "Expected an array.", "error"))
            else:
                for index, value in enumerate(sample_weights):
                    path = f"$.residuals.sample_weights[{index}]"
                    if (
                        isinstance(value, bool)
                        or not isinstance(value, (int, float))
                        or not math.isfinite(float(value))
                        or float(value) < 0.0
                    ):
                        issues.append(SysIdSchemaIssue(path, "Expected a finite, non-negative number.", "error"))
        _validate_integer(
            residuals,
            "end_effector_link_index",
            "$.residuals.end_effector_link_index",
            issues,
        )

    outputs = _mapping(payload.get("outputs"), "$.outputs", issues)
    if outputs is not None:
        _warn_unknown_keys(outputs, _OUTPUTS_KEYS, "$.outputs", issues)
        _validate_positive_int(
            outputs,
            "validation_animation_fps",
            "$.outputs.validation_animation_fps",
            issues,
        )
        _validate_positive_int(
            outputs,
            "validation_animation_max_frames",
            "$.outputs.validation_animation_max_frames",
            issues,
        )
        _validate_nonnegative_number(
            outputs,
            "validation_relative_change_threshold",
            "$.outputs.validation_relative_change_threshold",
            issues,
        )
        _validate_nonnegative_number(
            outputs,
            "validation_absolute_change_threshold",
            "$.outputs.validation_absolute_change_threshold",
            issues,
        )
        _validate_boolean(
            outputs,
            "compute_parameter_confidence",
            "$.outputs.compute_parameter_confidence",
            issues,
        )
        _validate_positive_int(
            outputs,
            "parameter_confidence_rollout_budget",
            "$.outputs.parameter_confidence_rollout_budget",
            issues,
        )
    sampling = _mapping(payload.get("sampling"), "$.sampling", issues)
    if sampling is not None:
        _warn_unknown_keys(sampling, _SAMPLING_KEYS, "$.sampling", issues)
        _validate_enum(
            sampling.get("mode", "auto"),
            {"off", "auto", "always"},
            "$.sampling.mode",
            issues,
        )
        _validate_enum(
            sampling.get("target_dt_mode", "physics_dt_capped"),
            {"physics_dt_capped"},
            "$.sampling.target_dt_mode",
            issues,
        )
        _validate_enum(
            sampling.get("command_interpolation", "zero_order_hold"),
            {"zero_order_hold", "linear"},
            "$.sampling.command_interpolation",
            issues,
        )
        _validate_enum(
            sampling.get("measured_interpolation", "linear"),
            {"linear"},
            "$.sampling.measured_interpolation",
            issues,
        )
        _validate_nonnegative_number(
            sampling,
            "auto_dt_ratio_threshold",
            "$.sampling.auto_dt_ratio_threshold",
            issues,
        )
        _validate_nonnegative_number(
            sampling,
            "auto_jitter_threshold",
            "$.sampling.auto_jitter_threshold",
            issues,
        )
        _validate_nonnegative_int(sampling, "max_resampled_steps", "$.sampling.max_resampled_steps", issues)

    simulation = _mapping(payload.get("simulation"), "$.simulation", issues)
    if simulation is not None:
        _warn_unknown_keys(simulation, _SIMULATION_KEYS, "$.simulation", issues)
        _validate_enum(
            simulation.get("engine", SIMULATION_ENGINE_ISAAC_SIM),
            {SIMULATION_ENGINE_ISAAC_SIM, SIMULATION_ENGINE_NEWTON},
            "$.simulation.engine",
            issues,
        )
        _validate_enum(
            simulation.get("physics_backend", "auto"),
            {"auto", "physx", "newton"},
            "$.simulation.physics_backend",
            issues,
        )
        _validate_enum(
            simulation.get("actuator_runtime", "auto"),
            {"auto", "implicit_drive", "newton_explicit", "mixed"},
            "$.simulation.actuator_runtime",
            issues,
        )
        _validate_enum(
            simulation.get("explicit_actuator_policy", "authored_only"),
            {"authored_only", "promote_selected"},
            "$.simulation.explicit_actuator_policy",
            issues,
        )
        newton = _mapping(simulation.get("newton"), "$.simulation.newton", issues)
        if newton is not None:
            _warn_unknown_keys(newton, _NEWTON_KEYS, "$.simulation.newton", issues)
            _validate_enum(
                newton.get("actuator_source", "drive_defaults"),
                {"drive_defaults", "usd"},
                "$.simulation.newton.actuator_source",
                issues,
            )
            _validate_enum(
                newton.get("controller", "pd"),
                {"pd", "pid"},
                "$.simulation.newton.controller",
                issues,
            )
            _validate_enum(
                newton.get("effort_clamp", "max_effort"),
                {"max_effort", "dc_motor", "none"},
                "$.simulation.newton.effort_clamp",
                issues,
            )
            _validate_enum(
                newton.get("actuator_dt_mode", "physics_dt"),
                {"physics_dt"},
                "$.simulation.newton.actuator_dt_mode",
                issues,
            )
            _validate_enum(
                newton.get("solver", NEWTON_SOLVER_MUJOCO),
                {NEWTON_SOLVER_MUJOCO, NEWTON_SOLVER_FEATHERSTONE_DIFF},
                "$.simulation.newton.solver",
                issues,
            )
            _validate_enum(
                newton.get("feedforward", "none"),
                {"none", "gravity", "inverse_dynamics"},
                "$.simulation.newton.feedforward",
                issues,
            )
            _validate_positive_int(
                newton,
                "featherstone_substeps",
                "$.simulation.newton.featherstone_substeps",
                issues,
            )
            _validate_positive_int(
                newton,
                "mujoco_substeps",
                "$.simulation.newton.mujoco_substeps",
                issues,
            )
    parameters = _mapping(payload.get("parameters"), "$.parameters", issues)
    if parameters is not None:
        _warn_unknown_keys(parameters, _PARAMETERS_KEYS, "$.parameters", issues)
        selected = parameters.get("selected")
        if selected is not None:
            if not isinstance(selected, list):
                issues.append(SysIdSchemaIssue("$.parameters.selected", "selected must be an array.", "error"))
            else:
                for idx, item in enumerate(selected):
                    item_path = f"$.parameters.selected[{idx}]"
                    if not isinstance(item, dict):
                        issues.append(
                            SysIdSchemaIssue(
                                item_path,
                                "selected parameter must be an object.",
                                "error",
                            )
                        )
                        continue
                    _warn_unknown_keys(item, _PARAMETER_KEYS, item_path, issues)
                    if "param_type" not in item:
                        issues.append(
                            SysIdSchemaIssue(
                                f"{item_path}.param_type",
                                "param_type is required.",
                                "error",
                            )
                        )
                    else:
                        _validate_enum(
                            item.get("param_type"),
                            {ptype.value for ptype in SysIdParameterType},
                            f"{item_path}.param_type",
                            issues,
                        )
                    _validate_parameter_bounds(item, item_path, issues)

    return issues


def _validate_parameter_bounds(item: dict[str, Any], path: str, issues: list[SysIdSchemaIssue]) -> None:
    """Validate that a selected parameter's min/initial/max are ordered finite numbers.

    Mirrors the constraint the run controller enforces at load time (min < max and
    min <= initial <= max), so an invalid spec is reported here instead of failing
    later with a raw ValueError.

    Args:
        item: Parameter row containing bounds to validate.
        path: JSON path used to locate reported bound errors.
        issues: Validation issues accumulated by the operation.
    """
    bounds: dict[str, float] = {}
    for key in ("min", "max", "initial"):
        if key not in item:
            continue
        value = item.get(key)
        if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(float(value)):
            issues.append(SysIdSchemaIssue(f"{path}.{key}", f"{key} must be a finite number.", "error"))
            continue
        bounds[key] = float(value)

    lo = bounds.get("min")
    hi = bounds.get("max")
    initial = bounds.get("initial")
    if lo is not None and hi is not None and lo >= hi:
        issues.append(
            SysIdSchemaIssue(
                f"{path}.min",
                f"min ({lo}) must be strictly less than max ({hi}).",
                "error",
            )
        )
    if initial is not None and lo is not None and initial < lo:
        issues.append(
            SysIdSchemaIssue(
                f"{path}.initial",
                f"initial ({initial}) must be >= min ({lo}).",
                "error",
            )
        )
    if initial is not None and hi is not None and initial > hi:
        issues.append(
            SysIdSchemaIssue(
                f"{path}.initial",
                f"initial ({initial}) must be <= max ({hi}).",
                "error",
            )
        )


def validate_topic_mapping_payload(payload: dict[str, Any]) -> list[SysIdSchemaIssue]:
    """Validate a ROS 2 / MCAP topic mapping payload.

    Args:
        payload: Serialized input payload.

    Returns:
        Ordered topic-map warnings and errors.
    """
    issues: list[SysIdSchemaIssue] = []
    if not isinstance(payload, dict):
        return [SysIdSchemaIssue("$", "Topic mapping must be a JSON/YAML object.", "error")]
    _warn_unknown_keys(payload, _TOPIC_MAPPING_KEYS, "$", issues)
    if not payload.get("position_topic"):
        issues.append(
            SysIdSchemaIssue(
                "$.position_topic",
                "position_topic is required for topic ingestion.",
                "error",
            )
        )
    if "torque_semantics" in payload:
        _validate_enum(
            payload.get("torque_semantics"),
            {"link_side", "external"},
            "$.torque_semantics",
            issues,
        )
    elif payload.get("torque_topic"):
        issues.append(
            SysIdSchemaIssue(
                "$.torque_semantics",
                "torque_semantics is required when torque_topic is configured.",
                "error",
            )
        )
    for key in (
        "position_fields",
        "velocity_fields",
        "command_fields",
        "torque_fields",
        "end_effector_pose_fields",
        "contact_force_fields",
        "joint_names",
    ):
        value = payload.get(key)
        if value is not None and not isinstance(value, list):
            issues.append(SysIdSchemaIssue(f"$.{key}", f"{key} must be an array.", "error"))
    return issues


def schema_issues_to_dicts(issues: list[SysIdSchemaIssue]) -> list[dict[str, Any]]:
    """Return JSON-ready dictionaries for schema validation issues.

    Args:
        issues: Schema issues to serialize.

    Returns:
        JSON-compatible issue dictionaries in input order.
    """
    return [issue.to_dict() for issue in issues]


def schema_error_issues(issues: Iterable[SysIdSchemaIssue]) -> list[SysIdSchemaIssue]:
    """Return only the error-severity subset of schema issues.

    Args:
        issues: Schema issues to filter.

    Returns:
        Issues whose severity is ``error``, in input order.
    """
    return [issue for issue in issues if issue.severity == "error"]


def raise_for_schema_errors(issues: Iterable[SysIdSchemaIssue], *, context: str) -> None:
    """Reject a payload that violates its own packaged JSON contract.

    Warnings stay advisory. Error-severity issues mean the payload does not
    satisfy the contract that downstream consumers validate against, so
    continuing would produce an authoritative-looking report built from input
    the extension itself considers invalid.

    Args:
        issues: Schema issues produced by one of the validators.
        context: Human-readable description of the validated payload.

    Raises:
        SysIdSchemaError: If any issue has ``error`` severity.
    """
    errors = schema_error_issues(issues)
    if not errors:
        return
    detail = "; ".join(f"{issue.path}: {issue.message}" for issue in errors)
    raise SysIdSchemaError(f"{context} failed schema validation: {detail}")


def _mapping(value: Any, path: str, issues: list[SysIdSchemaIssue]) -> dict[str, Any] | None:
    if value is None:
        return None
    if not isinstance(value, dict):
        issues.append(SysIdSchemaIssue(path, "Expected an object.", "error"))
        return None
    return value


def _warn_unknown_keys(
    payload: dict[str, Any],
    allowed: set[str],
    path: str,
    issues: list[SysIdSchemaIssue],
) -> None:
    for key in sorted(set(payload) - allowed):
        issues.append(
            SysIdSchemaIssue(
                f"{path}.{key}",
                "Unknown field will be ignored by current SysID loaders.",
            )
        )


def _validate_enum(value: Any, allowed: set[str], path: str, issues: list[SysIdSchemaIssue]) -> None:
    if str(value) not in allowed:
        issues.append(
            SysIdSchemaIssue(
                path,
                f"Unsupported value {value!r}; expected one of {sorted(allowed)}.",
                "error",
            )
        )


def _validate_positive_number(
    payload: dict[str, Any],
    key: str,
    path: str,
    issues: list[SysIdSchemaIssue],
) -> None:
    if key not in payload:
        return
    raw_value = payload[key]
    if isinstance(raw_value, bool) or not isinstance(raw_value, (int, float)):
        issues.append(SysIdSchemaIssue(path, "Expected a finite number.", "error"))
        return
    value = float(raw_value)
    if not math.isfinite(value):
        issues.append(SysIdSchemaIssue(path, "Expected a finite number.", "error"))
        return
    if value <= 0.0:
        issues.append(SysIdSchemaIssue(path, "Expected a positive number.", "error"))


def _validate_finite_number(
    payload: dict[str, Any],
    key: str,
    path: str,
    issues: list[SysIdSchemaIssue],
) -> None:
    if key not in payload:
        return
    value = payload[key]
    if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(float(value)):
        issues.append(SysIdSchemaIssue(path, "Expected a finite number.", "error"))


def _validate_nonnegative_number(
    payload: dict[str, Any],
    key: str,
    path: str,
    issues: list[SysIdSchemaIssue],
) -> None:
    if key not in payload:
        return
    raw_value = payload[key]
    if isinstance(raw_value, bool) or not isinstance(raw_value, (int, float)):
        issues.append(SysIdSchemaIssue(path, "Expected a finite number.", "error"))
        return
    value = float(raw_value)
    if not math.isfinite(value):
        issues.append(SysIdSchemaIssue(path, "Expected a finite number.", "error"))
        return
    if value < 0.0:
        issues.append(SysIdSchemaIssue(path, "Expected a non-negative number.", "error"))


def _validate_positive_int(payload: dict[str, Any], key: str, path: str, issues: list[SysIdSchemaIssue]) -> None:
    _validate_int_min(payload, key, path, issues, minimum=1)


def _validate_int_min(
    payload: dict[str, Any],
    key: str,
    path: str,
    issues: list[SysIdSchemaIssue],
    *,
    minimum: int,
) -> None:
    if key not in payload:
        return
    value = payload[key]
    if isinstance(value, bool) or not isinstance(value, int):
        issues.append(SysIdSchemaIssue(path, "Expected an integer.", "error"))
        return
    if value < minimum:
        issues.append(SysIdSchemaIssue(path, f"Expected an integer >= {minimum}.", "error"))


def _validate_integer(payload: dict[str, Any], key: str, path: str, issues: list[SysIdSchemaIssue]) -> None:
    if key not in payload:
        return
    value = payload[key]
    if isinstance(value, bool) or not isinstance(value, int):
        issues.append(SysIdSchemaIssue(path, "Expected an integer.", "error"))


def _validate_boolean(payload: dict[str, Any], key: str, path: str, issues: list[SysIdSchemaIssue]) -> None:
    if key in payload and not isinstance(payload[key], bool):
        issues.append(SysIdSchemaIssue(path, "Expected a boolean.", "error"))


def _validate_nonnegative_int(payload: dict[str, Any], key: str, path: str, issues: list[SysIdSchemaIssue]) -> None:
    if key not in payload:
        return
    value = payload[key]
    if isinstance(value, bool) or not isinstance(value, int):
        issues.append(SysIdSchemaIssue(path, "Expected an integer.", "error"))
        return
    if value < 0:
        issues.append(SysIdSchemaIssue(path, "Expected an integer >= 0.", "error"))
