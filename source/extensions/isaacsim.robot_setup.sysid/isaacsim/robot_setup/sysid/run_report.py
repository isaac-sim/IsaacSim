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

"""SysID run report JSON and validation plot artifacts."""

from __future__ import annotations

import json
import math
from collections.abc import Iterable
from dataclasses import asdict, dataclass, field
from pathlib import Path
from typing import Any

from .actuator_compatibility import is_actuator_delay_parameter
from .schema_validation import (
    SysIdSchemaIssue,
    schema_issues_to_dicts,
    validate_sysid_run_spec_payload,
)
from .telemetry_quality import TelemetryQualityReport, build_telemetry_quality_report
from .tuning_assessment import build_tuning_assessment_report

SYSID_RUN_REPORT_SCHEMA_VERSION = 4


@dataclass
class SysIdRunReport:
    """Machine-readable report for one SysID solve."""

    report_schema_version: int
    run_spec: dict[str, Any]
    schema_issues: list[dict[str, Any]]
    telemetry_quality: dict[str, Any]
    backend: str
    simulation_engine: str
    newton_config: dict[str, Any] | None
    robot_prim_path: str
    train_chunks: list[dict[str, Any]]
    untuned_validation_metrics: list[dict[str, Any]]
    tuned_validation_metrics: list[dict[str, Any]]
    validation_metrics: list[dict[str, Any]]
    untuned_validation_summary: dict[str, Any]
    tuned_validation_summary: dict[str, Any]
    validation_summary: dict[str, Any]
    validation_comparison: dict[str, Any]
    validation_artifacts: dict[str, Any]
    resampling_diagnostics: list[dict[str, Any]]
    tuning_assessment: dict[str, Any]
    parameters: list[dict[str, Any]]
    command_alignment_seconds: float
    actuator_command_delay_seconds: dict[str, Any]
    parameters_written: bool
    provenance_path: str
    final_status: dict[str, Any] | None
    last_accepted_status: dict[str, Any] | None
    selected_status: dict[str, Any] | None
    parameter_confidence: dict[str, Any] = field(default_factory=dict)
    extra: dict[str, Any] = field(default_factory=dict)

    def to_dict(self) -> dict[str, Any]:  # noqa: D102
        return asdict(self)


def export_sysid_run_report(
    path: str | Path,
    *,
    spec,  # noqa: ANN001
    prepared,  # noqa: ANN001
    result,  # noqa: ANN001
    schema_issues: Iterable[SysIdSchemaIssue | dict[str, Any]] | None = None,
    telemetry_quality: TelemetryQualityReport | None = None,
    validation_artifact_dir: str | Path | None = None,
    extra: dict[str, Any] | None = None,
) -> SysIdRunReport:
    """Write a JSON report and validation PNG artifacts for a completed run.

    Args:
        path: Value supplied for ``path``.
        spec: SysID run specification.
        prepared: Prepared SysID run state.
        result: SysID result being processed.
        schema_issues: Value supplied for ``schema_issues``.
        telemetry_quality: Value supplied for ``telemetry_quality``.
        validation_artifact_dir: Value supplied for ``validation_artifact_dir``.
        extra: Value supplied for ``extra``.

    Returns:
        Result produced by the operation.
    """  # noqa: DOC107
    report_path = Path(path).expanduser()
    report_path.parent.mkdir(parents=True, exist_ok=True)
    artifact_dir = Path(validation_artifact_dir).expanduser() if validation_artifact_dir else report_path.parent
    artifact_dir.mkdir(parents=True, exist_ok=True)
    artifacts = write_validation_artifacts(result, artifact_dir)
    report = build_sysid_run_report(
        spec=spec,
        prepared=prepared,
        result=result,
        schema_issues=schema_issues,
        telemetry_quality=telemetry_quality,
        validation_artifacts=artifacts,
        extra=extra,
    )
    report_path.write_text(json.dumps(report.to_dict(), indent=2), encoding="utf-8")
    return report


def build_sysid_run_report(  # noqa: D103
    *,
    spec,  # noqa: ANN001
    prepared,  # noqa: ANN001
    result,  # noqa: ANN001
    schema_issues: Iterable[SysIdSchemaIssue | dict[str, Any]] | None = None,
    telemetry_quality: TelemetryQualityReport | None = None,
    validation_artifacts: dict[str, Any] | None = None,
    extra: dict[str, Any] | None = None,
) -> SysIdRunReport:
    spec_payload = spec.to_dict()
    issue_payload = _issue_payloads(schema_issues)
    if schema_issues is None:
        issue_payload = schema_issues_to_dicts(validate_sysid_run_spec_payload(spec_payload))
    if telemetry_quality is None:
        chunk_specs = [chunk.spec for chunk in list(prepared.train_chunks) + list(prepared.validation_chunks)]
        telemetry_quality = build_telemetry_quality_report(prepared.raw_trajectory, chunk_specs)
    tuning_assessment = build_tuning_assessment_report(
        spec,
        prepared,
        result,
        telemetry_quality=telemetry_quality,
    )
    report_extra = dict(extra or {})
    report_extra["writeback"] = {
        "live_parameters_applied": bool(getattr(result, "live_parameters_applied", False)),
    }
    actuator_metadata = getattr(getattr(prepared, "bridge", None), "actuator_metadata", None)
    if isinstance(actuator_metadata, dict) and actuator_metadata:
        report_extra["actuator"] = actuator_metadata
    delay_payload = _command_delay_payload(prepared, result)
    alignment = float(spec.telemetry.resolved_command_alignment_seconds())
    return SysIdRunReport(
        report_schema_version=SYSID_RUN_REPORT_SCHEMA_VERSION,
        run_spec=spec_payload,
        schema_issues=issue_payload,
        telemetry_quality=telemetry_quality.to_dict(),
        backend=getattr(prepared, "backend_label", ""),
        simulation_engine=str(getattr(prepared, "simulation_engine", getattr(spec.simulation, "engine", ""))),
        newton_config=getattr(prepared, "newton_config", None),
        robot_prim_path=spec.simulation.robot_prim_path,
        train_chunks=_chunk_payloads(getattr(prepared, "train_chunks", [])),
        untuned_validation_metrics=_metric_payloads(getattr(result, "untuned_validation_metrics", [])),
        tuned_validation_metrics=_metric_payloads(
            getattr(result, "tuned_validation_metrics", getattr(result, "validation_metrics", []))
        ),
        validation_metrics=_metric_payloads(getattr(result, "validation_metrics", [])),
        untuned_validation_summary=validation_summary(getattr(result, "untuned_validation_metrics", [])),
        tuned_validation_summary=validation_summary(
            getattr(result, "tuned_validation_metrics", getattr(result, "validation_metrics", []))
        ),
        validation_summary=validation_summary(getattr(result, "validation_metrics", [])),
        validation_comparison=dict(getattr(result, "validation_comparison", {}) or {}),
        validation_artifacts=validation_artifacts or empty_validation_artifacts(result),
        resampling_diagnostics=_diagnostic_payloads(getattr(prepared, "resampling_diagnostics", []) or []),
        tuning_assessment=tuning_assessment.to_dict(),
        parameters=_parameter_rows(prepared, result),
        command_alignment_seconds=alignment,
        actuator_command_delay_seconds=delay_payload,
        parameters_written=bool(getattr(result, "parameters_written", False)),
        provenance_path=str(getattr(result, "provenance_path", "")),
        final_status=_status_to_dict(getattr(result, "final_status", None)),
        last_accepted_status=_status_to_dict(getattr(result, "last_accepted_status", None)),
        selected_status=_status_to_dict(
            getattr(result, "selected_status", None),
            source=str(getattr(result, "selected_status_source", "")),
        ),
        parameter_confidence=dict(getattr(result, "parameter_confidence", {}) or {}),
        extra=report_extra,
    )


def empty_validation_artifacts(result) -> dict[str, Any]:  # noqa: ANN001, D103
    return {
        "source": getattr(result, "validation_artifact_source", "configured_chunks"),
        "joint_position_overlay_png": "",
        "joint_position_overlay_skipped_reason": "",
        "joint_velocity_overlay_png": "",
        "joint_velocity_overlay_skipped_reason": "",
        "joint_angle_delta_png": "",
        "joint_angle_delta_skipped_reason": "",
        "joint_effort_overlay_png": "",
        "joint_effort_overlay_skipped_reason": "",
        "rmse_bar_png": "",
        "rmse_bar_skipped_reason": "",
        "render_png": getattr(result, "validation_render_path", ""),
        "render_skipped_reason": getattr(result, "validation_render_skipped_reason", ""),
        "trajectory_animation_mp4": "",
        "trajectory_animation_gif": "",
        "trajectory_animation_png": "",
        "trajectory_animation_skipped_reason": "",
    }


def write_validation_artifacts(result, output_dir: str | Path) -> dict[str, Any]:  # noqa: ANN001, D103
    output = empty_validation_artifacts(result)
    output_dir = Path(output_dir).expanduser().resolve()
    output_dir.mkdir(parents=True, exist_ok=True)
    rollouts = list(
        getattr(result, "tuned_validation_rollouts", None) or getattr(result, "validation_rollouts", []) or []
    )
    untuned_rollouts = list(getattr(result, "untuned_validation_rollouts", []) or [])
    output["source"] = getattr(result, "validation_artifact_source", "configured_chunks")
    path, skip = save_validation_series_overlay_png(
        rollouts,
        output_dir / "sysid_validation_joint_position_overlay.png",
        measured_attr="measured_positions",
        simulated_attr="simulated_positions",
        title="SysId validation joint positions",
        ylabel_prefix="q",
        untuned_rollouts=untuned_rollouts,
    )
    output["joint_position_overlay_png"] = path
    output["joint_position_overlay_skipped_reason"] = skip
    path, skip = save_validation_series_overlay_png(
        rollouts,
        output_dir / "sysid_validation_joint_velocity_overlay.png",
        measured_attr="measured_velocities",
        simulated_attr="simulated_velocities",
        title="SysId validation joint velocities",
        ylabel_prefix="dq",
        untuned_rollouts=untuned_rollouts,
    )
    output["joint_velocity_overlay_png"] = path
    output["joint_velocity_overlay_skipped_reason"] = skip
    path, skip = save_validation_joint_angle_delta_png(
        rollouts,
        output_dir / "sysid_validation_joint_angle_deltas.png",
    )
    output["joint_angle_delta_png"] = path
    output["joint_angle_delta_skipped_reason"] = skip
    path, skip = save_validation_series_overlay_png(
        rollouts,
        output_dir / "sysid_validation_joint_effort_overlay.png",
        measured_attr="measured_efforts",
        simulated_attr="simulated_efforts",
        title="SysId validation joint efforts",
        ylabel_prefix="tau",
        untuned_rollouts=untuned_rollouts,
    )
    output["joint_effort_overlay_png"] = path
    output["joint_effort_overlay_skipped_reason"] = skip
    path, skip = save_validation_rmse_bar_png(
        getattr(result, "validation_metrics", []),
        output_dir / "sysid_validation_rmse_by_channel.png",
        untuned_metrics=getattr(result, "untuned_validation_metrics", []),
    )
    output["rmse_bar_png"] = path
    output["rmse_bar_skipped_reason"] = skip
    if bool(getattr(result, "validation_animation_enabled", False)):
        output.update(
            save_validation_trajectory_animation(
                rollouts,
                output_dir,
                requested_path=str(getattr(result, "validation_animation_path", "") or ""),
                fps=int(getattr(result, "validation_animation_fps", 24) or 24),
                max_frames=int(getattr(result, "validation_animation_max_frames", 240) or 240),
            )
        )
    return output


def save_validation_series_overlay_png(  # noqa: D103
    rollouts,  # noqa: ANN001
    path: Path,
    *,
    measured_attr: str,
    simulated_attr: str,
    title: str,
    ylabel_prefix: str,
    untuned_rollouts=None,  # noqa: ANN001
) -> tuple[str, str]:
    rollouts = list(rollouts or [])
    untuned_rollouts = list(untuned_rollouts or [])
    if not rollouts:
        return "", "no validation rollout data available"
    num_series = 0
    for rollout in rollouts:
        measured = getattr(rollout, measured_attr, None) or []
        simulated = getattr(rollout, simulated_attr, None) or []
        num_series = max(num_series, min(len(measured), len(simulated)))
    if num_series < 1:
        return "", f"validation rollout has no {ylabel_prefix} series"
    plt, skip = _matplotlib_pyplot()
    if plt is None:
        return "", skip
    fig = None
    try:
        fig_height = max(3.0, min(18.0, 1.8 * num_series))
        fig, axes = plt.subplots(num_series, 1, sharex=False, figsize=(11.0, fig_height), squeeze=False)
        axes_flat = axes.reshape(-1)
        wrote_any = False
        for series_i in range(num_series):
            ax = axes_flat[series_i]
            for rollout_i, rollout in enumerate(rollouts):
                measured = getattr(rollout, measured_attr, None) or []
                simulated = getattr(rollout, simulated_attr, None) or []
                if series_i >= len(measured) or series_i >= len(simulated):
                    continue
                count = min(len(rollout.times), len(measured[series_i]), len(simulated[series_i]))
                if count < 2:
                    continue
                x_values, x_label = _x_values(rollout.times[:count])
                meas = [float(value) for value in measured[series_i][:count]]
                sim = [float(value) for value in simulated[series_i][:count]]
                if not _all_finite(meas) or not _all_finite(sim):
                    continue
                prefix = "validation" if len(rollouts) == 1 else f"validation {rollout_i + 1}"
                ax.plot(x_values, meas, linewidth=1.1, linestyle="--", label=f"{prefix} measured")
                if rollout_i < len(untuned_rollouts):
                    baseline = untuned_rollouts[rollout_i]
                    baseline_series = getattr(baseline, simulated_attr, None) or []
                    if series_i < len(baseline_series):
                        baseline_times = getattr(baseline, "times", []) or []
                        baseline_count = min(count, len(baseline_times), len(baseline_series[series_i]))
                        baseline_values = [float(value) for value in baseline_series[series_i][:baseline_count]]
                        if baseline_count >= 2 and _all_finite(baseline_values):
                            # The untuned pass can diverge onto its own timebase,
                            # so plot it against its own timestamps.
                            baseline_x, _ = _x_values(baseline_times[:baseline_count])
                            ax.plot(
                                baseline_x,
                                baseline_values,
                                linewidth=0.9,
                                alpha=0.85,
                                label=f"{prefix} untuned",
                            )
                tuned_label = f"{prefix} tuned" if untuned_rollouts else f"{prefix} simulated"
                ax.plot(x_values, sim, linewidth=1.0, label=tuned_label)
                wrote_any = True
            ax.set_ylabel(f"{ylabel_prefix}{series_i}")
            ax.grid(True, alpha=0.25)
        axes_flat[-1].set_xlabel(x_label if wrote_any else "sample index")
        axes_flat[0].set_title(title)
        if wrote_any:
            axes_flat[0].legend(loc="upper right", fontsize=8)
        else:
            return "", f"validation rollout data had no plottable finite {ylabel_prefix} series"
        fig.tight_layout()
        fig.savefig(path, dpi=140)
        return str(path), ""
    except Exception as exc:
        return "", f"failed to write {ylabel_prefix} overlay plot: {exc}"
    finally:
        if fig is not None:
            plt.close(fig)


def save_validation_joint_angle_delta_png(rollouts, path: Path) -> tuple[str, str]:  # noqa: ANN001, D103
    rollouts = list(rollouts or [])
    if not rollouts:
        return "", "no validation rollout data available"
    num_joints = 0
    for rollout in rollouts:
        num_joints = max(num_joints, min(len(rollout.measured_positions), len(rollout.simulated_positions)))
    if num_joints < 1:
        return "", "validation rollout has no joint position series"
    plt, skip = _matplotlib_pyplot()
    if plt is None:
        return "", skip
    fig = None
    try:
        fig_height = max(3.0, min(18.0, 1.8 * num_joints))
        fig, axes = plt.subplots(num_joints, 1, sharex=False, figsize=(11.0, fig_height), squeeze=False)
        axes_flat = axes.reshape(-1)
        wrote_any = False
        for joint_i in range(num_joints):
            ax = axes_flat[joint_i]
            for rollout_i, rollout in enumerate(rollouts):
                if joint_i >= len(rollout.measured_positions) or joint_i >= len(rollout.simulated_positions):
                    continue
                measured = rollout.measured_positions[joint_i]
                simulated = rollout.simulated_positions[joint_i]
                count = min(len(rollout.times), len(measured), len(simulated))
                if count < 2:
                    continue
                x_values, x_label = _x_values(rollout.times[:count])
                deltas = [float(simulated[idx]) - float(measured[idx]) for idx in range(count)]
                if not _all_finite(deltas):
                    continue
                label = "validation" if len(rollouts) == 1 else f"validation {rollout_i + 1}"
                ax.plot(x_values, deltas, linewidth=1.0, label=label)
                wrote_any = True
            ax.set_ylabel(f"q{joint_i} delta")
            ax.grid(True, alpha=0.25)
        axes_flat[-1].set_xlabel(x_label if wrote_any else "sample index")
        axes_flat[0].set_title("SysId validation joint angle deltas (simulated - measured)")
        if wrote_any:
            axes_flat[0].legend(loc="upper right", fontsize=8)
        else:
            return "", "validation rollout data had no plottable finite joint deltas"
        fig.tight_layout()
        fig.savefig(path, dpi=140)
        return str(path), ""
    except Exception as exc:
        return "", f"failed to write joint angle delta plot: {exc}"
    finally:
        if fig is not None:
            plt.close(fig)


def save_validation_rmse_bar_png(  # noqa: D103
    metrics,  # noqa: ANN001
    path: Path,
    *,
    untuned_metrics=None,  # noqa: ANN001
) -> tuple[str, str]:
    metrics = list(metrics or [])
    if not metrics:
        return "", "no validation metrics available"
    summary = validation_summary(metrics).get("overall", {})
    untuned_summary = validation_summary(untuned_metrics).get("overall", {})
    names = []
    tuned_values = []
    untuned_values = []
    for key, label in (
        ("position_rmse", "position"),
        ("velocity_rmse", "velocity"),
        ("torque_rmse", "torque"),
        ("end_effector_pose_rmse", "ee pose"),
        ("contact_force_rmse", "contact"),
    ):
        value = summary.get(key)
        untuned_value = untuned_summary.get(key)
        if value is not None and math.isfinite(float(value)):
            names.append(label)
            tuned_values.append(float(value))
            untuned_values.append(
                float(untuned_value)
                if untuned_value is not None and math.isfinite(float(untuned_value))
                else float("nan")
            )
    if not tuned_values:
        return "", "validation metrics had no finite RMSE values"
    plt, skip = _matplotlib_pyplot()
    if plt is None:
        return "", skip
    fig = None
    try:
        fig, ax = plt.subplots(1, 1, figsize=(8.0, 4.0))
        x_values = list(range(len(names)))
        have_untuned = any(math.isfinite(value) for value in untuned_values)
        if have_untuned:
            width = 0.38
            ax.bar(
                [value - width / 2 for value in x_values],
                untuned_values,
                width=width,
                color="#A0A0A0",
                label="untuned",
            )
            ax.bar(
                [value + width / 2 for value in x_values],
                tuned_values,
                width=width,
                color="#4F83CC",
                label="tuned",
            )
            ax.set_xticks(x_values, names)
            ax.legend(loc="upper right")
        else:
            ax.bar(names, tuned_values, color="#4F83CC")
        ax.set_ylabel("RMSE")
        ax.set_title("SysId validation RMSE by channel (native channel units)")
        ax.grid(True, axis="y", alpha=0.25)
        fig.tight_layout()
        fig.savefig(path, dpi=140)
        return str(path), ""
    except Exception as exc:
        return "", f"failed to write RMSE bar plot: {exc}"
    finally:
        if fig is not None:
            plt.close(fig)


def save_validation_trajectory_animation(
    rollouts,  # noqa: ANN001
    output_dir: str | Path,
    *,
    requested_path: str = "",
    fps: int = 24,
    max_frames: int = 240,
) -> dict[str, str]:
    """Write a validation ghost animation from measured and simulated joint traces.

    The left pane is a schematic kinematic ghost for quick phase checks; the right
    pane keeps the source joint traces visible with a moving time cursor.

    Args:
        rollouts: Value supplied for ``rollouts``.
        output_dir: Value supplied for ``output_dir``.
        requested_path: Value supplied for ``requested_path``.
        fps: Value supplied for ``fps``.
        max_frames: Value supplied for ``max_frames``.

    Returns:
        Result produced by the operation.
    """  # noqa: DOC107
    output = {
        "trajectory_animation_mp4": "",
        "trajectory_animation_gif": "",
        "trajectory_animation_png": "",
        "trajectory_animation_skipped_reason": "",
    }
    rollout, measured, simulated, times, skip = _first_plottable_position_rollout(rollouts)
    if rollout is None:
        output["trajectory_animation_skipped_reason"] = skip
        return output
    plt, skip = _matplotlib_pyplot()
    if plt is None:
        output["trajectory_animation_skipped_reason"] = skip
        return output

    try:
        from matplotlib import animation
    except Exception as exc:
        animation = None
        animation_skip = f"matplotlib animation unavailable: {exc}"
    else:
        animation_skip = ""

    output_dir = Path(output_dir).expanduser().resolve()
    output_dir.mkdir(parents=True, exist_ok=True)
    fps = max(1, int(fps))
    max_frames = max(1, int(max_frames))
    frame_indices = _animation_frame_indices(len(times), max_frames)
    if not frame_indices:
        output["trajectory_animation_skipped_reason"] = "validation rollout has no animation frames"
        return output

    requested = _contained_artifact_path(
        output_dir,
        requested_path,
        default_name="sysid_validation_trajectory_ghost.mp4",
    )
    mp4_path, gif_path, png_path = _animation_candidate_paths(requested)

    if animation is not None and (mp4_path is not None or gif_path is not None):
        fig = None
        anim = None
        try:
            fig, update_fn = _build_trajectory_animation_figure(plt, measured, simulated, times, frame_indices)
            anim = animation.FuncAnimation(
                fig,
                update_fn,
                frames=frame_indices,
                interval=1000.0 / fps,
                blit=False,
                repeat=True,
            )
            if mp4_path is not None and animation.writers.is_available("ffmpeg"):
                anim.save(str(mp4_path), writer=animation.FFMpegWriter(fps=fps, bitrate=1800), dpi=120)
                output["trajectory_animation_mp4"] = str(mp4_path)
                return output
            if gif_path is not None and animation.writers.is_available("pillow"):
                anim.save(str(gif_path), writer=animation.PillowWriter(fps=fps), dpi=100)
                output["trajectory_animation_gif"] = str(gif_path)
                return output
        except Exception as exc:
            animation_skip = f"failed to write trajectory animation: {exc}"
        finally:
            if fig is not None:
                plt.close(fig)
            del anim

    storyboard_path = png_path or output_dir / "sysid_validation_trajectory_ghost_storyboard.png"
    path, storyboard_skip = _save_validation_trajectory_storyboard_png(
        plt,
        measured,
        simulated,
        times,
        frame_indices,
        storyboard_path,
    )
    if path:
        output["trajectory_animation_png"] = path
        if animation_skip:
            output["trajectory_animation_skipped_reason"] = f"{animation_skip}; wrote PNG storyboard fallback"
        return output
    output["trajectory_animation_skipped_reason"] = storyboard_skip or animation_skip or "no animation writer available"
    return output


def _contained_artifact_path(output_dir: Path, requested_path: str, *, default_name: str) -> Path:
    """Resolve a requested artifact path without allowing writes outside ``output_dir``.

    Args:
        output_dir: Artifact directory that owns generated files.
        requested_path: Optional UI-provided relative or absolute path.
        default_name: Filename used when no path is requested.

    Returns:
        Resolved path contained by the artifact directory.
    """
    root = Path(output_dir).expanduser().resolve()
    if not requested_path:
        return root / default_name
    requested = Path(requested_path).expanduser()
    candidate = root / requested.name if requested.is_absolute() else (root / requested).resolve()
    try:
        candidate.relative_to(root)
    except ValueError:
        candidate = root / requested.name
    candidate.parent.mkdir(parents=True, exist_ok=True)
    return candidate


def validation_summary(metrics) -> dict[str, Any]:  # noqa: ANN001, D103
    metrics = list(metrics or [])
    return {
        "count": len(metrics),
        "overall": _validation_metric_group_summary(metrics),
        "by_excitation": {
            excitation: _validation_metric_group_summary(rows)
            for excitation, rows in _metrics_by_excitation(metrics).items()
        },
    }


def _metric_payloads(metrics) -> list[dict[str, Any]]:  # noqa: ANN001
    payloads = []
    for metric in metrics or []:
        if hasattr(metric, "to_dict"):
            payloads.append(metric.to_dict())
        elif isinstance(metric, dict):
            payloads.append(dict(metric))
    return payloads


def _chunk_payloads(chunks) -> list[dict[str, Any]]:  # noqa: ANN001
    payloads = []
    for index, chunk in enumerate(chunks or []):
        spec = chunk.spec
        payloads.append(
            {
                "name": spec.display_name(index),
                "role": spec.role,
                "excitation": spec.excitation,
                "start": float(spec.start),
                "end": float(spec.end),
                "sample_count": int(chunk.sample_count),
            }
        )
    return payloads


def _parameter_rows(prepared, result) -> list[dict[str, Any]]:  # noqa: ANN001
    config = getattr(prepared, "config", None)
    selected = getattr(result, "selected_status", None)
    if config is None or selected is None:
        return []
    authored_initial = getattr(config, "theta_authored_initial", None)
    initial_tensor = authored_initial if authored_initial is not None else config.theta_initial
    initial = initial_tensor.detach().cpu().tolist()
    final = list(getattr(selected, "theta", []) or [])
    confidence_by_index = {
        int(entry.get("index", -1)): entry
        for entry in (getattr(result, "parameter_confidence", {}) or {}).get("entries", [])
        if isinstance(entry, dict)
    }
    rows = []
    for idx, entry in enumerate(config.param_entries):
        if idx >= len(initial) or idx >= len(final):
            continue
        try:
            name = entry.display_name(config.trajectory.num_joints)
        except Exception:
            name = str(entry.param_type)
        row = {
            "name": name,
            "param_type": entry.param_type.value if hasattr(entry.param_type, "value") else str(entry.param_type),
            "initial": float(initial[idx]),
            "final": float(final[idx]),
            "delta": float(final[idx]) - float(initial[idx]),
        }
        confidence = confidence_by_index.get(idx)
        if confidence is not None:
            row["confidence"] = {
                "verdict": confidence.get("verdict", ""),
                "ten_percent_range": confidence.get("ten_percent_range"),
                "step": confidence.get("step"),
                "at_noise_floor": bool(confidence.get("at_noise_floor", False)),
            }
        rows.append(row)
    return rows


def _command_delay_payload(prepared, result) -> dict[str, Any]:  # noqa: ANN001
    config = getattr(prepared, "config", None)
    selected = getattr(result, "selected_status", None)
    if config is None or selected is None:
        return {}
    rows = []
    actuator_metadata = getattr(getattr(prepared, "bridge", None), "actuator_metadata", {})
    physics_dt = None
    if isinstance(actuator_metadata, dict):
        candidate_dt = actuator_metadata.get("physics_dt")
        if candidate_dt is not None:
            try:
                physics_dt = float(candidate_dt)
            except (TypeError, ValueError):
                physics_dt = None
    for idx, entry in enumerate(getattr(config, "param_entries", []) or []):
        value = entry.param_type.value if hasattr(entry.param_type, "value") else str(entry.param_type)
        if not is_actuator_delay_parameter(value) or idx >= len(selected.theta):
            continue
        row = {
            "dof_index": int(getattr(entry, "dof_index", -1)),
            "seconds": float(selected.theta[idx]),
        }
        if physics_dt is not None and physics_dt > 0.0:
            steps = max(0, int(round(row["seconds"] / physics_dt)))
            row["applied_steps"] = steps
            row["effective_seconds"] = steps * physics_dt
        rows.append(row)
    if not rows:
        return {}
    shared = next((row["seconds"] for row in rows if row["dof_index"] < 0), None)
    return {
        "shared_seconds": shared,
        "rows": rows,
        "physics_dt": physics_dt,
        "note": "Physical actuator delay is quantized to physics steps and persisted only to explicit actuator schemas.",
    }


def _status_to_dict(status, *, source: str = "") -> dict[str, Any] | None:  # noqa: ANN001
    if status is None:
        return None
    payload = {
        "iteration": int(status.iteration),
        "cost": float(status.cost),
        "damping": float(status.damping),
        "accepted": bool(status.accepted),
        "theta": [float(value) for value in status.theta],
    }
    if source:
        payload["source"] = source
    return payload


def _issue_payloads(issues: Iterable[SysIdSchemaIssue | dict[str, Any]] | None) -> list[dict[str, Any]]:
    payloads = []
    for issue in issues or []:
        if hasattr(issue, "to_dict"):
            payloads.append(issue.to_dict())
        elif isinstance(issue, dict):
            payloads.append(dict(issue))
    return payloads


def _diagnostic_payloads(diagnostics) -> list[dict[str, Any]]:  # noqa: ANN001
    payloads = []
    for diagnostic in diagnostics or []:
        if hasattr(diagnostic, "to_dict"):
            payloads.append(diagnostic.to_dict())
        elif isinstance(diagnostic, dict):
            payloads.append(dict(diagnostic))
    return payloads


def _first_plottable_position_rollout(
    rollouts: Any,
) -> tuple[Any | None, Any | None, Any | None, list[float], str]:
    try:
        import numpy as np
    except Exception as exc:
        return None, None, None, [], f"numpy unavailable: {exc}"
    for rollout in rollouts or []:
        measured = _joint_series_to_time_major_array(getattr(rollout, "measured_positions", None), np)
        simulated = _joint_series_to_time_major_array(getattr(rollout, "simulated_positions", None), np)
        if measured is None or simulated is None:
            continue
        count = min(
            measured.shape[0],
            simulated.shape[0],
            len(getattr(rollout, "times", []) or []),
        )
        joints = min(measured.shape[1], simulated.shape[1])
        if count < 2 or joints < 1:
            continue
        measured = measured[:count, :joints]
        simulated = simulated[:count, :joints]
        if not bool(np.isfinite(measured).all()) or not bool(np.isfinite(simulated).all()):
            continue
        times = [float(value) for value in getattr(rollout, "times", [])[:count]]
        if not _all_finite(times):
            times = [float(idx) for idx in range(count)]
        return rollout, measured, simulated, times, ""
    return None, None, None, [], "no validation rollout data with finite measured and simulated positions"


def _joint_series_to_time_major_array(series, np):  # noqa: ANN001, ANN202
    rows = list(series or [])
    if not rows:
        return None
    try:
        min_count = min(len(row) for row in rows)
    except TypeError:
        return None
    if min_count < 1:
        return None
    try:
        array = np.asarray([list(row)[:min_count] for row in rows], dtype=float)
    except (TypeError, ValueError):
        return None
    if array.ndim != 2:
        return None
    return array.T


def _animation_frame_indices(sample_count: int, max_frames: int) -> list[int]:
    sample_count = int(sample_count)
    max_frames = max(1, int(max_frames))
    if sample_count < 1:
        return []
    if sample_count <= max_frames:
        return list(range(sample_count))
    try:
        import numpy as np

        return sorted({int(value) for value in np.linspace(0, sample_count - 1, max_frames)})
    except Exception:
        stride = max(1, math.ceil(sample_count / max_frames))
        values = list(range(0, sample_count, stride))
        if values[-1] != sample_count - 1:
            values.append(sample_count - 1)
        return values


def _animation_candidate_paths(requested: Path) -> tuple[Path | None, Path | None, Path | None]:
    suffix = requested.suffix.lower()
    if suffix == ".gif":
        return None, requested, requested.with_suffix(".png")
    if suffix == ".png":
        return None, None, requested
    if suffix == ".mp4":
        return requested, requested.with_suffix(".gif"), requested.with_suffix(".png")
    base = requested.with_suffix("")
    return base.with_suffix(".mp4"), base.with_suffix(".gif"), base.with_suffix(".png")


def _build_trajectory_animation_figure(plt, measured, simulated, times, frame_indices):  # noqa: ANN001, ANN202
    fig, (ghost_ax, trace_ax) = plt.subplots(1, 2, figsize=(11.0, 5.0), gridspec_kw={"width_ratios": [1.0, 1.35]})
    joint_count = measured.shape[1]
    radius = max(1.5, float(joint_count) * 1.05)
    ghost_ax.set_xlim(-radius, radius)
    ghost_ax.set_ylim(-radius, radius)
    ghost_ax.set_aspect("equal", adjustable="box")
    ghost_ax.grid(True, alpha=0.2)
    ghost_ax.set_title("Validation pose ghost")
    ghost_ax.set_xlabel("schematic x")
    ghost_ax.set_ylabel("schematic y")
    (measured_line,) = ghost_ax.plot([], [], "--", color="#2F80ED", alpha=0.5, linewidth=2.2, label="measured ghost")
    (simulated_line,) = ghost_ax.plot([], [], "-", color="#F2994A", alpha=0.95, linewidth=2.4, label="simulated")
    ghost_ax.legend(loc="upper right", fontsize=8)
    time_label = ghost_ax.text(0.02, 0.02, "", transform=ghost_ax.transAxes, fontsize=9)

    x_values, x_label = _x_values(times)
    trace_count = min(4, joint_count)
    for joint_i in range(trace_count):
        trace_ax.plot(x_values, measured[:, joint_i], "--", color="#2F80ED", alpha=0.45, linewidth=1.0)
        trace_ax.plot(x_values, simulated[:, joint_i], "-", color="#F2994A", alpha=0.85, linewidth=1.0)
    cursor = trace_ax.axvline(x_values[frame_indices[0]], color="#30343B", linewidth=1.1)
    trace_ax.set_title("Joint traces")
    trace_ax.set_xlabel(x_label)
    trace_ax.set_ylabel("position")
    trace_ax.grid(True, alpha=0.2)
    fig.tight_layout()

    def update(frame_index: int):  # noqa: ANN202
        measured_line.set_data(*_schematic_chain_xy(measured[frame_index, :]))
        simulated_line.set_data(*_schematic_chain_xy(simulated[frame_index, :]))
        cursor.set_xdata([x_values[frame_index], x_values[frame_index]])
        time_label.set_text(f"t = {x_values[frame_index]:.3f}s")
        return measured_line, simulated_line, cursor, time_label

    return fig, update


def _save_validation_trajectory_storyboard_png(
    plt,  # noqa: ANN001
    measured,  # noqa: ANN001
    simulated,  # noqa: ANN001
    times,  # noqa: ANN001
    frame_indices,  # noqa: ANN001
    path: Path,
) -> tuple[str, str]:
    if not frame_indices:
        return "", "validation rollout has no storyboard frames"
    selected = _animation_frame_indices(len(frame_indices), 6)
    if not selected:
        selected = [0]
    storyboard_indices = [frame_indices[idx] for idx in selected]
    fig = None
    try:
        cols = min(3, len(storyboard_indices))
        rows = int(math.ceil(len(storyboard_indices) / cols))
        fig, axes = plt.subplots(rows, cols, figsize=(4.0 * cols, 3.6 * rows), squeeze=False)
        radius = max(1.5, float(measured.shape[1]) * 1.05)
        x_values, _ = _x_values(times)
        for ax in axes.reshape(-1):
            ax.axis("off")
        for ax, frame_index in zip(axes.reshape(-1), storyboard_indices):
            ax.axis("on")
            ax.plot(*_schematic_chain_xy(measured[frame_index, :]), "--", color="#2F80ED", alpha=0.55, linewidth=2.0)
            ax.plot(*_schematic_chain_xy(simulated[frame_index, :]), "-", color="#F2994A", alpha=0.95, linewidth=2.2)
            ax.set_xlim(-radius, radius)
            ax.set_ylim(-radius, radius)
            ax.set_aspect("equal", adjustable="box")
            ax.grid(True, alpha=0.2)
            ax.set_title(f"t = {x_values[frame_index]:.3f}s")
        fig.suptitle("SysId validation trajectory ghost storyboard")
        fig.tight_layout()
        fig.savefig(path, dpi=130)
        return str(path), ""
    except Exception as exc:
        return "", f"failed to write trajectory storyboard: {exc}"
    finally:
        if fig is not None:
            plt.close(fig)


def _schematic_chain_xy(q_values):  # noqa: ANN001, ANN202
    try:
        import numpy as np

        q = np.asarray(q_values, dtype=float)
        angles = np.cumsum(q)
        return np.r_[0.0, np.cumsum(np.cos(angles))], np.r_[0.0, np.cumsum(np.sin(angles))]
    except Exception:
        x_values = [0.0]
        y_values = [0.0]
        angle = 0.0
        for value in q_values:
            angle += float(value)
            x_values.append(x_values[-1] + math.cos(angle))
            y_values.append(y_values[-1] + math.sin(angle))
        return x_values, y_values


def _metrics_by_excitation(metrics) -> dict[str, list[Any]]:  # noqa: ANN001
    grouped: dict[str, list[Any]] = {}
    for metric in metrics:
        grouped.setdefault(metric.excitation or "custom", []).append(metric)
    return grouped


def _validation_metric_group_summary(metrics) -> dict[str, Any]:  # noqa: ANN001
    metrics = list(metrics or [])
    channels = (
        "position_rmse",
        "velocity_rmse",
        "torque_rmse",
        "end_effector_pose_rmse",
        "contact_force_rmse",
    )
    return {
        "normalized_cost": _weighted_mean_metric(metrics, "normalized_cost"),
        "position_rmse": _weighted_pooled_rmse(metrics, "position_rmse"),
        "velocity_rmse": _weighted_pooled_rmse(metrics, "velocity_rmse"),
        "torque_rmse": _weighted_pooled_rmse(metrics, "torque_rmse"),
        "end_effector_pose_rmse": _weighted_pooled_rmse(metrics, "end_effector_pose_rmse"),
        "contact_force_rmse": _weighted_pooled_rmse(metrics, "contact_force_rmse"),
        # Pooling silently skips non-finite chunks, so the untuned and tuned
        # summaries can cover different data. Publish what each pool actually
        # weighed to make that difference visible.
        "pooled_sample_counts": {name: _pooled_sample_count(metrics, name) for name in channels},
    }


def _weighted_mean_metric(metrics, name: str) -> float | None:  # noqa: ANN001
    weighted_sum = 0.0
    total_weight = 0
    for metric in metrics:
        value = getattr(metric, name, None)
        if value is None:
            continue
        value = float(value)
        if math.isfinite(value):
            weight = max(0, int(getattr(metric, "sample_count", 0)))
            weighted_sum += weight * value
            total_weight += weight
    if total_weight < 1:
        return None
    return float(weighted_sum / total_weight)


def _pooled_sample_count(metrics, name: str) -> int:  # noqa: ANN001
    """Return the sample count that actually contributed to one pooled channel."""
    total_weight = 0
    for metric in metrics:
        value = getattr(metric, name, None)
        if value is None:
            continue
        weight = max(0, int(getattr(metric, "sample_count", 0)))
        if math.isfinite(float(value)) and weight:
            total_weight += weight
    return total_weight


def _weighted_pooled_rmse(metrics, name: str) -> float | None:  # noqa: ANN001
    weighted_square_sum = 0.0
    total_weight = 0
    for metric in metrics:
        value = getattr(metric, name, None)
        if value is None:
            continue
        value = float(value)
        weight = max(0, int(getattr(metric, "sample_count", 0)))
        if math.isfinite(value) and weight:
            weighted_square_sum += weight * value * value
            total_weight += weight
    if total_weight < 1:
        return None
    return math.sqrt(weighted_square_sum / total_weight)


def _matplotlib_pyplot():  # noqa: ANN202
    try:
        import matplotlib

        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
    except Exception as exc:
        return None, f"matplotlib unavailable: {exc}"
    return plt, ""


def _x_values(times) -> tuple[list[float], str]:  # noqa: ANN001
    values = [float(value) for value in times]
    if values and _all_finite(values):
        t0 = values[0]
        return [value - t0 for value in values], "time from chunk start (s)"
    return list(range(len(values))), "sample index"


def _all_finite(values) -> bool:  # noqa: ANN001
    return all(math.isfinite(float(value)) for value in values)
