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

"""Run one complete SysID job inside an existing Kit runtime."""

from __future__ import annotations

from collections.abc import Callable
from dataclasses import dataclass
from typing import TYPE_CHECKING, Any

if TYPE_CHECKING:
    from .optimizer_base import OptimizationIterationStatus
    from .preflight import SysIdRunPreflight
    from .run_controller import SysIdPreparedRun
    from .run_session import SysIdRunResult
    from .run_spec import SysIdRunSpec
    from .schema_validation import SysIdSchemaIssue
    from .telemetry_quality import TelemetryQualityReport
    from .trajectory_csv import TrajectoryDataset

IterationCallback = Callable[[Any], None]
ProgressCallback = Callable[[str, float], None]
PreparedCallback = Callable[[Any, Any, Any, list[Any]], None]


def _report_preparation(on_progress: ProgressCallback | None, message: str) -> None:
    """Announce a preparation phase so long pre-solve work is not silent.

    Preparation reports a fraction of zero because the solve that follows
    reports its own progress from zero; any larger value here would make the
    reported fraction move backward once optimization starts.

    Args:
        on_progress: Caller's progress sink, if any.
        message: Name of the phase about to start.
    """
    if on_progress is not None:
        on_progress(message, 0.0)


@dataclass(slots=True)
class HeadlessSysIdJobRequest:
    """Configuration for one SysID run in an existing Kit process.

    Supply either an already opened `stage` or a path through `stage_path`. If
    neither is supplied, the run spec's stage path is used. Supplying an
    already loaded `trajectory` avoids reading telemetry a second time.

    Args:
        spec: Complete SysID run specification.
        stage_path: Optional stage path that overrides the run specification.
        stage: Optional already opened USD stage.
        trajectory: Optional already loaded trajectory.
        raw_spec_payload: Original JSON/YAML payload used to construct ``spec``.
            Supplying it preserves schema warnings for unknown or omitted fields
            that are normalized by :meth:`SysIdRunSpec.from_dict`.
        enable_extension: Whether to enable the backend extension before running.
        start_timeline: Whether to start the Kit timeline before preflight.
        timeline_warmup_updates: Number of Kit updates after starting the timeline.
        save_stage: Whether to save the job's stage after the selected parameters are written.

    Example:

    .. code-block:: python

        from isaacsim.robot_setup.sysid.headless_job import HeadlessSysIdJobRequest, run_headless_sysid_job
        from isaacsim.robot_setup.sysid.run_spec import SysIdRunSpec

        request = HeadlessSysIdJobRequest(spec=SysIdRunSpec.from_dict(payload))
        outcome = await run_headless_sysid_job(request)
        print(outcome.result.selected_status.cost)
    """

    spec: SysIdRunSpec
    stage_path: str = ""
    stage: Any | None = None
    trajectory: TrajectoryDataset | None = None
    raw_spec_payload: dict[str, Any] | None = None
    enable_extension: bool = True
    start_timeline: bool = True
    timeline_warmup_updates: int = 0
    save_stage: bool = False


@dataclass(slots=True)
class HeadlessSysIdJobOutcome:
    """Objects and diagnostics produced by one completed headless job.

    Args:
        request: Request used for the run.
        stage: USD stage used for preparation and execution.
        trajectory: Raw trajectory loaded for the run.
        prepared: Prepared optimizer and simulator state.
        preflight: Preflight result produced before optimization.
        telemetry_quality: Telemetry quality assessment.
        schema_issues: Run-spec schema issues.
        result: Completed SysID optimization result.
        stage_saved: Whether the requested stage save succeeded.
    """

    request: HeadlessSysIdJobRequest
    stage: Any
    trajectory: TrajectoryDataset
    prepared: SysIdPreparedRun
    preflight: SysIdRunPreflight
    telemetry_quality: TelemetryQualityReport
    schema_issues: list[SysIdSchemaIssue]
    result: SysIdRunResult
    stage_saved: bool = False


async def run_headless_sysid_job(
    request: HeadlessSysIdJobRequest,
    *,
    on_iteration: IterationCallback | None = None,
    on_progress: ProgressCallback | None = None,
    on_prepared: PreparedCallback | None = None,
) -> HeadlessSysIdJobOutcome:
    """Run one complete SysID job in the current Kit process.

    This function does not create or close `SimulationApp`. It is suitable for
    standalone launchers that already own the application lifecycle and for
    long-lived Kit services such as the Python server.

    Args:
        request: Job configuration and optional preloaded objects.
        on_iteration: Callback invoked for optimizer iteration updates.
        on_progress: Callback invoked for progress messages and fractions.
        on_prepared: Callback invoked after preflight and run preparation.

    Returns:
        Completed job outcome with preparation diagnostics and the solve result.

    Raises:
        ValueError: If request values are invalid.
        SysIdSchemaError: If the run specification violates its JSON contract.
        SysIdRuntimeUnavailableError: If no Kit runtime is available.
        RuntimeError: If the stage cannot be opened or saved.

    Example:

    .. code-block:: python

        outcome = await run_headless_sysid_job(request)
        print(outcome.prepared.backend_label)
    """
    if not isinstance(request.timeline_warmup_updates, int) or request.timeline_warmup_updates < 0:
        raise ValueError("timeline_warmup_updates must be a non-negative integer.")

    from .execution import run_sysid
    from .run_controller import SysIdRunController
    from .schema_validation import (
        raise_for_schema_errors,
        validate_sysid_run_spec_payload,
    )

    if request.enable_extension:
        await _enable_backend_extension()

    from .runtime import ensure_required_runtime_extensions_async

    await ensure_required_runtime_extensions_async(request.spec.simulation)

    stage = request.stage
    if stage is None:
        _report_preparation(on_progress, "Opening stage")
        stage = await _resolve_stage(request.stage_path or request.spec.stage.input_path)

    controller = SysIdRunController(request.spec)
    trajectory = request.trajectory
    if trajectory is None:
        _report_preparation(on_progress, "Loading telemetry")
        trajectory = controller.load_trajectory()
    schema_payload = request.raw_spec_payload if request.raw_spec_payload is not None else request.spec.to_dict()
    schema_issues = validate_sysid_run_spec_payload(schema_payload)
    raise_for_schema_errors(schema_issues, context="Run specification")

    if request.start_timeline:
        await _start_timeline(request.timeline_warmup_updates)
    timeline_playing = True if request.start_timeline else None

    _report_preparation(on_progress, "Running preflight")
    preflight = controller.preflight(
        stage=stage,
        trajectory=trajectory,
        timeline_playing=timeline_playing,
    )
    preflight.raise_for_errors()
    _report_preparation(on_progress, "Preparing run and pre-solve check")
    prepared = controller.prepare(
        stage,
        trajectory=trajectory,
        timeline_playing=timeline_playing,
    )
    # `prepare` assesses telemetry quality while building the check report; reuse
    # that result rather than repeating the scan of the same trajectory.
    telemetry_quality = prepared.telemetry_quality
    if on_prepared is not None:
        try:
            on_prepared(prepared, preflight, telemetry_quality, schema_issues)
        except BaseException:
            await _cleanup_prepared_run(prepared, request.spec.outputs.remove_clones_after_run)
            raise

    result = await run_sysid(
        request.spec,
        on_iteration=on_iteration,
        on_progress=on_progress,
        prepared_run=prepared,
    )
    stage_saved = await _save_stage(stage) if request.save_stage else False
    return HeadlessSysIdJobOutcome(
        request=request,
        stage=stage,
        trajectory=trajectory,
        prepared=prepared,
        preflight=preflight,
        telemetry_quality=telemetry_quality,
        schema_issues=schema_issues,
        result=result,
        stage_saved=stage_saved,
    )


def summarize_headless_sysid_job(outcome: HeadlessSysIdJobOutcome) -> dict[str, Any]:
    """Build a compact JSON-safe summary for a completed job.

    Args:
        outcome: Completed headless job outcome.

    ``ok`` covers the job's mechanical outcome: the solve completed and, when
    the caller asked for one, the stage save succeeded. A requested save that
    failed leaves identified parameters only in memory, so reporting success
    there would tell automation the run is durable when it is not. The
    pre-solve check verdict stays advisory and is surfaced separately as
    ``check_report_ok``.

    Returns:
        Compact summary containing status, parameters, validation, and provenance.

    Example:

    .. code-block:: python

        summary = summarize_headless_sysid_job(outcome)
        print(summary["selected_status"]["cost"])
    """
    result = outcome.result
    prepared = outcome.prepared
    from .run_report import SYSID_RUN_REPORT_SCHEMA_VERSION, validation_summary

    save_requested = bool(outcome.request.save_stage)
    stage_saved = bool(outcome.stage_saved)
    check_report = getattr(prepared, "check_report", None)
    return {
        "ok": stage_saved or not save_requested,
        "report_schema_version": SYSID_RUN_REPORT_SCHEMA_VERSION,
        "backend": prepared.backend_label,
        "simulation_engine": getattr(prepared, "simulation_engine", outcome.request.spec.simulation.engine),
        "robot_prim_path": outcome.request.spec.simulation.robot_prim_path,
        "telemetry_source": outcome.request.spec.telemetry.source_path,
        "stage_save_requested": save_requested,
        "stage_saved": stage_saved,
        "check_report_ok": None if check_report is None else bool(getattr(check_report, "ok", True)),
        "parameters_written": bool(result.parameters_written),
        "live_parameters_applied": bool(getattr(result, "live_parameters_applied", False)),
        "provenance_path": result.provenance_path,
        "final_status": _status_to_dict(result.final_status),
        "last_accepted_status": (
            _status_to_dict(result.last_accepted_status) if result.last_accepted_status is not None else None
        ),
        "selected_status": _status_to_dict(result.selected_status, source=result.selected_status_source),
        "untuned_validation_metrics": [metric.to_dict() for metric in result.untuned_validation_metrics],
        "tuned_validation_metrics": [metric.to_dict() for metric in result.tuned_validation_metrics],
        # Backward-compatible tuned alias.
        "validation_metrics": [metric.to_dict() for metric in result.validation_metrics],
        "untuned_validation_summary": validation_summary(result.untuned_validation_metrics),
        "tuned_validation_summary": validation_summary(result.tuned_validation_metrics),
        "validation_summary": validation_summary(result.validation_metrics),
        "validation_comparison": dict(result.validation_comparison or {}),
        "parameter_confidence": dict(result.parameter_confidence or {}),
        "schema_issues": [issue.to_dict() for issue in outcome.schema_issues],
    }


def _status_to_dict(status: OptimizationIterationStatus, *, source: str = "") -> dict[str, Any]:
    """Convert one optimizer status to JSON-safe values.

    Args:
        status: Optimizer status to serialize.
        source: Optional label describing how the status was selected.

    Returns:
        A JSON-compatible optimizer status.
    """
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


async def _enable_backend_extension() -> None:
    """Enable the backend extension and yield one Kit update."""
    import omni.kit.app

    app = omni.kit.app.get_app()
    manager = app.get_extension_manager()
    if not manager.is_extension_enabled("isaacsim.robot_setup.sysid"):
        manager.set_extension_enabled_immediate("isaacsim.robot_setup.sysid", True)
        await app.next_update_async()


async def _resolve_stage(stage_path: str) -> Any:
    """Open the requested stage or return the current stage.

    Args:
        stage_path: Optional USD stage path to open.

    Returns:
        The opened or current USD stage.
    """
    from .runtime import get_current_stage, open_stage_async, require_kit_runtime

    require_kit_runtime("A headless SysID job")
    if stage_path:
        return await open_stage_async(stage_path)
    stage = get_current_stage()
    if stage is None:
        raise RuntimeError("No USD stage is open and the run spec does not provide stage.input_path.")
    return stage


async def _start_timeline(warmup_updates: int) -> None:
    """Start the timeline and yield the requested Kit updates.

    Args:
        warmup_updates: Number of Kit updates to await after playback starts.
    """
    from isaacsim.core.experimental.utils import app as app_utils

    if not app_utils.is_playing():
        app_utils.play()
    if warmup_updates:
        await app_utils.update_app_async(steps=warmup_updates)


async def _save_stage(stage: Any) -> bool:
    """Save the stage used by this job.

    Args:
        stage: USD stage prepared and executed by the job.

    Returns:
        Whether the operation succeeded.
    """
    from isaacsim.core.experimental.utils import stage as stage_utils

    if stage is stage_utils.get_current_stage():
        result = stage_utils.save_stage()
        return True if result is None else bool(result)

    root_layer = stage.GetRootLayer()
    identifier = str(getattr(root_layer, "realPath", "") or getattr(root_layer, "identifier", ""))
    if bool(getattr(root_layer, "anonymous", False)) or not identifier or identifier.startswith("anon:"):
        raise RuntimeError("Cannot save a non-current stage whose root layer is anonymous.")
    result = root_layer.Save()
    return True if result is None else bool(result)


async def _cleanup_prepared_run(prepared: SysIdPreparedRun, remove_clones: bool) -> None:
    """Release bridge resources when execution cannot start.

    Args:
        prepared: Prepared run whose resources must be released.
        remove_clones: Whether rollout clones should also be removed.
    """
    optimizer = prepared.optimizer
    cleanup_async = getattr(optimizer, "cleanup_bridge_async", None)
    if callable(cleanup_async):
        await cleanup_async(remove_clones=remove_clones)
        return
    cleanup = getattr(optimizer, "cleanup_bridge", None)
    if callable(cleanup):
        cleanup(remove_clones=remove_clones)
