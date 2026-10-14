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

"""Async optimization session: owns the run loop, USD writes, provenance, and bridge cleanup."""

from __future__ import annotations

import asyncio
from collections.abc import Callable
from dataclasses import dataclass, field
from enum import Enum, auto

import carb

from .optimizer_base import OptimizationIterationStatus, OptimizerConfig
from .provenance import ProvenanceWriter, build_provenance_record
from .rollout_plot_data import SysIdRolloutPlotData
from .run_controller import SysIdPreparedRun, write_optimized_parameters_to_usd
from .trajectory_segments import (
    DEFAULT_VALIDATION_ABSOLUTE_CHANGE_THRESHOLD,
    DEFAULT_VALIDATION_RELATIVE_CHANGE_THRESHOLD,
    ChunkValidationMetric,
    TelemetryChunkRunSpec,
    TrajectoryChunk,
    build_validation_comparison,
)
from .usd_write_layer import resolve_multiphysics_write_targets

IterationCallback = Callable[[OptimizationIterationStatus], None]
ProgressCallback = Callable[[str, float], None]


class RunState(Enum):  # noqa: D101
    IDLE = auto()
    RUNNING = auto()
    CANCELLED = auto()
    DONE = auto()
    ERROR = auto()


@dataclass
class _LayerSnapshot:
    """Rollback state for one USD layer."""

    layer: object
    prim_paths: tuple[str, ...] = ()
    existing_paths: tuple[str, ...] = ()
    backup_layer: object | None = None
    serialized_content: str | None = None


@dataclass
class SysIdRunResult:
    """Everything produced by one completed optimization session."""

    final_status: OptimizationIterationStatus
    selected_status: OptimizationIterationStatus
    selected_status_source: str
    last_accepted_status: OptimizationIterationStatus | None = None
    # ``validation_metrics``/``validation_rollouts`` remain the tuned aliases
    # for report consumers written before paired validation was introduced.
    validation_metrics: list[ChunkValidationMetric] = field(default_factory=list)
    validation_rollouts: list[SysIdRolloutPlotData] = field(default_factory=list)
    untuned_validation_metrics: list[ChunkValidationMetric] = field(default_factory=list)
    untuned_validation_rollouts: list[SysIdRolloutPlotData] = field(default_factory=list)
    tuned_validation_metrics: list[ChunkValidationMetric] = field(default_factory=list)
    tuned_validation_rollouts: list[SysIdRolloutPlotData] = field(default_factory=list)
    validation_comparison: dict = field(default_factory=dict)
    validation_artifact_source: str = "configured_chunks"
    parameters_written: bool = False
    live_parameters_applied: bool = False
    provenance_path: str = ""
    validation_render_path: str = ""
    validation_render_skipped_reason: str = ""
    validation_animation_enabled: bool = False
    validation_animation_path: str = ""
    validation_animation_fps: int = 24
    validation_animation_max_frames: int = 240
    #: `ParameterConfidenceReport.to_dict()` payload (empty when estimation is off/skipped).
    parameter_confidence: dict = field(default_factory=dict)


class SysIdRunSession:
    """Executes a prepared SysID run asynchronously.

    The session owns:
    - the optimizer run loop
    - USD parameter write-back
    - validation chunk evaluation
    - provenance authoring
    - bridge cleanup

    The caller supplies lightweight callbacks to push progress to the UI.

    Args:
        prepared_run: Constructor value for ``prepared_run``.
        apply_params_to_usd: Constructor value for ``apply_params_to_usd``.
        write_usd_provenance: Constructor value for ``write_usd_provenance``.
        export_provenance_sidecar: Constructor value for ``export_provenance_sidecar``.
        provenance_sidecar_path: Constructor value for ``provenance_sidecar_path``.
        remove_clones_after_run: Constructor value for ``remove_clones_after_run``.
        robot_path: Constructor value for ``robot_path``.
        validation_animation_enabled: Constructor value for ``validation_animation_enabled``.
        validation_animation_path: Constructor value for ``validation_animation_path``.
        validation_animation_fps: Constructor value for ``validation_animation_fps``.
        validation_animation_max_frames: Constructor value for ``validation_animation_max_frames``.
        validation_relative_change_threshold: Relative RMSE change required for a comparison verdict.
        validation_absolute_change_threshold: Absolute RMSE change required for a comparison verdict.
        compute_parameter_confidence: Constructor value for ``compute_parameter_confidence``.
        parameter_confidence_rollout_budget: Constructor value for ``parameter_confidence_rollout_budget``.
        instance_local_writeback: Author accepted parameters as overrides in the open stage root.
    """

    def __init__(
        self,
        prepared_run: SysIdPreparedRun,
        *,
        apply_params_to_usd: bool = True,
        write_usd_provenance: bool = True,
        export_provenance_sidecar: bool = False,
        provenance_sidecar_path: str = "",
        remove_clones_after_run: bool = True,
        robot_path: str = "",
        validation_animation_enabled: bool = False,
        validation_animation_path: str = "",
        validation_animation_fps: int = 24,
        validation_animation_max_frames: int = 240,
        validation_relative_change_threshold: float = DEFAULT_VALIDATION_RELATIVE_CHANGE_THRESHOLD,
        validation_absolute_change_threshold: float = DEFAULT_VALIDATION_ABSOLUTE_CHANGE_THRESHOLD,
        compute_parameter_confidence: bool = True,
        parameter_confidence_rollout_budget: int = 2_000_000,
        instance_local_writeback: bool = False,
    ) -> None:
        self._prepared = prepared_run
        self._apply_params_to_usd = apply_params_to_usd
        self._write_usd_provenance = write_usd_provenance
        self._export_provenance_sidecar = export_provenance_sidecar
        self._provenance_sidecar_path = provenance_sidecar_path
        self._remove_clones = remove_clones_after_run
        self._robot_path = robot_path
        self._validation_animation_enabled = bool(validation_animation_enabled)
        self._validation_animation_path = validation_animation_path
        self._validation_animation_fps = int(validation_animation_fps)
        self._validation_animation_max_frames = int(validation_animation_max_frames)
        self._validation_relative_change_threshold = float(validation_relative_change_threshold)
        self._validation_absolute_change_threshold = float(validation_absolute_change_threshold)
        self._compute_parameter_confidence = bool(compute_parameter_confidence)
        self._parameter_confidence_budget = int(parameter_confidence_rollout_budget)
        self._instance_local_writeback = bool(instance_local_writeback)
        self._state = RunState.IDLE

    @property
    def state(self) -> RunState:  # noqa: D102
        return self._state

    def cancel(self) -> None:  # noqa: D102
        self._prepared.optimizer.request_cancel()

    async def run_async(
        self,
        on_iteration: IterationCallback | None = None,
        on_progress: ProgressCallback | None = None,
    ) -> SysIdRunResult:
        """Run optimization and all post-opt steps. Returns when complete.

        Raises ``asyncio.CancelledError`` on cancellation, any other exception
        on failure. Bridge cleanup always runs in ``finally``.

        Args:
            on_iteration: Optional iteration callback.
            on_progress: Optional progress callback.

        Returns:
            Completed run result with selected parameters and validation artifacts.
        """
        if self._state != RunState.IDLE:
            raise RuntimeError("SysIdRunSession may only be used once.")

        self._state = RunState.RUNNING
        prepared = self._prepared
        config = prepared.config
        optimizer = prepared.optimizer
        optimizer.reset_cancel()
        last_accepted: OptimizationIterationStatus | None = None
        params_written = False
        live_params_applied = False
        layer_snapshots: list[_LayerSnapshot] = []
        writeback_bridge: object | None = None
        live_apply_attempted = False
        authored_theta: object | None = None

        def track_iteration(status: OptimizationIterationStatus) -> None:
            nonlocal last_accepted
            if status.accepted and int(status.iteration) > 0:
                last_accepted = status
            if on_iteration is not None:
                on_iteration(status)

        try:
            validation_chunks = list(getattr(prepared, "validation_chunks", []) or [])
            validation_source = "configured_chunks"
            if not validation_chunks:
                validation_chunks = _training_replay_validation_chunks(prepared, config)
                if validation_chunks:
                    validation_source = "training_chunks_fallback"

            authored_theta = getattr(prepared, "theta_authored_initial", None)
            if authored_theta is None:
                authored_theta = getattr(config, "theta_authored_initial", None)
            if authored_theta is None:
                authored_theta = getattr(config, "theta_initial", None)
            final = await optimizer.run_async(
                config,
                on_iteration=track_iteration,
                on_progress=on_progress,
            )
            selected, selected_source = _select_status_for_outputs(config, final, last_accepted)

            stage = prepared.stage

            # Gradient work is finished: drop the differentiable bridge's captured
            # CUDA graphs, tapes, and per-substep buffers before the forward-only
            # validation replay and confidence batch (which needs a fresh
            # multi-world context and otherwise OOMs on 16 GB cards).
            writeback_bridge = optimizer.get_bridge() if hasattr(optimizer, "get_bridge") else None
            release_fn = getattr(writeback_bridge, "release_rollout_memory", None)
            if callable(release_fn):
                try:
                    release_fn()
                except Exception as exc:
                    carb.log_warn(f"SysId: could not release solve rollout memory: {exc}")

            validation_metrics: list[ChunkValidationMetric] = []
            validation_rollouts: list[SysIdRolloutPlotData] = []
            untuned_validation_metrics: list[ChunkValidationMetric] = []
            untuned_validation_rollouts: list[SysIdRolloutPlotData] = []
            if validation_chunks and authored_theta is not None:
                (
                    untuned_validation_metrics,
                    untuned_validation_rollouts,
                    validation_metrics,
                    validation_rollouts,
                ) = await _evaluate_validation_pair(
                    optimizer,
                    config,
                    validation_chunks,
                    authored_theta,
                    selected.theta,
                    validation_source=validation_source,
                    on_progress=on_progress,
                )
            elif validation_chunks:
                validation_metrics, validation_rollouts = await _evaluate_validation_pass(
                    optimizer,
                    config,
                    validation_chunks,
                    selected.theta,
                    pass_name="tuned",
                    validation_source=validation_source,
                    on_progress=on_progress,
                )
            validation_comparison = build_validation_comparison(
                untuned_validation_metrics,
                validation_metrics,
                relative_change_threshold=self._validation_relative_change_threshold,
                absolute_change_threshold=self._validation_absolute_change_threshold,
                source=validation_source,
                holdout=validation_source == "configured_chunks",
            )

            # Diagnostic only; failures degrade to an empty payload. Run this
            # before durable USD writes so cancellation cannot leave parameters
            # applied to a run that never completed.
            confidence_payload: dict = {}
            if self._compute_parameter_confidence and config.param_entries:
                confidence_chunks = validation_chunks or list(prepared.train_chunks)
                chunk_source = "validation_chunks" if validation_chunks else "train_chunks_fallback"
                try:
                    from .analysis import build_parameter_confidence_report

                    confidence_report = await build_parameter_confidence_report(
                        optimizer,
                        config,
                        confidence_chunks,
                        [float(value) for value in selected.theta],
                        max_rollout_sample_budget=self._parameter_confidence_budget,
                        on_progress=on_progress,
                    )
                    confidence_report.chunk_source = chunk_source
                    confidence_payload = confidence_report.to_dict()
                    for line in confidence_report.summary_lines():
                        carb.log_info(f"SysId confidence: {line}")
                except Exception as exc:
                    carb.log_warn(f"SysId: parameter confidence estimation failed: {exc}")

            # Evaluated here, ahead of the durable write, so that refusing the
            # write on a regression only needs a run-spec flag at this point.
            # The verdict is advisory today and never blocks the write.
            regressed_channels = _validation_regression_channels(validation_comparison)
            if regressed_channels:
                carb.log_warn(
                    f"SysId: accepted parameters regress validation channel(s) {', '.join(regressed_channels)} "
                    "against the authored baseline; continuing because regression gating is advisory."
                )

            # Durable writes happen only after the accepted candidate and the
            # authored baseline have both replayed successfully.  Temporary
            # explicit-actuator promotions remain session-layer-only until here.
            if self._apply_params_to_usd:
                if stage is None:
                    raise RuntimeError("USD parameter writeback was requested, but the prepared run has no stage.")
                bridge = writeback_bridge
                try:
                    neutral_write_target = None
                    solver_write_target = None
                    physics_backend = str(getattr(prepared, "physics_backend", ""))
                    if self._instance_local_writeback:
                        # Interactive runs must affect only the selected robot instance.  A root-layer
                        # override is stronger than the referenced multi-physics asset and is saved
                        # with the open stage rather than mutating a shared asset payload.
                        neutral_write_target = stage.GetRootLayer()
                        solver_write_target = neutral_write_target
                    elif self._robot_path and physics_backend in ("newton", "physx"):
                        physics_solver = str(getattr(prepared, "physics_solver", ""))
                        solver_layer_name = "physx" if physics_backend == "physx" else ""
                        if physics_backend == "newton" and physics_solver == "mujoco":
                            solver_layer_name = "mujoco"
                        write_targets = resolve_multiphysics_write_targets(
                            stage,
                            self._robot_path,
                            solver_layer_name=solver_layer_name,
                        )
                        if write_targets is not None:
                            neutral_write_target = write_targets.neutral
                            solver_write_target = write_targets.solver
                    layer_snapshots = _snapshot_stage_layers(
                        stage,
                        _writeback_prim_paths(
                            prepared,
                            bridge,
                            robot_path=self._robot_path,
                            include_provenance=self._write_usd_provenance,
                        ),
                        _current_edit_target(stage),
                        _root_layer(stage),
                        neutral_write_target,
                        solver_write_target,
                    )
                    params_written = write_optimized_parameters_to_usd(
                        stage,
                        prepared.parameter_space,
                        config,
                        selected.theta,
                        max(1, len(prepared.link_paths)),
                        bridge,
                        physics_backend=str(getattr(prepared, "physics_backend", "")),
                        neutral_write_layer=neutral_write_target,
                        solver_write_layer=solver_write_target,
                    )
                    live_apply = getattr(bridge, "apply_accepted_parameters_to_live_articulation", None)
                    if params_written and physics_backend == "physx" and callable(live_apply):
                        live_apply_attempted = True
                        live_params_applied = bool(live_apply(selected.theta, config.param_entries))
                        live_apply_attempted = live_params_applied
                except Exception as exc:
                    live_rollback_error = _try_restore_live_parameters(
                        bridge,
                        authored_theta,
                        getattr(config, "param_entries", []),
                        required=live_apply_attempted,
                    )
                    live_apply_attempted = False
                    try:
                        _restore_stage_layers(layer_snapshots)
                    except Exception as rollback_exc:
                        raise RuntimeError(
                            f"Requested USD parameter writeback failed: {exc}; rollback also failed: {rollback_exc}"
                        ) from exc
                    layer_snapshots = []
                    if live_rollback_error:
                        raise RuntimeError(
                            f"Requested USD parameter writeback failed: {exc}; "
                            f"live rollback also failed: {live_rollback_error}"
                        ) from exc
                    raise RuntimeError(f"Requested USD parameter writeback failed: {exc}") from exc
                if not params_written:
                    reason = "USD parameter writeback was requested, but no selected parameter was written."
                    try:
                        _restore_stage_layers(layer_snapshots)
                    except Exception as rollback_exc:
                        raise RuntimeError(f"{reason}; rollback also failed: {rollback_exc}") from rollback_exc
                    finally:
                        layer_snapshots = []
                    raise RuntimeError(reason)

            provenance_path = ""
            if stage is not None and (self._write_usd_provenance or self._export_provenance_sidecar):
                if self._write_usd_provenance and not layer_snapshots:
                    layer_snapshots = _snapshot_stage_layers(
                        stage,
                        _minimal_prim_paths([self._robot_path]),
                        _current_edit_target(stage),
                    )
                provenance_path = _write_provenance(
                    stage,
                    prepared,
                    config,
                    selected,
                    backend_label=prepared.backend_label,
                    robot_path=self._robot_path,
                    validation_metrics=validation_metrics,
                    validation_comparison=validation_comparison,
                    write_usd=self._write_usd_provenance,
                    export_sidecar=self._export_provenance_sidecar,
                    sidecar_path=self._provenance_sidecar_path,
                )

            self._state = RunState.DONE
            return SysIdRunResult(
                final_status=final,
                selected_status=selected,
                selected_status_source=selected_source,
                last_accepted_status=last_accepted,
                validation_metrics=validation_metrics,
                validation_rollouts=validation_rollouts,
                untuned_validation_metrics=untuned_validation_metrics,
                untuned_validation_rollouts=untuned_validation_rollouts,
                tuned_validation_metrics=validation_metrics,
                tuned_validation_rollouts=validation_rollouts,
                validation_comparison=validation_comparison,
                validation_artifact_source=validation_source,
                parameters_written=params_written,
                live_parameters_applied=live_params_applied,
                provenance_path=provenance_path,
                validation_animation_enabled=self._validation_animation_enabled,
                validation_animation_path=self._validation_animation_path,
                validation_animation_fps=self._validation_animation_fps,
                validation_animation_max_frames=self._validation_animation_max_frames,
                parameter_confidence=confidence_payload,
            )

        except asyncio.CancelledError:
            live_rollback_error = _try_restore_live_parameters(
                writeback_bridge,
                authored_theta,
                getattr(config, "param_entries", []),
                required=live_apply_attempted,
            )
            if live_rollback_error:
                carb.log_error(f"SysId: live-parameter rollback after cancellation failed: {live_rollback_error}")
            if layer_snapshots:
                try:
                    _restore_stage_layers(layer_snapshots)
                except Exception as exc:
                    carb.log_error(f"SysId: USD rollback after cancellation failed: {exc}")
            self._state = RunState.CANCELLED
            raise
        except Exception:
            live_rollback_error = _try_restore_live_parameters(
                writeback_bridge,
                authored_theta,
                getattr(config, "param_entries", []),
                required=live_apply_attempted,
            )
            if live_rollback_error:
                carb.log_error(f"SysId: live-parameter rollback after failure failed: {live_rollback_error}")
            if layer_snapshots:
                try:
                    _restore_stage_layers(layer_snapshots)
                except Exception as exc:
                    carb.log_error(f"SysId: USD rollback after failure failed: {exc}")
            self._state = RunState.ERROR
            raise
        finally:
            cleanup_async = getattr(optimizer, "cleanup_bridge_async", None)
            try:
                if callable(cleanup_async):
                    await cleanup_async(remove_clones=self._remove_clones)
                else:
                    optimizer.cleanup_bridge(remove_clones=self._remove_clones)
            except Exception as exc:
                carb.log_warn(f"SysId: bridge cleanup failed: {exc}")


def _select_status_for_outputs(
    config: OptimizerConfig,
    final: OptimizationIterationStatus,
    last_accepted: OptimizationIterationStatus | None,
) -> tuple[OptimizationIterationStatus, str]:
    if final.accepted and int(final.iteration) > 0:
        return final, "final_accepted"
    if last_accepted is not None:
        return last_accepted, "last_accepted"
    if final.theta:
        return final, "initial_fallback"
    theta = config.theta_initial.detach().cpu().tolist()
    return (
        OptimizationIterationStatus(
            iteration=0,
            cost=float(final.cost),
            damping=float(final.damping),
            accepted=False,
            theta=[float(value) for value in theta],
            rollout=final.rollout,
        ),
        "initial_fallback",
    )


def _training_replay_validation_chunks(
    prepared: SysIdPreparedRun,
    config: OptimizerConfig,
) -> list[TrajectoryChunk]:
    """Return validation views for every training chunk.

    Args:
        prepared: Prepared run containing the resampled training chunks.
        config: Optimizer configuration used as a legacy single-chunk fallback.

    Returns:
        Validation-role chunks covering all data used by the training objective.
    """
    train_chunks = list(getattr(prepared, "train_chunks", []) or [])
    if not train_chunks:
        trajectory = getattr(config, "trajectory", None)
        times = getattr(trajectory, "times", None)
        if trajectory is None or times is None or len(times) < 2:
            return []
        train_chunks = [
            TrajectoryChunk(
                spec=TelemetryChunkRunSpec(
                    name="Training replay",
                    role="train",
                    excitation="training_data",
                    start=float(times[0]),
                    end=float(times[-1]),
                    weight=1.0,
                ),
                trajectory=trajectory,
                sample_count=int(len(times)),
                duration_seconds=float(times[-1] - times[0]),
            )
        ]

    validation_chunks: list[TrajectoryChunk] = []
    for index, chunk in enumerate(train_chunks):
        validation_chunks.append(
            TrajectoryChunk(
                spec=TelemetryChunkRunSpec(
                    name=f"Training replay: {chunk.spec.display_name(index)}",
                    role="validation",
                    excitation=chunk.spec.excitation,
                    start=float(chunk.spec.start),
                    end=float(chunk.spec.end),
                    weight=float(chunk.spec.weight),
                ),
                trajectory=chunk.trajectory,
                sample_count=int(chunk.sample_count),
                duration_seconds=float(chunk.duration_seconds),
            )
        )
    return validation_chunks


async def _evaluate_validation_pair(
    optimizer: object,
    config: OptimizerConfig,
    validation_chunks: list[TrajectoryChunk],
    authored_theta: object,
    tuned_theta: object,
    *,
    validation_source: str,
    on_progress: ProgressCallback | None,
) -> tuple[
    list[ChunkValidationMetric],
    list[SysIdRolloutPlotData],
    list[ChunkValidationMetric],
    list[SysIdRolloutPlotData],
]:
    """Evaluate authored and tuned parameters on one validation sequence.

    Args:
        optimizer: Optimizer that owns the rollout bridge.
        config: Optimizer configuration to evaluate.
        validation_chunks: Trajectory chunks replayed for validation.
        authored_theta: Parameters authored when the run started.
        tuned_theta: Accepted output parameters.
        validation_source: Source classification attached to validation artifacts.
        on_progress: Optional progress callback.

    Returns:
        Untuned metrics and rollouts followed by tuned metrics and rollouts.
    """
    paired_fn = getattr(optimizer, "evaluate_validation_parameter_sets_with_rollouts", None)
    if callable(paired_fn):
        data_label = "holdout" if validation_source == "configured_chunks" else "training-data replay"
        if on_progress is not None:
            on_progress(f"Validating untuned and tuned parameters on {data_label}...", 0.0)

        def paired_progress(message: str, fraction: float) -> None:
            if on_progress is not None:
                on_progress(f"Paired validation: {message}", fraction)

        grouped = await paired_fn(
            config,
            validation_chunks,
            [authored_theta, tuned_theta],
            on_progress=paired_progress if on_progress is not None else None,
        )
        if len(grouped) != 2:
            raise RuntimeError(f"Paired validation returned {len(grouped)} parameter groups; expected 2.")
        untuned_replays, tuned_replays = grouped
        untuned_metrics = [replay.metric for replay in untuned_replays]
        untuned_rollouts = [replay.rollout for replay in untuned_replays if replay.rollout is not None]
        tuned_metrics = [replay.metric for replay in tuned_replays]
        tuned_rollouts = [replay.rollout for replay in tuned_replays if replay.rollout is not None]
        _annotate_validation_metrics(
            untuned_metrics,
            pass_name="untuned",
            validation_source=validation_source,
        )
        _annotate_validation_metrics(
            tuned_metrics,
            pass_name="tuned",
            validation_source=validation_source,
        )
        return untuned_metrics, untuned_rollouts, tuned_metrics, tuned_rollouts

    untuned_metrics, untuned_rollouts = await _evaluate_validation_pass(
        optimizer,
        config,
        validation_chunks,
        authored_theta,
        pass_name="untuned",
        validation_source=validation_source,
        on_progress=_phase_progress(on_progress, 0.0, 0.5),
    )
    tuned_metrics, tuned_rollouts = await _evaluate_validation_pass(
        optimizer,
        config,
        validation_chunks,
        tuned_theta,
        pass_name="tuned",
        validation_source=validation_source,
        on_progress=_phase_progress(on_progress, 0.5, 1.0),
    )
    return untuned_metrics, untuned_rollouts, tuned_metrics, tuned_rollouts


async def _evaluate_validation_pass(
    optimizer: object,
    config: OptimizerConfig,
    validation_chunks: list[TrajectoryChunk],
    theta: object,
    *,
    pass_name: str,
    validation_source: str,
    on_progress: ProgressCallback | None,
) -> tuple[list[ChunkValidationMetric], list[SysIdRolloutPlotData]]:
    """Evaluate one named validation pass with stable source annotations.

    Args:
        optimizer: Optimizer that owns the rollout bridge.
        config: Optimizer configuration to evaluate.
        validation_chunks: Trajectory chunks replayed for validation.
        theta: Parameter vector applied during the pass.
        pass_name: Name used to identify the parameter state.
        validation_source: Source classification attached to validation artifacts.
        on_progress: Optional progress callback.

    Returns:
        Validation metrics and rollout plot data produced by the pass.
    """
    if on_progress is not None:
        data_label = "holdout" if validation_source == "configured_chunks" else "training-data replay"
        on_progress(f"Validating {pass_name} parameters on {data_label}...", 0.0)

    def pass_progress(message: str, fraction: float) -> None:
        if on_progress is not None:
            on_progress(f"{pass_name.title()} validation: {message}", fraction)

    metrics: list[ChunkValidationMetric] = []
    rollouts: list[SysIdRolloutPlotData] = []
    replay_fn = getattr(optimizer, "evaluate_validation_chunks_with_rollouts", None)
    if callable(replay_fn):
        replays = await replay_fn(
            config,
            validation_chunks,
            theta,
            on_progress=pass_progress if on_progress is not None else None,
        )
        metrics = [replay.metric for replay in replays]
        rollouts = [replay.rollout for replay in replays if replay.rollout is not None]
    else:
        validate_fn = getattr(optimizer, "evaluate_validation_chunks", None)
        if callable(validate_fn):
            metrics = await validate_fn(
                config,
                validation_chunks,
                theta,
                on_progress=pass_progress if on_progress is not None else None,
            )
    _annotate_validation_metrics(metrics, pass_name=pass_name, validation_source=validation_source)
    return metrics, rollouts


def _annotate_validation_metrics(
    metrics: list[ChunkValidationMetric],
    *,
    pass_name: str,
    validation_source: str,
) -> None:
    """Attach parameter-state and data-source metadata to validation metrics.

    Args:
        metrics: Validation metrics to annotate.
        pass_name: Name of the evaluated parameter state.
        validation_source: Source classification for the replayed chunks.
    """
    for metric in metrics:
        metric.extra["parameter_state"] = pass_name
        metric.extra["source"] = validation_source
        metric.extra["holdout"] = validation_source == "configured_chunks"


def _phase_progress(
    callback: ProgressCallback | None,
    start: float,
    end: float,
) -> ProgressCallback | None:
    """Map progress into a bounded parent phase.

    Args:
        callback: Optional progress callback.
        start: Parent phase start fraction.
        end: Parent phase end fraction.

    Returns:
        Mapped callback, or None when no callback was supplied.
    """
    if callback is None:
        return None

    def report(message: str, fraction: float) -> None:
        local = max(0.0, min(1.0, float(fraction)))
        callback(message, start + (end - start) * local)

    return report


def _validation_regression_channels(comparison: dict) -> list[str]:
    """Return held-out channels whose RMSE regressed against the authored baseline.

    Args:
        comparison: Paired untuned-versus-tuned validation comparison.

    Returns:
        Sorted regressed channel names, empty when the comparison is unavailable
        or reports no regression.
    """
    if not comparison.get("available") or comparison.get("holdout") is False:
        return []
    overall = comparison.get("overall") or {}
    return sorted(
        name for name, payload in overall.items() if isinstance(payload, dict) and payload.get("status") == "regressed"
    )


def _writeback_prim_paths(
    prepared: SysIdPreparedRun,
    bridge: object | None,
    *,
    robot_path: str,
    include_provenance: bool,
) -> list[str]:
    """Return prim paths that can receive parameter or provenance opinions.

    Args:
        prepared: Prepared run containing USD baseline paths.
        bridge: Runtime bridge that may own explicit actuator prims.
        robot_path: Robot prim used for provenance.
        include_provenance: Include the robot prim that receives provenance attributes.

    Returns:
        Minimal set of prim paths needed for transactional rollback.
    """
    link_paths = list(getattr(prepared, "link_paths", []) or [])
    joint_paths = list(getattr(prepared, "joint_paths", []) or [])
    paths = [str(path) for path in (*link_paths, *joint_paths) if str(path)]
    baseline = getattr(getattr(prepared, "parameter_space", None), "baseline", None)
    for collection_name in ("usd_link_snapshots", "usd_joint_snapshots"):
        for snapshot in getattr(baseline, collection_name, []) or []:
            path = str(getattr(snapshot, "path", "") or "")
            if path:
                paths.append(path)
    metadata = getattr(bridge, "actuator_metadata", {}) or {}
    if isinstance(metadata, dict):
        paths.extend(str(path) for path in metadata.get("temporary_promotions", []) if str(path))
        paths.extend(
            str(item.get("prim_path"))
            for item in metadata.get("actuators", [])
            if isinstance(item, dict) and item.get("prim_path")
        )
    if include_provenance and robot_path:
        paths.append(str(robot_path))
    return _minimal_prim_paths(paths)


def _minimal_prim_paths(paths: list[str]) -> list[str]:
    """Remove duplicate and descendant prim paths from a rollback set.

    Args:
        paths: Candidate absolute prim paths.

    Returns:
        Minimal set of unique ancestor paths.
    """
    ordered = sorted({path.rstrip("/") for path in paths if path.startswith("/") and path != "/"}, key=len)
    result: list[str] = []
    for path in ordered:
        if any(path == parent or path.startswith(parent + "/") for parent in result):
            continue
        result.append(path)
    return result


def _current_edit_target(stage: object) -> object | None:
    """Return a stage edit target when the stage exposes one.

    Args:
        stage: USD stage or compatible test double.

    Returns:
        Current edit target, or None when unavailable.
    """
    getter = getattr(stage, "GetEditTarget", None)
    return getter() if callable(getter) else None


def _root_layer(stage: object) -> object | None:
    """Return the stage root layer when the stage exposes one.

    Args:
        stage: USD stage or compatible test double.

    Returns:
        Root layer, or None when unavailable.
    """
    getter = getattr(stage, "GetRootLayer", None)
    return getter() if callable(getter) else None


def _snapshot_stage_layers(
    stage: object,
    prim_paths: list[str],
    *extra_layers: object,
) -> list[_LayerSnapshot]:
    """Capture affected USD prim specs so a failed write can roll back.

    Real USD layers use prim-scoped anonymous backups. Lightweight test doubles
    fall back to serialized content, but only for the explicitly supplied write
    targets rather than every layer in the stage.

    Args:
        stage: USD stage that owns the write-target layers.
        prim_paths: Prim paths whose authored specs may change.
        *extra_layers: Additional layers that may receive parameter opinions.

    Returns:
        Layer snapshots suitable for :func:`_restore_stage_layers`.
    """

    def as_layer(candidate: object) -> object:
        get_layer = getattr(candidate, "GetLayer", None)
        return get_layer() if callable(get_layer) else candidate

    # A Usd.EditTarget carries a composition mapping in addition to its layer.
    # Snapshot the mapped spec paths, not the composed stage paths, otherwise a
    # payload/reference write can escape rollback even though its layer was saved.
    #
    # The session layer is not skipped: it only reaches this function when it is
    # an actual write target, and a write target that is not snapshotted escapes
    # rollback entirely.
    layer_entries: dict[int, tuple[object, set[str], bool]] = {}
    for candidate in extra_layers:
        layer = as_layer(candidate)
        if layer is None:
            continue
        mapped_paths = set(prim_paths)
        force_serialized = False
        map_to_spec_path = getattr(candidate, "MapToSpecPath", None)
        if callable(map_to_spec_path):
            try:
                from pxr import Sdf

                mapped_paths = {
                    str(mapped)
                    for path_text in prim_paths
                    if (mapped := map_to_spec_path(Sdf.Path(path_text))) is not None and str(mapped)
                }
            except Exception:
                # A full-layer snapshot is slower but preserves rollback if a
                # custom edit target cannot expose its mapping reliably.
                force_serialized = True
        if prim_paths and not mapped_paths:
            # Mapping produced no spec paths, so a prim-scoped snapshot would
            # cover nothing and rollback would be a silent no-op.
            force_serialized = True
        key = id(layer)
        if key in layer_entries:
            current_layer, current_paths, current_serialized = layer_entries[key]
            current_paths.update(mapped_paths)
            layer_entries[key] = (current_layer, current_paths, current_serialized or force_serialized)
        else:
            layer_entries[key] = (layer, mapped_paths, force_serialized)

    snapshots: list[_LayerSnapshot] = []
    for layer, layer_prim_paths, force_serialized in layer_entries.values():
        try:
            from pxr import Sdf

            if force_serialized:
                raise TypeError("edit-target mapping requires a serialized fallback")
            if not callable(getattr(layer, "GetPrimAtPath", None)):
                raise TypeError("layer does not expose prim-spec lookup")
            backup = Sdf.Layer.CreateAnonymous("sysid_rollback.usda")
            existing: list[str] = []
            snapshot_paths = tuple(sorted(layer_prim_paths))
            for path_text in snapshot_paths:
                path = Sdf.Path(path_text)
                if layer.GetPrimAtPath(path) is None:
                    continue
                parent = path.GetParentPath()
                if str(parent) not in ("", "/"):
                    Sdf.CreatePrimInLayer(backup, parent)
                Sdf.CopySpec(layer, path, backup, path)
                existing.append(path_text)
            snapshots.append(
                _LayerSnapshot(
                    layer=layer,
                    prim_paths=snapshot_paths,
                    existing_paths=tuple(existing),
                    backup_layer=backup,
                )
            )
        except Exception as exc:
            export = getattr(layer, "ExportToString", None)
            if not callable(export):
                # Dropping the layer here would leave the caller believing the
                # write is transactional while nothing could be restored.
                raise RuntimeError(
                    "Cannot snapshot write-target layer "
                    f"'{getattr(layer, 'identifier', '<unknown>')}' for rollback: {exc}"
                ) from exc
            snapshots.append(_LayerSnapshot(layer=layer, serialized_content=str(export())))
    return snapshots


def _restore_stage_layers(snapshots: list[_LayerSnapshot]) -> None:
    """Restore a stage snapshot after failed parameter or actuator writeback.

    Args:
        snapshots: Prim-scoped or serialized layer rollback state.
    """
    failures: list[str] = []
    for snapshot in reversed(snapshots):
        layer = snapshot.layer
        try:
            if snapshot.serialized_content is not None:
                restore = getattr(layer, "ImportFromString", None)
                if callable(restore) and restore(snapshot.serialized_content) is False:
                    failures.append(str(getattr(layer, "identifier", "<unknown>")))
                continue
            from pxr import Sdf

            existing = set(snapshot.existing_paths)
            for path_text in sorted(snapshot.prim_paths, key=len, reverse=True):
                path = Sdf.Path(path_text)
                if layer.GetPrimAtPath(path) is not None:
                    edit = Sdf.BatchNamespaceEdit()
                    edit.Add(Sdf.NamespaceEdit.Remove(path))
                    if layer.Apply(edit) is False:
                        raise RuntimeError(f"could not remove current prim spec at {path_text}")
                if path_text in existing and snapshot.backup_layer is not None:
                    Sdf.CopySpec(snapshot.backup_layer, path, layer, path)
        except Exception as exc:
            failures.append(f"{getattr(layer, 'identifier', '<unknown>')}: {exc}")
    if failures:
        raise RuntimeError("Failed to restore USD layer snapshot(s): " + "; ".join(failures))


def _try_restore_live_parameters(
    bridge: object | None,
    authored_theta: object,
    param_entries: list,
    *,
    required: bool,
) -> str:
    """Try to restore the live articulation and return an error message on failure.

    Args:
        bridge: Runtime bridge that may support live rollback.
        authored_theta: Authored launch values to restore.
        param_entries: Parameter definitions corresponding to the values.
        required: Whether live restoration is required for this rollback.

    Returns:
        Empty text on success, otherwise a restoration error message.
    """
    if not required:
        return ""
    restore = getattr(bridge, "restore_parameters_on_live_articulation", None)
    if not callable(restore):
        return "the bridge does not support live parameter rollback"
    if authored_theta is None:
        return "authored launch parameters are unavailable"
    try:
        if restore(authored_theta, param_entries) is not True:
            return "the bridge did not confirm live parameter restoration"
    except Exception as exc:
        return str(exc)
    return ""


def _write_provenance(
    stage,  # noqa: ANN001
    prepared: SysIdPreparedRun,
    config: OptimizerConfig,
    final: OptimizationIterationStatus,
    *,
    backend_label: str,
    robot_path: str,
    validation_metrics: list[ChunkValidationMetric],
    validation_comparison: dict,
    write_usd: bool,
    export_sidecar: bool,
    sidecar_path: str,
) -> str:
    """Author provenance to USD and/or a JSON sidecar. Returns the sidecar path (or '').

    Args:
        stage: USD stage used by the operation.
        prepared: Prepared SysID run state.
        config: Configuration for the operation.
        final: Selected optimization status to record.
        backend_label: Optimizer backend label.
        robot_path: Robot prim receiving stage metadata.
        validation_metrics: Per-chunk validation measurements.
        validation_comparison: Untuned-versus-tuned validation delta payload.
        write_usd: Whether to author provenance on the USD stage.
        export_sidecar: Whether to export a JSON sidecar.
        sidecar_path: Destination for the optional JSON sidecar.

    Returns:
        Written sidecar path, or an empty string when no sidecar was requested.
    """  # noqa: DOC107
    record = build_provenance_record(
        config=config,
        final_status=final,
        optimizer_backend=backend_label,
        robot_prim_path=robot_path,
        validation_metrics=validation_metrics,
        simulation_engine=str(getattr(prepared, "simulation_engine", "")),
        newton_config=getattr(prepared, "newton_config", None),
        validation_comparison=validation_comparison,
    )
    writer = ProvenanceWriter()
    if write_usd:
        writer.write_to_stage(stage, robot_path, record)
    if export_sidecar and sidecar_path:
        return writer.export_json_sidecar(sidecar_path, record)
    return ""
