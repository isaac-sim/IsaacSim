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

"""Main UI builder for the System Identification extension."""

from __future__ import annotations

import asyncio
import os
import traceback
from pathlib import Path
from types import SimpleNamespace
from typing import Any, Optional

import carb
import omni.kit.app
import omni.timeline
import omni.ui as ui
import omni.usd
from isaacsim.gui.components.element_wrappers import StateButton
from isaacsim.gui.components.ui_utils import get_style
from isaacsim.robot_setup.sysid.actuator_compatibility import resolve_simulation_compatibility

from ..articulation_utils import collect_robot_link_and_joint_paths
from ..execution import run_sysid
from ..optimizer_base import OptimizationIterationStatus
from ..optimizer_config import (
    _BACKEND_TO_LABEL,
    OptimizerBackend,
    resolve_backend,
)
from ..optimizer_factory import create_optimizer
from ..parameter_space import ParameterSpace
from ..preflight import preflight_sysid_run_spec
from ..preset_loader import load_builtin_parameter_preset
from ..recipes import recipe_parameter_type_values
from ..run_controller import SysIdPreparedRun, SysIdRunController
from ..run_report import export_sysid_run_report
from ..run_session import SysIdRunResult
from ..run_spec import (
    NEWTON_SOLVER_MUJOCO,
    SIMULATION_ENGINE_ISAAC_SIM,
    OutputsRunSpec,
    ParameterSpaceRunSpec,
    ParametersRunSpec,
    ResidualsRunSpec,
    SimulationRunSpec,
    SolverRunSpec,
    StageRunSpec,
    SysIdRunSpec,
    TelemetryRunSpec,
    TimeWindowRunSpec,
)
from ..runtime import ensure_required_runtime_extensions_async
from ..schema_validation import validate_sysid_run_spec_payload
from ..settings import CarbOptimizerSettingsStore
from ..telemetry_quality import build_telemetry_quality_report
from ..trajectory_csv import TrajectoryCsvError, TrajectoryDataset
from ..trajectory_segments import (
    TELEMETRY_CHUNK_ROLE_TRAIN,
    ChunkValidationMetric,
    build_trajectory_chunks,
    split_train_validation_chunks,
    summarize_validation_metrics,
)
from .check_panel import CheckPanel
from .parameter_panel import ParameterPanel
from .pipeline_state import SysIdPipelineState
from .recipe_bar import RecipeBar
from .results_panel import ResultsPanel
from .simulation_panel import SimulationPanel
from .solver_panel import SolverPanel
from .stage_status import (
    CHECK_STATE_NOT_RUN,
    CHECK_STATE_OK,
    CHECK_STATE_STALE,
    CHECK_STATE_WARNINGS,
    format_stage_titles,
)
from .telemetry_panel import TelemetryPanel
from .telemetry_preview import clear_gt_preview, preview_gt_link_positions
from .workflow_shell import WorkflowShell


class UIBuilder:
    """Coordinates all SysID panels and drives the optimization run."""

    def __init__(self) -> None:
        # Panels
        self._telemetry = TelemetryPanel(
            on_trajectory_changed=self._on_trajectory_changed,
            on_run_state_changed=self._update_run_button_state,
            on_preview_requested=self._on_preview_gt_requested,
            on_preview_cleared=self._on_preview_gt_cleared,
        )
        self._simulation = SimulationPanel(
            on_robot_path_changed=self._on_robot_path_changed,
            on_engine_changed=self._on_engine_changed,
            on_newton_settings_changed=self._on_newton_settings_changed,
        )
        self._parameters = ParameterPanel(
            on_flags_changed=self._refresh_parameter_space,
            on_selection_changed=self._on_parameter_selection_changed,
        )
        self._solver = SolverPanel(
            on_backend_changed=self._on_solver_backend_changed,
            on_export_run_spec=self._on_export_run_spec_clicked,
            on_check_inputs_changed=self._on_solver_check_inputs_changed,
        )
        self._results = ResultsPanel(
            on_export_run_report=self._on_export_run_report_clicked,
        )
        self._recipe_bar = RecipeBar(on_apply_recipe=self._on_apply_recipe)
        self._check = CheckPanel(
            on_run_check=self._on_run_check_clicked,
            on_apply_suggestion=self._on_apply_check_suggestion,
        )
        self._pipeline = SysIdPipelineState()

        # Run state
        self._run_task: Optional[asyncio.Task] = None
        self._running = False
        self._run_generation = 0
        self._run_button: Optional[StateButton] = None
        self._workflow_shell: Optional[WorkflowShell] = None
        self._page_frames: dict[str, ui.Frame] = {}
        self._results_refresh_task: Optional[asyncio.Task] = None
        self._paused_viewport_state: Optional[tuple[object, bool]] = None

        # Cached run artifacts (for report export)
        self._last_config = None
        self._last_final_status: Optional[OptimizationIterationStatus] = None
        self._last_validation_metrics: list[ChunkValidationMetric] = []
        self._last_prepared_run: Optional[SysIdPreparedRun] = None
        self._last_run_result: Optional[SysIdRunResult] = None

        # Active parameter space (kept in sync with robot path + flags)
        self._active_parameter_space: Optional[ParameterSpace] = None
        self._active_link_paths: list[str] = []
        self._active_joint_paths: list[str] = []
        self._active_physics_backend: str = ""

        # Optimizer (kept alive across runs for warm-start)
        backend_cfg = self._solver.get_backend_config()
        self._optimizer = create_optimizer(resolve_backend(backend_cfg, num_params=1, residual_cfg=None))

        self.wrapped_ui_elements: list = []

    # ------------------------------------------------------------------- UI

    def build_ui(self) -> None:
        """Build ui."""
        self._workflow_shell = WorkflowShell(on_stage_changed=self._on_stage_changed)
        with ui.VStack(style=get_style(), spacing=8):
            with ui.Frame(height=54):
                self._workflow_shell.build_header()
            self._recipe_bar.build()
            self._workflow_shell.build_navigation()
            self._workflow_shell.build_status()

            with ui.ScrollingFrame(
                style=get_style(),
                vertical_scrollbar_policy=ui.ScrollBarPolicy.SCROLLBAR_AS_NEEDED,
                horizontal_scrollbar_policy=ui.ScrollBarPolicy.SCROLLBAR_ALWAYS_OFF,
            ):
                with ui.VStack(style=get_style(), spacing=5, height=0):
                    self._page_frames = {
                        "sim": ui.Frame(visible=True),
                        "data": ui.Frame(visible=False),
                        "params": ui.Frame(visible=False),
                        "check": ui.Frame(visible=False),
                        "solve": ui.Frame(visible=False),
                        "results": ui.Frame(visible=False),
                    }
                    with self._page_frames["sim"]:
                        self._simulation.build()
                    with self._page_frames["data"]:
                        self._telemetry.build()
                    with self._page_frames["params"]:
                        self._parameters.build()
                    with self._page_frames["check"]:
                        self._check.build()
                    with self._page_frames["solve"]:
                        self._solver.build()
                    with self._page_frames["results"]:
                        self._results.build()

            self._workflow_shell.bind_pages(self._page_frames)
            self._results.set_open_callback(lambda: self._workflow_shell.activate("results"))
            action_frame = self._workflow_shell.build_footer()
            with action_frame:
                self._run_button = StateButton(
                    "",
                    "Run System ID Optimization",
                    "Cancel",
                    tooltip="Start or cancel asynchronous optimization",
                    on_a_click_fn=self._on_run_clicked,
                    on_b_click_fn=self._on_cancel_clicked,
                )
            self.wrapped_ui_elements.append(self._run_button)

        self._refresh_parameter_space()
        self._update_run_button_state()

    # ---------------------------------------------------------------- events

    def _on_stage_changed(self, stage: str) -> None:
        if stage == "solve":
            self._update_run_button_state()

    def _on_trajectory_changed(self, trajectory: Optional[TrajectoryDataset]) -> None:
        self._refresh_parameter_space()

    def _on_robot_path_changed(self, _path: str) -> None:
        self._refresh_parameter_space()
        self._update_run_button_state()

    def _on_engine_changed(self, _label: str) -> None:
        self._recipe_bar.mark_dirty()
        self._refresh_engine_backend_hint()
        self._refresh_parameter_space_for_backend()
        self._update_run_button_state()

    def _on_newton_settings_changed(self) -> None:
        self._recipe_bar.mark_dirty()
        self._refresh_parameter_space_for_backend()
        self._update_run_button_state()

    def _resolved_physics_backend(self) -> str:
        return resolve_simulation_compatibility(
            SimpleNamespace(
                engine=self._simulation.get_engine(),
                physics_backend=self._simulation.get_physics_backend(),
            )
        ).physics_backend

    def _refresh_parameter_space_for_backend(self) -> None:
        if self._resolved_physics_backend() != self._active_physics_backend:
            self._refresh_parameter_space()

    def _on_parameter_selection_changed(self) -> None:
        self._update_run_button_state()

    def _current_check_signature(self) -> tuple:
        trajectory = self._telemetry.get_trajectory()
        try:
            chunk_specs = self._telemetry.get_chunk_specs() if trajectory is not None else []
        except Exception:  # noqa: BLE001
            # This runs on every field edit via _update_run_button_state, so an invalid
            # input (e.g. an out-of-range automatic-holdout training share) must not raise into
            # the callback. Fail closed with no chunks; the error surfaces when the user
            # actually runs Check or Solve.
            chunk_specs = []
        selected = self._parameters.get_selected_parameter_run_specs()
        newton_spec = self._simulation.get_newton_spec()
        backend_config = self._solver.get_backend_config()
        try:
            residual_config = self._solver.get_residual_config()
            sample_weights = (
                tuple(float(value) for value in residual_config.sample_weights)
                if residual_config.sample_weights is not None
                else ()
            )
            residual_settings = (
                residual_config.position_weight,
                residual_config.velocity_weight,
                residual_config.torque_weight,
                residual_config.end_effector_pose_weight,
                residual_config.contact_force_weight,
                residual_config.end_effector_link_index,
                sample_weights,
            )
        except Exception as exc:
            # Invalid residual files still need a stable signature so the UI can
            # fail closed without breaking routine button-state refreshes.
            residual_settings = ("invalid", type(exc).__name__, str(exc))
        return SysIdPipelineState.compute_signature(
            source_signature=self._telemetry.get_loaded_signature(),
            chunk_specs=chunk_specs,
            selected_entries=[item.to_entry() for item in selected],
            robot_path=self._simulation.get_robot_path(),
            engine=self._simulation.get_engine(),
            simulation_settings=(
                self._simulation.get_physics_backend(),
                newton_spec.solver,
                newton_spec.feedforward,
                newton_spec.controller,
                newton_spec.effort_clamp,
                self._simulation.get_actuator_runtime(),
                self._simulation.get_explicit_actuator_policy(),
            ),
            residual_settings=residual_settings,
            solver_settings=(
                backend_config.backend.value,
                backend_config.use_analytical_presolve,
                self._solver.get_max_rollout_steps(),
            ),
        )

    def _on_run_check_clicked(self) -> None:
        try:
            from ..check_report import build_sysid_check_report

            trajectory = self._telemetry.get_trajectory()
            if trajectory is None:
                raise ValueError("Load telemetry data first.")
            spec = self._build_run_spec()
            stage = omni.usd.get_context().get_stage()
            chunks = build_trajectory_chunks(
                trajectory, self._telemetry.get_chunk_specs(), require_train=True, reject_overlaps=True
            )
            train_chunks, validation_chunks = split_train_validation_chunks(chunks)
            report = build_sysid_check_report(
                spec,
                stage=stage,
                trajectory=trajectory,
                train_chunks=train_chunks,
                validation_chunks=validation_chunks,
            )
            self._pipeline.record_check(report, self._current_check_signature())
            self._check.set_report(report)
            self._check.set_stale(False)
            self._push_feedforward_hint()
            not_ok = sum(1 for v in report.parameter_verdicts if v.verdict == "not_identifiable")
            summary = "Check complete."
            if not_ok:
                summary = f"Check complete: {not_ok} parameter(s) look unidentifiable with this data."
            suggestion_count = len(self._check.get_suggestions())
            if suggestion_count:
                summary += f" {suggestion_count} suggested setting(s) below can be applied with one click."
            self._check.set_status(summary)
            for line in report.summary_lines():
                carb.log_info(f"SysId check: {line}")
        except Exception as exc:
            carb.log_warn(f"SysId check failed: {exc}")
            self._pipeline.clear_check()
            self._check.set_report(None)
            self._check.set_stale(False)
            self._check.set_status(f"Check failed: {exc}")
            self._push_feedforward_hint()
        self._update_run_button_state()

    def _on_apply_check_suggestion(self, suggestion: Any) -> None:
        from .check_suggestions import SUGGESTION_KIND_COMMAND_ALIGNMENT, SUGGESTION_KIND_FEEDFORWARD

        if suggestion.kind == SUGGESTION_KIND_FEEDFORWARD:
            self._simulation.set_feedforward_mode(str(suggestion.value))
            self._check.set_status(f"Controller feedforward set to '{suggestion.value}'. Re-run Check.")
        elif suggestion.kind == SUGGESTION_KIND_COMMAND_ALIGNMENT:
            # The suggestion is a delta measured on the already-shifted loaded
            # trajectory; add it to the current field value and reload.
            new_delay = self._telemetry.get_command_alignment_seconds() + float(suggestion.value)
            self._telemetry.set_command_alignment_seconds(new_delay)
            self._check.set_status(
                f"Command delay set to {new_delay:.4f} s; reloading telemetry. Re-run Check after the load."
            )
        else:
            carb.log_warn(f"SysId: unknown check suggestion kind '{suggestion.kind}'.")
            return
        # Applying a suggestion invalidates the Check that produced it: the telemetry
        # has been reloaded (alignment) or the controller model changed (feedforward).
        # Drop the now-obsolete report so the already-applied suggestion does not stay
        # displayed and clickable; the user is prompted to re-run Check.
        self._check.set_report(None)
        self._pipeline.clear_check()
        self._recipe_bar.mark_dirty()
        # The changed setting flips the check signature, marking the check stale.
        self._update_run_button_state()

    def _push_feedforward_hint(self) -> None:
        """Mirror the Check stage's feedforward recommendation on the Robot panel."""
        from .check_suggestions import SUGGESTION_KIND_FEEDFORWARD

        hint = None
        for suggestion in self._check.get_suggestions():
            if suggestion.kind == SUGGESTION_KIND_FEEDFORWARD:
                hint = f"Check recommends '{suggestion.value}' — see step 4."
                break
        self._simulation.set_feedforward_hint(hint)

    def _on_solver_backend_changed(self, _label: str) -> None:
        self._recipe_bar.mark_dirty()
        self._refresh_engine_backend_hint()
        self._update_run_button_state()

    def _on_solver_check_inputs_changed(self) -> None:
        self._recipe_bar.mark_dirty()
        self._update_run_button_state()

    def _refresh_engine_backend_hint(self) -> None:
        """Recommend (never force) an optimizer matching the selected engine."""
        self._solver.set_differentiable_bridge_hint(self._simulation.is_differentiable_engine())
        backend = self._solver.get_backend_config().backend
        hint = None
        if self._simulation.is_differentiable_engine():
            if backend not in (OptimizerBackend.AUTO, OptimizerBackend.GRADIENT_DESCENT):
                hint = (
                    "Differentiable Newton bridge selected - Gradient Descent (Adam) is recommended; "
                    "the current backend will run but ignores the exact gradients."
                )
        elif (
            self._simulation.get_engine() != SIMULATION_ENGINE_ISAAC_SIM
            and backend == OptimizerBackend.LEVENBERG_MARQUARDT
        ):
            hint = (
                "Newton (MuJoCo) rollouts with finite-difference Levenberg-Marquardt need one rollout "
                "per parameter - CMA-ES is usually faster here."
            )
        elif backend == OptimizerBackend.GRADIENT_DESCENT and not self._simulation.is_differentiable_engine():
            hint = (
                "Gradient Descent requires the differentiable Newton engine; select "
                "'Newton differentiable (Featherstone)' or another backend."
            )
        self._solver.set_backend_hint(hint)

    def _on_apply_recipe(self, name: str) -> None:
        preset = load_builtin_parameter_preset(name)
        flags = {
            "include_com_offsets": preset.include_com_offsets,
            "include_inertia_log_cholesky": preset.include_inertia_log_cholesky,
            "include_joint_limit_scales": preset.include_joint_limit_scales,
            "include_per_link_mass": preset.include_per_link_mass,
            "include_command_delay": preset.include_command_delay,
        }
        self._parameters.set_advanced_flags(flags)
        self._refresh_parameter_space()
        self._apply_recipe_simulation_section(preset)
        self._apply_recipe_solver_section(preset)
        self._apply_recipe_chunking_section(preset)
        # Field values first, dropdown last: the backend dropdown callback persists
        # the config from the current widget values.
        self._solver.set_residual_config(preset.build_residual_weight_config())
        self._solver.set_backend_config(preset.build_optimizer_backend_config())
        self._parameters.select_parameters_for_recipe(recipe_parameter_type_values(preset))
        self._refresh_engine_backend_hint()
        self._update_run_button_state()
        detail = f" {preset.description}" if preset.description else ""
        self._recipe_bar.set_status(f"Applied '{name}'.{detail} All fields stay editable.")
        carb.log_info(f"SysId: applied recipe '{name}'.")

    def _apply_recipe_simulation_section(self, preset: Any) -> None:
        """Mirror recipes._apply_simulation_section onto the Robot panel widgets.

        Args:
            preset: Preset value.
        """
        section = dict(preset.simulation or {})
        newton_section = dict(section.get("newton") or {})
        engine = section.get("engine")
        if engine:
            self._simulation.set_engine_selection(
                str(engine), str(newton_section.get("solver") or NEWTON_SOLVER_MUJOCO)
            )
        if "actuator_runtime" in section:
            self._simulation.set_actuator_runtime(str(section["actuator_runtime"]))
        if "physics_backend" in section:
            self._simulation.set_physics_backend(str(section["physics_backend"]))
        if "explicit_actuator_policy" in section:
            self._simulation.set_explicit_actuator_policy(str(section["explicit_actuator_policy"]))
        feedforward = newton_section.get("feedforward")
        if feedforward is not None:
            self._simulation.set_feedforward_mode(str(feedforward).strip().lower() or "none")
        controller = newton_section.get("controller")
        if controller is not None:
            self._simulation.set_controller_mode(str(controller).strip().lower() or "pd")
        effort_clamp = newton_section.get("effort_clamp")
        if effort_clamp is not None:
            self._simulation.set_effort_clamp_mode(str(effort_clamp).strip().lower() or "max_effort")
        if "offline_stepping" in section:
            self._simulation.set_offline_stepping(bool(section["offline_stepping"]))
        if "cuda_graph_capture" in newton_section:
            self._simulation.set_cuda_graph_capture(bool(newton_section["cuda_graph_capture"]))

    def _apply_recipe_solver_section(self, preset: Any) -> None:
        """Mirror recipes._apply_solver_section onto the Solve panel widgets.

        Args:
            preset: Preset value.
        """
        section = dict(preset.solver or {})
        if not section:
            return
        if any(key in section for key in ("damping_initial", "epsilon", "max_iterations")):
            damping, epsilon, max_iter = self._solver.get_solver_floats()
            self._solver.set_solver_floats(
                float(section.get("damping_initial", damping)),
                float(section.get("epsilon", epsilon)),
                int(section.get("max_iterations", max_iter)),
            )
        if "max_rollout_steps" in section:
            self._solver.set_max_rollout_steps(int(section["max_rollout_steps"]))

    def _apply_recipe_chunking_section(self, preset: Any) -> None:
        """Mirror recipes._apply_chunking_section onto the Data panel widgets.

        Args:
            preset: Preset value.
        """
        section = dict(preset.chunking or {})
        if str(section.get("mode", "")).lower() != "auto":
            return
        train_fraction = section.get("train_fraction")
        min_chunk_seconds = section.get("min_chunk_seconds")
        self._telemetry.set_auto_split(
            True,
            float(train_fraction) if train_fraction is not None else None,
            float(min_chunk_seconds) if min_chunk_seconds is not None else None,
        )

    def _on_preview_gt_requested(self, trajectory: TrajectoryDataset, sample_time: float) -> str:
        stage = omni.usd.get_context().get_stage()
        result = preview_gt_link_positions(
            stage,
            trajectory,
            sample_time,
            robot_path=self._simulation.get_robot_path(),
            hide_source_prim=True,
        )
        if result.previewed:
            carb.log_info(f"SysId: {result.message}")
        else:
            carb.log_warn(f"SysId: {result.message}")
        return result.message

    def _on_preview_gt_cleared(self) -> None:
        clear_gt_preview(omni.usd.get_context().get_stage())

    def _refresh_parameter_space(self) -> None:
        trajectory = self._telemetry.get_trajectory()
        if trajectory is None:
            self._active_parameter_space = None
            self._active_link_paths = []
            self._active_joint_paths = []
            self._parameters.set_parameter_space(None)
            return

        num_joints = trajectory.num_joints
        flags = self._parameters.get_advanced_flags()
        stage = omni.usd.get_context().get_stage()
        robot_path = self._simulation.get_robot_path()
        link_paths: list[str] = []
        joint_paths: list[str] = []

        if stage is not None and robot_path:
            try:
                link_paths, joint_paths = collect_robot_link_and_joint_paths(stage, robot_path)
            except Exception as exc:
                carb.log_warn(f"SysId: could not read USD parameter baselines: {exc}")

        physics_backend = self._resolved_physics_backend()
        baselines_read = False
        advanced = any(flags.values())
        if not advanced:
            space = ParameterSpace.for_robot(num_joints)
            if stage is not None and link_paths:
                try:
                    space.read_usd_baselines(
                        stage, link_paths, joint_paths[:num_joints], physics_backend=physics_backend
                    )
                    baselines_read = True
                except Exception as exc:
                    carb.log_warn(f"SysId: could not read USD parameter baselines: {exc}")
        else:
            space = ParameterSpace.for_robot_extended(num_joints, max(1, len(link_paths)), include_basic=True, **flags)
            if stage is not None and link_paths:
                try:
                    space.read_usd_baselines(
                        stage, link_paths, joint_paths[:num_joints], physics_backend=physics_backend
                    )
                    baselines_read = True
                except Exception as exc:
                    carb.log_warn(f"SysId: could not read all USD parameter baselines: {exc}")

        self._active_parameter_space = space
        self._active_link_paths = link_paths
        self._active_joint_paths = joint_paths
        self._parameters.set_parameter_space(space)
        if hasattr(self._optimizer, "set_parameter_space"):
            self._optimizer.set_parameter_space(space)
        # Left unset after a failed read so the next backend callback retries
        # instead of treating the fallback baselines as current.
        self._active_physics_backend = physics_backend if baselines_read else ""

    # ---------------------------------------------------------------- run

    def _is_timeline_playing(self) -> bool:
        return omni.timeline.get_timeline_interface().is_playing()

    def _requires_timeline(self) -> bool:
        """Check whether the selected rollout mode requires a playing timeline.

        Returns:
            The resulting value.
        """
        return (
            self._simulation.get_engine() == SIMULATION_ENGINE_ISAAC_SIM and not self._simulation.is_offline_stepping()
        )

    def _update_run_button_state(self, *, preserve_status: bool = False) -> None:
        if self._run_button is None or self._running:
            return
        trajectory = self._telemetry.get_trajectory()
        signature = self._current_check_signature()
        enabled, hint = self._evaluate_run_gate(signature=signature)
        self._run_button.enabled = enabled
        self._set_export_run_spec_enabled(trajectory is not None)
        self._check.set_stale(self._pipeline.has_check() and self._pipeline.is_check_stale(signature))
        check_ready = (
            trajectory is not None
            and bool(self._parameters.get_selected_parameter_run_specs())
            and bool(self._simulation.get_robot_path())
        )
        self._check.set_run_enabled(check_ready)
        if not preserve_status:
            if enabled:
                self._set_global_status(
                    "Ready to solve. Review Outputs & stage changes, then run optimization.", "success"
                )
            elif hint:
                self._set_global_status(hint, "warning")
        self._refresh_stage_titles(run_ready=enabled, signature=signature)

    def _evaluate_run_gate(self, *, signature: tuple | None = None) -> tuple[bool, str]:
        trajectory = self._telemetry.get_trajectory()
        chunk_error = self._telemetry.chunk_validation_error(require_train=True) if trajectory is not None else ""
        return self._pipeline.run_gate(
            trajectory_loaded=trajectory is not None,
            timeline_ok=self._is_timeline_playing() or not self._requires_timeline(),
            chunk_error=chunk_error,
            current_signature=signature if signature is not None else self._current_check_signature(),
        )

    def _refresh_stage_titles(self, *, run_ready: bool, signature: tuple) -> None:
        if self._workflow_shell is None:
            return
        trajectory = self._telemetry.get_trajectory()
        chunk_summary = ""
        if trajectory is not None:
            try:
                specs = self._telemetry.get_chunk_specs()
                train = sum(1 for spec in specs if spec.role == TELEMETRY_CHUNK_ROLE_TRAIN)
                chunk_summary = f"{train} train / {len(specs) - train} validation chunk(s)"
            except Exception:
                chunk_summary = ""
        if not self._pipeline.has_check():
            check_state = CHECK_STATE_NOT_RUN
        elif self._pipeline.is_check_stale(signature):
            check_state = CHECK_STATE_STALE
        else:
            report = self._pipeline.current_report()
            check_state = CHECK_STATE_OK if getattr(report, "ok", True) else CHECK_STATE_WARNINGS
        titles = format_stage_titles(
            engine_label=self._simulation.get_engine_label(),
            sample_count=int(trajectory.times.shape[0]) if trajectory is not None else None,
            num_joints=trajectory.num_joints if trajectory is not None else None,
            chunk_summary=chunk_summary,
            selected_count=len(self._parameters.get_selected_parameter_run_specs()),
            check_state=check_state,
            run_ready=run_ready,
        )
        self._workflow_shell.set_stage_titles(titles)

    def _on_run_clicked(self) -> None:
        active_task = getattr(self, "_run_task", None)
        if self._running or (active_task is not None and not active_task.done()):
            return
        enabled, hint = self._evaluate_run_gate()
        if not enabled:
            carb.log_warn(f"SysId run blocked by the Check gate: {hint}")
            self._set_global_status(f"Run blocked: {hint}", "error")
            if self._run_button:
                self._run_button.reset()
            self._update_run_button_state(preserve_status=True)
            return

        self._run_task = asyncio.ensure_future(self._start_run_async())

    async def _start_run_async(self) -> None:
        """Activate optional runtime support and prepare an approved run."""
        viewport_paused = False

        try:
            run_spec = self._build_run_spec()
            CarbOptimizerSettingsStore().save(self._solver.get_backend_config())

            enabled_extensions = await ensure_required_runtime_extensions_async(run_spec.simulation)
            if enabled_extensions:
                carb.log_info(f"SysId: enabled required runtime extension(s): {', '.join(enabled_extensions)}")

            stage = omni.usd.get_context().get_stage()
            if stage is None:
                raise ValueError("No USD stage is open.")

            preflight = self._run_preflight(run_spec)
            if not preflight.ok:
                preflight.raise_for_errors()
            for issue in preflight.warnings:
                carb.log_warn(f"SysId preflight: {issue.message}")

            trajectory = self._telemetry.get_trajectory()
            chunk_specs = self._telemetry.get_chunk_specs()
            chunks = build_trajectory_chunks(trajectory, chunk_specs, require_train=True, reject_overlaps=True)
            train_chunks, validation_chunks = split_train_validation_chunks(chunks)
            viewport_paused = self._pause_viewport_rendering_if_enabled()

            controller = SysIdRunController(run_spec)
            prepared = controller.prepare(
                stage,
                trajectory=trajectory,
                train_chunks=train_chunks,
                validation_chunks=validation_chunks,
                timeline_playing=self._is_timeline_playing(),
            )

            try:
                self._optimizer.cleanup_bridge()
            except Exception:
                pass
            self._optimizer = prepared.optimizer
            self._last_prepared_run = prepared
            self._active_parameter_space = prepared.parameter_space
            self._active_link_paths = prepared.link_paths
            self._active_joint_paths = prepared.joint_paths

            m = len(prepared.param_entries)
            use_parallel = bool(run_spec.simulation.parallel_clones)
            env_root = run_spec.simulation.env_paths_root
            source_env = run_spec.simulation.source_env_path
            resolved_backend = prepared.resolved_backend
            config = prepared.config

            if m > 8 and resolved_backend != OptimizerBackend.GRADIENT_DESCENT:
                action = (
                    "Consider selecting fewer DOFs or enabling parallel clones."
                    if resolved_backend == OptimizerBackend.LEVENBERG_MARQUARDT
                    else "Consider selecting fewer parameters or enabling parallel clones."
                )
                carb.log_warn(f"SysId: optimizing {m} parameters can require many rollouts. {action}")

        except (TrajectoryCsvError, ValueError) as exc:
            restored = self._restore_viewport_rendering(update_status=False)
            carb.log_error(str(exc))
            suffix = "; viewport rendering restored" if restored else ""
            self._set_global_status(f"Run blocked: {exc}{suffix}", "error")
            if self._run_button:
                self._run_button.reset()
            return
        except Exception as exc:
            restored = self._restore_viewport_rendering(update_status=False)
            carb.log_error(f"Failed to start SysId optimization: {exc}\n{traceback.format_exc()}")
            suffix = "; viewport rendering restored" if restored else ""
            self._set_global_status(f"Run start failed: {exc}{suffix}", "error")
            if self._run_button:
                self._run_button.reset()
            return

        clone_msg = (
            f"Parallel: {m + 1} envs under {env_root}" if use_parallel else f"Sequential rollouts on {source_env} only"
        )
        backend_label = _BACKEND_TO_LABEL.get(resolved_backend, resolved_backend.value)
        carb.log_info(f"SysId: {clone_msg}")
        carb.log_info(f"SysId: optimizer backend = {backend_label}")

        n_samples = sum(int(c.trajectory.times.shape[0]) for c in train_chunks)
        effective = sum(min(int(c.trajectory.times.shape[0]), config.max_rollout_steps) for c in train_chunks)
        self._set_global_status(
            f"{clone_msg}. Solver uses {effective}/{n_samples} train samples across "
            f"{len(train_chunks)} chunk(s); {len(validation_chunks)} validation chunk(s) held out.",
            "info",
        )
        self._run_generation += 1
        run_gen = self._run_generation
        self._results.set_validation_results("Validation chunks run after optimization.")
        self._results.set_parameter_confidence(None)
        self._results.reset_for_run(prepared.trajectory.num_joints)

        out_flags = self._solver.get_output_flags()
        provenance_path = out_flags["provenance_sidecar_path"] or self._default_provenance_path()

        self._set_ui_running(True)
        self._schedule_results_live_refresh(run_gen)

        self._run_task = asyncio.ensure_future(
            self._run_optimization_async(
                run_spec,
                config,
                prepared,
                run_gen,
                provenance_path,
                viewport_paused,
            )
        )

    def _on_cancel_clicked(self) -> None:
        if self._last_prepared_run is not None:
            self._last_prepared_run.optimizer.request_cancel()
        self._set_global_status("Cancel requested...", "warning")

    def _ui_active(self, run_generation: int) -> bool:
        return run_generation == self._run_generation

    def _schedule_results_live_refresh(self, run_generation: int) -> None:
        if self._results_refresh_task is not None and not self._results_refresh_task.done():
            self._results_refresh_task.cancel()
        self._results_refresh_task = asyncio.ensure_future(self._refresh_results_live_after_layout(run_generation))

    async def _refresh_results_live_after_layout(self, run_generation: int) -> None:
        try:
            app = omni.kit.app.get_app()
            for _ in range(3):
                await app.next_update_async()
                if not self._ui_active(run_generation):
                    return
                self._results.refresh_live_progress_layout()
        except asyncio.CancelledError:
            raise
        except Exception as exc:
            carb.log_warn(f"SysId: failed to refresh Results live layout: {exc}")
        finally:
            if self._results_refresh_task is asyncio.current_task():
                self._results_refresh_task = None

    async def _run_optimization_async(
        self,
        run_spec: SysIdRunSpec,
        config: Any,
        prepared: SysIdPreparedRun,
        run_generation: int,
        provenance_path: str,
        viewport_paused: bool = False,
    ) -> None:
        try:

            def on_progress(label: str, fraction: float) -> None:
                """Handle progress.

                Args:
                    label: Label value.
                    fraction: Fraction value.
                """
                if self._ui_active(run_generation):
                    self._results.set_activity(label, fraction)

            def on_iteration(status: OptimizationIterationStatus) -> None:
                """Handle iteration.

                Args:
                    status: Status value.
                """
                if not self._ui_active(run_generation):
                    return
                self._parameters.update_theta_values(status.theta)
                self._results.add_cost_point(status.iteration, status.cost)
                accept_str = "accepted" if status.accepted else "rejected"
                self._set_global_status(
                    f"Iter {status.iteration}: cost={status.cost:.6e}, " f"lambda={status.damping:.3e}, {accept_str}"
                )

            result: SysIdRunResult = await run_sysid(
                run_spec,
                on_iteration=on_iteration,
                on_progress=on_progress,
                prepared_run=prepared,
                provenance_sidecar_path=provenance_path,
                instance_local_writeback=True,
            )

            if self._ui_active(run_generation):
                final = result.selected_status
                writeback_failed = bool(run_spec.outputs.apply_parameters_to_usd) and not result.parameters_written
                self._parameters.update_theta_values(final.theta)
                self._results.add_cost_point(final.iteration, final.cost)
                if final.rollout is not None:
                    try:
                        self._results.show_final_joint_errors(final.rollout)
                    except Exception as chart_exc:
                        carb.log_warn(f"SysId: failed to update final joint error plots: {chart_exc}")
                self._results.set_activity("Finished with writeback failure" if writeback_failed else "Finished", 1.0)

                status_parts = [f"Finished: iter={final.iteration}, cost={final.cost:.6e}"]
                if result.parameters_written:
                    status_parts.append("instance-local USD parameters authored and verified")
                    if result.live_parameters_applied:
                        status_parts.append("live PhysX parameters refreshed")
                    status_parts.append("save stage to persist")
                elif writeback_failed:
                    status_parts.append("requested USD parameter writeback did not complete")
                if result.validation_metrics:
                    self._results.set_validation_results(
                        self._format_validation_metrics(
                            result.validation_metrics,
                            comparison=getattr(result, "validation_comparison", None),
                            validation_source=getattr(result, "validation_artifact_source", "configured_chunks"),
                        )
                    )
                    status_parts.append(summarize_validation_metrics(result.validation_metrics))
                else:
                    self._results.set_validation_results(
                        "No validation chunks were configured."
                        if not prepared.validation_chunks
                        else "Validation unavailable for the active optimizer."
                    )
                self._set_global_status("; ".join(status_parts), "error" if writeback_failed else "success")

                self._last_config = config
                self._last_final_status = final
                self._last_validation_metrics = list(result.validation_metrics)
                self._last_run_result = result
                self._results.set_parameter_deltas(self._format_parameter_deltas(config, final))
                self._results.set_parameter_confidence(getattr(result, "parameter_confidence", None))
                self._results.set_residual_summary(self._format_residual_summary(config, final))

        except asyncio.CancelledError:
            if self._ui_active(run_generation):
                self._set_global_status("Optimization cancelled.", "warning")
        except Exception as exc:
            carb.log_error(f"SysId optimization failed: {exc}\n{traceback.format_exc()}")
            if self._ui_active(run_generation):
                self._set_global_status(f"Optimization failed: {exc}", "error")
        finally:
            if viewport_paused:
                self._restore_viewport_rendering(update_status=False)
            if run_generation == self._run_generation:
                self._set_ui_running(False)
                self._run_task = None

    # ---------------------------------------------------------------- helpers

    def _pause_viewport_rendering_if_enabled(self) -> bool:
        if not self._solver.is_viewport_pause_enabled() or self._paused_viewport_state is not None:
            return False
        try:
            from omni.kit.viewport.utility import get_active_viewport

            viewport = get_active_viewport()
            if viewport is None or not hasattr(viewport, "updates_enabled"):
                return False
            previous = bool(viewport.updates_enabled)
            viewport.updates_enabled = False
            self._paused_viewport_state = (viewport, previous)
            carb.log_info("SysId: viewport rendering paused for optimization.")
            self._set_global_status("Viewport rendering paused for the System Identification run.")
            return True
        except Exception as exc:
            carb.log_warn(f"SysId: failed to pause viewport rendering: {exc}")
            return False

    def _restore_viewport_rendering(self, *, update_status: bool = True) -> bool:
        if self._paused_viewport_state is None:
            return False
        viewport, previous = self._paused_viewport_state
        self._paused_viewport_state = None
        try:
            if hasattr(viewport, "updates_enabled"):
                viewport.updates_enabled = previous
            carb.log_info("SysId: viewport rendering restored after optimization.")
            if update_status:
                self._set_global_status("Viewport rendering restored.")
            return True
        except Exception as exc:
            carb.log_warn(f"SysId: failed to restore viewport rendering: {exc}")
            return False

    def _set_ui_running(self, running: bool) -> None:
        self._running = running
        if self._workflow_shell is not None:
            self._workflow_shell.set_running(running)
        if self._run_button and not running:
            self._run_button.reset()
        self._recipe_bar.set_enabled(not running)
        self._check.set_enabled(not running)
        self._telemetry.set_enabled(not running)
        self._simulation.set_enabled(not running)
        self._parameters.set_enabled(not running)
        self._solver.set_enabled(not running)
        self._results.set_controls_enabled(not running)
        self._set_export_run_spec_enabled(not running and self._telemetry.get_trajectory() is not None)
        if not running:
            self._update_run_button_state(preserve_status=True)

    def _set_export_run_spec_enabled(self, enabled: bool) -> None:
        self._solver.set_export_run_spec_enabled(enabled)

    def _set_global_status(self, text: str, severity: str = "info") -> None:
        if self._workflow_shell is not None:
            self._workflow_shell.set_status(text, severity)

    def _run_preflight(self, spec: SysIdRunSpec) -> Any:
        stage = omni.usd.get_context().get_stage()
        return preflight_sysid_run_spec(
            spec,
            stage=stage,
            trajectory=self._telemetry.get_trajectory(),
            # Newton engines roll out without the PhysX timeline.
            timeline_playing=self._is_timeline_playing() or not self._requires_timeline(),
        )

    def _build_run_spec(self) -> SysIdRunSpec:
        trajectory = self._telemetry.get_trajectory()
        if trajectory is None:
            raise TrajectoryCsvError("Load a valid telemetry source first.")

        chunk_specs = self._telemetry.get_chunk_specs()
        robot_path = self._simulation.get_robot_path()
        if not robot_path:
            raise ValueError("Robot prim path is required.")

        if self._active_parameter_space is None:
            raise ValueError("Load telemetry and configure parameters before running.")
        selected = self._parameters.get_selected_parameter_run_specs()
        if not selected:
            raise ValueError("Select at least one parameter to optimize.")

        source_path = self._telemetry.get_source_path()
        input_stage_path = self._current_stage_path()
        backend_cfg = self._solver.get_backend_config()
        residual_cfg = self._solver.get_residual_config()
        out_flags = self._solver.get_output_flags()
        provenance_path = out_flags["provenance_sidecar_path"] or self._default_provenance_path()
        flags = self._parameters.get_advanced_flags()
        damping, epsilon, max_iter = self._solver.get_solver_floats()
        auto_split_enabled, auto_train_fraction, auto_min_chunk_seconds = self._telemetry.get_auto_split()

        build_trajectory_chunks(trajectory, chunk_specs, require_train=True, reject_overlaps=True)
        t0 = min(c.start for c in chunk_specs)
        t1 = max(c.end for c in chunk_specs)

        telemetry_quality = build_telemetry_quality_report(trajectory, chunk_specs).to_dict()
        return SysIdRunSpec(
            telemetry_quality=telemetry_quality,
            stage=StageRunSpec(
                input_path=input_stage_path,
            ),
            telemetry=TelemetryRunSpec(
                source_type=self._telemetry.get_source_type().value,
                source_path=source_path,
                mapping_path=self._telemetry.get_mapping_path(),
                chunk_manifest_path=self._telemetry.get_chunk_manifest_path(),
                # The loaded trajectory already has the shift applied; recording it
                # keeps exported run specs reproducible headlessly.
                command_alignment_seconds=self._telemetry.get_command_alignment_seconds(),
                time_window=TimeWindowRunSpec(start=t0, end=t1),
                # chunk_specs already carry the materialized auto-split (headless
                # from_dict prefers explicit chunks); the policy fields are recorded
                # for provenance.
                chunks=chunk_specs,
                auto_split=auto_split_enabled and not self._telemetry.has_explicit_chunks(),
                auto_split_train_fraction=auto_train_fraction,
                auto_split_min_chunk_seconds=auto_min_chunk_seconds,
            ),
            simulation=SimulationRunSpec(
                engine=self._simulation.get_engine(),
                robot_prim_path=robot_path,
                source_env_path=self._simulation.get_source_env_path(),
                env_paths_root=self._simulation.get_env_paths_root(),
                parallel_clones=self._simulation.is_parallel_clones(),
                co_locate_clones=self._simulation.is_co_locate_clones(),
                fabric_clones=self._simulation.is_fabric_clones(),
                offline_stepping=self._simulation.is_offline_stepping(),
                physics_backend=self._simulation.get_physics_backend(),
                actuator_runtime=self._simulation.get_actuator_runtime(),
                explicit_actuator_policy=self._simulation.get_explicit_actuator_policy(),
                newton=self._simulation.get_newton_spec(),
            ),
            parameters=ParametersRunSpec(
                space=ParameterSpaceRunSpec(include_basic=True, **flags),
                selected=selected,
            ),
            solver=SolverRunSpec(
                optimizer=backend_cfg.backend.value,
                damping_initial=damping,
                epsilon=epsilon,
                max_iterations=max_iter,
                max_rollout_steps=self._solver.get_max_rollout_steps(),
                use_analytical_jacobian=False,
                use_analytical_presolve=backend_cfg.use_analytical_presolve,
                cma_population_size=backend_cfg.cma_population_size,
                cma_sigma=backend_cfg.cma_sigma,
                cma_batch_size=backend_cfg.cma_batch_size,
                cma_seed=backend_cfg.cma_seed,
                bo_initial_samples=backend_cfg.bo_initial_samples,
                bo_candidate_count=backend_cfg.bo_candidate_count,
                bo_batch_size=backend_cfg.bo_batch_size,
                bo_seed=backend_cfg.bo_seed,
                gd_learning_rate=backend_cfg.gd_learning_rate,
                gd_num_restarts=backend_cfg.gd_num_restarts,
                gd_seed=backend_cfg.gd_seed,
            ),
            residuals=ResidualsRunSpec.from_config(residual_cfg),
            outputs=OutputsRunSpec(
                apply_parameters_to_usd=out_flags["apply_params_to_usd"],
                write_usd_provenance=out_flags["write_usd_provenance"],
                export_provenance_sidecar=out_flags["export_provenance_sidecar"],
                provenance_path=provenance_path,
                remove_clones_after_run=True,
            ),
            sampling=self._solver.get_sampling_spec(),
        )

    def _default_provenance_path(self) -> str:
        source = self._telemetry.get_source_path()
        if source:
            base, _ = os.path.splitext(source)
            return f"{base}_sysid_provenance.json"
        return "sysid_provenance.json"

    def _default_run_spec_path(self) -> str:
        source = self._telemetry.get_source_path()
        if source:
            base, _ = os.path.splitext(source)
            return f"{base}_sysid_run_spec.json"
        stage = self._current_stage_path()
        if stage and not stage.startswith("anon:") and "://" not in stage:
            path = Path(stage)
            return str(path.with_name(f"{path.stem}_sysid_run_spec.json"))
        return "sysid_run_spec.json"

    def _current_stage_path(self) -> str:
        stage = omni.usd.get_context().get_stage()
        if stage is None:
            return ""
        layer = stage.GetRootLayer()
        return getattr(layer, "realPath", "") or getattr(layer, "identifier", "") or ""

    def _format_validation_metrics(
        self,
        metrics: list[ChunkValidationMetric],
        *,
        comparison: dict | None = None,
        validation_source: str = "configured_chunks",
    ) -> str:
        if not metrics:
            return "No validation chunks were configured."
        source_label = (
            "Held-out validation"
            if validation_source == "configured_chunks"
            else "Training-data replay (not a holdout)"
        )
        lines = [source_label, summarize_validation_metrics(metrics)]
        comparison = comparison or {}
        if comparison.get("available"):
            lines.append("Untuned -> tuned RMSE (native channel units; improvement):")
            channel_labels = {
                "position_rmse": "q [rad/m]",
                "velocity_rmse": "dq [rad/s or m/s]",
                "torque_rmse": "effort [N.m/N]",
                "end_effector_pose_rmse": "EE pose",
                "contact_force_rmse": "contact force [N]",
            }
            for row in comparison.get("chunks", []):
                channel_parts = []
                for key, label in channel_labels.items():
                    delta = (row.get("channels") or {}).get(key) or {}
                    before = delta.get("untuned")
                    after = delta.get("tuned")
                    if before is None or after is None:
                        continue
                    percent = delta.get("percent_improvement")
                    percent_text = "n/a" if percent is None else f"{float(percent):+.1f}%"
                    channel_parts.append(
                        f"{label} {float(before):.3e} -> {float(after):.3e} ({percent_text}, {delta.get('status', '')})"
                    )
                if channel_parts:
                    lines.append(f"{row.get('name', 'validation')}: " + "; ".join(channel_parts))
        for metric in metrics:
            lines.append(
                f"Tuned {metric.name} [{metric.excitation}]: cost={metric.normalized_cost:.3e}, "
                f"qRMSE={metric.position_rmse:.3e}, dqRMSE={metric.velocity_rmse:.3e}"
            )
        return "\n".join(lines)

    def _format_parameter_deltas(self, config: Any, final: Any) -> str:
        if not config.param_entries:
            return "No optimized parameters."
        authored_initial = getattr(config, "theta_authored_initial", None)
        initial_tensor = authored_initial if authored_initial is not None else config.theta_initial
        initial = initial_tensor.detach().cpu().tolist()
        final_theta = list(final.theta)
        lines = []
        for idx, entry in enumerate(config.param_entries[:8]):
            before = float(initial[idx])
            after = float(final_theta[idx])
            try:
                name = entry.display_name(config.trajectory.num_joints)
            except Exception:
                name = str(entry.param_type)
            lines.append(f"{name}: {before:.5g} -> {after:.5g} (delta {after - before:+.3g})")
        extra = len(config.param_entries) - 8
        if extra > 0:
            lines.append(f"+{extra} more parameter(s); export the run report for the full list.")
        return "\n".join(lines)

    def _format_residual_summary(self, config: Any, final: Any) -> str:
        cfg = config.residual_weight_config or self._solver.get_residual_config()
        active = [
            f"{name}={value:g}"
            for name, value in (
                ("position", cfg.position_weight),
                ("velocity", cfg.velocity_weight),
                ("torque", cfg.torque_weight),
                ("ee_pose", cfg.end_effector_pose_weight),
                ("contact", cfg.contact_force_weight),
            )
            if value > 0
        ]
        notes = [
            "Channels: " + (", ".join(active) if active else "none"),
            f"Final cost={final.cost:.6e}, iteration={final.iteration}/{config.max_iterations}",
        ]
        if final.iteration >= config.max_iterations:
            notes.append("Reached max iterations; inspect validation before accepting parameters.")
        if not final.accepted:
            notes.append("Final trial was rejected; displayed theta is the last accepted vector.")
        return "\n".join(notes)

    # ---------------------------------------------------------------- report

    def _on_export_run_spec_clicked(self) -> None:
        try:
            output_path = self._export_run_spec()
            self._set_global_status(f"RunSpec exported to {output_path}", "success")
        except Exception as exc:
            carb.log_error(f"Failed to export SysID RunSpec: {exc}")
            self._set_global_status(f"RunSpec export failed: {exc}", "error")

    def _export_run_spec(self) -> str:
        path = Path(self._default_run_spec_path()).expanduser()
        path.parent.mkdir(parents=True, exist_ok=True)
        spec = self._build_run_spec()
        path.write_text(spec.to_json(), encoding="utf-8")
        return str(path)

    def _on_export_run_report_clicked(self) -> None:
        try:
            output_path = self._export_run_report()
            artifact_dir = str(Path(output_path).expanduser().resolve().parent)
            self._results.set_artifact_summary(f"Run report: {output_path}\nValidation artifacts: {artifact_dir}")
            self._set_global_status(f"Run report exported to {output_path}", "success")
        except Exception as exc:
            carb.log_error(f"Failed to export SysID run report: {exc}")
            self._set_global_status(f"Run report export failed: {exc}", "error")

    def _export_run_report(self) -> str:
        source = self._telemetry.get_source_path()
        if source:
            base, _ = os.path.splitext(source)
            default_path = f"{base}_sysid_report.json"
        else:
            stage = self._current_stage_path()
            if stage and not stage.startswith("anon:") and "://" not in stage:
                p = Path(stage)
                default_path = str(p.with_name(f"{p.stem}_sysid_report.json"))
            else:
                default_path = "sysid_report.json"

        path = Path(default_path).expanduser()
        spec = self._build_run_spec()
        if self._last_prepared_run is None or self._last_run_result is None:
            raise ValueError("Run optimization before exporting a run report.")
        export_sysid_run_report(
            path,
            spec=spec,
            prepared=self._last_prepared_run,
            result=self._last_run_result,
            schema_issues=validate_sysid_run_spec_payload(spec.to_dict()),
            # Reuse the assessment made during the pre-solve check; on high-rate
            # telemetry, repeating it would stall the UI thread.
            telemetry_quality=self._last_prepared_run.telemetry_quality,
            validation_artifact_dir=path.parent,
        )
        return str(path)

    # ---------------------------------------------------------------- lifecycle

    def on_menu_callback(self) -> None:
        """Handle menu callback."""

    def on_timeline_event(self, _event: object) -> None:
        """Refresh run readiness after Play, Pause, or Stop.

        Args:
            _event: Timeline event that triggered the refresh.
        """
        self._update_run_button_state()

    def on_window_hidden(self) -> None:
        """Handle window hidden."""
        self.cleanup()

    def cleanup(self) -> None:
        """Release resources."""
        self._restore_viewport_rendering(update_status=False)
        if self._running and self._last_prepared_run is not None:
            try:
                self._last_prepared_run.optimizer.request_cancel()
            except Exception:
                pass
        if self._run_task is not None and not self._run_task.done():
            self._run_task.cancel()
        if self._results_refresh_task is not None and not self._results_refresh_task.done():
            self._results_refresh_task.cancel()
        self._run_generation += 1

        if self._last_prepared_run is not None:
            try:
                self._last_prepared_run.optimizer.cleanup_bridge()
            except Exception:
                pass

        try:
            clear_gt_preview(omni.usd.get_context().get_stage())
        except Exception:
            pass

        self._telemetry.cleanup()
        self._simulation.cleanup()
        self._parameters.cleanup()
        self._solver.cleanup()
        self._results.cleanup()
        self._recipe_bar.cleanup()
        self._check.cleanup()
        self._pipeline.clear_check()
        if self._workflow_shell is not None:
            self._workflow_shell.cleanup()

        for element in self.wrapped_ui_elements:
            if hasattr(element, "cleanup"):
                element.cleanup()
        self.wrapped_ui_elements.clear()

        self._run_button = None
        self._workflow_shell = None
        self._page_frames = {}
        self._results_refresh_task = None
        self._run_task = None
        self._paused_viewport_state = None
        self._running = False
        self._active_parameter_space = None
        self._active_link_paths = []
        self._active_joint_paths = []
        self._last_config = None
        self._last_final_status = None
        self._last_validation_metrics = []
        self._last_run_result = None
        self._last_prepared_run = None

    def reset(self) -> None:
        """Handle reset."""
        self._last_config = None
        self._last_final_status = None
        self._last_validation_metrics = []
        self._last_run_result = None
        self._last_prepared_run = None
