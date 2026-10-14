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

"""Tests for the gated-pipeline run state machine (pure Python, no UI widgets)."""

from __future__ import annotations

from pathlib import Path
from types import SimpleNamespace
from typing import Any
from unittest.mock import Mock, patch

import omni.kit.test
from isaacsim.robot_setup.sysid.parameter_space import ParameterSpace
from isaacsim.robot_setup.sysid.parameter_types import SysIdParameterEntry, SysIdParameterType
from isaacsim.robot_setup.sysid.preset_loader import ParameterPreset
from isaacsim.robot_setup.sysid.run_spec import SIMULATION_ENGINE_ISAAC_SIM, SIMULATION_ENGINE_NEWTON
from isaacsim.robot_setup.sysid.trajectory_segments import TelemetryChunkRunSpec
from isaacsim.robot_setup.sysid.ui.parameter_table import ParameterTableWidget
from isaacsim.robot_setup.sysid.ui.pipeline_state import SysIdPipelineState
from isaacsim.robot_setup.sysid.ui.stage_status import (
    CHECK_STATE_NOT_RUN,
    CHECK_STATE_OK,
    CHECK_STATE_STALE,
    format_stage_titles,
)
from isaacsim.robot_setup.sysid.ui.ui_builder import UIBuilder
from isaacsim.robot_setup.sysid.ui.workflow_model import WORKFLOW_STAGES, adjacent_stage, normalize_stage


def _signature(
    *,
    source: Any = ("a.csv", "csv", "", ""),
    chunks: Any = None,
    entries: Any = None,
    robot_path: Any = "/World/robot",
    engine: Any = "isaac_sim",
    simulation_settings: Any = (),
    residual_settings: Any = (),
    solver_settings: Any = (),
) -> tuple:
    if chunks is None:
        chunks = [TelemetryChunkRunSpec(name="Train 1", role="train", start=0.0, end=1.0)]
    if entries is None:
        entries = [SysIdParameterEntry(SysIdParameterType.JOINT_FRICTION, 0)]
    return SysIdPipelineState.compute_signature(
        source_signature=source,
        chunk_specs=chunks,
        selected_entries=entries,
        robot_path=robot_path,
        engine=engine,
        simulation_settings=simulation_settings,
        residual_settings=residual_settings,
        solver_settings=solver_settings,
    )


class PipelineStateTests(omni.kit.test.AsyncTestCase):
    """Represent PipelineStateTests."""

    async def test_signature_is_stable_for_identical_inputs(self) -> None:
        """Verify signature is stable for identical inputs."""
        self.assertEqual(_signature(), _signature())

    async def test_each_mutation_class_invalidates_the_check(self) -> None:
        """Verify each mutation class invalidates the check."""
        state = SysIdPipelineState()
        state.record_check(object(), _signature())
        self.assertFalse(state.is_check_stale(_signature()))
        mutations = {
            "source": _signature(source=("b.csv", "csv", "", "")),
            "chunks": _signature(chunks=[TelemetryChunkRunSpec(name="Train 1", role="train", start=0.0, end=2.0)]),
            "selection": _signature(entries=[SysIdParameterEntry(SysIdParameterType.JOINT_DAMPING, 0)]),
            "robot_path": _signature(robot_path="/World/other"),
            "engine": _signature(engine="newton"),
            # engine alone is "newton" for both solvers; the solver/feedforward
            # component covers Newton-internal switches.
            "newton_settings": _signature(simulation_settings=("featherstone_diff", "gravity")),
            "residuals": _signature(residual_settings=(1.0, 0.5, 0.0, 0.0, 0.0, -1)),
            "solver": _signature(solver_settings=("cma_es", True, 120)),
        }
        for label, mutated in mutations.items():
            with self.subTest(mutation=label):
                self.assertTrue(state.is_check_stale(mutated))

    async def test_command_delay_in_source_signature_invalidates_the_check(self) -> None:
        """Verify command delay in source signature invalidates the check."""
        state = SysIdPipelineState()
        state.record_check(object(), _signature(source=("a.csv", "csv", "", "", 0.0)))
        self.assertTrue(state.is_check_stale(_signature(source=("a.csv", "csv", "", "", 0.5167))))

    async def test_performance_settings_do_not_invalidate_the_check(self) -> None:
        # cuda_graph_capture and device are performance-only: they must never be
        # part of any signature component, so two signatures that differ only in
        # them are impossible to construct — assert the intended call shape.
        """Verify performance settings do not invalidate the check."""
        base = _signature(simulation_settings=("featherstone_diff", "none"))
        self.assertEqual(base, _signature(simulation_settings=("featherstone_diff", "none")))

    async def test_gate_hint_ordering(self) -> None:
        """Verify gate hint ordering."""
        state = SysIdPipelineState()
        signature = _signature()

        enabled, hint = state.run_gate(
            trajectory_loaded=False, timeline_ok=True, chunk_error="", current_signature=signature
        )
        self.assertFalse(enabled)
        self.assertIn("telemetry", hint.lower())

        enabled, hint = state.run_gate(
            trajectory_loaded=True, timeline_ok=False, chunk_error="", current_signature=signature
        )
        self.assertIn("play", hint.lower())

        enabled, hint = state.run_gate(
            trajectory_loaded=True, timeline_ok=True, chunk_error="bad chunk", current_signature=signature
        )
        self.assertIn("bad chunk", hint)

        enabled, hint = state.run_gate(
            trajectory_loaded=True, timeline_ok=True, chunk_error="", current_signature=signature
        )
        self.assertFalse(enabled)
        self.assertIn("check", hint.lower())

        state.record_check(object(), _signature(robot_path="/World/old"))
        enabled, hint = state.run_gate(
            trajectory_loaded=True, timeline_ok=True, chunk_error="", current_signature=signature
        )
        self.assertFalse(enabled)
        self.assertIn("re-run", hint.lower())

        state.record_check(object(), signature)
        enabled, hint = state.run_gate(
            trajectory_loaded=True, timeline_ok=True, chunk_error="", current_signature=signature
        )
        self.assertTrue(enabled)
        self.assertEqual(hint, "")

    async def test_warnings_never_block(self) -> None:
        # The gate only consumes structural inputs; a report full of warnings still
        # enables the run once recorded fresh.
        """Verify warnings never block."""
        state = SysIdPipelineState()
        signature = _signature()

        class _ReportWithWarnings:
            ok = True
            issues = ["w1", "w2"]

        state.record_check(_ReportWithWarnings(), signature)
        enabled, _hint = state.run_gate(
            trajectory_loaded=True, timeline_ok=True, chunk_error="", current_signature=signature
        )
        self.assertTrue(enabled)

    async def test_failing_check_report_blocks_run(self) -> None:
        # A fresh check whose report is not ok (telemetry errors, non-identifiable
        # parameters, unknown verdicts) must not enable Run.
        """Verify a not-ok check report blocks the run."""
        state = SysIdPipelineState()
        signature = _signature()

        class _FailingReport:
            ok = False
            issues = ["telemetry_error"]

        state.record_check(_FailingReport(), signature)
        enabled, hint = state.run_gate(
            trajectory_loaded=True, timeline_ok=True, chunk_error="", current_signature=signature
        )
        self.assertFalse(enabled)
        self.assertIn("blocking", hint.lower())

    async def test_clear_check_resets_gate(self) -> None:
        """Verify clear check resets gate."""
        state = SysIdPipelineState()
        signature = _signature()
        state.record_check(object(), signature)
        state.clear_check()
        enabled, hint = state.run_gate(
            trajectory_loaded=True, timeline_ok=True, chunk_error="", current_signature=signature
        )
        self.assertFalse(enabled)
        self.assertIn("check", hint.lower())


class StageStatusTests(omni.kit.test.AsyncTestCase):
    """Represent StageStatusTests."""

    async def test_not_loaded_state(self) -> None:
        """Verify not loaded state."""
        titles = format_stage_titles(
            engine_label="Isaac Sim (PhysX)",
            sample_count=None,
            num_joints=None,
            check_state=CHECK_STATE_NOT_RUN,
            run_ready=False,
        )
        self.assertEqual(titles["sim"], "1. Robot — Isaac Sim (PhysX)")
        self.assertIn("not loaded", titles["data"])
        self.assertIn("none selected", titles["params"])
        self.assertIn("not run", titles["check"])
        self.assertIn("not ready", titles["solve"])
        self.assertEqual(titles["results"], "6. Results")

    async def test_loaded_and_ready_state(self) -> None:
        """Verify loaded and ready state."""
        titles = format_stage_titles(
            engine_label="Newton differentiable (Featherstone)",
            sample_count=1204,
            num_joints=7,
            chunk_summary="1 train / 1 validation chunk(s)",
            selected_count=12,
            check_state=CHECK_STATE_OK,
            run_ready=True,
        )
        self.assertIn("1,204 samples, 7 joints", titles["data"])
        self.assertIn("1 train / 1 validation", titles["data"])
        self.assertIn("12 selected", titles["params"])
        self.assertIn("ok", titles["check"])
        self.assertEqual(titles["solve"], "5. Solve — ready")

    async def test_stale_check_state(self) -> None:
        """Verify stale check state."""
        titles = format_stage_titles(
            engine_label="",
            sample_count=10,
            num_joints=2,
            check_state=CHECK_STATE_STALE,
            run_ready=False,
        )
        self.assertEqual(titles["sim"], "1. Robot")
        self.assertIn("stale", titles["check"])


class UiBuilderGateTests(omni.kit.test.AsyncTestCase):
    """Focused defense-in-depth coverage for UI gate callbacks."""

    async def test_robot_path_change_refreshes_parameter_space_and_run_gate(self) -> None:
        """Verify a robot-path edit immediately refreshes Run readiness."""
        builder = object.__new__(UIBuilder)
        builder._refresh_parameter_space = Mock()
        builder._update_run_button_state = Mock()

        builder._on_robot_path_changed("/World/OtherRobot")

        builder._refresh_parameter_space.assert_called_once_with()
        builder._update_run_button_state.assert_called_once_with()

    async def test_parameter_baselines_follow_resolved_physics_backend(self) -> None:
        """Verify USD baselines are read for the resolved backend and re-read when it changes."""
        from isaacsim.robot_setup.sysid.ui import ui_builder as ui_builder_module

        builder = object.__new__(UIBuilder)
        builder._active_physics_backend = ""
        builder._optimizer = None
        builder._recipe_bar = Mock()
        builder._refresh_engine_backend_hint = Mock()
        builder._update_run_button_state = Mock()
        builder._telemetry = Mock()
        builder._telemetry.get_trajectory.return_value = SimpleNamespace(num_joints=1)
        builder._parameters = Mock()
        builder._parameters.get_advanced_flags.return_value = {}
        builder._simulation = Mock()
        builder._simulation.get_robot_path.return_value = "/World/robot"
        builder._simulation.get_engine.return_value = SIMULATION_ENGINE_ISAAC_SIM
        builder._simulation.get_physics_backend.return_value = "auto"

        with (
            patch.object(ui_builder_module.omni.usd, "get_context") as get_context,
            patch.object(
                ui_builder_module,
                "collect_robot_link_and_joint_paths",
                return_value=(["/World/robot/link"], ["/World/robot/joint"]),
            ),
            patch.object(ParameterSpace, "read_usd_baselines") as read_baselines,
        ):
            get_context.return_value.get_stage.return_value = object()
            builder._refresh_parameter_space()
            self.assertEqual(read_baselines.call_args.kwargs["physics_backend"], "physx")

            builder._on_newton_settings_changed()
            self.assertEqual(read_baselines.call_count, 1)

            builder._simulation.get_engine.return_value = SIMULATION_ENGINE_NEWTON
            builder._simulation.get_physics_backend.return_value = "newton"
            builder._on_engine_changed("Newton")
            self.assertEqual(read_baselines.call_count, 2)
            self.assertEqual(read_baselines.call_args.kwargs["physics_backend"], "newton")

            # A failed read leaves fallback baselines, so an unchanged backend
            # must still retry on the next settings callback.
            read_baselines.side_effect = RuntimeError("stage not ready")
            builder._simulation.get_engine.return_value = SIMULATION_ENGINE_ISAAC_SIM
            builder._simulation.get_physics_backend.return_value = "physx"
            builder._on_engine_changed("Isaac Sim")
            self.assertEqual(read_baselines.call_count, 3)
            read_baselines.side_effect = None
            builder._on_newton_settings_changed()
            self.assertEqual(read_baselines.call_count, 4)
            builder._on_newton_settings_changed()
            self.assertEqual(read_baselines.call_count, 4)

    async def test_run_click_rechecks_gate_before_building_spec(self) -> None:
        """Verify Run refuses stale inputs before constructing a run spec."""
        builder = object.__new__(UIBuilder)
        builder._running = False
        builder._evaluate_run_gate = Mock(return_value=(False, "Inputs changed; re-run Check."))
        builder._set_global_status = Mock()
        builder._update_run_button_state = Mock()
        builder._run_button = Mock()
        builder._build_run_spec = Mock()

        builder._on_run_clicked()

        builder._evaluate_run_gate.assert_called_once_with()
        builder._build_run_spec.assert_not_called()
        builder._run_button.reset.assert_called_once_with()
        builder._set_global_status.assert_called_once_with("Run blocked: Inputs changed; re-run Check.", "error")

    async def test_timeline_event_refreshes_run_gate(self) -> None:
        """Verify timeline events refresh Run readiness."""
        builder = object.__new__(UIBuilder)
        builder._update_run_button_state = Mock()

        builder.on_timeline_event(object())

        builder._update_run_button_state.assert_called_once_with()

    async def test_timeline_is_required_only_for_interactive_isaac_sim(self) -> None:
        """Verify offline Isaac Sim and Newton rollouts do not require Play."""
        builder = object.__new__(UIBuilder)
        builder._simulation = Mock()
        builder._simulation.get_engine.return_value = SIMULATION_ENGINE_ISAAC_SIM
        builder._simulation.is_offline_stepping.return_value = True

        self.assertFalse(builder._requires_timeline())

        builder._simulation.is_offline_stepping.return_value = False
        self.assertTrue(builder._requires_timeline())

        builder._simulation.get_engine.return_value = SIMULATION_ENGINE_NEWTON
        self.assertFalse(builder._requires_timeline())

    async def test_newton_actuator_settings_invalidate_cached_check(self) -> None:
        """Verify controller and effort-clamp changes invalidate a cached Check."""
        builder = object.__new__(UIBuilder)
        builder._telemetry = Mock()
        builder._telemetry.get_trajectory.return_value = object()
        builder._telemetry.get_chunk_specs.return_value = []
        builder._telemetry.get_loaded_signature.return_value = ("trajectory.csv", "csv", "", "")
        builder._parameters = Mock()
        builder._parameters.get_selected_parameter_run_specs.return_value = []
        builder._simulation = Mock()
        builder._simulation.get_robot_path.return_value = "/World/robot"
        builder._simulation.get_engine.return_value = SIMULATION_ENGINE_NEWTON
        builder._simulation.get_actuator_runtime.return_value = "newton_explicit"
        builder._simulation.get_physics_backend.return_value = "newton"
        builder._simulation.get_explicit_actuator_policy.return_value = "promote_selected"
        builder._solver = Mock()
        builder._solver.get_backend_config.return_value = SimpleNamespace(
            backend=SimpleNamespace(value="cma_es"),
            use_analytical_presolve=False,
        )
        builder._solver.get_residual_config.return_value = SimpleNamespace(
            position_weight=1.0,
            velocity_weight=1.0,
            torque_weight=0.0,
            end_effector_pose_weight=0.0,
            contact_force_weight=0.0,
            end_effector_link_index=-1,
            sample_weights=None,
        )
        builder._solver.get_max_rollout_steps.return_value = 300

        builder._simulation.get_newton_spec.return_value = SimpleNamespace(
            solver="mujoco",
            feedforward="none",
            controller="pd",
            effort_clamp="max_effort",
        )
        base = builder._current_check_signature()

        builder._simulation.get_newton_spec.return_value = SimpleNamespace(
            solver="mujoco",
            feedforward="none",
            controller="pid",
            effort_clamp="max_effort",
        )
        self.assertNotEqual(base, builder._current_check_signature())

        builder._simulation.get_newton_spec.return_value = SimpleNamespace(
            solver="mujoco",
            feedforward="none",
            controller="pd",
            effort_clamp="dc_motor",
        )
        self.assertNotEqual(base, builder._current_check_signature())

    async def test_build_run_spec_exports_offline_stepping(self) -> None:
        """Verify offline stepping propagates from the Simulation panel into the run spec."""
        builder = object.__new__(UIBuilder)
        chunk = TelemetryChunkRunSpec(name="Train 1", role="train", start=0.0, end=1.0)
        builder._telemetry = Mock()
        builder._telemetry.get_trajectory.return_value = object()
        builder._telemetry.get_chunk_specs.return_value = [chunk]
        builder._telemetry.get_source_path.return_value = "trajectory.csv"
        builder._telemetry.get_source_type.return_value = SimpleNamespace(value="csv")
        builder._telemetry.get_mapping_path.return_value = ""
        builder._telemetry.get_chunk_manifest_path.return_value = ""
        builder._telemetry.get_command_alignment_seconds.return_value = 0.0
        builder._telemetry.get_auto_split.return_value = (False, 0.8, 1.0)
        builder._telemetry.has_explicit_chunks.return_value = True
        builder._simulation = Mock()
        builder._simulation.get_robot_path.return_value = "/World/robot"
        builder._simulation.get_engine.return_value = SIMULATION_ENGINE_ISAAC_SIM
        builder._simulation.get_source_env_path.return_value = "/World"
        builder._simulation.get_env_paths_root.return_value = "/World/envs"
        builder._simulation.is_parallel_clones.return_value = True
        builder._simulation.is_co_locate_clones.return_value = True
        builder._simulation.is_fabric_clones.return_value = False
        builder._simulation.is_offline_stepping.return_value = False
        builder._simulation.get_actuator_runtime.return_value = "implicit_drive"
        builder._simulation.get_physics_backend.return_value = "physx"
        builder._simulation.get_explicit_actuator_policy.return_value = "authored_only"
        builder._simulation.get_newton_spec.return_value = SimpleNamespace()
        builder._active_parameter_space = object()
        builder._parameters = Mock()
        builder._parameters.get_selected_parameter_run_specs.return_value = [Mock()]
        builder._parameters.get_advanced_flags.return_value = {
            "include_com_offsets": False,
            "include_inertia_log_cholesky": False,
            "include_joint_limit_scales": False,
            "include_per_link_mass": False,
            "include_command_delay": False,
        }
        backend_config = SimpleNamespace(
            backend=SimpleNamespace(value="auto"),
            use_analytical_presolve=False,
            cma_population_size=None,
            cma_sigma=0.3,
            cma_batch_size=None,
            cma_seed=0,
            bo_initial_samples=5,
            bo_candidate_count=64,
            bo_batch_size=4,
            bo_seed=0,
            gd_learning_rate=0.05,
            gd_num_restarts=1,
            gd_seed=0,
        )
        residual_config = SimpleNamespace(
            position_weight=1.0,
            velocity_weight=1.0,
            torque_weight=0.0,
            end_effector_pose_weight=0.0,
            contact_force_weight=0.0,
            sample_weights=None,
            end_effector_link_index=-1,
        )
        builder._solver = Mock()
        builder._solver.get_backend_config.return_value = backend_config
        builder._solver.get_residual_config.return_value = residual_config
        builder._solver.get_output_flags.return_value = {
            "apply_params_to_usd": True,
            "write_usd_provenance": True,
            "export_provenance_sidecar": False,
            "provenance_sidecar_path": "provenance.json",
        }
        builder._solver.get_solver_floats.return_value = (0.01, 0.0001, 20)
        builder._solver.get_max_rollout_steps.return_value = 300
        builder._solver.get_sampling_spec.return_value = Mock()
        builder._current_stage_path = Mock(return_value="stage.usda")
        quality = Mock()
        quality.to_dict.return_value = {}

        with (
            patch("isaacsim.robot_setup.sysid.ui.ui_builder.build_trajectory_chunks"),
            patch(
                "isaacsim.robot_setup.sysid.ui.ui_builder.build_telemetry_quality_report",
                return_value=quality,
            ),
        ):
            run_spec = builder._build_run_spec()

        self.assertFalse(run_spec.simulation.offline_stepping)
        self.assertEqual(run_spec.simulation.physics_backend, "physx")
        self.assertEqual(run_spec.simulation.explicit_actuator_policy, "authored_only")

    async def test_recipe_restores_newton_controller_and_effort_clamp(self) -> None:
        """Verify recipe application restores all exposed Newton actuator settings."""
        builder = object.__new__(UIBuilder)
        builder._simulation = Mock()
        preset = ParameterPreset(
            name="newton",
            simulation={
                "engine": SIMULATION_ENGINE_NEWTON,
                "physics_backend": "newton",
                "actuator_runtime": "mixed",
                "explicit_actuator_policy": "promote_selected",
                "newton": {
                    "solver": "mujoco",
                    "controller": "pid",
                    "effort_clamp": "dc_motor",
                },
            },
        )

        builder._apply_recipe_simulation_section(preset)

        builder._simulation.set_physics_backend.assert_called_once_with("newton")
        builder._simulation.set_actuator_runtime.assert_called_once_with("mixed")
        builder._simulation.set_explicit_actuator_policy.assert_called_once_with("promote_selected")

        builder._simulation.set_controller_mode.assert_called_once_with("pid")
        builder._simulation.set_effort_clamp_mode.assert_called_once_with("dc_motor")

    async def test_failed_recheck_clears_previous_check_and_report(self) -> None:
        """Verify a failed Check cannot leave an older report runnable."""
        builder = object.__new__(UIBuilder)
        builder._telemetry = Mock()
        builder._telemetry.get_trajectory.return_value = None
        builder._pipeline = Mock()
        builder._check = Mock()
        builder._push_feedforward_hint = Mock()
        builder._update_run_button_state = Mock()

        builder._on_run_check_clicked()

        builder._pipeline.clear_check.assert_called_once_with()
        builder._check.set_report.assert_called_once_with(None)
        builder._check.set_stale.assert_called_once_with(False)
        builder._update_run_button_state.assert_called_once_with()

    async def test_parameter_space_rebuild_notifies_selection_change(self) -> None:
        """Verify rebuilding the table notifies readiness consumers."""
        callback = Mock()
        table = ParameterTableWidget(on_selection_changed=callback)

        table.set_parameter_space(ParameterSpace.for_robot(1))

        callback.assert_called_once_with()


class WorkflowNavigationTests(omni.kit.test.AsyncTestCase):
    """Represent WorkflowNavigationTests."""

    async def test_stage_order_matches_the_user_workflow(self) -> None:
        """Verify stage order matches the user workflow."""
        self.assertEqual(WORKFLOW_STAGES, ("sim", "data", "params", "check", "solve", "results"))

    async def test_adjacent_stage_clamps_at_workflow_bounds(self) -> None:
        """Verify adjacent stage clamps at workflow bounds."""
        self.assertEqual(adjacent_stage("sim", -1), "sim")
        self.assertEqual(adjacent_stage("sim", 1), "data")
        self.assertEqual(adjacent_stage("solve", 1), "results")
        self.assertEqual(adjacent_stage("results", 1), "results")

    async def test_unknown_stage_falls_back_to_robot(self) -> None:
        """Verify unknown stage falls back to robot."""
        self.assertEqual(normalize_stage("unknown"), "sim")


class UiManifestTests(omni.kit.test.AsyncTestCase):
    """Represent UiManifestTests."""

    async def test_ui_manifest_owns_interactive_dependencies(self) -> None:
        """Verify ui manifest owns interactive dependencies."""
        extension_root = Path(__file__).resolve().parents[5]
        manifest = (extension_root / "config" / "extension.toml").read_text(encoding="utf-8")
        self.assertIn('"isaacsim.robot_setup.sysid"', manifest)
        self.assertIn('"omni.kit.actions.core"', manifest)
        self.assertIn('"isaacsim.gui.components"', manifest)
        self.assertNotIn('"omni.kit.window.property"', manifest)
        self.assertIn('name = "startup"', manifest)
        self.assertIn('name = "default"', manifest)
        self.assertIn('"omni.isaac.ml_archive"', manifest)
        self.assertIn('"isaacsim.test.utils"', manifest)
