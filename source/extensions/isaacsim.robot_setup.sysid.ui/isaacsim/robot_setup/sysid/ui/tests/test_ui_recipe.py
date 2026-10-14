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

"""Tests for recipe/engine UI mapping logic and panel setter round-trips."""

from __future__ import annotations

from typing import Any
from unittest.mock import Mock

import omni.kit.app
import omni.kit.test
from isaacsim.robot_setup.sysid.actuator_compatibility import (
    ACTUATOR_RUNTIME_AUTO,
    ACTUATOR_RUNTIME_EXPLICIT,
    ACTUATOR_RUNTIME_MIXED,
    EXPLICIT_ACTUATOR_POLICY_PROMOTE_SELECTED,
    PHYSICS_BACKEND_NEWTON,
)
from isaacsim.robot_setup.sysid.optimizer_config import OptimizerBackend, OptimizerBackendConfig
from isaacsim.robot_setup.sysid.preset_loader import ParameterPreset, load_builtin_parameter_preset
from isaacsim.robot_setup.sysid.recipes import recipe_parameter_type_values
from isaacsim.robot_setup.sysid.residual_config import ResidualWeightConfig
from isaacsim.robot_setup.sysid.run_spec import (
    NEWTON_SOLVER_FEATHERSTONE_DIFF,
    NEWTON_SOLVER_MUJOCO,
    SIMULATION_ENGINE_ISAAC_SIM,
    SIMULATION_ENGINE_NEWTON,
)

# The UI modules depend on isaacsim.gui.components, which is unavailable in the
# headless test process; skip the UI-facing tests there.
try:
    import omni.ui as ui
    from isaacsim.robot_setup.sysid.ui.live_charts import _cost_plot_bounds, _CostPlot
    from isaacsim.robot_setup.sysid.ui.numeric_inputs import ExactFloatField, ExactIntField
    from isaacsim.robot_setup.sysid.ui.parameter_panel import ParameterPanel
    from isaacsim.robot_setup.sysid.ui.simulation_panel import (
        SimulationPanel,
        engine_label_from_selection,
        engine_selection_from_label,
    )
    from isaacsim.robot_setup.sysid.ui.solver_panel import SolverPanel, visible_groups_for_backend
    from isaacsim.robot_setup.sysid.ui.telemetry_panel import TelemetryPanel
    from isaacsim.robot_setup.sysid.ui.workflow_shell import WorkflowShell

    _UI_AVAILABLE = True
    _UI_REASON = ""
except ImportError as exc:  # pragma: no cover - environment dependent
    _UI_AVAILABLE = False
    _UI_REASON = str(exc)


class RecipeMappingTests(omni.kit.test.AsyncTestCase):
    """Represent RecipeMappingTests."""

    def setUp(self) -> None:
        """Handle setUp."""
        super().setUp()
        if not _UI_AVAILABLE:
            self.skipTest(_UI_REASON)

    async def test_visible_groups_decision_table(self) -> None:
        """Verify visible groups decision table."""
        self.assertEqual(visible_groups_for_backend(OptimizerBackend.AUTO), {"lm", "cma", "bo", "gd"})
        self.assertEqual(visible_groups_for_backend(OptimizerBackend.LEVENBERG_MARQUARDT), {"lm"})
        self.assertEqual(visible_groups_for_backend(OptimizerBackend.CMA_ES), {"cma"})
        self.assertEqual(visible_groups_for_backend(OptimizerBackend.BAYESIAN), {"bo"})
        self.assertEqual(visible_groups_for_backend(OptimizerBackend.GRADIENT_DESCENT), {"gd"})

    async def test_engine_label_mapping_round_trips(self) -> None:
        """Verify engine label mapping round trips."""
        selections = (
            (SIMULATION_ENGINE_ISAAC_SIM, NEWTON_SOLVER_MUJOCO),
            (SIMULATION_ENGINE_NEWTON, NEWTON_SOLVER_MUJOCO),
            (SIMULATION_ENGINE_NEWTON, NEWTON_SOLVER_FEATHERSTONE_DIFF),
        )
        for engine, newton_solver in selections:
            label = engine_label_from_selection(engine, newton_solver)
            self.assertEqual(engine_selection_from_label(label), (engine, newton_solver))
        self.assertEqual(engine_selection_from_label("bogus"), (SIMULATION_ENGINE_ISAAC_SIM, NEWTON_SOLVER_MUJOCO))

    async def test_recipe_parameter_types_follow_flags(self) -> None:
        """Verify recipe parameter types follow flags."""
        basic = recipe_parameter_type_values(ParameterPreset(name="basic"))
        self.assertIn("joint_friction", basic)
        self.assertIn("link_mass", basic)
        self.assertNotIn("joint_integral_gain", basic)
        self.assertNotIn("actuator_command_delay_seconds", basic)

        manipulator = recipe_parameter_type_values(load_builtin_parameter_preset("manipulator"))
        self.assertIn("link_com_offset_x", manipulator)
        self.assertIn("link_inertia_log_cholesky", manipulator)
        self.assertIn("joint_limit_lower_scale", manipulator)

        quadruped = recipe_parameter_type_values(load_builtin_parameter_preset("quadruped"))
        self.assertIn("joint_limit_lower_scale", quadruped)


class PanelRoundTripTests(omni.kit.test.AsyncTestCase):
    """Setter/getter round trips on panels built inside a throwaway window."""

    def setUp(self) -> None:
        """Handle setUp."""
        super().setUp()
        if not _UI_AVAILABLE:
            self.skipTest(_UI_REASON)

    async def _build_window(self, build_fn: Any) -> "ui.Window":
        window = ui.Window("SysId panel test", width=400, height=600, visible=True)
        with window.frame:
            build_fn()
        await omni.kit.app.get_app().next_update_async()
        return window

    async def test_workflow_shell_shows_one_stage_and_preserves_cancel_action(self) -> None:
        """Verify workflow shell shows one stage and preserves cancel action."""
        shell = WorkflowShell()
        pages = {}

        def _build() -> None:
            with ui.VStack():
                shell.build_header()
                shell.build_navigation()
                shell.build_status()
                with ui.VStack():
                    for stage in ("sim", "data", "params", "check", "solve", "results"):
                        pages[stage] = ui.Frame(visible=False)
                shell.bind_pages(pages)
                action = shell.build_footer()
                with action:
                    ui.Label("Run / Cancel")

        window = await self._build_window(_build)
        try:
            self.assertTrue(pages["sim"].visible)
            shell.activate("solve")
            self.assertTrue(pages["solve"].visible)
            self.assertEqual(sum(frame.visible for frame in pages.values()), 1)
            shell.activate("results")
            shell.set_running(True)
            self.assertTrue(shell._action_frame.visible)
        finally:
            shell.cleanup()
            window.destroy()

    async def test_compact_cost_plot_has_bounded_height(self) -> None:
        """Verify compact cost plot has bounded height."""
        plot = _CostPlot()
        window = await self._build_window(plot.build)
        try:
            plot.set_data([0.0, 1.0, 2.0], [10.0, 5.0, 2.5])
            await omni.kit.app.get_app().next_update_async()
            self.assertGreater(plot._container_frame.computed_height, 100)
            self.assertLess(plot._container_frame.computed_height, 240)
            self.assertEqual(
                _cost_plot_bounds([0.0, 1.0], [5.0, 2.5]),
                (-0.05, 1.05, 2.375, 5.125),
            )
        finally:
            plot.cleanup()
            window.destroy()

    async def test_solver_panel_backend_config_round_trip(self) -> None:
        """Verify solver panel backend config round trip."""
        panel = SolverPanel()
        window = await self._build_window(panel.build)
        try:
            config = OptimizerBackendConfig(
                backend=OptimizerBackend.GRADIENT_DESCENT,
                cma_sigma=0.4,
                cma_seed=20260730,
                bo_seed=31,
                gd_learning_rate=0.12,
                gd_num_restarts=3,
                gd_seed=47,
            )
            panel.set_backend_config(config)
            await omni.kit.app.get_app().next_update_async()
            result = panel.get_backend_config()
            self.assertEqual(result.backend, OptimizerBackend.GRADIENT_DESCENT)
            self.assertAlmostEqual(result.gd_learning_rate, 0.12, places=6)
            self.assertEqual(result.gd_num_restarts, 3)
            self.assertAlmostEqual(result.cma_sigma, 0.4, places=6)
            self.assertEqual(result.cma_seed, 20260730)
            self.assertEqual(result.bo_seed, 31)
            self.assertEqual(result.gd_seed, 47)

            panel.set_residual_config(ResidualWeightConfig(torque_weight=0.7, contact_force_weight=0.0))
            residuals = panel.get_residual_config()
            self.assertAlmostEqual(residuals.torque_weight, 0.7, places=6)

            panel.set_differentiable_bridge_hint(True)
            self.assertTrue(panel.get_backend_config().differentiable_bridge)

            # Output flags and viewport pause live in the "Outputs & stage changes" frame
            # since the Solve restructure; the getters must keep working.
            flags = panel.get_output_flags()
            self.assertTrue(flags["apply_params_to_usd"])
            self.assertTrue(flags["write_usd_provenance"])
            self.assertFalse(flags["export_provenance_sidecar"])
            self.assertTrue(panel.is_viewport_pause_enabled())
            panel.set_export_run_spec_enabled(False)

            self.assertIsInstance(panel._max_iter_field, ExactIntField)
            self.assertIsInstance(panel._lambda_field, ExactFloatField)
            panel._max_iter_field.set_value(0)
            self.assertEqual(panel._max_iter_field.get_value(), 1)
        finally:
            panel.cleanup()
            window.destroy()

    async def test_telemetry_panel_uses_exact_inputs_and_percentage_holdout(self) -> None:
        """Verify the Data panel uses exact inputs and presents holdout as a percentage."""
        on_run_state_changed = Mock()
        panel = TelemetryPanel(on_run_state_changed=on_run_state_changed)
        window = await self._build_window(panel.build)
        try:
            self.assertIsInstance(panel._command_alignment_field, ExactFloatField)
            self.assertIsInstance(panel._start_time_field, ExactFloatField)
            self.assertIsInstance(panel._end_time_field, ExactFloatField)

            panel.set_auto_split(True, train_fraction=0.75, min_chunk_seconds=0.5)
            enabled, fraction, minimum = panel.get_auto_split()
            self.assertTrue(enabled)
            self.assertAlmostEqual(fraction, 0.75)
            self.assertAlmostEqual(minimum, 0.5)
            self.assertAlmostEqual(panel._auto_split_fraction_model.get_value_as_float(), 75.0)

            panel._on_chunk_role_changed("Validation")
            self.assertFalse(panel._chunk_weight_widget.enabled)
            panel._on_chunk_role_changed("Train")
            self.assertTrue(panel._chunk_weight_widget.enabled)

            # A programmatic set is not a user edit: callers that set the window
            # (trajectory load, recipe apply) notify run state once themselves.
            on_run_state_changed.reset_mock()
            panel._start_time_field.set_value(0.25)
            self.assertAlmostEqual(panel._start_time_field.get_value(), 0.25)
            on_run_state_changed.assert_not_called()

            # A user edit arrives through the model and must notify.
            panel._end_time_field.float_field.model.set_value(2.0)
            on_run_state_changed.assert_called_once_with()
            self.assertAlmostEqual(panel._end_time_field.get_value(), 2.0)
        finally:
            panel.cleanup()
            window.destroy()

    async def test_bound_reclamp_notifies_end_edit_consumer(self) -> None:
        """Verify a bound change that moves the value reaches an end-edit-only owner."""
        on_value_changed = Mock()
        on_end_edit = Mock()
        fields = []
        window = await self._build_window(
            lambda: fields.append(
                ExactFloatField(
                    "Gain",
                    default_value=1.0,
                    lower_limit=0.0,
                    upper_limit=10.0,
                    on_value_changed_fn=on_value_changed,
                    on_end_edit_fn=on_end_edit,
                )
            )
        )
        try:
            field = fields[0]
            field.set_lower_limit(2.0)
            self.assertAlmostEqual(field.get_value(), 2.0)
            on_end_edit.assert_called_once_with(2.0)
            on_value_changed.assert_not_called()
        finally:
            window.destroy()

    async def test_simulation_panel_engine_round_trip(self) -> None:
        """Verify simulation panel engine round trip."""
        panel = SimulationPanel()
        window = await self._build_window(panel.build)
        try:
            self.assertEqual(panel.get_engine(), SIMULATION_ENGINE_ISAAC_SIM)
            self.assertFalse(panel.is_differentiable_engine())
            self.assertTrue(panel._clone_group.visible)
            panel.set_offline_stepping(False)
            self.assertFalse(panel.is_offline_stepping())

            panel.set_engine_selection(SIMULATION_ENGINE_NEWTON, NEWTON_SOLVER_FEATHERSTONE_DIFF)
            await omni.kit.app.get_app().next_update_async()
            self.assertEqual(panel.get_engine(), SIMULATION_ENGINE_NEWTON)
            self.assertTrue(panel.is_differentiable_engine())
            self.assertFalse(panel._clone_group.visible)
            self.assertTrue(panel._newton_compatibility_hint_label.visible)
            panel._controller_model.get_item_value_model().set_value(1)
            panel._effort_clamp_model.get_item_value_model().set_value(1)
            await omni.kit.app.get_app().next_update_async()
            newton_spec = panel.get_newton_spec()
            self.assertEqual(newton_spec.solver, NEWTON_SOLVER_FEATHERSTONE_DIFF)
            self.assertEqual(newton_spec.controller, "pd")
            self.assertEqual(newton_spec.effort_clamp, "max_effort")
            self.assertGreaterEqual(newton_spec.featherstone_substeps, 1)
            self.assertIsInstance(panel._featherstone_substeps_field, ExactIntField)
            self.assertIsInstance(panel._mujoco_substeps_field, ExactIntField)
            self.assertFalse(panel._mujoco_group.visible)

            panel.set_engine_selection(SIMULATION_ENGINE_NEWTON, NEWTON_SOLVER_MUJOCO)
            panel.set_controller_mode("pid")
            panel.set_effort_clamp_mode("dc_motor")
            panel.set_offline_stepping(False)
            panel.set_cuda_graph_capture(False)
            self.assertEqual(panel.get_newton_spec().mujoco_substeps, 1)
            panel._mujoco_substeps_field.set_value(8)
            await omni.kit.app.get_app().next_update_async()
            newton_spec = panel.get_newton_spec()
            self.assertTrue(panel._mujoco_group.visible)
            self.assertEqual(newton_spec.mujoco_substeps, 8)
            self.assertFalse(panel._newton_compatibility_hint_label.visible)
            self.assertEqual(newton_spec.controller, "pid")
            self.assertEqual(newton_spec.effort_clamp, "dc_motor")
            self.assertFalse(panel.is_offline_stepping())
            self.assertFalse(newton_spec.cuda_graph_capture)
        finally:
            panel.cleanup()
            window.destroy()

    async def test_simulation_panel_feedforward_round_trip(self) -> None:
        """Verify simulation panel feedforward round trip."""
        panel = SimulationPanel()
        window = await self._build_window(panel.build)
        try:
            self.assertEqual(panel.get_feedforward_mode(), "none")
            self.assertTrue(panel.get_newton_spec().cuda_graph_capture)

            panel.set_feedforward_mode("inverse_dynamics")
            await omni.kit.app.get_app().next_update_async()
            self.assertEqual(panel.get_feedforward_mode(), "inverse_dynamics")
            self.assertEqual(panel.get_newton_spec().feedforward, "inverse_dynamics")

            # Check must snapshot the current ComboBox selection, not a stale
            # callback cache. This reproduces selecting gravity and immediately
            # running Check before the cached label catches up.
            panel._feedforward_model.get_item_value_model().set_value(1)
            panel._feedforward_label = "none"
            self.assertEqual(panel.get_feedforward_mode(), "gravity")
            self.assertEqual(panel.get_newton_spec().feedforward, "gravity")

            panel.set_feedforward_mode("warp_drive")  # unknown values are ignored
            self.assertEqual(panel.get_feedforward_mode(), "gravity")

            panel.set_cuda_graph_capture(False)
            self.assertFalse(panel.get_newton_spec().cuda_graph_capture)
        finally:
            panel.cleanup()
            window.destroy()

    async def test_explicit_pd_actuator_and_delay_flags_round_trip(self) -> None:
        """Verify explicit pd actuator and delay flags round trip."""
        simulation = SimulationPanel()
        parameters = ParameterPanel()

        def _build() -> None:
            with ui.VStack():
                simulation.build()
                parameters.build()

        window = await self._build_window(_build)
        try:
            self.assertEqual(simulation.get_actuator_runtime(), ACTUATOR_RUNTIME_AUTO)
            simulation.set_actuator_runtime(ACTUATOR_RUNTIME_EXPLICIT)
            simulation.set_explicit_actuator_policy(EXPLICIT_ACTUATOR_POLICY_PROMOTE_SELECTED)
            parameters.set_advanced_flags({"include_command_delay": True})
            await omni.kit.app.get_app().next_update_async()

            self.assertEqual(simulation.get_actuator_runtime(), ACTUATOR_RUNTIME_EXPLICIT)
            self.assertTrue(parameters.get_advanced_flags()["include_command_delay"])

            simulation.set_engine_selection(SIMULATION_ENGINE_NEWTON, NEWTON_SOLVER_MUJOCO)
            await omni.kit.app.get_app().next_update_async()
            self.assertEqual(simulation.get_actuator_runtime(), ACTUATOR_RUNTIME_EXPLICIT)
            self.assertEqual(simulation.get_explicit_actuator_policy(), EXPLICIT_ACTUATOR_POLICY_PROMOTE_SELECTED)
            self.assertEqual(simulation.get_physics_backend(), PHYSICS_BACKEND_NEWTON)

            simulation.set_actuator_runtime(ACTUATOR_RUNTIME_MIXED)
            self.assertEqual(simulation.get_actuator_runtime(), ACTUATOR_RUNTIME_MIXED)

            simulation.set_engine_selection(SIMULATION_ENGINE_NEWTON, NEWTON_SOLVER_FEATHERSTONE_DIFF)
            simulation.set_actuator_runtime(ACTUATOR_RUNTIME_AUTO)
            self.assertEqual(simulation.get_actuator_runtime(), ACTUATOR_RUNTIME_AUTO)
        finally:
            simulation.cleanup()
            parameters.cleanup()
            window.destroy()
