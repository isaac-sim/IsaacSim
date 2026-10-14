# SPDX-FileCopyrightText: Copyright (c) 2024-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

"""UIBuilder test-run helpers: durations, overshoot, test modes, and dt bounds."""

from __future__ import annotations

import math
from types import SimpleNamespace
from unittest import mock

import numpy as np
import omni.kit.test
from isaacsim.robot_setup import gain_tuner
from isaacsim.robot_setup.gain_tuner.ui.backend_context import BackendContext
from isaacsim.robot_setup.gain_tuner.ui.gain_display import (
    UNLIMITED_FIELD_FORMAT,
    UNLIMITED_TEXT,
    gain_dof_unit,
    joint_param_info_text,
)
from isaacsim.robot_setup.gain_tuner.ui.gains_tuner_backend import GainsTestMode
from isaacsim.robot_setup.gain_tuner.ui.style import LABEL_COLOR
from isaacsim.robot_setup.gain_tuner.ui.test_table_widget import ColumnIndex, TestMode, default_test_column_id_map
from isaacsim.robot_setup.gain_tuner.ui.ui_builder import _TEST_MODE_COMBO_MODES, UIBuilder
from pxr import Usd, UsdPhysics


class TestComputeTestTotalDuration(omni.kit.test.AsyncTestCase):
    """Per-mode progress-bar denominator math (B)."""

    async def test_total_duration_per_mode(self) -> None:
        """Each mode uses the right denominator term (test / stress / hold estimate)."""
        # (mode, num_sequences, test_duration, expected).  SINUSOIDAL/STEP use
        # test_duration; STRESS uses stress_duration; SNAP uses 3 * (hold + 2).
        cases = [
            (GainsTestMode.SINUSOIDAL, 2, 5.0, 10.0),  # 2 * 5
            (GainsTestMode.STEP, 3, 4.0, 12.0),  # 3 * 4
            (GainsTestMode.STRESS_TEST, 2, 5.0, 20.0),  # 2 * stress_duration(10)
            (GainsTestMode.SNAP_TO_LIMITS, 2, 5.0, 18.0),  # 2 * 3 * (hold(1) + 2)
        ]
        for mode, num_sequences, test_duration, expected in cases:
            total = UIBuilder._compute_test_total_duration(
                mode, num_sequences=num_sequences, test_duration=test_duration, stress_duration=10.0, hold_duration=1.0
            )
            self.assertAlmostEqual(total, expected, msg=f"total for {mode}")

    async def test_discretization_total_duration(self) -> None:
        """dt sweep uses num_dt_steps * (hold + 0.5*timeout) and ignores num_sequences."""
        # 4 levels * (hold(1) + 0.5 * timeout(10)) = 4 * 6 = 24; sequence count ignored.
        total = UIBuilder._compute_test_total_duration(
            GainsTestMode.DISCRETIZATION,
            num_sequences=7,
            test_duration=5.0,
            stress_duration=10.0,
            hold_duration=1.0,
            num_dt_steps=4,
            timeout=10.0,
        )
        self.assertAlmostEqual(total, 24.0)

    async def test_discretization_total_duration_scales_with_timeout(self) -> None:
        """The dt-sweep denominator grows with the approach timeout (settle allowance)."""
        base = UIBuilder._compute_test_total_duration(
            GainsTestMode.DISCRETIZATION, 1, 0.0, 0.0, hold_duration=1.0, num_dt_steps=2, timeout=4.0
        )
        longer = UIBuilder._compute_test_total_duration(
            GainsTestMode.DISCRETIZATION, 1, 0.0, 0.0, hold_duration=1.0, num_dt_steps=2, timeout=20.0
        )
        # 2 * (1 + 0.5*4) = 6 ; 2 * (1 + 0.5*20) = 22.
        self.assertAlmostEqual(base, 6.0)
        self.assertAlmostEqual(longer, 22.0)


class TestProgressReadout(omni.kit.test.AsyncTestCase):
    """Progress-bar fraction and time readout, including estimate overrun."""

    async def test_progress_fraction_scales_with_elapsed(self) -> None:
        """The fraction tracks elapsed/total for a run inside its estimate."""
        self.assertAlmostEqual(UIBuilder._progress_fraction(0.0, 10.0), 0.0)
        self.assertAlmostEqual(UIBuilder._progress_fraction(2.5, 10.0), 0.25)
        self.assertAlmostEqual(UIBuilder._progress_fraction(9.0, 10.0), 0.9)

    async def test_progress_fraction_capped_below_full_while_running(self) -> None:
        """An overrunning run never reads 100%, so it is not mistaken for finished."""
        # Snap-to-limits with weak gains: every approach hits its 10 s timeout, so
        # the run outlasts the estimate several times over.
        for elapsed in (27.0, 45.0, 200.0):
            fraction = UIBuilder._progress_fraction(elapsed, 27.0)
            self.assertLess(fraction, 1.0, msg=f"elapsed={elapsed}")
            self.assertAlmostEqual(fraction, 0.99, msg=f"elapsed={elapsed}")

    async def test_progress_fraction_reaches_full_when_not_running(self) -> None:
        """Once the run reports done the fraction is allowed to reach 1.0."""
        self.assertAlmostEqual(UIBuilder._progress_fraction(45.0, 27.0, running=False), 1.0)
        self.assertAlmostEqual(UIBuilder._progress_fraction(27.0, 27.0, running=False), 1.0)

    async def test_progress_fraction_handles_nonpositive_total(self) -> None:
        """A zero or negative denominator yields zero rather than dividing by zero."""
        self.assertAlmostEqual(UIBuilder._progress_fraction(5.0, 0.0), 0.0)
        self.assertAlmostEqual(UIBuilder._progress_fraction(5.0, -1.0), 0.0)

    async def test_format_progress_time(self) -> None:
        """The readout switches to an estimate-relative form once elapsed exceeds it."""
        self.assertEqual(UIBuilder._format_progress_time(4.0, 10.0), "Time: 4.0s/10.0s")
        self.assertEqual(UIBuilder._format_progress_time(10.0, 10.0), "Time: 10.0s/10.0s")
        self.assertEqual(UIBuilder._format_progress_time(45.0, 27.0), "Time: 45.0s (est. 27.0s)")


class TestComputeOvershoot(omni.kit.test.AsyncTestCase):
    """Overshoot gating and computation (C)."""

    @staticmethod
    def _step_arrays() -> tuple[np.ndarray, np.ndarray]:
        """Step-like command 0 -> 10 with an observed response overshooting to 12."""
        cmd = np.concatenate([np.zeros(10), np.full(90, 10.0)])
        obs = np.concatenate([np.linspace(0.0, 12.0, 50), np.full(50, 10.0)])
        return cmd, obs

    async def test_step_overshoot_positive(self) -> None:
        """STEP with a step-like command + overshoot returns a positive percentage."""
        cmd, obs = self._step_arrays()
        overshoot = UIBuilder._compute_overshoot(cmd, obs, GainsTestMode.STEP)
        self.assertIsNotNone(overshoot)
        self.assertAlmostEqual(overshoot, 20.0, places=1)

    async def test_non_step_modes_return_none(self) -> None:
        """Only STEP reports overshoot; other modes are gated out even for step-like data.

        Snap-to-Limits reports its own develop-style per-limit metric set instead
        of overshoot, so it is gated out here alongside sinusoidal/stress.
        """
        cmd, obs = self._step_arrays()
        for mode in (GainsTestMode.SINUSOIDAL, GainsTestMode.STRESS_TEST, GainsTestMode.SNAP_TO_LIMITS):
            self.assertIsNone(UIBuilder._compute_overshoot(cmd, obs, mode), msg=f"overshoot gated for {mode}")

    async def test_step_non_step_command_returns_none(self) -> None:
        """STEP with a non-step-like (sinusoidal) command returns None (data heuristic)."""
        t = np.linspace(0.0, 4.0 * np.pi, 200)
        cmd = np.sin(t)
        obs = np.sin(t) * 1.2
        self.assertIsNone(UIBuilder._compute_overshoot(cmd, obs, GainsTestMode.STEP))

    async def test_descending_step_overshoot(self) -> None:
        """A descending step (10 -> 0) undershooting past 0 yields positive overshoot."""
        cmd = np.concatenate([np.full(10, 10.0), np.zeros(90)])
        obs = np.concatenate([np.linspace(10.0, -2.0, 50), np.zeros(50)])
        overshoot = UIBuilder._compute_overshoot(cmd, obs, GainsTestMode.STEP)
        self.assertIsNotNone(overshoot)
        self.assertAlmostEqual(overshoot, 20.0, places=1)

    async def test_empty_arrays_return_none(self) -> None:
        """Empty command/observed arrays return None (defensive)."""
        self.assertIsNone(UIBuilder._compute_overshoot(np.array([]), np.array([]), GainsTestMode.STEP))


class TestTestModeDisplayName(omni.kit.test.AsyncTestCase):
    """Test-mode display-name mapping and combo-index mapping (D)."""

    async def test_display_names(self) -> None:
        """Each mode maps to its combo label."""
        self.assertEqual(UIBuilder._test_mode_display_name(GainsTestMode.SNAP_TO_LIMITS), "Snap to Limits")
        self.assertEqual(UIBuilder._test_mode_display_name(GainsTestMode.SINUSOIDAL), "Sinusoidal")
        self.assertEqual(UIBuilder._test_mode_display_name(GainsTestMode.STEP), "Step Function")
        self.assertEqual(UIBuilder._test_mode_display_name(GainsTestMode.STRESS_TEST), "Stress")
        self.assertEqual(UIBuilder._test_mode_display_name(GainsTestMode.DISCRETIZATION), "dt Sweep")

    async def test_display_name_none_for_unset(self) -> None:
        """An unset (None) mode has no display name."""
        self.assertIsNone(UIBuilder._test_mode_display_name(None))

    async def test_combo_index_mapping(self) -> None:
        """Combo order matches ['Snap to Limits', 'Sinusoidal', 'Step Function', 'Stress', 'dt Sweep']."""
        self.assertEqual(
            list(_TEST_MODE_COMBO_MODES),
            [
                GainsTestMode.SNAP_TO_LIMITS,
                GainsTestMode.SINUSOIDAL,
                GainsTestMode.STEP,
                GainsTestMode.STRESS_TEST,
                GainsTestMode.DISCRETIZATION,
            ],
        )
        # The combo labels these indices correspond to round-trip through the
        # display-name mapping, confirming index -> mode -> label consistency.
        labels = [UIBuilder._test_mode_display_name(m) for m in _TEST_MODE_COMBO_MODES]
        self.assertEqual(labels, ["Snap to Limits", "Sinusoidal", "Step Function", "Stress", "dt Sweep"])


class TestTestTableColumnMap(omni.kit.test.AsyncTestCase):
    """Test-table column layout gating (populates the joint table per mode)."""

    async def test_every_selectable_mode_has_columns(self) -> None:
        """Every combo-selectable mode has a column layout so its joint table populates.

        The Test Gains table indexes ``column_id_map`` by the active mode when it
        builds its tree; a missing mode raises KeyError inside the frame build and
        the joint table renders empty (the dt Sweep population bug).
        """
        column_map = default_test_column_id_map()
        for mode in _TEST_MODE_COMBO_MODES:
            self.assertIn(mode, column_map, msg=f"no Test-table columns registered for {mode}")
            # JOINT / TEST columns are what render the selectable joint rows.
            self.assertIn(ColumnIndex.JOINT, column_map[mode])
            self.assertIn(ColumnIndex.TEST, column_map[mode])

    async def test_dt_sweep_matches_snap_layout(self) -> None:
        """dt Sweep uses the same minimal Joint / Test / Sequence layout as snap/stress."""
        column_map = default_test_column_id_map()
        self.assertEqual(column_map[TestMode.DISCRETIZATION], column_map[TestMode.SNAP_TO_LIMITS])


class TestDeferredTestStart(omni.kit.test.AsyncTestCase):
    """The deferred test-start guard rejects starts a cancel/reset has invalidated."""

    async def test_proceeds_when_generation_unchanged_and_running(self) -> None:
        """A start whose run generation is unchanged and still running proceeds."""
        self.assertTrue(UIBuilder._should_start_deferred_test(3, 3, is_running=True))

    async def test_aborts_when_generation_bumped(self) -> None:
        """A cancel/reset bumps the generation, so the stale deferred start aborts."""
        self.assertFalse(UIBuilder._should_start_deferred_test(3, 4, is_running=True))

    async def test_aborts_when_not_running(self) -> None:
        """A start whose running flag was cleared (cancelled) does not proceed."""
        self.assertFalse(UIBuilder._should_start_deferred_test(3, 3, is_running=False))


class TestValidateDtSweepParams(omni.kit.test.AsyncTestCase):
    """The UI dt-sweep parameter guard rejects reversed / invalid bounds before running."""

    @staticmethod
    def _params(dt_max: float, dt_min: float, num_steps: int = 5, target_dt: float = 0.008) -> dict:
        return {"dt_max": dt_max, "dt_min": dt_min, "num_dt_steps": num_steps, "target_dt": target_dt}

    async def test_valid_bounds_accepted(self) -> None:
        """Well-ordered positive bounds and target dt return no error."""
        self.assertIsNone(UIBuilder._validate_dt_sweep_params(self._params(0.02, 0.001)))

    async def test_reversed_bounds_rejected(self) -> None:
        """Reversed bounds (dt_max < dt_min) produce a user-facing error message."""
        self.assertIsNotNone(UIBuilder._validate_dt_sweep_params(self._params(0.001, 0.02)))

    async def test_equal_bounds_rejected(self) -> None:
        """Equal bounds are degenerate and rejected."""
        self.assertIsNotNone(UIBuilder._validate_dt_sweep_params(self._params(0.01, 0.01)))

    async def test_non_positive_and_non_finite_rejected(self) -> None:
        """Non-positive or non-finite bounds / target are rejected."""
        self.assertIsNotNone(UIBuilder._validate_dt_sweep_params(self._params(0.02, 0.0)))
        self.assertIsNotNone(UIBuilder._validate_dt_sweep_params(self._params(float("inf"), 0.001)))
        self.assertIsNotNone(UIBuilder._validate_dt_sweep_params(self._params(0.02, 0.001, target_dt=0.0)))
        self.assertIsNotNone(UIBuilder._validate_dt_sweep_params(self._params(0.02, 0.001, target_dt=float("nan"))))


class TestTunableJointFilter(omni.kit.test.AsyncTestCase):
    """The UI joint-list filter excludes mimic joints while keeping normal ones."""

    @staticmethod
    def _apply_mimic_schema(prim, token: str) -> None:
        """Author ``token`` into the prim's apiSchemas so it reads as a mimic joint.

        Uses ``AddAppliedSchema`` (like the core mimic-detection tests) so the
        token is authored without requiring the PhysX/Newton schema plugin.
        """
        prim.AddAppliedSchema(token)

    def _builder_with_entries(self, entries):
        """A bare UIBuilder whose gains tuner yields ``entries`` (no full init)."""
        builder = UIBuilder.__new__(UIBuilder)
        builder._gains_tuner = SimpleNamespace(get_joint_entries=lambda: list(entries))
        return builder

    async def test_mimic_joint_dropped_normal_kept(self) -> None:
        """A mimic joint is filtered out; the normal joint remains tunable."""
        stage = Usd.Stage.CreateInMemory()
        UsdPhysics.RevoluteJoint.Define(stage, "/normal_joint")
        UsdPhysics.RevoluteJoint.Define(stage, "/mimic_joint")
        normal = stage.GetPrimAtPath("/normal_joint")
        mimic = stage.GetPrimAtPath("/mimic_joint")
        self._apply_mimic_schema(mimic, "PhysxMimicJointAPI:rotZ")

        entries = [
            SimpleNamespace(joint=normal, dof_index=0, display_name="normal_joint"),
            SimpleNamespace(joint=mimic, dof_index=1, display_name="mimic_joint"),
        ]
        builder = self._builder_with_entries(entries)
        tunable = builder._get_tunable_joint_entries()
        self.assertEqual([e.joint for e in tunable], [normal])

    async def test_newton_mimic_joint_dropped(self) -> None:
        """A Newton (single-apply) mimic joint is also excluded."""
        stage = Usd.Stage.CreateInMemory()
        UsdPhysics.RevoluteJoint.Define(stage, "/normal_joint")
        UsdPhysics.RevoluteJoint.Define(stage, "/mimic_joint")
        normal = stage.GetPrimAtPath("/normal_joint")
        mimic = stage.GetPrimAtPath("/mimic_joint")
        self._apply_mimic_schema(mimic, "NewtonMimicAPI")

        entries = [
            SimpleNamespace(joint=normal, dof_index=0, display_name="normal_joint"),
            SimpleNamespace(joint=mimic, dof_index=1, display_name="mimic_joint"),
        ]
        builder = self._builder_with_entries(entries)
        tunable = builder._get_tunable_joint_entries()
        self.assertEqual([e.joint for e in tunable], [normal])

    async def test_only_mimic_yields_empty_list(self) -> None:
        """A robot whose only tunable joints are mimic yields an empty list (no error)."""
        stage = Usd.Stage.CreateInMemory()
        UsdPhysics.RevoluteJoint.Define(stage, "/mimic_joint")
        mimic = stage.GetPrimAtPath("/mimic_joint")
        self._apply_mimic_schema(mimic, "PhysxMimicJointAPI:rotZ")

        entries = [SimpleNamespace(joint=mimic, dof_index=0, display_name="mimic_joint")]
        builder = self._builder_with_entries(entries)
        self.assertEqual(builder._get_tunable_joint_entries(), [])


class TestStiffnessUnitHint(omni.kit.test.AsyncTestCase):
    """Backend-dependent stiffness/damping unit shown in the Kp/Kd headers."""

    @staticmethod
    def _row(is_angular: bool):
        return SimpleNamespace(is_angular=is_angular)

    async def test_all_angular_uses_backend_angle_unit(self) -> None:
        """All-angular joints show the backend angle unit (deg for PhysX, rad for Newton)."""
        builder = UIBuilder.__new__(UIBuilder)
        rows = [self._row(True), self._row(True)]
        self.assertEqual(builder._stiffness_unit_hint(rows, "PhysX"), gain_dof_unit(True, "PhysX"))
        self.assertEqual(builder._stiffness_unit_hint(rows, "NewtonAPI"), gain_dof_unit(True, "NewtonAPI"))

    async def test_all_prismatic_uses_linear_unit(self) -> None:
        """All-prismatic joints show the linear unit (backend-independent)."""
        builder = UIBuilder.__new__(UIBuilder)
        rows = [self._row(False)]
        self.assertEqual(builder._stiffness_unit_hint(rows, "PhysX"), gain_dof_unit(False, "PhysX"))

    async def test_mixed_dofs_report_mixed(self) -> None:
        """A selection mixing angular + linear DOFs reports 'mixed'."""
        builder = UIBuilder.__new__(UIBuilder)
        rows = [self._row(True), self._row(False)]
        self.assertEqual(builder._stiffness_unit_hint(rows, "PhysX"), "mixed")

    async def test_empty_defaults_to_angular_unit(self) -> None:
        """With no rows the hint defaults to the backend angle unit (no crash)."""
        builder = UIBuilder.__new__(UIBuilder)
        self.assertEqual(builder._stiffness_unit_hint([], "PhysX"), gain_dof_unit(True, "PhysX"))


class TestRunTestParamHelpers(omni.kit.test.AsyncTestCase):
    """Value-field read helpers + sequence-partition helper shared by ``_on_run_gains_test`` (pure)."""

    async def test_read_helpers_use_default_when_field_absent(self) -> None:
        """A missing (None) field falls back to the supplied default for each type."""
        self.assertEqual(UIBuilder._read_float(None, 5.0), 5.0)
        self.assertEqual(UIBuilder._read_int(None, 7), 7)
        self.assertFalse(UIBuilder._read_bool(None, False))

    async def test_read_helpers_return_field_value(self) -> None:
        """When present, each helper reads the field's model value of the matching type."""
        ffield = SimpleNamespace(model=SimpleNamespace(get_value_as_float=lambda: 3.5))
        ifield = SimpleNamespace(model=SimpleNamespace(get_value_as_int=lambda: 9))
        bfield = SimpleNamespace(model=SimpleNamespace(get_value_as_bool=lambda: True))
        self.assertEqual(UIBuilder._read_float(ffield, 0.0), 3.5)
        self.assertEqual(UIBuilder._read_int(ifield, 0), 9)
        self.assertTrue(UIBuilder._read_bool(bfield, False))

    async def test_joint_index_sequences_partitions_by_sequence(self) -> None:
        """Each sequence id yields a dict with the int32 joint-index array for that sequence."""
        params = [
            SimpleNamespace(joint_index=0, sequence=0),
            SimpleNamespace(joint_index=1, sequence=1),
            SimpleNamespace(joint_index=2, sequence=0),
        ]
        seqs = UIBuilder._joint_index_sequences(params, [0, 1])
        self.assertEqual(len(seqs), 2)
        np.testing.assert_array_equal(seqs[0]["joint_indices"], np.array([0, 2], dtype=np.int32))
        np.testing.assert_array_equal(seqs[1]["joint_indices"], np.array([1], dtype=np.int32))
        self.assertEqual(seqs[0]["joint_indices"].dtype, np.int32)


class TestUIJointDriveMode(omni.kit.test.AsyncTestCase):
    """The joint-drive-mode filter used by the UI enumerates every drive mode."""

    async def test_mimic_present_in_iteration(self) -> None:
        """MIMIC is enumerated so mimic joints are handled distinctly from NONE."""
        self.assertIn(gain_tuner.JointDriveMode.MIMIC, list(gain_tuner.JointDriveMode))


class _EffortTensor:
    """Minimal stand-in for the tensor object returned by the articulation APIs."""

    def __init__(self, arr) -> None:
        self._arr = np.asarray(arr, dtype=float)

    def numpy(self):
        return self._arr


class _FakeArticulation:
    """Fake articulation whose measured-force API can succeed, raise, or count calls."""

    def __init__(self, *, projected_error: Exception | None = None) -> None:
        self._projected_error = projected_error
        self.projected_calls = 0

    def get_dof_projected_joint_forces(self):
        self.projected_calls += 1
        if self._projected_error is not None:
            raise self._projected_error
        return _EffortTensor([[10.0, 20.0]])


class TestCaptureStepEffort(omni.kit.test.AsyncTestCase):
    """`_capture_step_effort` records measured forces on capable backends and skips
    backends without a measured-force API (Newton) without flooding the log."""

    @staticmethod
    def _builder(art: _FakeArticulation | None, backend: str) -> UIBuilder:
        builder = UIBuilder.__new__(UIBuilder)
        builder._gains_tuner = SimpleNamespace(get_articulation=lambda: art)
        builder._backend_ctx = SimpleNamespace(backend=backend)
        builder._projected_forces_unavailable = False
        builder._test_effort_history = []
        builder._test_effort_times = []
        builder._test_elapsed_sim = 0.5
        return builder

    async def test_measured_forces_available_by_backend(self) -> None:
        """Measured forces are reported available for PhysX and unavailable for Newton."""
        self.assertTrue(self._builder(None, "PhysX")._measured_forces_available())
        self.assertFalse(self._builder(None, "NewtonAPI")._measured_forces_available())

    async def test_physx_records_measured_force(self) -> None:
        """On a backend with measured forces, the projected joint force is recorded."""
        art = _FakeArticulation(projected_error=None)
        builder = self._builder(art, "PhysX")

        builder._capture_step_effort()

        self.assertEqual(art.projected_calls, 1)
        np.testing.assert_array_equal(builder._test_effort_history[0], [10.0, 20.0])
        self.assertEqual(builder._test_effort_times, [0.5])

    async def test_newton_skips_capture_entirely(self) -> None:
        """Newton is skipped up front: the measured-force API is never called; nothing recorded."""
        art = _FakeArticulation(projected_error=Exception("must not be called on Newton"))
        builder = self._builder(art, "NewtonAPI")

        builder._capture_step_effort()
        builder._capture_step_effort()

        self.assertEqual(art.projected_calls, 0)
        self.assertEqual(builder._test_effort_history, [])
        self.assertEqual(builder._test_effort_times, [])

    async def test_backend_gap_probes_once_without_flooding(self) -> None:
        """A 'capable' backend that still raises is probed once, then stops (no per-step flood)."""
        art = _FakeArticulation(projected_error=Exception("Failed to get dof projected joint forces from backend"))
        builder = self._builder(art, "PhysX")

        builder._capture_step_effort()
        builder._test_elapsed_sim = 1.0
        builder._capture_step_effort()

        self.assertTrue(builder._projected_forces_unavailable)
        self.assertEqual(art.projected_calls, 1, "projected forces must be probed at most once per run")
        self.assertEqual(builder._test_effort_history, [])
        self.assertEqual(builder._test_effort_times, [])

    async def test_none_articulation_does_not_trip_unavailable_flag(self) -> None:
        """A ``None`` articulation is nothing-to-capture, not a backend gap.

        ``get_articulation()`` legitimately returns ``None`` before setup and
        during teardown.  That must not permanently set
        ``_projected_forces_unavailable`` or emit the misleading
        backend-unavailable warning; otherwise no effort is captured for the rest
        of the run/sweep even once the articulation becomes valid again.
        """
        builder = self._builder(None, "PhysX")
        self.assertTrue(builder._measured_forces_available())

        with mock.patch("carb.log_warn") as log_warn:
            builder._capture_step_effort()

        self.assertFalse(builder._projected_forces_unavailable)
        self.assertEqual(builder._test_effort_history, [])
        self.assertEqual(builder._test_effort_times, [])
        log_warn.assert_not_called()


class TestAdvancedParamMessages(omni.kit.test.AsyncTestCase):
    """The inline lines under the Advanced Actuator Parameters fields."""

    def _joint(self):
        self._stage = Usd.Stage.CreateInMemory()
        return UsdPhysics.RevoluteJoint.Define(self._stage, "/Robot/joint").GetPrim()

    def _builder(self, engine_value, backend="PhysX", solver="", playing=True, armature=None) -> UIBuilder:
        builder = UIBuilder.__new__(UIBuilder)
        builder._backend_ctx = SimpleNamespace(backend=backend, solver_type=solver)
        builder._gains_tuner = SimpleNamespace(
            get_dof_effective_max_velocity=lambda _dof: engine_value,
            get_dof_engine_armature=lambda _dof: armature,
        )
        # The engine-truth queries are gated on a playing timeline: the accessors
        # fall back to reading USD without a physics view, and USD is what the
        # comparison is against.
        builder._timeline = SimpleNamespace(is_playing=lambda: playing)
        return builder

    async def test_active_backend_is_read_live_not_from_the_cached_context(self) -> None:
        """The active backend must come from the engine, not from a cache.

        The engine can change with no event to subscribe to, so a context filled at
        stage load goes stale silently.  Reading it live is what stops an edit from
        landing on a schema the running engine does not read.
        """
        builder = self._builder(None, "PhysX")
        with mock.patch.object(BackendContext, "active_backend_label", return_value="NewtonAPI"):
            self.assertEqual(builder._active_backend(), "NewtonAPI")
        with mock.patch.object(BackendContext, "active_backend_label", return_value="PhysX"):
            self.assertEqual(builder._active_backend(), "PhysX")

    async def test_active_backend_falls_back_to_the_cache_when_the_query_fails(self) -> None:
        """A broken engine query must not make the panel unusable."""
        builder = self._builder(None, "NewtonAPI")
        with mock.patch.object(BackendContext, "active_backend_label", side_effect=AttributeError):
            self.assertEqual(builder._active_backend(), "NewtonAPI")

    async def test_active_solver_selects_the_resolver_chain(self) -> None:
        """The solver is load-bearing now, so the panel must read it too."""
        self.assertEqual(self._builder(None, "NewtonAPI", "mujoco")._active_solver(), "mujoco")
        self.assertEqual(self._builder(None, "NewtonAPI")._active_solver(), "")

    async def test_resolution_lookup_reports_the_active_backend_value(self) -> None:
        """The panel resolves through the active backend, not a fixed schema order."""
        joint = self._joint()
        spec = gain_tuner.joint_param_spec("armature")
        joint.ApplyAPI(gain_tuner.NEWTON_JOINT_API)
        joint.ApplyAPI(gain_tuner.PHYSX_JOINT_API)
        joint.GetAttribute(spec.newton_attr).Set(0.25)
        joint.GetAttribute(spec.physx_attr).Set(0.75)

        physx = UIBuilder._joint_param_resolution(joint, spec, "PhysX", "")
        newton = UIBuilder._joint_param_resolution(joint, spec, "NewtonAPI", "xpbd")
        self.assertAlmostEqual(physx.effective_value, 0.75, places=5)
        self.assertAlmostEqual(newton.effective_value, 0.25, places=5)

    async def test_resolution_lookup_survives_a_non_usd_joint(self) -> None:
        """An unreadable joint yields no line rather than breaking the panel."""
        spec = gain_tuner.joint_param_spec("armature")
        resolution = UIBuilder._joint_param_resolution(object(), spec, "PhysX", "")
        # Nothing is readable off a non-USD object, so every schema reads as
        # unauthored and the panel prints no line -- it does not raise, and it does
        # not invent a value.
        self.assertFalse(resolution.any_authored)
        self.assertIsNone(resolution.effective_value)
        self.assertEqual(joint_param_info_text(resolution), "")
        self.assertIsNone(UIBuilder._joint_param_attr(object(), spec, None))
        self.assertIsNone(UIBuilder._joint_param_attr(object(), spec, resolution))
        # A resolution the panel could not build at all renders a blank, uneditable
        # field rather than a zero it would then let the user author.
        blank = UIBuilder._joint_param_display(object(), spec, None)
        self.assertTrue(blank.blank)
        self.assertFalse(blank.editable)

    async def test_cell_binds_to_whichever_schema_half_exists(self) -> None:
        """Editability follows the parameter applying to the joint, not the winner."""
        joint = self._joint()
        spec = gain_tuner.joint_param_spec("armature")
        joint.ApplyAPI(gain_tuner.NEWTON_JOINT_API)
        joint.GetAttribute(spec.newton_attr).Set(0.25)

        resolution = UIBuilder._joint_param_resolution(joint, spec, "PhysX", "")
        display = UIBuilder._joint_param_display(joint, spec, resolution)
        # PhysX reads nothing here, so the field shows PhysX's own default rather
        # than the Newton value it does not consume -- and marks it as a default.
        self.assertFalse(display.authored)
        self.assertEqual(display.value, 0.0)
        self.assertIn("PhysX", display.note)
        self.assertIn("default", display.field_format)
        self.assertIsNotNone(UIBuilder._joint_param_attr(joint, spec, resolution))

    async def test_unauthored_newton_armature_shows_newtons_own_default(self) -> None:
        """The primary blocker: an unauthored Newton armature is 0.1, not 0.

        Newton copies ``NewtonPhysicsConfig.armature`` onto its ``ModelBuilder``
        default, so a joint that authors none is still simulated with rotor inertia.
        Showing 0 states a value no engine uses, and the field is live enough to
        author that fiction with one drag.
        """
        joint = self._joint()
        spec = gain_tuner.joint_param_spec("armature")
        joint.ApplyAPI(gain_tuner.NEWTON_JOINT_API)

        resolution = UIBuilder._joint_param_resolution(joint, spec, "NewtonAPI", "xpbd")
        display = UIBuilder._joint_param_display(joint, spec, resolution)
        self.assertFalse(display.authored)
        self.assertAlmostEqual(display.value, gain_tuner.NEWTON_DEFAULT_ARMATURE, places=6)
        # The engine is named in the note (the cell's tooltip), not in the cell: the
        # field has room for the number and "(default)" and nothing more.
        self.assertIn("Newton", display.note)
        self.assertIn("0.1", display.note)
        self.assertIn("default", display.field_format)
        # Still writable: a marked default is not a read-only field.
        self.assertTrue(display.editable)
        self.assertFalse(display.blank)

    async def test_measured_engine_armature_wins_over_the_documented_default(self) -> None:
        """A running engine can be asked, and what it reports beats an assumption."""
        joint = self._joint()
        spec = gain_tuner.joint_param_spec("armature")
        joint.ApplyAPI(gain_tuner.NEWTON_JOINT_API)

        resolution = UIBuilder._joint_param_resolution(joint, spec, "NewtonAPI", "xpbd")
        display = UIBuilder._joint_param_display(joint, spec, resolution, engine_value=0.42)
        self.assertAlmostEqual(display.value, 0.42, places=6)
        self.assertIn("simulating", display.note)

    async def test_unauthored_velocity_limit_reads_as_unlimited_not_zero(self) -> None:
        """Zero would say "cannot move" where the truth is no limit at all."""
        joint = self._joint()
        spec = gain_tuner.joint_param_spec("max_joint_velocity")
        joint.ApplyAPI(gain_tuner.PHYSX_JOINT_API)

        resolution = UIBuilder._joint_param_resolution(joint, spec, "PhysX", "")
        display = UIBuilder._joint_param_display(joint, spec, resolution)
        self.assertTrue(display.unlimited)
        self.assertIsNone(display.value)
        self.assertIn(UNLIMITED_FIELD_FORMAT, display.field_format)
        self.assertTrue(display.editable)
        self.assertFalse(display.blank)

    async def test_engine_param_value_only_asks_a_running_engine(self) -> None:
        """Before play the accessor reads USD, so its answer is not a measurement."""
        entry = SimpleNamespace(joint=self._joint(), drive_axis="angular", dof_index=0)
        spec = gain_tuner.joint_param_spec("armature")
        self.assertAlmostEqual(self._builder(None, armature=0.3)._engine_param_value(entry, spec), 0.3, places=6)
        self.assertIsNone(self._builder(None, armature=0.3, playing=False)._engine_param_value(entry, spec))

    async def test_engine_param_value_has_nothing_to_offer_for_friction(self) -> None:
        """Newton's friction tensor accessor is a stub, so no measurement exists."""
        entry = SimpleNamespace(joint=self._joint(), drive_axis="angular", dof_index=0)
        spec = gain_tuner.joint_param_spec("joint_friction")
        self.assertIsNone(self._builder(None, armature=0.3)._engine_param_value(entry, spec))

    async def test_an_edit_writes_only_the_active_backend_schema(self) -> None:
        """The panel's write path leaves the other backend's value untouched."""
        joint = self._joint()
        spec = gain_tuner.joint_param_spec("armature")
        joint.ApplyAPI(gain_tuner.NEWTON_JOINT_API)
        joint.GetAttribute(spec.newton_attr).Set(0.25)

        with mock.patch("carb.log_warn") as log_warn:
            UIBuilder._author_joint_param(joint, spec, 0.5, "PhysX")
            log_warn.assert_not_called()

        self.assertAlmostEqual(joint.GetAttribute(spec.physx_attr).Get(), 0.5, places=5)
        self.assertAlmostEqual(joint.GetAttribute(spec.newton_attr).Get(), 0.25, places=5)

    async def test_an_edit_after_an_engine_switch_writes_the_new_backend(self) -> None:
        """The silent one: a write must never land on the schema the engine dropped.

        The cached context is filled at stage load and there is no engine-switch
        event, so after a switch it still names the previous engine.  Authoring off
        that cache would write ``physxJoint:*`` while Newton is running, and Newton
        reads no PhysX friction at all -- the user would tune a value that does
        nothing, with no error anywhere.
        """
        joint = self._joint()
        spec = gain_tuner.joint_param_spec("joint_friction")
        joint.ApplyAPI(gain_tuner.PHYSX_JOINT_API)
        joint.GetAttribute(spec.physx_attr).Set(0.75)

        # Cache still says PhysX; the engine has already moved to Newton.
        builder = self._builder(None, "PhysX")
        builder._gains_tuner = None
        builder._charts_frame = None
        builder._make_plot_on_next_frame = False
        # A successful write re-renders the registered advanced fields; this bare
        # builder has none registered, which is the state before any are built.
        builder._detail_adv_models = []
        builder._refresh_backend_from_app = lambda: setattr(
            builder, "_backend_ctx", SimpleNamespace(backend="NewtonAPI", solver_type="xpbd")
        )
        builder._resolve_save_targets = lambda: None

        with mock.patch.object(BackendContext, "active_backend_label", return_value="NewtonAPI"):
            builder._author_joint_param_live(joint, spec, 0.25)

        self.assertAlmostEqual(joint.GetAttribute(spec.newton_attr).Get(), 0.25, places=5)
        # The PhysX opinion the previous engine used is left exactly as it was.
        self.assertAlmostEqual(joint.GetAttribute(spec.physx_attr).Get(), 0.75, places=5)
        # The switch was noticed, so the rest of the panel is not left stale either.
        self.assertEqual(builder._backend_ctx.backend, "NewtonAPI")

    async def test_engine_switch_detection_is_a_comparison_and_fires_once(self) -> None:
        """Detection runs per frame, so it must be cheap and must not re-fire."""
        builder = self._builder(None, "PhysX")
        handled = []
        builder._on_engine_changed = handled.append

        with mock.patch.object(BackendContext, "active_backend_label", return_value="PhysX"):
            self.assertFalse(builder._sync_backend_if_engine_changed())
        self.assertEqual(handled, [])

        with mock.patch.object(BackendContext, "active_backend_label", return_value="NewtonAPI"):
            self.assertTrue(builder._sync_backend_if_engine_changed())
            self.assertEqual(handled, ["NewtonAPI"])
            # The stub handler did not update the cache, but a real refresh does;
            # once the cache agrees, a steady engine reports no further change.
            builder._backend_ctx = SimpleNamespace(backend="NewtonAPI", solver_type="xpbd")
            self.assertFalse(builder._sync_backend_if_engine_changed())
        self.assertEqual(handled, ["NewtonAPI"])

    async def test_an_unreadable_engine_is_not_treated_as_an_engine_change(self) -> None:
        """A failed query costs the user their recorded results, or did.

        ``active_backend_label`` reports "" when the simulation manager cannot be
        asked, rather than guessing PhysX -- correct, and it made a transient
        failure indistinguishable from a switch: "" against a cached "PhysX" fired
        :meth:`_on_engine_changed`, which logs a switch to an unnamed engine,
        re-acquires the articulation and **discards the recorded test results**.
        Then the query recovered and it happened again, in the other direction.

        So the panel adopts the label -- the header must not keep naming an engine
        that is no longer the answer -- and stops there.  It still settles after one
        pass, which was the only property the old test checked.
        """
        builder = self._builder(None, "PhysX")
        handled = []
        builder._on_engine_changed = handled.append
        builder._refresh_backend_from_app = lambda: setattr(builder, "_backend_ctx", BackendContext(backend=""))

        with mock.patch.object(BackendContext, "active_backend_label", return_value=""):
            self.assertFalse(builder._sync_backend_if_engine_changed(), "an unread engine is not a changed engine")
            # The label was adopted, so nothing downstream still claims PhysX.
            self.assertEqual(builder._backend_ctx.backend, "")
            self.assertFalse(builder._sync_backend_if_engine_changed(), "and it settles")
        self.assertEqual(handled, [], "nothing destructive ran")

    async def test_recovering_from_an_unreadable_engine_is_not_a_change_either(self) -> None:
        """The other half of the same transient: "" back to a real engine.

        Gating on one label would have left the recovery firing the destructive
        handler, so the results survived the failure and died on the recovery.
        """
        builder = self._builder(None, "")
        builder._backend_ctx = BackendContext(backend="")
        handled = []
        builder._on_engine_changed = handled.append
        builder._refresh_backend_from_app = lambda: setattr(builder, "_backend_ctx", BackendContext(backend="PhysX"))

        with mock.patch.object(BackendContext, "active_backend_label", return_value="PhysX"):
            self.assertFalse(builder._sync_backend_if_engine_changed())
            self.assertEqual(builder._backend_ctx.backend, "PhysX")
        self.assertEqual(handled, [])

    async def test_a_real_switch_still_fires_through_an_unsupported_engine(self) -> None:
        """Gating on non-empty labels is not gating on *supported* ones.

        An engine the panel cannot author for is still an engine the user switched
        to: the articulation behind the old views is just as invalid, and the
        recorded traces just as unmixable.
        """
        builder = self._builder(None, "PhysX")
        handled = []
        builder._on_engine_changed = handled.append

        with mock.patch.object(BackendContext, "active_backend_label", return_value="remotesim"):
            self.assertTrue(builder._sync_backend_if_engine_changed())
        self.assertEqual(handled, ["remotesim"])

    async def test_engine_switch_drops_the_invalidated_articulation(self) -> None:
        """A switch invalidates the physics views behind a still-non-None handle."""
        calls = []
        tuner = SimpleNamespace(
            _robot_prim_path="/World/Robot",
            invalidate_physics_views=lambda: calls.append("invalidated"),
        )
        builder = self._builder(None, "PhysX")
        builder._gains_tuner = tuner
        builder._make_plot_on_next_frame = True
        builder._awaiting_articulation_dofs = False
        builder._on_articulation_selection = lambda path, force=False: calls.append(("rebound", path, force))

        builder._reacquire_articulation_for_engine_switch()

        self.assertEqual(calls, ["invalidated", ("rebound", "/World/Robot", True)])
        # Recorded traces belong to the solver that produced them, so nothing is
        # replotted across the switch.
        self.assertFalse(builder._make_plot_on_next_frame)

    async def test_a_failed_write_is_warned_about(self) -> None:
        """A write that cannot land must not look like a silent no-op."""
        spec = gain_tuner.joint_param_spec("armature")
        with mock.patch("carb.log_warn") as log_warn:
            self.assertFalse(UIBuilder._author_joint_param(object(), spec, 0.5, "PhysX"))
            log_warn.assert_called_once()
            self.assertIn(spec.physx_attr, log_warn.call_args.args[0])

    async def test_a_failed_write_reverts_the_field_and_toasts(self) -> None:
        """The field must not keep a number the stage never took.

        Copy already toasts what it could not write.  A direct edit that only
        logged left the panel as the sole place the value existed, which is the
        silent no-op this MR exists to remove.
        """
        builder = self._builder(None, "PhysX")
        builder._suspend_detail_writes = False
        model = SimpleNamespace(values=[], set_value=lambda v: model.values.append(v))
        builder._sync_backend_if_engine_changed = lambda: False
        spec = gain_tuner.joint_param_spec("armature")

        with mock.patch.object(BackendContext, "active_backend_label", return_value="PhysX"):
            with mock.patch("carb.log_warn"):
                with mock.patch("omni.kit.notification_manager.post_notification") as toast:
                    builder._author_joint_param_live(object(), spec, 5.0, model, previous=0.1)

        toast.assert_called_once()
        self.assertEqual(model.values, [0.1])
        # The revert is itself a value change; the write-back it would trigger is
        # suppressed, and the flag is left as it was found.
        self.assertFalse(builder._suspend_detail_writes)

    async def test_a_successful_write_neither_reverts_nor_toasts(self) -> None:
        """The normal path stays silent."""
        joint = self._joint()
        spec = gain_tuner.joint_param_spec("armature")
        joint.ApplyAPI(gain_tuner.PHYSX_JOINT_API)
        builder = self._builder(None, "PhysX")
        builder._suspend_detail_writes = False
        builder._sync_backend_if_engine_changed = lambda: False
        builder._detail_adv_models = []
        model = SimpleNamespace(values=[], set_value=lambda v: model.values.append(v))

        with mock.patch.object(BackendContext, "active_backend_label", return_value="PhysX"):
            with mock.patch("omni.kit.notification_manager.post_notification") as toast:
                builder._author_joint_param_live(joint, spec, 0.5, model, previous=0.0)

        toast.assert_not_called()
        self.assertEqual(model.values, [])
        self.assertAlmostEqual(joint.GetAttribute(spec.physx_attr).Get(), 0.5, places=5)

    async def test_engine_velocity_warning_when_the_sweep_uses_another_limit(self) -> None:
        """The panel says so when the engine enforces a limit USD does not author."""
        entry = SimpleNamespace(joint=self._joint(), drive_axis="angular", dof_index=0)
        text = self._builder(90.0)._max_velocity_engine_warning(entry, 180.0)
        self.assertIn("enforces 90", text)

    async def test_engine_velocity_warning_when_usd_says_unlimited(self) -> None:
        """The case the safeguard used to miss.

        An unauthored velocity limit reads as unlimited, and if the engine is in
        fact clamping the joint that is exactly what needs saying -- it was
        reported as agreement before, which disabled the one check that could have
        caught the wrong number.
        """
        entry = SimpleNamespace(joint=self._joint(), drive_axis="angular", dof_index=0)
        text = self._builder(107.0)._max_velocity_engine_warning(entry, math.inf)
        self.assertIn("enforces 107", text)
        self.assertIn(UNLIMITED_TEXT, text)

    async def test_no_warning_when_both_sides_are_unlimited(self) -> None:
        """Unlimited on both sides is agreement, not a mismatch to report."""
        entry = SimpleNamespace(joint=self._joint(), drive_axis="angular", dof_index=0)
        self.assertEqual(self._builder(math.inf)._max_velocity_engine_warning(entry, math.inf), "")

    async def test_no_warning_when_engine_and_usd_agree(self) -> None:
        """The common case stays quiet."""
        entry = SimpleNamespace(joint=self._joint(), drive_axis="angular", dof_index=0)
        self.assertEqual(self._builder(180.0)._max_velocity_engine_warning(entry, 180.0), "")

    async def test_no_warning_without_engine_truth(self) -> None:
        """With no articulation (or before the timeline plays) there is nothing to compare."""
        entry = SimpleNamespace(joint=self._joint(), drive_axis="angular", dof_index=0)
        self.assertEqual(self._builder(None)._max_velocity_engine_warning(entry, 180.0), "")

    async def test_no_warning_before_the_timeline_plays(self) -> None:
        """Without a physics view the accessor reads USD, so it can only agree."""
        entry = SimpleNamespace(joint=self._joint(), drive_axis="angular", dof_index=0)
        self.assertEqual(self._builder(90.0, playing=False)._max_velocity_engine_warning(entry, 180.0), "")

    async def test_no_warning_without_a_dof_index(self) -> None:
        """An entry with no DOF index cannot be queried, and must not raise."""
        entry = SimpleNamespace(joint=self._joint(), drive_axis="angular", dof_index=None)
        self.assertEqual(self._builder(90.0)._max_velocity_engine_warning(entry, 180.0), "")


class TestReplayToApplyNote(omni.kit.test.AsyncTestCase):
    """The note saying a mid-run edit will not be simulated until a replay.

    Measured behaviour, not documented: PhysX picks up a mid-run
    ``physxJoint:armature`` / ``maxJointVelocity`` write on the next step, Newton
    does not, because it builds its model once in ``ModelBuilder.add_usd`` and
    registers no USD notice handler.  See the core extension's
    ``TestLiveUsdWriteReachesTheEngine``.
    """

    async def test_newton_is_told_to_replay_while_playing(self) -> None:
        """The one case where an edit lands in USD but not in the simulation."""
        note = UIBuilder.replay_to_apply_note(gain_tuner.BACKEND_NEWTON, playing=True)
        self.assertIn("Newton", note)
        self.assertIn("play again", note)
        self.assertTrue(note.isascii(), "omni.ui renders non-ASCII as '?'")

    async def test_physx_is_never_told_to_replay(self) -> None:
        """PhysX applies the write to the running articulation, so there is nothing to say."""
        self.assertEqual(UIBuilder.replay_to_apply_note(gain_tuner.BACKEND_PHYSX, playing=True), "")

    async def test_nothing_is_said_while_stopped(self) -> None:
        """The next play parses the stage regardless, so a stopped edit is not delayed."""
        for backend in (gain_tuner.BACKEND_NEWTON, gain_tuner.BACKEND_PHYSX):
            self.assertEqual(UIBuilder.replay_to_apply_note(backend, playing=False), "", msg=backend)

    async def test_an_unsupported_backend_is_warned_about_rather_than_assumed_live(self) -> None:
        """An engine whose behaviour was never measured is not assumed to be PhysX-like."""
        note = UIBuilder.replay_to_apply_note("remotesim", playing=True)
        self.assertIn("play again", note)
        self.assertNotIn("remotesim", note)

    def _builder(self, playing: bool) -> UIBuilder:
        builder = UIBuilder.__new__(UIBuilder)
        builder._backend_ctx = SimpleNamespace(backend=gain_tuner.BACKEND_PHYSX, solver_type="")
        builder._timeline = SimpleNamespace(is_playing=lambda: playing)
        return builder

    async def test_the_note_is_wired_into_the_panel_from_the_live_backend(self) -> None:
        """The panel resolves the engine live, so an engine switch re-decides the note."""
        with mock.patch.object(BackendContext, "active_backend_label", return_value=gain_tuner.BACKEND_NEWTON):
            self.assertIn("Newton", self._builder(playing=True)._replay_to_apply_note())
            self.assertEqual(self._builder(playing=False)._replay_to_apply_note(), "")
        with mock.patch.object(BackendContext, "active_backend_label", return_value=gain_tuner.BACKEND_PHYSX):
            self.assertEqual(self._builder(playing=True)._replay_to_apply_note(), "")

    async def test_a_broken_timeline_query_says_nothing(self) -> None:
        """A note is not worth an exception out of the panel build."""
        builder = UIBuilder.__new__(UIBuilder)
        builder._backend_ctx = SimpleNamespace(backend=gain_tuner.BACKEND_NEWTON, solver_type="")
        builder._timeline = SimpleNamespace(is_playing=lambda: (_ for _ in ()).throw(AttributeError("no timeline")))
        self.assertEqual(builder._replay_to_apply_note(), "")


class TestPlayDoesNotStripTheExplanation(omni.kit.test.AsyncTestCase):
    """Which value is in effect has to stay readable while the robot is moving.

    That is the moment the user is deciding what to change, so the per-field lines
    are pure over the resolution and the display: nothing about them is conditional
    on the timeline.  Playing only *adds* -- a measured engine value in place of an
    assumed default, and Newton's "stop and play again to apply" note.
    """

    def _joint(self, armature_newton=None, armature_physx=None):
        self._stage = Usd.Stage.CreateInMemory()
        joint = UsdPhysics.RevoluteJoint.Define(self._stage, "/Robot/joint").GetPrim()
        joint.ApplyAPI(gain_tuner.NEWTON_JOINT_API)
        joint.ApplyAPI(gain_tuner.PHYSX_JOINT_API)
        spec = gain_tuner.joint_param_spec("armature")
        if armature_newton is not None:
            joint.GetAttribute(spec.newton_attr).Set(armature_newton)
        if armature_physx is not None:
            joint.GetAttribute(spec.physx_attr).Set(armature_physx)
        return joint, spec

    async def test_a_divergence_reads_the_same_playing_or_stopped(self) -> None:
        """The sentence naming both values and the winner does not depend on play."""
        joint, spec = self._joint(armature_newton=0.25, armature_physx=0.87)
        resolution = UIBuilder._joint_param_resolution(joint, spec, "PhysX", "")

        stopped = joint_param_info_text(resolution, UIBuilder._joint_param_display(joint, spec, resolution))
        playing = joint_param_info_text(
            resolution, UIBuilder._joint_param_display(joint, spec, resolution, engine_value=0.87)
        )

        self.assertIn("newton:armature is 0.25", stopped)
        self.assertEqual(stopped, playing)

    async def test_a_placeholder_still_explains_itself_while_playing(self) -> None:
        """Playing swaps the assumed default for a measured one; it does not go quiet."""
        joint, spec = self._joint()
        resolution = UIBuilder._joint_param_resolution(joint, spec, gain_tuner.BACKEND_NEWTON, "xpbd")

        stopped = UIBuilder._joint_param_display(joint, spec, resolution)
        playing = UIBuilder._joint_param_display(joint, spec, resolution, engine_value=0.42)

        self.assertIn("applies its default", stopped.note)
        self.assertIn("simulating with 0.42", playing.note)
        self.assertFalse(playing.authored, "a measured value is still not the user's")

    async def test_the_fallback_annotation_survives_play_too(self) -> None:
        """The quietest case is the one that most needs to still be there."""
        joint, spec = self._joint(armature_physx=0.6)
        resolution = UIBuilder._joint_param_resolution(joint, spec, gain_tuner.BACKEND_NEWTON, "xpbd")
        display = UIBuilder._joint_param_display(joint, spec, resolution, engine_value=0.6)

        self.assertIn("falls back to physxJoint:armature", joint_param_info_text(resolution, display))


class TestChartColourContracts(omni.kit.test.AsyncTestCase):
    """Both plot widgets take one colour per *series*, which is what made this a bug.

    The velocity chart uses the older ``CustomXYPlot`` rather than
    ``JointGraphWidget``, and the two could in principle differ -- one expanding a
    per-joint list across a joint's series.  Neither does, so both need a list as
    long as ``y_data``.
    """

    async def test_the_joint_graph_widget_indexes_colours_by_series(self) -> None:
        """``_series_color`` falls back per index, so a short list greys the tail."""
        from isaacsim.robot_setup.gain_tuner.ui.chart_widget import JointGraphWidget

        widget = JointGraphWidget.__new__(JointGraphWidget)
        widget._data_colors = [0x11, 0x22]
        self.assertEqual(widget._series_color(0), 0x11)
        self.assertEqual(widget._series_color(1), 0x22)
        self.assertEqual(widget._series_color(2), LABEL_COLOR, "a short list silently greys the rest")

    async def test_the_xy_plot_pads_a_short_colour_list_per_series(self) -> None:
        """It does not repeat a per-joint list across that joint's series either."""
        from isaacsim.robot_setup.gain_tuner.ui.plot_widget import CustomXYPlot

        plot = CustomXYPlot.__new__(CustomXYPlot)
        plot._data_colors = [0x11, 0x22]
        resolved = plot._get_data_colors(4)
        self.assertEqual(len(resolved), 4, "the widget wants one colour per series")
        self.assertEqual(resolved[:2], [0x11, 0x22])
        self.assertNotEqual(resolved[2:], [0x11, 0x22], "a short list is padded, not repeated per group")


class TestChartSeriesColors(omni.kit.test.AsyncTestCase):
    """Chart colours have to follow the series, not the selection.

    ``JointGraphWidget.data_colors`` is indexed per series.  Every chart filters its
    selection down to the joints that produced data, so a colour list indexed by
    position in the selection puts each trace after a skipped joint on somebody
    else's colour -- with no error, and no way to notice except by knowing what
    colour a joint is supposed to be.
    """

    # Two swatches per joint: the saturated one for the commanded trace, the
    # lighter one for the observed response (ColorJointItem.colors).
    _GROUPS = {0: (0x11, 0xAA), 1: (0x22, 0xBB), 2: (0x33, 0xCC)}

    async def test_one_colour_per_series_for_a_two_group_chart(self) -> None:
        """A command + observed chart needs 2N colours, ordered as y_data is built."""
        colors = UIBuilder.series_colors(self._GROUPS, [0, 1, 2], groups=2)
        self.assertEqual(colors, [0x11, 0x22, 0x33, 0xAA, 0xBB, 0xCC])

    async def test_a_middle_joint_with_no_trajectory_shifts_nothing(self) -> None:
        """The regression: joint 1 is skipped, so joint 2 keeps its own colour."""
        colors = UIBuilder.series_colors(self._GROUPS, [0, 2], groups=2)
        self.assertEqual(colors, [0x11, 0x33, 0xAA, 0xCC])

    async def test_the_observed_half_gets_the_observed_swatches(self) -> None:
        """Not the command swatches, and not the widget's grey fallback."""
        colors = UIBuilder.series_colors(self._GROUPS, [0, 1], groups=2)
        observed = colors[2:]
        self.assertEqual(observed, [0xAA, 0xBB])
        self.assertNotIn(LABEL_COLOR, colors)

    async def test_a_single_group_chart_gets_one_colour_per_joint(self) -> None:
        """The Effort and dt-sweep charts plot one trace per joint."""
        self.assertEqual(UIBuilder.series_colors(self._GROUPS, [2, 0]), [0x33, 0x11])

    async def test_an_unselected_joint_falls_back_rather_than_shifting(self) -> None:
        """A DOF with no swatch takes the widget's own fallback, in place."""
        colors = UIBuilder.series_colors(self._GROUPS, [0, 9, 2])
        self.assertEqual(colors, [0x11, LABEL_COLOR, 0x33])

    async def test_no_plotted_series_means_no_colours(self) -> None:
        """An empty chart asks for nothing."""
        self.assertEqual(UIBuilder.series_colors(self._GROUPS, [], groups=2), [])

    async def test_a_joint_with_one_swatch_reuses_it_for_every_group(self) -> None:
        """Fewer swatches than groups must not raise or drop a series."""
        self.assertEqual(UIBuilder.series_colors({4: (0x77,)}, [4], groups=2), [0x77, 0x77])


class _BackendGateProbe(UIBuilder):
    """UIBuilder stub that reports a fixed live backend label.

    Args:
        backend: What :meth:`UIBuilder._active_backend` should report.
    """

    def __init__(self, backend: str) -> None:
        self._backend = backend
        self.restore_calls = 0
        self._gains_tuner = SimpleNamespace(
            restore_self_collision_override=self._count_restore,
        )

    def _count_restore(self) -> None:
        """Record that the UI delegated the restore to the core tuner."""
        self.restore_calls += 1

    def _active_backend(self) -> str:
        """Return the backend label this probe was constructed with."""
        return self._backend


class TestSelfCollisionBackendGate(omni.kit.test.AsyncTestCase):
    """Which engines the "disable self collisions" test option can be honoured on."""

    async def test_physx_is_supported(self) -> None:
        """PhysX reads the property, so nothing is reported."""
        self.assertIsNone(_BackendGateProbe("PhysX")._self_collision_override_unsupported_reason())

    async def test_newton_is_supported(self) -> None:
        """Newton defines its own self-collision property, which is authored too."""
        self.assertIsNone(_BackendGateProbe("NewtonAPI")._self_collision_override_unsupported_reason())

    async def test_unreadable_backend_is_reported_as_unsupported(self) -> None:
        """An engine that could not be read must not be assumed to honour the override.

        Reporting nothing for the empty label authors properties with no evidence
        anything consumes them and leaves the user to discover it from the results.
        """
        reason = _BackendGateProbe("")._self_collision_override_unsupported_reason()
        self.assertIsNotNone(reason)
        self.assertIn("could not be read", reason)

    async def test_unknown_engine_is_reported_as_unsupported(self) -> None:
        """A third-party engine defines neither property."""
        self.assertIsNotNone(_BackendGateProbe("remotesim")._self_collision_override_unsupported_reason())

    async def test_restore_is_delegated_to_the_core_tuner(self) -> None:
        """The panel owns no override state; the core tuner does."""
        probe = _BackendGateProbe("PhysX")
        probe._restore_self_collision_override()
        self.assertEqual(probe.restore_calls, 1)
