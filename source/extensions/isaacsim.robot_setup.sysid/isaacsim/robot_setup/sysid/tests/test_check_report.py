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

# ruff: noqa: ANN001, ANN202, D101, D102

"""Tests for the pre-solve check report (telemetry quality + identifiability)."""

from __future__ import annotations

import json

import numpy as np
import omni.kit.test
from isaacsim.robot_setup.sysid.check_report import (
    VERDICT_IDENTIFIABLE,
    VERDICT_NOT_IDENTIFIABLE,
    VERDICT_UNKNOWN,
    VERDICT_WEAK,
    ParameterIdentifiabilityVerdict,
    apply_zero_baseline_verdicts,
    build_sysid_check_report,
)
from isaacsim.robot_setup.sysid.parameter_space import ParameterBaselineState
from isaacsim.robot_setup.sysid.parameter_types import (
    SysIdParameterEntry,
    SysIdParameterType,
)
from isaacsim.robot_setup.sysid.provenance import build_dataset_metadata
from isaacsim.robot_setup.sysid.resource_loader import read_schema_resource
from isaacsim.robot_setup.sysid.run_spec import ParameterRunSpec, SysIdRunSpec
from isaacsim.robot_setup.sysid.schema_validation import (
    validate_sysid_check_report_payload,
)
from isaacsim.robot_setup.sysid.trajectory_csv import TrajectoryDataset
from isaacsim.robot_setup.sysid.trajectory_segments import (
    TelemetryChunkRunSpec,
    TrajectoryChunk,
)


def _trajectory(*, excite_joint_zero: bool = True) -> TrajectoryDataset:
    num_samples = 100
    times = np.arange(num_samples, dtype=np.float64) * 0.01
    positions = np.zeros((num_samples, 2), dtype=np.float64)
    if excite_joint_zero:
        positions[:, 0] = 0.5 * np.sin(2.0 * np.pi * times)
    velocities = np.gradient(positions, times, axis=0)
    return TrajectoryDataset(times=times, positions=positions, velocities=velocities, commands=positions.copy())


def _spec_with_selection() -> SysIdRunSpec:
    spec = SysIdRunSpec()
    spec.parameters.selected = [
        ParameterRunSpec(param_type="joint_friction", dof_index=0, initial=1.0, min=0.0, max=2.0),
        ParameterRunSpec(param_type="joint_damping", dof_index=1, initial=1.0, min=0.0, max=2.0),
    ]
    return spec


class CheckReportTests(omni.kit.test.AsyncTestCase):
    async def test_stage_free_report_yields_unknown_verdicts(self) -> None:
        report = build_sysid_check_report(_spec_with_selection(), stage=None, trajectory=_trajectory())
        self.assertEqual(len(report.parameter_verdicts), 2)
        for verdict in report.parameter_verdicts:
            self.assertEqual(verdict.verdict, VERDICT_UNKNOWN)
            self.assertTrue(verdict.notes)
        self.assertIsNone(report.presolve)
        self.assertFalse(report.ok)
        self.assertIn("identifiability_unavailable", [issue.code for issue in report.issues])

    async def test_weak_excitation_joint_downgrades_verdict_with_note(self) -> None:
        # Joint 1 is never excited; its selected damping parameter must be flagged.
        report = build_sysid_check_report(_spec_with_selection(), stage=None, trajectory=_trajectory())
        weak = [verdict for verdict in report.parameter_verdicts if verdict.dof_index == 1]
        self.assertEqual(len(weak), 1)
        self.assertTrue(any("weak excitation" in note.lower() for note in weak[0].notes))
        # Stage-free the presolve cannot upgrade to identifiable, so the verdict stays
        # unknown; with a presolve it would be downgraded to weak.
        self.assertIn(weak[0].verdict, (VERDICT_UNKNOWN, VERDICT_WEAK))

    async def test_report_round_trips_through_json(self) -> None:
        report = build_sysid_check_report(_spec_with_selection(), stage=None, trajectory=_trajectory())
        payload = json.loads(json.dumps(report.to_dict()))
        self.assertEqual(payload["schema_version"], 1)
        self.assertEqual(len(payload["parameter_verdicts"]), 2)
        self.assertIn("telemetry_quality", payload)
        self.assertEqual(validate_sysid_check_report_payload(payload), [])
        schema = json.loads(read_schema_resource("sysid_check_report.schema.json"))
        self.assertEqual(schema["properties"]["schema_version"]["const"], 1)

    async def test_check_report_validator_rejects_wrong_version(self) -> None:
        payload = build_sysid_check_report(
            _spec_with_selection(),
            stage=None,
            trajectory=_trajectory(),
        ).to_dict()
        payload["schema_version"] = 2

        issues = validate_sysid_check_report_payload(payload)

        self.assertEqual([issue.path for issue in issues if issue.severity == "error"], ["$.schema_version"])

    async def test_summary_lines_are_compact(self) -> None:
        report = build_sysid_check_report(_spec_with_selection(), stage=None, trajectory=_trajectory())
        lines = report.summary_lines()
        self.assertTrue(any("Identifiability:" in line for line in lines))

    async def test_no_selection_produces_no_verdicts(self) -> None:
        spec = SysIdRunSpec()
        report = build_sysid_check_report(spec, stage=None, trajectory=_trajectory())
        self.assertEqual(report.parameter_verdicts, [])

    async def test_check_report_exposes_quality_object_for_downstream_reuse(self) -> None:
        """The built assessment is handed back as an object, not just its dict."""
        report = build_sysid_check_report(_spec_with_selection(), stage=None, trajectory=_trajectory())

        self.assertIsNotNone(report.telemetry_quality_report)
        self.assertEqual(report.telemetry_quality_report.to_dict(), report.telemetry_quality)
        # The object is a reuse channel only; it must stay out of the JSON contract.
        self.assertNotIn("telemetry_quality_report", report.to_dict())

    async def test_presolve_stacks_all_training_chunks_without_boundary_acceleration(self) -> None:
        from isaacsim.robot_setup.sysid.check_report import (
            _stack_presolve_training_data,
        )

        first = _trajectory()
        second = _trajectory()
        first.velocities[:] = 1.0
        second.velocities[:] = 100.0
        chunks = [
            TrajectoryChunk(
                spec=TelemetryChunkRunSpec(name=f"train-{index}", role="train", start=0.0, end=0.99),
                trajectory=item,
                sample_count=item.times.shape[0],
                duration_seconds=item.duration,
            )
            for index, item in enumerate((first, second))
        ]

        stacked, accelerations = _stack_presolve_training_data(first, chunks, max_steps_per_chunk=60)

        self.assertEqual(stacked.positions.shape[0], 120)
        self.assertTrue(np.allclose(stacked.velocities[:60], 1.0))
        self.assertTrue(np.allclose(stacked.velocities[60:], 100.0))
        self.assertTrue(np.allclose(accelerations, 0.0))
        self.assertTrue(np.all(np.diff(stacked.times) > 0.0))

    async def test_zero_baseline_friction_multiplier_is_not_identifiable(self) -> None:
        # Friction is multiplier-parameterized (value = theta * baseline); a USD with
        # no authored friction snapshots baseline 0, so theta provably cannot affect
        # the rollout even when the analytical presolve rated the joint identifiable.
        entries = [
            SysIdParameterEntry(param_type=SysIdParameterType.JOINT_FRICTION, dof_index=0),
            SysIdParameterEntry(param_type=SysIdParameterType.JOINT_FRICTION, dof_index=1),
        ]
        verdicts = [
            ParameterIdentifiabilityVerdict(
                index=idx,
                name=f"Joint {idx + 1} - Friction",
                param_type="joint_friction",
                verdict=VERDICT_IDENTIFIABLE,
                dof_index=idx,
            )
            for idx in range(2)
        ]
        baseline = ParameterBaselineState(
            joint_friction={0: 0.0, 1: 0.8},
            joint_static_friction={0: 0.0, 1: 0.9},
            joint_dynamic_friction={0: 0.0, 1: 0.8},
        )
        issues = []
        apply_zero_baseline_verdicts(verdicts, entries, baseline, issues)

        self.assertEqual(verdicts[0].verdict, VERDICT_NOT_IDENTIFIABLE)
        self.assertTrue(any("zero baseline" in note.lower() for note in verdicts[0].notes))
        self.assertTrue(any("absolute parameterization" in note.lower() for note in verdicts[0].notes))
        self.assertEqual([issue.code for issue in issues], ["zero_baseline_parameter"])
        # The joint with an authored friction baseline keeps its presolve verdict.
        self.assertEqual(verdicts[1].verdict, VERDICT_IDENTIFIABLE)
        self.assertEqual(verdicts[1].notes, [])

    async def test_zero_baseline_rule_skips_absolute_and_uncaptured_parameters(self) -> None:
        entries = [
            # Armature is absolute-parameterized; a zero baseline is not a dead end.
            SysIdParameterEntry(param_type=SysIdParameterType.JOINT_ARMATURE, dof_index=0),
            # Friction on a DOF whose joint snapshot was never captured: cannot assess.
            SysIdParameterEntry(param_type=SysIdParameterType.JOINT_FRICTION, dof_index=5),
        ]
        verdicts = [
            ParameterIdentifiabilityVerdict(
                index=0,
                name="Joint 1 - Armature",
                param_type="joint_armature",
                verdict=VERDICT_IDENTIFIABLE,
                dof_index=0,
            ),
            ParameterIdentifiabilityVerdict(
                index=1,
                name="Joint 6 - Friction",
                param_type="joint_friction",
                verdict=VERDICT_IDENTIFIABLE,
                dof_index=5,
            ),
        ]
        baseline = ParameterBaselineState(joint_armature={0: 0.0}, joint_friction={0: 0.0})
        issues = []
        apply_zero_baseline_verdicts(verdicts, entries, baseline, issues)

        for verdict in verdicts:
            self.assertEqual(verdict.verdict, VERDICT_IDENTIFIABLE)
            self.assertEqual(verdict.notes, [])
        self.assertEqual(issues, [])


class FeedforwardModelFitTests(omni.kit.test.AsyncTestCase):
    """Measured torque may compare with models but cannot identify controller decomposition."""

    def _context_and_baseline(self, *, with_unmapped_hand: bool = False):
        from types import SimpleNamespace

        from isaacsim.robot_setup.sysid.regressor import (
            AnalyticalDynamicsContext,
            UnmappedSubtreeLink,
        )

        arm = 0.3
        # The hand variant models a light arm carrying a heavy end-effector that
        # the telemetry mapping does not cover (Franka arm + gripper): source
        # link 2 exists only in the baseline and hangs rigidly off context link 1.
        link_masses = (0.15, 0.1) if with_unmapped_hand else (3.0, 2.0)

        def _skew(v):
            return np.array([[0, -v[2], v[1]], [v[2], 0, -v[0]], [-v[1], v[0], 0]], dtype=np.float64)

        def _spatial(rotation, translation):
            out = np.zeros((6, 6), dtype=np.float64)
            out[:3, :3] = rotation
            out[3:, :3] = _skew(translation) @ rotation
            out[3:, 3:] = rotation
            return out

        def _rot_y(angle):
            c, s = np.cos(angle), np.sin(angle)
            return np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]], dtype=np.float64)

        def xup_fn(link_index, q_value):
            translation = np.zeros(3) if link_index == 0 else np.array([arm, 0.0, 0.0])
            return _spatial(_rot_y(q_value), translation)

        lumps = ()
        if with_unmapped_hand:
            lumps = (
                UnmappedSubtreeLink(
                    context_link_index=1,
                    source_link_index=2,
                    rotation=np.eye(3, dtype=np.float64),
                    translation=np.array([0.25, 0.0, 0.0], dtype=np.float64),
                ),
            )
        context = AnalyticalDynamicsContext(
            parents=np.array([-1, 0], dtype=np.int64),
            motion_subspaces=np.array([[0, 1, 0, 0, 0, 0], [0, 1, 0, 0, 0, 0]], dtype=np.float64),
            xup_fn=xup_fn,
            gravity=None,
            link_masses=np.array(link_masses, dtype=np.float64),
            source_link_indices=np.array([0, 1], dtype=np.int64),
            link_paths=("/arm/link0", "/arm/link1"),
            joint_paths=("/arm/joint0", "/arm/joint1"),
            unmapped_subtree_links=lumps,
        )
        from isaacsim.robot_setup.sysid.inertia_param import (
            inertia_matrix_to_log_cholesky,
        )

        baseline = ParameterBaselineState(
            link_inertia_lc={
                0: inertia_matrix_to_log_cholesky(np.diag([0.02, 0.03, 0.02])),
                1: inertia_matrix_to_log_cholesky(np.diag([0.01, 0.015, 0.01])),
            },
            link_com={0: np.array([0.15, 0.0, 0.0]), 1: np.array([0.12, 0.0, 0.04])},
            link_masses={0: link_masses[0], 1: link_masses[1]},
            joint_friction={0: 0.5, 1: 0.4},
            joint_stiffness={0: 100.0, 1: 80.0},
            joint_damping={0: 5.0, 1: 4.0},
        )
        if with_unmapped_hand:
            baseline.link_masses[2] = 3.0
            baseline.link_com[2] = np.array([0.04, 0.0, 0.0])
            baseline.link_inertia_lc[2] = inertia_matrix_to_log_cholesky(np.diag([0.008, 0.01, 0.008]))
        spec = SimpleNamespace(simulation=SimpleNamespace(newton=SimpleNamespace(feedforward="none")))
        return context, baseline, spec

    def _dynamic_trajectory(self, context, baseline, mode: str) -> TrajectoryDataset:
        from isaacsim.robot_setup.sysid.analytical_presolve import (
            _baseline_inertial_params,
        )
        from isaacsim.robot_setup.sysid.regressor import (
            finite_difference_acceleration,
            rnea_inverse_dynamics,
        )

        num_samples = 120
        times = np.arange(num_samples, dtype=np.float64) * 0.02
        positions = np.stack(
            (0.6 * np.sin(2.0 * np.pi * 0.8 * times), 0.4 * np.sin(2.0 * np.pi * 1.1 * times + 0.5)),
            axis=1,
        )
        velocities = np.gradient(positions, times, axis=0)
        accelerations = finite_difference_acceleration(times, velocities)
        inertial = _baseline_inertial_params(context, baseline, include_unmapped_subtrees=True)
        zeros = np.zeros_like(velocities)
        rows = []
        for t in range(num_samples):
            if mode == "gravity":
                rows.append(rnea_inverse_dynamics(context, positions[t], zeros[t], zeros[t], inertial))
            else:
                rows.append(rnea_inverse_dynamics(context, positions[t], velocities[t], accelerations[t], inertial))
        torques = np.asarray(rows, dtype=np.float64) + 0.02 * np.sin(9.0 * times)[:, None]
        metadata = build_dataset_metadata(
            source_type="csv",
            source_path="synthetic.csv",
            times=times,
            positions=positions,
            velocities=velocities,
            commands=positions,
            extra={"torque_semantics": "link_side"},
        )
        return TrajectoryDataset(
            times=times,
            positions=positions,
            velocities=velocities,
            commands=positions.copy(),
            metadata=metadata,
            torques=torques,
        )

    async def test_inverse_dynamics_fit_does_not_become_controller_recommendation(self) -> None:
        from isaacsim.robot_setup.sysid.check_report import _recommend_feedforward_mode

        context, baseline, spec = self._context_and_baseline()
        trajectory = self._dynamic_trajectory(context, baseline, mode="inverse_dynamics")
        issues = []
        payload = _recommend_feedforward_mode(spec, trajectory, context, baseline, issues)
        self.assertIsNotNone(payload)
        self.assertEqual(payload["best_fit"], "inverse_dynamics")
        self.assertIsNone(payload["recommended"])
        self.assertEqual(payload["configured"], "none")
        self.assertFalse(any(issue.code == "feedforward_mode_mismatch" for issue in issues))
        self.assertIn("cannot identify", payload["note"])

    async def test_unmapped_distal_mass_lumps_into_candidate_models(self) -> None:
        from dataclasses import replace

        from isaacsim.robot_setup.sysid.check_report import _recommend_feedforward_mode

        context, baseline, spec = self._context_and_baseline(with_unmapped_hand=True)
        # Measured torque from a gravity-compensating controller on the TRUE
        # robot (arm plus rigidly attached hand): the rigid composite is exact
        # for a fixed attachment, so the lumped candidate must explain it.
        trajectory = self._dynamic_trajectory(context, baseline, mode="gravity")
        issues = []
        payload = _recommend_feedforward_mode(spec, trajectory, context, baseline, issues)
        self.assertIsNotNone(payload)
        self.assertEqual(payload["best_fit"], "gravity")
        self.assertIsNone(payload["recommended"])
        self.assertFalse(any(issue.code == "feedforward_mode_mismatch" for issue in issues))

        # Omitting the rigidly attached hand makes the gravity model materially
        # worse even though total torque still cannot identify controller mode.
        bare_context = replace(context, unmapped_subtree_links=())
        bare_payload = _recommend_feedforward_mode(spec, trajectory, bare_context, baseline, [])
        self.assertIsNotNone(bare_payload)
        self.assertGreater(
            bare_payload["residual_rms"]["gravity"],
            payload["residual_rms"]["gravity"],
        )

    async def test_no_mismatch_warning_when_configured_matches(self) -> None:
        from isaacsim.robot_setup.sysid.check_report import _recommend_feedforward_mode

        context, baseline, spec = self._context_and_baseline()
        spec.simulation.newton.feedforward = "inverse_dynamics"
        trajectory = self._dynamic_trajectory(context, baseline, mode="inverse_dynamics")
        issues = []
        payload = _recommend_feedforward_mode(spec, trajectory, context, baseline, issues)
        self.assertEqual(payload["best_fit"], "inverse_dynamics")
        self.assertIsNone(payload["recommended"])
        self.assertFalse(any(issue.code == "feedforward_mode_mismatch" for issue in issues))
