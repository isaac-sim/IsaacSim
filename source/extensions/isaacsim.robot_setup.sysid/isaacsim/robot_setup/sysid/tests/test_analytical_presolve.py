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

# ruff: noqa: ANN001, ANN202, D102

"""Tests for the effort-semantics-aware analytical presolve torque regression."""

from __future__ import annotations

from dataclasses import replace
from unittest.mock import patch

import numpy as np
import omni.kit.test
import torch
from isaacsim.robot_setup.sysid.analytical_presolve import (
    EFFORT_SEMANTICS_EXTERNAL,
    EFFORT_SEMANTICS_LINK_SIDE,
    MAX_IDENTIFIABLE_CONDITION,
    VELOCITY_DEADBAND,
    _assemble_inertial_delta_regressor,
    _baseline_inertial_params,
    _collect_unmapped_subtree_links,
    _inertial_params_for_theta,
    assemble_joint_scale_torque_regressor,
    run_analytical_presolve,
)
from isaacsim.robot_setup.sysid.inertia_param import inertia_matrix_to_log_cholesky
from isaacsim.robot_setup.sysid.ingest.config_types import (
    CsvColumnMapping,
    TopicSignalMapping,
    TrajectoryIngestError,
)
from isaacsim.robot_setup.sysid.parameter_space import ParameterBaselineState
from isaacsim.robot_setup.sysid.parameter_types import (
    GLOBAL_DOF_INDEX,
    SysIdParameterEntry,
    SysIdParameterType,
)
from isaacsim.robot_setup.sysid.provenance import build_dataset_metadata
from isaacsim.robot_setup.sysid.regressor import (
    AnalyticalDynamicsContext,
    UnmappedSubtreeLink,
    assemble_inverse_dynamics_regressor,
    finite_difference_acceleration,
    rnea_inverse_dynamics,
)
from isaacsim.robot_setup.sysid.trajectory_csv import TrajectoryDataset

_ARM_LENGTH = 0.3
_NUM_STEPS = 120
_DT = 0.02

# True deviations from the baseline the presolve should recover.
_TRUE_MASS_SCALE = 1.15
_TRUE_COM_DELTA = np.array([0.015, 0.0, -0.008], dtype=np.float64)
_TRUE_FRICTION_SCALE = 1.4


class UnmappedSubtreeDiagnosticsTests(omni.kit.test.AsyncTestCase):
    """Verify optional unmapped-subtree diagnostics remain observable and nonfatal."""

    async def test_collection_failure_is_logged_and_remains_nonfatal(self) -> None:
        with patch("carb.log_verbose") as log_verbose:
            result = _collect_unmapped_subtree_links(None, [], {}, {})

        self.assertEqual(result, ())
        log_verbose.assert_called_once()
        self.assertIn("collection failed", log_verbose.call_args.args[0])
        self.assertIn("Traceback", log_verbose.call_args.args[0])


def _skew(v: np.ndarray) -> np.ndarray:
    return np.array(
        [[0.0, -v[2], v[1]], [v[2], 0.0, -v[0]], [-v[1], v[0], 0.0]],
        dtype=np.float64,
    )


def _spatial_transform(rotation: np.ndarray, translation: np.ndarray) -> np.ndarray:
    out = np.zeros((6, 6), dtype=np.float64)
    out[:3, :3] = rotation
    out[3:, :3] = _skew(translation) @ rotation
    out[3:, 3:] = rotation
    return out


def _rot_y(angle: float) -> np.ndarray:
    c, s = np.cos(angle), np.sin(angle)
    return np.array([[c, 0.0, s], [0.0, 1.0, 0.0], [-s, 0.0, c]], dtype=np.float64)


def _build_two_dof_context(baseline: ParameterBaselineState) -> AnalyticalDynamicsContext:
    """Planar 2-DOF arm: both joints revolute about Y, link 1 offset along X.

    Args:
        baseline: Baseline model parameters.

    Returns:
        Result produced by the operation.
    """

    def xup_fn(link_index: int, q_value: float) -> np.ndarray:
        translation = np.zeros(3) if link_index == 0 else np.array([_ARM_LENGTH, 0.0, 0.0])
        return _spatial_transform(_rot_y(q_value), translation)

    return AnalyticalDynamicsContext(
        parents=np.array([-1, 0], dtype=np.int64),
        motion_subspaces=np.array([[0.0, 1.0, 0.0, 0.0, 0.0, 0.0], [0.0, 1.0, 0.0, 0.0, 0.0, 0.0]], dtype=np.float64),
        xup_fn=xup_fn,
        gravity=None,
        link_masses=np.array([baseline.link_masses[0], baseline.link_masses[1]], dtype=np.float64),
        link_coms=np.stack([baseline.link_com[0], baseline.link_com[1]]).astype(np.float64),
        source_link_indices=np.array([0, 1], dtype=np.int64),
        link_paths=("/arm/link0", "/arm/link1"),
        joint_paths=("/arm/joint0", "/arm/joint1"),
    )


def _build_baseline() -> ParameterBaselineState:
    return ParameterBaselineState(
        link_inertia_lc={
            0: inertia_matrix_to_log_cholesky(np.diag([0.02, 0.03, 0.02])),
            1: inertia_matrix_to_log_cholesky(np.diag([0.01, 0.015, 0.01])),
        },
        link_com={0: np.array([0.15, 0.0, 0.0]), 1: np.array([0.12, 0.0, 0.04])},
        link_masses={0: 3.0, 1: 2.0},
        joint_friction={0: 0.5, 1: 0.4},
        joint_stiffness={0: 100.0, 1: 80.0},
        joint_damping={0: 5.0, 1: 4.0},
    )


def _build_baseline_with_hand() -> ParameterBaselineState:
    """Arm baseline plus an end-effector body (source link 2) absent from the mapping.

    Returns:
        Result produced by the operation.
    """
    baseline = _build_baseline()
    baseline.link_masses[2] = 1.0
    baseline.link_com[2] = np.array([0.04, 0.0, 0.0])
    baseline.link_inertia_lc[2] = inertia_matrix_to_log_cholesky(np.diag([0.008, 0.01, 0.008]))
    return baseline


def _attach_unmapped_hand(context: AnalyticalDynamicsContext) -> AnalyticalDynamicsContext:
    """Hang source link 2 rigidly off context link 1 (Franka arm + gripper shape).

    Args:
        context: Runtime context used by the operation.

    Returns:
        Result produced by the operation.
    """
    return replace(
        context,
        unmapped_subtree_links=(
            UnmappedSubtreeLink(
                context_link_index=1,
                source_link_index=2,
                rotation=np.eye(3, dtype=np.float64),
                translation=np.array([0.25, 0.0, 0.0], dtype=np.float64),
            ),
        ),
    )


def _build_entries() -> list[SysIdParameterEntry]:
    return [
        SysIdParameterEntry(SysIdParameterType.LINK_MASS, GLOBAL_DOF_INDEX, link_index=1),
        SysIdParameterEntry(SysIdParameterType.LINK_COM_OFFSET_X, GLOBAL_DOF_INDEX, link_index=1),
        SysIdParameterEntry(SysIdParameterType.LINK_COM_OFFSET_Z, GLOBAL_DOF_INDEX, link_index=1),
        SysIdParameterEntry(SysIdParameterType.JOINT_FRICTION, 0),
        SysIdParameterEntry(SysIdParameterType.JOINT_FRICTION, 1),
        SysIdParameterEntry(SysIdParameterType.JOINT_STIFFNESS, 0),
        SysIdParameterEntry(SysIdParameterType.JOINT_STIFFNESS, 1),
        SysIdParameterEntry(SysIdParameterType.JOINT_DAMPING, 0),
        SysIdParameterEntry(SysIdParameterType.JOINT_DAMPING, 1),
    ]


def _theta_vectors() -> tuple[torch.Tensor, torch.Tensor, torch.Tensor]:
    theta_initial = torch.tensor([1.0, 0.0, 0.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0], dtype=torch.float32)
    theta_min = torch.tensor([0.5, -0.05, -0.05, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1], dtype=torch.float32)
    theta_max = torch.tensor([2.0, 0.05, 0.05, 3.0, 3.0, 3.0, 3.0, 3.0, 3.0], dtype=torch.float32)
    return theta_initial, theta_min, theta_max


def _deadband_signs(velocities: np.ndarray) -> np.ndarray:
    signs = np.sign(velocities)
    signs[np.abs(velocities) <= VELOCITY_DEADBAND] = 0.0
    return signs


def _true_inertial_params(context: AnalyticalDynamicsContext, baseline: ParameterBaselineState) -> np.ndarray:
    """Baseline 10-parameter rows with the true mass/COM deviation applied to link 1.

    Args:
        context: Runtime context used by the operation.
        baseline: Baseline model parameters.

    Returns:
        Result produced by the operation.
    """
    params = _baseline_inertial_params(context, baseline)
    true_mass = baseline.link_masses[1] * _TRUE_MASS_SCALE
    true_com = np.asarray(baseline.link_com[1], dtype=np.float64) + _TRUE_COM_DELTA
    params[1, 0] = true_mass
    params[1, 1:4] = true_mass * true_com
    return params


def _synthesize_gravity_compensated_trajectory(
    context: AnalyticalDynamicsContext,
    baseline: ParameterBaselineState,
    *,
    true_params: np.ndarray | None = None,
) -> tuple[TrajectoryDataset, np.ndarray]:
    """Trajectory whose recorded effort is the link-side torque tau_J.

    The measured torque is generated from the true dynamics (RNEA at
    ``true_params``, defaulting to the baseline with the module-level true
    mass/COM deviation applied, plus load-side Coulomb friction at the true
    friction scale). Commands are chosen so a
    gravity-compensated PD drive at the baseline gains reproduces exactly that
    torque -- as on a real Franka, the drive output and the link-side measurement
    agree at the true parameters, which is what makes summing both models predict
    twice the measurement.

    Args:
        context: Runtime context used by the operation.
        baseline: Baseline model parameters.
        true_params: Value supplied for ``true_params``.

    Returns:
        Result produced by the operation.
    """
    times = np.arange(_NUM_STEPS, dtype=np.float64) * _DT
    q = np.stack(
        (
            0.6 * np.sin(1.7 * times) + 0.2 * np.sin(0.9 * times + 0.5),
            0.8 * np.sin(1.3 * times + 0.3) + 0.15 * np.sin(2.1 * times),
        ),
        axis=1,
    )
    qd = np.stack(
        (
            0.6 * 1.7 * np.cos(1.7 * times) + 0.2 * 0.9 * np.cos(0.9 * times + 0.5),
            0.8 * 1.3 * np.cos(1.3 * times + 0.3) + 0.15 * 2.1 * np.cos(2.1 * times),
        ),
        axis=1,
    )
    # The presolve differentiates the recorded velocities; generating the torque from
    # the same finite-difference accelerations keeps the regression exact.
    qdd = finite_difference_acceleration(times, qd)

    if true_params is None:
        true_params = _true_inertial_params(context, baseline)
    true_friction = np.array(
        [baseline.joint_friction[0] * _TRUE_FRICTION_SCALE, baseline.joint_friction[1] * _TRUE_FRICTION_SCALE]
    )
    signs = _deadband_signs(qd)
    tau_measured = np.stack(
        [rnea_inverse_dynamics(context, q[t], qd[t], qdd[t], true_params) for t in range(_NUM_STEPS)]
    )
    tau_measured += true_friction[np.newaxis, :] * signs

    stiffness = np.array([baseline.joint_stiffness[0], baseline.joint_stiffness[1]])
    damping = np.array([baseline.joint_damping[0], baseline.joint_damping[1]])
    friction = np.array([baseline.joint_friction[0], baseline.joint_friction[1]])
    commands = q + (tau_measured + damping * qd + friction * signs) / stiffness

    metadata = build_dataset_metadata(
        source_type="csv",
        source_path="synthetic.csv",
        times=times,
        positions=q,
        velocities=qd,
        commands=commands,
        extra={"torque_semantics": EFFORT_SEMANTICS_LINK_SIDE},
    )
    trajectory = TrajectoryDataset(
        times=times,
        positions=q,
        velocities=qd,
        commands=commands,
        metadata=metadata,
        torques=tau_measured,
    )
    return trajectory, tau_measured


class TestAnalyticalPresolveLinkSideTorque(omni.kit.test.AsyncTestCase):
    """Link-side measured torque: RNEA + friction regression, no PD columns."""

    def setUp(self) -> None:
        self.baseline = _build_baseline()
        self.context = _build_two_dof_context(self.baseline)
        self.entries = _build_entries()
        self.theta_initial, self.theta_min, self.theta_max = _theta_vectors()
        self.trajectory, self.tau_measured = _synthesize_gravity_compensated_trajectory(self.context, self.baseline)

    async def test_recovers_mass_com_and_friction_from_link_side_torque(self) -> None:
        result = run_analytical_presolve(
            self.trajectory,
            self.entries,
            self.theta_initial,
            self.theta_min,
            self.theta_max,
            self.baseline,
            dynamics_context=self.context,
        )

        self.assertEqual(result.target_source, "measured_torque")
        seed = result.theta_seed.numpy().astype(np.float64)

        # Inertial + friction parameters are seeded from the measurement.
        self.assertTrue(all(result.identifiable[idx] for idx in (0, 1, 2, 3, 4)))
        self.assertEqual(result.seeded_count, 5)
        self.assertAlmostEqual(float(seed[0]), _TRUE_MASS_SCALE, delta=0.02)
        self.assertAlmostEqual(float(seed[3]), _TRUE_FRICTION_SCALE, delta=0.05)
        self.assertAlmostEqual(float(seed[4]), _TRUE_FRICTION_SCALE, delta=0.05)

        # mass*com (the first moment, what RNEA actually observes) matches the truth.
        recovered_mass = self.baseline.link_masses[1] * float(seed[0])
        recovered_com = np.asarray(self.baseline.link_com[1]) + np.array([seed[1], 0.0, seed[2]])
        recovered_first_moment = recovered_mass * recovered_com
        true_first_moment = (
            self.baseline.link_masses[1] * _TRUE_MASS_SCALE * (np.asarray(self.baseline.link_com[1]) + _TRUE_COM_DELTA)
        )
        error = np.linalg.norm(recovered_first_moment - true_first_moment)
        self.assertLess(error, 0.03 * np.linalg.norm(true_first_moment))

    async def test_sample_weights_exclude_corrupt_torque_rows_from_fit(self) -> None:
        corrupt_torque = self.trajectory.torques.copy()
        corrupt_torque[-20:] += 1000.0
        sample_weights = np.ones(_NUM_STEPS, dtype=np.float64)
        sample_weights[-20:] = 0.0
        trajectory = TrajectoryDataset(
            times=self.trajectory.times,
            positions=self.trajectory.positions,
            velocities=self.trajectory.velocities,
            commands=self.trajectory.commands,
            metadata=self.trajectory.metadata,
            torques=corrupt_torque,
            residual_sample_weights=sample_weights,
        )

        result = run_analytical_presolve(
            trajectory,
            self.entries,
            self.theta_initial,
            self.theta_min,
            self.theta_max,
            self.baseline,
            dynamics_context=self.context,
        )

        seed = result.theta_seed.numpy().astype(np.float64)
        self.assertAlmostEqual(float(seed[0]), _TRUE_MASS_SCALE, delta=0.02)
        self.assertAlmostEqual(float(seed[3]), _TRUE_FRICTION_SCALE, delta=0.05)

    async def test_drive_gains_are_not_seeded_from_link_side_torque(self) -> None:
        result = run_analytical_presolve(
            self.trajectory,
            self.entries,
            self.theta_initial,
            self.theta_min,
            self.theta_max,
            self.baseline,
            dynamics_context=self.context,
        )

        seed = result.theta_seed.numpy().astype(np.float64)
        for idx in (5, 6, 7, 8):
            self.assertFalse(result.identifiable[idx])
            self.assertTrue(result.torque_equation_excluded[idx])
            self.assertAlmostEqual(float(seed[idx]), 1.0, places=6)
        self.assertTrue(any("stiffness/damping" in warning for warning in result.warnings))

    async def test_old_summed_model_double_counts_and_biases_the_seed(self) -> None:
        """The pre-fix regression (drive PD + RNEA vs measured torque) is provably biased.

        Both halves individually equal the link-side measurement at the true parameters,
        so their sum predicts ~2x the measurement. Least squares then cancels the excess
        by collapsing the drive-gain scales to zero and driving friction negative (on
        real, non-ideally-consistent telemetry the bias additionally spreads into the
        inertial seeds). The new link-side regression has no PD columns to corrupt.
        """
        steps = _NUM_STEPS
        q = self.trajectory.positions
        qd = self.trajectory.velocities
        qdd = finite_difference_acceleration(self.trajectory.times, qd)
        theta_np = self.theta_initial.numpy().astype(np.float64)
        y = self.tau_measured.reshape(-1)

        drive_regressor = assemble_joint_scale_torque_regressor(
            q, qd, self.trajectory.commands, self.entries, self.baseline
        )
        inertial_regressor, initial_inertial_tau, inertial_ok = _assemble_inertial_delta_regressor(
            q,
            qd,
            qdd,
            self.entries,
            self.theta_initial,
            self.theta_min,
            self.theta_max,
            self.baseline,
            self.context,
            [0, 1, 2],
            warnings=None,
        )
        self.assertTrue(inertial_ok)

        # Both models individually predict the measurement at (approximately) the true
        # parameters, so the summed initial prediction is ~2x the measurement.
        old_prediction = drive_regressor @ theta_np + initial_inertial_tau.reshape(-1)
        double_count_ratio = float(np.dot(old_prediction, y) / np.dot(y, y))
        self.assertGreater(double_count_ratio, 1.7)

        old_solution, *_ = np.linalg.lstsq(drive_regressor + inertial_regressor, y - old_prediction, rcond=None)
        old_seed = theta_np + old_solution
        # True drive-gain scales are 1.0 (commands were generated with the baseline
        # gains) and the true friction scale is 1.4 -- the old model collapses the
        # gains and flips friction to a nonphysical negative value.
        self.assertLess(abs(old_seed[5]), 0.3)
        self.assertLess(abs(old_seed[7]), 0.3)
        self.assertLess(float(old_seed[3]), 0.0)

        new_result = run_analytical_presolve(
            self.trajectory,
            self.entries,
            self.theta_initial,
            self.theta_min,
            self.theta_max,
            self.baseline,
            dynamics_context=self.context,
            max_steps=steps,
        )
        new_seed = new_result.theta_seed.numpy().astype(np.float64)
        self.assertLess(abs(float(new_seed[0]) - _TRUE_MASS_SCALE), 0.02)
        self.assertLess(abs(float(new_seed[3]) - _TRUE_FRICTION_SCALE), 0.05)
        for idx in (5, 6, 7, 8):
            self.assertAlmostEqual(float(new_seed[idx]), 1.0, places=6)

    async def test_external_semantics_ignores_measured_torque(self) -> None:
        result = run_analytical_presolve(
            self.trajectory,
            self.entries,
            self.theta_initial,
            self.theta_min,
            self.theta_max,
            self.baseline,
            dynamics_context=self.context,
            torque_semantics=EFFORT_SEMANTICS_EXTERNAL,
        )

        self.assertEqual(result.target_source, "baseline_inverse_dynamics")
        self.assertTrue(any("tau_ext" in warning for warning in result.warnings))
        # Inertial parameters need a link-side measurement; drive gains fall back to
        # the drive-balance equation against baseline inverse dynamics.
        for idx in (0, 1, 2):
            self.assertFalse(result.identifiable[idx])
        for idx in (3, 4, 5, 6, 7, 8):
            self.assertTrue(result.identifiable[idx])

    async def test_external_semantics_read_from_trajectory_metadata(self) -> None:
        metadata = build_dataset_metadata(
            source_type="csv",
            source_path="synthetic.csv",
            times=self.trajectory.times,
            positions=self.trajectory.positions,
            velocities=self.trajectory.velocities,
            commands=self.trajectory.commands,
            extra={"torque_semantics": EFFORT_SEMANTICS_EXTERNAL},
        )
        trajectory = TrajectoryDataset(
            times=self.trajectory.times,
            positions=self.trajectory.positions,
            velocities=self.trajectory.velocities,
            commands=self.trajectory.commands,
            metadata=metadata,
            torques=self.trajectory.torques,
        )

        result = run_analytical_presolve(
            trajectory,
            self.entries,
            self.theta_initial,
            self.theta_min,
            self.theta_max,
            self.baseline,
            dynamics_context=self.context,
        )

        self.assertEqual(result.target_source, "baseline_inverse_dynamics")
        self.assertFalse(result.identifiable[0])

    async def test_missing_torque_semantics_fails_closed(self) -> None:
        trajectory = TrajectoryDataset(
            times=self.trajectory.times,
            positions=self.trajectory.positions,
            velocities=self.trajectory.velocities,
            commands=self.trajectory.commands,
            torques=self.trajectory.torques,
        )

        result = run_analytical_presolve(
            trajectory,
            self.entries,
            self.theta_initial,
            self.theta_min,
            self.theta_max,
            self.baseline,
            dynamics_context=self.context,
        )

        self.assertEqual(result.target_source, "baseline_inverse_dynamics")
        self.assertFalse(result.identifiable[0])
        self.assertTrue(any("unspecified" in warning for warning in result.warnings))

    async def test_no_torque_fallback_seeds_drive_gains_only(self) -> None:
        trajectory = TrajectoryDataset(
            times=self.trajectory.times,
            positions=self.trajectory.positions,
            velocities=self.trajectory.velocities,
            commands=self.trajectory.commands,
        )

        result = run_analytical_presolve(
            trajectory,
            self.entries,
            self.theta_initial,
            self.theta_min,
            self.theta_max,
            self.baseline,
            dynamics_context=self.context,
        )

        self.assertEqual(result.target_source, "baseline_inverse_dynamics")
        for idx in (0, 1, 2):
            self.assertFalse(result.identifiable[idx])
        for idx in (3, 4, 5, 6, 7, 8):
            self.assertTrue(result.identifiable[idx])

    async def test_baseline_inertial_params_match_theta_decode_at_initial_theta(self) -> None:
        baseline_params = _baseline_inertial_params(self.context, self.baseline)
        decoded_params = _inertial_params_for_theta(self.baseline, self.context, self.entries, self.theta_initial)
        np.testing.assert_allclose(baseline_params, decoded_params, rtol=1e-6, atol=1e-9)


class TestAnalyticalPresolveNumericalGates(omni.kit.test.AsyncTestCase):
    """Fail closed on unstable or malformed least-squares systems."""

    def setUp(self) -> None:
        values = np.zeros((4, 2), dtype=np.float64)
        self.trajectory = TrajectoryDataset(
            times=np.arange(4, dtype=np.float64) * 0.01,
            positions=values.copy(),
            velocities=values.copy(),
            commands=values.copy(),
        )
        self.baseline = ParameterBaselineState(joint_friction={0: 1.0, 1: 1.0})

    async def test_ill_conditioned_full_rank_system_is_not_seeded(self) -> None:
        entries = [
            SysIdParameterEntry(SysIdParameterType.JOINT_FRICTION, 0),
            SysIdParameterEntry(SysIdParameterType.JOINT_FRICTION, 1),
        ]
        theta_initial = torch.ones(2)
        direction = np.linspace(1.0, 2.0, 8)
        perturbation = np.asarray([1.0, -1.0, 1.0, -1.0, 1.0, -1.0, 1.0, -1.0])
        regressor = np.column_stack((direction, direction + 1e-9 * perturbation))
        self.assertEqual(np.linalg.matrix_rank(regressor), 2)

        with (
            patch(
                "isaacsim.robot_setup.sysid.analytical_presolve.assemble_joint_scale_torque_regressor",
                return_value=regressor,
            ),
            patch(
                "isaacsim.robot_setup.sysid.analytical_presolve._baseline_inverse_dynamics",
                return_value=np.zeros((4, 2), dtype=np.float64),
            ),
        ):
            result = run_analytical_presolve(
                self.trajectory,
                entries,
                theta_initial,
                torch.zeros(2),
                torch.full((2,), 2.0),
                self.baseline,
            )

        self.assertEqual(result.rank, 2)
        self.assertGreater(result.condition, MAX_IDENTIFIABLE_CONDITION)
        self.assertEqual(result.identifiable, [False, False])
        torch.testing.assert_close(result.theta_seed, theta_initial)
        self.assertTrue(any("ill-conditioned" in warning for warning in result.warnings))

    async def test_out_of_bounds_unconstrained_seed_is_not_clipped_but_remains_identifiable(self) -> None:
        entry = SysIdParameterEntry(SysIdParameterType.JOINT_FRICTION, 0)
        regressor = np.ones((8, 1), dtype=np.float64)
        with (
            patch(
                "isaacsim.robot_setup.sysid.analytical_presolve.assemble_joint_scale_torque_regressor",
                return_value=regressor,
            ),
            patch(
                "isaacsim.robot_setup.sysid.analytical_presolve._baseline_inverse_dynamics",
                return_value=np.full((4, 2), 100.0, dtype=np.float64),
            ),
        ):
            result = run_analytical_presolve(
                self.trajectory,
                [entry],
                torch.ones(1),
                torch.zeros(1),
                torch.full((1,), 2.0),
                self.baseline,
            )

        self.assertEqual(result.identifiable, [True])
        self.assertEqual(result.seeded_count, 0)
        torch.testing.assert_close(result.theta_seed, torch.ones(1))
        self.assertTrue(any("unconstrained seed" in warning for warning in result.warnings))

    async def test_target_and_regressor_row_mismatch_raises(self) -> None:
        entry = SysIdParameterEntry(SysIdParameterType.JOINT_FRICTION, 0)
        with (
            patch(
                "isaacsim.robot_setup.sysid.analytical_presolve.assemble_joint_scale_torque_regressor",
                return_value=np.ones((8, 1), dtype=np.float64),
            ),
            patch(
                "isaacsim.robot_setup.sysid.analytical_presolve._baseline_inverse_dynamics",
                return_value=np.zeros((3, 2), dtype=np.float64),
            ),
        ):
            with self.assertRaisesRegex(ValueError, "target has 6 rows.*regressor has 8 rows"):
                run_analytical_presolve(
                    self.trajectory,
                    [entry],
                    torch.ones(1),
                    torch.zeros(1),
                    torch.full((1,), 2.0),
                    self.baseline,
                )

    async def test_rnea_rejects_non_topological_parent_indices(self) -> None:
        context = replace(_build_two_dof_context(_build_baseline()), parents=np.asarray([1, -1], dtype=np.int64))
        inertial = np.zeros((2, 10), dtype=np.float64)

        with self.assertRaisesRegex(ValueError, "not topologically ordered"):
            rnea_inverse_dynamics(context, np.zeros(2), np.zeros(2), np.zeros(2), inertial)

    async def test_rnea_rejects_incomplete_parent_indices(self) -> None:
        context = replace(_build_two_dof_context(_build_baseline()), parents=np.asarray([-1], dtype=np.int64))
        inertial = np.zeros((2, 10), dtype=np.float64)

        with self.assertRaisesRegex(ValueError, "provide 2 parent indices"):
            rnea_inverse_dynamics(context, np.zeros(2), np.zeros(2), np.zeros(2), inertial)


class TestAnalyticalPresolveUnmappedSubtrees(omni.kit.test.AsyncTestCase):
    """Unmapped end-effector mass is composited into the presolve torque model.

    Mirrors `test_check_report.test_unmapped_distal_mass_lumps_into_candidate_models`:
    the compact RNEA context omits distal bodies the telemetry mapping never covers
    (an attached gripper), so without the rigid composite their gravity torque leaks
    into whatever seeds the regression can bend.
    """

    def setUp(self) -> None:
        self.baseline = _build_baseline_with_hand()
        self.context = _attach_unmapped_hand(_build_two_dof_context(self.baseline))
        self.bare_context = replace(self.context, unmapped_subtree_links=())
        self.entries = _build_entries()
        self.theta_initial, self.theta_min, self.theta_max = _theta_vectors()
        # True robot: baseline mapped links (mass scale exactly 1.0) carrying the
        # rigidly attached hand; only friction deviates from the baseline.
        self.true_params = _baseline_inertial_params(self.context, self.baseline, include_unmapped_subtrees=True)
        self.trajectory, self.tau_measured = _synthesize_gravity_compensated_trajectory(
            self.context, self.baseline, true_params=self.true_params
        )

    async def test_measured_torque_path_lumps_hand_into_prediction(self) -> None:
        result = run_analytical_presolve(
            self.trajectory,
            self.entries,
            self.theta_initial,
            self.theta_min,
            self.theta_max,
            self.baseline,
            dynamics_context=self.context,
        )

        self.assertEqual(result.target_source, "measured_torque")
        seed = result.theta_seed.numpy().astype(np.float64)
        # The hand's torque is explained by the composite, so the mapped seeds stay
        # at the truth: link-1 mass/COM unchanged, friction at its true scale.
        self.assertAlmostEqual(float(seed[0]), 1.0, delta=0.02)
        self.assertAlmostEqual(float(seed[1]), 0.0, delta=0.005)
        self.assertAlmostEqual(float(seed[2]), 0.0, delta=0.005)
        self.assertAlmostEqual(float(seed[3]), _TRUE_FRICTION_SCALE, delta=0.05)
        self.assertAlmostEqual(float(seed[4]), _TRUE_FRICTION_SCALE, delta=0.05)

        # Documents the pre-fix bug: with the hand absent from the RNEA prediction
        # (the old mapped-only params), its gravity torque is absorbed by the
        # link-1 inertial seeds -- a wrong answer the downstream rollout then
        # double-counts, since the simulated robot still carries the real gripper.
        bare_result = run_analytical_presolve(
            self.trajectory,
            self.entries,
            self.theta_initial,
            self.theta_min,
            self.theta_max,
            self.baseline,
            dynamics_context=self.bare_context,
        )
        bare_seed = bare_result.theta_seed.numpy().astype(np.float64)
        self.assertGreater(abs(float(bare_seed[0]) - 1.0), 0.15)

    async def test_drive_balance_target_includes_hand_gravity(self) -> None:
        # No measured torque: drive gains are seeded against baseline inverse
        # dynamics via the drive-balance equation k*(cmd-q) - d*qd - f*sign = tau_id.
        # Build commands so the baseline-gain PD supplies exactly the FULL load
        # (arm + hand): the true gain scales are then exactly 1.0.
        q = self.trajectory.positions
        qd = self.trajectory.velocities
        times = self.trajectory.times
        qdd = finite_difference_acceleration(times, qd)
        tau_full = np.stack(
            [rnea_inverse_dynamics(self.context, q[t], qd[t], qdd[t], self.true_params) for t in range(_NUM_STEPS)]
        )
        signs = _deadband_signs(qd)
        stiffness = np.array([self.baseline.joint_stiffness[0], self.baseline.joint_stiffness[1]])
        damping = np.array([self.baseline.joint_damping[0], self.baseline.joint_damping[1]])
        friction = np.array([self.baseline.joint_friction[0], self.baseline.joint_friction[1]])
        commands = q + (tau_full + damping * qd + friction * signs) / stiffness
        trajectory = TrajectoryDataset(times=times, positions=q, velocities=qd, commands=commands)

        result = run_analytical_presolve(
            trajectory,
            self.entries,
            self.theta_initial,
            self.theta_min,
            self.theta_max,
            self.baseline,
            dynamics_context=self.context,
        )
        self.assertEqual(result.target_source, "baseline_inverse_dynamics")
        seed = result.theta_seed.numpy().astype(np.float64)
        for idx in (3, 4, 5, 6, 7, 8):
            self.assertTrue(result.identifiable[idx])
            self.assertAlmostEqual(float(seed[idx]), 1.0, delta=0.02)

        # Pre-fix: the target understates the load by the hand's torque, so the
        # gain seeds are pulled away from the truth to cancel the missing gravity.
        bare_result = run_analytical_presolve(
            trajectory,
            self.entries,
            self.theta_initial,
            self.theta_min,
            self.theta_max,
            self.baseline,
            dynamics_context=self.bare_context,
        )
        bare_seed = bare_result.theta_seed.numpy().astype(np.float64)
        bare_gain_error = max(abs(float(bare_seed[idx]) - 1.0) for idx in (3, 4, 5, 6, 7, 8))
        self.assertGreater(bare_gain_error, 0.05)

    async def test_theta_decode_includes_unmapped_lumps(self) -> None:
        decoded = _inertial_params_for_theta(self.baseline, self.context, self.entries, self.theta_initial)
        lumped = _baseline_inertial_params(self.context, self.baseline, include_unmapped_subtrees=True)
        np.testing.assert_allclose(decoded, lumped, rtol=1e-6, atol=1e-9)

    async def test_lump_shifts_prediction_but_not_regressor_columns(self) -> None:
        # The hand carries no theta entries, so it must appear as a constant in the
        # predicted torque and cancel out of the finite-difference columns.
        q = self.trajectory.positions
        qd = self.trajectory.velocities
        qdd = finite_difference_acceleration(self.trajectory.times, qd)
        lumped_regressor, lumped_tau, lumped_ok = _assemble_inertial_delta_regressor(
            q,
            qd,
            qdd,
            self.entries,
            self.theta_initial,
            self.theta_min,
            self.theta_max,
            self.baseline,
            self.context,
            [0, 1, 2],
            warnings=None,
        )
        bare_regressor, bare_tau, bare_ok = _assemble_inertial_delta_regressor(
            q,
            qd,
            qdd,
            self.entries,
            self.theta_initial,
            self.theta_min,
            self.theta_max,
            self.baseline,
            self.bare_context,
            [0, 1, 2],
            warnings=None,
        )
        self.assertTrue(lumped_ok)
        self.assertTrue(bare_ok)
        np.testing.assert_allclose(lumped_regressor, bare_regressor, rtol=1e-6, atol=1e-9)
        self.assertGreater(float(np.max(np.abs(lumped_tau - bare_tau))), 0.5)


class TestTorqueSemanticsMappingConfig(omni.kit.test.AsyncTestCase):
    """`torque_semantics` parsing in the CSV/topic mapping configs."""

    async def test_topic_mapping_leaves_semantics_unspecified(self) -> None:
        mapping = TopicSignalMapping.from_dict({"position_topic": "/joint_states"})
        self.assertIsNone(mapping.torque_semantics)

    async def test_topic_mapping_accepts_external(self) -> None:
        mapping = TopicSignalMapping.from_dict({"position_topic": "/joint_states", "torque_semantics": "external"})
        self.assertEqual(mapping.torque_semantics, "external")

    async def test_csv_mapping_rejects_unknown_semantics(self) -> None:
        with self.assertRaises(TrajectoryIngestError):
            CsvColumnMapping.from_dict({"position_columns": ["q1"], "torque_semantics": "motor"})


_GRAVITY_VEC = np.array([0.0, 0.0, -9.80665], dtype=np.float64)

# Serial chain with rotated AND offset joint frames on both the parent and child
# side -- the geometry that distinguishes the three USD-context conventions this
# suite locks down (see TestUsdContextAgainstWorldNewtonEuler). Joint 1's axis is
# vertical in the world so its static gravity torque is exactly zero.
_CHAIN_JOINTS = (
    {"axis": "Z", "pos0": (0.0, 0.0, 0.10), "rot0": ("X", 0.0), "pos1": (0.03, -0.02, 0.11), "rot1": ("Y", 35.0)},
    {"axis": "Y", "pos0": (0.05, 0.0, 0.15), "rot0": ("Z", 20.0), "pos1": (0.0, 0.04, -0.12), "rot1": ("X", -40.0)},
    {"axis": "X", "pos0": (0.20, 0.05, 0.0), "rot0": ("Y", -25.0), "pos1": (-0.06, 0.0, 0.05), "rot1": ("Z", 15.0)},
)
_CHAIN_MASSES = (3.0, 2.0, 1.2)
_CHAIN_COMS = (
    np.array([0.08, -0.03, 0.05]),
    np.array([-0.04, 0.10, 0.02]),
    np.array([0.06, 0.01, -0.07]),
)
_CHAIN_INERTIAS = (
    np.diag([0.030, 0.040, 0.025]),
    np.diag([0.020, 0.015, 0.022]),
    np.diag([0.010, 0.012, 0.008]),
)


def _axis_unit(axis: str) -> "Gf.Vec3d":
    from pxr import Gf

    return {"X": Gf.Vec3d(1, 0, 0), "Y": Gf.Vec3d(0, 1, 0), "Z": Gf.Vec3d(0, 0, 1)}[axis]


def _local_matrix(pos, rot) -> "Gf.Matrix4d":
    from pxr import Gf

    matrix = Gf.Matrix4d(1.0)
    matrix.SetRotate(Gf.Rotation(_axis_unit(rot[0]), float(rot[1])))
    matrix.SetTranslateOnly(Gf.Vec3d(*[float(v) for v in pos]))
    return matrix


def _author_chain_stage(joints):
    """In-memory USD stage: base plus one revolute joint/link per joint spec.

    Args:
        joints: Value supplied for ``joints``.

    Returns:
        Result produced by the operation.
    """  # noqa: DOC106, DOC107
    from pxr import Gf, Usd, UsdPhysics

    stage = Usd.Stage.CreateInMemory()
    link_paths = ["/robot/base"] + [f"/robot/link{i + 1}" for i in range(len(joints))]
    for path in link_paths:
        stage.DefinePrim(path, "Xform")
    joint_paths = []
    for i, spec in enumerate(joints):
        joint = UsdPhysics.RevoluteJoint.Define(stage, f"/robot/joint{i + 1}")
        joint.CreateBody0Rel().SetTargets([link_paths[i]])
        joint.CreateBody1Rel().SetTargets([link_paths[i + 1]])
        joint.CreateAxisAttr(spec["axis"])
        joint.CreateLocalPos0Attr(Gf.Vec3f(*[float(v) for v in spec["pos0"]]))
        joint.CreateLocalPos1Attr(Gf.Vec3f(*[float(v) for v in spec["pos1"]]))
        rot0 = Gf.Rotation(_axis_unit(spec["rot0"][0]), float(spec["rot0"][1])).GetQuat()
        rot1 = Gf.Rotation(_axis_unit(spec["rot1"][0]), float(spec["rot1"][1])).GetQuat()
        joint.CreateLocalRot0Attr(Gf.Quatf(float(rot0.GetReal()), Gf.Vec3f(*rot0.GetImaginary())))
        joint.CreateLocalRot1Attr(Gf.Quatf(float(rot1.GetReal()), Gf.Vec3f(*rot1.GetImaginary())))
        joint_paths.append(str(joint.GetPrim().GetPath()))
    return stage, link_paths, joint_paths


def _chain_world_frames(joints, q):
    """World-frame FK from the authored joint data (independent of the USD read-back).

    Row-vector Gf convention: the child-to-parent point map is
    ``local1^-1 * R(q) * local0`` and cumulative products append on the right.

    Args:
        joints: Value supplied for ``joints``.
        q: Value supplied for ``q``.

    Returns:
        Result produced by the operation.
    """  # noqa: DOC106, DOC107
    from pxr import Gf

    body_rot = []
    com_world = []
    joint_world = []
    axis_world = []
    cum = Gf.Matrix4d(1.0)
    for i, spec in enumerate(joints):
        local0 = _local_matrix(spec["pos0"], spec["rot0"])
        local1 = _local_matrix(spec["pos1"], spec["rot1"])
        motion = Gf.Matrix4d(1.0).SetRotate(Gf.Rotation(_axis_unit(spec["axis"]), float(np.rad2deg(q[i]))))
        joint_in_world = local0 * cum
        joint_world.append(np.array(joint_in_world.Transform(Gf.Vec3d(0, 0, 0)), dtype=np.float64))
        axis = np.array(joint_in_world.TransformDir(_axis_unit(spec["axis"])), dtype=np.float64)
        axis_world.append(axis / np.linalg.norm(axis))
        cum = local1.GetInverse() * motion * local0 * cum
        arr = np.asarray(cum, dtype=np.float64).reshape(4, 4)
        body_rot.append(arr[:3, :3].T.copy())
        com_world.append(np.array(cum.Transform(Gf.Vec3d(*[float(v) for v in _CHAIN_COMS[i]])), dtype=np.float64))
    return body_rot, com_world, joint_world, axis_world


def _chain_reference_torque(joints, q, qd, qdd):
    """Independent world-frame recursive Newton-Euler over the authored chain.

    Mirrors the differentiable bridge's ``feedforward.compute_feedforward``
    sweep (world-frame rates, D'Alembert base acceleration of ``-g``,
    CoM-referenced inertias, axis-projected interbody wrenches) without
    needing a Newton ``Model``. With ``qd = qdd = 0`` it reduces to the
    moment-arm gravity sum.

    Args:
        joints: Value supplied for ``joints``.
        q: Value supplied for ``q``.
        qd: Value supplied for ``qd``.
        qdd: Value supplied for ``qdd``.

    Returns:
        Result produced by the operation.
    """  # noqa: DOC106, DOC107
    n = len(joints)
    body_rot, com_world, joint_world, axis_world = _chain_world_frames(joints, q)
    omega = [np.zeros(3)] * n
    alpha = [np.zeros(3)] * n
    acom = [np.zeros(3)] * n
    for i in range(n):
        w_p = omega[i - 1] if i > 0 else np.zeros(3)
        al_p = alpha[i - 1] if i > 0 else np.zeros(3)
        if i > 0:
            r_a = joint_world[i] - com_world[i - 1]
            a_anchor = acom[i - 1] + np.cross(al_p, r_a) + np.cross(w_p, np.cross(w_p, r_a))
        else:
            a_anchor = -_GRAVITY_VEC
        w_rel = axis_world[i] * qd[i]
        omega[i] = w_p + w_rel
        alpha[i] = al_p + axis_world[i] * qdd[i] + np.cross(w_p, w_rel)
        r_c = com_world[i] - joint_world[i]
        acom[i] = a_anchor + np.cross(alpha[i], r_c) + np.cross(omega[i], np.cross(omega[i], r_c))

    tau = np.zeros(n, dtype=np.float64)
    f_acc = np.zeros(3)
    n_acc = np.zeros(3)
    for i in range(n - 1, -1, -1):
        inertia_w = body_rot[i] @ _CHAIN_INERTIAS[i] @ body_rot[i].T
        torque_b = inertia_w @ alpha[i] + np.cross(omega[i], inertia_w @ omega[i])
        f_total = _CHAIN_MASSES[i] * acom[i] + f_acc
        n_total = torque_b + n_acc
        tau[i] = float(axis_world[i] @ (n_total + np.cross(com_world[i] - joint_world[i], f_total)))
        if i > 0:
            f_acc = f_total
            n_acc = n_total + np.cross(com_world[i] - com_world[i - 1], f_total)
    return tau


class TestUsdContextAgainstWorldNewtonEuler(omni.kit.test.AsyncTestCase):
    """USD-built compact RNEA vs an independent world-frame Newton-Euler reference.

    Regression for the >12 Nm RMS nominal-model disagreement between the
    compact RNEA (`build_fixed_base_context_from_usd` + `rnea_inverse_dynamics`)
    and the bridge feedforward (`feedforward.compute_feedforward`, validated at
    0.95 Nm RMS against real Franka telemetry). Locks the three conventions the
    USD path got wrong: (1) the Plucker ``xup`` lower-left block direction in
    ``matrix_to_spatial_motion_transform``, (2) the joint motion subspace
    expressed in the child *body* frame (rotated/offset ``local1``), and
    (3) origin-referenced spatial inertia (parallel-axis transport of the
    CoM-referenced baseline). The chain geometry authors rotated and offset
    joint frames on both sides specifically so each of the three is observable.
    """

    def _build_context(self):
        from isaacsim.robot_setup.sysid.analytical_presolve import (
            build_fixed_base_context_from_usd,
        )

        stage, link_paths, joint_paths = _author_chain_stage(_CHAIN_JOINTS)
        baseline = ParameterBaselineState(
            link_inertia_lc={
                idx + 1: inertia_matrix_to_log_cholesky(_CHAIN_INERTIAS[idx]) for idx in range(len(_CHAIN_JOINTS))
            },
            link_com={idx + 1: _CHAIN_COMS[idx].copy() for idx in range(len(_CHAIN_JOINTS))},
            link_masses={0: 5.0, **{idx + 1: _CHAIN_MASSES[idx] for idx in range(len(_CHAIN_JOINTS))}},
        )
        context = build_fixed_base_context_from_usd(stage, link_paths, joint_paths, baseline)
        self.assertIsNotNone(context)
        return context, _baseline_inertial_params(context, baseline, include_unmapped_subtrees=True)

    async def test_single_joint_gravity_matches_closed_form(self) -> None:
        """Anchor the sign convention: axis Y at the origin, CoM at +x => tau = -m*g*x."""
        from isaacsim.robot_setup.sysid.analytical_presolve import (
            build_fixed_base_context_from_usd,
        )

        joints = (
            {"axis": "Y", "pos0": (0.0, 0.0, 0.0), "rot0": ("X", 0.0), "pos1": (0.0, 0.0, 0.0), "rot1": ("X", 0.0)},
        )
        stage, link_paths, joint_paths = _author_chain_stage(joints)
        mass, com = 2.5, np.array([0.3, 0.0, 0.1])
        baseline = ParameterBaselineState(
            link_inertia_lc={1: inertia_matrix_to_log_cholesky(np.eye(3) * 0.01)},
            link_com={1: com},
            link_masses={0: 1.0, 1: mass},
        )
        context = build_fixed_base_context_from_usd(stage, link_paths, joint_paths, baseline)
        inertial = _baseline_inertial_params(context, baseline)
        tau = rnea_inverse_dynamics(context, np.zeros(1), np.zeros(1), np.zeros(1), inertial)
        self.assertAlmostEqual(float(tau[0]), -mass * 9.80665 * float(com[0]), places=9)

    async def test_context_rejects_non_topological_joint_order(self) -> None:
        from isaacsim.robot_setup.sysid.analytical_presolve import (
            build_fixed_base_context_from_usd,
        )

        stage, link_paths, joint_paths = _author_chain_stage(_CHAIN_JOINTS)
        baseline = ParameterBaselineState(
            link_inertia_lc={
                idx + 1: inertia_matrix_to_log_cholesky(_CHAIN_INERTIAS[idx]) for idx in range(len(_CHAIN_JOINTS))
            },
            link_com={idx + 1: _CHAIN_COMS[idx].copy() for idx in range(len(_CHAIN_JOINTS))},
            link_masses={0: 5.0, **{idx + 1: _CHAIN_MASSES[idx] for idx in range(len(_CHAIN_JOINTS))}},
        )

        context = build_fixed_base_context_from_usd(stage, link_paths, list(reversed(joint_paths)), baseline)

        self.assertIsNone(context)

    async def test_context_rejects_reversed_usd_joint_bodies(self) -> None:
        from isaacsim.robot_setup.sysid.analytical_presolve import (
            build_fixed_base_context_from_usd,
        )

        stage, link_paths, joint_paths = _author_chain_stage(_CHAIN_JOINTS[:1])
        joint = stage.GetPrimAtPath(joint_paths[0])
        joint.GetRelationship("physics:body0").SetTargets([link_paths[1]])
        joint.GetRelationship("physics:body1").SetTargets([link_paths[0]])
        baseline = ParameterBaselineState(
            link_inertia_lc={1: inertia_matrix_to_log_cholesky(_CHAIN_INERTIAS[0])},
            link_com={1: _CHAIN_COMS[0].copy()},
            link_masses={0: 5.0, 1: _CHAIN_MASSES[0]},
        )

        context = build_fixed_base_context_from_usd(stage, link_paths, joint_paths, baseline)

        self.assertIsNone(context)

    async def test_rotated_fixed_base_receives_gravity_in_base_frame(self) -> None:
        from isaacsim.robot_setup.sysid.analytical_presolve import (
            build_fixed_base_context_from_usd,
        )
        from pxr import UsdGeom

        joints = (
            {
                "axis": "Y",
                "pos0": (0.0, 0.0, 0.0),
                "rot0": ("X", 0.0),
                "pos1": (0.0, 0.0, 0.0),
                "rot1": ("X", 0.0),
            },
        )
        stage, link_paths, joint_paths = _author_chain_stage(joints)
        UsdGeom.Xformable(stage.GetPrimAtPath("/robot/base")).AddRotateYOp().Set(90.0)
        baseline = ParameterBaselineState(
            link_inertia_lc={1: inertia_matrix_to_log_cholesky(np.eye(3) * 0.01)},
            link_com={1: np.array([0.3, 0.0, 0.1])},
            link_masses={0: 1.0, 1: 2.5},
        )

        context = build_fixed_base_context_from_usd(stage, link_paths, joint_paths, baseline)

        np.testing.assert_allclose(
            context.gravity_acceleration(),
            np.asarray([0.0, 0.0, 0.0, 9.80665, 0.0, 0.0]),
            atol=1e-9,
        )

    async def test_gravity_torque_matches_world_frame_reference(self) -> None:
        context, inertial = self._build_context()
        rng = np.random.default_rng(11)
        poses = [np.zeros(3)] + [rng.uniform(-1.5, 1.5, 3) for _ in range(4)]
        zeros = np.zeros(3)
        for q in poses:
            tau = rnea_inverse_dynamics(context, q, zeros, zeros, inertial)
            expected = _chain_reference_torque(_CHAIN_JOINTS, q, zeros, zeros)
            np.testing.assert_allclose(tau, expected, atol=1e-5)
            # Joint 1's axis is vertical: gravity can exert no torque about it.
            self.assertLess(abs(float(tau[0])), 1e-6)

    async def test_full_inverse_dynamics_matches_world_frame_reference(self) -> None:
        context, inertial = self._build_context()
        rng = np.random.default_rng(23)
        for _ in range(4):
            q = rng.uniform(-1.5, 1.5, 3)
            qd = rng.uniform(-2.0, 2.0, 3)
            qdd = rng.uniform(-6.0, 6.0, 3)
            tau = rnea_inverse_dynamics(context, q, qd, qdd, inertial)
            expected = _chain_reference_torque(_CHAIN_JOINTS, q, qd, qdd)
            np.testing.assert_allclose(tau, expected, atol=1e-5)


class TestInverseDynamicsRegressorComCoupling(omni.kit.test.AsyncTestCase):
    """Analytic regressor columns vs finite differences of the exact theta path.

    Params rows 4-9 are origin-referenced inertia, so with a nonzero baseline CoM
    the mass-scale and CoM-offset columns couple into the inertia rows through the
    parallel-axis terms. The exact reference is ``_inertial_params_for_theta`` +
    RNEA -- the same path the presolve finite-differences -- evaluated at the
    baseline theta on the 2-DOF fixture (link 1 CoM = [0.12, 0, 0.04] != 0).
    """

    def setUp(self) -> None:
        self.baseline = _build_baseline()
        self.context = _build_two_dof_context(self.baseline)
        self.entries = [
            SysIdParameterEntry(SysIdParameterType.LINK_MASS, GLOBAL_DOF_INDEX, link_index=1),
            SysIdParameterEntry(SysIdParameterType.LINK_COM_OFFSET_X, GLOBAL_DOF_INDEX, link_index=1),
            SysIdParameterEntry(SysIdParameterType.LINK_COM_OFFSET_Y, GLOBAL_DOF_INDEX, link_index=1),
            SysIdParameterEntry(SysIdParameterType.LINK_COM_OFFSET_Z, GLOBAL_DOF_INDEX, link_index=1),
            # Component 2 is the Iyy log-diagonal (observable about the Y-axis joints);
            # component 0 (Ixx) is structurally unobservable on this planar arm and
            # checks that the analytic column agrees with the exact ~0 response.
            SysIdParameterEntry(
                SysIdParameterType.LINK_INERTIA_LOG_CHOLESKY, GLOBAL_DOF_INDEX, link_index=1, component_index=2
            ),
            SysIdParameterEntry(
                SysIdParameterType.LINK_INERTIA_LOG_CHOLESKY, GLOBAL_DOF_INDEX, link_index=1, component_index=0
            ),
        ]
        times = np.arange(_NUM_STEPS, dtype=np.float64) * _DT
        self.positions = np.stack(
            (0.6 * np.sin(1.7 * times) + 0.2, 0.8 * np.sin(1.3 * times + 0.3)),
            axis=1,
        )
        self.velocities = np.stack(
            (0.6 * 1.7 * np.cos(1.7 * times), 0.8 * 1.3 * np.cos(1.3 * times + 0.3)),
            axis=1,
        )
        self.accelerations = finite_difference_acceleration(times, self.velocities)
        # Baseline theta: unit mass scale, zero CoM deltas, baseline Log-Cholesky components.
        lc1 = self.baseline.link_inertia_lc[1]
        self.theta0 = np.array([1.0, 0.0, 0.0, 0.0, float(lc1[2]), float(lc1[0])], dtype=np.float64)

    def _exact_torque(self, theta: np.ndarray) -> np.ndarray:
        inertial = _inertial_params_for_theta(self.baseline, self.context, self.entries, theta)
        return np.stack(
            [
                rnea_inverse_dynamics(
                    self.context, self.positions[t], self.velocities[t], self.accelerations[t], inertial
                )
                for t in range(self.positions.shape[0])
            ]
        )

    async def test_analytic_columns_match_exact_theta_finite_differences(self) -> None:
        analytic = assemble_inverse_dynamics_regressor(
            self.positions,
            self.velocities,
            self.accelerations,
            self.entries,
            self.context,
            baseline_link_inertia_lc={1: self.baseline.link_inertia_lc[1]},
        )

        eps = 1e-3
        for col, entry in enumerate(self.entries):
            theta_plus = self.theta0.copy()
            theta_minus = self.theta0.copy()
            theta_plus[col] += eps
            theta_minus[col] -= eps
            fd_col = ((self._exact_torque(theta_plus) - self._exact_torque(theta_minus)) / (2.0 * eps)).reshape(-1)
            out_of_plane = entry.param_type == SysIdParameterType.LINK_COM_OFFSET_Y or (
                entry.param_type == SysIdParameterType.LINK_INERTIA_LOG_CHOLESKY and entry.component_index == 0
            )
            if not out_of_plane:
                # Out-of-plane columns on this planar arm are legitimately ~0.
                self.assertGreater(np.max(np.abs(fd_col)), 1e-2, msg=f"column {col} unexcited")
            np.testing.assert_allclose(
                analytic[:, col],
                fd_col,
                rtol=2e-3,
                atol=1e-4,
                err_msg=f"analytic column {col} ({entry.param_type}) disagrees with exact finite differences",
            )

    async def test_baseline_com_coupling_is_load_bearing(self) -> None:
        """Dropping ``link_coms`` (the pre-fix basis) must change the mass/CoM columns."""
        kwargs = {"baseline_link_inertia_lc": {1: self.baseline.link_inertia_lc[1]}}
        coupled = assemble_inverse_dynamics_regressor(
            self.positions, self.velocities, self.accelerations, self.entries, self.context, **kwargs
        )
        uncoupled = assemble_inverse_dynamics_regressor(
            self.positions,
            self.velocities,
            self.accelerations,
            self.entries,
            replace(self.context, link_coms=None),
            **kwargs,
        )
        for col in (0, 1, 3):  # mass scale, CoM x, CoM z
            self.assertGreater(np.max(np.abs(coupled[:, col] - uncoupled[:, col])), 1e-2)
