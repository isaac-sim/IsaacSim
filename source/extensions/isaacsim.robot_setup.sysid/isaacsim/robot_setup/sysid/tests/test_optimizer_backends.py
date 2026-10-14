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

"""Unit tests for SysID optimizer backend edge cases."""

from __future__ import annotations

import asyncio
import math
from contextlib import contextmanager
from dataclasses import dataclass
from unittest.mock import patch

import numpy as np
import omni.kit.test
import torch
from isaacsim.robot_setup.sysid.env_bridge import SysIdEnvironmentBridgeError
from isaacsim.robot_setup.sysid.lm_solver import (
    build_bounded_perturbed_theta_batch,
    compute_jacobian,
    compute_jacobian_finite_difference,
    levenberg_marquardt_step,
)
from isaacsim.robot_setup.sysid.optimizer_base import OptimizerConfig
from isaacsim.robot_setup.sysid.optimizer_config import (
    OptimizerBackend,
    OptimizerBackendConfig,
)
from isaacsim.robot_setup.sysid.optimizer_factory import create_optimizer
from isaacsim.robot_setup.sysid.optimizers.bayesian import BayesianOptimizer
from isaacsim.robot_setup.sysid.optimizers.cma_es import CmaEsOptimizer
from isaacsim.robot_setup.sysid.optimizers.cma_es_core import (
    CmaEsState,
    _compute_weights,
    tell,
)
from isaacsim.robot_setup.sysid.optimizers.cost_utils import penalize_nonfinite_costs
from isaacsim.robot_setup.sysid.optimizers.gp_core import (
    expected_improvement_numpy,
    gp_posterior,
    standardize_observations,
)
from isaacsim.robot_setup.sysid.optimizers.levenberg_marquardt import (
    LevenbergMarquardtOptimizer,
    _parameters_support_constant_analytical_jacobian,
    _scale_analytical_jacobian_for_residual,
)
from isaacsim.robot_setup.sysid.optimizers.parameter_transform import (
    BoxParameterTransform,
)
from isaacsim.robot_setup.sysid.parameter_space import ParameterSpace
from isaacsim.robot_setup.sysid.parameter_types import (
    SysIdParameterEntry,
    SysIdParameterType,
)
from isaacsim.robot_setup.sysid.regressor import (
    estimate_analytical_jacobian,
    finite_difference_acceleration,
)
from isaacsim.robot_setup.sysid.residual_config import ResidualWeightConfig
from isaacsim.robot_setup.sysid.residual_engine import (
    ResidualSignalType,
    ResidualSpec,
    RolloutResidualInputs,
    WeightedResidualEngine,
)
from isaacsim.robot_setup.sysid.rollout_optimizer_base import RolloutOptimizerBase
from isaacsim.robot_setup.sysid.rollout_plot_data import SysIdRolloutPlotData
from isaacsim.robot_setup.sysid.rollout_result import SysIdRolloutResult
from isaacsim.robot_setup.sysid.runtime import set_yield_hook
from isaacsim.robot_setup.sysid.trajectory_csv import TrajectoryDataset
from isaacsim.robot_setup.sysid.trajectory_segments import (
    TelemetryChunkRunSpec,
    TrajectoryChunk,
)


@contextmanager
def _patched_kit_app():
    async def yield_once() -> None:
        return None

    set_yield_hook(yield_once)
    try:
        yield
    finally:
        set_yield_hook(None)


def _trajectory(sample_count: int = 2) -> TrajectoryDataset:
    values = np.zeros((sample_count, 1), dtype=np.float64)
    return TrajectoryDataset(
        times=np.arange(sample_count, dtype=np.float64),
        positions=values.copy(),
        velocities=values.copy(),
        commands=values.copy(),
    )


def _config(
    *,
    theta_initial: float = 0.0,
    max_iterations: int = 3,
    backend_config: OptimizerBackendConfig | None = None,
    trajectory: TrajectoryDataset | None = None,
    training_chunks: list[TrajectoryDataset] | None = None,
) -> OptimizerConfig:
    return OptimizerConfig(
        trajectory=trajectory or _trajectory(),
        param_entries=[SysIdParameterEntry(SysIdParameterType.JOINT_FRICTION, 0)],
        theta_initial=torch.tensor([theta_initial], dtype=torch.float32),
        theta_min=torch.tensor([0.0], dtype=torch.float32),
        theta_max=torch.tensor([1.0], dtype=torch.float32),
        epsilon=1e-4,
        damping_initial=1e-2,
        max_iterations=max_iterations,
        max_rollout_steps=2,
        backend_config=backend_config,
        training_chunks=training_chunks,
    )


class _ThetaBridge:
    device = torch.device("cpu")
    supports_arbitrary_candidate_batch = True

    def __init__(self, *, nonfinite_initial: bool = False, all_nonfinite: bool = False) -> None:
        self.nonfinite_initial = nonfinite_initial
        self.all_nonfinite = all_nonfinite
        self.batches: list[torch.Tensor] = []
        self.signal_requests: list[tuple[bool, bool, bool]] = []
        self.set_trajectory_calls = 0

    def configure_rollout_signals(
        self,
        *,
        torque: bool,
        end_effector_pose: bool,
        contact_force: bool,
    ) -> None:
        self.signal_requests.append((torque, end_effector_pose, contact_force))

    def set_trajectory(self, trajectory: TrajectoryDataset) -> None:
        self.set_trajectory_calls += 1
        self.trajectory = trajectory

    async def run_rollout_async(
        self,
        theta_per_env: torch.Tensor,
        param_entries: list[SysIdParameterEntry],
        commands: torch.Tensor,
        num_steps: int,
    ) -> SysIdRolloutResult:
        theta = theta_per_env.detach().to(dtype=torch.float32).clone()
        self.batches.append(theta.cpu())
        num_envs = int(theta.shape[0])
        positions = theta[:, 0].reshape(num_envs, 1, 1).expand(num_envs, num_steps, 1).clone()
        if self.all_nonfinite:
            positions[:] = torch.nan
        if self.nonfinite_initial:
            positions[torch.isclose(theta[:, 0], torch.zeros((), dtype=theta.dtype)), :, :] = torch.nan
        velocities = torch.zeros_like(positions)
        collect_torque = bool(self.signal_requests and self.signal_requests[-1][0])
        return SysIdRolloutResult(
            positions=positions,
            velocities=velocities,
            torques=torch.zeros_like(positions) if collect_torque else None,
        )

    def cleanup(self, remove_clones: bool = True) -> None:
        _ = remove_clones


class _OutOfBoundsParameterSpace:
    def reparameterize_inertia_theta(
        self,
        theta: torch.Tensor,
        param_entries: list[SysIdParameterEntry],
    ) -> torch.Tensor:
        del param_entries
        return torch.full_like(theta, 2.0)


class _CrossCoupledRepairSpace:
    def reparameterize_inertia_theta(
        self,
        theta: torch.Tensor,
        param_entries: list[SysIdParameterEntry],
    ) -> torch.Tensor:
        del param_entries
        repaired = theta.clone()
        repaired[1 if not torch.isclose(theta[0], torch.tensor(0.5)) else 0] += 0.1
        return repaired


@dataclass(frozen=True)
class _LinkSnapshot:
    mass: float
    com_offset: np.ndarray
    diagonal_inertia: np.ndarray
    inertia_matrix: np.ndarray | None


class _ExtraEnvironmentBridge(_ThetaBridge):
    async def run_rollout_async(self, theta_per_env, param_entries, commands, num_steps):
        result = await super().run_rollout_async(theta_per_env, param_entries, commands, num_steps)
        return SysIdRolloutResult(
            positions=result.positions.repeat(2, 1, 1),
            velocities=result.velocities.repeat(2, 1, 1),
            torques=result.torques.repeat(2, 1, 1) if result.torques is not None else None,
        )


class _WrongJointWidthBridge(_ThetaBridge):
    async def run_rollout_async(self, theta_per_env, param_entries, commands, num_steps):
        result = await super().run_rollout_async(theta_per_env, param_entries, commands, num_steps)
        return SysIdRolloutResult(
            positions=result.positions[..., :1],
            velocities=result.velocities[..., :1],
            torques=result.torques[..., :1] if result.torques is not None else None,
        )


class _PartialTorqueBridge:
    device = torch.device("cpu")
    supports_arbitrary_candidate_batch = False
    use_parallel_clones = False

    def __init__(self) -> None:
        self.calls = 0

    def configure_rollout_signals(self, **_kwargs: object) -> None:
        return None

    def set_trajectory(self, trajectory: TrajectoryDataset) -> None:
        self.trajectory = trajectory

    async def run_rollout_async(
        self,
        theta_per_env: torch.Tensor,
        param_entries: list[SysIdParameterEntry],
        commands: torch.Tensor,
        num_steps: int,
    ) -> SysIdRolloutResult:
        self.calls += 1
        values = torch.zeros((1, num_steps, 1), dtype=torch.float32)
        torques = torch.ones_like(values) if self.calls == 1 else None
        return SysIdRolloutResult(positions=values, velocities=values, torques=torques)

    def cleanup(self, remove_clones: bool = True) -> None:
        _ = remove_clones


class _DeterministicCmaEs(CmaEsOptimizer):
    async def _evaluate_population_in_batches(
        self, config, transform, candidates_norm, *, batch_size, device, on_progress=None
    ):
        self._last_candidates = np.asarray(candidates_norm, dtype=np.float64)
        costs = torch.tensor([10.0, 9.0, 0.0, 8.0], dtype=torch.float32)
        return costs[: candidates_norm.shape[0]], SysIdRolloutPlotData(
            times=[0.0],
            measured_positions=[[0.0]],
            simulated_positions=[[-999.0]],
            measured_velocities=[[0.0]],
            simulated_velocities=[[0.0]],
        )

    async def compute_costs_for_theta_batch(self, config, theta_batch, on_progress=None, include_plot_data=True):
        theta_value = float(theta_batch[0, 0].detach().cpu().item())
        plot = (
            SysIdRolloutPlotData(
                times=[0.0],
                measured_positions=[[0.0]],
                simulated_positions=[[theta_value]],
                measured_velocities=[[0.0]],
                simulated_velocities=[[0.0]],
            )
            if include_plot_data
            else None
        )
        costs = theta_batch[:, 0].detach().cpu().to(dtype=torch.float32)
        residuals = costs.reshape(-1, 1)
        return costs, residuals, plot


class _AllNonfiniteCmaEs(_DeterministicCmaEs):
    async def _evaluate_population_in_batches(
        self, config, transform, candidates_norm, *, batch_size, device, on_progress=None
    ):
        return torch.full((candidates_norm.shape[0],), torch.nan), None


class OptimizerBackendTests(omni.kit.test.AsyncTestCase):
    async def test_stochastic_backend_seeds_round_trip_through_config(self) -> None:
        config = OptimizerBackendConfig(bo_seed=17, gd_seed=29)

        restored = OptimizerBackendConfig.from_dict(config.to_dict())

        self.assertEqual(restored.bo_seed, 17)
        self.assertEqual(restored.gd_seed, 29)

    async def test_rollout_rejects_joint_width_that_would_broadcast(self) -> None:
        values = np.zeros((2, 2), dtype=np.float64)
        trajectory = TrajectoryDataset(
            times=np.asarray([0.0, 1.0]),
            positions=values.copy(),
            velocities=values.copy(),
            commands=values.copy(),
        )
        optimizer = RolloutOptimizerBase()
        optimizer.set_bridge(_WrongJointWidthBridge())

        with (
            _patched_kit_app(),
            self.assertRaisesRegex(
                SysIdEnvironmentBridgeError,
                r"positions must have shape \(1, 2, 2\)",
            ),
        ):
            await optimizer.compute_costs_for_theta_batch(
                _config(trajectory=trajectory),
                torch.tensor([[0.5]], dtype=torch.float32),
                include_plot_data=False,
            )

    async def test_validation_configures_signals_and_rejects_extra_environments(self) -> None:
        trajectory = _trajectory(sample_count=2)
        trajectory.torques = np.zeros_like(trajectory.positions)
        config = _config(trajectory=trajectory)
        config.residual_weight_config = ResidualWeightConfig(
            position_weight=0.0,
            velocity_weight=0.0,
            torque_weight=1.0,
        )
        chunk = TrajectoryChunk(
            spec=TelemetryChunkRunSpec(
                name="validation",
                role="validation",
                start=0.0,
                end=1.0,
            ),
            trajectory=trajectory,
            sample_count=2,
            duration_seconds=1.0,
        )
        bridge = _ExtraEnvironmentBridge()
        optimizer = RolloutOptimizerBase()
        optimizer.set_bridge(bridge)

        with (
            _patched_kit_app(),
            self.assertRaisesRegex(
                SysIdEnvironmentBridgeError,
                r"leading shape \(1, 2\)",
            ),
        ):
            await optimizer.evaluate_validation_chunk(config, chunk, [0.5])

        self.assertEqual(bridge.signal_requests, [(True, False, False)])

    async def test_paired_validation_batches_parameter_sets_per_chunk(self) -> None:
        trajectory = _trajectory(sample_count=2)
        config = _config(trajectory=trajectory)
        chunk = TrajectoryChunk(
            spec=TelemetryChunkRunSpec(
                name="validation",
                role="validation",
                start=0.0,
                end=1.0,
            ),
            trajectory=trajectory,
            sample_count=2,
            duration_seconds=1.0,
        )
        bridge = _ThetaBridge()
        optimizer = RolloutOptimizerBase()
        optimizer.set_bridge(bridge)

        with _patched_kit_app():
            grouped = await optimizer.evaluate_validation_parameter_sets_with_rollouts(
                config,
                [chunk],
                [[0.25], [0.75]],
            )

        self.assertEqual(len(bridge.batches), 1)
        torch.testing.assert_close(bridge.batches[0], torch.tensor([[0.25], [0.75]]))
        self.assertEqual([len(group) for group in grouped], [1, 1])

    async def test_paired_validation_honors_optimizer_cancellation_before_rollout(self) -> None:
        trajectory = _trajectory(sample_count=2)
        config = _config(trajectory=trajectory)
        chunk = TrajectoryChunk(
            spec=TelemetryChunkRunSpec(
                name="validation",
                role="validation",
                start=0.0,
                end=1.0,
            ),
            trajectory=trajectory,
            sample_count=2,
            duration_seconds=1.0,
        )
        bridge = _ThetaBridge()
        optimizer = RolloutOptimizerBase()
        optimizer.set_bridge(bridge)

        def cancel_during_progress(_message: str, _fraction: float) -> None:
            optimizer.request_cancel()

        with _patched_kit_app(), self.assertRaises(asyncio.CancelledError):
            await optimizer.evaluate_validation_parameter_sets_with_rollouts(
                config,
                [chunk],
                [[0.25], [0.75]],
                on_progress=cancel_during_progress,
            )

        self.assertEqual(bridge.batches, [])

    async def test_sequential_rollout_rejects_partial_optional_signal_batches(self) -> None:
        """Do not broadcast one candidate's optional output onto another candidate."""
        trajectory = _trajectory(sample_count=2)
        trajectory.torques = np.zeros_like(trajectory.positions)
        config = _config(trajectory=trajectory)
        config.residual_weight_config = ResidualWeightConfig(
            position_weight=0.0,
            velocity_weight=0.0,
            torque_weight=1.0,
        )
        optimizer = RolloutOptimizerBase()
        optimizer.set_bridge(_PartialTorqueBridge())

        with (
            _patched_kit_app(),
            self.assertRaisesRegex(
                SysIdEnvironmentBridgeError,
                "torques for 1 of 2 candidates",
            ),
        ):
            await optimizer.compute_costs_for_theta_batch(
                config,
                torch.tensor([[0.0], [1.0]], dtype=torch.float32),
                include_plot_data=False,
            )

    async def test_lm_rejects_nonfinite_nominal_rollout(self) -> None:
        """Fail clearly rather than returning a completed status with NaN cost."""
        optimizer = LevenbergMarquardtOptimizer()
        optimizer.set_bridge(_ThetaBridge(nonfinite_initial=True))

        with (
            _patched_kit_app(),
            self.assertRaisesRegex(
                SysIdEnvironmentBridgeError,
                "nominal rollout produced non-finite residuals",
            ),
        ):
            await optimizer.run_async(_config(theta_initial=0.0, max_iterations=1))

    async def test_weighted_optional_residual_requires_requested_signal(self) -> None:
        zeros = torch.zeros((1, 1), dtype=torch.float64)
        cases = (
            (ResidualSignalType.TORQUE, "Torque"),
            (ResidualSignalType.END_EFFECTOR_POSE, "End-effector pose"),
            (ResidualSignalType.CONTACT_FORCE, "Contact-force"),
        )
        for signal, message in cases:
            with self.subTest(signal=signal), self.assertRaisesRegex(ValueError, message):
                WeightedResidualEngine((ResidualSpec(signal),)).compute(
                    RolloutResidualInputs(
                        meas_pos=zeros,
                        meas_vel=zeros,
                        sim_pos=zeros.unsqueeze(0),
                        sim_vel=zeros.unsqueeze(0),
                    )
                )

    async def test_gp_standardization_preserves_acquisition_under_affine_cost_scale(self) -> None:
        x_train = np.asarray([[0.0], [0.5], [1.0]], dtype=np.float64)
        x_query = np.asarray([[0.1], [0.35], [0.8]], dtype=np.float64)
        costs = np.asarray([10.0, 4.0, 8.0], dtype=np.float64)

        standardized, _, _ = standardize_observations(costs)
        scaled, _, _ = standardize_observations(costs * 1000.0 + 123.0)
        np.testing.assert_allclose(standardized, scaled)

        mu, sigma = gp_posterior(x_train, standardized, x_query)
        ei = expected_improvement_numpy(mu, sigma, float(np.min(standardized)))
        self.assertTrue(np.all(np.isfinite(mu)))
        self.assertTrue(np.all(np.isfinite(sigma)))
        self.assertTrue(np.all(np.isfinite(ei)))
        self.assertGreater(float(np.max(ei) - np.min(ei)), 0.0)

    async def test_gp_posterior_retries_cholesky_with_adaptive_jitter(self) -> None:
        original_cholesky = np.linalg.cholesky
        calls = 0

        def flaky_cholesky(matrix):
            nonlocal calls
            calls += 1
            if calls < 3:
                raise np.linalg.LinAlgError("ill-conditioned")
            return original_cholesky(matrix)

        with patch(
            "isaacsim.robot_setup.sysid.optimizers.gp_core.np.linalg.cholesky",
            side_effect=flaky_cholesky,
        ):
            mu, sigma = gp_posterior(
                np.asarray([[0.0], [0.0]], dtype=np.float64),
                np.asarray([1.0, 1.0], dtype=np.float64),
                np.asarray([[0.5]], dtype=np.float64),
            )

        self.assertEqual(calls, 3)
        self.assertTrue(np.all(np.isfinite(mu)))
        self.assertTrue(np.all(np.isfinite(sigma)))

    async def test_expected_improvement_uses_accurate_normal_cdf(self) -> None:
        mu = np.asarray([-3.0], dtype=np.float64)
        sigma = np.asarray([1.0], dtype=np.float64)

        actual = expected_improvement_numpy(mu, sigma, 0.0, xi=0.0)

        z = 3.0
        expected = z * 0.5 * (1.0 + math.erf(z / math.sqrt(2.0))) + math.exp(-0.5 * z * z) / math.sqrt(2.0 * math.pi)
        self.assertAlmostEqual(float(actual[0]), expected, places=12)

    async def test_optimizer_factory_rejects_unresolved_auto_backend(self) -> None:
        with self.assertRaisesRegex(ValueError, "resolved"):
            create_optimizer(OptimizerBackend.AUTO)

    async def test_real_cma_tell_moves_mean_toward_low_cost_candidates(self) -> None:
        state = CmaEsState.create(1, initial_mean=np.asarray([0.5]), sigma=0.2)
        original_mean = state.mean.copy()
        original_covariance = state.covariance.copy()
        candidates = np.asarray([[0.1], [0.2], [0.8], [0.9]], dtype=np.float64)
        costs = np.asarray([0.0, 1.0, 10.0, 20.0], dtype=np.float64)

        updated = tell(state, candidates, costs, population_size=4)

        self.assertEqual(updated.generation, 1)
        self.assertIsNot(updated, state)
        self.assertEqual(state.generation, 0)
        np.testing.assert_array_equal(state.mean, original_mean)
        np.testing.assert_array_equal(state.covariance, original_covariance)
        self.assertLess(float(updated.mean[0]), 0.5)
        self.assertTrue(np.all(np.linalg.eigvalsh(updated.covariance) > 0.0))

    async def test_cma_state_rejects_invalid_dimension_and_sigma(self) -> None:
        with self.assertRaisesRegex(ValueError, "dimension"):
            CmaEsState.create(0, initial_mean=np.asarray([]), sigma=0.2)
        for sigma in (0.0, -0.1, np.nan, np.inf):
            with self.subTest(sigma=sigma):
                with self.assertRaisesRegex(ValueError, "finite positive"):
                    CmaEsState.create(1, initial_mean=np.asarray([0.5]), sigma=sigma)

    async def test_cma_stalled_path_retains_covariance_correction(self) -> None:
        state = CmaEsState.create(1, initial_mean=np.asarray([0.0]), sigma=1.0)
        state.evolution_path_sigma[:] = 10.0
        candidates = np.zeros((4, 1), dtype=np.float64)
        costs = np.arange(4, dtype=np.float64)
        _, _, mueff = _compute_weights(4)
        c_c = (4.0 + mueff) / (5.0 + 2.0 * mueff)
        c_1 = 2.0 / (2.3**2 + mueff)
        c_mu = min(1.0 - c_1, 2.0 * (mueff - 2.0 + 1.0 / mueff) / (3.0**2 + mueff))
        expected = 1.0 - c_1 - c_mu + c_1 * c_c * (2.0 - c_c)

        updated = tell(state, candidates, costs, population_size=4)

        self.assertAlmostEqual(float(updated.covariance[0, 0]), expected, places=12)

    async def test_box_parameter_transform_round_trips_physical_values(self) -> None:
        theta_min = torch.tensor([-2.0, 10.0], dtype=torch.float32)
        theta_max = torch.tensor([2.0, 30.0], dtype=torch.float32)
        transform = BoxParameterTransform.from_bounds(theta_min, theta_max)
        physical = np.asarray([[-1.0, 15.0], [1.5, 28.0]], dtype=np.float64)

        normalized = transform.physical_to_normalized(physical)
        recovered = transform.normalized_to_theta_batch(normalized, device=torch.device("cpu"))

        np.testing.assert_allclose(recovered.numpy(), physical, atol=1e-6)

    async def test_end_effector_quaternion_residual_is_sign_invariant(self) -> None:
        """Treat opposite-sign quaternions as the same physical orientation."""
        zeros = torch.zeros((1, 1), dtype=torch.float64)
        measured_pose = torch.tensor([[0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0]], dtype=torch.float64)
        simulated_pose = torch.tensor(
            [[[0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -1.0]]],
            dtype=torch.float64,
        )
        engine = WeightedResidualEngine((ResidualSpec(ResidualSignalType.END_EFFECTOR_POSE),))

        residuals, costs = engine.compute(
            RolloutResidualInputs(
                meas_pos=zeros,
                meas_vel=zeros,
                sim_pos=zeros.unsqueeze(0),
                sim_vel=zeros.unsqueeze(0),
                meas_ee_pose=measured_pose,
                sim_ee_pose=simulated_pose,
            )
        )

        torch.testing.assert_close(residuals, torch.zeros_like(residuals))
        torch.testing.assert_close(costs, torch.zeros_like(costs))

    async def test_end_effector_quaternion_residual_rejects_zero_norm(self) -> None:
        """Reject corrupt standard-pose telemetry instead of scoring it as a match."""
        valid_pose = torch.tensor([[[0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0]]], dtype=torch.float64)
        invalid_pose = valid_pose.clone()
        invalid_pose[..., 3:7] = 0.0

        with self.assertRaisesRegex(ValueError, "Measured end-effector pose"):
            WeightedResidualEngine._end_effector_pose_difference(valid_pose, invalid_pose)

    async def test_cancellation_is_not_returned_as_success(self) -> None:
        optimizer = CmaEsOptimizer()
        optimizer.set_bridge(_ThetaBridge())
        config = _config(
            max_iterations=1,
            backend_config=OptimizerBackendConfig(cma_population_size=4),
        )

        def cancel_on_progress(_message: str, _fraction: float) -> None:
            optimizer.request_cancel()

        with _patched_kit_app():
            with self.assertRaises(asyncio.CancelledError):
                await optimizer.run_async(config, on_progress=cancel_on_progress)

    async def test_bayesian_evaluates_initial_theta_first_and_keeps_matching_rollout(self) -> None:
        optimizer = BayesianOptimizer()
        bridge = _ThetaBridge()
        optimizer.set_bridge(bridge)
        config = _config(
            theta_initial=0.0,
            backend_config=OptimizerBackendConfig(bo_initial_samples=2, bo_batch_size=2),
        )

        with _patched_kit_app():
            status = await optimizer.run_async(config)

        self.assertAlmostEqual(float(bridge.batches[0][0, 0]), 0.0)
        self.assertAlmostEqual(status.theta[0], 0.0)
        self.assertIsNotNone(status.rollout)
        self.assertAlmostEqual(status.rollout.simulated_positions[0][0], status.theta[0])

    async def test_bayesian_ignores_nonfinite_observations_and_recovers(self) -> None:
        optimizer = BayesianOptimizer()
        optimizer.set_bridge(_ThetaBridge(nonfinite_initial=True))
        config = _config(
            theta_initial=0.0,
            max_iterations=4,
            backend_config=OptimizerBackendConfig(bo_initial_samples=1, bo_batch_size=2),
        )

        with _patched_kit_app():
            status = await optimizer.run_async(config)

        self.assertTrue(np.isfinite(status.cost))
        self.assertTrue(np.isfinite(status.theta[0]))

    async def test_bayesian_rejects_run_without_finite_observation(self) -> None:
        optimizer = BayesianOptimizer()
        optimizer.set_bridge(_ThetaBridge(all_nonfinite=True))
        config = _config(
            max_iterations=2,
            backend_config=OptimizerBackendConfig(bo_initial_samples=1, bo_batch_size=2),
        )

        with (
            _patched_kit_app(),
            self.assertRaisesRegex(
                SysIdEnvironmentBridgeError,
                "did not produce a finite rollout cost",
            ),
        ):
            await optimizer.run_async(config)

    async def test_cma_rejects_run_without_finite_observation(self) -> None:
        optimizer = _AllNonfiniteCmaEs()
        optimizer.set_bridge(_ThetaBridge())
        config = _config(
            max_iterations=1,
            backend_config=OptimizerBackendConfig(cma_population_size=4),
        )

        with (
            _patched_kit_app(),
            self.assertRaisesRegex(
                SysIdEnvironmentBridgeError,
                "did not produce a finite rollout cost",
            ),
        ):
            await optimizer.run_async(config)

    async def test_bayesian_seed_reproduces_candidate_batches(self) -> None:
        configs = [
            _config(
                max_iterations=5,
                backend_config=OptimizerBackendConfig(
                    bo_initial_samples=3,
                    bo_batch_size=2,
                    bo_candidate_count=8,
                    bo_seed=37,
                ),
            )
            for _ in range(2)
        ]
        bridges = [_ThetaBridge(), _ThetaBridge()]

        with _patched_kit_app():
            for config, bridge in zip(configs, bridges):
                optimizer = BayesianOptimizer()
                optimizer.set_bridge(bridge)
                await optimizer.run_async(config)

        self.assertEqual(len(bridges[0].batches), len(bridges[1].batches))
        for first, second in zip(bridges[0].batches, bridges[1].batches):
            torch.testing.assert_close(first, second)

    async def test_cma_sends_population_as_one_arbitrary_candidate_batch(self) -> None:
        optimizer = CmaEsOptimizer()
        bridge = _ThetaBridge()
        optimizer.set_bridge(bridge)
        config = _config(
            max_iterations=1,
            backend_config=OptimizerBackendConfig(
                cma_population_size=4,
                cma_batch_size=4,
            ),
        )

        with _patched_kit_app():
            await optimizer.run_async(config)

        self.assertEqual(tuple(bridge.batches[0].shape), (4, 1))

    async def test_cma_refreshes_status_rollout_for_best_nonzero_candidate(self) -> None:
        optimizer = _DeterministicCmaEs()
        optimizer.set_bridge(_ThetaBridge())
        config = _config(
            max_iterations=1,
            backend_config=OptimizerBackendConfig(cma_population_size=4),
        )

        with _patched_kit_app():
            status = await optimizer.run_async(config)

        self.assertTrue(status.accepted)
        self.assertIsNotNone(status.rollout)
        self.assertNotEqual(status.rollout.simulated_positions[0][0], -999.0)
        self.assertAlmostEqual(status.rollout.simulated_positions[0][0], status.theta[0])

    async def test_bayesian_theta_fallback_joins_first_batch(self) -> None:
        # With a presolve seed in theta_initial, the pre-presolve initials
        # (theta_fallback) must be observed in the first batch so the incumbent
        # best keeps the better start.
        optimizer = BayesianOptimizer()
        bridge = _ThetaBridge()
        optimizer.set_bridge(bridge)
        config = _config(
            theta_initial=0.5,
            max_iterations=2,
            backend_config=OptimizerBackendConfig(bo_initial_samples=2, bo_batch_size=2),
        )
        config.theta_fallback = torch.tensor([0.25], dtype=torch.float32)

        with _patched_kit_app():
            status = await optimizer.run_async(config)

        first_batch = bridge.batches[0]
        self.assertEqual(int(first_batch.shape[0]), 2)
        self.assertAlmostEqual(float(first_batch[0, 0]), 0.5, places=5)
        self.assertAlmostEqual(float(first_batch[1, 0]), 0.25, places=5)
        # Cost equals theta here, so the fallback must beat the seed.
        self.assertAlmostEqual(status.theta[0], 0.25, places=5)

    async def test_cma_theta_fallback_enters_first_generation(self) -> None:
        optimizer = _DeterministicCmaEs()
        optimizer.set_bridge(_ThetaBridge())
        config = _config(
            theta_initial=0.5,
            max_iterations=1,
            backend_config=OptimizerBackendConfig(cma_population_size=4),
        )
        config.theta_fallback = torch.tensor([0.25], dtype=torch.float32)

        with _patched_kit_app():
            await optimizer.run_async(config)

        # Bounds are [0, 1], so normalized coordinates equal physical values:
        # slot 0 is the seed mean, slot 1 must carry the fallback.
        self.assertAlmostEqual(float(optimizer._last_candidates[0][0]), 0.5, places=5)
        self.assertAlmostEqual(float(optimizer._last_candidates[1][0]), 0.25, places=5)

    async def test_cma_initial_mean_is_clamped_to_normalized_box(self) -> None:
        optimizer = _DeterministicCmaEs()
        optimizer.set_bridge(_ThetaBridge())
        config = _config(
            theta_initial=2.0,
            max_iterations=1,
            backend_config=OptimizerBackendConfig(cma_population_size=4),
        )

        with _patched_kit_app():
            await optimizer.run_async(config)

        self.assertAlmostEqual(float(optimizer._last_candidates[0][0]), 1.0, places=5)

    async def test_explicit_single_training_chunk_matches_plain_objective(self) -> None:
        trajectory = _trajectory(sample_count=2)
        optimizer = RolloutOptimizerBase()
        optimizer.set_bridge(_ThetaBridge())
        plain_config = _config(theta_initial=1.0, trajectory=trajectory)
        chunk_config = _config(theta_initial=1.0, trajectory=trajectory, training_chunks=[trajectory])

        with _patched_kit_app():
            plain_costs, _, _ = await optimizer.compute_costs_for_theta_batch(
                plain_config,
                torch.tensor([[1.0]], dtype=torch.float32),
                include_plot_data=False,
            )
            chunk_costs, _, _ = await optimizer.compute_costs_for_theta_batch(
                chunk_config,
                torch.tensor([[1.0]], dtype=torch.float32),
                include_plot_data=False,
            )

        torch.testing.assert_close(chunk_costs, plain_costs)

    async def test_rollout_requests_weighted_optional_signals(self) -> None:
        trajectory = _trajectory(sample_count=2)
        trajectory.torques = np.zeros_like(trajectory.positions)
        bridge = _ThetaBridge()
        optimizer = RolloutOptimizerBase()
        optimizer.set_bridge(bridge)
        config = _config(trajectory=trajectory)
        config.residual_weight_config = ResidualWeightConfig(
            torque_weight=1.0,
            end_effector_pose_weight=0.0,
            contact_force_weight=0.0,
        )

        with _patched_kit_app():
            await optimizer.compute_costs_for_theta_batch(
                config,
                torch.tensor([[0.5]], dtype=torch.float32),
                include_plot_data=False,
            )

        self.assertEqual(bridge.signal_requests, [(True, False, False)])

    async def test_plot_data_collection_does_not_write_debug_pngs(self) -> None:
        optimizer = RolloutOptimizerBase()
        optimizer.set_bridge(_ThetaBridge())
        config = _config(theta_initial=1.0)

        with (
            patch("isaacsim.robot_setup.sysid.rollout_optimizer_base.save_clone_joint_angles_debug_png") as angles,
            patch("isaacsim.robot_setup.sysid.rollout_optimizer_base.save_clone_joint_efforts_debug_png") as efforts,
            _patched_kit_app(),
        ):
            await optimizer.compute_costs_for_theta_batch(
                config,
                torch.tensor([[1.0]], dtype=torch.float32),
                include_plot_data=True,
            )

        angles.assert_not_called()
        efforts.assert_not_called()

    async def test_bounded_lm_perturbations_stay_in_bounds_and_zero_fixed_columns(self) -> None:
        theta = torch.tensor([0.95, 1.0, 0.5], dtype=torch.float32)
        theta_min = torch.tensor([0.0, 0.0, 0.5], dtype=torch.float32)
        theta_max = torch.tensor([1.0, 1.0, 0.5], dtype=torch.float32)

        batch, steps = build_bounded_perturbed_theta_batch(theta, 0.1, theta_min, theta_max)

        self.assertTrue(torch.all(batch >= theta_min))
        self.assertTrue(torch.all(batch <= theta_max))
        self.assertAlmostEqual(float(steps[0].item()), 0.05, places=6)
        self.assertAlmostEqual(float(steps[1].item()), -0.1, places=6)
        self.assertAlmostEqual(float(steps[2].item()), 0.0, places=6)

        jacobian = compute_jacobian_finite_difference(
            torch.tensor([0.0, 0.0]),
            torch.tensor([[1.0, 1.0], [2.0, 2.0]]),
            0.1,
            column_steps=torch.tensor([0.5, 0.0]),
        )

        torch.testing.assert_close(jacobian[:, 0], torch.tensor([2.0, 2.0]))
        torch.testing.assert_close(jacobian[:, 1], torch.tensor([0.0, 0.0]))

    async def test_lm_skips_fd_columns_changed_by_coupled_reparameterization(self) -> None:
        theta = torch.tensor([0.5, 0.5], dtype=torch.float32)
        entries = [
            SysIdParameterEntry(SysIdParameterType.LINK_INERTIA_LOG_CHOLESKY, -1, link_index=0, component_index=0),
            SysIdParameterEntry(SysIdParameterType.LINK_INERTIA_LOG_CHOLESKY, -1, link_index=0, component_index=1),
        ]

        batch, steps = build_bounded_perturbed_theta_batch(
            theta,
            0.1,
            torch.zeros(2),
            torch.ones(2),
            parameter_space=_CrossCoupledRepairSpace(),
            param_entries=entries,
        )

        torch.testing.assert_close(batch, theta.expand(3, 2))
        torch.testing.assert_close(steps, torch.zeros(2))

    async def test_lm_reclamps_trial_after_parameter_space_reparameterization(self) -> None:
        bridge = _ThetaBridge()
        optimizer = LevenbergMarquardtOptimizer()
        optimizer.set_bridge(bridge)
        optimizer.set_parameter_space(_OutOfBoundsParameterSpace())

        with _patched_kit_app():
            await optimizer.run_async(_config(theta_initial=0.5, max_iterations=1))

        self.assertTrue(bridge.batches)
        self.assertTrue(all(bool(torch.all(batch <= 1.0)) for batch in bridge.batches))

    async def test_lm_reports_both_linear_solver_failures(self) -> None:
        with (
            patch("torch.linalg.solve", side_effect=RuntimeError("direct failure")),
            patch("torch.linalg.lstsq", side_effect=RuntimeError("fallback failure")),
            self.assertRaisesRegex(RuntimeError, "linear solve and least-squares fallback both failed"),
        ):
            levenberg_marquardt_step(torch.ones((1, 1)), torch.ones(1), 0.1)

    async def test_acceleration_uses_second_order_boundary_difference(self) -> None:
        times = np.asarray([0.0, 1.0, 2.0], dtype=np.float64)
        velocities = (times**2).reshape(-1, 1)

        acceleration = finite_difference_acceleration(times, velocities)

        np.testing.assert_allclose(acceleration[:, 0], [0.0, 2.0, 4.0])

    async def test_segmented_training_rejects_invalid_weights(self) -> None:
        optimizer = RolloutOptimizerBase()
        for weight in (0.0, -1.0, np.nan, np.inf):
            config = _config(training_chunks=[_trajectory()])
            config.training_chunk_weights = [weight]
            segments = optimizer._training_segments(config)
            with self.subTest(weight=weight):
                with self.assertRaisesRegex(ValueError, "finite positive"):
                    optimizer._normalized_training_weights(segments)

    async def test_analytical_jacobian_uses_exact_residual_row_scaling(self) -> None:
        jacobian = torch.ones((4, 1), dtype=torch.float32)

        scaled = _scale_analytical_jacobian_for_residual(
            jacobian,
            sample_weights=torch.tensor([0.5, 2.0]),
            torque_weight=4.0,
            num_steps=2,
            num_dof=2,
        )

        torch.testing.assert_close(scaled[:, 0], torch.tensor([1.0, 1.0, 4.0, 4.0]))

    async def test_analytical_jacobian_rejects_row_mismatch(self) -> None:
        with self.assertRaisesRegex(ValueError, "rows"):
            compute_jacobian(
                torch.zeros(3),
                None,
                1e-4,
                analytical_jacobian=torch.ones((2, 1)),
                parameter_count=1,
            )

    async def test_nonlinear_parameters_do_not_use_constant_analytical_path(self) -> None:
        config = _config()
        config.param_entries = [
            SysIdParameterEntry(
                SysIdParameterType.LINK_INERTIA_LOG_CHOLESKY,
                -1,
                link_index=0,
                component_index=0,
            )
        ]

        self.assertFalse(_parameters_support_constant_analytical_jacobian(config))

    async def test_lm_falls_back_to_finite_difference_for_nonlinear_parameters(self) -> None:
        trajectory = _trajectory(sample_count=2)
        trajectory.torques = np.zeros_like(trajectory.positions)
        config = _config(
            max_iterations=1,
            trajectory=trajectory,
            backend_config=OptimizerBackendConfig(use_analytical_jacobian=True),
        )
        config.use_analytical_jacobian = True
        config.residual_weight_config = ResidualWeightConfig(
            position_weight=0.0,
            velocity_weight=0.0,
            torque_weight=1.0,
        )
        config.param_entries = [
            SysIdParameterEntry(
                SysIdParameterType.LINK_INERTIA_LOG_CHOLESKY,
                -1,
                link_index=0,
                component_index=0,
            )
        ]
        bridge = _ThetaBridge()
        optimizer = LevenbergMarquardtOptimizer()
        optimizer.set_bridge(bridge)

        with _patched_kit_app():
            await optimizer.run_async(config)

        self.assertEqual(tuple(bridge.batches[0].shape), (2, 1))

    async def test_inertia_derivative_is_not_added_directly_to_torque_rows(self) -> None:
        entry = SysIdParameterEntry(
            SysIdParameterType.LINK_INERTIA_LOG_CHOLESKY,
            -1,
            link_index=0,
            component_index=0,
        )
        with (
            patch(
                "isaacsim.robot_setup.sysid.regressor.finite_difference_acceleration",
                return_value=np.zeros((1, 1), dtype=np.float64),
            ),
            patch(
                "isaacsim.robot_setup.sysid.regressor.assemble_inverse_dynamics_regressor",
                return_value=np.asarray([[2.0]], dtype=np.float64),
            ),
            patch(
                "isaacsim.robot_setup.sysid.regressor.assemble_joint_regressor",
                return_value=np.asarray([[0.0]], dtype=np.float64),
            ),
            patch("isaacsim.robot_setup.sysid.regressor.inertia_log_cholesky_parameter_jacobian") as raw_inertia,
        ):
            jacobian = estimate_analytical_jacobian(
                np.zeros((1, 1), dtype=np.float64),
                np.zeros((1, 1), dtype=np.float64),
                np.zeros(1, dtype=np.float64),
                [entry],
                num_links=1,
                baseline_link_inertia_lc={0: np.zeros(6, dtype=np.float64)},
                residual_dim=1,
                dynamics_context=object(),
            )

        raw_inertia.assert_not_called()
        torch.testing.assert_close(jacobian, torch.tensor([[2.0]]))

    async def test_analytical_jacobian_without_context_keeps_only_aligned_joint_columns(self) -> None:
        entries = [
            SysIdParameterEntry(SysIdParameterType.LINK_MASS, -1, link_index=0),
            SysIdParameterEntry(SysIdParameterType.JOINT_FRICTION, 0),
        ]
        jacobian = estimate_analytical_jacobian(
            np.zeros((2, 1), dtype=np.float64),
            np.asarray([[0.0], [1.0]], dtype=np.float64),
            np.asarray([0.0, 1.0], dtype=np.float64),
            entries,
            num_links=1,
            residual_dim=2,
        )

        torch.testing.assert_close(jacobian, torch.tensor([[0.0, 0.0], [0.0, -1.0]]))

    async def test_joint_scale_jacobian_includes_authored_baselines(self) -> None:
        entries = [
            SysIdParameterEntry(SysIdParameterType.JOINT_FRICTION, 0),
            SysIdParameterEntry(SysIdParameterType.JOINT_DAMPING, 0),
            SysIdParameterEntry(SysIdParameterType.JOINT_STIFFNESS, 0),
        ]

        jacobian = estimate_analytical_jacobian(
            np.asarray([[1.0], [-1.0]], dtype=np.float64),
            np.asarray([[-2.0], [3.0]], dtype=np.float64),
            np.asarray([0.0, 1.0], dtype=np.float64),
            entries,
            num_links=1,
            residual_dim=2,
            commands=np.zeros((2, 1), dtype=np.float64),
            baseline_joint_friction={0: 4.0},
            baseline_joint_damping={0: 5.0},
            baseline_joint_stiffness={0: 6.0},
        )

        torch.testing.assert_close(
            jacobian,
            torch.tensor([[4.0, 10.0, -6.0], [-4.0, -15.0, 6.0]]),
        )

    async def test_bridge_and_tensor_caches_invalidate_after_in_place_trajectory_mutation(self) -> None:
        bridge = _ThetaBridge()
        optimizer = RolloutOptimizerBase()
        optimizer.set_bridge(bridge)
        config = _config()
        theta = torch.tensor([[0.5]], dtype=torch.float32)

        with _patched_kit_app():
            await optimizer.compute_costs_for_theta_batch(config, theta)
            await optimizer.compute_costs_for_theta_batch(config, theta)
            config.trajectory.positions[0, 0] = 1.0
            await optimizer.compute_costs_for_theta_batch(config, theta)

        self.assertEqual(bridge.set_trajectory_calls, 2)

    async def test_bridge_setup_rebinds_when_trajectory_tail_changes(self) -> None:
        # The bridge derives its physics substep rate from every sample, so a
        # chunk sharing the consumed prefix but not the tail must be rebound.
        bridge = _ThetaBridge()
        optimizer = RolloutOptimizerBase()
        optimizer.set_bridge(bridge)
        config = _config(trajectory=_trajectory(sample_count=4))
        theta = torch.tensor([[0.5]], dtype=torch.float32)

        with _patched_kit_app():
            await optimizer.compute_costs_for_theta_batch(config, theta)
            config.trajectory.times[3] = 7.0
            await optimizer.compute_costs_for_theta_batch(config, theta)

        self.assertEqual(bridge.set_trajectory_calls, 2)
        self.assertEqual(float(bridge.trajectory.times[3]), 7.0)

    async def test_per_run_residual_config_does_not_mutate_optimizer_defaults(self) -> None:
        bridge = _ThetaBridge()
        optimizer = RolloutOptimizerBase()
        optimizer.set_bridge(bridge)
        config = _config()
        config.residual_weight_config = ResidualWeightConfig(position_weight=2.0, velocity_weight=1.0)

        with (
            _patched_kit_app(),
            patch.object(optimizer, "set_residual_config", wraps=optimizer.set_residual_config) as setter,
        ):
            await optimizer.compute_costs_for_theta_batch(config, torch.tensor([[0.5]], dtype=torch.float32))

        setter.assert_not_called()

    async def test_nonfinite_cost_penalty_only_when_some_costs_are_finite(self) -> None:
        penalized, finite = penalize_nonfinite_costs(np.asarray([1.0, np.nan, np.inf]))
        self.assertEqual(finite.tolist(), [True, False, False])
        self.assertTrue(np.all(np.isfinite(penalized)))
        self.assertGreater(penalized[1], penalized[0])

        all_bad, all_bad_finite = penalize_nonfinite_costs(np.asarray([np.nan, np.inf]))
        self.assertEqual(all_bad_finite.tolist(), [False, False])
        self.assertFalse(np.any(np.isfinite(all_bad)))

    async def test_reparameterize_inertia_theta_preserves_device(self) -> None:
        space = ParameterSpace.for_robot_extended(
            1,
            1,
            include_basic=False,
            include_inertia_log_cholesky=True,
        )
        entries = space.entries()
        theta = torch.zeros(len(entries), dtype=torch.float32)

        updated = space.reparameterize_inertia_theta(theta, entries)

        self.assertEqual(updated.device, theta.device)

    async def test_reparameterize_inertia_theta_repairs_nonfinite_spd_decode(self) -> None:
        space = ParameterSpace.for_robot_extended(
            1,
            1,
            include_basic=False,
            include_inertia_log_cholesky=True,
        )
        space.baseline.link_inertia_lc[0] = np.zeros(6, dtype=np.float64)
        entries = space.entries()
        theta = torch.full((len(entries),), 1000.0, dtype=torch.float32)

        updated = space.reparameterize_inertia_theta(theta, entries)

        torch.testing.assert_close(updated, torch.zeros_like(updated))

    async def test_runtime_inertia_baselines_preserve_link_dimension(self) -> None:
        expected_coms = np.asarray([[0.1, 0.2, 0.3], [0.4, 0.5, 0.6]], dtype=np.float64)
        expected_inertias = np.asarray(
            [
                np.diag([1.0, 2.0, 3.0]),
                np.diag([4.0, 5.0, 6.0]),
            ],
            dtype=np.float64,
        )

        for inertias in (expected_inertias, expected_inertias[None, ...]):
            with self.subTest(shape=inertias.shape):
                space = ParameterSpace.for_robot_extended(1, 2)
                space.baseline.usd_link_snapshots = [
                    _LinkSnapshot(
                        mass=1.0,
                        com_offset=np.zeros(3, dtype=np.float64),
                        diagonal_inertia=np.ones(3, dtype=np.float64),
                        inertia_matrix=np.eye(3, dtype=np.float64),
                    )
                    for _ in range(2)
                ]

                space.override_link_runtime_baselines(
                    coms=expected_coms[None, ...],
                    inertias=inertias,
                )

                for link_index, snapshot in enumerate(space.baseline.usd_link_snapshots):
                    np.testing.assert_allclose(snapshot.com_offset, expected_coms[link_index])
                    np.testing.assert_allclose(snapshot.inertia_matrix, expected_inertias[link_index])
