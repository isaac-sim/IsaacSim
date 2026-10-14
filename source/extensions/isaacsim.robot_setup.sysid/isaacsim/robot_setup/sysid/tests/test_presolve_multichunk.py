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

# ruff: noqa: ANN201, ANN202, D101, D102

"""Tests for multi-chunk analytical presolve inputs and the acceleration override."""

from __future__ import annotations

from unittest.mock import patch

import numpy as np
import omni.kit.test
import torch
from isaacsim.robot_setup.sysid.optimizer_config import (
    OptimizerBackend,
    OptimizerBackendConfig,
)
from isaacsim.robot_setup.sysid.parameter_space import ParameterSpace
from isaacsim.robot_setup.sysid.parameter_types import (
    SysIdParameterEntry,
    SysIdParameterType,
)
from isaacsim.robot_setup.sysid.regressor import finite_difference_acceleration
from isaacsim.robot_setup.sysid.run_controller import (
    _apply_analytical_presolve,
    _apply_multichunk_jacobian_policy,
    _presolve_training_inputs,
)
from isaacsim.robot_setup.sysid.run_spec import SysIdRunSpec
from isaacsim.robot_setup.sysid.trajectory_csv import TrajectoryDataset
from isaacsim.robot_setup.sysid.trajectory_segments import (
    TelemetryChunkRunSpec,
    TrajectoryChunk,
)


def _chunk(name: str, t0: float, steps: int, num_dof: int = 2, *, offset: float = 0.0, torques: bool = True):
    times = t0 + np.arange(steps) * 0.02
    q = offset + np.sin(times)[:, np.newaxis] * np.ones((1, num_dof))
    qd = np.cos(times)[:, np.newaxis] * np.ones((1, num_dof))
    trajectory = TrajectoryDataset(
        times=times,
        positions=q,
        velocities=qd,
        commands=q.copy(),
        torques=q.copy() if torques else None,
    )
    return TrajectoryChunk(
        spec=TelemetryChunkRunSpec(name=name, start=float(times[0]), end=float(times[-1])),
        trajectory=trajectory,
        sample_count=steps,
        duration_seconds=float(times[-1] - times[0]),
    )


class TestPresolveTrainingInputs(omni.kit.test.AsyncTestCase):
    async def test_boundary_accelerations_are_per_chunk(self):
        # A large positional jump between chunks: naive concatenated finite
        # differences would smear the discontinuity into the boundary rows.
        chunk_a = _chunk("a", 0.0, 40)
        chunk_b = _chunk("b", 10.0, 40, offset=2.5)
        stitched, accelerations, steps = _presolve_training_inputs([chunk_a, chunk_b], chunk_a.trajectory, 100)

        self.assertEqual(steps, 80)
        self.assertEqual(stitched.positions.shape, (80, 2))
        self.assertIsNotNone(stitched.torques)
        self.assertIsNotNone(accelerations)

        expected_a = finite_difference_acceleration(
            np.asarray(chunk_a.trajectory.times), np.asarray(chunk_a.trajectory.velocities)
        )
        expected_b = finite_difference_acceleration(
            np.asarray(chunk_b.trajectory.times), np.asarray(chunk_b.trajectory.velocities)
        )
        np.testing.assert_allclose(accelerations[:40], expected_a)
        np.testing.assert_allclose(accelerations[40:], expected_b)

        contaminated = finite_difference_acceleration(np.asarray(stitched.times), np.asarray(stitched.velocities))
        # Sanity: the naive concatenated estimate differs at the first row of the
        # second chunk, so the per-chunk computation is load-bearing rather than vacuous.
        self.assertFalse(np.allclose(contaminated[40], expected_b[0]))

    async def test_chunk_step_cap_and_missing_torques(self):
        chunk_a = _chunk("a", 0.0, 60)
        chunk_b = _chunk("b", 10.0, 60, torques=False)
        stitched, accelerations, steps = _presolve_training_inputs([chunk_a, chunk_b], chunk_a.trajectory, 50)
        self.assertEqual(steps, 100)
        self.assertEqual(accelerations.shape, (100, 2))
        self.assertIsNone(stitched.torques)

    async def test_no_chunks_passes_trajectory_through(self):
        trajectory = _chunk("a", 0.0, 30).trajectory
        stitched, accelerations, steps = _presolve_training_inputs([], trajectory, 25)
        self.assertIs(stitched, trajectory)
        self.assertIsNone(accelerations)
        self.assertEqual(steps, 25)


class TestPresolveBackendGate(omni.kit.test.AsyncTestCase):
    async def test_multichunk_analytical_jacobian_downgrade_is_reported(self):
        config = OptimizerBackendConfig(use_analytical_jacobian=True)

        with patch("carb.log_warn") as log_warn:
            _apply_multichunk_jacobian_policy(config, chunk_count=2)

        self.assertFalse(config.use_analytical_jacobian)
        log_warn.assert_called_once_with(
            "SysId: analytical_jacobian disabled for multi-chunk training; falling back to numerical LM."
        )

    def _run_backend(self, backend: OptimizerBackend):
        chunk = _chunk("a", 0.0, 40)
        entries = [
            SysIdParameterEntry(param_type=SysIdParameterType.JOINT_FRICTION, dof_index=0),
            SysIdParameterEntry(param_type=SysIdParameterType.JOINT_FRICTION, dof_index=1),
        ]
        theta = torch.ones(len(entries), dtype=torch.float32)
        space = ParameterSpace.for_robot(2)
        spec = SysIdRunSpec()
        return _apply_analytical_presolve(
            None,
            space,
            [],
            [],
            chunk.trajectory,
            [chunk],
            entries,
            theta,
            torch.zeros_like(theta),
            theta * 10.0,
            spec.residuals.to_config(),
            backend,
            max_rollout_steps=40,
        )

    async def test_gradient_descent_is_not_rejected(self):
        result = self._run_backend(OptimizerBackend.GRADIENT_DESCENT)
        self.assertFalse(any("applies only" in warning for warning in result.warnings))

    async def test_levenberg_marquardt_stays_rejected(self):
        result = self._run_backend(OptimizerBackend.LEVENBERG_MARQUARDT)
        self.assertTrue(any("applies only" in warning for warning in result.warnings))
