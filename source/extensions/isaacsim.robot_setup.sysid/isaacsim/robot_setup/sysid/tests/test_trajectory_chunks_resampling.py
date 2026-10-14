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

"""Unit tests for SysID telemetry chunking and rollout resampling helpers."""

from __future__ import annotations

import numpy as np
import omni.kit.test
from isaacsim.robot_setup.sysid.trajectory_csv import TrajectoryDataset
from isaacsim.robot_setup.sysid.trajectory_csv_core import (
    TrajectoryCsvError,
    infer_telemetry_layout_from_columns,
)
from isaacsim.robot_setup.sysid.trajectory_resampling import (
    SamplingConfig,
    combine_sample_weights,
    resample_trajectory_for_rollout,
)
from isaacsim.robot_setup.sysid.trajectory_segments import (
    ChunkValidationMetric,
    TelemetryChunkRunSpec,
    TrajectorySegmentError,
    build_auto_chunks,
    build_trajectory_chunks,
    build_validation_comparison,
    resolve_chunk_specs,
    split_train_validation_chunks,
    summarize_validation_metrics,
)


def _trajectory(*, sample_count: int = 4, num_joints: int = 1) -> TrajectoryDataset:
    times = np.arange(sample_count, dtype=np.float64) * 0.1
    positions = np.arange(sample_count * num_joints, dtype=np.float64).reshape(sample_count, num_joints)
    return TrajectoryDataset(
        times=times,
        positions=positions,
        velocities=np.zeros_like(positions),
        commands=positions.copy(),
    )


class TrajectoryChunkingAndResamplingTests(omni.kit.test.AsyncTestCase):
    """Tests for trajectory chunk validation and rollout resampling."""

    async def test_chunked_trajectory_times_are_float64(self) -> None:
        """Chunked trajectories normalize timestamps to float64."""
        trajectory = TrajectoryDataset(
            times=np.asarray([0.0, 0.1, 0.2], dtype=np.float32),
            positions=np.zeros((3, 1), dtype=np.float64),
            velocities=np.zeros((3, 1), dtype=np.float64),
            commands=np.zeros((3, 1), dtype=np.float64),
        )

        chunks = build_trajectory_chunks(
            trajectory,
            [TelemetryChunkRunSpec(name="train", role="train", excitation="step", start=0.0, end=0.2)],
        )

        self.assertEqual(chunks[0].trajectory.times.dtype, np.float64)

    async def test_chunk_validation_rejects_train_validation_overlap(self) -> None:
        """Reject overlapping train and validation chunk windows."""
        with self.assertRaisesRegex(TrajectorySegmentError, "share telemetry samples"):
            build_trajectory_chunks(
                _trajectory(sample_count=5),
                [
                    TelemetryChunkRunSpec(name="train", role="train", start=0.0, end=0.2),
                    TelemetryChunkRunSpec(name="val", role="validation", start=0.2, end=0.4),
                ],
            )

    async def test_chunk_validation_allows_abutting_same_role_phases(self) -> None:
        """Allow same-role metadata phases to share an inclusive boundary sample."""
        chunks = build_trajectory_chunks(
            _trajectory(sample_count=5),
            [
                TelemetryChunkRunSpec(name="phase 1", role="train", start=0.0, end=0.2),
                TelemetryChunkRunSpec(name="phase 2", role="train", start=0.2, end=0.4),
            ],
        )

        self.assertEqual([chunk.sample_count for chunk in chunks], [3, 3])

    async def test_chunk_validation_allows_train_validation_sample_gap(self) -> None:
        """Allow adjacent train and validation phases when they share no sample."""
        chunks = build_trajectory_chunks(
            _trajectory(sample_count=5),
            [
                TelemetryChunkRunSpec(name="train", role="train", start=0.0, end=0.2),
                TelemetryChunkRunSpec(name="validation", role="validation", start=0.3, end=0.4),
            ],
        )

        self.assertEqual([chunk.sample_count for chunk in chunks], [3, 2])

    async def test_auto_and_resolved_chunks_follow_precedence(self) -> None:
        """Build a sample-safe holdout and preserve explicit chunk precedence."""
        trajectory = _trajectory(sample_count=10)
        automatic = build_auto_chunks(trajectory, train_fraction=0.6, min_chunk_seconds=0.1)

        self.assertEqual([spec.role for spec in automatic], ["train", "validation"])
        self.assertLess(automatic[0].end, automatic[1].start)
        explicit = [TelemetryChunkRunSpec(name="manual", start=0.1, end=0.4)]
        self.assertEqual(resolve_chunk_specs(trajectory, explicit, auto_split=True), explicit)
        windowed = resolve_chunk_specs(trajectory, None, window=(0.2, 0.6))
        self.assertEqual((windowed[0].start, windowed[0].end), (0.2, 0.6))

    async def test_chunk_validation_rejects_nonfinite_values(self) -> None:
        """Reject non-finite windows and weights before optimizer construction."""
        trajectory = _trajectory()
        for field_name, value in (
            ("start", float("nan")),
            ("end", float("inf")),
            ("weight", float("nan")),
            ("weight", float("inf")),
        ):
            kwargs = {"start": 0.0, "end": 0.3, "weight": 1.0}
            kwargs[field_name] = value
            with self.subTest(field_name=field_name, value=value):
                with self.assertRaisesRegex(TrajectorySegmentError, "finite"):
                    build_trajectory_chunks(trajectory, [TelemetryChunkRunSpec(**kwargs)])

    async def test_header_layout_rejects_reordered_signal_columns(self) -> None:
        """Do not silently interpret velocity or command columns as positions."""
        data = np.zeros((2, 10), dtype=np.float64)
        header = ["time", "q1", "q3", "dq1", "dq2", "dq3", "cmd1", "cmd2", "cmd3", "q2"]

        with self.assertRaisesRegex(TrajectoryCsvError, "contiguous, ordered"):
            infer_telemetry_layout_from_columns(data, header)

    async def test_split_and_validation_summary_group_by_role_and_excitation(
        self,
    ) -> None:
        """Split chunks by role and summarize validation metrics by excitation."""
        chunks = build_trajectory_chunks(
            _trajectory(sample_count=5),
            [
                TelemetryChunkRunSpec(name="train", role="train", excitation="sine", start=0.0, end=0.1),
                TelemetryChunkRunSpec(name="val", role="validation", excitation="step", start=0.3, end=0.4),
            ],
        )

        train, validation = split_train_validation_chunks(chunks)
        self.assertEqual([chunk.spec.name for chunk in train], ["train"])
        self.assertEqual([chunk.spec.name for chunk in validation], ["val"])

        summary = summarize_validation_metrics(
            [ChunkValidationMetric("val", "validation", "step", 0.3, 0.4, 2, 1.0, 0.5, 0.1, 0.2)]
        )
        self.assertIn("step", summary)
        self.assertIn("qRMSE", summary)

    async def test_validation_comparison_pairs_chunks_and_pools_rmse_by_sample_count(self) -> None:
        """Pool paired validation RMSE using each chunk's sample count."""
        untuned = [
            ChunkValidationMetric("a", "validation", "step", 0.0, 1.0, 1, 1.0, 1.0, 1.0, 2.0),
            ChunkValidationMetric("b", "validation", "sine", 1.0, 2.0, 3, 1.0, 1.0, 3.0, 4.0),
        ]
        tuned = [
            ChunkValidationMetric("a", "validation", "step", 0.0, 1.0, 1, 1.0, 1.0, 0.5, 1.0),
            ChunkValidationMetric("b", "validation", "sine", 1.0, 2.0, 3, 1.0, 1.0, 1.5, 2.0),
        ]

        comparison = build_validation_comparison(untuned, tuned)

        self.assertTrue(comparison["available"])
        self.assertEqual(comparison["chunk_count"], 2)
        self.assertAlmostEqual(comparison["overall"]["position_rmse"]["untuned"], (7.0**0.5))
        self.assertAlmostEqual(comparison["overall"]["position_rmse"]["tuned"], (1.75**0.5))
        self.assertAlmostEqual(comparison["overall"]["position_rmse"]["percent_improvement"], 50.0)

    async def test_validation_comparison_handles_zero_and_mismatched_baselines(self) -> None:
        """Handle zero RMSE and reject mismatched validation chunk identities."""
        zero = ChunkValidationMetric("a", "validation", "step", 0.0, 1.0, 2, 1.0, 1.0, 0.0, 0.0)
        tuned = ChunkValidationMetric("a", "validation", "step", 0.0, 1.0, 2, 1.0, 1.0, 0.1, 0.0)
        comparison = build_validation_comparison([zero], [tuned])
        self.assertIsNone(comparison["overall"]["position_rmse"]["percent_improvement"])
        self.assertEqual(comparison["overall"]["position_rmse"]["status"], "regressed")

        mismatched = ChunkValidationMetric("other", "validation", "step", 0.0, 1.0, 2, 1.0, 1.0, 0.1, 0.0)
        comparison = build_validation_comparison([zero], [mismatched])
        self.assertFalse(comparison["available"])

    async def test_validation_comparison_ignores_sub_threshold_rmse_noise(self) -> None:
        """Do not label numerical noise as a meaningful holdout regression."""
        untuned = ChunkValidationMetric("a", "validation", "step", 0.0, 1.0, 2, 1.0, 1.0, 1.0, 1.0)
        tuned = ChunkValidationMetric("a", "validation", "step", 0.0, 1.0, 2, 1.0, 1.0, 1.005, 1.0)

        comparison = build_validation_comparison(
            [untuned],
            [tuned],
            source="configured_chunks",
            holdout=True,
        )

        self.assertEqual(comparison["overall"]["position_rmse"]["status"], "unchanged")
        self.assertTrue(comparison["holdout"])
        self.assertEqual(comparison["source"], "configured_chunks")

    async def test_validation_comparison_refuses_verdict_when_a_chunk_diverges(self) -> None:
        """A chunk that diverges only after tuning must not be pooled away as an improvement."""
        untuned = [
            ChunkValidationMetric("chunk_a", "validation", "step", 0.0, 1.0, 10, 1.0, 1.0, 2.0, 1.0),
            ChunkValidationMetric("chunk_b", "validation", "sine", 1.0, 2.0, 10, 1.0, 1.0, 2.0, 1.0),
        ]
        # `chunk_b` blew up after tuning, so its tuned RMSE is non-finite.
        tuned = [
            ChunkValidationMetric("chunk_a", "validation", "step", 0.0, 1.0, 10, 1.0, 1.0, 0.5, 1.0),
            ChunkValidationMetric("chunk_b", "validation", "sine", 1.0, 2.0, 10, 1.0, 1.0, float("inf"), 1.0),
        ]

        comparison = build_validation_comparison(untuned, tuned)

        channel = comparison["overall"]["position_rmse"]
        self.assertEqual(channel["status"], "unavailable")
        self.assertIsNone(channel["percent_improvement"])
        self.assertEqual(channel["unpaired_chunks"], ["chunk_b"])

    async def test_validation_comparison_pools_over_the_common_chunk_set(self) -> None:
        """A channel absent from both passes is skipped without flagging a divergence."""
        untuned = [ChunkValidationMetric("chunk_a", "validation", "step", 0.0, 1.0, 4, 1.0, 1.0, 2.0, None)]
        tuned = [ChunkValidationMetric("chunk_a", "validation", "step", 0.0, 1.0, 4, 1.0, 1.0, 1.0, None)]

        comparison = build_validation_comparison(untuned, tuned)

        self.assertEqual(comparison["overall"]["position_rmse"]["unpaired_chunks"], [])
        self.assertAlmostEqual(comparison["overall"]["position_rmse"]["percent_improvement"], 50.0)

    async def test_resampling_interpolates_measured_state_and_holds_commands(
        self,
    ) -> None:
        """Interpolate measured signals while holding commands by default."""
        trajectory = TrajectoryDataset(
            times=np.asarray([0.0, 1.0], dtype=np.float64),
            positions=np.asarray([[0.0], [10.0]], dtype=np.float64),
            velocities=np.asarray([[0.0], [2.0]], dtype=np.float64),
            commands=np.asarray([[0.0], [100.0]], dtype=np.float64),
            torques=np.asarray([[1.0], [3.0]], dtype=np.float64),
            end_effector_poses=np.asarray([[0.0, 0.0], [2.0, 4.0]], dtype=np.float64),
            contact_forces=np.asarray([[5.0], [9.0]], dtype=np.float64),
        )

        result = resample_trajectory_for_rollout(
            trajectory,
            physics_dt=0.25,
            config=SamplingConfig(),
            max_rollout_steps=10,
        )

        self.assertTrue(result.diagnostic.enabled)
        self.assertEqual(result.diagnostic.reason, "low_rate")
        np.testing.assert_allclose(result.trajectory.times, [0.0, 0.25, 0.5, 0.75, 1.0])
        np.testing.assert_allclose(result.trajectory.positions[:, 0], [0.0, 2.5, 5.0, 7.5, 10.0])
        np.testing.assert_allclose(result.trajectory.velocities[:, 0], [0.0, 0.5, 1.0, 1.5, 2.0])
        np.testing.assert_allclose(result.trajectory.commands[:, 0], [0.0, 0.0, 0.0, 0.0, 100.0])
        np.testing.assert_allclose(result.trajectory.torques[:, 0], [1.0, 1.5, 2.0, 2.5, 3.0])
        np.testing.assert_allclose(result.trajectory.end_effector_poses[:, 1], [0.0, 1.0, 2.0, 3.0, 4.0])
        np.testing.assert_allclose(result.trajectory.contact_forces[:, 0], [5.0, 6.0, 7.0, 8.0, 9.0])
        self.assertIsNotNone(result.trajectory.residual_sample_weights)
        self.assertLess(float(result.trajectory.residual_sample_weights[2]), 1.0)

    async def test_resampling_can_linearly_interpolate_commands(self) -> None:
        """Allow command interpolation to be switched to linear mode."""
        trajectory = TrajectoryDataset(
            times=np.asarray([0.0, 1.0], dtype=np.float64),
            positions=np.asarray([[0.0], [1.0]], dtype=np.float64),
            velocities=np.zeros((2, 1), dtype=np.float64),
            commands=np.asarray([[0.0], [100.0]], dtype=np.float64),
        )

        result = resample_trajectory_for_rollout(
            trajectory,
            physics_dt=0.25,
            config=SamplingConfig(command_interpolation="linear"),
            max_rollout_steps=10,
        )

        np.testing.assert_allclose(result.trajectory.commands[:, 0], [0.0, 25.0, 50.0, 75.0, 100.0])

    async def test_resampling_uses_sign_corrected_slerp_for_quaternions(self) -> None:
        """Keep equivalent opposite-sign quaternion endpoints on one rotation."""
        trajectory = TrajectoryDataset(
            times=np.asarray([0.0, 1.0], dtype=np.float64),
            positions=np.zeros((2, 1), dtype=np.float64),
            velocities=np.zeros((2, 1), dtype=np.float64),
            commands=np.zeros((2, 1), dtype=np.float64),
            end_effector_poses=np.asarray(
                [
                    [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0],
                    [1.0, 0.0, 0.0, 0.0, 0.0, 0.0, -1.0],
                ],
                dtype=np.float64,
            ),
        )

        result = resample_trajectory_for_rollout(
            trajectory,
            physics_dt=0.25,
            config=SamplingConfig(),
            max_rollout_steps=10,
        )

        poses = result.trajectory.end_effector_poses
        np.testing.assert_allclose(poses[:, 0], [0.0, 0.25, 0.5, 0.75, 1.0])
        np.testing.assert_allclose(np.linalg.norm(poses[:, 3:7], axis=1), np.ones(5))
        np.testing.assert_allclose(np.abs(poses[:, 6]), np.ones(5))

    async def test_quaternion_resampling_rejects_nonmonotonic_timestamps(self) -> None:
        """Reject duplicate or decreasing timestamps before SLERP interval division."""
        for times in (
            np.asarray([0.0, 0.0, 1.0], dtype=np.float64),
            np.asarray([0.0, 1.0, 0.5], dtype=np.float64),
        ):
            with self.subTest(times=times.tolist()):
                trajectory = TrajectoryDataset(
                    times=times,
                    positions=np.zeros((3, 1), dtype=np.float64),
                    velocities=np.zeros((3, 1), dtype=np.float64),
                    commands=np.zeros((3, 1), dtype=np.float64),
                    end_effector_poses=np.asarray(
                        [
                            [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0],
                            [0.5, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0],
                            [1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0],
                        ],
                        dtype=np.float64,
                    ),
                )

                with self.assertRaisesRegex(ValueError, "strictly increasing"):
                    resample_trajectory_for_rollout(
                        trajectory,
                        physics_dt=0.25,
                        config=SamplingConfig(mode="always"),
                        max_rollout_steps=10,
                    )

    async def test_resampling_off_preserves_recorded_rows(self) -> None:
        """Leave recorded rows and weights untouched when resampling is disabled."""
        trajectory = TrajectoryDataset(
            times=np.asarray([0.0, 1.0], dtype=np.float64),
            positions=np.asarray([[0.0], [1.0]], dtype=np.float64),
            velocities=np.zeros((2, 1), dtype=np.float64),
            commands=np.asarray([[0.0], [1.0]], dtype=np.float64),
        )

        result = resample_trajectory_for_rollout(
            trajectory,
            physics_dt=0.01,
            config=SamplingConfig(mode="off"),
            max_rollout_steps=10,
        )

        self.assertFalse(result.diagnostic.enabled)
        self.assertEqual(result.diagnostic.reason, "disabled")
        np.testing.assert_allclose(result.trajectory.times, trajectory.times)
        self.assertIsNone(result.trajectory.residual_sample_weights)

    async def test_resampling_caps_physics_dt_grid_to_rollout_budget(self) -> None:
        """Cap the physics-dt target grid to the rollout step budget."""
        trajectory = TrajectoryDataset(
            times=np.asarray([0.0, 1.0], dtype=np.float64),
            positions=np.asarray([[0.0], [1.0]], dtype=np.float64),
            velocities=np.zeros((2, 1), dtype=np.float64),
            commands=np.asarray([[0.0], [1.0]], dtype=np.float64),
        )

        result = resample_trajectory_for_rollout(
            trajectory,
            physics_dt=0.1,
            config=SamplingConfig(mode="always", max_resampled_steps=4),
            max_rollout_steps=20,
        )

        self.assertTrue(result.diagnostic.capped)
        self.assertEqual(result.diagnostic.resampled_sample_count, 4)
        np.testing.assert_allclose(result.trajectory.times, [0.0, 0.1, 0.2, 0.3])

    async def test_resampling_uncapped_grid_matches_physics_steps(self) -> None:
        """Keep every output row aligned with one fixed physics step."""
        trajectory = TrajectoryDataset(
            times=np.asarray([0.0, 1.05], dtype=np.float64),
            positions=np.asarray([[0.0], [1.0]], dtype=np.float64),
            velocities=np.zeros((2, 1), dtype=np.float64),
            commands=np.asarray([[0.0], [1.0]], dtype=np.float64),
        )

        result = resample_trajectory_for_rollout(
            trajectory,
            physics_dt=0.1,
            config=SamplingConfig(mode="always"),
            max_rollout_steps=20,
        )

        self.assertFalse(result.diagnostic.capped)
        self.assertEqual(float(result.diagnostic.target_dt), 0.1)
        self.assertEqual(result.diagnostic.resampled_sample_count, 11)
        np.testing.assert_allclose(result.trajectory.times, np.arange(11, dtype=np.float64) * 0.1)
        self.assertEqual(
            float(result.trajectory.times[-1] - result.trajectory.times[0]),
            (result.diagnostic.resampled_sample_count - 1) * 0.1,
        )

    async def test_resampling_rejects_invalid_physics_dt(self) -> None:
        """Reject timing configurations that cannot define fixed simulation steps."""
        trajectory = _trajectory()
        for physics_dt in (0.0, -0.1, float("nan"), float("inf")):
            with self.subTest(physics_dt=physics_dt):
                with self.assertRaisesRegex(ValueError, "physics_dt"):
                    resample_trajectory_for_rollout(
                        trajectory,
                        physics_dt=physics_dt,
                        config=SamplingConfig(mode="always"),
                        max_rollout_steps=20,
                    )

    async def test_sampling_mode_typo_is_rejected(self) -> None:
        """Reject a typo instead of forcing always-on resampling."""
        with self.assertRaisesRegex(ValueError, "Sampling mode"):
            SamplingConfig(mode="alwyas")

    async def test_sample_weight_combination_preserves_user_weights(self) -> None:
        """Combine user weights and trajectory weights without changing scale."""
        combined = combine_sample_weights(
            np.asarray([2.0, 4.0, 8.0], dtype=np.float64),
            np.asarray([0.5, 0.25, 0.125], dtype=np.float64),
            num_steps=3,
        )

        np.testing.assert_allclose(combined, [1.0, 1.0, 1.0])

    async def test_sample_weight_combination_rejects_length_mismatch(self) -> None:
        """Reject weight vectors that do not match the rollout length."""
        with self.assertRaisesRegex(ValueError, "user_weights must contain exactly 3"):
            combine_sample_weights(
                np.asarray([1.0, 2.0], dtype=np.float64),
                None,
                num_steps=3,
            )
