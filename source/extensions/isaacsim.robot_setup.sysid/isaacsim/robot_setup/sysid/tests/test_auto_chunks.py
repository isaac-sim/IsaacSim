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

# ruff: noqa: D101, D102

"""Tests for automatic train/validation chunk splitting."""

from __future__ import annotations

import numpy as np
import omni.kit.test
from isaacsim.robot_setup.sysid.trajectory_csv import TrajectoryDataset
from isaacsim.robot_setup.sysid.trajectory_segments import (
    TELEMETRY_CHUNK_ROLE_TRAIN,
    TELEMETRY_CHUNK_ROLE_VALIDATION,
    TelemetryChunkRunSpec,
    build_auto_chunks,
    build_trajectory_chunks,
    resolve_chunk_specs,
)


def _trajectory(num_samples: int = 200, dt: float = 0.02) -> TrajectoryDataset:
    times = np.arange(num_samples, dtype=np.float64) * dt
    values = np.zeros((num_samples, 2), dtype=np.float64)
    return TrajectoryDataset(times=times, positions=values.copy(), velocities=values.copy(), commands=values.copy())


def _epoch_trajectory(num_samples: int = 4000, dt: float = 0.001) -> TrajectoryDataset:
    trajectory = _trajectory(num_samples=num_samples, dt=dt)
    trajectory.times += 1_700_000_000.0
    return trajectory


class BuildAutoChunksTests(omni.kit.test.AsyncTestCase):
    async def test_plain_split_produces_valid_train_validation_pair(self) -> None:
        trajectory = _trajectory()
        specs = build_auto_chunks(trajectory, train_fraction=0.8, min_chunk_seconds=0.5)
        self.assertEqual(len(specs), 2)
        self.assertEqual(specs[0].role, TELEMETRY_CHUNK_ROLE_TRAIN)
        self.assertEqual(specs[1].role, TELEMETRY_CHUNK_ROLE_VALIDATION)
        self.assertLess(specs[0].end, specs[1].start)
        duration = float(trajectory.times[-1] - trajectory.times[0])
        train_duration = specs[0].end - specs[0].start
        self.assertAlmostEqual(train_duration / duration, 0.8, delta=0.05)
        # Must pass the strict overlap/leakage validation used at prepare time.
        chunks = build_trajectory_chunks(trajectory, specs, require_train=True, reject_overlaps=True)
        self.assertEqual(len(chunks), 2)
        self.assertGreaterEqual(chunks[1].sample_count, 2)

    async def test_epoch_timestamps_do_not_create_split_overlap(self) -> None:
        trajectory = _epoch_trajectory()
        specs = build_auto_chunks(trajectory, train_fraction=0.8, min_chunk_seconds=0.5)
        chunks = build_trajectory_chunks(trajectory, specs, require_train=True, reject_overlaps=True)

        self.assertEqual(len(chunks), 2)
        self.assertLess(chunks[0].trajectory.times[-1], chunks[1].trajectory.times[0])
        self.assertEqual(
            np.intersect1d(chunks[0].trajectory.times, chunks[1].trajectory.times).size,
            0,
        )

    async def test_epoch_time_slice_does_not_include_neighbouring_samples(self) -> None:
        trajectory = _epoch_trajectory()
        sliced = trajectory.slice_time_window(trajectory.times[100], trajectory.times[200])

        self.assertEqual(sliced.times.shape[0], 101)
        self.assertEqual(sliced.times[0], trajectory.times[100])
        self.assertEqual(sliced.times[-1], trajectory.times[200])

    async def test_short_telemetry_falls_back_to_single_train_chunk(self) -> None:
        trajectory = _trajectory(num_samples=20, dt=0.02)
        specs = build_auto_chunks(trajectory, train_fraction=0.8, min_chunk_seconds=1.0)
        self.assertEqual(len(specs), 1)
        self.assertEqual(specs[0].role, TELEMETRY_CHUNK_ROLE_TRAIN)

    async def test_metadata_phases_reassign_trailing_phase_and_keep_tags(self) -> None:
        trajectory = _trajectory()
        metadata = [
            TelemetryChunkRunSpec(name="chirp", role="train", excitation="chirp", start=0.0, end=1.5),
            TelemetryChunkRunSpec(name="sine", role="train", excitation="sine", start=1.6, end=2.8),
            TelemetryChunkRunSpec(name="step", role="train", excitation="step", start=2.9, end=3.9),
        ]
        specs = build_auto_chunks(trajectory, train_fraction=0.7, metadata_chunks=metadata)
        self.assertEqual(len(specs), 3)
        self.assertEqual(specs[0].role, TELEMETRY_CHUNK_ROLE_TRAIN)
        self.assertEqual(specs[-1].role, TELEMETRY_CHUNK_ROLE_VALIDATION)
        self.assertEqual(specs[-1].excitation, "step")

    async def test_metadata_with_existing_validation_is_untouched(self) -> None:
        trajectory = _trajectory()
        metadata = [
            TelemetryChunkRunSpec(name="a", role="train", excitation="chirp", start=0.0, end=2.0),
            TelemetryChunkRunSpec(name="b", role="validation", excitation="sine", start=2.1, end=3.9),
        ]
        specs = build_auto_chunks(trajectory, metadata_chunks=metadata)
        self.assertEqual([spec.role for spec in specs], ["train", "validation"])

    async def test_metadata_split_honors_min_chunk_seconds(self) -> None:
        """A trailing phase shorter than the floor must not become the holdout."""
        trajectory = _trajectory()
        metadata = [
            TelemetryChunkRunSpec(name="chirp", role="train", excitation="chirp", start=0.0, end=3.0),
            TelemetryChunkRunSpec(name="step", role="train", excitation="step", start=3.1, end=3.3),
        ]

        specs = build_auto_chunks(
            trajectory,
            train_fraction=0.8,
            min_chunk_seconds=1.0,
            metadata_chunks=metadata,
        )

        self.assertEqual([spec.role for spec in specs], ["train", "train"])

    async def test_metadata_split_returns_sorted_specs_for_pre_tagged_input(self) -> None:
        """Both metadata branches return one chunk ordering."""
        trajectory = _trajectory()
        metadata = [
            TelemetryChunkRunSpec(name="b", role="validation", excitation="sine", start=2.1, end=3.9),
            TelemetryChunkRunSpec(name="a", role="train", excitation="chirp", start=0.0, end=2.0),
        ]

        specs = build_auto_chunks(trajectory, metadata_chunks=metadata)

        self.assertEqual([spec.name for spec in specs], ["a", "b"])

    async def test_first_phase_never_reassigned(self) -> None:
        trajectory = _trajectory()
        metadata = [
            TelemetryChunkRunSpec(name="only", role="train", excitation="chirp", start=0.0, end=3.9),
        ]
        specs = build_auto_chunks(trajectory, train_fraction=0.5, metadata_chunks=metadata)
        self.assertEqual([spec.role for spec in specs], ["train"])


class ResolveChunkSpecsTests(omni.kit.test.AsyncTestCase):
    async def test_explicit_chunks_win_over_auto_split(self) -> None:
        trajectory = _trajectory()
        explicit = [TelemetryChunkRunSpec(name="Manual", role="train", start=0.0, end=1.0)]
        specs = resolve_chunk_specs(trajectory, explicit, auto_split=True)
        self.assertEqual([spec.name for spec in specs], ["Manual"])

    async def test_auto_split_respects_time_window(self) -> None:
        trajectory = _trajectory()  # 0.0 .. 3.98 s
        specs = resolve_chunk_specs(
            trajectory, [], auto_split=True, train_fraction=0.8, min_chunk_seconds=0.2, window=(0.0, 2.0)
        )
        self.assertEqual(len(specs), 2)
        self.assertEqual(specs[0].role, TELEMETRY_CHUNK_ROLE_TRAIN)
        self.assertEqual(specs[1].role, TELEMETRY_CHUNK_ROLE_VALIDATION)
        self.assertLessEqual(specs[-1].end, 2.0 + 1e-9)

    async def test_no_auto_split_falls_back_to_default_train_chunk(self) -> None:
        trajectory = _trajectory()
        specs = resolve_chunk_specs(trajectory, [], auto_split=False, window=(0.5, 1.5))
        self.assertEqual(len(specs), 1)
        self.assertEqual(specs[0].role, TELEMETRY_CHUNK_ROLE_TRAIN)
        self.assertAlmostEqual(specs[0].start, 0.5)
        self.assertAlmostEqual(specs[0].end, 1.5)
