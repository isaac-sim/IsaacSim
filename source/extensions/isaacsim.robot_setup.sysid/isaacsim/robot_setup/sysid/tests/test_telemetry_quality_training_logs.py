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

"""Unit tests for telemetry quality reports and companion training-log discovery."""

from __future__ import annotations

import json
import tempfile
from pathlib import Path
from unittest.mock import patch

import numpy as np
import omni.kit.test
from isaacsim.robot_setup.sysid.provenance import build_dataset_metadata
from isaacsim.robot_setup.sysid.telemetry_quality import (
    build_telemetry_quality_report,
    estimate_command_lag,
)
from isaacsim.robot_setup.sysid.training_logs import (
    _load_json_object,
    _load_jsonl_objects,
    attach_companion_training_details,
    find_companion_mcap_path,
    find_companion_root,
    find_companion_topic_mapping_path,
    load_companion_training_details,
    phase_chunks_from_metadata,
)
from isaacsim.robot_setup.sysid.trajectory_csv import TrajectoryDataset
from isaacsim.robot_setup.sysid.trajectory_segments import TelemetryChunkRunSpec


def _trajectory() -> TrajectoryDataset:
    return TrajectoryDataset(
        times=np.asarray([0.0, 0.1, 0.4, 0.5], dtype=np.float64),
        positions=np.zeros((4, 1), dtype=np.float64),
        velocities=np.zeros((4, 1), dtype=np.float64),
        commands=np.zeros((4, 1), dtype=np.float64),
        torques=np.ones((4, 1), dtype=np.float64),
    )


def _messages(log_mock) -> list[str]:  # noqa: ANN001
    return [call.args[0] for call in log_mock.call_args_list]


class TelemetryQualityAndTrainingLogTests(omni.kit.test.AsyncTestCase):
    """Tests for telemetry quality diagnostics and companion log parsing."""

    async def test_telemetry_quality_reports_channels_chunks_and_warnings(self) -> None:
        """Report enabled channels, chunk counts, and warn-first diagnostics."""
        report = build_telemetry_quality_report(
            _trajectory(),
            [TelemetryChunkRunSpec(name="train", role="train", start=0.0, end=0.1)],
        )

        codes = {issue.code for issue in report.issues}
        self.assertTrue(report.channels["torque"])
        self.assertEqual(report.train_chunk_count, 1)
        self.assertEqual(report.validation_chunk_count, 0)
        self.assertIn("weak_excitation", codes)
        self.assertIn("no_validation_holdout", codes)
        self.assertIn("command_matches_position", codes)
        self.assertGreater(report.warning_count, 0)
        self.assertIn("samples", report.compact_summary())

    async def test_telemetry_quality_reports_chunk_validation_issue(self) -> None:
        """Surface invalid train/validation chunk overlap as a quality issue."""
        report = build_telemetry_quality_report(
            _trajectory(),
            [
                TelemetryChunkRunSpec(name="train", role="train", start=0.0, end=0.4),
                TelemetryChunkRunSpec(name="val", role="validation", start=0.3, end=0.5),
            ],
        )

        self.assertIn("chunk_validation", {issue.code for issue in report.issues})
        self.assertEqual(report.chunk_coverage_fraction, 0.0)

    async def test_telemetry_quality_treats_nonfinite_channels_as_errors(self) -> None:
        """Make invalid numeric telemetry block quality-gated workflows."""
        trajectory = _trajectory()
        trajectory.positions[1, 0] = np.nan

        report = build_telemetry_quality_report(trajectory, [])

        self.assertIn("nonfinite_position", {issue.code for issue in report.issues})
        self.assertEqual(report.error_count, 1)

    async def test_telemetry_quality_treats_nonmonotonic_time_as_error(self) -> None:
        """Make non-increasing timestamps block quality-gated workflows."""
        trajectory = _trajectory()
        trajectory.times = np.asarray([0.0, 0.1, 0.1, 0.5], dtype=np.float64)

        report = build_telemetry_quality_report(trajectory, [])

        timestamp_issue = next(issue for issue in report.issues if issue.code == "timestamp_order")
        self.assertEqual(timestamp_issue.severity, "error")

    async def test_command_lag_rejects_invalid_search_window(self) -> None:
        """Reject non-positive and non-finite lag search windows."""
        for value in (0.0, -0.1, np.nan, np.inf):
            with self.subTest(max_lag_seconds=value):
                with self.assertRaisesRegex(ValueError, "finite positive"):
                    estimate_command_lag(_trajectory(), max_lag_seconds=value)

    async def test_companion_path_discovery_prefers_run_folder_files(self) -> None:
        """Find companion topic maps and MCAP files from a run folder."""
        with tempfile.TemporaryDirectory(prefix="sysid_training_logs_") as tmp_dir:
            root = Path(tmp_dir)
            bag_dir = root / "bag"
            bag_dir.mkdir()
            source = bag_dir / "recording.db3"
            source.write_text("", encoding="utf-8")
            topic_map = root / "franka_sysid_topic_map.yaml"
            topic_map.write_text("position_topic: /joint_states\n", encoding="utf-8")
            mcap_path = bag_dir / "recording.mcap"
            mcap_path.write_bytes(b"")

            self.assertEqual(find_companion_root(source), root)
            self.assertEqual(find_companion_topic_mapping_path(source), str(topic_map))
            self.assertEqual(find_companion_mcap_path(root), str(mcap_path))

    async def test_companion_mcap_discovery_rejects_ambiguous_recordings(self) -> None:
        """Require an explicit file path when a run contains multiple MCAP files."""
        with tempfile.TemporaryDirectory(prefix="sysid_multiple_mcap_") as tmp_dir:
            root = Path(tmp_dir)
            (root / "first.mcap").write_bytes(b"")
            (root / "second.mcap").write_bytes(b"")

            with self.assertRaisesRegex(ValueError, "Multiple companion MCAP files"):
                find_companion_mcap_path(root)

    async def test_malformed_optional_manifest_logs_warning(self) -> None:
        """Warn when optional companion JSON cannot be parsed."""
        with tempfile.TemporaryDirectory(prefix="sysid_bad_manifest_") as tmp_dir:
            manifest = Path(tmp_dir) / "collection_manifest.json"
            manifest.write_text("{malformed", encoding="utf-8")

            with patch("carb.log_warn") as log_warn:
                payload = _load_json_object(manifest)

        self.assertEqual(payload, {})
        self.assertTrue(any("could not parse optional companion JSON" in message for message in _messages(log_warn)))

    async def test_malformed_optional_phase_event_logs_warning(self) -> None:
        """Warn with a row number when malformed optional JSONL is skipped."""
        with tempfile.TemporaryDirectory(prefix="sysid_bad_phase_events_") as tmp_dir:
            events_path = Path(tmp_dir) / "phase_events.jsonl"
            events_path.write_text('{"phase": "train"}\n{malformed\n', encoding="utf-8")

            with patch("carb.log_warn") as log_warn:
                rows = _load_jsonl_objects(events_path)

        self.assertEqual(rows, [{"phase": "train"}])
        self.assertTrue(any("row 2" in message for message in _messages(log_warn)))

    async def test_companion_training_details_convert_phase_events_to_chunks(self) -> None:
        """Convert phase event logs and manifest metadata into chunk specs."""
        with tempfile.TemporaryDirectory(prefix="sysid_phase_logs_") as tmp_dir:
            root = Path(tmp_dir)
            (root / "collection_manifest.json").write_text(
                json.dumps(
                    {
                        "phases": [
                            {"name": "slow_sine", "split": "train", "purpose": "sine sweep"},
                            {"name": "holdout_step", "split": "validation", "purpose": "step response"},
                        ]
                    }
                ),
                encoding="utf-8",
            )
            events = [
                {"phase": "slow_sine", "event": "start", "ros_time_sec": 0.0},
                {"phase": "slow_sine", "event": "end", "ros_time_sec": 0.2},
                {"phase": "holdout_step", "event": "start", "ros_time_sec": 0.3},
                {"phase": "holdout_step", "event": "end", "ros_time_sec": 0.5},
            ]
            (root / "phase_events.jsonl").write_text(
                "\n".join(json.dumps(event) for event in events),
                encoding="utf-8",
            )
            trajectory = TrajectoryDataset(
                times=np.asarray([0.0, 0.1, 0.2, 0.3, 0.4, 0.5], dtype=np.float64),
                positions=np.zeros((6, 1), dtype=np.float64),
                velocities=np.zeros((6, 1), dtype=np.float64),
                commands=np.zeros((6, 1), dtype=np.float64),
            )

            details = load_companion_training_details(root, trajectory)

            self.assertIsNotNone(details)
            self.assertEqual(details.time_key, "ros_time_sec")
            self.assertEqual([chunk.role for chunk in details.chunks], ["train", "validation"])
            self.assertEqual([chunk.excitation for chunk in details.chunks], ["sine", "step"])

    async def test_attach_companion_training_details_updates_metadata_extra(self) -> None:
        """Attach parsed phase chunks to trajectory metadata extras."""
        with tempfile.TemporaryDirectory(prefix="sysid_attach_phase_logs_") as tmp_dir:
            root = Path(tmp_dir)
            (root / "phase_events.jsonl").write_text(
                "\n".join(
                    [
                        json.dumps({"phase": "prbs_train", "event": "start", "wall_time": 0.0}),
                        json.dumps({"phase": "prbs_train", "event": "end", "wall_time": 0.5}),
                    ]
                ),
                encoding="utf-8",
            )
            trajectory = TrajectoryDataset(
                times=np.asarray([0.0, 0.25, 0.5], dtype=np.float64),
                positions=np.zeros((3, 1), dtype=np.float64),
                velocities=np.zeros((3, 1), dtype=np.float64),
                commands=np.zeros((3, 1), dtype=np.float64),
            )
            trajectory.metadata = build_dataset_metadata(
                source_type="csv",
                source_path=str(root / "telemetry.csv"),
                times=trajectory.times,
                positions=trajectory.positions,
                velocities=trajectory.velocities,
                commands=trajectory.commands,
            )

            attach_companion_training_details(trajectory, root)
            chunks = phase_chunks_from_metadata(trajectory)

            self.assertEqual(len(chunks), 1)
            self.assertEqual(chunks[0].name, "prbs_train")
            self.assertEqual(chunks[0].excitation, "PRBS")

    async def test_nonoverlapping_phase_events_are_not_rebased(self) -> None:
        """Ignore stale absolute-time logs beside an unrelated relative-time recording."""
        with tempfile.TemporaryDirectory(prefix="sysid_stale_phase_logs_") as tmp_dir:
            root = Path(tmp_dir)
            (root / "phase_events.jsonl").write_text(
                "\n".join(
                    [
                        json.dumps({"phase": "stale", "event": "start", "wall_time": 100.0}),
                        json.dumps({"phase": "stale", "event": "end", "wall_time": 101.0}),
                    ]
                ),
                encoding="utf-8",
            )

            details = load_companion_training_details(root, _trajectory())

        self.assertIsNotNone(details)
        self.assertEqual(details.chunks, [])
        self.assertEqual(details.time_key, "")

    async def test_phase_events_on_mismatched_timebase_are_rejected(self) -> None:
        """Reject a foreign-timebase phase log that only brushes the telemetry window."""
        with tempfile.TemporaryDirectory(prefix="sysid_mismatched_phase_logs_") as tmp_dir:
            root = Path(tmp_dir)
            (root / "phase_events.jsonl").write_text(
                "\n".join(
                    [
                        json.dumps({"phase": "other_run", "event": "start", "monotonic_time": 0.49}),
                        json.dumps({"phase": "other_run", "event": "end", "monotonic_time": 900.0}),
                    ]
                ),
                encoding="utf-8",
            )

            with patch("carb.log_warn") as log_warn:
                details = load_companion_training_details(root, _trajectory())

        self.assertEqual(details.chunks, [])
        self.assertEqual(details.time_key, "")
        self.assertTrue(any("no time key matches the telemetry window" in message for message in _messages(log_warn)))

    async def test_phase_chunks_from_metadata_skips_invalid_specs(self) -> None:
        """Keep valid companion chunks when one user-authored spec is corrupt."""
        trajectory = _trajectory()
        trajectory.metadata = build_dataset_metadata(
            source_type="csv",
            source_path="telemetry.csv",
            times=trajectory.times,
            positions=trajectory.positions,
            velocities=trajectory.velocities,
            commands=trajectory.commands,
            extra={
                "phase_chunks": [
                    {"name": "valid", "role": "train", "start": 0.0, "end": 0.1},
                    {"name": "invalid", "role": "validation", "start": 0.4, "end": 0.2},
                ]
            },
        )

        with patch("carb.log_warn") as log_warn:
            chunks = phase_chunks_from_metadata(trajectory)

        self.assertEqual([chunk.name for chunk in chunks], ["valid"])
        self.assertTrue(any("ignoring invalid companion phase chunk" in message for message in _messages(log_warn)))
