# SPDX-FileCopyrightText: Copyright (c) 2025-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
# http://www.apache.org/licenses/LICENSE-2.0

"""Lifecycle tests for :class:`EpisodeRecorder`.

Covers:

- State machine guards (``IDLE`` → ``SESSION_OPEN`` → ``EPISODE_ACTIVE`` → ``SESSION_OPEN`` → ``CLOSED``).
- ``dispatch_episode_command`` semantics for ``toggle`` / ``start`` / ``end`` with per-session filtering.
- Sampling decimation (``SamplingConfig.decimation``) and ``pause`` / ``resume`` lifecycle.
- :func:`export_stage_snapshot` one-shot module-level entry point.

The physics / app update subscription is bypassed; :meth:`EpisodeRecorder._tick` is invoked
directly so the tests stay deterministic without requiring a running simulation.
"""

from __future__ import annotations

import os
import tempfile
from unittest import mock

import numpy as np
import omni.kit.app
import omni.kit.test
import omni.usd
from isaacsim.replicator.episode_recorder import (
    ChannelDescriptor,
    EpisodeRecorder,
    Recordable,
    ReplayPolicy,
    SamplingConfig,
    SessionReader,
    dispatch_episode_command,
    export_stage_snapshot,
    register_recordable,
    registered_types,
    unregister_recordable,
)

_COUNTER_TYPE_ID = "_test_counter_v2"


class _CounterRecordable(Recordable):
    """Records a monotonically increasing counter; ignores the live stage entirely."""

    TYPE_ID = _COUNTER_TYPE_ID

    def __init__(self, *, group: str = "state/counter") -> None:
        super().__init__(group=group)
        self._value: int = 0

    def describe_channels(self) -> dict[str, ChannelDescriptor]:
        return {"count": ChannelDescriptor(shape=(), dtype="i8")}

    def on_session_open(self, stage) -> None:
        self._value = 0

    def sample(self) -> dict[str, np.ndarray | int]:
        v = self._value
        self._value += 1
        return {"count": np.int64(v)}

    def apply(self, frame, *, policy: ReplayPolicy) -> None:
        pass

    def to_manifest(self) -> dict[str, object]:
        return {"type": self.TYPE_ID, "group": self.group}

    @classmethod
    def from_manifest(cls, entry):
        return cls(group=str(entry["group"]))


class _MalformedFrameRecordable(Recordable):
    """Returns an invalid frame on the second tick to exercise partial-write handling."""

    TYPE_ID = "_test_malformed_frame_v2"

    def __init__(self, *, group: str = "state/malformed") -> None:
        super().__init__(group=group)
        self._sample_count: int = 0

    def describe_channels(self) -> dict[str, ChannelDescriptor]:
        return {"value": ChannelDescriptor(shape=(), dtype="i8")}

    def on_session_open(self, stage) -> None:
        self._sample_count = 0

    def sample(self) -> dict[str, np.ndarray | int]:
        self._sample_count += 1
        if self._sample_count == 2:
            return {}
        return {"value": np.int64(self._sample_count)}

    def apply(self, frame, *, policy: ReplayPolicy) -> None:
        pass

    def to_manifest(self) -> dict[str, object]:
        return {"type": self.TYPE_ID, "group": self.group}

    @classmethod
    def from_manifest(cls, entry):
        return cls(group=str(entry["group"]))


class EpisodeRecorderLifecycleTests(omni.kit.test.AsyncTestCase):
    async def setUp(self) -> None:
        await omni.kit.app.get_app().next_update_async()
        omni.usd.get_context().new_stage()
        await omni.kit.app.get_app().next_update_async()
        if _COUNTER_TYPE_ID not in registered_types():
            register_recordable(_CounterRecordable)
        if _MalformedFrameRecordable.TYPE_ID not in registered_types():
            register_recordable(_MalformedFrameRecordable)

    async def tearDown(self) -> None:
        if _COUNTER_TYPE_ID in registered_types():
            unregister_recordable(_COUNTER_TYPE_ID)
        if _MalformedFrameRecordable.TYPE_ID in registered_types():
            unregister_recordable(_MalformedFrameRecordable.TYPE_ID)
        omni.usd.get_context().close_stage()
        await omni.kit.app.get_app().next_update_async()
        while omni.usd.get_context().get_stage_loading_status()[2] > 0:
            await omni.kit.app.get_app().next_update_async()

    def _make_recorder(
        self,
        output_dir: str,
        *,
        decimation: int = 1,
        session_id: str | None = "lifecycle_test",
        buffer_frames: int = 4,
    ) -> EpisodeRecorder:
        return EpisodeRecorder(
            output_dir,
            file_prefix="lifecycle",
            sampling=SamplingConfig(mode="app_update", decimation=decimation),
            session_id=session_id,
            buffer_frames=buffer_frames,
            link_stage_snapshot=False,
            auto_attach_sim_time=False,
        )

    # ---- state machine ------------------------------------------------

    async def test_state_transitions_and_guards(self) -> None:
        with tempfile.TemporaryDirectory(prefix="recorder_lifecycle_") as output_dir:
            rec = self._make_recorder(output_dir)
            rec.add(_CounterRecordable())
            self.assertEqual(rec.state, "idle")

            with self.assertRaises(RuntimeError):
                rec.start_episode()

            hdf5_path = rec.open_session()
            try:
                self.assertTrue(os.path.isfile(hdf5_path))
                self.assertEqual(rec.state, "session_open")
                self.assertTrue(rec.is_session_open)
                self.assertFalse(rec.is_recording)

                with self.assertRaises(RuntimeError):
                    rec.open_session()

                idx = rec.start_episode()
                self.assertEqual(idx, 0)
                self.assertEqual(rec.state, "episode_active")
                self.assertTrue(rec.is_recording)

                rec.end_episode(success=True)
                self.assertEqual(rec.state, "session_open")
                self.assertFalse(rec.is_recording)
            finally:
                rec.close_session()
                self.assertEqual(rec.state, "closed")

    async def test_open_session_creates_explicit_output_parent(self) -> None:
        with tempfile.TemporaryDirectory(prefix="recorder_lifecycle_") as output_dir:
            rec = self._make_recorder(output_dir)
            rec.add(_CounterRecordable())
            explicit_path = os.path.join(output_dir, "nested", "explicit.hdf5")
            hdf5_path = rec.open_session(explicit_path)
            try:
                self.assertEqual(hdf5_path, explicit_path)
                self.assertTrue(os.path.isfile(explicit_path))
            finally:
                rec.close_session()

    # ---- command bus --------------------------------------------------

    async def test_dispatch_toggle_alternates_start_and_end(self) -> None:
        with tempfile.TemporaryDirectory(prefix="recorder_lifecycle_") as output_dir:
            rec = self._make_recorder(output_dir, session_id="toggle_test")
            rec.add(_CounterRecordable())
            rec.open_session()
            try:
                self.assertFalse(rec.is_recording)
                dispatch_episode_command("toggle", session_id="toggle_test")
                self.assertTrue(rec.is_recording)
                dispatch_episode_command("toggle", session_id="toggle_test")
                self.assertFalse(rec.is_recording)
                dispatch_episode_command("start", session_id="toggle_test")
                self.assertTrue(rec.is_recording)
                dispatch_episode_command("end", session_id="toggle_test", success=False)
                self.assertFalse(rec.is_recording)
            finally:
                rec.close_session()

    async def test_command_ignored_for_other_session_id(self) -> None:
        with tempfile.TemporaryDirectory(prefix="recorder_lifecycle_") as output_dir:
            rec = self._make_recorder(output_dir, session_id="session_A")
            rec.add(_CounterRecordable())
            rec.open_session()
            try:
                dispatch_episode_command("toggle", session_id="session_B")
                self.assertFalse(rec.is_recording)
                dispatch_episode_command("toggle", session_id="session_A")
                self.assertTrue(rec.is_recording)
            finally:
                rec.close_session()

    # ---- sampling / decimation / pause ------------------------------

    async def test_decimation_produces_expected_frame_count(self) -> None:
        with tempfile.TemporaryDirectory(prefix="recorder_lifecycle_") as output_dir:
            rec = self._make_recorder(output_dir, decimation=3)
            rec.add(_CounterRecordable())
            hdf5_path = rec.open_session()
            try:
                rec.start_episode()
                for _ in range(9):
                    rec._tick()
                self.assertEqual(rec.current_episode_frames, 3)
                rec.end_episode(success=True)
            finally:
                rec.close_session()

            reader = SessionReader(hdf5_path)
            try:
                self.assertEqual(reader.num_frames(0), 3)
                counts = reader.read_channel(0, "state/counter", "count")
                np.testing.assert_array_equal(counts, np.array([0, 1, 2], dtype=np.int64))
            finally:
                reader.close()

    async def test_pause_resume_suppresses_samples(self) -> None:
        with tempfile.TemporaryDirectory(prefix="recorder_lifecycle_") as output_dir:
            rec = self._make_recorder(output_dir)
            rec.add(_CounterRecordable())
            rec.open_session()
            try:
                rec.start_episode()
                for _ in range(2):
                    rec._tick()
                rec.pause()
                self.assertTrue(rec.is_paused)
                self.assertFalse(rec.is_recording)
                for _ in range(5):
                    rec._tick()
                self.assertEqual(rec.current_episode_frames, 2)
                rec.resume()
                self.assertTrue(rec.is_recording)
                for _ in range(3):
                    rec._tick()
                self.assertEqual(rec.current_episode_frames, 5)
                rec.end_episode(success=True)
            finally:
                rec.close_session()

    async def test_partial_append_failure_clamps_episode_length(self) -> None:
        with tempfile.TemporaryDirectory(prefix="recorder_lifecycle_") as output_dir:
            rec = self._make_recorder(output_dir)
            rec.add(_CounterRecordable())
            rec.add(_MalformedFrameRecordable())
            hdf5_path = rec.open_session()
            try:
                rec.start_episode()
                rec._tick()
                # This test intentionally generates a malformed frame to validate clamping behavior.
                # Suppress the expected error log so stdout fail-pattern matching does not treat
                # this exercised path as a harness-level failure.
                with mock.patch("carb.log_error"):
                    rec._tick()
                self.assertEqual(rec.current_episode_frames, 1)
                rec.end_episode(success=True)
            finally:
                rec.close_session()

            reader = SessionReader(hdf5_path)
            try:
                self.assertEqual(reader.num_frames(0), 1)
                counts = reader.read_channel(0, "state/counter", "count")
                np.testing.assert_array_equal(counts, np.array([0], dtype=np.int64))
            finally:
                reader.close()

    # ---- session events ----------------------------------------------

    async def test_session_events_fire_for_full_lifecycle(self) -> None:
        with tempfile.TemporaryDirectory(prefix="recorder_lifecycle_") as output_dir:
            rec = self._make_recorder(output_dir)
            rec.add(_CounterRecordable())
            seen: list[str] = []
            rec.events.add_session_opened(lambda: seen.append("opened"))
            rec.events.add_episode_started(lambda idx: seen.append(f"started:{idx}"))
            rec.events.add_episode_ended(lambda idx, s, f: seen.append(f"ended:{idx}:{s}:{f}"))
            rec.events.add_session_closed(lambda: seen.append("closed"))

            rec.open_session()
            rec.start_episode()
            for _ in range(4):
                rec._tick()
            rec.end_episode(success=True)
            rec.close_session()

            self.assertEqual(seen, ["opened", "started:0", "ended:0:True:4", "closed"])

    # ---- export_stage_snapshot ---------------------------------------

    async def test_export_stage_snapshot_writes_usd_and_sidecar(self) -> None:
        with tempfile.TemporaryDirectory(prefix="recorder_lifecycle_") as output_dir:
            usd_path = export_stage_snapshot(output_dir)
            self.assertTrue(os.path.isfile(usd_path))
            self.assertTrue(usd_path.endswith(".usd"))
            sidecar = os.path.splitext(usd_path)[0] + ".sidecar.json"
            if os.path.exists(sidecar):
                self.assertGreater(os.path.getsize(sidecar), 0)

    async def test_stage_snapshot_auto_linked_into_session_attrs(self) -> None:
        with tempfile.TemporaryDirectory(prefix="recorder_lifecycle_") as output_dir:
            export_stage_snapshot(output_dir)
            rec = EpisodeRecorder(
                output_dir,
                file_prefix="linked",
                sampling=SamplingConfig(mode="app_update"),
                link_stage_snapshot=True,
                auto_attach_sim_time=False,
            )
            rec.add(_CounterRecordable())
            hdf5_path = rec.open_session()
            try:
                rec.start_episode()
                rec._tick()
                rec.end_episode(success=True)
            finally:
                rec.close_session()

            import h5py

            with h5py.File(hdf5_path, "r") as f:
                self.assertIn("stage_snapshot", f.attrs)
                self.assertTrue(str(f.attrs["stage_snapshot"]).endswith(".usd"))
