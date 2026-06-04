# SPDX-FileCopyrightText: Copyright (c) 2025-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
# http://www.apache.org/licenses/LICENSE-2.0

"""End-to-end :class:`EpisodeReplayer` tests.

Records a small session using a stage-free synthetic :class:`Recordable`, then
replays it through the public replayer API and validates:

- Best-effort mode skips unknown recordable types with a warning, strict mode raises.
- :meth:`apply_frame` / :meth:`prepare_episode` round-trip values exactly as recorded.
- :meth:`episode_attrs` / :meth:`num_frames` / :meth:`list_episodes` reflect the manifest.
- :meth:`replay_episode` walks every frame once without Kit timeline wiring.
"""

from __future__ import annotations

import asyncio
import tempfile

import isaacsim.core.experimental.utils.stage as stage_utils
import numpy as np
import omni.kit.app
import omni.kit.test
import omni.usd
from isaacsim.replicator.episode_recorder import (
    ChannelDescriptor,
    EpisodeRecorder,
    EpisodeReplayer,
    Recordable,
    ReplayPolicy,
    SamplingConfig,
    SessionStorage,
    XformRecordable,
    build_manifest,
    register_recordable,
    registered_types,
    unregister_recordable,
)

_ECHO_TYPE_ID = "_test_echo_v2"


class _EchoRecordable(Recordable):
    """Records an externally-supplied value per tick and records what it was asked to apply."""

    TYPE_ID = _ECHO_TYPE_ID

    def __init__(self, *, group: str = "state/echo", value: float = 0.0) -> None:
        super().__init__(group=group)
        self._value: float = float(value)
        self.applied_frames: list[dict[str, np.ndarray]] = []

    def set_value(self, value: float) -> None:
        self._value = float(value)

    def describe_channels(self) -> dict[str, ChannelDescriptor]:
        return {"value": ChannelDescriptor(shape=(3,), dtype="f4")}

    def sample(self) -> dict[str, np.ndarray]:
        return {"value": np.full(3, self._value, dtype=np.float32)}

    def apply(self, frame, *, policy: ReplayPolicy) -> None:
        self.applied_frames.append({k: np.asarray(v) for k, v in frame.items()})

    def to_manifest(self) -> dict[str, object]:
        return {"type": self.TYPE_ID, "group": self.group}

    @classmethod
    def from_manifest(cls, entry):
        return cls(group=str(entry["group"]))


def _record_session(output_dir: str, rec: _EchoRecordable, values: list[float]) -> str:
    recorder = EpisodeRecorder(
        output_dir,
        file_prefix="replay",
        sampling=SamplingConfig(mode="app_update"),
        link_stage_snapshot=False,
        auto_attach_sim_time=False,
    )
    recorder.add(rec)
    hdf5_path = recorder.open_session()
    try:
        recorder.start_episode(metadata={"tag": "replay-test"})
        for v in values:
            rec.set_value(v)
            recorder._tick()
        recorder.end_episode(success=True)
    finally:
        recorder.close_session()
    return hdf5_path


class EpisodeReplayerRoundtripTests(omni.kit.test.AsyncTestCase):
    async def setUp(self) -> None:
        await omni.kit.app.get_app().next_update_async()
        omni.usd.get_context().new_stage()
        await omni.kit.app.get_app().next_update_async()
        if _ECHO_TYPE_ID not in registered_types():
            register_recordable(_EchoRecordable)

    async def tearDown(self) -> None:
        if _ECHO_TYPE_ID in registered_types():
            unregister_recordable(_ECHO_TYPE_ID)
        omni.usd.get_context().close_stage()
        await omni.kit.app.get_app().next_update_async()
        while omni.usd.get_context().get_stage_loading_status()[2] > 0:
            await omni.kit.app.get_app().next_update_async()

    async def test_manifest_frame_and_attrs_roundtrip(self) -> None:
        with tempfile.TemporaryDirectory(prefix="replayer_test_") as tmp_dir:
            values = [1.0, 2.5, -3.0, 4.25]
            path = _record_session(tmp_dir, _EchoRecordable(), values)

            replayer = EpisodeReplayer(path)
            try:
                self.assertEqual(replayer.list_episodes(), ["episode_00000"])
                self.assertEqual(replayer.num_frames(0), len(values))

                attrs = replayer.episode_attrs(0)
                self.assertEqual(int(attrs.get("num_frames", -1)), len(values))
                self.assertTrue(bool(attrs.get("success", False)))
                user_md = attrs.get("user_metadata", {})
                if isinstance(user_md, dict):
                    self.assertEqual(user_md.get("tag"), "replay-test")

                replayer.prepare_episode(0)
                applied: list[np.ndarray] = []
                for rec in replayer.prepared_recordables:
                    self.assertIsInstance(rec, _EchoRecordable)
                    rec.applied_frames.clear()
                for i in range(len(values)):
                    replayer.apply_frame(i)
                for rec in replayer.prepared_recordables:
                    applied = [f["value"] for f in rec.applied_frames]

                self.assertEqual(len(applied), len(values))
                for v, arr in zip(values, applied, strict=True):
                    np.testing.assert_allclose(arr, np.full(3, v, dtype=np.float32))
            finally:
                replayer.close()

    async def test_replay_episode_drives_every_frame(self) -> None:
        with tempfile.TemporaryDirectory(prefix="replayer_test_") as tmp_dir:
            values = [0.1, 0.2, 0.3]
            path = _record_session(tmp_dir, _EchoRecordable(), values)

            replayer = EpisodeReplayer(path)
            try:
                replayer.replay_episode(0, app_update_per_frame=False)
                for rec in replayer.prepared_recordables:
                    self.assertIsInstance(rec, _EchoRecordable)
                    applied = [f["value"] for f in rec.applied_frames]
                    self.assertEqual(len(applied), len(values))
                    for v, arr in zip(values, applied, strict=True):
                        np.testing.assert_allclose(arr, np.full(3, v, dtype=np.float32))
            finally:
                replayer.close()

    async def test_prepare_episode_async_prefetches_with_worker_reader(self) -> None:
        with tempfile.TemporaryDirectory(prefix="replayer_test_") as tmp_dir:
            values = [5.0, 6.0]
            path = _record_session(tmp_dir, _EchoRecordable(), values)

            replayer = EpisodeReplayer(path)
            try:
                await replayer.prepare_episode_async(0)
                self.assertEqual(len(replayer.prepared_recordables), 1)
                replayer.apply_frame(1)
                rec = replayer.prepared_recordables[0]
                self.assertIsInstance(rec, _EchoRecordable)
                assert isinstance(rec, _EchoRecordable)
                np.testing.assert_allclose(rec.applied_frames[-1]["value"], np.full(3, 6.0, dtype=np.float32))
            finally:
                replayer.close()

    async def test_prepare_episode_async_cancelled_prefetch_releases_state(self) -> None:
        with tempfile.TemporaryDirectory(prefix="replayer_test_") as tmp_dir:
            path = _record_session(tmp_dir, _EchoRecordable(), [1.0])

            class _CancellingReplayer(EpisodeReplayer):
                def _prefetch_frames_from_new_reader(
                    self, episode_name: str, groups: tuple[str, ...]
                ) -> dict[str, dict[str, np.ndarray]]:
                    raise asyncio.CancelledError()

            replayer = _CancellingReplayer(path)
            try:
                with self.assertRaises(asyncio.CancelledError):
                    await replayer.prepare_episode_async(0)
                self.assertEqual(replayer.prepared_recordables, [])
            finally:
                replayer.close()

    async def test_replay_pose_batch_applies_xform_recordable(self) -> None:
        stage_utils.define_prim("/World", "Xform")
        stage_utils.define_prim("/World/Replayed", "Xform")

        rec = XformRecordable(group="state/replayed", prim_path="/World/Replayed")
        with tempfile.TemporaryDirectory(prefix="replayer_test_") as tmp_dir:
            path = f"{tmp_dir}/pose_batch.hdf5"
            storage = SessionStorage(path, buffer_frames=2)
            storage.open()
            storage.write_manifest(build_manifest([rec.to_manifest()]))
            storage.begin_episode({rec.group: rec.describe_channels()})
            storage.append_frame(
                rec.group,
                {
                    "position": np.array([0.0, 0.0, 0.0], dtype=np.float32),
                    "orientation": np.array([1.0, 0.0, 0.0, 0.0], dtype=np.float32),
                },
            )
            storage.advance_episode_frame()
            storage.append_frame(
                rec.group,
                {
                    "position": np.array([1.0, 2.0, 3.0], dtype=np.float32),
                    "orientation": np.array([1.0, 0.0, 0.0, 0.0], dtype=np.float32),
                },
            )
            storage.advance_episode_frame()
            storage.end_episode(success=True)
            storage.close()

            replayer = EpisodeReplayer(path)
            try:
                replayer.prepare_episode(0)
                self.assertEqual(replayer.pose_batch_size, 1)
                replayer.apply_frame(1)

                from isaacsim.core.experimental.prims import XformPrim

                pos, _quat = XformPrim("/World/Replayed").get_world_poses()
                pos_np = pos.numpy() if hasattr(pos, "numpy") else np.asarray(pos)
                np.testing.assert_allclose(pos_np.reshape(-1, 3)[0], np.array([1.0, 2.0, 3.0], dtype=np.float32))
            finally:
                replayer.close()

    async def test_best_effort_skips_unknown_recordable_type(self) -> None:
        with tempfile.TemporaryDirectory(prefix="replayer_test_") as tmp_dir:
            path = _record_session(tmp_dir, _EchoRecordable(), [1.0, 2.0])
            unregister_recordable(_ECHO_TYPE_ID)
            try:
                replayer = EpisodeReplayer(path, policy=ReplayPolicy(strictness="best_effort"))
                try:
                    replayer.prepare_episode(0)
                    self.assertEqual(replayer.prepared_recordables, [])
                finally:
                    replayer.close()
            finally:
                register_recordable(_EchoRecordable)

    async def test_strict_mode_raises_on_unknown_type(self) -> None:
        with tempfile.TemporaryDirectory(prefix="replayer_test_") as tmp_dir:
            path = _record_session(tmp_dir, _EchoRecordable(), [1.0])
            unregister_recordable(_ECHO_TYPE_ID)
            try:
                replayer = EpisodeReplayer(path, policy=ReplayPolicy(strictness="strict"))
                try:
                    with self.assertRaises(KeyError):
                        replayer.prepare_episode(0)
                finally:
                    replayer.close()
            finally:
                register_recordable(_EchoRecordable)

    async def test_frame_index_out_of_range_raises(self) -> None:
        with tempfile.TemporaryDirectory(prefix="replayer_test_") as tmp_dir:
            path = _record_session(tmp_dir, _EchoRecordable(), [1.0, 2.0])
            replayer = EpisodeReplayer(path)
            try:
                replayer.prepare_episode(0)
                with self.assertRaises(IndexError):
                    replayer.apply_frame(5)
                with self.assertRaises(IndexError):
                    replayer.apply_frame(-1)
            finally:
                replayer.close()

    async def test_start_replay_empty_episode_releases_prepared_state(self) -> None:
        with tempfile.TemporaryDirectory(prefix="replayer_test_") as tmp_dir:
            path = _record_session(tmp_dir, _EchoRecordable(), [])
            replayer = EpisodeReplayer(path)
            try:
                with self.assertRaises(RuntimeError):
                    replayer.start_replay(episode=0)
                self.assertFalse(replayer.is_replaying)
                self.assertEqual(replayer.total_frames, 0)
                self.assertEqual(replayer.prepared_recordables, [])
            finally:
                replayer.close()

    async def test_step_frame_clamps_at_bounds(self) -> None:
        """`step_frame` must clamp at 0 and at total_frames - 1 and auto-pause."""
        with tempfile.TemporaryDirectory(prefix="replayer_test_") as tmp_dir:
            values = [0.0, 1.0, 2.0]
            path = _record_session(tmp_dir, _EchoRecordable(), values)
            replayer = EpisodeReplayer(path)
            try:
                replayer.start_replay(episode=0, seek_timeline=False)
                self.assertTrue(replayer.is_replaying, "Replay should be attached for the step test")

                idx = replayer.step_frame(-5)
                self.assertEqual(idx, 0, "step_frame from frame 0 with negative delta must clamp to 0")
                self.assertEqual(replayer.current_frame, 0)
                self.assertTrue(replayer.is_paused, "step_frame must auto-pause the replay")

                replayer._replay_frame = replayer.total_frames - 1
                idx = replayer.step_frame(+10)
                self.assertEqual(
                    idx,
                    replayer.total_frames - 1,
                    "step_frame from the last frame with positive delta must clamp to total_frames - 1",
                )
                self.assertEqual(replayer.current_frame, replayer.total_frames - 1)
                self.assertTrue(replayer.is_paused, "step_frame must keep the replay paused at the boundary")
            finally:
                replayer.stop_replay()
                replayer.close()

    async def test_loop_replay_wraps_to_first_frame(self) -> None:
        """With ``loop=True`` the app-update tick wraps the cursor back to 0 instead of stopping."""
        with tempfile.TemporaryDirectory(prefix="replayer_test_") as tmp_dir:
            values = [0.5, 1.5]
            path = _record_session(tmp_dir, _EchoRecordable(), values)
            replayer = EpisodeReplayer(path)
            try:
                replayer.start_replay(episode=0, loop=True, seek_timeline=False)
                self.assertTrue(replayer.is_replaying, "Replay should be attached for the loop test")

                wrapped = False
                last_seen = replayer.current_frame
                for _ in range(40):
                    await omni.kit.app.get_app().next_update_async()
                    cur = replayer.current_frame
                    if last_seen == replayer.total_frames - 1 and cur == 0:
                        wrapped = True
                        break
                    last_seen = cur

                self.assertTrue(wrapped, "loop=True must wrap from the last frame back to frame 0")
                self.assertTrue(replayer.is_replaying, "loop=True must not stop the replay at the end")
            finally:
                replayer.stop_replay()
                replayer.close()
