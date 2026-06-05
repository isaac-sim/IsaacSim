# SPDX-FileCopyrightText: Copyright (c) 2025-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
# http://www.apache.org/licenses/LICENSE-2.0

"""Storage + manifest round-trip tests for the V2 recorder layer.

These tests validate the :class:`SessionStorage` → :class:`SessionReader`
write/read contract plus the :class:`Recordable` registry rehydration path.
They run under Isaac Sim's standard Kit test harness, but do not require
simulation stepping or a populated stage.
"""

from __future__ import annotations

import os
import tempfile
import unittest

import numpy as np
import omni.kit.app
import omni.kit.test
import omni.usd
from isaacsim.replicator.episode_recorder import (
    ChannelDescriptor,
    Recordable,
    ReplayPolicy,
    SamplingConfig,
    SessionReader,
    SessionStorage,
    build_manifest,
    register_recordable,
    registered_types,
    rehydrate,
    unregister_recordable,
)


class _DummyRec(Recordable):
    """Minimal in-memory recordable for storage/registry tests."""

    TYPE_ID = "_test_dummy_v2"

    def __init__(self, *, group: str, dim: int = 3) -> None:
        super().__init__(group=group)
        self._dim = int(dim)
        self._last_applied: dict[str, np.ndarray] | None = None

    def describe_channels(self) -> dict[str, ChannelDescriptor]:
        return {"value": ChannelDescriptor(shape=(self._dim,), dtype="f4")}

    def sample(self) -> dict[str, np.ndarray]:
        return {"value": np.zeros(self._dim, dtype=np.float32)}

    def apply(self, frame, *, policy: ReplayPolicy) -> None:
        self._last_applied = {k: np.asarray(v) for k, v in frame.items()}

    def to_manifest(self) -> dict[str, object]:
        return {"type": self.TYPE_ID, "group": self.group, "dim": self._dim}

    @classmethod
    def from_manifest(cls, entry):
        return cls(group=str(entry["group"]), dim=int(entry.get("dim", 3)))


class SessionStorageRoundtripTests(omni.kit.test.AsyncTestCase):
    async def setUp(self) -> None:
        await omni.kit.app.get_app().next_update_async()
        omni.usd.get_context().new_stage()
        await omni.kit.app.get_app().next_update_async()
        if _DummyRec.TYPE_ID not in registered_types():
            register_recordable(_DummyRec)

    async def tearDown(self) -> None:
        if _DummyRec.TYPE_ID in registered_types():
            unregister_recordable(_DummyRec.TYPE_ID)
        omni.usd.get_context().close_stage()
        await omni.kit.app.get_app().next_update_async()
        while omni.usd.get_context().get_stage_loading_status()[2] > 0:
            await omni.kit.app.get_app().next_update_async()

    async def test_manifest_and_frame_roundtrip(self) -> None:
        rec = _DummyRec(group="dummy", dim=4)

        with tempfile.TemporaryDirectory(prefix="session_storage_test_") as tmp_dir:
            hdf5_path = os.path.join(tmp_dir, "roundtrip.hdf5")
            storage = SessionStorage(hdf5_path, buffer_frames=2)
            storage.open()
            storage.set_root_attr("session_id", "unit_test_session")
            storage.write_manifest(
                build_manifest(
                    [rec.to_manifest()],
                    sampling={"mode": SamplingConfig().mode, "decimation": 1},
                    session_metadata={"unit": "test"},
                )
            )

            storage.begin_episode({rec.group: rec.describe_channels()})
            for i in range(5):
                storage.append_frame(rec.group, {"value": np.full(4, float(i), dtype=np.float32)})
                storage.advance_episode_frame()
            storage.end_episode(success=True)
            storage.close()

            reader = SessionReader(hdf5_path)
            try:
                self.assertEqual(reader.list_episodes(), ["episode_00000"])
                self.assertEqual(reader.num_frames("episode_00000"), 5)
                attrs = reader.episode_attrs("episode_00000")
                self.assertTrue(bool(attrs.get("success", False)))

                manifest = reader.manifest()
                self.assertEqual(len(manifest.tracks), 1)
                rehydrated = rehydrate(manifest.tracks[0])
                self.assertIsInstance(rehydrated, _DummyRec)
                self.assertEqual(rehydrated.group, "dummy")

                mid = reader.read_frame("episode_00000", "dummy", 3)
                np.testing.assert_allclose(mid["value"], np.full(4, 3.0, dtype=np.float32))

                full = reader.read_channel("episode_00000", "dummy", "value")
                self.assertEqual(full.shape, (5, 4))
                np.testing.assert_allclose(full[0], np.zeros(4, dtype=np.float32))
                np.testing.assert_allclose(full[-1], np.full(4, 4.0, dtype=np.float32))
            finally:
                reader.close()

    async def test_registry_rejects_unknown_type(self) -> None:
        with self.assertRaises(KeyError):
            rehydrate({"type": "__nonexistent__", "group": "x"})

    async def test_append_frame_rejects_missing_channels(self) -> None:
        with tempfile.TemporaryDirectory(prefix="session_storage_test_") as tmp_dir:
            hdf5_path = os.path.join(tmp_dir, "roundtrip.hdf5")
            storage = SessionStorage(hdf5_path, buffer_frames=2)
            storage.open()
            storage.begin_episode({"dummy": {"value": ChannelDescriptor(shape=(2,), dtype="f4")}})
            try:
                with self.assertRaises(KeyError):
                    storage.append_frame("dummy", {})
                self.assertEqual(storage.current_episode_frames, 0)
            finally:
                storage.close()

    async def test_multi_group_buffer_flush_keeps_channels_separate(self) -> None:
        with tempfile.TemporaryDirectory(prefix="session_storage_test_") as tmp_dir:
            hdf5_path = os.path.join(tmp_dir, "roundtrip.hdf5")
            storage = SessionStorage(hdf5_path, buffer_frames=2)
            storage.open()
            storage.write_manifest(
                build_manifest(
                    [
                        {"type": "_test", "group": "state/a"},
                        {"type": "_test", "group": "state/b"},
                    ]
                )
            )
            storage.begin_episode(
                {
                    "state/a": {"scalar": ChannelDescriptor(shape=(), dtype="i8")},
                    "state/b": {"vector": ChannelDescriptor(shape=(2,), dtype="f4")},
                }
            )
            for i in range(5):
                storage.append_frame("state/a", {"scalar": np.int64(i)})
                storage.append_frame("state/b", {"vector": np.array([i, i + 10], dtype=np.float32)})
                storage.advance_episode_frame()
            storage.end_episode(success=True)
            storage.close()

            reader = SessionReader(hdf5_path)
            try:
                np.testing.assert_array_equal(reader.read_channel(0, "state/a", "scalar"), np.arange(5, dtype=np.int64))
                np.testing.assert_allclose(
                    reader.read_channel(0, "state/b", "vector"),
                    np.array([[i, i + 10] for i in range(5)], dtype=np.float32),
                )
            finally:
                reader.close()


if __name__ == "__main__":
    unittest.main()
