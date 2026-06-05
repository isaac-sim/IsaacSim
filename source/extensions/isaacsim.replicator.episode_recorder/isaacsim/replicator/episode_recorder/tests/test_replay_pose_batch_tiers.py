# SPDX-FileCopyrightText: Copyright (c) 2025-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
# http://www.apache.org/licenses/LICENSE-2.0

"""Tests for the replayer's ancestry-ordered pose batch tiers.

Covers:

* :func:`_assign_ancestry_tiers` — pure-Python topological grouping, including the
  flat / nested / mixed / deeply-nested cases.
* End-to-end: a moving wrapper xform plus a child link in the same batch must
  end up with the child tracking the wrapper's recorded world pose exactly,
  even though both prims are in the same shared batch (the regression that the
  tier split fixes — without it, the child lags by one frame).
* :class:`EpisodeRecorder` / :class:`EpisodeReplayer` honor the ``pose_backend``
  argument, defaulting to ``"usd"``.
* The per-tier missing-xformOps reset retry is scoped per tier, so a second
  tier hitting the same fault on its first frame still recovers cleanly
  instead of inheriting an earlier tier's already-reset flag.
"""

from __future__ import annotations

import tempfile
from contextlib import contextmanager
from typing import Any
from unittest.mock import patch

import carb
import isaacsim.core.experimental.utils.stage as stage_utils
import numpy as np
import omni.kit.app
import omni.kit.test
import omni.usd
from isaacsim.replicator.episode_recorder import (
    EpisodeRecorder,
    EpisodeReplayer,
    SessionStorage,
    XformRecordable,
)
from isaacsim.replicator.episode_recorder import _pose_backend as pb_module
from isaacsim.replicator.episode_recorder import (
    build_manifest,
)
from isaacsim.replicator.episode_recorder.replayer import EpisodeReplayer as _Replayer
from isaacsim.replicator.episode_recorder.replayer import _assign_ancestry_tiers


@contextmanager
def _capture_log_warn():
    """Capture ``carb.log_warn`` calls, scoped to the ``with`` block.

    Restores the original ``carb.log_warn`` on exit even if the body raises,
    so test isolation is preserved across the rest of the suite.
    """
    captured: list[str] = []
    original = carb.log_warn

    def _capture(msg: str) -> None:
        captured.append(msg)
        original(msg)

    with patch.object(carb, "log_warn", new=_capture):
        yield captured


class AncestryTierAssignmentTests(omni.kit.test.AsyncTestCase):
    async def test_empty_paths_returns_empty_tiers(self) -> None:
        self.assertEqual(_assign_ancestry_tiers([]), [])

    @staticmethod
    def _assert_tiers(tiers: list[list[int]] | None) -> list[list[int]]:
        """Assert ``tiers`` is non-None and return it for downstream type-safe access.

        Both ``unittest.assertIsNotNone`` *and* a plain ``assert`` are needed:
        the former produces a proper test failure, the latter narrows the type
        for the rest of the body (basedpyright cannot see through assert helpers).
        Wrapped here so each test stays a single-statement check.
        """
        assert tiers is not None, "_assign_ancestry_tiers returned None unexpectedly"
        return tiers

    async def test_flat_paths_collapse_into_single_tier(self) -> None:
        paths = ["/World/A", "/World/B", "/World/C"]
        tiers = self._assert_tiers(_assign_ancestry_tiers(paths))
        self.assertEqual(len(tiers), 1)
        self.assertEqual(sorted(tiers[0]), [0, 1, 2])

    async def test_parent_and_child_split_into_two_tiers(self) -> None:
        paths = ["/World/Wrapper", "/World/Wrapper/Robot/base"]
        tiers = self._assert_tiers(_assign_ancestry_tiers(paths))
        self.assertEqual(len(tiers), 2)
        self.assertEqual(tiers[0], [0])
        self.assertEqual(tiers[1], [1])

    async def test_deeply_nested_paths_assign_increasing_tiers(self) -> None:
        paths = ["/A", "/A/B", "/A/B/C", "/A/B/C/D"]
        tiers = self._assert_tiers(_assign_ancestry_tiers(paths))
        self.assertEqual([t for t in tiers], [[0], [1], [2], [3]])

    async def test_mixed_ancestry_groups_siblings_correctly(self) -> None:
        paths = [
            "/World/Wrapper",
            "/World/Wrapper/Robot/base",
            "/World/Wrapper/Robot/wheel",
            "/World/Other",
        ]
        tiers = self._assert_tiers(_assign_ancestry_tiers(paths))
        self.assertEqual(len(tiers), 2)
        self.assertEqual(sorted(tiers[0]), [0, 3])
        self.assertEqual(sorted(tiers[1]), [1, 2])

    async def test_input_order_independent_of_tier_correctness(self) -> None:
        paths = ["/World/Wrapper/Robot/base", "/World/Wrapper"]
        tiers = self._assert_tiers(_assign_ancestry_tiers(paths))
        self.assertEqual(len(tiers), 2)
        self.assertEqual(tiers[0], [1])
        self.assertEqual(tiers[1], [0])

    async def test_duplicate_paths_share_tier(self) -> None:
        """Two recordables targeting the same prim must land in one tier (last write wins)."""
        paths = ["/World/Wrapper", "/World/Wrapper/Child", "/World/Wrapper/Child"]
        tiers = self._assert_tiers(_assign_ancestry_tiers(paths))
        self.assertEqual(len(tiers), 2)
        self.assertEqual(tiers[0], [0])
        self.assertEqual(sorted(tiers[1]), [1, 2])

    async def test_invalid_path_returns_none(self) -> None:
        self.assertIsNone(_assign_ancestry_tiers(["not-a-path"]))

    async def test_property_path_returns_none(self) -> None:
        """Property paths (``/Foo.attr``) are not prim paths and must reject."""
        self.assertIsNone(_assign_ancestry_tiers(["/Foo.attr"]))


class ReplayerPoseBatchTierIntegrationTests(omni.kit.test.AsyncTestCase):
    async def setUp(self) -> None:
        await omni.kit.app.get_app().next_update_async()
        omni.usd.get_context().new_stage()
        await omni.kit.app.get_app().next_update_async()

    async def tearDown(self) -> None:
        omni.usd.get_context().close_stage()
        await omni.kit.app.get_app().next_update_async()
        while omni.usd.get_context().get_stage_loading_status()[2] > 0:
            await omni.kit.app.get_app().next_update_async()

    async def test_nested_xform_replay_tracks_parent_exactly(self) -> None:
        """A moving wrapper plus a child in the same batch must replay without lag.

        Before the tier split: ``XformPrim.set_world_poses`` over both prims read the
        child's parent transform once *before* writing the wrapper's new pose, so the
        child's authored local pose was computed against the stale wrapper position
        and the resulting world pose lagged by one frame's worth of locomotion.

        With the split, tier 0 (wrapper) is written first; tier 1 (child) reads the
        already-updated wrapper world transform and tracks the recorded world pose
        exactly.
        """
        stage_utils.define_prim("/World", "Xform")
        stage_utils.define_prim("/World/Wrapper", "Xform")
        stage_utils.define_prim("/World/Wrapper/Child", "Xform")

        wrapper_rec = XformRecordable(group="state/wrapper", prim_path="/World/Wrapper")
        child_rec = XformRecordable(group="state/child", prim_path="/World/Wrapper/Child")

        with tempfile.TemporaryDirectory(prefix="replayer_tier_test_") as tmp_dir:
            path = f"{tmp_dir}/nested_replay.hdf5"
            storage = SessionStorage(path, buffer_frames=4)
            storage.open()
            storage.write_manifest(build_manifest([wrapper_rec.to_manifest(), child_rec.to_manifest()]))
            storage.begin_episode(
                {
                    wrapper_rec.group: wrapper_rec.describe_channels(),
                    child_rec.group: child_rec.describe_channels(),
                }
            )

            wrapper_positions = [
                np.array([0.0, 0.0, 0.0], dtype=np.float32),
                np.array([5.0, 0.0, 0.0], dtype=np.float32),
                np.array([10.0, 1.0, 0.0], dtype=np.float32),
            ]
            child_positions = [
                np.array([0.0, 0.0, 0.0], dtype=np.float32),
                np.array([5.0, 0.0, 1.5], dtype=np.float32),
                np.array([10.0, 1.0, 1.5], dtype=np.float32),
            ]
            identity_quat = np.array([1.0, 0.0, 0.0, 0.0], dtype=np.float32)

            for wp_pos, ch_pos in zip(wrapper_positions, child_positions, strict=True):
                storage.append_frame(wrapper_rec.group, {"position": wp_pos, "orientation": identity_quat})
                storage.append_frame(child_rec.group, {"position": ch_pos, "orientation": identity_quat})
                storage.advance_episode_frame()
            storage.end_episode(success=True)
            storage.close()

            replayer = EpisodeReplayer(path)
            try:
                replayer.prepare_episode(0)
                self.assertEqual(replayer.pose_batch_size, 2)
                self.assertEqual(
                    replayer.pose_batch_tier_count,
                    2,
                    "Wrapper + nested child must split into two ancestry tiers",
                )

                from isaacsim.core.experimental.prims import XformPrim

                wrapper_xform = XformPrim("/World/Wrapper")
                child_xform = XformPrim("/World/Wrapper/Child")

                for frame_idx, (expected_wp, expected_child) in enumerate(
                    zip(wrapper_positions, child_positions, strict=True)
                ):
                    replayer.apply_frame(frame_idx)
                    wp_pos, _ = wrapper_xform.get_world_poses()
                    ch_pos, _ = child_xform.get_world_poses()
                    wp_np = wp_pos.numpy() if hasattr(wp_pos, "numpy") else np.asarray(wp_pos)
                    ch_np = ch_pos.numpy() if hasattr(ch_pos, "numpy") else np.asarray(ch_pos)
                    np.testing.assert_allclose(
                        wp_np.reshape(-1, 3)[0],
                        expected_wp,
                        atol=1e-5,
                        err_msg=f"frame {frame_idx} wrapper pose mismatch",
                    )
                    np.testing.assert_allclose(
                        ch_np.reshape(-1, 3)[0],
                        expected_child,
                        atol=1e-5,
                        err_msg=(
                            f"frame {frame_idx} child world pose lagged the wrapper — the tier "
                            "split should keep nested replay in lockstep"
                        ),
                    )
            finally:
                replayer.close()

    async def test_flat_batch_collapses_to_single_tier(self) -> None:
        stage_utils.define_prim("/World", "Xform")
        stage_utils.define_prim("/World/A", "Xform")
        stage_utils.define_prim("/World/B", "Xform")

        rec_a = XformRecordable(group="state/a", prim_path="/World/A")
        rec_b = XformRecordable(group="state/b", prim_path="/World/B")

        with tempfile.TemporaryDirectory(prefix="replayer_tier_test_") as tmp_dir:
            path = f"{tmp_dir}/flat_replay.hdf5"
            storage = SessionStorage(path, buffer_frames=2)
            storage.open()
            storage.write_manifest(build_manifest([rec_a.to_manifest(), rec_b.to_manifest()]))
            storage.begin_episode({rec_a.group: rec_a.describe_channels(), rec_b.group: rec_b.describe_channels()})
            identity = np.array([1.0, 0.0, 0.0, 0.0], dtype=np.float32)
            storage.append_frame(rec_a.group, {"position": np.zeros(3, dtype=np.float32), "orientation": identity})
            storage.append_frame(rec_b.group, {"position": np.zeros(3, dtype=np.float32), "orientation": identity})
            storage.advance_episode_frame()
            storage.end_episode(success=True)
            storage.close()

            replayer = EpisodeReplayer(path)
            try:
                replayer.prepare_episode(0)
                self.assertEqual(replayer.pose_batch_size, 2)
                self.assertEqual(
                    replayer.pose_batch_tier_count,
                    1,
                    "Sibling batches with no overlapping ancestry should collapse to one tier",
                )
            finally:
                replayer.close()

    async def test_pose_backend_default_is_usd(self) -> None:
        with tempfile.TemporaryDirectory(prefix="pose_backend_test_") as tmp_dir:
            recorder = EpisodeRecorder(tmp_dir)
            self.assertEqual(recorder.pose_backend, "usd")

            stage_utils.define_prim("/World", "Xform")
            stage_utils.define_prim("/World/A", "Xform")
            rec = XformRecordable(group="state/a", prim_path="/World/A")
            path = f"{tmp_dir}/dummy.hdf5"
            storage = SessionStorage(path, buffer_frames=1)
            storage.open()
            storage.write_manifest(build_manifest([rec.to_manifest()]))
            storage.begin_episode({rec.group: rec.describe_channels()})
            storage.append_frame(
                rec.group,
                {
                    "position": np.zeros(3, dtype=np.float32),
                    "orientation": np.array([1.0, 0.0, 0.0, 0.0], dtype=np.float32),
                },
            )
            storage.advance_episode_frame()
            storage.end_episode(success=True)
            storage.close()

            replayer = EpisodeReplayer(path)
            try:
                self.assertEqual(replayer.pose_backend, "usd")
            finally:
                replayer.close()

    async def test_pose_backend_unknown_falls_back_to_usd(self) -> None:
        with tempfile.TemporaryDirectory(prefix="pose_backend_test_") as tmp_dir:
            with _capture_log_warn() as warns:
                recorder = EpisodeRecorder(tmp_dir, pose_backend="bogus")  # type: ignore[arg-type]
            self.assertEqual(
                recorder.pose_backend,
                "usd",
                "Unsupported pose_backend selectors must fall back to the safe default",
            )
            self.assertTrue(
                any("'bogus'" in w and "not supported" in w for w in warns),
                f"Expected a 'not supported' warning for the typo'd backend; got {warns!r}",
            )


class _RecordingTierBatch:
    """Stand-in for an :class:`XformPrim` tier batch used by ``_apply_replay_pose_batch``.

    Each call to :meth:`set_world_poses` is recorded; the first ``fail_first_n``
    calls raise a synthetic missing-xformOps error so the per-tier reset retry
    is exercised. ``reset_xform_op_properties`` is recorded so the test can
    assert each tier reset itself independently.
    """

    def __init__(self, *, fail_first_n: int) -> None:
        self.fail_first_n = fail_first_n
        self.write_count = 0
        self.reset_count = 0

    def set_world_poses(self, *, positions: Any, orientations: Any) -> None:  # noqa: ARG002
        self.write_count += 1
        if self.write_count <= self.fail_first_n:
            # Mirror the assertion `XformPrim.set_world_poses` raises when the
            # target prim has no `xformOp:translate` authored - it's the same
            # text `is_missing_xform_ops_error` matches against.
            raise RuntimeError(
                "Undefined 'xformOp:translate' property for the prim. "
                "Set the 'reset_xform_op_properties' parameter to True when initializing the class, "
                "or call '.reset_xform_op_properties()' manually before doing transform operations"
            )

    def reset_xform_op_properties(self) -> None:
        self.reset_count += 1


class ReplayPoseBatchPerTierResetTests(omni.kit.test.AsyncTestCase):
    """Direct unit tests for the per-tier reset retry in ``_apply_replay_pose_batch``.

    The end-to-end nested-xform test exercises the happy path; these tests target
    the failure path that the previous instance-wide ``_replay_pose_batch_reset``
    flag broke for any tier after the first.
    """

    @staticmethod
    def _seed_replayer(tier_batches: list[_RecordingTierBatch]) -> _Replayer:
        replayer = _Replayer.__new__(_Replayer)
        replayer._pose_backend = "usd"
        replayer._replay_pose_batch_tiers = list(tier_batches)
        replayer._replay_pose_batch_tier_indices = [np.array([i], dtype=np.int64) for i in range(len(tier_batches))]
        replayer._replay_pose_batch_tier_selectors = [np.array([i], dtype=np.int64) for i in range(len(tier_batches))]
        replayer._replay_pose_batch_reset_tiers = set()
        replayer._replay_pose_positions = np.zeros((len(tier_batches), 3), dtype=np.float32)
        replayer._replay_pose_orientations = np.tile(
            np.array([1.0, 0.0, 0.0, 0.0], dtype=np.float32), (len(tier_batches), 1)
        )
        return replayer

    async def test_each_tier_resets_independently_on_missing_xform_ops(self) -> None:
        """Two tiers, each failing once - both must reset and recover."""
        tier0 = _RecordingTierBatch(fail_first_n=1)
        tier1 = _RecordingTierBatch(fail_first_n=1)
        replayer = self._seed_replayer([tier0, tier1])
        replayer._apply_replay_pose_batch()
        self.assertEqual(tier0.reset_count, 1, "tier 0 should reset once on its first-frame failure")
        self.assertEqual(tier1.reset_count, 1, "tier 1 should reset once on its first-frame failure")
        self.assertEqual(tier0.write_count, 2, "tier 0 should retry the write after the reset")
        self.assertEqual(tier1.write_count, 2, "tier 1 should retry the write after the reset")
        self.assertEqual(replayer._replay_pose_batch_reset_tiers, {0, 1})

    async def test_second_failure_in_same_tier_propagates(self) -> None:
        """A tier that fails twice in a row must not be retried again - the second failure escapes."""
        tier0 = _RecordingTierBatch(fail_first_n=2)
        replayer = self._seed_replayer([tier0])
        with self.assertRaises(RuntimeError):
            replayer._apply_replay_pose_batch()
        self.assertEqual(tier0.reset_count, 1, "second failure on the same tier must NOT trigger another reset")

    async def test_unrelated_exception_propagates_without_reset(self) -> None:
        """Non-missing-xformOps errors must propagate immediately, no reset attempted."""

        class _NonRecoverableBatch(_RecordingTierBatch):
            def set_world_poses(self, *, positions: Any, orientations: Any) -> None:  # noqa: ARG002
                raise ValueError("not a missing-xformOps error")

        tier0 = _NonRecoverableBatch(fail_first_n=1)
        replayer = self._seed_replayer([tier0])
        with self.assertRaises(ValueError):
            replayer._apply_replay_pose_batch()
        self.assertEqual(tier0.reset_count, 0)


class PoseBackendCtxApplicationTests(omni.kit.test.AsyncTestCase):
    """Verify ``pose_backend_ctx`` actually invokes ``use_backend`` for non-USD backends.

    FSD state is mocked at the module-level ``_fsd_enabled`` helper instead of
    poking ``carb.settings`` directly: the live settings object is a
    pybind11-bound singleton whose methods cannot be reliably patched, and the
    helper exists exactly so tests can swap it out.
    """

    async def setUp(self) -> None:
        # ``_demotion_warned`` is module-level state for the one-shot warning;
        # clear it before each test so warning counts are deterministic.
        pb_module._demotion_warned.clear()

    async def tearDown(self) -> None:
        pb_module._demotion_warned.clear()

    async def test_usd_backend_returns_nullcontext(self) -> None:
        ctx = pb_module.pose_backend_ctx("usd")
        self.assertEqual(type(ctx).__name__, "nullcontext")

    async def test_non_usd_backend_invokes_use_backend_when_fsd_enabled(self) -> None:
        seen: list[str] = []

        @contextmanager
        def _fake_use_backend(backend: str):
            seen.append(backend)
            yield

        with patch.object(pb_module, "_fsd_enabled", lambda: True):
            with patch("isaacsim.core.experimental.utils.backend.use_backend", _fake_use_backend):
                with pb_module.pose_backend_ctx("usdrt"):
                    pass
        self.assertEqual(seen, ["usdrt"], "pose_backend_ctx must delegate to use_backend(backend) when FSD is on")

    async def test_non_usd_backend_silently_demotes_when_fsd_off(self) -> None:
        """A mid-session FSD toggle must demote to nullcontext + log once per backend."""
        with patch.object(pb_module, "_fsd_enabled", lambda: False):
            with _capture_log_warn() as warns:
                with pb_module.pose_backend_ctx("usdrt"):
                    pass
                with pb_module.pose_backend_ctx("usdrt"):
                    pass
        usdrt_warns = [w for w in warns if "usdrt" in w and "demoting" in w]
        self.assertEqual(len(usdrt_warns), 1, f"Expected exactly one demotion warning for 'usdrt'; got {warns!r}")
