# SPDX-FileCopyrightText: Copyright (c) 2025-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

"""Tests for the NuRec replay-flag policy and replay setup."""

import argparse
from unittest import mock

import omni.kit.test
from isaacsim.replicator.experimental.mobility_gen import ensure_nurec_replay_flags, setup_for_replay
from isaacsim.replicator.experimental.mobility_gen.impl import nurec_overrides

# setup_for_replay only checks this for None; a non-None placeholder reaches setup_for_rendering,
# which the tests below patch.
_STAGE_SENTINEL = object()


class TestNurecReplayOverrides(omni.kit.test.AsyncTestCase):
    """NuRec replay preserves RGB/depth choices and disables restricted modalities."""

    async def test_ensure_preserves_rgb_and_depth_flags(self) -> None:
        """Every RGB/depth combination survives the NuRec modality restrictions."""
        for rgb_enabled in (False, True):
            for depth_enabled in (False, True):
                with self.subTest(rgb_enabled=rgb_enabled, depth_enabled=depth_enabled):
                    args = argparse.Namespace(
                        rgb_enabled=rgb_enabled,
                        segmentation_enabled=True,
                        depth_enabled=depth_enabled,
                        instance_id_segmentation_enabled=True,
                        normals_enabled=True,
                        render_rt_subframes=64,
                    )
                    ensure_nurec_replay_flags(args)
                    self.assertEqual(args.rgb_enabled, rgb_enabled)
                    self.assertEqual(args.depth_enabled, depth_enabled)
                    self.assertFalse(args.segmentation_enabled)
                    self.assertFalse(args.instance_id_segmentation_enabled)
                    self.assertFalse(args.normals_enabled)

    async def test_ensure_skips_missing_attributes(self) -> None:
        """Flags absent from the namespace are left absent (skipped, not created)."""
        args = argparse.Namespace(rgb_enabled=False)
        ensure_nurec_replay_flags(args)
        self.assertFalse(args.rgb_enabled)
        self.assertFalse(hasattr(args, "depth_enabled"))

    async def test_setup_for_replay_none_stage_is_noop(self) -> None:
        """A None stage is treated as non-NuRec: nothing detected and flags untouched."""
        args = argparse.Namespace(rgb_enabled=False, depth_enabled=True)
        result = setup_for_replay(args, None)
        self.assertEqual(result, (True, False, False, []))
        self.assertFalse(args.rgb_enabled)
        self.assertTrue(args.depth_enabled)

    async def test_setup_for_replay_raises_on_unmet_prerequisites(self) -> None:
        """An unmet NuRec launch prerequisite stops replay instead of proceeding into a crash."""
        args = argparse.Namespace(rgb_enabled=True, depth_enabled=True)
        problems = ["/renderer/multiGpu/enabled is true (launch with: --/renderer/multiGpu/enabled=false)"]
        with mock.patch.object(nurec_overrides, "setup_for_rendering", return_value=(False, True, False, problems)):
            with self.assertRaises(RuntimeError) as ctx:
                setup_for_replay(args, _STAGE_SENTINEL)

        # The message must carry the actionable launch setting, not just "failed".
        self.assertIn("multiGpu", str(ctx.exception))
        # The stage is unusable, so the replay flags must be left as the caller set them.
        self.assertTrue(args.depth_enabled)

    async def test_setup_for_replay_reports_success_when_it_returns(self) -> None:
        """An unmet prerequisite raises, so a caller that gets a value back can trust it."""
        args = argparse.Namespace(rgb_enabled=True, depth_enabled=True)
        with mock.patch.object(nurec_overrides, "setup_for_rendering", return_value=(True, True, False, [])):
            success, _, _, problems = setup_for_replay(args, _STAGE_SENTINEL)

        self.assertTrue(success)
        self.assertEqual(problems, [])

    async def test_setup_for_replay_does_not_raise_when_prerequisites_met(self) -> None:
        """A NuRec stage that satisfies its prerequisites preserves RGB and depth choices."""
        args = argparse.Namespace(rgb_enabled=True, depth_enabled=True)
        with mock.patch.object(nurec_overrides, "setup_for_rendering", return_value=(True, True, False, [])):
            success, nurec, _, problems = setup_for_replay(args, _STAGE_SENTINEL)

        self.assertTrue(success)
        self.assertTrue(nurec)
        self.assertEqual(problems, [])
        self.assertTrue(args.rgb_enabled)
        self.assertTrue(args.depth_enabled)

    async def test_setup_for_replay_preserves_depth_only_request(self) -> None:
        """NuRec setup does not enable RGB for a depth-only replay."""
        args = argparse.Namespace(rgb_enabled=False, depth_enabled=True)
        with mock.patch.object(nurec_overrides, "setup_for_rendering", return_value=(True, True, True, [])):
            setup_for_replay(args, _STAGE_SENTINEL)

        self.assertFalse(args.rgb_enabled)
        self.assertTrue(args.depth_enabled)

    async def test_setup_for_replay_non_nurec_preserves_all_flags(self) -> None:
        """Ordinary scenes keep every requested modality."""
        args = argparse.Namespace(
            rgb_enabled=False, depth_enabled=True, segmentation_enabled=True, normals_enabled=True
        )
        expected = vars(args).copy()
        with mock.patch.object(nurec_overrides, "setup_for_rendering", return_value=(True, False, False, [])):
            setup_for_replay(args, _STAGE_SENTINEL)

        self.assertEqual(vars(args), expected)
