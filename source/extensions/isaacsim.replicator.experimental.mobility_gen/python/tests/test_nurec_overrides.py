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

import omni.kit.test
from isaacsim.replicator.experimental.mobility_gen import ensure_nurec_replay_flags, setup_for_replay


class TestNurecReplayOverrides(omni.kit.test.AsyncTestCase):
    """The replay-flag policy restricts NuRec replay to RGB; setup is a no-op without a stage."""

    async def test_ensure_forces_rgb_only(self) -> None:
        """All non-RGB modality flags are turned off; rgb_enabled stays on."""
        args = argparse.Namespace(
            rgb_enabled=True,
            segmentation_enabled=True,
            depth_enabled=True,
            instance_id_segmentation_enabled=True,
            normals_enabled=True,
            render_rt_subframes=64,
        )
        ensure_nurec_replay_flags(args)
        self.assertTrue(args.rgb_enabled)
        self.assertFalse(args.segmentation_enabled)
        self.assertFalse(args.depth_enabled)
        self.assertFalse(args.instance_id_segmentation_enabled)
        self.assertFalse(args.normals_enabled)

    async def test_ensure_skips_missing_attributes(self) -> None:
        """Flags absent from the namespace are left absent (skipped, not created)."""
        args = argparse.Namespace(rgb_enabled=False)
        ensure_nurec_replay_flags(args)
        self.assertTrue(args.rgb_enabled)
        self.assertFalse(hasattr(args, "depth_enabled"))

    async def test_setup_for_replay_none_stage_is_noop(self) -> None:
        """A None stage is treated as non-NuRec: nothing detected and flags untouched."""
        args = argparse.Namespace(rgb_enabled=False, depth_enabled=True)
        result = setup_for_replay(args, None)
        self.assertEqual(result, (True, False, False, []))
        self.assertFalse(args.rgb_enabled)
        self.assertTrue(args.depth_enabled)
