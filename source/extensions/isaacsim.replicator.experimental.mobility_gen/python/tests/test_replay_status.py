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

"""Tests for replay completion marker and configuration matching."""

import argparse
import os
import shutil
import tempfile
from typing import Any

import omni.kit.test
from isaacsim.replicator.experimental.mobility_gen import (
    COMPLETE_MARKER_NAME,
    is_complete,
    mark_replay_complete,
    replay_config_from_args,
    write_replay_config,
)


def _args(**overrides: Any) -> argparse.Namespace:
    """Build parsed replay arguments with optional overrides.

    Args:
        **overrides: Values to override in the default argument namespace.

    Returns:
        Parsed replay argument namespace.
    """
    values = {
        "self_contained": False,
        "render_interval": 5,
        "render_rt_subframes": 36,
        "warmup_frames": 4,
        "max_frames": None,
        "rgb_enabled": True,
        "segmentation_enabled": False,
        "depth_enabled": False,
        "instance_id_segmentation_enabled": False,
        "normals_enabled": False,
    }
    values.update(overrides)
    return argparse.Namespace(**values)


class TestReplayStatus(omni.kit.test.AsyncTestCase):
    """Replay status helpers match marker files to replay configurations."""

    async def setUp(self) -> None:
        """Create a temporary recording and output location."""
        self._tmp = tempfile.mkdtemp(prefix="test_replay_status_")
        self._recording = os.path.join(self._tmp, "recording")
        self._output = os.path.join(self._tmp, "output")
        os.makedirs(self._recording)

    async def tearDown(self) -> None:
        """Remove the temporary replay status workspace."""
        shutil.rmtree(self._tmp, ignore_errors=True)

    async def test_replay_config_from_args_includes_max_frames(self) -> None:
        """Replay configs include the requested max frame count."""
        config = replay_config_from_args(self._recording, _args(max_frames=12))
        self.assertEqual(config["source_recording"], os.path.abspath(self._recording))
        self.assertEqual(config["max_frames"], 12)

    async def test_complete_requires_marker_and_matching_config(self) -> None:
        """A replay is complete only with a marker and matching config."""
        config = replay_config_from_args(self._recording, _args())
        write_replay_config(self._output, config)
        self.assertFalse(is_complete(self._output, config))

        mark_replay_complete(self._output, frames_rendered=5)
        self.assertTrue(is_complete(self._output, config))

    async def test_config_mismatch_prevents_skip(self) -> None:
        """A replay config mismatch prevents completion reuse."""
        config = replay_config_from_args(self._recording, _args(render_interval=5))
        other_config = replay_config_from_args(self._recording, _args(render_interval=10))
        write_replay_config(self._output, config)
        mark_replay_complete(self._output, frames_rendered=5)

        self.assertFalse(is_complete(self._output, other_config))

    async def test_max_frames_mismatch_removes_stale_marker(self) -> None:
        """A max-frame mismatch removes a stale completion marker."""
        config = replay_config_from_args(self._recording, _args(max_frames=25))
        next_config = replay_config_from_args(self._recording, _args(max_frames=None))
        write_replay_config(self._output, config)
        mark_replay_complete(self._output, frames_rendered=25)

        self.assertFalse(is_complete(self._output, next_config))
        self.assertFalse(os.path.exists(os.path.join(self._output, COMPLETE_MARKER_NAME)))

    async def test_matching_config_keeps_marker(self) -> None:
        """A matching replay config keeps the completion marker."""
        config = replay_config_from_args(self._recording, _args())
        write_replay_config(self._output, config)
        mark_replay_complete(self._output, frames_rendered=5)

        self.assertTrue(is_complete(self._output, config))
        self.assertTrue(os.path.exists(os.path.join(self._output, COMPLETE_MARKER_NAME)))

    async def test_missing_config_removes_stale_marker(self) -> None:
        """A missing replay config removes a stale completion marker."""
        os.makedirs(self._output)
        mark_replay_complete(self._output, frames_rendered=5)
        config = replay_config_from_args(self._recording, _args())

        self.assertFalse(is_complete(self._output, config))
        self.assertFalse(os.path.exists(os.path.join(self._output, COMPLETE_MARKER_NAME)))

    async def test_invalid_config_removes_stale_marker(self) -> None:
        """An invalid replay config removes a stale completion marker."""
        os.makedirs(self._output)
        with open(os.path.join(self._output, "replay_config.yaml"), "w") as f:
            f.write("[not: valid: yaml\n")
        mark_replay_complete(self._output, frames_rendered=5)
        config = replay_config_from_args(self._recording, _args())

        self.assertFalse(is_complete(self._output, config))
        self.assertFalse(os.path.exists(os.path.join(self._output, COMPLETE_MARKER_NAME)))
