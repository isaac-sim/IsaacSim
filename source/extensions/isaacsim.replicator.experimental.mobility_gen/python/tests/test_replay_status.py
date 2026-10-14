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

"""Tests for replay completion markers, configuration matching, and output management."""

import argparse
import os
import shutil
import tempfile
from typing import Any

import omni.kit.test
from isaacsim.replicator.experimental.mobility_gen import (
    COMPLETE_MARKER_NAME,
    MAX_RENDER_RETRIES,
    REPLAY_CONFIG_NAME,
    clear_replay_outputs,
    discard_step_common,
    format_dropped_steps,
    is_complete,
    is_in_place_replay,
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
        "write_interval": 20,
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

    async def test_replay_config_from_args_includes_write_interval_when_present(self) -> None:
        """Replay configs include the requested write interval when provided."""
        config = replay_config_from_args(self._recording, _args(write_interval=20))
        self.assertEqual(config["write_interval"], 20)

    async def test_completion_marker_tracks_written_frames(self) -> None:
        """Sparse replay reports the number of output frames separately from rendered frames."""
        mark_replay_complete(self._output, frames_rendered=25, frames_written=2)
        with open(os.path.join(self._output, COMPLETE_MARKER_NAME)) as marker:
            self.assertEqual(marker.read(), "frames_rendered: 25\nframes_written: 2\n")

    async def test_completion_marker_defaults_to_writing_every_frame(self) -> None:
        """Existing callers retain matching rendered and written frame counts."""
        mark_replay_complete(self._output, frames_rendered=5)
        with open(os.path.join(self._output, COMPLETE_MARKER_NAME)) as marker:
            self.assertEqual(marker.read(), "frames_rendered: 5\nframes_written: 5\n")

    async def test_write_interval_mismatch_removes_stale_marker(self) -> None:
        """Changing the output cadence requires regenerating a completed replay."""
        config = replay_config_from_args(self._recording, _args(write_interval=1))
        write_replay_config(self._output, config)
        mark_replay_complete(self._output, frames_rendered=25)
        next_config = replay_config_from_args(self._recording, _args(write_interval=20))

        self.assertFalse(is_complete(self._output, next_config))
        self.assertFalse(os.path.exists(os.path.join(self._output, COMPLETE_MARKER_NAME)))

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


class TestReplayOutputs(omni.kit.test.AsyncTestCase):
    """Replay output helpers clear regenerated state without destroying recorded input."""

    async def setUp(self) -> None:
        """Create a temporary recording and output location."""
        self._tmp = tempfile.mkdtemp(prefix="test_replay_outputs_")
        self._recording = os.path.join(self._tmp, "recording")
        self._output = os.path.join(self._tmp, "output")
        os.makedirs(self._recording)

    async def tearDown(self) -> None:
        """Remove the temporary replay output workspace."""
        shutil.rmtree(self._tmp, ignore_errors=True)

    def _populate(self, path: str, steps: int = 2) -> None:
        """Write a recording-shaped tree of common state and rendered images.

        Args:
            path: Directory to populate.
            steps: Number of steps to write.
        """
        for modality, extension in (
            ("common", "npz"),
            ("rgb", "jpg"),
            ("instance_id_segmentation", "npy"),
            ("depth", "png"),
        ):
            os.makedirs(os.path.join(path, "state", modality), exist_ok=True)
            for step in range(steps):
                with open(os.path.join(path, "state", modality, f"{step:08d}.{extension}"), "w") as f:
                    f.write("x")
        with open(os.path.join(path, REPLAY_CONFIG_NAME), "w") as f:
            f.write("source_recording: x\n")

    def _common_steps(self, path: str) -> list[str]:
        """List the step files remaining under state/common.

        Args:
            path: Replay directory to inspect.

        Returns:
            The sorted file names under state/common, empty if the directory is gone.
        """
        common_dir = os.path.join(path, "state", "common")
        return sorted(os.listdir(common_dir)) if os.path.isdir(common_dir) else []

    async def test_format_dropped_steps_truncates_long_lists(self) -> None:
        """A dropped-step summary lists every index up to the cap, then counts the rest."""
        self.assertEqual(format_dropped_steps([]), "")
        self.assertEqual(format_dropped_steps([3, 7]), "3, 7")
        self.assertEqual(format_dropped_steps([0, 1, 2], max_listed=3), "0, 1, 2")
        self.assertEqual(format_dropped_steps(list(range(5)), max_listed=3), "0, 1, 2, ... (+2 more)")

    async def test_in_place_replay_is_detected_through_path_spellings(self) -> None:
        """An in-place replay is recognised however the two paths happen to be spelled."""
        self.assertFalse(is_in_place_replay(self._output, self._recording))
        self.assertTrue(is_in_place_replay(self._recording, self._recording))
        self.assertTrue(is_in_place_replay(self._recording + os.sep, self._recording))
        self.assertTrue(is_in_place_replay(os.path.join(self._recording, ".", ""), self._recording))

    async def test_discard_step_common_removes_only_the_named_step(self) -> None:
        """Discarding a step's common entry leaves every other step untouched."""
        self._populate(self._recording, steps=3)

        discard_step_common(self._recording, 1)

        self.assertEqual(self._common_steps(self._recording), ["00000000.npz", "00000002.npz"])
        # A step with no common entry is the out-of-place case, where there is nothing to remove.
        discard_step_common(self._recording, 1)
        self.assertEqual(self._common_steps(self._recording), ["00000000.npz", "00000002.npz"])

    async def test_out_of_place_replay_clears_stale_common_entries(self) -> None:
        """A replay writing to its own directory clears the common state an earlier run left."""
        self._populate(self._recording)
        self._populate(self._output)
        mark_replay_complete(self._output, frames_rendered=2)

        clear_replay_outputs(self._output, self._recording)

        # state/common is regenerated step by step out of place, so an entry surviving with its
        # images deleted would be a step MobilityGenReader indexes but cannot read.
        self.assertEqual(self._common_steps(self._output), [])
        self.assertFalse(os.path.isdir(os.path.join(self._output, "state", "rgb")))
        self.assertFalse(os.path.isdir(os.path.join(self._output, "state", "instance_id_segmentation")))
        self.assertFalse(os.path.exists(os.path.join(self._output, REPLAY_CONFIG_NAME)))
        self.assertFalse(os.path.exists(os.path.join(self._output, COMPLETE_MARKER_NAME)))
        # The source recording is untouched.
        self.assertEqual(self._common_steps(self._recording), ["00000000.npz", "00000001.npz"])

    async def test_in_place_replay_keeps_common_state(self) -> None:
        """A replay writing over its own recording keeps the recorded poses it reads as input."""
        self._populate(self._recording)
        mark_replay_complete(self._recording, frames_rendered=2)

        clear_replay_outputs(self._recording, self._recording)

        # In place, state/common IS the input; clearing it would destroy the recording.
        self.assertEqual(self._common_steps(self._recording), ["00000000.npz", "00000001.npz"])
        self.assertFalse(os.path.isdir(os.path.join(self._recording, "state", "rgb")))
        self.assertFalse(os.path.isdir(os.path.join(self._recording, "state", "instance_id_segmentation")))
        self.assertFalse(os.path.exists(os.path.join(self._recording, COMPLETE_MARKER_NAME)))

    async def test_render_retry_budget_is_bounded_and_positive(self) -> None:
        """The retry budget re-renders a transient miss without letting a permanent one spin."""
        self.assertGreater(MAX_RENDER_RETRIES, 0)
        self.assertLess(MAX_RENDER_RETRIES, 10)
