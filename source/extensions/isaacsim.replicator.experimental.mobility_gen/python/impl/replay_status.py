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

"""Replay completion, settings manifest, and output management helpers."""

from __future__ import annotations

import argparse
import os
import shutil
from collections.abc import Callable
from typing import Any

import yaml

REPLAY_CONFIG_NAME = "replay_config.yaml"
COMPLETE_MARKER_NAME = ".complete"

# How many times a step whose capture came back incomplete is re-rendered before it is dropped.
# An annotator hands back an empty buffer until the renderer has produced a frame for its render
# product, which is usually transient, so one more render pass normally recovers it.
MAX_RENDER_RETRIES = 3

# Rendered-sensor state subdirs a replay (re)generates. state/common (recorded poses) is not one of
# them: an in-place replay reads it as its input. clear_replay_outputs() adds it back when the
# output is a separate directory, where it is regenerated step by step.
_RENDERED_STATE_DIRS = ("rgb", "segmentation", "instance_id_segmentation", "depth", "normals")


def replay_config_from_args(source_recording: str, args: argparse.Namespace) -> dict[str, Any]:
    """Build the replay settings manifest for a recording.

    Args:
        source_recording: Path to the source recording directory.
        args: Parsed replay arguments.

    Returns:
        Replay settings manifest dictionary.
    """
    config = {
        "source_recording": os.path.abspath(source_recording),
        "self_contained": args.self_contained,
        "render_interval": args.render_interval,
        "write_interval": args.write_interval,
        "render_rt_subframes": args.render_rt_subframes,
        "warmup_frames": args.warmup_frames,
        "max_frames": args.max_frames,
        "rgb_enabled": args.rgb_enabled,
        "segmentation_enabled": args.segmentation_enabled,
        "depth_enabled": args.depth_enabled,
        "instance_id_segmentation_enabled": args.instance_id_segmentation_enabled,
        "normals_enabled": args.normals_enabled,
    }
    return config


def write_replay_config(output_path: str, replay_config: dict[str, Any]) -> None:
    """Write replay settings to disk.

    Args:
        output_path: Replay output directory.
        replay_config: Replay settings manifest to write.
    """
    os.makedirs(output_path, exist_ok=True)
    with open(os.path.join(output_path, REPLAY_CONFIG_NAME), "w") as f:
        yaml.safe_dump(replay_config, f, default_flow_style=False, sort_keys=False)


def _read_replay_config(output_path: str) -> dict[str, Any] | None:
    path = os.path.join(output_path, REPLAY_CONFIG_NAME)
    if not os.path.isfile(path):
        return None
    try:
        with open(path) as f:
            config = yaml.safe_load(f)
    except yaml.YAMLError:
        return None
    if not isinstance(config, dict):
        return None
    return config


def mark_replay_complete(output_path: str, frames_rendered: int, *, frames_written: int | None = None) -> None:
    """Write the replay completion marker.

    Args:
        output_path: Replay output directory.
        frames_rendered: Number of frames rendered during replay.
        frames_written: Number of frames written to disk. Defaults to the rendered count.
    """
    os.makedirs(output_path, exist_ok=True)
    with open(os.path.join(output_path, COMPLETE_MARKER_NAME), "w") as f:
        f.write(f"frames_rendered: {frames_rendered}\n")
        f.write(f"frames_written: {frames_rendered if frames_written is None else frames_written}\n")


def is_complete(
    output_path: str,
    expected_config: dict[str, Any],
    *,
    replay_label: str | None = None,
    log_warn: Callable[[str], None] | None = None,
) -> bool:
    """Return True when the replay has a valid completion marker for this config.

    If a marker exists but the stored config is missing, invalid, or different,
    the marker is removed so the next run cannot skip stale output.

    Args:
        output_path: Replay output directory to inspect.
        expected_config: Replay configuration required for the marker to be valid.
        replay_label: Optional label used in warning messages. Defaults to None.
        log_warn: Optional warning callback. Defaults to None.

    Returns:
        True if the replay completion marker exists and matches ``expected_config``.
    """
    complete_marker = os.path.join(output_path, COMPLETE_MARKER_NAME)
    if not os.path.isfile(complete_marker):
        return False

    label = replay_label or os.path.basename(output_path)
    if _read_replay_config(output_path) != expected_config:
        os.remove(complete_marker)
        if log_warn is not None:
            log_warn(
                f"============== Ignoring stale completion marker for {label}: replay_config.yaml changed =============="
            )
        return False

    if log_warn is not None:
        log_warn(f"============== Skipping {label} (already complete) ==============")
        log_warn(f"Run without --skip_completed or delete {complete_marker} to rerun this replay.")
    return True


def format_dropped_steps(dropped_steps: list[int], max_listed: int = 20) -> str:
    """Format dropped step indices for a log message, truncating long lists.

    Args:
        dropped_steps: The step indices that were dropped.
        max_listed: The maximum number of indices to list before truncating. Defaults to 20.

    Returns:
        A comma-separated list of step indices.
    """
    listed = ", ".join(str(step) for step in dropped_steps[:max_listed])
    if len(dropped_steps) > max_listed:
        listed += f", ... (+{len(dropped_steps) - max_listed} more)"
    return listed


def is_in_place_replay(output_path: str, recording_path: str) -> bool:
    """Check whether a replay writes back over the recording it reads.

    Args:
        output_path: Directory the replay writes to.
        recording_path: Directory the replay reads its recorded state from.

    Returns:
        True if both paths resolve to the same directory.
    """
    return os.path.realpath(output_path) == os.path.realpath(recording_path)


def discard_step_common(output_path: str, step: int) -> None:
    """Remove the common state entry of a step whose images are not being written.

    Args:
        output_path: Replay directory whose common state entry should be removed.
        step: Index of the step to remove.
    """
    # MobilityGenReader derives its step list from state/common and builds the image filenames from
    # it, so a common entry with no images is a step it indexes but cannot read. Out of place
    # clear_replay_outputs() has already emptied state/common and this is a no-op; in place
    # state/common is the source, so the entry for a dropped step is on disk and has to go.
    common_path = os.path.join(output_path, "state", "common", f"{step:08d}.npz")
    if os.path.isfile(common_path):
        os.remove(common_path)


def clear_replay_outputs(output_path: str, recording_path: str) -> None:
    """Remove the files a replay regenerates, leaving any source data in place.

    Lets the output directory equal the input one: the recorded poses, scene, and config survive
    while the rendered sensor outputs and manifest are refreshed.

    Args:
        output_path: Replay directory whose generated sensor state and manifest should be removed.
        recording_path: Recording the replay reads, to tell an in-place replay from a copy.
    """
    targets = [os.path.join(output_path, "state", name) for name in _RENDERED_STATE_DIRS]
    if not is_in_place_replay(output_path, recording_path):
        # Out of place, state/common is regenerated step by step from the source, so an entry an
        # earlier run left behind would survive with its images deleted — a step MobilityGenReader
        # indexes but cannot read. In place it is the source and has to survive.
        targets.append(os.path.join(output_path, "state", "common"))
    targets.append(os.path.join(output_path, REPLAY_CONFIG_NAME))
    targets.append(os.path.join(output_path, COMPLETE_MARKER_NAME))
    for target in targets:
        if os.path.isdir(target):
            shutil.rmtree(target)
        elif os.path.isfile(target):
            os.remove(target)
