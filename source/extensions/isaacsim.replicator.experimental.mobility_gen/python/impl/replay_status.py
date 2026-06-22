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

"""Replay completion and settings manifest helpers."""

from __future__ import annotations

import argparse
import os
from collections.abc import Callable
from typing import Any

import yaml

REPLAY_CONFIG_NAME = "replay_config.yaml"
COMPLETE_MARKER_NAME = ".complete"


def replay_config_from_args(source_recording: str, args: argparse.Namespace) -> dict[str, Any]:
    """Build the replay settings manifest for a recording.

    Args:
        source_recording: Path to the source recording directory.
        args: Parsed replay arguments.

    Returns:
        Replay settings manifest dictionary.
    """
    return {
        "source_recording": os.path.abspath(source_recording),
        "self_contained": args.self_contained,
        "render_interval": args.render_interval,
        "render_rt_subframes": args.render_rt_subframes,
        "warmup_frames": args.warmup_frames,
        "max_frames": args.max_frames,
        "rgb_enabled": args.rgb_enabled,
        "segmentation_enabled": args.segmentation_enabled,
        "depth_enabled": args.depth_enabled,
        "instance_id_segmentation_enabled": args.instance_id_segmentation_enabled,
        "normals_enabled": args.normals_enabled,
    }


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


def mark_replay_complete(output_path: str, frames_rendered: int) -> None:
    """Write the replay completion marker.

    Args:
        output_path: Replay output directory.
        frames_rendered: Number of frames rendered during replay.
    """
    os.makedirs(output_path, exist_ok=True)
    with open(os.path.join(output_path, COMPLETE_MARKER_NAME), "w") as f:
        f.write(f"frames_rendered: {frames_rendered}\n")


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
