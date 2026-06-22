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

"""Manifest schema + IO for the render step's output.

Records, per rendered frame: the rendered image path, the camera, the frame timestamp, and
the camera world `position`. `rendered` is stored relative to the manifest's own directory.

On-disk shape::

    {
      "stage": "<abs path or omniverse:// URL of the rendered NuRec USD>",
      "cameras": ["front_stereo_camera_left"],
      "pairs": [
        {
          "camera": "front_stereo_camera_left",
          "rendered": "front_stereo_camera_left/1760584964060200000.png",
          "ts_ns": 1760584964060200000,
          "position": [1.2, 0.4, 0.9]
        }
      ]
    }
"""

from __future__ import annotations

import json
import os

MANIFEST_NAME = "manifest.json"


def write_manifest(out_dir: str, stage: str, cameras: list[str], pairs: list[dict]) -> str:
    """Write a manifest under `out_dir`, storing `rendered` relative to it.

    Args:
        out_dir: Directory the manifest is written into; `rendered` paths are stored
            relative to it.
        stage: Path/URL of the rendered NuRec USD, recorded for reference.
        cameras: The camera logical names that were rendered.
        pairs: Per-frame records with absolute `rendered`, `ts_ns`, and `position`.

    Returns:
        The absolute path of the written manifest file.
    """
    out_dir = os.path.abspath(os.path.expanduser(out_dir))
    out_pairs = [
        {
            "camera": p["camera"],
            "rendered": os.path.relpath(p["rendered"], out_dir).replace(os.sep, "/"),
            "ts_ns": p["ts_ns"],
            "position": p["position"],
        }
        for p in pairs
    ]

    manifest = {"stage": stage, "cameras": list(cameras), "pairs": out_pairs}
    os.makedirs(out_dir, exist_ok=True)
    path = os.path.join(out_dir, MANIFEST_NAME)
    with open(path, "w") as f:
        json.dump(manifest, f, indent=2)
    return path


def load_manifest(manifest_path: str) -> tuple[dict, list[dict]]:
    """Load a manifest and resolve `rendered` back to absolute.

    Args:
        manifest_path: Path to the manifest.json to load.

    Returns:
        Tuple of (header, pairs). `header` carries `stage` and `cameras`; each entry in
        `pairs` has absolute `rendered`, `ts_ns`, and `position`.
    """
    manifest_path = os.path.abspath(os.path.expanduser(manifest_path))
    manifest_dir = os.path.dirname(manifest_path)
    with open(manifest_path) as f:
        manifest = json.load(f)

    header = {"stage": manifest.get("stage"), "cameras": manifest.get("cameras", [])}
    pairs = [
        {
            "camera": p.get("camera", ""),
            "rendered": os.path.normpath(os.path.join(manifest_dir, p["rendered"])),
            "ts_ns": p.get("ts_ns"),
            "position": p.get("position"),
        }
        for p in manifest.get("pairs", [])
    ]
    return header, pairs
