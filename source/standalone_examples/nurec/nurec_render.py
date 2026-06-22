#!/usr/bin/env python3
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

"""Render a NuRec USD at explicit camera poses or at sensor-rig keyframes.

Writes each rendered frame as `<output>/<camera>/<ts_ns>.png` plus a `manifest.json`. Auto-
detects whether the NuRec USD is PPISP (SPG) or plain NuRec and sets up capture + photometry
accordingly. Quality knobs come from the shipped render config (override with `--config`).

`poses` — render at explicit TUM camera poses (`--poses`, one trajectory file per camera);
each `timestamp tx ty tz qx qy qz qw` line places the camera at that world pose. The
timestamp names the output frame. Works on any NuRec USD.

`keyframes` — render the sensor-rig keyframe matching each requested timestamp
(`--timestamps`, one integer-ns-timestamp file per camera; `--keyframe-tolerance-us` is the
match window). Requires the NuRec USD to carry a sensor rig.

Usage:
    ./python.sh nurec_render.py poses --stage <NuRec USD> --output <dir> \
        --cameras front_stereo_camera_left --poses front_left.tum
    ./python.sh nurec_render.py keyframes --stage <NuRec USD> --output <dir> \
        --cameras front_stereo_camera_left --timestamps front_left_ts.txt
"""

from __future__ import annotations

import argparse
import os
import sys


def _build_parser() -> argparse.ArgumentParser:
    """Build the two-mode CLI: `keyframes` (timestamp-driven) and `poses` (TUM-driven).

    Returns:
        The configured argument parser.
    """
    parser = argparse.ArgumentParser(description="Render a NuRec USD at keyframes or at explicit poses.")
    sub = parser.add_subparsers(dest="mode", required=True, metavar="{keyframes,poses}")

    def _common(p: argparse.ArgumentParser) -> None:
        p.add_argument("--stage", required=True, help="input USD stage (NuRec USD)")
        p.add_argument("--output", required=True, help="output dir for <camera>/<ts_ns>.png + manifest")
        p.add_argument("--config", default=None, help="YAML overriding the shipped render config")
        p.add_argument(
            "--resolution", default="1920x1080", help="WxH for the RenderProduct created on a non-SPG NuRec USD"
        )

    kf = sub.add_parser("keyframes", help="render at sensor-rig keyframes for given timestamps (needs a rig)")
    _common(kf)
    kf.add_argument("--cameras", nargs="+", required=True, help="camera names (paired with --timestamps)")
    kf.add_argument(
        "--timestamps", nargs="+", required=True, help="per-camera ns-timestamp files (paired with --cameras)"
    )
    kf.add_argument(
        "--keyframe-tolerance-us", type=float, default=1.0, help="requested-ts -> rig-keyframe match window (us)"
    )

    po = sub.add_parser("poses", help="render at explicit TUM camera poses")
    _common(po)
    po.add_argument("--cameras", nargs="+", required=True, help="camera names (paired with --poses)")
    po.add_argument("--poses", nargs="+", required=True, help="per-camera TUM trajectory files (paired with --cameras)")
    return parser


def _parse_resolution(text: str) -> tuple[int, int]:
    """Parse a `WxH` resolution string into a (width, height) tuple.

    Args:
        text: Resolution as `<width>x<height>` (e.g. "1920x1080").

    Returns:
        The (width, height) integer tuple.
    """
    w, h = text.lower().split("x")
    return int(w), int(h)


def main() -> int:
    """Render the NuRec USD at keyframes (timestamp mode) or explicit poses (pose mode).

    Returns:
        Process exit code 0 when at least one frame was rendered, else 2.
    """
    args, _ = _build_parser().parse_known_args()

    resolution = _parse_resolution(args.resolution)
    # The stage may be a local path or an omniverse:// URL; hand it straight through
    # (don't abspath — that would mangle a URL). expanduser handles a leading ~.
    stage_path = os.path.expanduser(args.stage)
    output_dir = os.path.abspath(os.path.expanduser(args.output))

    if args.mode == "keyframes" and len(args.cameras) != len(args.timestamps):
        print("ERROR: --cameras and --timestamps must have the same length.", file=sys.stderr)
        return 2
    if args.mode == "poses" and len(args.cameras) != len(args.poses):
        print("ERROR: --cameras and --poses must have the same length.", file=sys.stderr)
        return 2

    # Enable the rendering utilities (which pulls omni.rtx.spg) at launch and render single-GPU.
    extra_args = ["--enable", "isaacsim.replicator.nurec_utils", "--/renderer/multiGpu/enabled=false"]
    print(f"[nurec_render] mode={args.mode} stage={stage_path} extra_args={extra_args}", flush=True)

    from isaacsim import SimulationApp

    simulation_app = SimulationApp(launch_config={"headless": True, "multi_gpu": False, "extra_args": extra_args})

    # The rendering utilities import only once a SimulationApp exists.
    from isaacsim.replicator.nurec_utils.render import render_keyframes, render_poses
    from isaacsim.replicator.nurec_utils.rendering_setup import enable_omni_rtx_spg, load_config
    from isaacsim.replicator.nurec_utils.usd_utils import read_timestamps_file, read_tum

    exit_code = 1
    try:
        simulation_app.update()
        enable_omni_rtx_spg(simulation_app)
        warmup_steps = int(load_config(args.config).get("rendering", {}).get("warmup_steps", 800))
        if args.mode == "keyframes":
            per_camera_ts = {cam: read_timestamps_file(f) for cam, f in zip(args.cameras, args.timestamps)}
            manifest_path = render_keyframes(
                simulation_app,
                stage_path,
                output_dir,
                per_camera_ts,
                warmup_steps=warmup_steps,
                keyframe_tol_us=args.keyframe_tolerance_us,
                config_path=args.config,
                resolution=resolution,
            )
        else:
            per_camera_poses = {cam: read_tum(f) for cam, f in zip(args.cameras, args.poses)}
            manifest_path = render_poses(
                simulation_app,
                stage_path,
                output_dir,
                per_camera_poses,
                warmup_steps=warmup_steps,
                config_path=args.config,
                resolution=resolution,
            )
        exit_code = 0 if manifest_path else 2
    finally:
        simulation_app.close()
    return exit_code


if __name__ == "__main__":
    sys.exit(main())
