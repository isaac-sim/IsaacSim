#!/usr/bin/env python3
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
"""generate_sensor_rigs.py — Discover sensors in robot USDs and update robot YAML configs.

Two modes:

  Default (generate):
    For each robot YAML, opens the robot USD, discovers all sensor prims (cameras
    with full intrinsics; lidar/IMU/radar marked # unsupported), and rewrites the
    sensor_rig: block.  All other robot config keys are preserved.  Edit the output
    to set correct names and resolution, then commit.

  --list:
    Prints a table of every sensor prim found — no files are written.  Useful for
    inspecting an unknown robot USD before committing a config.

Sensor support
--------------
  camera   — full params (placement, resolution, focal_length, aperture)  [supported]
  lidar    — sensor_prim_path only, marked # unsupported                   [not yet]
  imu      — sensor_prim_path only, marked # unsupported                   [not yet]
  radar    — sensor_prim_path only, marked # unsupported                   [not yet]

Usage
-----
    BUILD=./_build/linux-x86_64/release
    SCRIPT=source/extensions/isaacsim.replicator.mobility_gen.examples/scripts/generate_sensor_rigs.py
    DATA=source/extensions/isaacsim.replicator.mobility_gen.examples/isaacsim/replicator/mobility_gen/examples/data/robots

    # Regenerate all robot YAMLs in-place:
    "$BUILD/python.sh" "$SCRIPT" "$DATA"/*.yaml

    # List only — no files written:
    "$BUILD/python.sh" "$SCRIPT" --list "$DATA"/*.yaml

    # Write to a separate directory (originals untouched):
    "$BUILD/python.sh" "$SCRIPT" --output-dir /tmp/generated "$DATA"/*.yaml
"""

import argparse
import os
import sys

from isaacsim import SimulationApp

simulation_app = SimulationApp({"headless": True})

import omni.usd
import yaml
from isaacsim.core.utils.stage import is_stage_loading, open_stage
from isaacsim.storage.native import get_assets_root_path
from pxr import Usd

# ---------------------------------------------------------------------------
# Sensor discovery (self-contained — mirrors sensor_rig.py for standalone use)
# ---------------------------------------------------------------------------

_SENSOR_TYPE_MAP = {
    "Camera": "camera",
    "OmniLidar": "lidar",
    "IsaacRtxLidar": "lidar",
    "IsaacImuSensor": "imu",
    "IsaacRtxRadarSensor": "radar",
}
_SUPPORTED_SENSOR_TYPES = {"Camera"}


def _discover_sensors(stage: Usd.Stage, default_resolution: tuple) -> list:
    root_prim = stage.GetDefaultPrim()
    if not root_prim or not root_prim.IsValid():
        root_prim = stage.GetPseudoRoot()
    root_str = str(root_prim.GetPath()).rstrip("/")

    entries = []
    for prim in Usd.PrimRange(root_prim):
        usd_type = prim.GetTypeName()
        if usd_type not in _SENSOR_TYPE_MAP:
            continue
        sensor_type = _SENSOR_TYPE_MAP[usd_type]
        supported = usd_type in _SUPPORTED_SENSOR_TYPES
        subpath = str(prim.GetPath()).removeprefix(root_str + "/")
        entry = {
            "name": subpath.replace("/", "_"),
            "type": sensor_type,
            "sensor_prim_path": subpath,
            "_unsupported": not supported,
        }
        if supported:
            entry["width_px"] = default_resolution[0]
            entry["height_px"] = default_resolution[1]
        entries.append(entry)
    return entries


def _render_sensor_rig_yaml(entries: list) -> str:
    pad = "  "
    lines = [
        "sensor_rig:",
        f"{pad}sensors:",
    ]
    for e in entries:
        unsupported = e.get("_unsupported", False)
        suffix = "  # unsupported" if unsupported else "  # TODO: rename"
        lines.append(f"{pad}  - name: {e['name']}{suffix}")
        lines.append(f"{pad}    type: {e['type']}")
        lines.append(f"{pad}    sensor_prim_path: {e['sensor_prim_path']}")
        if not unsupported:
            lines.append(f"{pad}    width_px: {e['width_px']}  # TODO: verify")
            lines.append(f"{pad}    height_px: {e['height_px']}  # TODO: verify")
        lines.append("")
    return "\n".join(lines)


# ---------------------------------------------------------------------------
# Stage helpers
# ---------------------------------------------------------------------------


def _open_stage_for_yaml(yaml_path: str):
    import time

    with open(yaml_path, "r") as f:
        cfg = yaml.safe_load(f)
    asset_path = cfg.get("asset_path", "")
    if not asset_path:
        print(f"  SKIP: no asset_path in {yaml_path}")
        return None
    usd_url = get_assets_root_path() + asset_path
    print(f"  opening: {usd_url}", flush=True)
    success = open_stage(usd_url)
    if not success:
        print(f"  ERROR: open_stage returned False for {usd_url!r}", flush=True)
        return None
    simulation_app.update()
    simulation_app.update()
    t0 = time.monotonic()
    last_log = 0
    while is_stage_loading():
        simulation_app.update()
        elapsed = time.monotonic() - t0
        if elapsed - last_log >= 10:
            print(f"  loading... {elapsed:.0f}s", flush=True)
            last_log = elapsed
    print(f"  stage ready ({time.monotonic() - t0:.1f}s)", flush=True)
    return omni.usd.get_context().get_stage()


def _print_sensor_table(entries: list) -> None:
    print(f"  {'TYPE':<10} {'SUBPATH':<55} PARAMS", flush=True)
    print(f"  {'-'*10} {'-'*55} ------", flush=True)
    for e in entries:
        unsupported = e.get("_unsupported", False)
        params = "# unsupported" if unsupported else f"{e.get('width_px')}x{e.get('height_px')}"
        print(f"  {e['type']:<10} {e['sensor_prim_path']:<55} {params}", flush=True)


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("yaml_files", nargs="+", help="Robot YAML file(s) to process")
    parser.add_argument("--list", action="store_true", help="Print discovered sensors only; do not write files")
    parser.add_argument("--output-dir", default=None, help="Write generated YAMLs here instead of in-place")
    parser.add_argument("--width", type=int, default=960, help="Default camera width in pixels (default: 960)")
    parser.add_argument("--height", type=int, default=600, help="Default camera height in pixels (default: 600)")
    args = parser.parse_args()

    results = []
    for yaml_path in args.yaml_files:
        print(f"\n[{os.path.basename(yaml_path)}]", flush=True)
        stage = _open_stage_for_yaml(yaml_path)
        if stage is None:
            results.append((yaml_path, "SKIPPED (stage not available)"))
            continue

        entries = _discover_sensors(stage, default_resolution=(args.width, args.height))
        n_cam = sum(1 for e in entries if e["type"] == "camera")
        n_other = len(entries) - n_cam
        print(f"  {len(entries)} sensor(s): {n_cam} camera(s), {n_other} unsupported", flush=True)
        _print_sensor_table(entries)

        if args.list:
            results.append((yaml_path, f"listed ({len(entries)} sensors)"))
            continue

        if args.output_dir:
            os.makedirs(args.output_dir, exist_ok=True)
            output_path = os.path.join(args.output_dir, os.path.basename(yaml_path))
        else:
            output_path = yaml_path

        try:
            with open(yaml_path, "r") as f:
                robot_cfg = yaml.safe_load(f)
            robot_cfg.pop("sensor_rig", None)
            with open(output_path, "w") as f:
                yaml.dump(robot_cfg, f, default_flow_style=False, sort_keys=False, allow_unicode=True)
                if entries:
                    f.write("\n")
                    f.write(_render_sensor_rig_yaml(entries))
            results.append((yaml_path, f"OK -> {output_path}"))
        except Exception as e:
            results.append((yaml_path, f"ERROR: {e}"))

    print("\n\n=== Summary ===", flush=True)
    for path, status in results:
        print(f"  {os.path.basename(path):<25} {status}", flush=True)

    simulation_app.close()


if __name__ == "__main__":
    main()
