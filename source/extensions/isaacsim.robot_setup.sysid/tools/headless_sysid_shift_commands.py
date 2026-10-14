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

"""Create a derived CSV trajectory with command samples shifted in time.

This is a diagnostic helper for evaluating controller/reference latency without
modifying the original telemetry source. For production runs prefer the
``telemetry.command_alignment_seconds`` run-spec key, which applies the same
common-mode shift at load for every ingestion backend (no derived CSV needed);
the telemetry quality report suggests the value from cross-correlation.
"""

from __future__ import annotations

import argparse
import asyncio
import csv
import json
import sys
import traceback
from pathlib import Path

from headless_sysid_solve import (
    _enable_extension,
    _ensure_extension_package_visible,
    _load_spec,
    _rerun_with_bundled_python,
)


def _parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--run-spec", required=True, help="Input SysIdRunSpec JSON.")
    parser.add_argument("--output-csv", required=True, help="Derived CSV output path.")
    parser.add_argument(
        "--command-delay",
        type=float,
        required=True,
        help="Seconds to delay commands: delayed_command(t)=command(t-delay).",
    )
    parser.add_argument("--no-enable-extension", action="store_true")
    parser.add_argument("--overwrite", action="store_true", help="Replace existing CSV and column-map outputs.")
    return parser.parse_args()


async def _run(args: argparse.Namespace) -> int:
    if not args.no_enable_extension:
        await _enable_extension()
    _ensure_extension_package_visible()

    import numpy as np
    from isaacsim.robot_setup.sysid.run_controller import SysIdRunController

    spec = _load_spec(Path(args.run_spec))
    controller = SysIdRunController(spec)
    trajectory = controller.load_trajectory()

    times = np.asarray(trajectory.times, dtype=np.float64).reshape(-1)
    positions = np.asarray(trajectory.positions, dtype=np.float64)
    velocities = np.asarray(trajectory.velocities, dtype=np.float64)
    commands = np.asarray(trajectory.commands, dtype=np.float64)
    if commands.shape != positions.shape:
        raise ValueError(f"commands shape {commands.shape} does not match positions {positions.shape}")

    query_times = times - float(args.command_delay)
    shifted_commands = np.empty_like(commands)
    for joint_index in range(commands.shape[1]):
        shifted_commands[:, joint_index] = np.interp(
            query_times,
            times,
            commands[:, joint_index],
            left=commands[0, joint_index],
            right=commands[-1, joint_index],
        )

    output = Path(args.output_csv).expanduser().resolve()
    mapping_output = output.with_name(f"{output.stem}_column_map.json")
    existing = [path for path in (output, mapping_output) if path.exists()]
    if existing and not args.overwrite:
        raise FileExistsError(
            "Refusing to overwrite existing output(s): "
            + ", ".join(str(path) for path in existing)
            + ". Pass --overwrite to replace them."
        )
    output.parent.mkdir(parents=True, exist_ok=True)
    header = (
        ["time"]
        + [f"q{idx + 1}" for idx in range(positions.shape[1])]
        + [f"dq{idx + 1}" for idx in range(velocities.shape[1])]
        + [f"cmd{idx + 1}" for idx in range(shifted_commands.shape[1])]
    )
    with output.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.writer(stream)
        writer.writerow(header)
        for row_index, t_value in enumerate(times):
            writer.writerow(
                [f"{float(t_value):.9f}"]
                + [f"{float(value):.17g}" for value in positions[row_index]]
                + [f"{float(value):.17g}" for value in velocities[row_index]]
                + [f"{float(value):.17g}" for value in shifted_commands[row_index]]
            )

    metadata = getattr(trajectory, "metadata", None)
    extra = getattr(metadata, "extra", None)
    joint_names = [f"joint_{index}" for index in range(positions.shape[1])]
    if isinstance(extra, dict):
        for mapping_key in ("column_mapping", "topic_mapping"):
            source_mapping = extra.get(mapping_key)
            if isinstance(source_mapping, dict) and source_mapping.get("joint_names"):
                joint_names = [str(name) for name in source_mapping["joint_names"][: positions.shape[1]]]
                break
    mapping_output.write_text(
        json.dumps(
            {
                "time_column": "time",
                "position_columns": [f"q{index + 1}" for index in range(positions.shape[1])],
                "velocity_columns": [f"dq{index + 1}" for index in range(velocities.shape[1])],
                "command_columns": [f"cmd{index + 1}" for index in range(shifted_commands.shape[1])],
                "joint_names": joint_names,
            },
            indent=2,
        )
        + "\n",
        encoding="utf-8",
    )

    print(f"[sysid-shift] source={spec.telemetry.source_path}", flush=True)
    print(f"[sysid-shift] output_csv={output}", flush=True)
    print(f"[sysid-shift] column_mapping={mapping_output}", flush=True)
    print(f"[sysid-shift] samples={times.shape[0]} joints={positions.shape[1]}", flush=True)
    print(f"[sysid-shift] command_alignment_seconds={float(args.command_delay):.6f}", flush=True)
    return 0


def main() -> int:  # noqa: D103
    args = _parse_args()
    sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
    try:
        from isaacsim import SimulationApp
    except ImportError:
        code = _rerun_with_bundled_python()
        if code is not None:
            return code
        raise

    simulation_app = SimulationApp({"headless": True})
    exit_code = 1
    try:
        loop = asyncio.get_event_loop()
        task = loop.create_task(_run(args))
        while not task.done():
            simulation_app.update()
        exit_code = int(task.result())
    except Exception as exc:
        print(f"[sysid-shift] ERROR: {exc}", file=sys.stderr, flush=True)
        traceback.print_exc()
        exit_code = 1
    finally:
        # Fast shutdown terminates the process inside `close()`, so the status must be passed here.
        simulation_app.close(exit_code=exit_code)
    return exit_code


if __name__ == "__main__":
    raise SystemExit(main())
