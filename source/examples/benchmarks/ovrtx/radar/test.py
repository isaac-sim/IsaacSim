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

"""Check a short run of the radar benchmark through its own command line."""

from __future__ import annotations

import json
import subprocess
import sys
import tempfile
from collections.abc import Mapping, Sequence
from pathlib import Path
from typing import Any

sys.dont_write_bytecode = True


_SENSOR_COUNT = 2


_WARMUP_FRAMES = 1


_RESOLUTION = (640, 480)


_DELTA_TIME = 0.02


def check_benchmark(
    main: Path,
    stage_contains: Sequence[str],
    *,
    frames: int = 8,
    arguments: Sequence[str] = (),
) -> None:
    """Compose and run the benchmark, then validate its result."""
    composed = subprocess.run(
        [sys.executable, str(main), "--print-stage", "--num-sensors", str(_SENSOR_COUNT), *arguments],
        capture_output=True,
        text=True,
        check=False,
    )
    assert composed.returncode == 0, f"Composing the stage exited with {composed.returncode}\n{composed.stderr}"
    for fragment in stage_contains:
        assert fragment in composed.stdout, f"Composed stage does not contain: {fragment}"

    with tempfile.TemporaryDirectory() as directory:
        output = Path(directory) / "result.json"
        subprocess.run(
            [
                sys.executable,
                str(main),
                "--num-sensors",
                str(_SENSOR_COUNT),
                "--num-frames",
                str(frames),
                "--warmup-frames",
                str(_WARMUP_FRAMES),
                "--resolution",
                str(_RESOLUTION[0]),
                str(_RESOLUTION[1]),
                "--time-step",
                str(_DELTA_TIME),
                "--output",
                str(output),
                *arguments,
            ],
            check=True,
        )
        document = json.loads(output.read_text())

    _check_document(document, frames)
    print("Benchmark smoke test passed.")


def _check_document(document: Mapping[str, Any], frames: int) -> None:
    """Check that the result document reports a valid run of the requested configuration."""
    assert document["status"]["valid"], document["status"]["errors"]

    parameters = document["benchmark"]["parameters"]
    assert parameters["sensor_count"] == _SENSOR_COUNT, parameters
    assert (parameters["width"], parameters["height"]) == _RESOLUTION, parameters
    assert parameters["delta_time_s"] == _DELTA_TIME, parameters

    metrics = document["metrics"]
    assert len(metrics["duration_ms"]["samples"]) == frames, "Measured frames do not match what was requested"
    simulated = [sample["value"] for sample in metrics["simulated_time_s"]["samples"]]
    assert simulated == [_DELTA_TIME] * frames, "Recorded simulated time does not follow the delta-time parameter"


if __name__ == "__main__":
    try:
        check_benchmark(
            Path(__file__).with_name("main.py"),
            (
                'def OmniRadar "Radar_001"',
                'def RenderProduct "RadarProduct_001"',
            ),
            # At the smoke-test time step, a scan takes five steps; measure several scans.
            frames=20,
        )
    except KeyboardInterrupt:
        raise SystemExit(130) from None
