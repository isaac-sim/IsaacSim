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

"""Check cube motion and benchmark sample retention."""

import tempfile
import unittest
from pathlib import Path
from unittest.mock import patch

from isaacsim.performance.benchmarking import benchmark
from main import FallingCube
from main import main as run_benchmark


def main() -> None:
    """Run the workload and reject invalid results."""
    for device in ("cpu", "gpu"):
        workload = benchmark.workflow(
            "test.falling_cube",
            metrics=("simulation",),
            simulated_time="time_step",
            warmup_frames=2,
            measured_frames=10,
            completion="fetched",
        )(FallingCube)
        with tempfile.TemporaryDirectory() as directory:
            result = benchmark.run(
                workload, parameters={"physics_device": device}, output=Path(directory) / "result.json"
            )
        if not result["status"]["valid"]:
            raise AssertionError(result["status"]["errors"])
        if len(result["metrics"]["duration_ms"]["samples"]) != 10:
            raise AssertionError("Unexpected measured sample count")
        if not all(sample["value"] == 0.005 for sample in result["metrics"]["simulated_time_s"]["samples"]):
            raise AssertionError("Unexpected simulated time per frame")
    with tempfile.TemporaryDirectory() as directory:
        arguments = [
            "main.py",
            "--warmup-frames",
            "0",
            "--measured-frames",
            "1",
            "--output",
            f"{directory}/invalid.json",
        ]
        with patch("sys.argv", arguments), patch.object(FallingCube, "run", autospec=True, return_value=None):
            with unittest.TestCase().assertRaisesRegex(RuntimeError, "validate"):
                run_benchmark()
    print("Falling cube benchmark test passed.")


if __name__ == "__main__":
    main()
