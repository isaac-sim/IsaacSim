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

"""Prepare Nsight Systems profiling for the examples runner."""

from __future__ import annotations

import os
import shutil
from pathlib import Path


def prepare_nsys_command(
    command: list[str], environment: dict[str, str], *, scope: str, output: Path, options: tuple[str, ...] = ()
) -> tuple[list[str], Path]:
    """Wrap a workload command and configure its owned child environment.

    Args:
        command: Workload executable and arguments.
        environment: Prepared child environment, updated for measured-phase capture.
        scope: ``benchmark`` for the measured NVTX range, or ``full`` for the process.
        output: Report prefix or path, resolved against the caller's working directory.
        options: Additional ``NAME=VALUE`` options for ``nsys profile``.

    Returns:
        Profiler command and absolute report path. Process ownership stays with the runner.
    """
    executable = shutil.which("nsys", path=environment.get("PATH", os.defpath))
    if executable is None:
        raise FileNotFoundError("nsys is not on PATH. Run through examples.sh or examples.bat, or omit --nsys.")
    output = output.expanduser().resolve()
    report = output if output.suffix == ".nsys-rep" else Path(f"{output}.nsys-rep")
    if report.exists():
        raise FileExistsError(f"Nsight report already exists: {report}. Choose another --nsys-output.")
    report.parent.mkdir(parents=True, exist_ok=True)
    profile = [str(Path(executable).absolute()), "profile", f"--output={report}"]
    if not any(option.startswith("trace=") for option in options):
        profile.append("--trace=nvtx,cuda")
    if scope == "benchmark":
        profile += [
            "--capture-range=nvtx",
            "--nvtx-capture=MEASURE@isaacsim.performance.benchmarking",
            "--capture-range-end=stop",
        ]
        # The collector uses an ordinary NVTX string, not a registered string.
        environment["NSYS_NVTX_PROFILER_REGISTER_ONLY"] = "0"
    return [*profile, *(f"--{option}" for option in options), *command], report
