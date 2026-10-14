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

"""Build the installer wheel and optionally run its source tests."""

from __future__ import annotations

import argparse
import os
import subprocess
import sys
import tempfile
from pathlib import Path


def main() -> int:
    """Build a wheel and optionally run backend, CLI, and frontend tests."""
    package_root = Path(__file__).resolve().parent
    repository_root = package_root.parents[1]
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output-dir", type=Path, default=repository_root / "_build/packages/installer")
    parser.add_argument("--test", action="store_true", help="Run backend, CLI, and frontend tests.")
    options = parser.parse_args()
    output_dir = options.output_dir.resolve()
    # Build through an sdist so stale setuptools build files cannot enter the wheel.
    with tempfile.TemporaryDirectory(prefix="isaacsim-installer-wheel-") as temporary:
        staging = Path(temporary)
        subprocess.run(
            [sys.executable, "-m", "build", "--no-isolation", "--outdir", str(staging), str(package_root)],
            check=True,
        )
        (wheel,) = staging.glob("*.whl")
        if options.test:
            subprocess.run(
                [sys.executable, "-m", "pytest", str(package_root / "tests"), "-q"],
                env={**os.environ, "PYTHONDONTWRITEBYTECODE": "1"},
                check=True,
            )
        output_dir.mkdir(parents=True, exist_ok=True)
        destination = output_dir / wheel.name
        destination.write_bytes(wheel.read_bytes())
        print(f"Wheel: {destination}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
