#!/usr/bin/env python3
# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

"""Run OpenUSD's schema generator with isolated CMake build dependencies."""

from __future__ import annotations

import argparse
import os
import runpy
import sys
from pathlib import Path


def _main() -> None:
    """Prepend generator dependencies and execute usdGenSchema."""
    parser = argparse.ArgumentParser()
    parser.add_argument("--build-deps", type=Path, required=True)
    parser.add_argument("--usd-root", type=Path, required=True)
    parser.add_argument("--expected-usd-version", required=True)
    parser.add_argument("--schema", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    arguments = parser.parse_args()
    usd_python = arguments.usd_root / "lib" / "python"
    generator = usd_python / "pxr" / "Usd" / "usdGenSchema.py"
    for path in (arguments.build_deps, usd_python, generator, arguments.schema):
        if not path.exists():
            raise RuntimeError(f"Required schema generation path is missing: {path}")
    dll_directory_handles = []
    if sys.platform == "win32":
        for directory in (arguments.usd_root / "bin", arguments.usd_root / "lib"):
            if directory.is_dir():
                dll_directory_handles.append(os.add_dll_directory(directory))
    arguments.output.mkdir(parents=True, exist_ok=True)
    sys.path[:0] = [str(arguments.build_deps), str(usd_python)]

    from pxr import Usd

    actual_version = ".".join(str(part) for part in Usd.GetVersion())
    if actual_version != arguments.expected_usd_version:
        raise RuntimeError(
            f"USD schema generator version {actual_version} does not match "
            f"the requested version {arguments.expected_usd_version}"
        )
    sys.argv = [str(generator), str(arguments.schema), str(arguments.output)]
    runpy.run_path(str(generator), run_name="__main__")


if __name__ == "__main__":
    _main()
