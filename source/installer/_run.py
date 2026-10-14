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

"""Run the installer directly from checkout sources without installing it."""

from __future__ import annotations

import importlib.util
import sys
from pathlib import Path


def load_package() -> None:
    """Load this checkout under its installed package name."""
    source = Path(__file__).resolve().parent / "__init__.py"
    spec = importlib.util.spec_from_file_location("isaacsim_installer", source)
    package = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = package
    spec.loader.exec_module(package)


def main() -> int:
    """Run the shared CLI using this checkout's package."""
    load_package()
    from isaacsim_installer.cli.main import main as run_installer

    return run_installer()


if __name__ == "__main__":
    raise SystemExit(main())
