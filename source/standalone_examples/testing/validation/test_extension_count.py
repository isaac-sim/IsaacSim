# SPDX-FileCopyrightText: Copyright (c) 2023-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

"""Counts enabled Isaac Sim extensions after SimulationApp startup and optionally prints their names for validation and reporting."""

import argparse
import sys

# Initialize simulation app first
from isaacsim import SimulationApp

# Parse command-line arguments
parser = argparse.ArgumentParser(description="Extension Count Validator")
parser.add_argument("--list", action="store_true", help="List all enabled extensions")
parser.add_argument("--no-window", action="store_true", help="Run without a window")
args, _ = parser.parse_known_args()

launch_config = {}
if args.no_window:
    launch_config["headless"] = True

simulation_app = SimulationApp(launch_config=launch_config)

# Update the app to ensure extensions are loaded
simulation_app.update()
simulation_app.update()

# Isaac Sim / Omniverse imports
import omni.kit.app  # noqa: E402


def get_enabled_extensions() -> dict[str, bool]:
    """Retrieve a dictionary of enabled extensions.

    Returns:
        Dict[str, bool]: A dictionary where keys are extension names and values are boolean status (True).
    """
    app = omni.kit.app.get_app()
    ext_manager = app.get_extension_manager()
    ext_summaries = ext_manager.get_extensions()

    enabled_exts: dict[str, bool] = {}

    for ext_summary in ext_summaries:
        ext_name = ext_summary["name"]
        ext_enabled = bool(ext_summary["enabled"])
        if ext_enabled:
            enabled_exts[ext_name] = True

    return enabled_exts


def main() -> None:
    """Main execution function."""
    try:
        enabled_exts = get_enabled_extensions()
        count = len(enabled_exts)

        print(f"Enabled extensions count: {count}")

        if args.list:
            print("\nEnabled Extensions:")
            print("===================")
            for ext_name in sorted(enabled_exts.keys()):
                print(f"- {ext_name}")
            print("===================")

    except Exception as e:
        print(f"[error] Error during extension counting: {e}", file=sys.stderr)
        sys.exit(1)
    finally:
        # Cleanup application
        simulation_app.close()


if __name__ == "__main__":
    main()
