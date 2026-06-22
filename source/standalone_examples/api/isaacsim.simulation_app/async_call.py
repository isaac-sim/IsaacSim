# SPDX-FileCopyrightText: Copyright (c) 2021-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

"""Demonstrate running asynchronous functions using Kit's task engine."""

from __future__ import annotations

# Parse any command-line arguments specific to the standalone application (only known arguments).
import argparse

# 1. --------------------------------------------------------------------


parser = argparse.ArgumentParser()
parser.add_argument("--headless", action="store_true", help="Run in headless mode")
args, _ = parser.parse_known_args()

# Launch the `SimulationApp` (see DEFAULT_LAUNCHER_CONFIG for available configuration):
# https://docs.isaacsim.omniverse.nvidia.com/latest/py/source/extensions/isaacsim.simulation_app/docs/index.html
from isaacsim import SimulationApp

simulation_app = SimulationApp({"headless": args.headless})

# Any Omniverse level imports must occur after the `SimulationApp` class is instantiated (because APIs are provided
# by the extension/runtime plugin system, it must be loaded before they will be available to import).
import isaacsim.core.experimental.utils.stage as stage_utils
import omni.timeline
from isaacsim.storage.native import get_assets_root_path_async
from omni.kit.usd.collect import Collector

# 2. --------------------------------------------------------------------


async def populate_stage() -> None:
    """Populate the stage with a Franka robot asynchronously."""
    print("Populating stage asynchronously:")
    print(" - Creating new stage...")
    await stage_utils.create_new_stage_async(template="sunlight")
    print(" - Getting assets root path...")
    assets_root_path = await get_assets_root_path_async()
    print(" - Adding reference to stage...")
    stage_utils.add_reference_to_stage(
        usd_path=assets_root_path + "/Isaac/Robots/FrankaRobotics/FrankaPanda/franka.usd",
        path="/World/robot",
        variants=[("Gripper", "AlternateFinger"), ("Mesh", "Performance")],
    )
    print(" - Done populating stage")


async def save_stage(collect_dir: str) -> bool:
    """Save the current stage to a directory asynchronously."""
    print("Saving stage asynchronously:")
    print(" - Collecting a USD file with all of its dependencies...")
    usd_path = stage_utils.get_current_stage().GetRootLayer().identifier
    collector = Collector(usd_path=usd_path, collect_dir=collect_dir)
    success, collected_root_usd = await collector.collect()
    print(" - Done saving stage")
    return success, collected_root_usd


# 3. --------------------------------------------------------------------

# Populate stage asynchronously.
simulation_app.run_coroutine(populate_stage())

# Play the simulation and run it for several iterations.
omni.timeline.get_timeline_interface().play()
for _ in range(100):
    simulation_app.update()

# Save the stage asynchronously.
success, collected_root_usd = simulation_app.run_coroutine(save_stage(collect_dir="local_folder"))
if success:
    print(f"Collected USD saved at: {collected_root_usd}")
else:
    print("Failed to save stage")

# Close the `SimulationApp`.
simulation_app.close()
