# SPDX-FileCopyrightText: Copyright (c) 2020-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

"""Demonstrate loading a USD stage in simulation."""

import argparse
import sys

from isaacsim import SimulationApp

# This sample loads a usd stage and starts simulation
CONFIG = {"width": 1280, "height": 720, "sync_loads": True, "headless": False, "renderer": "RealTimePathTracing"}


# Set up command line arguments
parser = argparse.ArgumentParser("Usd Load sample")
parser.add_argument(
    "--usd_path",
    type=str,
    help="Path to usd file. Either a full URL (http://, https://, omniverse://) used directly, "
    "or a path relative to your default assets folder",
    required=True,
)
parser.add_argument("--headless", default=False, action="store_true", help="Run stage headless")
parser.add_argument("--test", default=False, action="store_true", help="Run in test mode")

args, unknown = parser.parse_known_args()
# Start the omniverse application
CONFIG["headless"] = args.headless
kit = SimulationApp(launch_config=CONFIG)

import carb
import omni

# Locate Isaac Sim assets folder to load sample
from isaacsim.storage.native import get_assets_root_path, is_file

# If a full URL is provided, use it directly. Otherwise treat it as a path
# relative to the default Isaac Sim assets folder.
if args.usd_path.startswith(("http://", "https://", "omniverse://")):
    usd_path = args.usd_path
else:
    assets_root_path = get_assets_root_path()
    if assets_root_path is None:
        carb.log_error("Could not find Isaac Sim assets folder")
        kit.close()
        sys.exit()
    usd_path = assets_root_path + args.usd_path

# make sure the file exists before we try to open it
try:
    result = is_file(usd_path)
except BaseException:
    result = False

if result:
    omni.usd.get_context().open_stage(usd_path)
else:
    carb.log_error(f"the usd path {usd_path} could not be opened, please make sure that it is a valid usd file")
    kit.close()
    sys.exit()
# Wait two frames so that stage starts loading
kit.update()
kit.update()

print("Loading stage...")
from isaacsim.core.experimental.utils.stage import is_stage_loading

while is_stage_loading():
    kit.update()
print("Loading Complete")
omni.timeline.get_timeline_interface().play()
# Run in test mode, exit after a fixed number of steps
if args.test is True:
    for i in range(10):
        # Run in realtime mode, we don't specify the step size
        kit.update()
else:
    while kit.is_running():
        # Run in realtime mode, we don't specify the step size
        kit.update()
omni.timeline.get_timeline_interface().stop()
kit.close()
