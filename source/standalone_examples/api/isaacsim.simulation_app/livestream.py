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

"""Demonstrate headless simulation with livestream server."""

import argparse

from isaacsim import SimulationApp

parser = argparse.ArgumentParser()
parser.add_argument("--test", default=False, action="store_true", help="Run in test mode")
args, _ = parser.parse_known_args()

# This sample enables a livestream server to connect to when running headless
CONFIG = {
    "width": 1280,
    "height": 720,
    "window_width": 1920,
    "window_height": 1080,
    "headless": True,
    "hide_ui": False,  # Show the GUI
    "renderer": "RealTimePathTracing",
    "display_options": 3286,  # Set display options to show default grid
}


# Start the omniverse application
kit = SimulationApp(launch_config=CONFIG)

from isaacsim.core.experimental.utils.app import enable_extension

# Default Livestream settings
kit.set_setting("/app/window/drawMouse", True)

# Enable Livestream extension
enable_extension("omni.kit.livestream.app")

# Run until closed
frame_count = 0
while kit._app.is_running() and not kit.is_exiting():
    # Run in realtime mode, we don't specify the step size
    kit.update()
    frame_count += 1
    if args.test and frame_count >= 10:
        break

kit.close()
