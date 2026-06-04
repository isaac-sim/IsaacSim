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

"""Test multi-robot Carter camera navigation with ROS2 bridge."""

import sys

from isaacsim import SimulationApp

# Default environment: Hospital

ENV_USD_PATH = "/Isaac/Samples/ROS2/Scenario/multiple_robot_carter_hospital_navigation.usd"

CONFIG = {"renderer": "RealTimePathTracing", "headless": False}

# Example ROS2 bridge sample demonstrating the manual loading of Multiple Robot Navigation scenario
simulation_app = SimulationApp(CONFIG)
import carb
import isaacsim.core.experimental.utils.app as app_utils
import isaacsim.core.experimental.utils.stage as stage_utils
import omni
from isaacsim.storage.native import get_assets_root_path

# enable ROS2 bridge extension
app_utils.enable_extension("isaacsim.ros2.bridge")

simulation_app.update()

# Locate assets root folder to load sample
assets_root_path = get_assets_root_path()
if assets_root_path is None:
    carb.log_error("Could not find Isaac Sim assets folder")
    simulation_app.close()
    sys.exit()

usd_path = assets_root_path + ENV_USD_PATH

omni.usd.get_context().open_stage(usd_path, None)

# Wait two frames so that stage starts loading
simulation_app.update()
simulation_app.update()

print("Loading stage...")
while stage_utils.is_stage_loading():
    simulation_app.update()
print("Loading Complete")

stage_utils.set_stage_units(meters_per_unit=1.0)

frame = 0

# need to initialize physics getting any articulation..etc
app_utils.play()

simulation_app.update()

while simulation_app.is_running() and frame < 10:

    simulation_app.update()
    frame = frame + 1

app_utils.stop()
simulation_app.update()
simulation_app.close()
