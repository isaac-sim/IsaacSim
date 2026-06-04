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

"""Basic RTX Lidar creation and visualization example using the new experimental API.

This example demonstrates how to:
- Create an RTX Lidar sensor using the ``Lidar.create()`` factory method
- Attach the ``RtxSensorDebugDrawPointCloud`` writer for viewport visualization
- Customize debug draw point size and color
- Run simulation and observe the lidar scanning the environment

This is the recommended starting point for learning the new RTX Lidar API
in ``isaacsim.sensors.experimental.rtx``.
"""

import argparse
import os
import sys

from isaacsim import SimulationApp

parser = argparse.ArgumentParser(description="Basic RTX Lidar example with debug draw visualization.")
parser.add_argument(
    "-c",
    "--config",
    type=str,
    default="Example_Rotary",
    help="Lidar config name (e.g., Example_Rotary, Example_Solid_State).",
)
parser.add_argument("--test", default=False, action="store_true", help="Run in test mode.")
args, _ = parser.parse_known_args()

# Note: headless=False is required for debug draw visualization
simulation_app = SimulationApp({"headless": False})

output_dir = os.path.join(os.getcwd(), "_example_output_isaacsim.sensors.experimental.rtx", "create_lidar_basic")
os.makedirs(output_dir, exist_ok=True)

import carb
import numpy as np
import omni
from isaacsim.core.experimental.utils.app import enable_extension
from isaacsim.core.experimental.utils.stage import is_stage_loading, open_stage
from isaacsim.sensors.experimental.rtx import Lidar, LidarSensor
from isaacsim.storage.native import get_assets_root_path

enable_extension("isaacsim.sensors.rtx.nodes")

# =============================================================================
# LOAD ENVIRONMENT
# =============================================================================
# Locate Isaac Sim assets folder and load a warehouse for the lidar to scan.

assets_root_path = get_assets_root_path()
if assets_root_path is None:
    carb.log_error("Could not find Isaac Sim assets folder")
    simulation_app.close()
    sys.exit()

open_stage(usd_path=assets_root_path + "/Isaac/Environments/Simple_Warehouse/full_warehouse.usd")
while is_stage_loading():
    simulation_app.update()

# =============================================================================
# CREATE RTX LIDAR USING THE NEW EXPERIMENTAL API
# =============================================================================
# ``Lidar.create()`` is a factory method that creates an OmniLidar prim from a
# named configuration. Available configs include:
#   - Example_Rotary: A basic rotating lidar (360 deg horizontal scan)
#   - Example_Solid_State: A solid-state lidar (limited FOV, no rotation)
#   - Vendor-specific configs: OS0, OS1, OS2 (Ouster), HESAI_XT32_SD10, etc.
#
# The ``translations`` parameter positions the lidar in local frame coordinates.

lidar = Lidar.create(
    "/World/lidar",
    config=args.config,
    translations=np.array([0, 0, 1.0]),  # Position 1 meter above ground
)

print(f"Created RTX Lidar at {lidar.paths[0]} with config '{args.config}'")

# =============================================================================
# CREATE LIDAR SENSOR FOR RUNTIME
# =============================================================================
# LidarSensor wraps the Lidar authoring object and creates a render product.
# The "draw-point-cloud" writer (registered by isaacsim.sensors.rtx.nodes)
# extracts a Cartesian point cloud and draws it in the viewport via debug draw.
#
# The constructor's ``writers=`` parameter only forwards the writer's registered
# defaults, so to customize ``size`` (point radius in world units) and ``color``
# (RGBA in [0, 1]) we attach the writer explicitly via ``attach_writer``.

sensor = LidarSensor(lidar, annotators=[])
sensor.attach_writer(
    "draw-point-cloud",
    size=0.05,  # Point size in meters (smaller = more detailed)
    color=[0.0, 1.0, 0.5, 1.0],  # Bright green with full opacity
)

print("Created LidarSensor with debug draw visualization (green points, size=0.05)")

# =============================================================================
# RUN SIMULATION
# =============================================================================
if args.test:
    stage = omni.usd.get_context().get_stage()
    stage.Export(os.path.join(output_dir, "stage.usda"))

timeline = omni.timeline.get_timeline_interface()
timeline.play()

print("Starting simulation - observe the lidar point cloud in the viewport")

frame_count = 0
while simulation_app.is_running():
    simulation_app.update()
    frame_count += 1

    # In test mode, exit after a few frames
    if args.test and frame_count >= 10:
        break

# Cleanup
timeline.stop()
simulation_app.close()
