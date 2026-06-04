# SPDX-FileCopyrightText: Copyright (c) 2025-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

"""Basic RTX Lidar creation and visualization example.

This example demonstrates how to:
- Create an RTX Lidar sensor using the command API
- Create a render product for the sensor
- Attach a debug draw writer to visualize the point cloud
- Customize debug draw point size and color
- Run simulation and observe the lidar scanning the environment

This is the recommended starting point for learning RTX Lidar in Isaac Sim.
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

output_dir = os.path.join(os.getcwd(), "_example_output_isaacsim.sensors.rtx", "create_lidar_basic")
os.makedirs(output_dir, exist_ok=True)

import carb
import omni
import omni.kit.commands
import omni.replicator.core as rep
from isaacsim.core.utils.stage import is_stage_loading, open_stage
from isaacsim.storage.native import get_assets_root_path
from pxr import Gf

# Locate Isaac Sim assets folder
assets_root_path = get_assets_root_path()
if assets_root_path is None:
    carb.log_error("Could not find Isaac Sim assets folder")
    simulation_app.close()
    sys.exit()

# Load a warehouse environment for the lidar to scan
open_stage(usd_path=assets_root_path + "/Isaac/Environments/Simple_Warehouse/full_warehouse.usd")
while is_stage_loading():
    simulation_app.update()

# =============================================================================
# CREATE RTX LIDAR USING COMMAND API
# =============================================================================
# The IsaacSensorCreateRtxLidar command creates an OmniLidar prim with the
# specified configuration. Available configs include:
#   - Example_Rotary: A basic rotating lidar (360° horizontal scan)
#   - Example_Solid_State: A solid-state lidar (limited FOV, no rotation)
#   - Vendor-specific configs: OS0, OS1, OS2 (Ouster), HESAI_XT32_SD10, etc.

_, lidar_prim = omni.kit.commands.execute(
    "IsaacSensorCreateRtxLidar",
    path="/Lidar",
    parent=None,
    config=args.config,
    translation=Gf.Vec3d(0, 0, 1.0),  # Position 1 meter above ground
    orientation=Gf.Quatd(1.0, 0.0, 0.0, 0.0),  # Identity rotation (w, x, y, z)
)

carb.log_info(f"Created RTX Lidar at {lidar_prim.GetPath()} with config '{args.config}'")

# =============================================================================
# CREATE RENDER PRODUCT
# =============================================================================
# A render product connects the sensor to the rendering pipeline, enabling
# data extraction through annotators and writers.

render_product = rep.create.render_product(lidar_prim.GetPath(), resolution=(1, 1), name="LidarRenderProduct")

# =============================================================================
# ATTACH DEBUG DRAW WRITER FOR VISUALIZATION
# =============================================================================
# The RtxLidarDebugDrawPointCloudBuffer writer visualizes the lidar point cloud
# directly in the viewport. Points are drawn at their 3D positions.
#
# You can customize the appearance of the debug draw points:
#   - size: Point size in world units (default varies by writer)
#   - color: RGBA color as [r, g, b, a] with values 0.0-1.0

writer = rep.writers.get("RtxLidarDebugDrawPointCloudBuffer")

# Initialize with custom point size and color
# Color is [R, G, B, A] where each component is 0.0 to 1.0
writer.initialize(
    size=0.05,  # Point size in meters (smaller = more detailed)
    color=[0.0, 1.0, 0.5, 1.0],  # Bright green with full opacity
)

writer.attach([render_product.path])

carb.log_info("Attached debug draw writer with custom green points (size=0.05)")

if args.test:
    stage = omni.usd.get_context().get_stage()
    stage.Export(os.path.join(output_dir, "stage.usda"))

# =============================================================================
# RUN SIMULATION
# =============================================================================
timeline = omni.timeline.get_timeline_interface()
timeline.play()

carb.log_info("Starting simulation - observe the lidar point cloud in the viewport")

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
