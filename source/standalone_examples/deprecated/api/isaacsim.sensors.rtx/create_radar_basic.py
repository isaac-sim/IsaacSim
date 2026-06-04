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

"""Basic RTX Radar creation and visualization example.

This example demonstrates how to:
- Create an RTX Radar sensor using the command API
- Create a render product for the sensor
- Attach a debug draw writer to visualize radar detections
- Customize debug draw point size and color
- Run simulation and observe the radar scanning the environment

Note: RTX Radar requires Motion BVH to be enabled. This is done by passing
"enable_motion_bvh": True to the SimulationApp configuration.

This is the recommended starting point for learning RTX Radar in Isaac Sim.
"""

import argparse
import os
import sys

from isaacsim import SimulationApp

parser = argparse.ArgumentParser(description="Basic RTX Radar example with debug draw visualization.")
parser.add_argument("--test", default=False, action="store_true", help="Run in test mode.")
args, _ = parser.parse_known_args()

# Note: headless=False is required for debug draw visualization
# Note: enable_motion_bvh=True is REQUIRED for RTX Radar to function
simulation_app = SimulationApp({"headless": False, "enable_motion_bvh": True})

output_dir = os.path.join(os.getcwd(), "_example_output_isaacsim.sensors.rtx", "create_radar_basic")
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

# Load a warehouse environment for the radar to scan
open_stage(usd_path=assets_root_path + "/Isaac/Environments/Simple_Warehouse/full_warehouse.usd")
while is_stage_loading():
    simulation_app.update()

# =============================================================================
# CREATE RTX RADAR USING COMMAND API
# =============================================================================
# The IsaacSensorCreateRtxRadar command creates an OmniRadar prim.
# Unlike lidar, radar detects objects based on their radar cross-section and
# provides velocity information (Doppler).
#
# IMPORTANT: Radar will not be created if Motion BVH is not enabled.
# Ensure "enable_motion_bvh": True is set in SimulationApp config.

_, radar_prim = omni.kit.commands.execute(
    "IsaacSensorCreateRtxRadar",
    path="/Radar",
    parent=None,
    translation=Gf.Vec3d(0, 0, 1.0),  # Position 1 meter above ground
    # Rotate 90° about Z-axis so radar faces the warehouse shelves
    orientation=Gf.Quatd(0.70711, 0.0, 0.0, 0.70711),  # (w, x, y, z)
)

carb.log_info(f"Created RTX Radar at {radar_prim.GetPath()}")

# =============================================================================
# CREATE RENDER PRODUCT
# =============================================================================
# A render product connects the sensor to the rendering pipeline, enabling
# data extraction through annotators and writers.

render_product = rep.create.render_product(radar_prim.GetPath(), resolution=(1, 1), name="RadarRenderProduct")

# =============================================================================
# ATTACH DEBUG DRAW WRITER FOR VISUALIZATION
# =============================================================================
# The RtxRadarDebugDrawPointCloud writer visualizes radar detections in the
# viewport. Detections are shown as colored points at their 3D positions.
#
# You can customize the appearance of the debug draw points:
#   - size: Point size in world units (larger = more visible for sparse radar data)
#   - color: RGBA color as [r, g, b, a] with values 0.0-1.0
#
# Radar typically returns fewer points than lidar, so larger point sizes
# help make detections visible.

writer = rep.writers.get("RtxRadarDebugDrawPointCloud")

# Initialize with custom point size and color
# Using larger points and a distinct orange/red color for radar
writer.initialize(
    size=0.2,  # Larger point size for visibility (radar has fewer points)
    color=[1.0, 0.3, 0.1, 1.0],  # Orange-red with full opacity
)

writer.attach([render_product.path])

carb.log_info("Attached debug draw writer with custom orange points (size=0.2)")

if args.test:
    stage = omni.usd.get_context().get_stage()
    stage.Export(os.path.join(output_dir, "stage.usda"))

# =============================================================================
# RUN SIMULATION
# =============================================================================
timeline = omni.timeline.get_timeline_interface()
timeline.play()

carb.log_info("Starting simulation - observe the radar detections in the viewport")

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
