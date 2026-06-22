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

"""RTX Lidar creation with vendor configurations, variants, and attribute modification.

This example demonstrates how to:
- Create RTX Lidar sensors using vendor-specific configurations (Ouster, SICK, HESAI, etc.)
- Select sensor variants (e.g., different channel counts, scan rates)
- Pass custom attributes when creating a sensor
- Modify sensor attributes on an existing prim

Supported configurations and variants can be found in:
    isaacsim.sensors.rtx.SUPPORTED_LIDAR_CONFIGS

Common lidar attributes (omni:sensor:Core:*):
    - auxOutputType: "NONE", "BASIC", "EXTRA", "FULL" - controls GMO data richness
    - outputFrameOfReference: "WORLD" or "SENSOR" - coordinate frame for output data
    - nearRangeM, farRangeM: Min/max range in meters
    - scanRateBaseHz: Scan rotation rate in Hz
    - numberOfChannels: Number of vertical channels
    - maxReturns: Maximum returns per beam
"""

import argparse
import os
import sys

from isaacsim import SimulationApp

parser = argparse.ArgumentParser(description="RTX Lidar with vendor configs and variants.")
parser.add_argument("--test", default=False, action="store_true", help="Run in test mode.")
args, _ = parser.parse_known_args()

simulation_app = SimulationApp({"headless": False})

output_dir = os.path.join(os.getcwd(), "_example_output_isaacsim.sensors.rtx", "create_lidar_with_config_and_variants")
os.makedirs(output_dir, exist_ok=True)

import carb
import omni
import omni.kit.commands
import omni.replicator.core as rep
from isaacsim.core.utils.stage import is_stage_loading, open_stage
from isaacsim.sensors.rtx import SUPPORTED_LIDAR_CONFIGS
from isaacsim.storage.native import get_assets_root_path
from pxr import Gf

# Locate Isaac Sim assets folder
assets_root_path = get_assets_root_path()
if assets_root_path is None:
    carb.log_error("Could not find Isaac Sim assets folder")
    simulation_app.close()
    sys.exit()

# Load environment
open_stage(usd_path=assets_root_path + "/Isaac/Environments/Simple_Warehouse/full_warehouse.usd")
while is_stage_loading():
    simulation_app.update()

# =============================================================================
# LIST AVAILABLE CONFIGURATIONS AND VARIANTS
# =============================================================================
# SUPPORTED_LIDAR_CONFIGS is a dictionary mapping asset paths to their available variants.
# Configs with empty sets have no variants (single configuration).


def _format_variant(variant: str | dict[str, str]) -> str:
    # Variants are either a flat string (single "sensor" variant set) or a
    # dict mapping variant-set names to selections (e.g. SICK family USDs).
    if isinstance(variant, dict):
        return ", ".join(f"{k}={v}" for k, v in sorted(variant.items()))
    return str(variant)


print("\n=== Available Lidar Configurations ===")
for config_path, variants in SUPPORTED_LIDAR_CONFIGS.items():
    # Extract config name from path
    config_name = config_path.split("/")[-1].replace(".usd", "").replace(".usda", "")
    if variants:
        print(f"  {config_name}: {len(variants)} variants")
        for variant in sorted(variants, key=_format_variant):
            print(f"    - {_format_variant(variant)}")
    else:
        print(f"  {config_name}: (no variants)")

# =============================================================================
# EXAMPLE 1: NVIDIA EXAMPLE LIDAR WITH CUSTOM ATTRIBUTES
# =============================================================================
# Pass custom attributes as keyword arguments to override defaults.

carb.log_info("\n--- Creating Example_Rotary lidar with custom attributes ---")

custom_attrs = {
    # Control how much auxiliary data is included in GenericModelOutput
    # Options: "NONE", "BASIC", "EXTRA", "FULL"
    "omni:sensor:Core:auxOutputType": "FULL",
    # Output point cloud in world coordinates (vs sensor-local)
    "omni:sensor:Core:outputFrameOfReference": "WORLD",
}

_, lidar1 = omni.kit.commands.execute(
    "IsaacSensorCreateRtxLidar",
    path="/World/Lidar_Example",
    config="Example_Rotary",
    translation=Gf.Vec3d(-5, 0, 1.0),
    **custom_attrs,
)

carb.log_info(f"Created {lidar1.GetPath()}")

# =============================================================================
# EXAMPLE 2: OUSTER LIDAR WITH VARIANT SELECTION
# =============================================================================
# Ouster lidars (OS0, OS1, OS2) support multiple variants for different
# channel counts, scan rates, and resolutions.

carb.log_info("\n--- Creating Ouster OS1 lidar with variant ---")

_, lidar2 = omni.kit.commands.execute(
    "IsaacSensorCreateRtxLidar",
    path="/World/Lidar_Ouster",
    config="OS1",  # Base Ouster OS1 model
    variant="OS1_REV7_128ch10hz1024res",  # 128 channels, 10Hz, 1024 resolution
    translation=Gf.Vec3d(0, 0, 1.0),
)

carb.log_info(f"Created {lidar2.GetPath()}")
carb.log_info(f"  numberOfChannels: {lidar2.GetAttribute('omni:sensor:Core:numberOfChannels').Get()}")
carb.log_info(f"  scanRateBaseHz: {lidar2.GetAttribute('omni:sensor:Core:scanRateBaseHz').Get()}")

# =============================================================================
# EXAMPLE 3: SICK LIDAR WITH PROFILE VARIANT
# =============================================================================
# SICK lidars use multiple variant sets ("Product", "Profile") rather than the
# default "sensor" set name, so the variant must be passed as a dict mapping
# each variant set to its selection. Pairs are applied in dict insertion order
# (outer variant sets first).

carb.log_info("\n--- Creating SICK picoScan100 lidar with profile variant ---")

_, lidar3 = omni.kit.commands.execute(
    "IsaacSensorCreateRtxLidar",
    path="/World/Lidar_SICK",
    config="SICK_picoScan100",
    variant={"Product": "picoScan150Pro", "Profile": "Profile01_15Hz_0p5deg"},
    translation=Gf.Vec3d(5, 0, 1.0),
)

carb.log_info(f"Created {lidar3.GetPath()}")

# =============================================================================
# EXAMPLE 4: MODIFY ATTRIBUTES ON EXISTING PRIM
# =============================================================================
# After creating a lidar, you can modify its attributes directly via the prim API.

carb.log_info("\n--- Modifying attributes on existing lidar prim ---")

# Read current values
near_range = lidar1.GetAttribute("omni:sensor:Core:nearRangeM").Get()
far_range = lidar1.GetAttribute("omni:sensor:Core:farRangeM").Get()
carb.log_info(f"Original range: {near_range}m - {far_range}m")

# Modify the range (if the attribute exists and is not locked by the config)
if lidar1.HasAttribute("omni:sensor:Core:farRangeM"):
    # Note: Some attributes may be read-only depending on the config
    try:
        lidar1.GetAttribute("omni:sensor:Core:farRangeM").Set(200.0)
        new_far_range = lidar1.GetAttribute("omni:sensor:Core:farRangeM").Get()
        carb.log_info(f"Modified far range to: {new_far_range}m")
    except Exception as e:
        carb.log_warn(f"Could not modify farRangeM: {e}")

# =============================================================================
# ATTACH DEBUG DRAW FOR VISUALIZATION
# =============================================================================
# Attach debug draw writers to both lidars with different colors/sizes

# Lidar 1: Example_Rotary - use cyan color with larger points
render_product1 = rep.create.render_product(lidar1.GetPath(), resolution=(1, 1))
writer1 = rep.writers.get("RtxLidarDebugDrawPointCloudBuffer")
writer1.initialize(
    size=0.08,  # Larger points
    color=[0.0, 1.0, 1.0, 1.0],  # Cyan
)
writer1.attach([render_product1.path])

# Lidar 2: Ouster - use magenta color with smaller points
render_product2 = rep.create.render_product(lidar2.GetPath(), resolution=(1, 1))
writer2 = rep.writers.get("RtxLidarDebugDrawPointCloudBuffer")
writer2.initialize(
    size=0.03,  # Smaller points
    color=[1.0, 0.0, 1.0, 1.0],  # Magenta
)
writer2.attach([render_product2.path])

# Lidar 3: SICK - use yellow color
render_product3 = rep.create.render_product(lidar3.GetPath(), resolution=(1, 1))
writer3 = rep.writers.get("RtxLidarDebugDrawPointCloudBuffer")
writer3.initialize(
    size=0.05,  # Medium points
    color=[1.0, 1.0, 0.0, 1.0],  # Yellow
)
writer3.attach([render_product3.path])

carb.log_info("\nAttached debug draw writers:")
carb.log_info("  Lidar 1 (Example_Rotary): Cyan, large points")
carb.log_info("  Lidar 2 (Ouster): Magenta, small points")
carb.log_info("  Lidar 3 (SICK): Yellow, medium points")

if args.test:
    stage = omni.usd.get_context().get_stage()
    stage.Export(os.path.join(output_dir, "stage.usda"))

# =============================================================================
# RUN SIMULATION
# =============================================================================
timeline = omni.timeline.get_timeline_interface()
timeline.play()

carb.log_info("\nStarting simulation - observe the different lidar point clouds in the viewport")

frame_count = 0
while simulation_app.is_running():
    simulation_app.update()
    frame_count += 1

    if args.test and frame_count >= 10:
        break

# Cleanup
timeline.stop()
simulation_app.close()
