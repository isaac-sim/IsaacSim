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

"""RTX Lidar creation with vendor configurations, variants, and debug draw visualization.

This example demonstrates how to:
- List all supported lidar configurations and their variants using ``SUPPORTED_LIDAR_CONFIGS``
- Create RTX Lidar sensors using vendor-specific configurations (Ouster OS1, SICK picoScan100, etc.)
- Select sensor variants (e.g., different channel counts, scan rates)
- Attach the ``RtxSensorDebugDrawPointCloud`` writer with per-sensor colors
- Modify sensor attributes on an existing prim

Supported configurations and variants can be found in:
    ``isaacsim.sensors.experimental.rtx.SUPPORTED_LIDAR_CONFIGS``

Common lidar attributes (omni:sensor:Core:*):
    - aux_output_level: "NONE", "BASIC", "EXTRA", "FULL" - controls GMO data richness
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

output_dir = os.path.join(
    os.getcwd(), "_example_output_isaacsim.sensors.experimental.rtx", "create_lidar_with_config_and_variants"
)
os.makedirs(output_dir, exist_ok=True)

import carb
import numpy as np
import omni
from isaacsim.core.experimental.utils.app import enable_extension
from isaacsim.core.experimental.utils.stage import is_stage_loading, open_stage
from isaacsim.sensors.experimental.rtx import SUPPORTED_LIDAR_CONFIGS, Lidar, LidarSensor
from isaacsim.storage.native import get_assets_root_path

enable_extension("isaacsim.sensors.rtx.nodes")

# =============================================================================
# LOAD ENVIRONMENT
# =============================================================================
# Locate Isaac Sim assets folder and load a warehouse for the lidars to scan.

assets_root_path = get_assets_root_path()
if assets_root_path is None:
    carb.log_error("Could not find Isaac Sim assets folder")
    simulation_app.close()
    sys.exit()

open_stage(usd_path=assets_root_path + "/Isaac/Environments/Simple_Warehouse/full_warehouse.usd")
while is_stage_loading():
    simulation_app.update()

# =============================================================================
# LIST AVAILABLE CONFIGURATIONS AND VARIANTS
# =============================================================================
# SUPPORTED_LIDAR_CONFIGS is a dictionary mapping asset paths to their available
# variants. Configs with empty sets have no variants (single configuration).


def _format_variant(variant: str | dict[str, str]) -> str:
    # Variants are either a flat string (single "sensor" variant set) or a
    # dict mapping variant-set names to selections (e.g. SICK family USDs).
    if isinstance(variant, dict):
        return ", ".join(f"{k}={v}" for k, v in sorted(variant.items()))
    return str(variant)


print("\n=== Available Lidar Configurations ===")
for config_path, variants in SUPPORTED_LIDAR_CONFIGS.items():
    # Extract config name from path (e.g., "/Isaac/Sensors/Ouster/OS1/OS1.usd" -> "OS1")
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
# Lidar.create() is a factory method that creates an OmniLidar prim from a
# named configuration. Custom attributes can be passed via the ``attributes``
# parameter to override defaults.

print("\n--- Creating Example_Rotary lidar with custom attributes ---")

lidar1 = Lidar.create(
    "/World/Lidar_Example",
    config="Example_Rotary",
    translations=np.array([-5, 0, 1.0]),
    aux_output_level="FULL",
    attributes={
        # Output point cloud in world coordinates (vs sensor-local)
        "omni:sensor:Core:outputFrameOfReference": "WORLD",
    },
)

print(f"Created {lidar1.paths[0]}")

# =============================================================================
# EXAMPLE 2: OUSTER LIDAR WITH VARIANT SELECTION
# =============================================================================
# Ouster lidars (OS0, OS1, OS2) support multiple variants for different
# channel counts, scan rates, and resolutions. Specify the variant name
# via the ``variant`` parameter.

print("\n--- Creating Ouster OS1 lidar with variant ---")

lidar2 = Lidar.create(
    "/World/Lidar_Ouster",
    config="OS1",  # Base Ouster OS1 model
    variant="OS1_REV7_128ch10hz1024res",  # 128 channels, 10Hz, 1024 resolution
    translations=np.array([0, 0, 1.0]),
)

print(f"Created {lidar2.paths[0]}")

# Read back attributes from the prim to verify the variant was applied
prim = lidar2.prims[0]
if prim.HasAttribute("omni:sensor:Core:numberOfChannels"):
    print(f"  numberOfChannels: {prim.GetAttribute('omni:sensor:Core:numberOfChannels').Get()}")
if prim.HasAttribute("omni:sensor:Core:scanRateBaseHz"):
    print(f"  scanRateBaseHz: {prim.GetAttribute('omni:sensor:Core:scanRateBaseHz').Get()}")

# =============================================================================
# EXAMPLE 3: SICK LIDAR WITH PROFILE VARIANT
# =============================================================================
# SICK lidars use multiple variant sets ("Product", "Profile") rather than the
# default "sensor" set name, so the variant must be passed as a dict mapping
# each variant set to its selection. Pairs are applied in dict insertion order
# (outer variant sets first).

print("\n--- Creating SICK picoScan100 lidar with profile variant ---")

lidar3 = Lidar.create(
    "/World/Lidar_SICK",
    config="SICK_picoScan100",
    variant={"Product": "picoScan150Pro", "Profile": "Profile01_15Hz_0p5deg"},
    translations=np.array([5, 0, 1.0]),
)

print(f"Created {lidar3.paths[0]}")

# =============================================================================
# EXAMPLE 4: MODIFY ATTRIBUTES ON AN EXISTING PRIM
# =============================================================================
# After creating a lidar, you can modify its attributes directly via the
# underlying USD prim API.

print("\n--- Modifying attributes on existing lidar prim ---")

prim1 = lidar1.prims[0]
near_range = prim1.GetAttribute("omni:sensor:Core:nearRangeM").Get()
far_range = prim1.GetAttribute("omni:sensor:Core:farRangeM").Get()
print(f"Original range: {near_range}m - {far_range}m")

if prim1.HasAttribute("omni:sensor:Core:farRangeM"):
    try:
        prim1.GetAttribute("omni:sensor:Core:farRangeM").Set(200.0)
        new_far_range = prim1.GetAttribute("omni:sensor:Core:farRangeM").Get()
        print(f"Modified far range to: {new_far_range}m")
    except Exception as e:
        carb.log_warn(f"Could not modify farRangeM: {e}")

# =============================================================================
# CREATE LIDAR SENSORS AND ATTACH DEBUG DRAW WRITERS
# =============================================================================
# LidarSensor wraps each Lidar and creates a render product. The constructor's
# ``writers=`` parameter only forwards the writer's registered defaults, so to
# customize per-sensor appearance we attach the "draw-point-cloud" writer
# explicitly with ``size`` (point radius) and ``color`` (RGBA in [0, 1]).

sensor1 = LidarSensor(lidar1, annotators=[])
sensor1.attach_writer("draw-point-cloud", size=0.10, color=[0.0, 1.0, 1.0, 1.0])  # cyan, large

sensor2 = LidarSensor(lidar2, annotators=[])
sensor2.attach_writer("draw-point-cloud", size=0.03, color=[1.0, 0.0, 1.0, 1.0])  # magenta, small

sensor3 = LidarSensor(lidar3, annotators=[])
sensor3.attach_writer("draw-point-cloud", size=0.05, color=[1.0, 1.0, 0.0, 1.0])  # yellow, medium

print("\nCreated LidarSensors with debug draw writers:")
print("  Lidar 1 (Example_Rotary): Cyan, large points")
print("  Lidar 2 (Ouster OS1): Magenta, small points")
print("  Lidar 3 (SICK picoScan100): Yellow, medium points")

if args.test:
    stage = omni.usd.get_context().get_stage()
    stage.Export(os.path.join(output_dir, "stage.usda"))

# =============================================================================
# RUN SIMULATION
# =============================================================================
timeline = omni.timeline.get_timeline_interface()
timeline.play()

print("\nStarting simulation - observe the different lidar point clouds in the viewport")

frame_count = 0
while simulation_app.is_running():
    simulation_app.update()
    frame_count += 1

    if args.test and frame_count >= 10:
        break

# Cleanup
timeline.stop()
simulation_app.close()
