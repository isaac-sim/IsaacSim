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

"""Resolve lidar point object IDs to USD prim paths using StableIdMap.

This example demonstrates how to:
- Attach multiple annotators (StableIdMap + GenericModelOutput) to a lidar
- Extract object IDs from GMO data using LidarRtx.get_object_ids()
- Decode the StableIdMap buffer using LidarRtx.decode_stable_id_mapping()
- Map object IDs to their corresponding USD prim paths

This is useful for:
- Identifying which objects in the scene were hit by lidar beams
- Semantic segmentation of point cloud data
- Object-level filtering and analysis

Requirements:
- StableIdMap output must be enabled via the extra_args setting
- auxOutputType must be EXTRA or FULL to include object IDs in GMO
"""

import argparse
import os
import sys

from isaacsim import SimulationApp

parser = argparse.ArgumentParser(description="Resolve lidar object IDs to USD prim paths.")
parser.add_argument("--test", default=False, action="store_true", help="Run in test mode.")
args, _ = parser.parse_known_args()

# =============================================================================
# SIMULATION APP CONFIGURATION
# =============================================================================
# IMPORTANT: StableIdMap requires the stableIds setting to be enabled.
# This is done via extra_args passed to SimulationApp.

simulation_app = SimulationApp(
    {
        "headless": True,
        "enable_motion_bvh": True,
        "extra_args": ["--/rtx-transient/stableIds/enabled=true"],
    }
)

output_dir = os.path.join(os.getcwd(), "_example_output_isaacsim.sensors.rtx", "resolve_lidar_object_ids")
os.makedirs(output_dir, exist_ok=True)

import carb
import omni.timeline
from isaacsim.core.utils.stage import open_stage
from isaacsim.sensors.rtx import LidarRtx, get_gmo_data
from isaacsim.storage.native import get_assets_root_path

# =============================================================================
# LOAD ENVIRONMENT
# =============================================================================
assets_root_path = get_assets_root_path()
if assets_root_path is None:
    carb.log_error("Could not find Isaac Sim assets folder")
    simulation_app.close()
    sys.exit()

# Use warehouse scene with forklifts for more interesting objects to detect
open_stage(assets_root_path + "/Isaac/Environments/Simple_Warehouse/warehouse_with_forklifts.usd")

carb.log_info("Loaded warehouse scene with forklifts")

# =============================================================================
# CREATE LIDAR WITH FULL AUXILIARY OUTPUT
# =============================================================================
# auxOutputType must be EXTRA or FULL to include objId in GMO data.

lidar_attributes = {
    "omni:sensor:Core:auxOutputType": "FULL",
}

my_lidar = LidarRtx(
    prim_path="/World/lidar",
    name="lidar",
    position=(0, 0, 1.5),  # Position above ground to see objects
    **lidar_attributes,
)

carb.log_info(f"Created lidar at {my_lidar.prim_path}")

# =============================================================================
# ATTACH ANNOTATORS FOR OBJECT ID RESOLUTION
# =============================================================================
# We need TWO annotators:
# 1. StableIdMap: Provides mapping from stable IDs to prim paths
# 2. GenericModelOutput: Provides object IDs (objId) for each lidar point

my_lidar.initialize()
my_lidar.attach_annotator("StableIdMap")
my_lidar.attach_annotator("GenericModelOutput")

carb.log_info("Attached StableIdMap and GenericModelOutput annotators")

# =============================================================================
# RUN SIMULATION TO COLLECT DATA
# =============================================================================
if args.test:
    stage = omni.usd.get_context().get_stage()
    stage.Export(os.path.join(output_dir, "stage.usda"))

timeline = omni.timeline.get_timeline_interface()
timeline.play()

# Wait for the lidar to warm up and produce valid data
carb.log_info("Waiting for lidar to produce data...")
for _ in range(20):
    simulation_app.update()

    gmo = get_gmo_data(my_lidar.get_current_frame()["GenericModelOutput"])

    # =============================================================================
    # EXTRACT AND RESOLVE OBJECT IDS FROM GMO
    # =============================================================================
    # The GMO objId field contains 128-bit stable IDs for each point.
    # LidarRtx.get_object_ids() converts these to Python integers.

    if gmo.numElements > 0:
        object_ids = LidarRtx.get_object_ids(gmo.objId)
        break

# Step one more frame to ensure we have data
simulation_app.update()

# =============================================================================
# DECODE STABLE ID MAPPING
# =============================================================================
# The StableIdMap annotator returns a raw buffer that needs to be decoded.
# LidarRtx.decode_stable_id_mapping() converts it to a dict[int, str]
# mapping stable IDs to prim path strings.

stable_id_map_buffer = my_lidar.get_current_frame()["StableIdMap"]
stable_id_map = LidarRtx.decode_stable_id_mapping(stable_id_map_buffer.tobytes())

carb.log_info(f"Decoded StableIdMap with {len(stable_id_map)} entries")
carb.log_info(f"Found {len(object_ids)} points with {len(set(object_ids))} unique objects")

# =============================================================================
# MAP OBJECT IDS TO PRIM PATHS
# =============================================================================
# Look up each unique object ID in the stable ID map to get the prim path.

print(f"\n{'='*60}")
print("Object ID to Prim Path Mapping")
print(f"{'='*60}")

for obj_id in set(object_ids):
    if obj_id in stable_id_map:
        prim_path = stable_id_map[obj_id]
        # Count how many points hit this object
        point_count = object_ids.count(obj_id)
        print(f"Object ID {obj_id}:")
        print(f"    Prim path: {prim_path}")
        print(f"    Points hitting: {point_count}")
    else:
        carb.log_warn(f"Object ID {obj_id} not found in stable ID map (may be background/sky)")

# =============================================================================
# CLEANUP
# =============================================================================
timeline.stop()
simulation_app.close()
