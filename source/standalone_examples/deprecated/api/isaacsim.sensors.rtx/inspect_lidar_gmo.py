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

"""Inspect RTX Lidar GenericModelOutput (GMO) data at different auxiliary levels.

This example demonstrates how to:
- Create an RTX Lidar using the LidarRtx class with custom attributes
- Attach the GenericModelOutput annotator to access raw sensor data
- Use the get_gmo_data() utility to parse the GMO buffer
- Explore GMO fields available at different auxiliary data levels

GenericModelOutput (GMO) is the raw output format from RTX sensors. The amount
of data included depends on the auxOutputType attribute:
    - NONE: Minimal data (x, y, z coordinates only)
    - BASIC: Adds emitterId, channelId, tickId, echoId, tickStates, scanComplete
    - EXTRA: Adds objId (object IDs), matId (material IDs)
    - FULL: Adds hitNormals, velocities

Usage:
    python inspect_lidar_gmo.py --aux-data-level FULL
"""

import argparse
import os
import sys

from isaacsim import SimulationApp

parser = argparse.ArgumentParser(description="Inspect RTX Lidar GMO data at different auxiliary levels.")
parser.add_argument("--test", default=False, action="store_true", help="Run in test mode.")
parser.add_argument(
    "--aux-data-level",
    default="FULL",
    choices=["NONE", "BASIC", "EXTRA", "FULL"],
    help="Lidar auxiliary data level (controls how much data is in GMO output).",
)
args, _ = parser.parse_known_args()

# headless=True since we're just inspecting data, not visualizing
simulation_app = SimulationApp({"headless": True, "enable_motion_bvh": True})

output_dir = os.path.join(os.getcwd(), "_example_output_isaacsim.sensors.rtx", "inspect_lidar_gmo")
os.makedirs(output_dir, exist_ok=True)

import carb
import omni
from isaacsim.core.utils.stage import is_stage_loading, open_stage
from isaacsim.sensors.rtx import LidarRtx, get_gmo_data
from isaacsim.storage.native import get_assets_root_path

# =============================================================================
# AUXILIARY DATA LEVEL DEFINITIONS
# =============================================================================
# These levels control how much auxiliary data is included in GMO output.
# Higher levels include all data from lower levels plus additional fields.

LIDAR_AUX_DATA_LEVELS = {"NONE": 0, "BASIC": 1, "EXTRA": 2, "FULL": 3}
aux_data_level = args.aux_data_level

# =============================================================================
# LOAD ENVIRONMENT
# =============================================================================
assets_root_path = get_assets_root_path()
if assets_root_path is None:
    carb.log_error("Could not find Isaac Sim assets folder")
    simulation_app.close()
    sys.exit()

open_stage(usd_path=assets_root_path + "/Isaac/Environments/Simple_Warehouse/warehouse.usd")
while is_stage_loading():
    simulation_app.update()

# =============================================================================
# CREATE LIDAR WITH CUSTOM AUXILIARY OUTPUT LEVEL
# =============================================================================
# The auxOutputType attribute controls how much data is included in GMO.
# Pass it as a keyword argument when creating the LidarRtx.

custom_attributes = {"omni:sensor:Core:auxOutputType": args.aux_data_level}
lidar = LidarRtx(prim_path="/lidar", name="lidar", **custom_attributes)

carb.log_info(f"Created lidar with auxOutputType={args.aux_data_level}")

# =============================================================================
# ATTACH GENERICMODELOUTPUT ANNOTATOR
# =============================================================================
# The GenericModelOutput annotator provides access to raw sensor output data.

ANNOTATOR_NAME = "GenericModelOutput"
lidar.initialize()
lidar.attach_annotator(ANNOTATOR_NAME)

carb.log_info(f"Attached {ANNOTATOR_NAME} annotator")


# =============================================================================
# GMO DATA INSPECTION FUNCTION
# =============================================================================
def inspect_lidar_gmo(frame: int, gmo_buffer: dict) -> None:
    """Parse and print GMO data fields based on the current auxiliary level.

    Args:
        frame: Current frame number.
        gmo_buffer: Raw GMO buffer from the annotator.
    """
    # Parse the GMO buffer into a structured object
    gmo = get_gmo_data(gmo_buffer)

    print(f"\n{'='*60}")
    print(f"Frame {frame} -- Auxiliary level: {aux_data_level}")
    print(f"{'='*60}")

    # Always available: basic point data
    print(f"numElements (number of points): {gmo.numElements}")

    # BASIC level fields
    if LIDAR_AUX_DATA_LEVELS[aux_data_level] >= LIDAR_AUX_DATA_LEVELS["BASIC"]:
        print(f"\n-- BASIC level fields --")
        print(f"scanComplete: {gmo.scanComplete}")
        print(f"azimuthOffset: {gmo.azimuthOffset}")
        print(f"emitterId: {type(gmo.emitterId).__name__} with {gmo.numElements} elements")
        print(f"channelId: {type(gmo.channelId).__name__} with {gmo.numElements} elements")
        print(f"tickId: {type(gmo.tickId).__name__} with {gmo.numElements} elements")
        print(f"echoId: {type(gmo.echoId).__name__} with {gmo.numElements} elements")
        print(f"tickStates: {type(gmo.tickStates).__name__} with {gmo.numElements} elements")

    # EXTRA level fields
    if LIDAR_AUX_DATA_LEVELS[aux_data_level] >= LIDAR_AUX_DATA_LEVELS["EXTRA"]:
        print(f"\n-- EXTRA level fields --")
        print(f"objId (object IDs): {type(gmo.objId).__name__} with {gmo.numElements} elements")
        print(f"matId (material IDs): {type(gmo.matId).__name__} with {gmo.numElements} elements")

    # FULL level fields
    if LIDAR_AUX_DATA_LEVELS[aux_data_level] >= LIDAR_AUX_DATA_LEVELS["FULL"]:
        print(f"\n-- FULL level fields --")
        print(f"hitNormals: {type(gmo.hitNormals).__name__} with {gmo.numElements} elements")
        print(f"velocities: {type(gmo.velocities).__name__} with {gmo.numElements} elements")

    # Print sample point cloud data (first 5 points)
    print(f"\n-- Sample Point Cloud Data (first 5 points) --")
    num_samples = min(5, gmo.numElements)
    for i in range(num_samples):
        print(f"  Point {i}: x={gmo.x[i]:.3f}, y={gmo.y[i]:.3f}, z={gmo.z[i]:.3f}, timeOffsetNs={gmo.timeOffsetNs[i]}")


# =============================================================================
# RUN SIMULATION AND INSPECT DATA
# =============================================================================
if args.test:
    stage = omni.usd.get_context().get_stage()
    stage.Export(os.path.join(output_dir, "stage.usda"))

timeline = omni.timeline.get_timeline_interface()
timeline.play()

carb.log_info("Starting simulation - inspecting GMO data each frame")

frame_count = 0
while simulation_app.is_running() and (not args.test or frame_count < 5):
    simulation_app.update()

    # Get GMO data from the current frame
    data = lidar.get_current_frame()[ANNOTATOR_NAME]
    if len(data) > 0:
        inspect_lidar_gmo(frame=frame_count, gmo_buffer=data)

    frame_count += 1

# =============================================================================
# CLEANUP
# =============================================================================
timeline.stop()
simulation_app.close()
