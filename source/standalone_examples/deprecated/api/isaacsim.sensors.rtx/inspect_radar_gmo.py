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

"""Inspect RTX Radar GenericModelOutput (GMO) data.

This example demonstrates how to:
- Create an RTX Radar sensor using the command API
- Manually create a render product and attach annotators (low-level approach)
- Use the get_gmo_data() utility to parse the radar GMO buffer
- Explore radar-specific GMO fields including velocity (Doppler) data

Note: Unlike the lidar example which uses the LidarRtx class, this example
uses the lower-level command and replicator APIs directly. This is because
there is no RadarRtx wrapper class available.

Radar GMO fields include:
    - x, y, z: Detection coordinates
    - rv_ms: Radial velocity in m/s (Doppler)
    - scalar: Signal strength/RCS
    - sensorID: Sensor identifier
    - scanIdx, cycleCnt: Scan timing information
    - min/max range, velocity, azimuth, elevation bounds

Note: RTX Radar requires Motion BVH to be enabled.
"""

import argparse
import os
import sys

from isaacsim import SimulationApp

parser = argparse.ArgumentParser(description="Inspect RTX Radar GMO data.")
parser.add_argument("--test", default=False, action="store_true", help="Run in test mode.")
args, _ = parser.parse_known_args()

# headless=True since we're just inspecting data, not visualizing
# enable_motion_bvh=True is REQUIRED for radar
simulation_app = SimulationApp({"headless": True, "enable_motion_bvh": True})

output_dir = os.path.join(os.getcwd(), "_example_output_isaacsim.sensors.rtx", "inspect_radar_gmo")
os.makedirs(output_dir, exist_ok=True)

import carb
import omni
import omni.kit.commands
import omni.replicator.core as rep
from isaacsim.core.utils.stage import is_stage_loading, open_stage
from isaacsim.sensors.rtx import get_gmo_data
from isaacsim.storage.native import get_assets_root_path
from pxr import Gf

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
# CREATE RTX RADAR USING COMMAND API
# =============================================================================
# Radar uses the WpmDmat sensor model. Set auxOutputType to BASIC for all data.
# Note: Radar attribute prefix is "omni:sensor:WpmDmat:" (not "Core:" like lidar)

custom_attributes = {"omni:sensor:WpmDmat:auxOutputType": "BASIC"}

_, radar = omni.kit.commands.execute(
    "IsaacSensorCreateRtxRadar",
    path="/radar",
    parent=None,
    translation=Gf.Vec3d(0, 0, 1),  # 1 meter above ground
    orientation=Gf.Quatd(1, 0, 0, 0),  # Identity rotation
    visibility=False,
    **custom_attributes,
)

if radar is None:
    carb.log_error("Failed to create radar - ensure Motion BVH is enabled")
    simulation_app.close()
    sys.exit()

carb.log_info(f"Created RTX Radar at {radar.GetPath()}")

# =============================================================================
# CREATE RENDER PRODUCT AND ATTACH ANNOTATOR (LOW-LEVEL API)
# =============================================================================
# Since there's no RadarRtx wrapper class, we use the replicator API directly.
# This is the same pattern used internally by LidarRtx.

render_product = rep.create.render_product(radar.GetPath(), resolution=(1024, 1024))

# Attach the GenericModelOutput annotator to get raw radar data
annotator = rep.AnnotatorRegistry.get_annotator("GenericModelOutput")
annotator.attach([render_product.path])

carb.log_info("Attached GenericModelOutput annotator via replicator API")

# =============================================================================
# RADAR-SPECIFIC GMO FIELDS
# =============================================================================
# These are the fields available in radar GMO output

RADAR_GMO_FIELDS = [
    ("numElements", "Number of radar detections"),
    ("x", "X coordinate of detection (meters)"),
    ("y", "Y coordinate of detection (meters)"),
    ("z", "Z coordinate of detection (meters)"),
    ("scalar", "Signal strength / radar cross-section"),
    ("sensorID", "Sensor identifier"),
    ("scanIdx", "Scan index"),
    ("cycleCnt", "Cycle count"),
    ("maxRangeM", "Maximum detection range (meters)"),
    ("minVelMps", "Minimum detectable velocity (m/s)"),
    ("maxVelMps", "Maximum detectable velocity (m/s)"),
    ("minAzRad", "Minimum azimuth angle (radians)"),
    ("maxAzRad", "Maximum azimuth angle (radians)"),
    ("minElRad", "Minimum elevation angle (radians)"),
    ("maxElRad", "Maximum elevation angle (radians)"),
    ("rv_ms", "Radial velocity / Doppler (m/s)"),
]


# =============================================================================
# GMO DATA INSPECTION FUNCTION
# =============================================================================
def inspect_radar_gmo(frame: int, gmo_buffer: dict) -> None:
    """Parse and print radar GMO data fields.

    Args:
        frame: Current frame number.
        gmo_buffer: Raw GMO buffer from the annotator.
    """
    # Parse the GMO buffer into a structured object
    gmo = get_gmo_data(gmo_buffer)

    print(f"\n{'='*60}")
    print(f"Frame {frame} -- Radar GMO Data")
    print(f"{'='*60}")

    for field_name, description in RADAR_GMO_FIELDS:
        value = getattr(gmo, field_name, "N/A")
        print(f"{field_name}: {value}")
        print(f"    ({description})")

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

carb.log_info("Starting simulation - inspecting radar GMO data each frame")

frame_count = 0
while simulation_app.is_running() and (not args.test or frame_count < 5):
    simulation_app.update()

    # Get GMO data from the annotator
    data = annotator.get_data()
    if len(data) > 0:
        inspect_radar_gmo(frame=frame_count, gmo_buffer=data)

    frame_count += 1

# =============================================================================
# CLEANUP
# =============================================================================
timeline.stop()
simulation_app.close()
