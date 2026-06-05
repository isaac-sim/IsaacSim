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

"""Inspect RTX Lidar GenericModelOutput (GMO) data at different auxiliary levels.

This example demonstrates how to:
- Create an RTX Lidar using ``Lidar.create()`` with a custom auxiliary output level
- Create a ``LidarSensor`` and attach a custom Writer via ``sensor.attach_writer()``
- Use ``parse_generic_model_output_data()`` to parse the GMO buffer inside a Writer
- Explore GMO fields available at different auxiliary data levels

GenericModelOutput (GMO) is the raw output format from RTX sensors. The amount
of data included depends on the ``auxOutputType`` attribute:
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

# headless=True since we are just inspecting data, not visualizing
simulation_app = SimulationApp({"headless": True, "enable_motion_bvh": True})

output_dir = os.path.join(os.getcwd(), "_example_output_isaacsim.sensors.experimental.rtx", "inspect_lidar_gmo")
os.makedirs(output_dir, exist_ok=True)

import numpy as np
import omni
import omni.replicator.core as rep
from isaacsim.core.experimental.objects import Cube
from isaacsim.sensors.experimental.rtx import Lidar, LidarSensor, parse_generic_model_output_data
from omni.replicator.core import Writer

# =============================================================================
# AUXILIARY DATA LEVEL DEFINITIONS
# =============================================================================
# These levels control how much auxiliary data is included in GMO output.
# Higher levels include all data from lower levels plus additional fields.

LIDAR_AUX_DATA_LEVELS = {"NONE": 0, "BASIC": 1, "EXTRA": 2, "FULL": 3}
aux_data_level = args.aux_data_level

# =============================================================================
# CREATE A SIMPLE SCENE WITH CUBES
# =============================================================================
# Create a few cubes at known positions so the lidar has geometry to detect.

print("Creating simple test scene with cubes")

Cube("/World/cube_front", positions=np.array([5.0, 0.0, 0.0]), scales=np.array([2.0, 2.0, 2.0]))
Cube("/World/cube_left", positions=np.array([0.0, 5.0, 0.0]), scales=np.array([2.0, 2.0, 2.0]))
Cube("/World/cube_right", positions=np.array([0.0, -5.0, 0.0]), scales=np.array([2.0, 2.0, 2.0]))
Cube("/World/cube_above", positions=np.array([0.0, 0.0, 5.0]), scales=np.array([2.0, 2.0, 2.0]))

# =============================================================================
# CREATE LIDAR WITH CUSTOM AUXILIARY OUTPUT LEVEL
# =============================================================================
# The ``aux_output_level`` parameter controls how much data is included in GMO.

lidar = Lidar.create(
    "/World/lidar",
    config="Example_Rotary",
    translations=np.array([0, 0, 1.0]),  # Position 1 meter above ground
    aux_output_level=aux_data_level,
)

print(f"Created lidar at {lidar.paths[0]} with auxOutputType={aux_data_level}")

# =============================================================================
# CREATE LIDAR SENSOR
# =============================================================================
# ``LidarSensor`` wraps a ``Lidar`` object and creates a render product.
# We pass ``annotators=[]`` because the writer brings its own annotator.

sensor = LidarSensor(lidar, annotators=[])

print("Created LidarSensor")


# =============================================================================
# GMO DATA INSPECTION FUNCTION
# =============================================================================
def inspect_lidar_gmo(frame: int, gmo) -> None:
    """Print GMO data fields based on the current auxiliary level.

    Args:
        frame: Current frame number.
        gmo: Parsed GenericModelOutput structure.
    """
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
# CUSTOM WRITER FOR GMO DATA INSPECTION
# =============================================================================
# A custom ``Writer`` receives data via its ``write()`` callback each frame.
# The writer brings its own ``GenericModelOutput`` annotator, so the sensor
# does not need to specify one.


class GmoLidarInspectWriter(Writer):
    """Writer that parses GenericModelOutput and prints lidar GMO fields."""

    def __init__(self):
        self.data_structure = "renderProduct"
        self.annotators = [rep.annotators.get("GenericModelOutput")]
        self._frame_count = 0

    def write(self, data):
        if "renderProducts" not in data:
            return
        for _rp_name, rp_data in data["renderProducts"].items():
            gmo_raw = rp_data.get("GenericModelOutput")
            if isinstance(gmo_raw, dict):
                gmo_raw = gmo_raw.get("data")
            gmo = parse_generic_model_output_data(gmo_raw)
            if gmo.numElements > 0:
                inspect_lidar_gmo(frame=self._frame_count, gmo=gmo)
        self._frame_count += 1


rep.WriterRegistry.register(GmoLidarInspectWriter)
sensor.attach_writer("GmoLidarInspectWriter")

print("Attached GmoLidarInspectWriter to sensor")

# =============================================================================
# RUN SIMULATION AND INSPECT DATA
# =============================================================================
if args.test:
    stage = omni.usd.get_context().get_stage()
    stage.Export(os.path.join(output_dir, "stage.usda"))

timeline = omni.timeline.get_timeline_interface()
timeline.play()

print("Starting simulation - inspecting GMO data each frame")

frame_count = 0
while simulation_app.is_running() and (not args.test or frame_count < 5):
    simulation_app.update()
    frame_count += 1

# =============================================================================
# CLEANUP
# =============================================================================
timeline.stop()
simulation_app.close()
