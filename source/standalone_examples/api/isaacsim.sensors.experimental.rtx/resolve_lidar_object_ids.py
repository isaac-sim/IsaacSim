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

"""Resolve lidar point object IDs to USD prim paths using StableIdMap.

This example demonstrates how to:
- Create a scene with identifiable objects (cubes at known positions)
- Create a ``LidarSensor`` and attach a custom Writer that brings both GMO and StableIdMap annotators
- Use ``parse_generic_model_output_data()`` to extract point data (including object IDs)
- Use ``parse_stable_id_map_data()`` to build a mapping from stable IDs to prim paths
- Correlate lidar points to the USD prims they hit

This is useful for:
- Identifying which objects in the scene were hit by lidar beams
- Semantic segmentation of point cloud data
- Object-level filtering and analysis

Requirements:
- StableIdMap output must be enabled via ``--/rtx-transient/stableIds/enabled=true``
- aux_output_level must be EXTRA or FULL to include object IDs in GMO
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
# StableIdMap requires the stableIds setting to be enabled.
# This is done via extra_args passed to SimulationApp.

simulation_app = SimulationApp(
    {
        "headless": True,
        "extra_args": ["--/rtx-transient/stableIds/enabled=true"],
    }
)

output_dir = os.path.join(os.getcwd(), "_example_output_isaacsim.sensors.experimental.rtx", "resolve_lidar_object_ids")
os.makedirs(output_dir, exist_ok=True)

import carb
import numpy as np
import omni.replicator.core as rep
import omni.timeline
from isaacsim.core.experimental.objects import Cube, DistantLight, GroundPlane
from isaacsim.sensors.experimental.rtx import (
    Lidar,
    LidarSensor,
    parse_generic_model_output_data,
    parse_object_ids,
    parse_stable_id_map_data,
)
from omni.replicator.core import Writer

# =============================================================================
# CREATE SCENE WITH IDENTIFIABLE OBJECTS
# =============================================================================
# Create several cubes at different positions. Each will have a unique prim path
# that we can later resolve from the lidar's stable ID map.

print("\n=== Creating scene objects ===")

cube_positions = {
    "/World/cube_front": np.array([5, 0, 1]),
    "/World/cube_left": np.array([0, 5, 1]),
    "/World/cube_right": np.array([0, -5, 1]),
    "/World/cube_back": np.array([-5, 0, 1]),
}

for path, position in cube_positions.items():
    Cube(path, positions=position, scales=np.array([2, 2, 2]), colors=[0.5, 0.5, 0.5])
    print(f"  Created {path} at position {position}")

# Add a ground plane
GroundPlane("/World/GroundPlane")

# Add lighting
light = DistantLight("/World/light")
light.set_intensities(3000.0)

# =============================================================================
# CREATE LIDAR WITH FULL AUXILIARY OUTPUT
# =============================================================================
# aux_output_level must be EXTRA or FULL to include objId in GMO data.
# The objId field contains 128-bit stable IDs for each point.

lidar = Lidar.create(
    "/World/lidar",
    config="Example_Rotary",
    translations=np.array([0, 0, 1.5]),  # Position above ground to see objects
    aux_output_level="FULL",
)

print(f"Created lidar at {lidar.paths[0]}")

# =============================================================================
# CREATE LIDAR SENSOR
# =============================================================================
# ``LidarSensor`` wraps a ``Lidar`` object and creates a render product.
# We pass ``annotators=[]`` because the writer brings its own annotators.

sensor = LidarSensor(lidar, annotators=[])

print("Created LidarSensor")


# =============================================================================
# CUSTOM WRITER FOR GMO + STABLE ID MAP COLLECTION
# =============================================================================
# A custom ``Writer`` receives data via its ``write()`` callback each frame.
# The writer brings both ``GenericModelOutput`` and ``StableIdMap`` annotators.
# After the simulation loop, we read ``writer.gmo`` and ``writer.stable_id_map``
# to print the object resolution table.


class GmoObjectIdWriter(Writer):
    """Writer that collects GenericModelOutput and StableIdMap data each frame."""

    def __init__(self) -> None:
        self.data_structure = "renderProduct"
        self.annotators = [
            rep.annotators.get("GenericModelOutput"),
            rep.annotators.get("StableIdMap"),
        ]
        self.gmo = None
        self.stable_id_map = None

    def write(self, data: dict[str, object]) -> None:
        """Collect GenericModelOutput and StableIdMap payloads."""
        if "renderProducts" not in data:
            return
        for _rp_name, rp_data in data["renderProducts"].items():
            gmo_raw = rp_data.get("GenericModelOutput")
            if isinstance(gmo_raw, dict):
                gmo_raw = gmo_raw.get("data")
            self.gmo = parse_generic_model_output_data(gmo_raw)

            sid_raw = rp_data.get("StableIdMap")
            if isinstance(sid_raw, dict):
                sid_raw = sid_raw.get("data")
            if sid_raw is not None:
                self.stable_id_map = parse_stable_id_map_data(sid_raw)


rep.WriterRegistry.register(GmoObjectIdWriter)
sensor.attach_writer("GmoObjectIdWriter")

# `attach_writer` creates a fresh writer instance internally and stores it on
# the sensor; reach into `_writers` to read back `gmo` / `stable_id_map` after
# the simulation loop below.
writer = sensor._writers["GmoObjectIdWriter"]

print("Attached GmoObjectIdWriter to sensor")

# =============================================================================
# RUN SIMULATION TO COLLECT DATA
# =============================================================================
if args.test:
    stage = omni.usd.get_context().get_stage()
    stage.Export(os.path.join(output_dir, "stage.usda"))

timeline = omni.timeline.get_timeline_interface()
timeline.play()

# Run for enough frames to collect valid data
print("Running simulation to collect lidar data...")
for _ in range(20):
    simulation_app.update()

# =============================================================================
# EXTRACT COLLECTED DATA FROM THE WRITER
# =============================================================================
gmo = writer.gmo
stable_id_map = writer.stable_id_map

if stable_id_map is None:
    carb.log_error("No stable ID map data available")
    timeline.stop()
    simulation_app.close()
    sys.exit()

print(f"Decoded StableIdMap with {len(stable_id_map)} entries")

if gmo is None:
    carb.log_error("No GMO data available")
    timeline.stop()
    simulation_app.close()
    sys.exit()

print(f"GMO contains {gmo.numElements} points")

# =============================================================================
# MAP OBJECT IDS TO PRIM PATHS
# =============================================================================
# objId from the GenericModelOutput is a buffer of 128-bit stable IDs. The
# StableIdMap uses the full 128-bit value as its key, so the lookup key must
# also be the full 128-bit integer — extracting only the lower 32 bits would
# silently miss for any hit on multi-subset meshes (where the upper 32 bits
# carry the submesh index) or procedural geometry (where the upper 32 bits
# carry the per-triangle primitive index).
#
# parse_object_ids() converts the raw buffer into a list of full 128-bit ints
# that match the StableIdMap keys directly.

print(f"\n{'='*60}")
print("Stable ID Map Contents")
print(f"{'='*60}")

for stable_id, prim_path in sorted(stable_id_map.items(), key=lambda x: x[1]):
    print(f"  ID {stable_id:>10d} -> {prim_path}")

# Count points per object using objId from GMO
if hasattr(gmo, "objId") and gmo.objId is not None and gmo.objId.size > 0:
    print(f"\n{'='*60}")
    print("Object Hit Count from GMO")
    print(f"{'='*60}")

    # Convert raw objId buffer to full 128-bit ints that match stable_id_map keys.
    obj_id_ints = parse_object_ids(gmo.objId)

    # Tally hits per unique 128-bit ID.
    counts = {}
    for oid in obj_id_ints:
        counts[oid] = counts.get(oid, 0) + 1

    for oid, count in sorted(counts.items(), key=lambda x: -x[1]):
        # Use map.get() because some IDs (e.g. procedural-geometry per-primitive
        # IDs) may legitimately have no map entry — see parse_stable_id_map_data
        # docstring for the full list of cases.
        prim_path = stable_id_map.get(oid, "<unknown>")
        print(f"  Object ID {oid:>40d}: {count:>6d} points -> {prim_path}")
else:
    carb.log_warn("No objId data available in GMO (ensure aux_output_level is EXTRA or FULL)")

# =============================================================================
# CLEANUP
# =============================================================================
timeline.stop()
simulation_app.close()
