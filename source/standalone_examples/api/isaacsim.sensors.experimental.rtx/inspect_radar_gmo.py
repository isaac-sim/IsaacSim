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

"""Inspect RTX Radar GenericModelOutput (GMO) data using the new experimental API.

This example demonstrates how to:
- Create an RTX Radar sensor using the ``Radar()`` constructor
- Create a ``RadarSensor`` and attach a custom Writer via ``sensor.attach_writer()``
- Use ``parse_generic_model_output_data()`` to parse the radar GMO buffer inside a Writer
- Explore radar-specific GMO fields including velocity (Doppler) data

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

from isaacsim import SimulationApp

parser = argparse.ArgumentParser(description="Inspect RTX Radar GMO data.")
parser.add_argument("--test", default=False, action="store_true", help="Run in test mode.")
args, _ = parser.parse_known_args()

# headless=True since we are just inspecting data, not visualizing
# enable_motion_bvh=True is REQUIRED for radar
simulation_app = SimulationApp({"headless": True, "enable_motion_bvh": True})

output_dir = os.path.join(os.getcwd(), "_example_output_isaacsim.sensors.experimental.rtx", "inspect_radar_gmo")
os.makedirs(output_dir, exist_ok=True)

import numpy as np
import omni
import omni.replicator.core as rep
from isaacsim.core.experimental.objects import Cube
from isaacsim.sensors.experimental.rtx import Radar, RadarSensor, parse_generic_model_output_data
from omni.replicator.core import Writer

# =============================================================================
# CREATE A SIMPLE SCENE WITH CUBES
# =============================================================================
# Create a few cubes at known positions so the radar has geometry to detect.

print("Creating simple test scene with cubes")

Cube("/World/cube_front", positions=np.array([5.0, 0.0, 0.0]), scales=np.array([2.0, 2.0, 2.0]))
Cube("/World/cube_left", positions=np.array([0.0, 5.0, 0.0]), scales=np.array([2.0, 2.0, 2.0]))
Cube("/World/cube_right", positions=np.array([0.0, -5.0, 0.0]), scales=np.array([2.0, 2.0, 2.0]))
Cube("/World/cube_above", positions=np.array([0.0, 0.0, 5.0]), scales=np.array([2.0, 2.0, 2.0]))

# =============================================================================
# CREATE RTX RADAR USING THE NEW EXPERIMENTAL API
# =============================================================================
# The ``Radar()`` constructor creates an OmniRadar prim at the specified path.
# Radar uses the WpmDmat sensor model. Set ``aux_output_level`` to ``BASIC`` for
# all available radar data fields.

radar = Radar(
    "/World/radar",
    translations=np.array([0, 0, 1.0]),  # Position 1 meter above ground
    orientations=np.array([1.0, 0.0, 0.0, 0.0]),  # Identity rotation (w, x, y, z)
    aux_output_level="BASIC",
)

print(f"Created RTX Radar at {radar.paths[0]}")

# =============================================================================
# CREATE RADAR SENSOR
# =============================================================================
# ``RadarSensor`` wraps a ``Radar`` object and creates a render product.
# We pass ``annotators=[]`` because the writer brings its own annotator.

sensor = RadarSensor(radar, annotators=[])

print("Created RadarSensor")

# =============================================================================
# RADAR-SPECIFIC GMO FIELDS
# =============================================================================
# These are the fields available in radar GMO output.

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
def inspect_radar_gmo(frame: int, gmo: object) -> None:
    """Print radar GMO data fields.

    Args:
        frame: Current frame number.
        gmo: Parsed GenericModelOutput structure.
    """
    print(f"\n{'='*60}")
    print(f"Frame {frame} -- Radar GMO Data")
    print(f"{'='*60}")

    for field_name, description in RADAR_GMO_FIELDS:
        value = getattr(gmo, field_name, "N/A")
        print(f"{field_name}: {value}")
        print(f"    ({description})")

    # Print sample detection data (first 5 points)
    print(f"\n-- Sample Radar Detections (first 5 points) --")
    num_samples = min(5, gmo.numElements)
    for i in range(num_samples):
        rv = getattr(gmo, "rv_ms", None)
        rv_str = f", rv_ms={rv[i]:.3f}" if rv is not None and hasattr(rv, "__getitem__") else ""
        print(
            f"  Detection {i}: x={gmo.x[i]:.3f}, y={gmo.y[i]:.3f}, z={gmo.z[i]:.3f},"
            f" timeOffsetNs={gmo.timeOffsetNs[i]}{rv_str}"
        )


# =============================================================================
# CUSTOM WRITER FOR RADAR GMO DATA INSPECTION
# =============================================================================
# A custom ``Writer`` receives data via its ``write()`` callback each frame.
# The writer brings its own ``GenericModelOutput`` annotator, so the sensor
# does not need to specify one.


class GmoRadarInspectWriter(Writer):
    """Writer that parses GenericModelOutput and prints radar GMO fields."""

    def __init__(self) -> None:
        self.data_structure = "renderProduct"
        self.annotators = [rep.annotators.get("GenericModelOutput")]
        self._frame_count = 0

    def write(self, data: dict[str, object]) -> None:
        """Inspect radar GenericModelOutput data."""
        if "renderProducts" not in data:
            return
        for _rp_name, rp_data in data["renderProducts"].items():
            gmo_raw = rp_data.get("GenericModelOutput")
            if isinstance(gmo_raw, dict):
                gmo_raw = gmo_raw.get("data")
            gmo = parse_generic_model_output_data(gmo_raw)
            if gmo.numElements > 0:
                inspect_radar_gmo(frame=self._frame_count, gmo=gmo)
        self._frame_count += 1


rep.WriterRegistry.register(GmoRadarInspectWriter)
sensor.attach_writer("GmoRadarInspectWriter")

print("Attached GmoRadarInspectWriter to sensor")

# =============================================================================
# RUN SIMULATION AND INSPECT DATA
# =============================================================================
if args.test:
    stage = omni.usd.get_context().get_stage()
    stage.Export(os.path.join(output_dir, "stage.usda"))

timeline = omni.timeline.get_timeline_interface()
timeline.play()

print("Starting simulation - inspecting radar GMO data each frame")

frame_count = 0
while simulation_app.is_running() and (not args.test or frame_count < 5):
    simulation_app.update()
    frame_count += 1

# =============================================================================
# CLEANUP
# =============================================================================
timeline.stop()
simulation_app.close()
