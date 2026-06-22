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

"""Inspect RTX Acoustic GenericModelOutput (GMO) data.

This example demonstrates how to:
- Create an RTX Acoustic sensor with multiple sensor mounts
- Attach a custom Writer via ``sensor.attach_writer()`` to receive GMO data
- Understand the acoustic-specific GMO data structure

Acoustic GMO data is organized as "signal ways".  Each signal way represents
the waveform received at one receiver from one transmitter on a specific
channel.  The BasicElements fields have the following meaning for acoustic:

  x[i]      — transmitter sensor mount ID
  y[i]      — receiver sensor mount ID
  z[i]      — channel ID
  scalar[i] — amplitude sample value

When auxiliary data is enabled, the AcousticAuxiliaryData provides:
  numSgws           — number of signal ways in the frame
  numSamplesPerSgw  — number of amplitude samples per signal way
"""

import argparse
import os

from isaacsim import SimulationApp

parser = argparse.ArgumentParser(description="Inspect RTX Acoustic GMO data.")
parser.add_argument("--test", default=False, action="store_true", help="Run in test mode.")
args, _ = parser.parse_known_args()

simulation_app = SimulationApp({"headless": True})

output_dir = os.path.join(os.getcwd(), "_example_output_isaacsim.sensors.experimental.rtx", "inspect_acoustic_gmo")
os.makedirs(output_dir, exist_ok=True)

import numpy as np
import omni
import omni.replicator.core as rep
from isaacsim.core.experimental.objects import Cube
from isaacsim.sensors.experimental.rtx import Acoustic, AcousticSensor, parse_generic_model_output_data
from omni.replicator.core import Writer

# =============================================================================
# CREATE SCENE
# =============================================================================

# Place reflective targets at various distances
for i, (x, y) in enumerate([(3, 0), (5, 2), (4, -2)]):
    Cube(
        f"/World/target_{i}",
        positions=np.array([float(x), float(y), 0.0]),
        scales=np.array([1.0, 1.0, 2.0]),
    )

print("Created 3 target cubes")

# =============================================================================
# CREATE ACOUSTIC SENSOR WITH MULTIPLE MOUNTS
# =============================================================================

acoustic = Acoustic(
    "/World/acoustic",
    tick_rate=30.0,
    translations=np.array([0.0, 0.0, 0.5]),
    attributes={
        "omni:sensor:WpmAcoustic:centerFrequency": 51200.0,
        # Three sensor mounts forming a small array
        "omni:sensor:WpmAcoustic:sensorMount:m001:position": (0.0, -0.05, 0.0),
        "omni:sensor:WpmAcoustic:sensorMount:m001:rotation": (0.0, 0.0, 0.0),
        "omni:sensor:WpmAcoustic:sensorMount:m002:position": (0.0, 0.0, 0.0),
        "omni:sensor:WpmAcoustic:sensorMount:m002:rotation": (0.0, 0.0, 0.0),
        "omni:sensor:WpmAcoustic:sensorMount:m003:position": (0.0, 0.05, 0.0),
        "omni:sensor:WpmAcoustic:sensorMount:m003:rotation": (0.0, 0.0, 0.0),
        # Two receiver groups
        "omni:sensor:WpmAcoustic:rxGroup:g001:receiverIndices": [0, 1],
        "omni:sensor:WpmAcoustic:rxGroup:g002:receiverIndices": [1, 2],
    },
)

print(f"Created acoustic sensor at {acoustic.paths[0]}")
print("  centerFrequency: 51200 Hz")
print("  3 sensor mounts, 2 receiver groups")

# =============================================================================
# CREATE RUNTIME SENSOR
# =============================================================================
# We pass ``annotators=[]`` because the writer brings its own annotator.

sensor = AcousticSensor(acoustic, annotators=[])

print("Created AcousticSensor")


# =============================================================================
# CUSTOM WRITER FOR ACOUSTIC GMO DATA INSPECTION
# =============================================================================
# A custom ``Writer`` receives data via its ``write()`` callback each frame.
# The writer brings its own ``GenericModelOutput`` annotator, so the sensor
# does not need to specify one.


class GmoAcousticInspectWriter(Writer):
    """Writer that parses GenericModelOutput and prints acoustic GMO fields."""

    def __init__(self) -> None:
        self.data_structure = "renderProduct"
        self.annotators = [rep.annotators.get("GenericModelOutput")]
        self._frame_count = 0
        self._printed_details = False

    def write(self, data: dict[str, object]) -> None:
        """Inspect acoustic GenericModelOutput data."""
        if "renderProducts" not in data:
            return
        for _rp_name, rp_data in data["renderProducts"].items():
            gmo_raw = rp_data.get("GenericModelOutput")
            if isinstance(gmo_raw, dict):
                gmo_raw = gmo_raw.get("data")
            gmo = parse_generic_model_output_data(gmo_raw)
            if gmo.numElements == 0:
                self._frame_count += 1
                continue

            if not self._printed_details:
                self._printed_details = True
                print(f"\n{'='*60}")
                print(f"ACOUSTIC GMO DATA — frame {self._frame_count}")
                print(f"{'='*60}")
                print(f"  numElements:  {gmo.numElements}")
                print(f"  timestampNs:  {gmo.timestampNs}")

                n = gmo.numElements
                tx_ids = np.ctypeslib.as_array(gmo.x, shape=(n,))
                rx_ids = np.ctypeslib.as_array(gmo.y, shape=(n,))
                ch_ids = np.ctypeslib.as_array(gmo.z, shape=(n,))
                amplitudes = np.ctypeslib.as_array(gmo.scalar, shape=(n,))

                print(f"\n  --- Per-element fields ---")
                print(f"  x  (transmitter mount IDs): unique = {np.unique(tx_ids).tolist()}")
                print(f"  y  (receiver mount IDs):    unique = {np.unique(rx_ids).tolist()}")
                print(f"  z  (channel IDs):           unique = {np.unique(ch_ids).tolist()}")
                print(f"  scalar (amplitude):         min={amplitudes.min():.6f}, max={amplitudes.max():.6f}")

                # Count signal ways: each unique (tx, rx, channel) combination
                # is one signal way
                if n > 0:
                    sgw_keys = set()
                    for j in range(min(n, 10000)):
                        sgw_keys.add((int(tx_ids[j]), int(rx_ids[j]), int(ch_ids[j])))
                    print(f"\n  Detected signal ways (tx, rx, ch): {len(sgw_keys)}")
                    for key in sorted(sgw_keys):
                        mask = (tx_ids == key[0]) & (rx_ids == key[1]) & (ch_ids == key[2])
                        count = int(mask.sum())
                        amps = amplitudes[mask]
                        print(
                            f"    tx={key[0]}, rx={key[1]}, ch={key[2]}: "
                            f"{count} samples, amp range [{amps.min():.6f}, {amps.max():.6f}]"
                        )

                print(f"{'='*60}\n")

        self._frame_count += 1


rep.WriterRegistry.register(GmoAcousticInspectWriter)
sensor.attach_writer("GmoAcousticInspectWriter")

print("Attached GmoAcousticInspectWriter to sensor")

# =============================================================================
# RUN SIMULATION AND INSPECT DATA
# =============================================================================
if args.test:
    stage = omni.usd.get_context().get_stage()
    stage.Export(os.path.join(output_dir, "stage.usda"))

timeline = omni.timeline.get_timeline_interface()
timeline.play()

print("Starting simulation")

frame_count = 0
while simulation_app.is_running() and (not args.test or frame_count < 20):
    simulation_app.update()
    frame_count += 1

timeline.stop()
simulation_app.close()
