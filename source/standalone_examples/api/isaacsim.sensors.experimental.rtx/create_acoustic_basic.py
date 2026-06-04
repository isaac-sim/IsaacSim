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

"""Basic RTX Acoustic sensor creation example.

This example demonstrates how to:
- Create an RTX Acoustic sensor using the Acoustic class
- Configure sensor mounts (transmitters and receivers) via multi-apply schemas
- Configure receiver groups for grouping receivers
- Attach a custom Writer via ``sensor.attach_writer()`` to receive GMO data
- Run simulation and verify data is being produced

Unlike lidar and radar, acoustic sensors do not produce a point cloud.
They produce "signal ways" — amplitude samples for each transmitter-receiver
pair on each channel. The GenericModelOutput fields have different meanings:
  - x: transmitter sensor mount ID
  - y: receiver sensor mount ID
  - z: channel ID
  - scalar: amplitude sample value
"""

import argparse
import os
import sys

from isaacsim import SimulationApp

parser = argparse.ArgumentParser(description="Basic RTX Acoustic sensor example.")
parser.add_argument("--test", default=False, action="store_true", help="Run in test mode.")
args, _ = parser.parse_known_args()

simulation_app = SimulationApp({"headless": True})

output_dir = os.path.join(os.getcwd(), "_example_output_isaacsim.sensors.experimental.rtx", "create_acoustic_basic")
os.makedirs(output_dir, exist_ok=True)

import numpy as np
import omni
import omni.replicator.core as rep
from isaacsim.core.experimental.objects import Cube
from isaacsim.sensors.experimental.rtx import Acoustic, AcousticSensor, parse_generic_model_output_data
from omni.replicator.core import Writer

# =============================================================================
# CREATE A SIMPLE SCENE
# =============================================================================
# Acoustic sensors need objects to reflect waves off of.

Cube("/World/target", positions=np.array([5.0, 0.0, 0.0]), scales=np.array([2.0, 2.0, 2.0]))

print("Created target cube at (5, 0, 0)")

# =============================================================================
# CREATE ACOUSTIC SENSOR
# =============================================================================
# The Acoustic class creates an OmniAcoustic prim with the
# OmniSensorGenericAcousticWpmAPI schema applied.
#
# Acoustic sensors use multi-apply schemas to define sensor mounts (transmitter
# and receiver positions) and receiver groups.  Attribute prefixes determine
# which schema instance is applied:
#   - omni:sensor:WpmAcoustic:sensorMount:<instance>:*  -> OmniSensorWpmAcousticSensorMountAPI
#   - omni:sensor:WpmAcoustic:rxGroup:<instance>:*      -> OmniSensorWpmAcousticRxGroupAPI

acoustic = Acoustic(
    "/World/acoustic",
    tick_rate=20.0,
    translations=np.array([0.0, 0.0, 1.0]),
    attributes={
        # Center frequency of the acoustic wave in Hz
        "omni:sensor:WpmAcoustic:centerFrequency": 40000.0,
        # Define two sensor mounts (transmitter + receiver positions)
        "omni:sensor:WpmAcoustic:sensorMount:m001:position": (0.0, 0.0, 0.0),
        "omni:sensor:WpmAcoustic:sensorMount:m001:rotation": (0.0, 0.0, 0.0),
        "omni:sensor:WpmAcoustic:sensorMount:m002:position": (0.1, 0.0, 0.0),
        "omni:sensor:WpmAcoustic:sensorMount:m002:rotation": (0.0, 0.0, 0.0),
        # Define a receiver group that includes both mounts
        "omni:sensor:WpmAcoustic:rxGroup:g001:receiverIndices": [0, 1],
    },
)

print(f"Created acoustic sensor at {acoustic.paths[0]} with centerFrequency=40000 Hz")

# =============================================================================
# CREATE ACOUSTIC SENSOR RUNTIME
# =============================================================================
# AcousticSensor wraps the authoring object and creates a render product.
# We pass ``annotators=[]`` because the writer brings its own annotator.

sensor = AcousticSensor(acoustic, annotators=[])

print("Created AcousticSensor")


# =============================================================================
# CUSTOM WRITER FOR ACOUSTIC GMO DATA
# =============================================================================
# A custom ``Writer`` receives data via its ``write()`` callback each frame.
# The writer brings its own ``GenericModelOutput`` annotator, so the sensor
# does not need to specify one.


class GmoAcousticBasicWriter(Writer):
    """Writer that parses GenericModelOutput and prints first-data-received info."""

    def __init__(self):
        self.data_structure = "renderProduct"
        self.annotators = [rep.annotators.get("GenericModelOutput")]
        self._frame_count = 0
        self._data_received = False

    def write(self, data):
        if "renderProducts" not in data:
            return
        for _rp_name, rp_data in data["renderProducts"].items():
            gmo_raw = rp_data.get("GenericModelOutput")
            if isinstance(gmo_raw, dict):
                gmo_raw = gmo_raw.get("data")
            gmo = parse_generic_model_output_data(gmo_raw)
            if gmo.numElements > 0 and not self._data_received:
                self._data_received = True
                print(f"First data received at frame {self._frame_count}")
                print(f"  Number of elements (signal way samples): {gmo.numElements}")
                # For acoustic sensors:
                #   x = transmitter sensor mount ID
                #   y = receiver sensor mount ID
                #   z = channel ID
                #   scalar = amplitude sample value
                tx_ids = np.ctypeslib.as_array(gmo.x, shape=(gmo.numElements,))
                rx_ids = np.ctypeslib.as_array(gmo.y, shape=(gmo.numElements,))
                ch_ids = np.ctypeslib.as_array(gmo.z, shape=(gmo.numElements,))
                amplitudes = np.ctypeslib.as_array(gmo.scalar, shape=(gmo.numElements,))
                print(f"  Unique transmitter IDs: {np.unique(tx_ids).tolist()}")
                print(f"  Unique receiver IDs: {np.unique(rx_ids).tolist()}")
                print(f"  Unique channel IDs: {np.unique(ch_ids).tolist()}")
                print(f"  Amplitude range: [{amplitudes.min():.6f}, {amplitudes.max():.6f}]")
        self._frame_count += 1


rep.WriterRegistry.register(GmoAcousticBasicWriter)
sensor.attach_writer("GmoAcousticBasicWriter")

print("Attached GmoAcousticBasicWriter to sensor")

# =============================================================================
# RUN SIMULATION AND COLLECT DATA
# =============================================================================
if args.test:
    stage = omni.usd.get_context().get_stage()
    stage.Export(os.path.join(output_dir, "stage.usda"))

timeline = omni.timeline.get_timeline_interface()
timeline.play()

print("Starting simulation — collecting acoustic data")

frame_count = 0
while simulation_app.is_running() and (not args.test or frame_count < 20):
    simulation_app.update()
    frame_count += 1

timeline.stop()
simulation_app.close()
