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

"""Basic RTX Radar creation and visualization example using the new experimental API.

This example demonstrates how to:
- Create an RTX Radar sensor using the ``Radar()`` constructor
- Attach the ``RtxSensorDebugDrawPointCloud`` writer for viewport visualization
- Customize debug draw point size and color for radar detections
- Run simulation and observe radar detections in the environment

Note: RTX Radar requires Motion BVH to be enabled. This is done by passing
``"enable_motion_bvh": True`` to the SimulationApp configuration.

This is the recommended starting point for learning the new RTX Radar API
in ``isaacsim.sensors.experimental.rtx``.
"""

import argparse
import os
import sys

from isaacsim import SimulationApp

parser = argparse.ArgumentParser(description="Basic RTX Radar example with debug draw visualization.")
parser.add_argument("--test", default=False, action="store_true", help="Run in test mode.")
args, _ = parser.parse_known_args()

# Note: headless=False is required for debug draw visualization
# Note: enable_motion_bvh=True is REQUIRED for RTX Radar to function
simulation_app = SimulationApp({"headless": False, "enable_motion_bvh": True})

output_dir = os.path.join(os.getcwd(), "_example_output_isaacsim.sensors.experimental.rtx", "create_radar_basic")
os.makedirs(output_dir, exist_ok=True)

import carb
import isaacsim.core.experimental.utils.stage as stage_utils
import numpy as np
import omni
from isaacsim.core.experimental.utils.app import enable_extension
from isaacsim.core.experimental.utils.stage import is_stage_loading, open_stage
from isaacsim.sensors.experimental.rtx import Radar, RadarSensor
from isaacsim.storage.native import get_assets_root_path

enable_extension("isaacsim.sensors.rtx.nodes")

# =============================================================================
# LOAD ENVIRONMENT
# =============================================================================
# Locate Isaac Sim assets folder and load a warehouse for the radar to scan.

assets_root_path = get_assets_root_path()
if assets_root_path is None:
    carb.log_error("Could not find Isaac Sim assets folder")
    simulation_app.close()
    sys.exit()

open_stage(usd_path=assets_root_path + "/Isaac/Environments/Simple_Warehouse/full_warehouse.usd")
while is_stage_loading():
    simulation_app.update()

# Replicator's radar-create parent check runs on pxr USD; ensure `/World` is
# present on the pxr stage before constructing the radar (warehouse load may
# complete on the Fabric/USDRT side first).
stage_utils.define_prim("/World", type_name="Xform")

# =============================================================================
# CREATE RTX RADAR USING THE NEW EXPERIMENTAL API
# =============================================================================
# The ``Radar()`` constructor creates an OmniRadar prim at the specified path.
# Unlike lidar, radar detects objects based on their radar cross-section and
# provides velocity information (Doppler).
#
# IMPORTANT: Radar will not be created if Motion BVH is not enabled.
# Ensure ``"enable_motion_bvh": True`` is set in SimulationApp config.
#
# The ``translations`` parameter positions the radar in local frame coordinates.
# The ``orientations`` parameter (wxyz quaternion) sets the radar facing direction.

radar = Radar(
    "/World/radar",
    translations=np.array([0, 0, 1.0]),  # Position 1 meter above ground
    # Rotate 90 deg about Z-axis so radar faces the warehouse shelves
    orientations=np.array([0.70711, 0.0, 0.0, 0.70711]),  # (w, x, y, z)
)

print(f"Created RTX Radar at {radar.paths[0]}")

# =============================================================================
# CREATE RADAR SENSOR FOR RUNTIME
# =============================================================================
# RadarSensor wraps the Radar authoring object and creates a render product.
# The "draw-point-cloud" writer (registered by isaacsim.sensors.rtx.nodes)
# extracts the radar detections and draws them in the viewport via debug draw.
#
# Radar typically returns far fewer points than lidar, so we use a larger
# ``size`` and a distinctive color to make the detections visible. The
# constructor's ``writers=`` parameter only forwards the writer's registered
# defaults, so we attach the writer explicitly to pass ``size``/``color``.

sensor = RadarSensor(radar, annotators=[])
sensor.attach_writer(
    "draw-point-cloud",
    size=0.2,  # Larger point size for visibility (radar has fewer points)
    color=[1.0, 0.2, 0.3, 1.0],  # Orange-red with full opacity
)

print("Created RadarSensor with debug draw visualization (orange points, size=0.2)")

# =============================================================================
# RUN SIMULATION
# =============================================================================
if args.test:
    stage = omni.usd.get_context().get_stage()
    stage.Export(os.path.join(output_dir, "stage.usda"))

timeline = omni.timeline.get_timeline_interface()
timeline.play()

print("Starting simulation - observe the radar detections in the viewport")

frame_count = 0
while simulation_app.is_running():
    simulation_app.update()
    frame_count += 1

    # In test mode, exit after a few frames
    if args.test and frame_count >= 10:
        break

# Cleanup
timeline.stop()
simulation_app.close()
