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

"""Basic RTX Camera creation and rendering example.

This example demonstrates how to:
- Create an RTX Camera sensor using ``RtxCamera``
- Configure optical parameters (focal length, clipping range) via the ``.camera`` property
- Attach annotators (RGB, depth) via ``CameraSensor``
- Retrieve rendered images during simulation
"""

import argparse
import sys

from isaacsim import SimulationApp

parser = argparse.ArgumentParser(description="Basic RTX Camera example.")
parser.add_argument("--test", default=False, action="store_true", help="Run in test mode.")
parser.add_argument("--disable-output", action="store_true", help="Disable debug output.")
args, _ = parser.parse_known_args()

simulation_app = SimulationApp({"headless": False})

import os

import carb
import matplotlib.pyplot as plt
import numpy as np
import omni
import omni.usd
from isaacsim.core.experimental.utils.stage import is_stage_loading, open_stage
from isaacsim.sensors.experimental.rtx import CameraSensor, RtxCamera
from isaacsim.storage.native import get_assets_root_path

output_dir = os.path.join(os.getcwd(), "_example_output_isaacsim.sensors.experimental.rtx", "create_camera_basic")
os.makedirs(output_dir, exist_ok=True)

# =============================================================================
# LOAD ENVIRONMENT
# =============================================================================

assets_root_path = get_assets_root_path()
if assets_root_path is None:
    carb.log_error("Could not find Isaac Sim assets folder")
    simulation_app.close()
    sys.exit()

open_stage(usd_path=assets_root_path + "/Isaac/Environments/Simple_Warehouse/full_warehouse.usd")
while is_stage_loading():
    simulation_app.update()

# =============================================================================
# CREATE RTX CAMERA
# =============================================================================
# RtxCamera creates a USD Camera prim with the OmniSensorAPI schema applied,
# enabling tick-rate-controlled rendering. Optical parameters are accessible
# via the .camera property.

cam = RtxCamera(
    "/World/camera",
    tick_rate=30.0,
    translations=np.array([0.0, 0.0, 1.5]),
    # wxyz; 90 deg about world +X. Looking direction is world +Y (forward
    # into the warehouse), image up is world +Z, image right is world +X.
    orientations=np.array([1.0, 1.0, 0.0, 0.0]) / np.sqrt(2.0),
)

# Configure optical parameters via the Camera wrapper
cam.camera.set_focal_lengths(24.0)
cam.camera.set_clipping_ranges(0.01, 1000.0)

print(f"Created RTX Camera at {cam.paths[0]}")
print(f"  Focal length: {cam.camera.get_focal_lengths().numpy()[0]}")

# =============================================================================
# CREATE CAMERA SENSOR FOR RUNTIME
# =============================================================================
# CameraSensor wraps the RtxCamera authoring object, creates a render product
# at the specified resolution, and attaches annotators for data retrieval.

resolution = (480, 640)  # (height, width) — OpenCV/NumPy convention
sensor = CameraSensor(
    cam,
    resolution=resolution,
    annotators=["rgb", "distance_to_image_plane"],
)

print(f"Created CameraSensor with resolution {resolution}")
print(f"  Annotators: {sensor.annotators}")

# =============================================================================
# RUN SIMULATION AND RETRIEVE DATA
# =============================================================================

timeline = omni.timeline.get_timeline_interface()

if args.test:
    stage = omni.usd.get_context().get_stage()
    stage.Export(os.path.join(output_dir, "stage.usda"))

timeline.play()

print("Starting simulation")

frame_count = 0

while simulation_app.is_running() and (not args.test or frame_count < 601):
    simulation_app.update()

    rgb_data, rgb_info = sensor.get_data("rgb")
    depth_data, depth_info = sensor.get_data("distance_to_image_plane")

    if not args.disable_output and rgb_data is not None:
        print(f"\nFrame {frame_count}:")
        print(f"  RGB shape: {rgb_data.shape}, dtype: {rgb_data.dtype}")
        if depth_data is not None:
            print(f"  Depth shape: {depth_data.shape}, dtype: {depth_data.dtype}")
            depth_np = depth_data.numpy()
            print(f"  Depth range: [{depth_np.min():.2f}, {depth_np.max():.2f}] meters")

    if (frame_count + 1) % 100 == 0 and rgb_data is not None:
        rgb_np = rgb_data.numpy()
        imgplot = plt.imshow(rgb_np)
        output_path = os.path.join(output_dir, f"camera.frame{frame_count:03d}.png")
        print(f"Saving image to: {output_path}")
        plt.draw()
        plt.savefig(output_path)

    frame_count += 1

timeline.stop()
simulation_app.close()
