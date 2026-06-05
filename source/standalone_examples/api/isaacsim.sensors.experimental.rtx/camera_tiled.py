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

"""Batch/tiled multi-camera rendering with TiledCameraSensor.

This example demonstrates how to:
- Create multiple cameras at different positions
- Use ``TiledCameraSensor`` for efficient batched rendering
- Retrieve data as a single tiled frame or as a batch of individual frames
- Save RGB and depth data in both tiled and batched formats

Scene and parameters match the deprecated ``isaacsim.sensors.camera/camera_view.py``
example for output comparison.
"""

import argparse
import os

from isaacsim import SimulationApp

parser = argparse.ArgumentParser(description="Tiled multi-camera rendering example.")
parser.add_argument("--num-cameras", type=int, default=4, help="Number of cameras.")
parser.add_argument("--test", default=False, action="store_true", help="Run in test mode.")
args, _ = parser.parse_known_args()

simulation_app = SimulationApp({"headless": False})

output_dir = os.path.join(os.getcwd(), "_example_output_isaacsim.sensors.experimental.rtx", "camera_tiled")
os.makedirs(output_dir, exist_ok=True)
os.makedirs(os.path.join(output_dir, "tiled"), exist_ok=True)
os.makedirs(os.path.join(output_dir, "batched"), exist_ok=True)

import numpy as np
import omni
import omni.usd
from isaacsim.core.experimental.materials import OmniPbrMaterial
from isaacsim.core.experimental.objects import Cube, DomeLight, GroundPlane
from isaacsim.sensors.experimental.rtx import RtxCamera, TiledCameraSensor
from PIL import Image

NUM_CAPTURES = 2
RESOLUTION = (256, 256)

# =============================================================================
# CREATE SCENE (matches deprecated camera_view.py)
# =============================================================================

cube_material = OmniPbrMaterial("/World/Materials/cube_blue")
cube_material.set_input_values("diffuse_color_constant", [0.0, 0.0, 1.0])
for i in range(2):
    cube = Cube(f"/new_cube_{i}", sizes=1.0, positions=np.array([0, i * 0.5, 0.2]), scales=np.array([0.1, 0.1, 0.1]))
    cube.apply_visual_materials(cube_material)

dome_light = DomeLight("/World/DomeLight")
dome_light.set_intensities(500)
GroundPlane("/World/defaultGroundPlane", sizes=100.0)

# =============================================================================
# CREATE MULTIPLE CAMERAS (same positions as deprecated example)
# =============================================================================

cam_positions = [(0, 0, 2), (0, 1, 2), (1, 0, 2), (1, 1, 2)]
for i, pos in enumerate(cam_positions):
    RtxCamera(f"/World/camera_{i}", translations=np.array(pos, dtype=float))

print(f"Created {len(cam_positions)} cameras")

# =============================================================================
# CREATE TILED CAMERA SENSOR
# =============================================================================

sensor = TiledCameraSensor(
    "/World/camera_.*",
    resolution=RESOLUTION,
    annotators=["rgb", "distance_to_image_plane"],
)

print(f"Created TiledCameraSensor:")
print(f"  Number of cameras: {len(sensor)}")
print(f"  Per-camera resolution: {sensor.resolution}")
print(f"  Tiled resolution: {sensor.tiled_resolution}")

# =============================================================================
# RUN SIMULATION AND CAPTURE DATA
# =============================================================================

if args.test:
    stage = omni.usd.get_context().get_stage()
    stage.Export(os.path.join(output_dir, "stage.usda"))

timeline = omni.timeline.get_timeline_interface()
timeline.play()

for _ in range(5):
    simulation_app.update()

for i in range(NUM_CAPTURES):
    print(f" ** Step {i} ** ")
    simulation_app.update()

    # --- RGB tiled ---
    print(" ** Running RGB data tests:")
    rgb_tiled, _ = sensor.get_data("rgb", tiled=True)
    if rgb_tiled is not None:
        rgb_tiled_np = rgb_tiled.numpy()
        print(f"rgb_tiled_np.shape: {rgb_tiled_np.shape}, type: {type(rgb_tiled_np)}, dtype: {rgb_tiled_np.dtype}")
        Image.fromarray(rgb_tiled_np).save(os.path.join(output_dir, "tiled", f"{str(i).zfill(3)}_rgb_tiled_np.png"))

    # --- RGB batched ---
    print(" ** Batched:")
    rgb_batch, _ = sensor.get_data("rgb", tiled=False)
    if rgb_batch is not None:
        rgb_batch_np = rgb_batch.numpy()
        print(f"rgb_batched.shape: {rgb_batch_np.shape}, type: {type(rgb_batch_np)}, dtype: {rgb_batch_np.dtype}")
        for camera_id in range(rgb_batch_np.shape[0]):
            rgb_cam = rgb_batch_np[camera_id]
            print(
                f"camera_id={camera_id}: rgb_batched.shape: {rgb_cam.shape}, type: {type(rgb_cam)}, dtype: {rgb_cam.dtype}"
            )
            Image.fromarray(rgb_cam).save(
                os.path.join(output_dir, "batched", f"{str(i).zfill(3)}_rgb_batched_{camera_id}.png")
            )

    # --- Depth tiled ---
    print(" ** Running depth data tests:")
    depth_tiled, _ = sensor.get_data("distance_to_image_plane", tiled=True)
    if depth_tiled is not None:
        depth_np = depth_tiled.numpy()
        print(f"depth_tiled_np.shape: {depth_np.shape}, type: {type(depth_np)}, dtype: {depth_np.dtype}")
        depth_np[np.isinf(depth_np)] = 0.0
        depth_np = np.clip(depth_np, 0.0, 1.0)
        depth_uint8 = (depth_np * 255).squeeze().astype(np.uint8)
        Image.fromarray(depth_uint8, mode="L").save(
            os.path.join(output_dir, "tiled", f"{str(i).zfill(3)}_depth_tiled_np.png")
        )

    # --- Depth batched ---
    print(" ** Batched:")
    depth_batch, _ = sensor.get_data("distance_to_image_plane", tiled=False)
    if depth_batch is not None:
        depth_batch_np = depth_batch.numpy()
        print(
            f"depth_batched.shape: {depth_batch_np.shape}, type: {type(depth_batch_np)}, dtype: {depth_batch_np.dtype}"
        )
        depth_batch_np[np.isinf(depth_batch_np)] = 0.0
        depth_batch_np = np.clip(depth_batch_np, 0.0, 1.0)
        for camera_id in range(depth_batch_np.shape[0]):
            depth_cam = (depth_batch_np[camera_id] * 255).squeeze().astype(np.uint8)
            Image.fromarray(depth_cam, mode="L").save(
                os.path.join(output_dir, "batched", f"{str(i).zfill(3)}_depth_batched_{camera_id}.png")
            )

    simulation_app.update()

timeline.stop()
simulation_app.close()
