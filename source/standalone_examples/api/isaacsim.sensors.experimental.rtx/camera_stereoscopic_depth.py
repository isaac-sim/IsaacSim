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

"""Configure a single-view stereoscopic depth camera sensor.

This example demonstrates how to:
- Create a camera with ``SingleViewDepthCameraSensor`` for depth simulation
- Configure depth-specific parameters (baseline, focal length, disparity, noise)
- Retrieve depth data from the ``depth_sensor_distance`` annotator

The single-view depth sensor simulates a stereo camera pair from a single viewpoint,
computing disparity and depth using the ``OmniSensorDepthSensorSingleViewAPI`` schema
applied to the render product.
"""

import argparse
import os

from isaacsim import SimulationApp

parser = argparse.ArgumentParser(description="Single-view stereoscopic depth camera example.")
parser.add_argument("--test", default=False, action="store_true", help="Run in test mode.")
args, _ = parser.parse_known_args()

simulation_app = SimulationApp({"headless": args.test})

output_dir = os.path.join(os.getcwd(), "_example_output_isaacsim.sensors.experimental.rtx", "camera_stereoscopic_depth")
os.makedirs(output_dir, exist_ok=True)

import numpy as np
import omni
import omni.usd
from isaacsim.core.experimental.materials import OmniPbrMaterial
from isaacsim.core.experimental.objects import Cone, Cube
from isaacsim.core.experimental.utils.transform import euler_angles_to_quaternion
from isaacsim.sensors.experimental.rtx import RtxCamera, SingleViewDepthCameraSensor
from isaacsim.storage.native.nucleus import get_assets_root_path

# =============================================================================
# CREATE SCENE (matches deprecated isaacsim.sensors.camera example)
# =============================================================================

cube_1 = Cube("/cube_1", sizes=1.0, positions=np.array([0.25, 0.25, 0.25]), scales=np.array([0.5, 0.5, 0.5]))
cube_1_material = OmniPbrMaterial("/World/Materials/cube_1")
cube_1_material.set_input_values("diffuse_color_constant", [1.0, 0.0, 0.0])
cube_1.apply_visual_materials(cube_1_material)

cube_2 = Cube("/cube_2", sizes=1.0, positions=np.array([-1.0, -1.0, 0.25]), scales=np.array([1.0, 1.0, 1.0]))
cube_2_material = OmniPbrMaterial("/World/Materials/cube_2")
cube_2_material.set_input_values("diffuse_color_constant", [0.0, 0.0, 1.0])
cube_2.apply_visual_materials(cube_2_material)

cone = Cone("/cone", radii=0.5, heights=1.0, positions=np.array([-0.1, -0.3, 0.2]), scales=np.array([1.0, 1.0, 1.0]))
cone_material = OmniPbrMaterial("/World/Materials/cone")
cone_material.set_input_values("diffuse_color_constant", [0.0, 1.0, 0.0])
cone.apply_visual_materials(cone_material)

assets_root_path = get_assets_root_path()
omni.kit.commands.execute(
    "CreateReferenceCommand",
    usd_context=omni.usd.get_context(),
    path_to="/World/black_grid",
    asset_path=assets_root_path + "/Isaac/Environments/Grid/gridroom_black.usd",
    instanceable=False,
)

# =============================================================================
# CREATE CAMERA
# =============================================================================
# Camera position/orientation matches the deprecated example.
# The deprecated Camera class uses euler [0,0,180] which maps to prim
# quaternion (0.5, 0.5, 0.5, 0.5) via its internal coordinate transform.
# For RtxCamera (which writes directly to prim), euler [90,90,0] produces
# the equivalent orientation.

cam = RtxCamera(
    "/World/camera",
    translations=np.array([3.0, 0.0, 0.6]),
    orientations=euler_angles_to_quaternion(np.array([90, 90, 0]), degrees=True, extrinsic=False).numpy(),
)

cam.camera.set_focal_lengths(1.814756)
cam.camera.set_focus_distances(400.0)

print(f"Created depth camera at {cam.paths[0]}")

# =============================================================================
# CREATE DEPTH SENSOR
# =============================================================================

sensor = SingleViewDepthCameraSensor(
    cam,
    resolution=(1080, 1920),
    annotators=["depth_sensor_distance", "distance_to_image_plane"],
)

# Same depth sensor parameters as the deprecated example
sensor.set_sensor_baseline(55.0)
sensor.set_sensor_focal_length(891.0)
sensor.set_sensor_size(1280.0)
sensor.set_sensor_maximum_disparity(110.0)
sensor.set_sensor_disparity_confidence(0.99)
sensor.set_sensor_distance_cutoffs(minimum_distance=0.5, maximum_distance=9999.9)
sensor.set_sensor_noise_parameters(noise_mean=0.5, noise_sigma=1.0)
sensor.set_enabled_post_processing(True)

print(f"Depth sensor configured:")
print(f"  Baseline: {sensor.get_sensor_baseline()} mm")
print(f"  Focal length: {sensor.get_sensor_focal_length()} px")
print(f"  Disparity range: 0 - {sensor.get_sensor_maximum_disparity()} px")
print(f"  Distance cutoffs: {sensor.get_sensor_distance_cutoffs()}")

# =============================================================================
# RUN SIMULATION AND RETRIEVE DEPTH DATA
# =============================================================================

if args.test:
    stage = omni.usd.get_context().get_stage()
    stage.Export(os.path.join(output_dir, "stage.usda"))

timeline = omni.timeline.get_timeline_interface()
timeline.play()

frame_count = 0
printed = False

while simulation_app.is_running():
    simulation_app.update()
    frame_count += 1

    depth_data, _ = sensor.get_data("depth_sensor_distance")

    if depth_data is not None and not printed:
        printed = True
        depth_np = depth_data.numpy()
        print(f"\nFrame {frame_count}:")
        print(f"  Depth shape: {depth_data.shape}")
        print(f"  Depth range: [{depth_np.min():.2f}, {depth_np.max():.2f}] meters")
        valid = np.isfinite(depth_np) & (depth_np > 0)
        print(f"  Valid samples: {valid.sum()} / {depth_np.size} ({100*valid.mean():.1f}%)")

    if args.test and frame_count >= 10:
        break

# Save depth images
from isaacsim.core.utils.extensions import enable_extension

enable_extension("isaacsim.test.utils")
from isaacsim.test.utils import save_depth_image

depth_data, _ = sensor.get_data("depth_sensor_distance")
if depth_data is not None:
    save_depth_image(depth_data.numpy(), output_dir, "depth_sensor_distance.png", normalize=True)

dist_data, _ = sensor.get_data("distance_to_image_plane")
if dist_data is not None:
    save_depth_image(dist_data.numpy(), output_dir, "distance_to_image_plane.png", normalize=True)

timeline.stop()
simulation_app.close()
