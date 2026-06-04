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

"""Configure a camera with OpenCV fisheye (equidistant) distortion model.

This example demonstrates how to:
- Apply the ``OmniLensDistortionOpenCvFisheyeAPI`` schema to a camera prim
- Set fisheye intrinsic parameters (cx, cy, fx, fy) and distortion coefficients (k1-k4)
- Render a distorted image using ``CameraSensor``
"""

import argparse

from isaacsim import SimulationApp

parser = argparse.ArgumentParser(description="Camera with OpenCV fisheye distortion.")
parser.add_argument("--test", default=False, action="store_true", help="Run in test mode.")
args, _ = parser.parse_known_args()

simulation_app = SimulationApp({"headless": True})

import os

import cv2
import numpy as np
import omni
import omni.usd
from isaacsim.core.experimental.materials import OmniPbrMaterial
from isaacsim.core.experimental.objects import Cube, DomeLight, GroundPlane
from isaacsim.core.experimental.prims import GeomPrim, RigidPrim
from isaacsim.core.experimental.utils.transform import euler_angles_to_quaternion
from isaacsim.sensors.experimental.rtx import CameraSensor, RtxCamera
from pxr import Gf

output_dir = os.path.join(os.getcwd(), "_example_output_isaacsim.sensors.experimental.rtx", "camera_opencv_fisheye")
os.makedirs(output_dir, exist_ok=True)

# Same parameters as the deprecated isaacsim.sensors.camera example for consistency.
WIDTH, HEIGHT = 1920, 1200
camera_matrix = [[455.8, 0.0, 943.8], [0.0, 454.7, 602.3], [0.0, 0.0, 1.0]]
distortion_coefficients = [0.05, 0.01, -0.003, -0.0005]

(fx, _, cx), (_, fy, cy), (_, _, _) = camera_matrix

# =============================================================================
# CREATE SCENE
# =============================================================================

dome_light = DomeLight("/World/DomeLight")
dome_light.set_intensities(500)
GroundPlane("/World/defaultGroundPlane", sizes=100.0)

cube_positions = [np.array([0, 0, 0.5]), np.array([2, 0, 0.5]), np.array([0, 4, 1])]
cube_scale = [1.0, 1.0, 2.0]
cube_colors = [np.array([255, 0, 0]), np.array([0, 255, 0]), np.array([0, 0, 255])]

for i, (position, scale, color) in enumerate(zip(cube_positions, cube_scale, cube_colors)):
    cube_path = f"/new_cube_{i}"
    cube = Cube(cube_path, sizes=1.0, positions=position, scales=np.array([scale, scale, scale]))
    GeomPrim(cube_path, apply_collision_apis=True)
    RigidPrim(cube_path)
    cube_material = OmniPbrMaterial(f"/World/Materials/cube_{i}")
    cube_material.set_input_values("diffuse_color_constant", (color / 255.0).tolist())
    cube.apply_visual_materials(cube_material)

# =============================================================================
# CREATE CAMERA WITH FISHEYE DISTORTION
# =============================================================================
# The OmniLensDistortionOpenCvFisheyeAPI schema adds fisheye distortion
# attributes to the camera prim. Uses the same intrinsics and distortion
# coefficients as the deprecated example.

cam = RtxCamera(
    "/World/camera",
    schemas=["OmniLensDistortionOpenCvFisheyeAPI"],
    attributes={
        "omni:lensdistortion:opencvFisheye:cx": cx,
        "omni:lensdistortion:opencvFisheye:cy": cy,
        "omni:lensdistortion:opencvFisheye:fx": fx,
        "omni:lensdistortion:opencvFisheye:fy": fy,
        "omni:lensdistortion:opencvFisheye:k1": distortion_coefficients[0],
        "omni:lensdistortion:opencvFisheye:k2": distortion_coefficients[1],
        "omni:lensdistortion:opencvFisheye:k3": distortion_coefficients[2],
        "omni:lensdistortion:opencvFisheye:k4": distortion_coefficients[3],
        "omni:lensdistortion:opencvFisheye:imageSize": Gf.Vec2i(WIDTH, HEIGHT),
    },
    translations=np.array([0.0, 0.0, 3.5]),
    orientations=euler_angles_to_quaternion(np.array([0, 0, -90]), degrees=True, extrinsic=False).numpy(),
)

cam.prims[0].GetAttribute("omni:lensdistortion:model").Set("opencvFisheye")

print(f"Created camera with OpenCV fisheye distortion at {cam.paths[0]}")

# =============================================================================
# CREATE SENSOR AND RENDER
# =============================================================================

sensor = CameraSensor(cam, resolution=(HEIGHT, WIDTH), annotators=["rgb"])

timeline = omni.timeline.get_timeline_interface()

if args.test:
    stage = omni.usd.get_context().get_stage()
    stage.Export(os.path.join(output_dir, "stage.usda"))

timeline.play()

for i in range(10):
    simulation_app.update()

data, _ = sensor.get_data("rgb")
if data is not None:
    rgb_np = data.numpy()
    print(f"  RGB shape: {rgb_np.shape}, dtype: {rgb_np.dtype}")
    print(f"  Fisheye distortion is applied in the rendered image")
    img = cv2.cvtColor(rgb_np, cv2.COLOR_RGB2BGR)
    image_path = os.path.join(output_dir, "camera_opencv_fisheye.png")
    print(f"Saving the rendered image to: {image_path}")
    cv2.imwrite(image_path, img)

timeline.stop()
simulation_app.close()
