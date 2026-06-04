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

"""Configure a camera from ROS camera_info parameters.

This example demonstrates how to:
- Parse a ROS ``camera_info`` YAML-style parameter set
- Map ROS intrinsic matrix (K), distortion coefficients (D), and image size
  to ``RtxCamera`` schemas and attributes
- Create a camera matching a real-world ROS camera calibration
"""

import argparse

from isaacsim import SimulationApp

parser = argparse.ArgumentParser(description="Configure camera from ROS camera_info.")
parser.add_argument("--test", default=False, action="store_true", help="Run in test mode.")
args, _ = parser.parse_known_args()

simulation_app = SimulationApp({"headless": True})

import os

import numpy as np
import omni
import omni.usd
from isaacsim.core.experimental.objects import Cube, DomeLight, GroundPlane
from isaacsim.core.experimental.prims import GeomPrim, RigidPrim
from isaacsim.core.experimental.utils.transform import euler_angles_to_quaternion
from isaacsim.sensors.experimental.rtx import CameraSensor, RtxCamera
from PIL import Image
from pxr import Gf

output_dir = os.path.join(os.getcwd(), "_example_output_isaacsim.sensors.experimental.rtx", "camera_ros")
os.makedirs(output_dir, exist_ok=True)

# =============================================================================
# ROS CAMERA_INFO PARAMETERS
# =============================================================================
# Same parameters as the deprecated isaacsim.sensors.camera example for consistency.
# Based on Intel RealSense D435i camera_info topic.

IMAGE_WIDTH = 640
IMAGE_HEIGHT = 480

# Intrinsic matrix K (3x3, row-major)
K = [612.4178466796875, 0.0, 309.72296142578125, 0.0, 612.362060546875, 245.35870361328125, 0.0, 0.0, 1.0]
fx, fy = K[0], K[4]
cx, cy = K[2], K[5]

# Distortion model and coefficients (rational_polynomial with zero distortion)
DISTORTION_MODEL = "rational_polynomial"
D = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

# =============================================================================
# CREATE SCENE
# =============================================================================

dome_light = DomeLight("/World/DomeLight")
dome_light.set_intensities(500)
GroundPlane("/World/defaultGroundPlane", sizes=100.0)

cube_1 = Cube("/new_cube_1", sizes=1.0, positions=np.array([0, 0, 0.5]), scales=np.array([1.0, 1.0, 1.0]))
GeomPrim("/new_cube_1", apply_collision_apis=True)
RigidPrim("/new_cube_1")

# =============================================================================
# CREATE CAMERA FROM ROS PARAMETERS
# =============================================================================
# Map ROS distortion_model to the appropriate lens distortion schema.
# - "plumb_bob" / "rational_polynomial" -> OmniLensDistortionOpenCvPinholeAPI
# - "equidistant" -> OmniLensDistortionOpenCvFisheyeAPI

if DISTORTION_MODEL in ("plumb_bob", "rational_polynomial"):
    schema = "OmniLensDistortionOpenCvPinholeAPI"
    prefix = "omni:lensdistortion:opencvPinhole"
    model_name = "opencvPinhole"
    # Map D coefficients to attribute names
    coeff_names = ["k1", "k2", "p1", "p2", "k3", "k4", "k5", "k6", "s1", "s2", "s3", "s4"]
    distortion_attrs = {f"{prefix}:{coeff_names[i]}": D[i] for i in range(len(D))}
elif DISTORTION_MODEL == "equidistant":
    schema = "OmniLensDistortionOpenCvFisheyeAPI"
    prefix = "omni:lensdistortion:opencvFisheye"
    model_name = "opencvFisheye"
    coeff_names = ["k1", "k2", "k3", "k4"]
    distortion_attrs = {f"{prefix}:{coeff_names[i]}": D[i] for i in range(len(D))}
else:
    raise ValueError(f"Unsupported distortion model: {DISTORTION_MODEL}")

# Build the full attribute dict
attributes = {
    f"{prefix}:cx": cx,
    f"{prefix}:cy": cy,
    f"{prefix}:fx": fx,
    f"{prefix}:fy": fy,
    f"{prefix}:imageSize": Gf.Vec2i(IMAGE_WIDTH, IMAGE_HEIGHT),
    **distortion_attrs,
}

cam = RtxCamera(
    "/World/camera",
    schemas=[schema],
    attributes=attributes,
    translations=np.array([0.0, 0.0, 3.0]),
    orientations=euler_angles_to_quaternion(np.array([0, 0, -90]), degrees=True, extrinsic=False).numpy(),
)

# Set the distortion model selector
cam.prims[0].GetAttribute("omni:lensdistortion:model").Set(model_name)

print(f"Created camera from ROS camera_info:")
print(f"  Resolution: {IMAGE_WIDTH}x{IMAGE_HEIGHT}")
print(f"  Intrinsics: fx={fx}, fy={fy}, cx={cx}, cy={cy}")
print(f"  Distortion model: {DISTORTION_MODEL} -> {schema}")
print(f"  D: {D}")

# =============================================================================
# CREATE SENSOR AND RENDER
# =============================================================================

sensor = CameraSensor(cam, resolution=(IMAGE_HEIGHT, IMAGE_WIDTH), annotators=["rgb"])

timeline = omni.timeline.get_timeline_interface()

if args.test:
    stage = omni.usd.get_context().get_stage()
    stage.Export(os.path.join(output_dir, "stage.usda"))

timeline.play()

for i in range(100):
    simulation_app.update()

data, _ = sensor.get_data("rgb")
if data is not None:
    rgb_np = data.numpy()
    print(f"\n  RGB shape: {rgb_np.shape}")
    img = Image.fromarray(rgb_np)
    image_path = os.path.join(output_dir, "camera_ros.png")
    print(f"Saving the rendered image to: {image_path}")
    img.save(image_path)

timeline.stop()
simulation_app.close()
