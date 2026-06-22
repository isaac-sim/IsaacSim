# SPDX-FileCopyrightText: Copyright (c) 2021-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

"""Create a camera from ROS camera_info topic parameters and render an image."""

# Given a printout of ROS topic, containing the intrinsic and extrinsic parameters of the camera,
# creates a camera and a sample scene, renders an image and saves it to camera_ros.png file.
# The asset is also saved to camera_ros.usd file. The camera model is based on Intel RealSense D435i.

import argparse

from isaacsim import SimulationApp

parser = argparse.ArgumentParser(description="Camera ROS example.")
parser.add_argument("--test", default=False, action="store_true", help="Run in test mode.")
args, _ = parser.parse_known_args()

simulation_app = SimulationApp({"headless": True})

import math
import os

import numpy as np
import omni.timeline
import omni.usd
import yaml
from isaacsim.core.experimental.objects import Cube, DomeLight, GroundPlane
from isaacsim.core.experimental.prims import GeomPrim, RigidPrim
from isaacsim.core.experimental.utils.stage import get_current_stage
from isaacsim.core.experimental.utils.transform import euler_angles_to_quaternion
from isaacsim.sensors.camera import Camera
from PIL import Image, ImageDraw

output_dir = os.path.join(os.getcwd(), "_example_output_isaacsim.sensors.camera", "camera_ros")
os.makedirs(output_dir, exist_ok=True)

# To create a model of a given ROS camera, print the camera_info topic with:
#       rostopicecho /camera/color/camera_info
# And copy the output into the yaml_data variable below. Populate additional parameters using the sensor manual.
#
# Note: only rational_polynomial model is supported in this example. For plump_bob or pinhole
# models set the distortion_model to "rational_polynomial" and compliment array D with 0.0 to 8 elements
# The camera_info topic in the Isaac Sim ROS bridge will be in the rational_polynomial format.
#
# Note: when fx is not equal to fy (pixels are not square), the average of fx and fy is used as the focal length.
# and the intrinsic matrix is adjusted to have square pixels. This updated matrix is used for rendering and
# it is also populated into the camera_info topic in the Isaac Sim ROS bridge.

yaml_data = """
# rostopic echo /camera/color/camera_info
header:
  seq: 211
  stamp:
    secs: 1694379352
    nsecs: 176209771
  frame_id: "camera_color_optical_frame"
height: 480
width: 640
distortion_model: "rational_polynomial"
D: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
K: [612.4178466796875, 0.0, 309.72296142578125, 0.0, 612.362060546875, 245.35870361328125, 0.0, 0.0, 1.0]
R: [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
P: [612.4178466796875, 0.0, 309.72296142578125, 0.0, 0.0, 612.362060546875, 245.35870361328125, 0.0, 0.0, 0.0, 1.0, 0.0]
"""

# Camera sensor size and optical path parameters. These parameters are not the part of the
# OpenCV camera model, but they are nessesary to simulate the depth of field effect.
#
# To disable the depth of field effect, set the f_stop to 0.0. This is useful for debugging.
pixel_size = 1.4  # Pixel size in microns, 3 microns is common
f_stop = 2.0  # F-number, the ratio of the lens focal length to the diameter of the entrance pupil
focus_distance = 0.5  # Focus distance in meters, the distance from the camera to the object plane


# Parsing the YAML data
data = yaml.safe_load(yaml_data)
print("Header Frame ID:", data["header"]["frame_id"])
width, height, K, D = data["width"], data["height"], data["K"], data["D"]

# Create ground plane and dome light
dome_light = DomeLight("/World/DomeLight")
dome_light.set_intensities(500)
GroundPlane("/World/defaultGroundPlane", sizes=100.0)

cube_1 = Cube(
    "/new_cube_1",
    sizes=1.0,
    positions=np.array([0, 0, 0.5]),
    scales=np.array([1.0, 1.0, 1.0]),
)
GeomPrim("/new_cube_1", apply_collision_apis=True)
RigidPrim("/new_cube_1")

camera = Camera(
    prim_path="/World/camera",
    position=np.array([0.0, 0.0, 3.0]),  # 2 meter away from the side of the cube
    frequency=30,
    resolution=(width, height),
    orientation=euler_angles_to_quaternion(np.array([0, 90, 0]), degrees=True, extrinsic=False).numpy(),
)
# Start the timeline and initialize the camera
timeline = omni.timeline.get_timeline_interface()

if args.test:
    stage_for_export = omni.usd.get_context().get_stage()
    stage_for_export.Export(os.path.join(output_dir, "stage.usda"))

timeline.play()
timeline.commit()
camera.initialize()

# Calculate the focal length and aperture size from the camera matrix
fx, _, cx, _, fy, cy, _, _, _ = K
horizontal_aperture = pixel_size * 1e-3 * width
vertical_aperture = pixel_size * 1e-3 * height
focal_length_x = fx * pixel_size * 1e-3
focal_length_y = fy * pixel_size * 1e-3
focal_length = (focal_length_x + focal_length_y) / 2  # in mm

# Set the camera parameters, note the unit conversion between Isaac Sim sensor and Kit
camera.set_focal_length(focal_length / 10.0)
camera.set_focus_distance(focus_distance)
camera.set_lens_aperture(f_stop * 100.0)
camera.set_horizontal_aperture(horizontal_aperture / 10.0)
camera.set_vertical_aperture(vertical_aperture / 10.0)
camera.set_clipping_range(0.05, 1.0e5)

# Set the distortion coefficients, this is nessesary, when cx, cy are not in the center of the image
diagonal = 2 * math.sqrt(max(cx, width - cx) ** 2 + max(cy, height - cy) ** 2)
diagonal_fov = 2 * math.atan2(diagonal, fx + fy) * 180 / math.pi
camera.set_projection_type("fisheyePolynomial")
camera.set_rational_polynomial_properties(width, height, cx, cy, diagonal_fov, D)

# Get the rendered frame and save it to a file
for i in range(100):
    simulation_app.update()
camera.get_current_frame()
img = Image.fromarray(camera.get_rgba()[:, :, :3])


# Optional step, draw the 3D points to the image plane using the OpenCV fisheye model
def draw_points_opencv(points3d: np.ndarray) -> None:
    """Draw 3D points projected onto the image plane using OpenCV."""
    try:
        # To install, run python.sh -m pip install opencv-python
        import cv2

        rvecs, tvecs = np.array([0.0, 0.0, 0.0]), np.array([0.0, 0.0, 0.0])
        points, jac = cv2.projectPoints(
            np.expand_dims(points3d, 1), rvecs, tvecs, np.array(K).reshape(3, 3), np.array(D)
        )
        draw = ImageDraw.Draw(img)
        for pt in points:
            x, y = pt[0]
            print("Drawing point at: ", x, y)
            draw.ellipse((x - 4, y - 4, x + 4, y + 4), fill="orange", outline="orange")
    except ImportError:
        print("OpenCV is not installed, skipping OpenCV overlay")
        print("To install OpenCV, run: python.sh -m pip install opencv-python")


# Draw the 3D points to the image plane
draw_points_opencv(points3d=np.array([[0.5, 0.5, 4.0], [-0.5, 0.5, 4.0], [0.5, -0.5, 4.0], [-0.5, -0.5, 4.0]]))

image_path = os.path.join(output_dir, "camera_ros.png")
print(f"Saving the rendered image to: {image_path}")
img.save(image_path)

usd_path = os.path.join(output_dir, "camera_ros.usd")
print(f"Saving the asset to: {usd_path}")
stage = get_current_stage()
stage.Export(usd_path)

simulation_app.close()
