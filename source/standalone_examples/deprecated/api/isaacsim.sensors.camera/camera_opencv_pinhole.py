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

"""Demonstrate camera setup with OpenCV pinhole model."""

import argparse

from isaacsim import SimulationApp

parser = argparse.ArgumentParser(description="Camera OpenCV pinhole example.")
parser.add_argument("--test", default=False, action="store_true", help="Run in test mode.")
args, _ = parser.parse_known_args()

simulation_app = SimulationApp({"headless": True})

import os

import cv2
import numpy as np
import omni.timeline
import omni.usd
from isaacsim.core.experimental.materials import OmniPbrMaterial
from isaacsim.core.experimental.objects import Cube, DomeLight, GroundPlane
from isaacsim.core.experimental.prims import GeomPrim, RigidPrim
from isaacsim.core.experimental.utils.stage import get_current_stage
from isaacsim.core.experimental.utils.transform import euler_angles_to_quaternion
from isaacsim.sensors.camera import Camera
from scipy.spatial.transform import Rotation

output_dir = os.path.join(os.getcwd(), "_example_output_isaacsim.sensors.camera", "camera_opencv_pinhole")
os.makedirs(output_dir, exist_ok=True)

# Given the OpenCV camera matrix and distortion coefficients (Rational Polynomial model),
# creates a camera and a sample scene, renders an image and saves it to
# camera_opencv_pinhole.png file. The asset is also saved to camera_opencv_pinhole.usd file.
width, height = 1920, 1200
camera_matrix = [[958.8, 0.0, 957.8], [0.0, 956.7, 589.5], [0.0, 0.0, 1.0]]
distortion_coefficients = [0.14, -0.03, -0.0002, -0.00003, 0.009, 0.5, -0.07, 0.017]

# Camera sensor size and optical path parameters. These parameters are not the part of the
# OpenCV camera model, but they are nessesary to simulate the depth of field effect.
#
# Note: To disable the depth of field effect, set the f_stop to 0.0. This is useful for debugging.
# Set pixel size (microns)
pixel_size = 3
# Set f-number, the ratio of the lens focal length to the diameter of the entrance pupil (unitless)
f_stop = 1.8
# Set focus distance (meters) - chosen as distance from camera to cube
focus_distance = 3.0

# Create ground plane and dome light
dome_light = DomeLight("/World/DomeLight")
dome_light.set_intensities(500)
GroundPlane("/World/defaultGroundPlane", sizes=100.0)

# Define the position, scale and color of each cube, then add them to the stage
cube_positions = [
    np.array([0, 0, 0.5]),
    np.array([2, 0, 0.5]),
    np.array([0, 4, 1]),
]

cube_scale = [1.0, 1.0, 2.0]

cube_colors = [
    np.array([255, 0, 0]),
    np.array([0, 255, 0]),
    np.array([0, 0, 255]),
]

for i, (position, scale, color) in enumerate(zip(cube_positions, cube_scale, cube_colors)):
    cube_path = f"/new_cube_{i}"
    cube = Cube(
        cube_path,
        sizes=1.0,
        positions=position,
        scales=np.array([scale, scale, scale]),
    )
    GeomPrim(cube_path, apply_collision_apis=True)
    RigidPrim(cube_path)
    cube_material = OmniPbrMaterial(f"/World/Materials/cube_{i}")
    cube_material.set_input_values("diffuse_color_constant", (color / 255.0).tolist())
    cube.apply_visual_materials(cube_material)

# Define camera position and orientation, then add the camera to the stage
camera_position = np.array([0.0, 0.0, 4.5])
camera_rotation_as_euler = np.array([0, 90, 0])
camera = Camera(
    prim_path="/World/camera",
    position=camera_position,
    frequency=30,
    resolution=(width, height),
    orientation=euler_angles_to_quaternion(camera_rotation_as_euler, degrees=True, extrinsic=False).numpy(),
)

# Start the timeline and initialize the camera
timeline = omni.timeline.get_timeline_interface()

if args.test:
    stage = omni.usd.get_context().get_stage()
    stage.Export(os.path.join(output_dir, "stage.usda"))

timeline.play()
timeline.commit()
camera.initialize()

# Calculate the focal length and aperture size from the camera matrix
(fx, _, cx), (_, fy, cy), (_, _, _) = camera_matrix  # fx, fy are in pixels, cx, cy are in pixels
horizontal_aperture = pixel_size * width * 1e-6  # convert to meters
vertical_aperture = pixel_size * height * 1e-6  # convert to meters
focal_length_x = pixel_size * fx * 1e-6  # convert to meters
focal_length_y = pixel_size * fy * 1e-6  # convert to meters
focal_length = (focal_length_x + focal_length_y) / 2  # convert to meters

# Set the camera parameters, note the unit conversion between Isaac Sim sensor and Kit
camera.set_focal_length(focal_length)
camera.set_focus_distance(focus_distance)
camera.set_lens_aperture(f_stop)
camera.set_horizontal_aperture(horizontal_aperture)
camera.set_vertical_aperture(vertical_aperture)

camera.set_clipping_range(0.05, 1.0e5)

# Set the distortion coefficients
camera.set_opencv_pinhole_properties(cx=cx, cy=cy, fx=fx, fy=fy, pinhole=distortion_coefficients)

# Render 10 frames, then load the rendered image into a CV2-compatible format
for i in range(10):
    simulation_app.update()
img = cv2.cvtColor(camera.get_rgb().astype(np.uint8), cv2.COLOR_RGB2BGR)

# Plot cube corners on the rendered image. Code adapted from snippet provided by forum user @ericpedley.
# Resolve the corners of each cube in world space
cube_corners = np.array(
    [
        [0.5, 0.5, 0.5],
        [-0.5, 0.5, 0.5],
        [0.5, -0.5, 0.5],
        [-0.5, -0.5, 0.5],
        [0.5, 0.5, -0.5],
        [-0.5, 0.5, -0.5],
        [0.5, -0.5, -0.5],
        [-0.5, -0.5, -0.5],
    ],
    dtype=np.float64,
)

cube_corners_world = []
for position, scale in zip(cube_positions, cube_scale):
    cube_corners_world.append(cube_corners * scale + position)
object_points_world = np.vstack(cube_corners_world)

# Transform from Isaac Sim (Y-forward, Z-up) to OpenCV (Z-forward, Y-down) coordinates
isaac_to_cv2_mat = np.array([[0, -1, 0], [0, 0, -1], [1, 0, 0]], dtype=np.float64)
isaac_to_cv2 = Rotation.from_matrix(isaac_to_cv2_mat)

# Convert camera rotation to OpenCV coordinate system
cam_rotation = Rotation.from_euler("xyz", camera_rotation_as_euler, degrees=True)
cam_rotation_cv2 = isaac_to_cv2 * cam_rotation.inv() * isaac_to_cv2.inv()

# Transform object points to OpenCV coordinates for projection
object_points_cv2 = np.expand_dims(object_points_world @ isaac_to_cv2_mat.T, axis=1)

# Camera rotation vector in OpenCV coordinates
rvec_cv2 = cam_rotation_cv2.as_rotvec()

# Translation vector (world origin to camera) in OpenCV coordinates
tvec_cv2 = -cam_rotation_cv2.apply(camera_position @ isaac_to_cv2_mat.T)

# Camera intrinsic matrix
K = np.array(camera_matrix, dtype=np.float64)

# Distortion coefficients (rational polynomial model)
D = np.array(distortion_coefficients, dtype=np.float64)

# Project 3D points to 2D image coordinates
image_points, _ = cv2.projectPoints(object_points_cv2, rvec_cv2, tvec_cv2, K, D)

# Draw the projected corners of each cube on the rendered image
for i, pt in enumerate(image_points):
    # Map cube color from RGB to BGR
    cube_color = cube_colors[i // 8].astype(np.uint8)[::-1]
    color = tuple(cube_color.tolist())
    # Skip points outside the camera field of view
    if np.any(pt[0] < 0):
        continue
    # Draw the point as a filled circle with yellow border for contrast
    cv2.circle(img, tuple(pt[0].astype(int)), 5, (0, 255, 255), -1)
    cv2.circle(img, tuple(pt[0].astype(int)), 3, color, -1)

image_path = os.path.join(output_dir, "camera_opencv_pinhole.png")
print(f"Saving the rendered image to: {image_path}")
cv2.imwrite(image_path, img)

usd_path = os.path.join(output_dir, "camera_opencv_pinhole.usd")
print(f"Saving the asset to: {usd_path}")
stage = get_current_stage()
stage.Export(usd_path)

simulation_app.close()
