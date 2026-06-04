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

"""Demonstrate stereoscopic depth camera setup."""

import argparse
import os

# Parse command line arguments
parser = argparse.ArgumentParser()
parser.add_argument("--test", default=False, action="store_true", help="Run in test mode")
args, unknown = parser.parse_known_args()

from isaacsim import SimulationApp

simulation_app = SimulationApp({"headless": args.test})

output_dir = os.path.join(os.getcwd(), "_example_output_isaacsim.sensors.camera", "camera_stereoscopic_depth")
os.makedirs(output_dir, exist_ok=True)

import numpy as np
import omni
from isaacsim.core.experimental.materials import OmniPbrMaterial
from isaacsim.core.experimental.objects import Cone, Cube
from isaacsim.core.experimental.utils.transform import euler_angles_to_quaternion
from isaacsim.sensors.camera import SingleViewDepthSensor
from isaacsim.storage.native.nucleus import get_assets_root_path

# Add two cubes and a cone
cube_1 = Cube(
    "/cube_1",
    sizes=1.0,
    positions=np.array([0.25, 0.25, 0.25]),
    scales=np.array([0.5, 0.5, 0.5]),
)
cube_1_material = OmniPbrMaterial("/World/Materials/cube_1")
cube_1_material.set_input_values("diffuse_color_constant", [1.0, 0.0, 0.0])
cube_1.apply_visual_materials(cube_1_material)

cube_2 = Cube(
    "/cube_2",
    sizes=1.0,
    positions=np.array([-1.0, -1.0, 0.25]),
    scales=np.array([1.0, 1.0, 1.0]),
)
cube_2_material = OmniPbrMaterial("/World/Materials/cube_2")
cube_2_material.set_input_values("diffuse_color_constant", [0.0, 0.0, 1.0])
cube_2.apply_visual_materials(cube_2_material)

cone = Cone(
    "/cone",
    radii=0.5,
    heights=1.0,
    positions=np.array([-0.1, -0.3, 0.2]),
    scales=np.array([1.0, 1.0, 1.0]),
)
cone_material = OmniPbrMaterial("/World/Materials/cone")
cone_material.set_input_values("diffuse_color_constant", [0.0, 1.0, 0.0])
cone.apply_visual_materials(cone_material)

# Add a stereoscopic camera
camera = SingleViewDepthSensor(
    prim_path="/World/camera",
    name="depth_camera",
    position=np.array([3.0, 0.0, 0.6]),
    orientation=euler_angles_to_quaternion(np.array([0, 0, 180]), degrees=True, extrinsic=False).numpy(),
    frequency=20,
    resolution=(1920, 1080),
)

# Initialize the black grid scene for the background
assets_root_path = get_assets_root_path()
path_to = omni.kit.commands.execute(
    "CreateReferenceCommand",
    usd_context=omni.usd.get_context(),
    path_to="/World/black_grid",
    asset_path=assets_root_path + "/Isaac/Environments/Grid/gridroom_black.usd",
    instanceable=False,
)

if args.test:
    stage = omni.usd.get_context().get_stage()
    stage.Export(os.path.join(output_dir, "stage.usda"))

# Start the timeline and initialize the camera
timeline = omni.timeline.get_timeline_interface()
timeline.play()
timeline.commit()

# Initialize the camera, applying the appropriate schemas to the render product to enable depth sensing
camera.initialize(attach_rgb_annotator=False)

# Now that the camera is initialized, we can configure its parameters
# First, camera lens parameters
camera.set_focal_length(1.814756)
camera.set_focus_distance(400.0)
# Next, depth sensor parameters
camera.set_baseline_mm(55)
camera.set_focal_length_pixel(891.0)
camera.set_sensor_size_pixel(1280.0)
camera.set_max_disparity_pixel(110.0)
camera.set_confidence_threshold(0.99)
camera.set_noise_mean(0.5)
camera.set_noise_sigma(1.0)
camera.set_noise_downscale_factor_pixel(1.0)
camera.set_min_distance(0.5)
camera.set_max_distance(9999.9)

# Attach the DepthSensorDistance annotator and DistanceToImagePlane annotator to the camera
camera.attach_annotator("DepthSensorDistance")
camera.attach_annotator("distance_to_image_plane")

# Run for 10 frames in test mode
i = 0
while simulation_app.is_running() and (not args.test or i < 10):
    simulation_app.update()
    i += 1

# Saved the rendered frames as PNGs
from isaacsim.core.utils.extensions import enable_extension

enable_extension("isaacsim.test.utils")
from isaacsim.test.utils import save_depth_image

latest_frame = camera.get_current_frame()
save_depth_image(latest_frame["DepthSensorDistance"], output_dir, "depth_sensor_distance.png", normalize=True)
save_depth_image(latest_frame["distance_to_image_plane"], output_dir, "distance_to_image_plane.png", normalize=True)

simulation_app.close()
