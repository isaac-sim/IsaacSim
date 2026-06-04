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

"""Demonstrate basic camera sensor setup and usage."""

from isaacsim import SimulationApp

simulation_app = SimulationApp({"headless": False})

import argparse

parser = argparse.ArgumentParser()
parser.add_argument("--disable-output", action="store_true", help="Disable debug output.")
parser.add_argument("--test", action="store_true", help="Enable test mode (fixed frame count).")
args, _ = parser.parse_known_args()

import os

import matplotlib.pyplot as plt
import numpy as np
import omni.timeline
import omni.usd
from isaacsim.core.experimental.materials import OmniPbrMaterial
from isaacsim.core.experimental.objects import Cube, DomeLight, GroundPlane
from isaacsim.core.experimental.prims import GeomPrim, RigidPrim
from isaacsim.core.experimental.utils.transform import euler_angles_to_quaternion
from isaacsim.sensors.camera import Camera

output_dir = os.path.join(os.getcwd(), "_example_output_isaacsim.sensors.camera", "camera")
os.makedirs(output_dir, exist_ok=True)

cube_2 = Cube(
    "/new_cube_2",
    sizes=1.0,
    positions=np.array([5.0, 3, 1.0]),
    scales=np.array([0.6, 0.5, 0.2]),
)
GeomPrim("/new_cube_2", apply_collision_apis=True)
cube_2_rigid = RigidPrim("/new_cube_2")
cube_2_material = OmniPbrMaterial("/World/Materials/cube_2")
cube_2_material.set_input_values("diffuse_color_constant", [1.0, 0.0, 0.0])
cube_2_rigid.apply_visual_materials(cube_2_material)

cube_3 = Cube(
    "/new_cube_3",
    sizes=1.0,
    positions=np.array([-5, 1, 3.0]),
    scales=np.array([0.1, 0.1, 0.1]),
)
GeomPrim("/new_cube_3", apply_collision_apis=True)
cube_3_rigid = RigidPrim("/new_cube_3")
cube_3_material = OmniPbrMaterial("/World/Materials/cube_3")
cube_3_material.set_input_values("diffuse_color_constant", [0.0, 0.0, 1.0])
cube_3_rigid.apply_visual_materials(cube_3_material)
cube_3_rigid.set_velocities(linear_velocities=np.array([[0, 0, 0.4]], dtype=np.float32))

camera = Camera(
    prim_path="/World/camera",
    position=np.array([0.0, 0.0, 25.0]),
    frequency=20,
    resolution=(256, 256),
    orientation=euler_angles_to_quaternion(np.array([0, 90, 0]), degrees=True, extrinsic=False).numpy(),
)

# Create ground plane and dome light
dome_light = DomeLight("/World/DomeLight")
dome_light.set_intensities(500)
GroundPlane("/World/defaultGroundPlane", sizes=100.0)

# Start the timeline and initialize the camera
timeline = omni.timeline.get_timeline_interface()

if args.test:
    stage = omni.usd.get_context().get_stage()
    stage.Export(os.path.join(output_dir, "stage.usda"))

timeline.play()
timeline.commit()
camera.initialize()

i = 0
camera.add_motion_vectors_to_frame()
while simulation_app.is_running() and (not args.test or i < 10):
    simulation_app.update()
    if not timeline.is_playing():
        continue
    if not args.disable_output:
        print("Frame data:")
        for key, value in camera.get_current_frame().items():
            if value is None:
                print(f"  - {key}: None")
            elif key in ["rgb", "motion_vectors"]:
                print(f"  - {key} data shape: {value.shape}")
            else:
                print(f"  - {key}: {value}")
    if (i + 1) % 100 == 0:
        cube_3_positions, _ = cube_3_rigid.get_world_poses()
        cube_2_positions, _ = cube_2_rigid.get_world_poses()
        points_2d = camera.get_image_coords_from_world_points(
            np.array([cube_3_positions.numpy()[0], cube_2_positions.numpy()[0]])
        )
        points_3d = camera.get_world_points_from_image_coords(points_2d, np.array([24.94, 24.9]))
        imgplot = plt.imshow(camera.get_rgba()[:, :, :3])
        output_path = os.path.join(output_dir, f"camera.frame{i:03d}.png")
        print(f"Saving image to: {output_path}")
        plt.draw()
        plt.savefig(output_path)
    i += 1


simulation_app.close()
