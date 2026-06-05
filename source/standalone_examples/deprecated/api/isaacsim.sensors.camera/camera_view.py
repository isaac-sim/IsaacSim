# SPDX-FileCopyrightText: Copyright (c) 2024-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

"""Demonstrate camera view for capturing multiple camera outputs."""

import argparse

parser = argparse.ArgumentParser()
parser.add_argument("--test", default=False, action="store_true", help="Run in test mode.")
args, unknown = parser.parse_known_args()

from isaacsim import SimulationApp

simulation_app = SimulationApp({"headless": False})

NUM_CAPTURES = 2
RESOLUTION = (256, 256)

import os

import numpy as np
import omni.replicator.core as rep
import omni.timeline
import omni.usd
from isaacsim.core.deprecation_manager import import_module
from isaacsim.core.experimental.materials import OmniPbrMaterial
from isaacsim.core.experimental.objects import Cube, DomeLight, GroundPlane
from isaacsim.sensors.camera import CameraView
from PIL import Image

torch = import_module("torch")

# Create visual cubes
cube_material = OmniPbrMaterial("/World/Materials/cube_blue")
cube_material.set_input_values("diffuse_color_constant", [0.0, 0.0, 1.0])
for i in range(2):
    cube = Cube(
        f"/new_cube_{i}",
        sizes=1.0,
        positions=np.array([0, i * 0.5, 0.2]),
        scales=np.array([0.1, 0.1, 0.1]),
    )
    cube.apply_visual_materials(cube_material)


# Create the cameras
camera_01 = rep.create.camera(position=(0, 0, 2), look_at=(0, 0, 0))
camera_02 = rep.create.camera(position=(0, 1, 2), look_at=(0, 0, 0))
camera_03 = rep.create.camera(position=(1, 0, 2), look_at=(0, 0, 0))
camera_04 = rep.create.camera(position=(1, 1, 2), look_at=(0, 0, 0))

# Create the camera view from the camera prims
camera_view = CameraView(
    name="camera_prim_view",
    camera_resolution=RESOLUTION,
    prim_paths_expr="/Replicator/Camera_Xform*/Camera",
    output_annotators=["rgb", "depth"],
)

# Create ground plane and dome light
dome_light = DomeLight("/World/DomeLight")
dome_light.set_intensities(500)
GroundPlane("/World/defaultGroundPlane", sizes=100.0)

# Create output directory for the test data as images
out_dir = os.path.join(os.getcwd(), "_example_output_isaacsim.sensors.camera", "camera_view")
print(f"out_dir: {out_dir}")
os.makedirs(out_dir, exist_ok=True)
os.makedirs(f"{out_dir}/tiled", exist_ok=True)
os.makedirs(f"{out_dir}/batched", exist_ok=True)

# Use pre-allocated arrays as output for the camera view (RGB and depth, numpy and torch, tiled and batched)
rgb_np_tiled_out = np.zeros((*camera_view.tiled_resolution, 3), dtype=np.uint8)
rgb_tiled_torch_out = torch.zeros((*camera_view.tiled_resolution, 3), device="cuda", dtype=torch.uint8)
batched_rgb_shape = (len(camera_view.prims), *camera_view.camera_resolution, 3)
rgb_batched_out = torch.zeros(batched_rgb_shape, device="cuda", dtype=torch.uint8)

depth_np_tiled_out = np.zeros((*camera_view.tiled_resolution, 1), dtype=np.float32)
depth_tiled_torch_out = torch.zeros(camera_view.tiled_resolution, device="cuda", dtype=torch.float32)
depth_batched_shape = (len(camera_view.prims), *camera_view.camera_resolution, 1)
depth_batched_out = torch.zeros(depth_batched_shape, device="cuda", dtype=torch.float32)

if args.test:
    stage = omni.usd.get_context().get_stage()
    stage.Export(os.path.join(out_dir, "stage.usda"))

# Start the timeline and run some warmup frames
timeline = omni.timeline.get_timeline_interface()
timeline.play()
timeline.commit()
for i in range(5):
    simulation_app.update()

# Capture the data for the required number of frames
for i in range(NUM_CAPTURES):
    print(f" ** Step {i} ** ")
    simulation_app.update()

    #### RGB
    print(f" ** Running RGB data tests:")

    ## Numpy
    print(f" ** Numpy:")
    rgb_tiled_np = camera_view.get_rgb_tiled(device="cpu")
    print(f"rgb_tiled_np.shape: {rgb_tiled_np.shape}, type: {type(rgb_tiled_np)}, dtype: {rgb_tiled_np.dtype}")
    rgb_tiled_np_uint8 = (rgb_tiled_np * 255).astype(np.uint8)
    print(
        f"rgb_tiled_uint8.shape: {rgb_tiled_np_uint8.shape}, type: {type(rgb_tiled_np_uint8)}, dtype: {rgb_tiled_np_uint8.dtype}"
    )
    rgb_tiled_img = Image.fromarray(rgb_tiled_np_uint8)
    rgb_tiled_img.save(f"{out_dir}/tiled/{str(i).zfill(3)}_rgb_tiled_np.png")

    # Using pre-allocated memory for out argument
    camera_view.get_rgb_tiled(out=rgb_np_tiled_out, device="cpu")
    print(
        f"rgb_np_tiled_out.shape: {rgb_np_tiled_out.shape}, type: {type(rgb_np_tiled_out)}, dtype: {rgb_np_tiled_out.dtype}"
    )
    rgb_np_tiled_out_uint8 = (rgb_np_tiled_out * 255).astype(np.uint8)
    print(
        f"rgb_np_tiled_out_uint8.shape: {rgb_np_tiled_out_uint8.shape}, type: {type(rgb_np_tiled_out_uint8)}, dtype: {rgb_np_tiled_out_uint8.dtype}"
    )
    rgb_np_tiled_out_img = Image.fromarray(rgb_np_tiled_out_uint8)
    rgb_np_tiled_out_img.save(f"{out_dir}/tiled/{str(i).zfill(3)}_rgb_tiled_np_out.png")

    ## Torch
    print(f" ** Torch:")
    rgb_tiled_torch = camera_view.get_rgb_tiled(device="cuda")
    print(
        f"rgb_tiled_torch.shape: {rgb_tiled_torch.shape}, type: {type(rgb_tiled_torch)}, dtype: {rgb_tiled_torch.dtype}"
    )
    rgb_tiled_torch_uint8 = (rgb_tiled_torch * 255).to(dtype=torch.uint8)
    print(
        f"rgb_tiled_torch_uint8.shape: {rgb_tiled_torch_uint8.shape}, type: {type(rgb_tiled_torch_uint8)}, dtype: {rgb_tiled_torch_uint8.dtype}"
    )
    rgb_tiled_torch_img = Image.fromarray(rgb_tiled_torch_uint8.cpu().numpy())
    rgb_tiled_torch_img.save(f"{out_dir}/tiled/{str(i).zfill(3)}_rgb_tiled_torch.png")

    # Using pre-allocated memory for out argument
    camera_view.get_rgb_tiled(out=rgb_tiled_torch_out, device="cuda")
    print(
        f"rgb_tiled_torch_out.shape: {rgb_tiled_torch_out.shape}, type: {type(rgb_tiled_torch_out)}, dtype: {rgb_tiled_torch_out.dtype}"
    )
    rgb_tiled_torch_out_uint8 = (rgb_tiled_torch_out * 255).to(dtype=torch.uint8)
    print(
        f"rgb_tiled_torch_out_uint8.shape: {rgb_tiled_torch_out_uint8.shape}, type: {type(rgb_tiled_torch_out_uint8)}, dtype: {rgb_tiled_torch_out_uint8.dtype}"
    )
    rgb_tiled_torch_out_img = Image.fromarray(rgb_tiled_torch_out_uint8.cpu().numpy())
    rgb_tiled_torch_out_img.save(f"{out_dir}/tiled/{str(i).zfill(3)}_rgb_tiled_torch_out.png")

    ## Batched
    print(f" ** Batched:")
    rgb_batched = camera_view.get_rgb()
    print(f"rgb_batched.shape: {rgb_batched.shape}, type: {type(rgb_batched)}, dtype: {rgb_batched.dtype}")
    for camera_id in range(rgb_batched.shape[0]):
        rgb_batched_uint8 = (rgb_batched[camera_id] * 255).to(dtype=torch.uint8)
        print(
            f"camera_id={camera_id}: rgb_batched_uint8.shape: {rgb_batched_uint8.shape}, type: {type(rgb_batched_uint8)}, dtype: {rgb_batched_uint8.dtype}"
        )
        rgb_batched_img = Image.fromarray(rgb_batched_uint8.cpu().numpy())
        rgb_batched_img.save(f"{out_dir}/batched/{str(i).zfill(3)}_rgb_batched_{camera_id}.png")

    # Using pre-allocated memory for out argument
    camera_view.get_rgb(out=rgb_batched_out)
    print(
        f"rgb_batched_out.shape: {rgb_batched_out.shape}, type: {type(rgb_batched_out)}, dtype: {rgb_batched_out.dtype}"
    )
    for camera_id in range(rgb_batched_out.shape[0]):
        rgb_batched_out_uint8 = (rgb_batched_out[camera_id] * 255).to(dtype=torch.uint8)
        print(
            f"camera_id={camera_id}: rgb_batched_out_uint8.shape: {rgb_batched_out_uint8.shape}, type: {type(rgb_batched_out_uint8)}, dtype: {rgb_batched_out_uint8.dtype}"
        )
        rgb_batched_out_img = Image.fromarray(rgb_batched_out_uint8.cpu().numpy())
        rgb_batched_out_img.save(f"{out_dir}/batched/{str(i).zfill(3)}_rgb_batched_out_{camera_id}.png")

    #### Depth
    print(f" ** Running depth data tests:")

    ## Numpy
    print(f" ** Numpy:")
    depth_tiled_np = camera_view.get_depth_tiled(device="cpu")
    print(f"depth_tiled_np.shape: {depth_tiled_np.shape}, type: {type(depth_tiled_np)}, dtype: {depth_tiled_np.dtype}")
    # Change inf to 0.0 and clip to range [0.0, 1.0]
    depth_tiled_np[np.isinf(depth_tiled_np)] = 0.0
    depth_tiled_np = np.clip(depth_tiled_np, 0.0, 1.0)
    depth_tiled_np_uint8 = (depth_tiled_np * 255).squeeze().astype(np.uint8)
    depth_tiled_img = Image.fromarray(depth_tiled_np_uint8, mode="L")
    depth_tiled_img.save(f"{out_dir}/tiled/{str(i).zfill(3)}_depth_tiled_np.png")

    # Using pre-allocated memory for out argument
    camera_view.get_depth_tiled(out=depth_np_tiled_out, device="cpu")
    print(
        f"depth_np_tiled_out.shape: {depth_np_tiled_out.shape}, type: {type(depth_np_tiled_out)}, dtype: {depth_np_tiled_out.dtype}"
    )
    # Change inf to 0.0 and clip to range [0.0, 1.0]
    depth_np_tiled_out[np.isinf(depth_np_tiled_out)] = 0.0
    depth_np_tiled_out = np.clip(depth_np_tiled_out, 0.0, 1.0)
    depth_np_tiled_out_uint8 = (depth_np_tiled_out * 255).squeeze().astype(np.uint8)
    depth_np_tiled_out_img = Image.fromarray(depth_np_tiled_out_uint8, mode="L")
    depth_np_tiled_out_img.save(f"{out_dir}/tiled/{str(i).zfill(3)}_depth_tiled_np_out.png")

    ## Torch
    print(f" ** Torch:")
    depth_tiled_torch = camera_view.get_depth_tiled(device="cuda")
    print(
        f"depth_tiled_torch.shape: {depth_tiled_torch.shape}, type: {type(depth_tiled_torch)}, dtype: {depth_tiled_torch.dtype}"
    )
    # Change inf to 0.0 and clip to range [0.0, 1.0]
    depth_tiled_torch[torch.isinf(depth_tiled_torch)] = 0.0
    depth_tiled_torch = torch.clip(depth_tiled_torch, 0.0, 1.0)
    depth_tiled_torch_uint8 = (depth_tiled_torch * 255).squeeze().to(dtype=torch.uint8)
    depth_tiled_torch_img = Image.fromarray(depth_tiled_torch_uint8.cpu().numpy(), mode="L")
    depth_tiled_torch_img.save(f"{out_dir}/tiled/{str(i).zfill(3)}_depth_tiled_torch.png")

    # Using pre-allocated memory for out argument
    camera_view.get_depth_tiled(out=depth_tiled_torch_out, device="cuda")
    print(
        f"depth_tiled_torch_out.shape: {depth_tiled_torch_out.shape}, type: {type(depth_tiled_torch_out)}, dtype: {depth_tiled_torch_out.dtype}"
    )
    # Change inf to 0.0 and clip to range [0.0, 1.0]
    depth_tiled_torch_out[torch.isinf(depth_tiled_torch_out)] = 0.0
    depth_tiled_torch_out = torch.clip(depth_tiled_torch_out, 0.0, 1.0)
    depth_tiled_torch_out_uint8 = (depth_tiled_torch_out * 255).squeeze().to(dtype=torch.uint8)
    depth_tiled_torch_out_img = Image.fromarray(depth_tiled_torch_out_uint8.cpu().numpy(), mode="L")
    depth_tiled_torch_out_img.save(f"{out_dir}/tiled/{str(i).zfill(3)}_depth_tiled_torch_out.png")

    ## Batched
    print(f" ** Batched:")
    depth_batched = camera_view.get_depth()
    print(f"depth_batched.shape: {depth_batched.shape}, type: {type(depth_batched)}, dtype: {depth_batched.dtype}")
    # Change inf to 0.0 and clip to range [0.0, 1.0]
    depth_batched[torch.isinf(depth_batched)] = 0.0
    depth_batched = torch.clip(depth_batched, 0.0, 1.0)
    # Split the batched data and save each image
    for camera_id in range(depth_batched.shape[0]):
        depth_batched_uint8 = (depth_batched[camera_id] * 255).squeeze().to(dtype=torch.uint8)
        depth_batched_img = Image.fromarray(depth_batched_uint8.cpu().numpy(), mode="L")
        depth_batched_img.save(f"{out_dir}/batched/{str(i).zfill(3)}_depth_batched_{camera_id}.png")

    # Using pre-allocated memory for out argument
    camera_view.get_depth(out=depth_batched_out)
    print(
        f"depth_batched_out.shape: {depth_batched_out.shape}, type: {type(depth_batched_out)}, dtype: {depth_batched_out.dtype}"
    )
    # Change inf to 0.0 and clip to range [0.0, 1.0]
    depth_batched_out[torch.isinf(depth_batched_out)] = 0.0
    depth_batched_out = torch.clip(depth_batched_out, 0.0, 1.0)
    # Split the batched data and save each image
    for camera_id in range(depth_batched_out.shape[0]):
        depth_batched_out_uint8 = (depth_batched_out[camera_id] * 255).squeeze().to(dtype=torch.uint8)
        depth_batched_out_img = Image.fromarray(depth_batched_out_uint8.cpu().numpy(), mode="L")
        depth_batched_out_img.save(f"{out_dir}/batched/{str(i).zfill(3)}_depth_batched_out_{camera_id}.png")

    # API
    print(f" ** Running API calls:")
    print(f"camera_view.get_local_poses(camera_axes='ros'): {camera_view.get_local_poses(camera_axes='ros')}")
    print(f"camera_view.get_local_poses(camera_axes='usd'): {camera_view.get_local_poses(camera_axes='usd')}")
    print(f"camera_view.get_local_poses(camera_axes='world'): {camera_view.get_local_poses(camera_axes='world')}")
    print(f"camera_view.get_world_poses(): {camera_view.get_world_poses()}")

    print(f"camera_view.get_focal_lengths(): {camera_view.get_focal_lengths()}")
    print(f"camera_view.get_focus_distances(): {camera_view.get_focus_distances()}")
    print(f"camera_view.get_lens_apertures(): {camera_view.get_lens_apertures()}")
    print(f"camera_view.get_horizontal_apertures(): {camera_view.get_horizontal_apertures()}")
    print(f"camera_view.get_vertical_apertures(): {camera_view.get_vertical_apertures()}")
    print(f"camera_view.get_projection_types(): {camera_view.get_projection_types()}")
    print(f"camera_view.get_projection_modes(): {camera_view.get_projection_modes()}")
    print(f"camera_view.get_stereo_roles(): {camera_view.get_stereo_roles()}")
    print(f"camera_view.get_shutter_properties(): {camera_view.get_shutter_properties()}")

    print(
        f"camera_view.set_shutter_properties(): {camera_view.set_shutter_properties(camera_view.get_shutter_properties())}"
    )

    print(f"camera_view.get_focus_distances(): {camera_view.get_focus_distances()}")

    simulation_app.update()

simulation_app.close()
