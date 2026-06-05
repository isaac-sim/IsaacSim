# SPDX-FileCopyrightText: Copyright (c) 2023-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

"""Generate augmented synthetic data from a writer."""

from isaacsim import SimulationApp

simulation_app = SimulationApp(launch_config={"headless": False})

import argparse
import os
import time

import carb.settings
import numpy as np
import omni.replicator.core as rep
import omni.usd
import warp as wp
from isaacsim.core.utils.stage import open_stage
from isaacsim.storage.native import get_assets_root_path

parser = argparse.ArgumentParser()
parser.add_argument("--num_frames", type=int, default=5, help="The number of frames to capture")
parser.add_argument(
    "--use_warp",
    action="store_true",
    help="Use warp augmentations instead of numpy",
)
parser.add_argument("--resolution", nargs=2, type=int, default=[512, 512], help="Camera resolution")
parser.add_argument("--env_url", type=str, default="", help="USD environment URL (empty for basic scene)")
args, unknown = parser.parse_known_args()

num_frames = args.num_frames
use_warp = args.use_warp
resolution = args.resolution
env_url = args.env_url or None
SEED = 42

# Enable warp scripts
carb.settings.get_settings().set_bool("/app/omni.graph.scriptnode/opt_in", True)


def gaussian_noise_rgb_np(data_in, sigma: float, seed: int):
    """Add Gaussian noise to RGB data using NumPy (CPU)."""
    np.random.seed(seed)
    # Convert to float32 space
    data_in = data_in.astype(np.float32)
    # Add Gaussian noise to each channel
    data_in[:, :, 0] = data_in[:, :, 0] + np.random.randn(*data_in.shape[:-1]) * sigma
    data_in[:, :, 1] = data_in[:, :, 1] + np.random.randn(*data_in.shape[:-1]) * sigma
    data_in[:, :, 2] = data_in[:, :, 2] + np.random.randn(*data_in.shape[:-1]) * sigma
    # Clip to [0, 255] and convert to uint8
    data_in = np.clip(data_in, 0, 255).astype(np.uint8)
    return data_in


@wp.kernel
def gaussian_noise_rgb_wp(
    data_in: wp.array3d(dtype=wp.uint8), data_out: wp.array3d(dtype=wp.uint8), sigma: float, seed: int
):
    """Add Gaussian noise to RGB data using Warp (GPU)."""
    # Get thread coordinates and image dimensions to calculate unique pixel ID for random generation
    i, j = wp.tid()
    dim_i = data_in.shape[0]
    dim_j = data_in.shape[1]
    pixel_id = i * dim_i + j

    # Use pixel_id as offset to create unique seeds for each pixel and channel (ensure independent noise patterns across R,G,B channels)
    state_r = wp.rand_init(seed, pixel_id + (dim_i * dim_j * 0))
    state_g = wp.rand_init(seed, pixel_id + (dim_i * dim_j * 1))
    state_b = wp.rand_init(seed, pixel_id + (dim_i * dim_j * 2))

    # Apply noise to each channel independently using unique seeds; work in float32 space, then clip and convert to uint8
    val_r = wp.float32(data_in[i, j, 0]) + sigma * wp.randn(state_r)
    val_g = wp.float32(data_in[i, j, 1]) + sigma * wp.randn(state_g)
    val_b = wp.float32(data_in[i, j, 2]) + sigma * wp.randn(state_b)

    # Clip to [0, 255] and convert to uint8
    data_out[i, j, 0] = wp.uint8(wp.clamp(val_r, 0.0, 255.0))
    data_out[i, j, 1] = wp.uint8(wp.clamp(val_g, 0.0, 255.0))
    data_out[i, j, 2] = wp.uint8(wp.clamp(val_b, 0.0, 255.0))
    data_out[i, j, 3] = data_in[i, j, 3]


def gaussian_noise_depth_np(data_in, sigma: float, seed: int):
    """Add Gaussian noise to depth values using NumPy (CPU)."""
    np.random.seed(seed)
    result = data_in.astype(np.float32) + np.random.randn(*data_in.shape) * sigma
    return np.clip(result, 0, None).astype(data_in.dtype)


rep.AnnotatorRegistry.register_augmentation(
    "gn_depth_np", rep.annotators.Augmentation.from_function(gaussian_noise_depth_np, sigma=0.1, seed=None)
)


@wp.kernel
def gaussian_noise_depth_wp(
    data_in: wp.array2d(dtype=wp.float32), data_out: wp.array2d(dtype=wp.float32), sigma: float, seed: int
):
    """Add Gaussian noise to depth values using Warp (GPU)."""
    i, j = wp.tid()
    # Unique ID for random seed per pixel
    scalar_pixel_id = i * data_in.shape[1] + j
    state = wp.rand_init(seed, scalar_pixel_id)
    data_out[i, j] = data_in[i, j] + sigma * wp.randn(state)


rep.AnnotatorRegistry.register_augmentation(
    "gn_depth_wp", rep.annotators.Augmentation.from_function(gaussian_noise_depth_wp, sigma=0.1, seed=None)
)


def run_example(num_frames: int, resolution: tuple[int, int], use_warp: bool, env_url: str | None = None) -> float:
    """Run the capture pipeline using step() to trigger a randomization and data capture."""
    print(f"Running example with num_frames: {num_frames}, resolution: {resolution}, use_warp: {use_warp}")

    if env_url is not None and env_url != "":
        assets_root_path = get_assets_root_path()
        stage_path = assets_root_path + env_url
        print(f"Opening stage: {stage_path}")
        open_stage(stage_path)
    else:
        omni.usd.get_context().new_stage()
        rep.functional.create.dome_light(intensity=1000, rotation=(270, 0, 0))
        ground_plane = rep.functional.create.plane(scale=(10, 10, 1), position=(0, 0, 0))
        rep.functional.physics.apply_collider(ground_plane)

    # Use a fixed global seed for reproducibility
    rep.set_global_seed(SEED)

    # Disable capture on play, data is captured manually using the step function
    rep.orchestrator.set_capture_on_play(False)

    # Set DLSS to Quality mode (2) for best SDG results (Options: 0 (Performance), 1 (Balanced), 2 (Quality), 3 (Auto)
    carb.settings.get_settings().set("rtx/post/dlss/execMode", 2)

    # Augment the annotators
    rgb_to_hsv_augm = rep.annotators.Augmentation.from_function(rep.augmentations_default.aug_rgb_to_hsv)
    hsv_to_rgb_augm = rep.annotators.Augmentation.from_function(rep.augmentations_default.aug_hsv_to_rgb)

    # Augment the RGB and depth annotators
    gn_rgb_augm = rep.annotators.Augmentation.from_function(
        gaussian_noise_rgb_wp if use_warp else gaussian_noise_rgb_np, sigma=15.0, seed=SEED
    )
    gn_depth_augm = rep.annotators.get_augmentation("gn_depth_wp" if use_warp else "gn_depth_np")

    # Create a writer and apply the augmentations to its corresponding annotators
    out_dir = os.path.join(os.getcwd(), f"_out_augm_writer_{'warp' if use_warp else 'numpy'}")
    backend = rep.backends.get("DiskBackend")
    backend.initialize(output_dir=out_dir)
    print(f"Writing data to: {out_dir}")
    writer = rep.writers.get("BasicWriter")
    writer.initialize(backend=backend, rgb=True, distance_to_camera=True, colorize_depth=True)

    # Apply the augmentations to the RGB and depth annotators
    augmented_rgb_annot = rep.annotators.get("rgb").augment_compose(
        [rgb_to_hsv_augm, gn_rgb_augm, hsv_to_rgb_augm], name="rgb"
    )
    writer.add_annotator(augmented_rgb_annot)
    writer.augment_annotator("distance_to_camera", gn_depth_augm)

    # Create a camera and a render product and attach them to the writer
    cam = rep.functional.create.camera(position=(0, 0, 5), look_at=(0, 0, 0))
    rp = rep.create.render_product(cam, resolution)
    writer.attach(rp)

    # Create a red cube and randomize its rotation on a custom event sent before each capture step
    red_cube = rep.functional.create.cube(position=(0, 0, 0.71))
    rep.functional.create.material(mdl="OmniPBR.mdl", bind_prims=[red_cube], diffuse_color_constant=(1, 0, 0))
    with rep.trigger.on_custom_event(event_name="randomize_red_cube"):
        red_cube_node = rep.get.prim_at_path(red_cube.GetPath())
        with red_cube_node:
            rep.randomizer.rotation()

    capture_start = time.time()
    for frame_idx in range(num_frames):
        print(f"  Capturing frame {frame_idx + 1}/{num_frames}")
        rep.utils.send_og_event(event_name="randomize_red_cube")
        rep.orchestrator.step(rt_subframes=32)

    # Wait for the data to be written to disk and release resources
    rep.orchestrator.wait_until_complete()
    writer.detach()
    rp.destroy()

    return time.time() - capture_start


duration = run_example(num_frames, resolution, use_warp, env_url)
average = duration / num_frames if num_frames else 0.0
mode_label = "warp" if use_warp else "numpy"
print(
    f"The duration for capturing {num_frames} frames using '{mode_label}' was: {duration:.4f} seconds, "
    f"with an average of {average:.4f} seconds per frame."
)

simulation_app.close()
