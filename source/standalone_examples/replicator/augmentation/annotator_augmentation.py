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

"""Generate augmented synthetic data from annotators."""

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
from omni.replicator.core.functional import write_image

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


def rgb_to_bgr_np(data_in):
    """Swap RGBA red and blue channels using NumPy (CPU)."""
    data_in[:, :, [0, 2]] = data_in[:, :, [2, 0]]
    return data_in


@wp.kernel
def rgb_to_bgr_wp(data_in: wp.array3d(dtype=wp.uint8), data_out: wp.array3d(dtype=wp.uint8)):
    """Swap RGBA red and blue channels using Warp (GPU)."""
    i, j = wp.tid()
    data_out[i, j, 0] = data_in[i, j, 2]
    data_out[i, j, 1] = data_in[i, j, 1]
    data_out[i, j, 2] = data_in[i, j, 0]
    data_out[i, j, 3] = data_in[i, j, 3]


def gaussian_noise_depth_np(data_in, sigma, seed):
    """Add Gaussian noise to depth values using NumPy (CPU)."""
    np.random.seed(seed)
    result = data_in.astype(np.float32) + np.random.randn(*data_in.shape) * sigma
    return np.clip(result, 0, None).astype(data_in.dtype)


rep.annotators.register_augmentation(
    "gn_depth_np", rep.annotators.Augmentation.from_function(gaussian_noise_depth_np, sigma=0.1, seed=SEED)
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


rep.annotators.register_augmentation(
    "gn_depth_wp", rep.annotators.Augmentation.from_function(gaussian_noise_depth_wp, sigma=0.1, seed=SEED)
)


def convert_depth_to_uint8(data: np.ndarray | wp.array) -> np.ndarray:
    """Normalize depth data and convert it to uint8 grayscale."""
    if isinstance(data, wp.array):
        data = data.numpy()
    depth = data.astype(np.float32, copy=False)
    depth[np.isinf(depth)] = np.nan
    mean_val = np.nanmean(depth)
    if np.isnan(mean_val):
        mean_val = 0.0
    depth = np.nan_to_num(depth, nan=mean_val, copy=False)
    min_val = depth.min()
    max_val = depth.max()
    if max_val <= min_val:
        return np.zeros(depth.shape, dtype=np.uint8)
    normalized = (depth - min_val) / (max_val - min_val)
    return (normalized * 255.0).astype(np.uint8)


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

    # Augment the RGB and depth annotators
    rgb_to_bgr_augm = rep.annotators.Augmentation.from_function(rgb_to_bgr_wp if use_warp else rgb_to_bgr_np)
    depth_aug = rep.annotators.get_augmentation("gn_depth_wp" if use_warp else "gn_depth_np")
    rgb_to_bgr_annot = rep.annotators.augment(
        source_annotator=rep.annotators.get("rgb"),
        augmentation=rgb_to_bgr_augm,
    )
    depth_annot_1 = rep.annotators.get("distance_to_camera")
    depth_annot_1.augment(depth_aug)
    depth_annot_2 = rep.annotators.get("distance_to_camera")
    depth_annot_2.augment(depth_aug, sigma=0.5)

    # Create the render product and attach the annotators to it
    cam = rep.functional.create.camera(position=(0, 0, 5), look_at=(0, 0, 0))
    rp = rep.create.render_product(cam, resolution)
    rgb_to_bgr_annot.attach(rp)
    depth_annot_1.attach(rp)
    depth_annot_2.attach(rp)

    # Create a red cube and randomize its rotation on a custom event sent before each capture step
    red_cube = rep.functional.create.cube(position=(0, 0, 0.71))
    rep.functional.create.material(mdl="OmniPBR.mdl", bind_prims=[red_cube], diffuse_color_constant=(1, 0, 0))

    with rep.trigger.on_custom_event(event_name="randomize_red_cube"):
        red_cube_node = rep.get.prim_at_path(red_cube.GetPath())
        with red_cube_node:
            rep.randomizer.rotation()

    # Output directory
    out_dir = os.path.join(os.getcwd(), f"_out_augm_annot_{'warp' if use_warp else 'numpy'}")
    print(f"Writing data to: {out_dir}")
    os.makedirs(out_dir, exist_ok=True)

    capture_start = time.time()
    for frame_idx in range(num_frames):
        print(f"  Capturing frame {frame_idx + 1}/{num_frames}")
        rep.utils.send_og_event(event_name="randomize_red_cube")
        rep.orchestrator.step(rt_subframes=32)

        # Get the data from the annotators
        rgb_data = rgb_to_bgr_annot.get_data()
        depth_data_1 = depth_annot_1.get_data()
        depth_data_2 = depth_annot_2.get_data()

        # Schedule the write of the data to disk
        write_image(path=os.path.join(out_dir, f"annot_rgb_{frame_idx}.png"), data=rgb_data)
        write_image(
            path=os.path.join(out_dir, f"annot_depth_1_{frame_idx}.png"),
            data=convert_depth_to_uint8(depth_data_1),
        )
        write_image(
            path=os.path.join(out_dir, f"annot_depth_2_{frame_idx}.png"),
            data=convert_depth_to_uint8(depth_data_2),
        )

    # Wait for the data to be written to disk and release resources
    rep.orchestrator.wait_until_complete()
    rgb_to_bgr_annot.detach()
    depth_annot_1.detach()
    depth_annot_2.detach()
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
