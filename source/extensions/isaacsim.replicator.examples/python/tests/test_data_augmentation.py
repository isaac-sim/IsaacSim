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


import tempfile

import carb.settings
import omni.kit
import omni.usd
from isaacsim.test.utils.file_validation import validate_folder_contents
from isaacsim.test.utils.image_comparison import compare_images_in_directories


class TestDataAugmentation(omni.kit.test.AsyncTestCase):

    # Per-channel mean-diff tolerances. Bit-stable augmentations (annotator RGB channel
    # swap) stay close to the golden, while noisy augmentations (gaussian noise on RGB
    # or depth, amplified for depth by the min/max normalization in convert_depth_to_uint8)
    # accumulate larger per-pixel deltas.
    NO_NOISE_MEAN_DIFF_TOLERANCE = 5
    NOISE_MEAN_DIFF_TOLERANCE = 30

    async def setUp(self):
        await omni.kit.app.get_app().next_update_async()
        omni.usd.get_context().new_stage()
        await omni.kit.app.get_app().next_update_async()
        self.original_dlss_exec_mode = carb.settings.get_settings().get("rtx/post/dlss/execMode")

    async def tearDown(self):
        omni.usd.get_context().close_stage()
        await omni.kit.app.get_app().next_update_async()
        while omni.usd.get_context().get_stage_loading_status()[2] > 0:
            await omni.kit.app.get_app().next_update_async()
        carb.settings.get_settings().set("rtx/post/dlss/execMode", self.original_dlss_exec_mode)

    async def test_data_augmentation_annotator(self):
        import asyncio
        import os
        import time

        import carb.settings
        import numpy as np
        import omni.replicator.core as rep
        import warp as wp
        from isaacsim.core.experimental.utils.stage import open_stage
        from isaacsim.storage.native import get_assets_root_path_async
        from omni.replicator.core.functional import write_image

        NUM_FRAMES = 5
        RESOLUTION = (512, 512)
        USE_WARP = False
        ENV_URL = "/Isaac/Environments/Grid/default_environment.usd"
        SEED = 42

        annot_test_root = tempfile.mkdtemp(prefix="test_augm_annot_")
        print(f"Test output root: {annot_test_root}")

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

        def gaussian_noise_depth_np(data_in, sigma: float, seed: int):
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

        def convert_depth_to_uint8(data):
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

        async def run_example_async(
            num_frames: int, resolution: tuple[int, int], use_warp: bool, env_url: str | None = None
        ) -> float:
            """Run the capture pipeline using step_async() to trigger a randomization and data capture."""
            print(f"Running example with num_frames: {num_frames}, resolution: {resolution}, use_warp: {use_warp}")

            if env_url is not None and env_url != "":
                assets_root_path = await get_assets_root_path_async()
                stage_path = assets_root_path + env_url
                print(f"Opening stage: {stage_path}")
                open_stage(stage_path)
            else:
                await omni.usd.get_context().new_stage_async()
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
            out_dir = os.path.join(annot_test_root, f"{'warp' if use_warp else 'numpy'}")
            print(f"Writing data to: {out_dir}")
            os.makedirs(out_dir, exist_ok=True)

            capture_start = time.time()
            for frame_idx in range(num_frames):
                print(f"  Capturing frame {frame_idx + 1}/{num_frames}")
                rep.utils.send_og_event(event_name="randomize_red_cube")
                await rep.orchestrator.step_async(rt_subframes=32)

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
            await rep.orchestrator.wait_until_complete_async()
            rgb_to_bgr_annot.detach()
            depth_annot_1.detach()
            depth_annot_2.detach()
            rp.destroy()

            return time.time() - capture_start

        def on_task_done(task: asyncio.Task):
            """Report timing information when capture completes."""
            duration = task.result()
            average = duration / NUM_FRAMES if NUM_FRAMES else 0.0
            mode_label = "warp" if USE_WARP else "numpy"
            print(
                f"The duration for capturing {NUM_FRAMES} frames using '{mode_label}' was: {duration:.4f} seconds, "
                f"with an average of {average:.4f} seconds per frame."
            )

        # task = asyncio.ensure_future(run_example_async(NUM_FRAMES, RESOLUTION, USE_WARP))
        # task.add_done_callback(on_task_done)

        test_num_frames = 2
        for test_use_warp in [False, True]:
            mode_label = "warp" if test_use_warp else "numpy"
            golden_dir = os.path.join(
                os.path.dirname(os.path.realpath(__file__)), "data", "golden", f"_out_augm_annot_{mode_label}"
            )
            await run_example_async(
                num_frames=test_num_frames, resolution=RESOLUTION, use_warp=test_use_warp, env_url=""
            )
            out_dir = os.path.join(annot_test_root, mode_label)

            folder_contents_success = validate_folder_contents(
                path=out_dir, expected_counts={"png": test_num_frames * 3}
            )
            self.assertTrue(
                folder_contents_success, f"Folder contents validation failed ({mode_label}). Output dir: {out_dir}"
            )

            rgb_result = compare_images_in_directories(
                golden_dir=golden_dir,
                test_dir=out_dir,
                path_pattern=r"^annot_rgb_.*\.png$",
                allclose_rtol=None,
                allclose_atol=None,
                mean_tolerance=self.NO_NOISE_MEAN_DIFF_TOLERANCE,
                print_all_stats=False,
            )
            self.assertTrue(
                rgb_result["all_passed"],
                f"RGB image comparison failed ({mode_label}, tol={self.NO_NOISE_MEAN_DIFF_TOLERANCE}). Output dir: {out_dir}",
            )

            depth_result = compare_images_in_directories(
                golden_dir=golden_dir,
                test_dir=out_dir,
                path_pattern=r"^annot_depth_.*\.png$",
                allclose_rtol=None,
                allclose_atol=None,
                mean_tolerance=self.NOISE_MEAN_DIFF_TOLERANCE,
                print_all_stats=False,
            )
            self.assertTrue(
                depth_result["all_passed"],
                f"Depth image comparison failed ({mode_label}, tol={self.NOISE_MEAN_DIFF_TOLERANCE}). Output dir: {out_dir}",
            )

    async def test_data_augmentation_writer(self):
        import asyncio
        import os
        import time

        import carb.settings
        import numpy as np
        import omni.replicator.core as rep
        import warp as wp
        from isaacsim.core.experimental.utils.stage import open_stage
        from isaacsim.storage.native import get_assets_root_path_async

        NUM_FRAMES = 5
        RESOLUTION = (512, 512)
        USE_WARP = False
        ENV_URL = "/Isaac/Environments/Grid/default_environment.usd"
        SEED = 42

        writer_test_root = tempfile.mkdtemp(prefix="test_augm_writer_")
        print(f"Test output root: {writer_test_root}")

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

        async def run_example_async(
            num_frames: int, resolution: tuple[int, int], use_warp: bool, env_url: str | None = None
        ) -> float:
            """Run the capture pipeline using step_async() to trigger a randomization and data capture."""
            print(f"Running example with num_frames: {num_frames}, resolution: {resolution}, use_warp: {use_warp}")

            if env_url is not None and env_url != "":
                assets_root_path = await get_assets_root_path_async()
                stage_path = assets_root_path + env_url
                print(f"Opening stage: {stage_path}")
                open_stage(stage_path)
            else:
                await omni.usd.get_context().new_stage_async()
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
            out_dir = os.path.join(writer_test_root, f"{'warp' if use_warp else 'numpy'}")
            os.makedirs(out_dir)
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
                await rep.orchestrator.step_async(rt_subframes=32)

            # Wait for the data to be written to disk and release resources
            await rep.orchestrator.wait_until_complete_async()
            writer.detach()
            rp.destroy()

            return time.time() - capture_start

        def on_task_done(task: asyncio.Task):
            """Report timing information when capture completes."""
            duration = task.result()
            average = duration / NUM_FRAMES if NUM_FRAMES else 0.0
            mode_label = "warp" if USE_WARP else "numpy"
            print(
                f"The duration for capturing {NUM_FRAMES} frames using '{mode_label}' was: {duration:.4f} seconds, "
                f"with an average of {average:.4f} seconds per frame."
            )

        # task = asyncio.ensure_future(run_example_async(NUM_FRAMES, RESOLUTION, USE_WARP))
        # task.add_done_callback(on_task_done)

        test_num_frames = 2
        for test_use_warp in [False, True]:
            mode_label = "warp" if test_use_warp else "numpy"
            golden_dir = os.path.join(
                os.path.dirname(os.path.realpath(__file__)), "data", "golden", f"_out_augm_writer_{mode_label}"
            )
            await run_example_async(
                num_frames=test_num_frames, resolution=RESOLUTION, use_warp=test_use_warp, env_url=""
            )
            out_dir = os.path.join(writer_test_root, mode_label)

            folder_contents_success = validate_folder_contents(
                path=out_dir, expected_counts={"png": test_num_frames * 2, "npy": test_num_frames}
            )
            self.assertTrue(
                folder_contents_success, f"Folder contents validation failed ({mode_label}). Output dir: {out_dir}"
            )

            rgb_result = compare_images_in_directories(
                golden_dir=golden_dir,
                test_dir=out_dir,
                path_pattern=r"^rgb_.*\.png$",
                allclose_rtol=None,
                allclose_atol=None,
                mean_tolerance=self.NOISE_MEAN_DIFF_TOLERANCE,
                print_all_stats=False,
            )
            self.assertTrue(
                rgb_result["all_passed"],
                f"RGB image comparison failed ({mode_label}, tol={self.NOISE_MEAN_DIFF_TOLERANCE}). Output dir: {out_dir}",
            )

            depth_result = compare_images_in_directories(
                golden_dir=golden_dir,
                test_dir=out_dir,
                path_pattern=r"^distance_to_camera_.*\.png$",
                allclose_rtol=None,
                allclose_atol=None,
                mean_tolerance=self.NOISE_MEAN_DIFF_TOLERANCE,
                print_all_stats=False,
            )
            self.assertTrue(
                depth_result["all_passed"],
                f"Depth image comparison failed ({mode_label}, tol={self.NOISE_MEAN_DIFF_TOLERANCE}). Output dir: {out_dir}",
            )
