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

"""Tests for camera sensor point cloud generation."""

from __future__ import annotations

import os

import numpy as np
import omni.kit.app
import omni.kit.test
import omni.timeline
from isaacsim.core.experimental.objects import Cube, DomeLight, GroundPlane
from isaacsim.core.experimental.utils.stage import create_new_stage_async
from isaacsim.core.experimental.utils.transform import euler_angles_to_quaternion
from isaacsim.sensors.camera import Camera
from isaacsim.sensors.camera.tests.utils import debug_draw_clear_points, debug_draw_pointcloud
from isaacsim.test.utils.image_capture import capture_rgb_data_async
from isaacsim.test.utils.image_io import save_rgb_image


class TestCameraSensorPointcloud(omni.kit.test.AsyncTestCase):
    """Test suite for Camera pointcloud functionality."""

    GOLDEN_DIR = os.path.join(os.path.dirname(os.path.realpath(__file__)), "data", "golden", "camera_pointcloud")
    DEBUG_DIR = os.path.join(os.path.dirname(os.path.realpath(__file__)), "_out_test_camera_pointcloud")
    OUTPUT_DIR = None  # None=skip writes; GOLDEN_DIR=overwrite goldens; DEBUG_DIR=debug outputs.
    CAMERA_RESOLUTION = (256, 256)
    CAMERA_FREQUENCY = 60
    NUM_WARMUP_FRAMES = 5

    async def setUp(self) -> None:
        """Create a new stage for each test."""
        await omni.kit.app.get_app().next_update_async()
        await create_new_stage_async()
        await omni.kit.app.get_app().next_update_async()

    async def tearDown(self) -> None:
        """Stop the timeline and close the stage after each test."""
        timeline = omni.timeline.get_timeline_interface()
        timeline.stop()
        omni.usd.get_context().close_stage()
        await omni.kit.app.get_app().next_update_async()
        while omni.usd.get_context().get_stage_loading_status()[2] > 0:
            await omni.kit.app.get_app().next_update_async()

    async def _create_test_environment(self) -> tuple:
        """Create test environment with ground plane, cubes, and camera.

        Returns:
            None.

        """
        await create_new_stage_async()

        # Create a plane and dome light
        dome_light = DomeLight("/World/DomeLight")
        dome_light.set_intensities(500)
        GroundPlane("/World/defaultGroundPlane", sizes=10.0, templates=None)

        # Create cubes with different positions, scales, and orientations
        cube_1 = Cube(
            "/World/cube_1",
            sizes=1.0,
            positions=np.array([0.1, 0.15, 0.25]),
            scales=np.array([1.2, 0.8, 0.1]),
            orientations=euler_angles_to_quaternion(np.array([15, -10, 0]), degrees=True, extrinsic=False).numpy(),
        )
        cube_2 = Cube(
            "/World/cube_2",
            sizes=1.0,
            positions=np.array([1, -1, 0]),
            scales=np.array([0.5, 0.6, 0.7]),
        )
        cube_3 = Cube(
            "/World/cube_3",
            sizes=1.0,
            positions=np.array([-1, -1, -0.25]),
        )

        # Create camera looking down at the scene
        camera = Camera(
            prim_path="/World/camera",
            name="camera",
            frequency=self.CAMERA_FREQUENCY,
            position=np.array([0.0, 0.0, 10]),
            resolution=self.CAMERA_RESOLUTION,
            orientation=euler_angles_to_quaternion(np.array([0, 90, 30]), degrees=True).numpy(),
        )

        return camera, cube_1, cube_2, cube_3

    async def _start_data_capture(self, num_warm_up_frames: int = 0) -> None:
        """Start the timeline and optionally wait for valid data to be available.

        Args:
            num_warm_up_frames: Number of warm-up frames to render before capturing data.

        """
        timeline = omni.timeline.get_timeline_interface()
        timeline.play()
        timeline.commit()
        for _ in range(num_warm_up_frames):
            await omni.kit.app.get_app().next_update_async()

    async def _compare_pointcloud_data_resolution(
        self,
        camera: object,
        resolution: object = (64, 64),
        output_dir: str | None = None,
        save_debug_imgs: bool = True,
        print_stats: bool = False,
    ) -> None:
        """Compare pointcloud data from depth and pointcloud annotators against golden data.

        Args:
            camera: The camera instance to capture data from.
            resolution: Resolution to set for the camera.
            output_dir: Output directory for saving debug images.
            save_debug_imgs: Whether to save debug images.
            print_stats: Whether to print statistics.

        """
        res_str = f"{resolution[0]}_{resolution[1]}"
        camera.set_resolution(resolution)
        resolution = camera.get_resolution()
        expected_shape = (resolution[0] * resolution[1], 3)

        # Get pointcloud data from depth annotator
        camera.add_distance_to_image_plane_to_frame()
        for _ in range(self.NUM_WARMUP_FRAMES):
            await omni.kit.app.get_app().next_update_async()

        pointcloud_data_world_frame_from_depth = camera.get_pointcloud()
        pointcloud_data_camera_frame_from_depth = camera.get_pointcloud(world_frame=False)

        data_dir = None
        if output_dir is not None:
            is_golden_output = output_dir == self.GOLDEN_DIR
            data_dir = output_dir if is_golden_output else os.path.join(output_dir, "test_data")
            os.makedirs(data_dir, exist_ok=True)
            if is_golden_output:
                print(f"WARNING: Saving/overwriting golden data to {data_dir}")
            else:
                print(f"Saving test data to {data_dir}")
            np.save(
                os.path.join(data_dir, f"pointcloud_data_world_from_depth_res_{res_str}.npy"),
                pointcloud_data_world_frame_from_depth,
            )
            np.save(
                os.path.join(data_dir, f"pointcloud_data_camera_from_depth_res_{res_str}.npy"),
                pointcloud_data_camera_frame_from_depth,
            )

        golden_world_frame = np.load(os.path.join(self.GOLDEN_DIR, f"pointcloud_data_world_frame_res_{res_str}.npy"))
        golden_camera_frame = np.load(os.path.join(self.GOLDEN_DIR, f"pointcloud_data_camera_frame_res_{res_str}.npy"))
        golden_world_from_depth = np.load(
            os.path.join(self.GOLDEN_DIR, f"pointcloud_data_world_from_depth_res_{res_str}.npy")
        )
        golden_camera_from_depth = np.load(
            os.path.join(self.GOLDEN_DIR, f"pointcloud_data_camera_from_depth_res_{res_str}.npy")
        )
        if print_stats:
            print(f"Golden world frame shape: {golden_world_frame.shape}")
            print(f"Golden camera frame shape: {golden_camera_frame.shape}")
        golden_world_sorted = np.sort(golden_world_frame, axis=0)
        golden_camera_sorted = np.sort(golden_camera_frame, axis=0)
        golden_world_from_depth_sorted = np.sort(golden_world_from_depth, axis=0)
        golden_camera_from_depth_sorted = np.sort(golden_camera_from_depth, axis=0)

        # Get pointcloud data from pointcloud annotator
        camera.add_pointcloud_to_frame()
        for _ in range(self.NUM_WARMUP_FRAMES):
            await omni.kit.app.get_app().next_update_async()

        pointcloud_data_world_frame = camera.get_pointcloud()
        pointcloud_data_camera_frame = camera.get_pointcloud(world_frame=False)

        # Save test data to the output directory for debug
        if data_dir is not None:
            np.save(
                os.path.join(data_dir, f"pointcloud_data_world_frame_res_{res_str}.npy"),
                pointcloud_data_world_frame,
            )
            np.save(
                os.path.join(data_dir, f"pointcloud_data_camera_frame_res_{res_str}.npy"),
                pointcloud_data_camera_frame,
            )

        # Print debug info about the data
        print(f"\n=== Pointcloud Debug Info (resolution {res_str}) ===")
        print(f"Expected shape: {expected_shape}")
        print(f"World frame from depth: shape={pointcloud_data_world_frame_from_depth.shape}")
        print(f"Camera frame from depth: shape={pointcloud_data_camera_frame_from_depth.shape}")
        print(f"World frame from annotator: shape={pointcloud_data_world_frame.shape}")
        print(f"Camera frame from annotator: shape={pointcloud_data_camera_frame.shape}")

        # Save debug images BEFORE assertions (so they're saved even if test fails)
        if output_dir is not None and save_debug_imgs:
            debug_img_dir = os.path.join(output_dir, "debug_imgs")
            os.makedirs(debug_img_dir, exist_ok=True)
            print(f"Saving debug images to {debug_img_dir}")
            green_color = (0, 1, 0, 0.75)
            green_size = 4
            red_color = (1, 0, 0, 0.5)
            red_size = 8

            # Draw world frame pointcloud data (green=annotator, red=depth)
            debug_draw_clear_points()
            debug_draw_pointcloud(pointcloud_data_world_frame, color=green_color, size=green_size)
            debug_draw_pointcloud(pointcloud_data_world_frame_from_depth, color=red_color, size=red_size)
            rgb_data = await capture_rgb_data_async(
                camera_position=(5, 5, 5), camera_look_at=(0, 0, 0), resolution=(1280, 720)
            )
            save_rgb_image(rgb_data, debug_img_dir, f"pointcloud_world_frame_res_{res_str}_rgb.png")

            # Draw camera frame pointcloud data (green=annotator, red=depth)
            debug_draw_clear_points()
            debug_draw_pointcloud(pointcloud_data_camera_frame, color=green_color, size=green_size)
            debug_draw_pointcloud(pointcloud_data_camera_frame_from_depth, color=red_color, size=red_size)
            rgb_data = await capture_rgb_data_async(
                camera_position=(5, 5, 15), camera_look_at=(0, 0, 10), resolution=(1280, 720)
            )
            save_rgb_image(rgb_data, debug_img_dir, f"pointcloud_camera_frame_res_{res_str}_rgb.png")

        datasets = [
            (
                "depth",
                pointcloud_data_world_frame_from_depth,
                pointcloud_data_camera_frame_from_depth,
                golden_world_from_depth_sorted,
                golden_camera_from_depth_sorted,
            ),
            (
                "annotator",
                pointcloud_data_world_frame,
                pointcloud_data_camera_frame,
                golden_world_sorted,
                golden_camera_sorted,
            ),
        ]
        atol = 1e-4
        for label, world_data, camera_data, golden_world, golden_camera in datasets:
            world_sorted = np.sort(world_data, axis=0)
            camera_sorted = np.sort(camera_data, axis=0)

            self.assertEqual(
                world_sorted.shape,
                golden_world.shape,
                f"World frame ({label}) shape mismatch: test={world_sorted.shape} vs golden={golden_world.shape}",
            )
            self.assertEqual(
                camera_sorted.shape,
                golden_camera.shape,
                f"Camera frame ({label}) shape mismatch: test={camera_sorted.shape} vs golden={golden_camera.shape}",
            )

            world_diff = np.abs(world_sorted - golden_world)
            camera_diff = np.abs(camera_sorted - golden_camera)
            world_mean, world_max = world_diff.mean(), world_diff.max()
            camera_mean, camera_max = camera_diff.mean(), camera_diff.max()

            if print_stats:
                world_hist, world_bins = np.histogram(world_diff, bins=10)
                camera_hist, camera_bins = np.histogram(camera_diff, bins=10)
                print(
                    f"{label} world diff: mean={world_mean:.6f}, max={world_max:.6f}, "
                    f"hist={world_hist.tolist()}, bins={world_bins.tolist()}"
                )
                print(
                    f"{label} camera diff: mean={camera_mean:.6f}, max={camera_max:.6f}, "
                    f"hist={camera_hist.tolist()}, bins={camera_bins.tolist()}"
                )

            self.assertTrue(
                np.allclose(world_sorted, golden_world, atol=atol),
                f"World frame (sorted) pointcloud from {label} does not match golden data. "
                f"mean={world_mean:.6f}, max={world_max:.6f}",
            )
            self.assertTrue(
                np.allclose(camera_sorted, golden_camera, atol=atol),
                f"Camera frame (sorted) pointcloud from {label} does not match golden data. "
                f"mean={camera_mean:.6f}, max={camera_max:.6f}",
            )

    async def test_pointcloud_data_resolution_4_4(self) -> None:
        """Test pointcloud data at 4x4 resolution."""
        camera, _, _, _ = await self._create_test_environment()
        for _ in range(self.NUM_WARMUP_FRAMES):
            await omni.kit.app.get_app().next_update_async()
        timeline = omni.timeline.get_timeline_interface()
        timeline.play()
        timeline.commit()
        camera.initialize()
        await self._compare_pointcloud_data_resolution(
            camera,
            resolution=(4, 4),
            output_dir=self.OUTPUT_DIR,
            print_stats=True,
        )

    async def test_pointcloud_data_resolution_64_64(self) -> None:
        """Test pointcloud data at 64x64 resolution."""
        camera, _, _, _ = await self._create_test_environment()
        for _ in range(self.NUM_WARMUP_FRAMES):
            await omni.kit.app.get_app().next_update_async()
        timeline = omni.timeline.get_timeline_interface()
        timeline.play()
        timeline.commit()
        camera.initialize()
        await self._compare_pointcloud_data_resolution(
            camera,
            resolution=(64, 64),
            output_dir=self.OUTPUT_DIR,
            print_stats=True,
        )

    async def test_pointcloud_data_resolution_67_137(self) -> None:
        """Test pointcloud data at 67x137 non-square resolution."""
        camera, _, _, _ = await self._create_test_environment()
        for _ in range(self.NUM_WARMUP_FRAMES):
            await omni.kit.app.get_app().next_update_async()
        timeline = omni.timeline.get_timeline_interface()
        timeline.play()
        timeline.commit()
        camera.initialize()
        await self._compare_pointcloud_data_resolution(
            camera,
            resolution=(67, 137),
            output_dir=self.OUTPUT_DIR,
            print_stats=True,
        )

    async def test_pointcloud_data_resolution_211_99(self) -> None:
        """Test pointcloud data at 211x99 non-square resolution."""
        camera, _, _, _ = await self._create_test_environment()
        for _ in range(self.NUM_WARMUP_FRAMES):
            await omni.kit.app.get_app().next_update_async()
        timeline = omni.timeline.get_timeline_interface()
        timeline.play()
        timeline.commit()
        camera.initialize()
        await self._compare_pointcloud_data_resolution(
            camera,
            resolution=(211, 99),
            output_dir=self.OUTPUT_DIR,
            print_stats=True,
        )
