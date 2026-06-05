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

"""Tests for camera view sensor functionality."""

import os

import carb
import cv2
import numpy as np
import omni.kit.test
import omni.replicator.core as rep
import omni.timeline
import warp as wp
from isaacsim.core.deprecation_manager import import_module
from isaacsim.core.experimental.materials import OmniPbrMaterial
from isaacsim.core.experimental.objects import Cube, DomeLight, GroundPlane
from isaacsim.core.experimental.utils.stage import create_new_stage_async
from isaacsim.sensors.camera.camera_view import ANNOTATOR_SPEC, CameraView
from isaacsim.test.utils.image_comparison import compare_images_in_directories

torch = import_module("torch")


class TestCameraViewSensor(omni.kit.test.AsyncTestCase):
    """Test cases for CameraViewSensor."""

    GOLDEN_DIR = os.path.join(os.path.dirname(os.path.realpath(__file__)), "data", "golden", "camera_view")
    USE_LOCAL_TEST_OUTPUT_DIR = False
    LOCAL_OUTPUT_DIR = os.path.join(os.path.dirname(os.path.realpath(__file__)), "_out_test_camera_view_sensor")
    TEMP_OUTPUT_DIR = carb.tokens.get_tokens_interface().resolve("${temp}/test_camera_view_sensor")
    TEST_OUTPUT_DIR = LOCAL_OUTPUT_DIR if USE_LOCAL_TEST_OUTPUT_DIR else TEMP_OUTPUT_DIR
    RGB_MEAN_DIFF_TOLERANCE = 5
    DEPTH_MEAN_DIFF_TOLERANCE = 2
    NUM_CAMERAS = 4  # Number of cameras to create for the tiled sensor
    CAMERA_VIEW_RESOLUTION = (256, 256)
    EXPECTED_ANNOTATOR_SPEC = {
        "rgb": {"name": "rgba", "channels": 3, "dtype": wp.uint8},
        "rgba": {"name": "rgba", "channels": 4, "dtype": wp.uint8},
        "depth": {"name": "distance_to_image_plane", "channels": 1, "dtype": wp.float32},
        "distance_to_image_plane": {"name": "distance_to_image_plane", "channels": 1, "dtype": wp.float32},
        "distance_to_camera": {"name": "distance_to_camera", "channels": 1, "dtype": wp.float32},
        "normals": {"name": "normals", "channels": 4, "dtype": wp.float32},
        "motion_vectors": {"name": "motion_vectors", "channels": 4, "dtype": wp.float32},
        "semantic_segmentation": {"name": "semantic_segmentation", "channels": 1, "dtype": wp.uint32},
        "instance_segmentation_fast": {"name": "instance_segmentation_fast", "channels": 1, "dtype": wp.uint32},
        "instance_id_segmentation_fast": {"name": "instance_id_segmentation_fast", "channels": 1, "dtype": wp.uint32},
    }

    async def setUp(self) -> None:
        """Set up test fixtures."""
        await omni.kit.app.get_app().next_update_async()
        await create_new_stage_async()
        await omni.kit.app.get_app().next_update_async()

    async def tearDown(self) -> None:
        """Tear down test fixtures."""
        timeline = omni.timeline.get_timeline_interface()
        timeline.stop()
        omni.usd.get_context().close_stage()
        await omni.kit.app.get_app().next_update_async()
        while omni.usd.get_context().get_stage_loading_status()[2] > 0:
            await omni.kit.app.get_app().next_update_async()

    async def test_annotator_spec_keys(self) -> None:
        """Verify that EXPECTED_ANNOTATOR_SPEC matches the actual ANNOTATOR_SPEC keys."""
        self.assertEqual(
            sorted(ANNOTATOR_SPEC.keys()),
            sorted(self.EXPECTED_ANNOTATOR_SPEC.keys()),
        )

    async def _create_test_environment(self) -> None:
        """Create the test environment with ground plane, cubes, and cameras."""
        await create_new_stage_async()

        # Create a plane and dome light
        dome_light = DomeLight("/World/DomeLight")
        dome_light.set_intensities(500)
        GroundPlane("/World/defaultGroundPlane", sizes=10.0, templates=None)

        # Add a red and blue cube
        red_cube_1 = Cube(
            "/World/cube_1",
            sizes=1.0,
            positions=np.array([0.25, 0.25, 0.25]),
            scales=np.array([0.5, 0.5, 0.5]),
        )
        red_material = OmniPbrMaterial("/World/Materials/cube_red")
        red_material.set_input_values("diffuse_color_constant", [1.0, 0.0, 0.0])
        red_cube_1.apply_visual_materials(red_material)

        blue_cube_2 = Cube(
            "/World/cube_2",
            sizes=1.0,
            positions=np.array([-0.25, -0.25, 0.0]),
            scales=np.array([0.5, 0.5, 0.5]),
        )
        blue_material = OmniPbrMaterial("/World/Materials/cube_blue")
        blue_material.set_input_values("diffuse_color_constant", [0.0, 0.0, 1.0])
        blue_cube_2.apply_visual_materials(blue_material)

        # All cameras will be looking down the -z axis
        camera_positions = [(0.5, 0, 2), (0, 0.5, 2), (-0.5, 0, 2), (0, -0.5, 2)]
        for pos in camera_positions:
            rep.functional.create.camera(position=pos, look_at=(pos[0], pos[1], 0), parent="/World")
        await omni.kit.app.get_app().next_update_async()

    async def _start_data_capture(self, num_warm_up_frames: int = 0) -> None:
        """Start the timeline and optionally wait for valid data to be available.

        Args:
            num_warm_up_frames: Number of warm-up frames to render before capturing data.

        """
        timeline = omni.timeline.get_timeline_interface()
        timeline.play()
        for _ in range(num_warm_up_frames):
            await omni.kit.app.get_app().next_update_async()

    async def _setup_default_camera_view(self) -> "CameraView":
        """Set up the default square resolution camera view used by most tests.

        Returns:
            The configured CameraView instance.

        """
        await self._create_test_environment()
        # Warmup frames
        for _ in range(5):
            await omni.kit.app.get_app().next_update_async()
        camera_view = CameraView(
            prim_paths_expr="/World/Camera*",
            name="camera_prim_view",
            camera_resolution=self.CAMERA_VIEW_RESOLUTION,
            output_annotators=sorted(self.EXPECTED_ANNOTATOR_SPEC.keys()),
        )
        await self._start_data_capture(num_warm_up_frames=5)
        return camera_view

    async def test_tiled_rgb_data(self) -> None:
        """Verify tiled RGB data consistency between CPU/NumPy and CUDA/Torch outputs."""
        camera_view = await self._setup_default_camera_view()
        # cpu / numpy
        rgb_np_tiled_out = np.zeros((*camera_view.tiled_resolution, 3), dtype=np.uint8)
        camera_view.get_rgb_tiled(out=rgb_np_tiled_out, device="cpu")
        rgb_tiled_np = camera_view.get_rgb_tiled(device="cpu")

        self.assertEqual(rgb_np_tiled_out.dtype, rgb_tiled_np.dtype)
        self.assertEqual(rgb_np_tiled_out.shape, rgb_tiled_np.shape)
        self.assertTrue(np.allclose(rgb_np_tiled_out, rgb_tiled_np, atol=1e-5))

        # cuda / torch
        rgb_tiled_torch_out = torch.zeros((*camera_view.tiled_resolution, 3), device="cuda", dtype=torch.uint8)
        camera_view.get_rgb_tiled(out=rgb_tiled_torch_out, device="cuda")
        rgb_tiled_torch = camera_view.get_rgb_tiled(device="cuda")
        # copy output to the same device so we can compare it
        rgb_tiled_torch = rgb_tiled_torch.to(rgb_tiled_torch_out.device)
        self.assertEqual(rgb_tiled_torch_out.dtype, rgb_tiled_torch.dtype)
        self.assertEqual(rgb_tiled_torch_out.shape, rgb_tiled_torch.shape)
        self.assertTrue(torch.allclose(rgb_tiled_torch_out, rgb_tiled_torch, atol=1e-5))

        # Compare numpy and torch outputs as normalized uin8 arrays (images)
        A = (rgb_tiled_np).astype(np.uint8)
        B = (rgb_np_tiled_out).astype(np.uint8)
        C = (rgb_tiled_torch.cpu().numpy()).astype(np.uint8)
        D = (rgb_tiled_torch_out.cpu().numpy()).astype(np.uint8)
        self.assertTrue(np.all([np.allclose(A, B, atol=1), np.allclose(A, C, atol=1), np.allclose(A, D, atol=1)]))

    async def test_tiled_depth_data(self) -> None:
        """Verify tiled depth data consistency between CPU/NumPy and CUDA/Torch outputs."""
        camera_view = await self._setup_default_camera_view()
        # cpu / numpy
        depth_np_tiled_out = np.zeros((*camera_view.tiled_resolution, 1), dtype=np.float32)
        camera_view.get_depth_tiled(out=depth_np_tiled_out, device="cpu")
        depth_tiled_np = camera_view.get_depth_tiled(device="cpu")

        self.assertEqual(depth_np_tiled_out.dtype, depth_tiled_np.dtype)
        self.assertEqual(depth_np_tiled_out.shape, depth_tiled_np.shape)
        self.assertTrue(np.allclose(depth_np_tiled_out, depth_tiled_np, atol=1e-5))

        # cuda / torch
        depth_tiled_torch_out = torch.zeros((*camera_view.tiled_resolution, 1), device="cuda", dtype=torch.float32)
        camera_view.get_depth_tiled(out=depth_tiled_torch_out, device="cuda")
        depth_tiled_torch = camera_view.get_depth_tiled(device="cuda")
        # copy output to the same device so we can compare it
        depth_tiled_torch = depth_tiled_torch.to(depth_tiled_torch_out.device)
        self.assertEqual(depth_tiled_torch_out.dtype, depth_tiled_torch.dtype)
        self.assertEqual(depth_tiled_torch_out.shape, depth_tiled_torch.shape)
        self.assertTrue(torch.allclose(depth_tiled_torch_out, depth_tiled_torch, atol=1e-5))

        # Compare numpy and torch outputs as normalized uin8 arrays (images)
        A = (depth_tiled_np * 255).astype(np.uint8)
        B = (depth_np_tiled_out * 255).astype(np.uint8)
        C = (depth_tiled_torch.cpu().numpy() * 255).astype(np.uint8)
        D = (depth_tiled_torch_out.cpu().numpy() * 255).astype(np.uint8)
        self.assertTrue(np.all([np.allclose(A, B, atol=1), np.allclose(A, C, atol=1), np.allclose(A, D, atol=1)]))

    async def test_tiled_rgb_image(self) -> None:
        """Compare tiled RGB image output against golden reference."""
        camera_view = await self._setup_default_camera_view()
        golden_dir = os.path.join(self.GOLDEN_DIR, "tiled_rgb")
        out_dir = os.path.join(self.TEST_OUTPUT_DIR, "tiled_rgb")
        os.makedirs(out_dir, exist_ok=True)

        image = camera_view.get_rgb_tiled(device="cpu").astype(np.uint8)
        image = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
        cv2.imwrite(os.path.join(out_dir, "camera_view_rgb_tiled.png"), image)

        result = compare_images_in_directories(
            golden_dir=golden_dir,
            test_dir=out_dir,
            path_pattern=r"\.png$",
            allclose_rtol=None,
            allclose_atol=None,
            mean_tolerance=self.RGB_MEAN_DIFF_TOLERANCE,
            print_all_stats=False,
        )
        self.assertTrue(result["all_passed"], f"Image comparison failed. Output dir: {out_dir}")

    async def test_tiled_depth_image(self) -> None:
        """Compare tiled depth image output against golden reference."""
        camera_view = await self._setup_default_camera_view()
        golden_dir = os.path.join(self.GOLDEN_DIR, "tiled_depth")
        out_dir = os.path.join(self.TEST_OUTPUT_DIR, "tiled_depth")
        os.makedirs(out_dir, exist_ok=True)

        image = (camera_view.get_depth_tiled(device="cpu") * 255).astype(np.uint8)
        cv2.imwrite(os.path.join(out_dir, "camera_view_depth_tiled.png"), image)

        result = compare_images_in_directories(
            golden_dir=golden_dir,
            test_dir=out_dir,
            path_pattern=r"\.png$",
            allclose_rtol=None,
            allclose_atol=None,
            mean_tolerance=self.DEPTH_MEAN_DIFF_TOLERANCE,
            print_all_stats=False,
        )
        self.assertTrue(result["all_passed"], f"Image comparison failed. Output dir: {out_dir}")

    async def test_batched_rgb_data(self) -> None:
        """Verify batched RGB data shape, dtype, and pre-allocated output consistency."""
        camera_view = await self._setup_default_camera_view()
        rgb_batched_shape = (len(camera_view.prims), *self.CAMERA_VIEW_RESOLUTION, 3)
        # Make sure the pre-allocated output tensor is on the appropriate cuda device
        cuda_device = str(wp.get_cuda_device())
        print(f"Pre-allocating output tensor of shape {rgb_batched_shape} on cuda device {cuda_device}")
        rgb_batched_out = torch.zeros(rgb_batched_shape, device=cuda_device, dtype=torch.uint8)
        camera_view.get_rgb(out=rgb_batched_out)
        rgb_batched = camera_view.get_rgb()
        self.assertEqual(rgb_batched.dtype, rgb_batched_out.dtype)
        self.assertEqual(rgb_batched.shape, rgb_batched_out.shape)
        # copy output to the same device so we can compare it
        rgb_batched = rgb_batched.to(cuda_device)
        self.assertTrue(torch.allclose(rgb_batched.to(torch.float32), rgb_batched_out.to(torch.float32), atol=1e-5))

    async def test_batched_depth_data(self) -> None:
        """Verify batched depth data shape, dtype, and pre-allocated output consistency."""
        camera_view = await self._setup_default_camera_view()
        depth_batched_shape = (len(camera_view.prims), *self.CAMERA_VIEW_RESOLUTION, 1)
        # Make sure the pre-allocated output tensor is on the appropriate cuda device
        cuda_device = str(wp.get_cuda_device())
        print(f"Pre-allocating output tensor of shape {depth_batched_shape} on cuda device {cuda_device}")
        depth_batched_out = torch.zeros(depth_batched_shape, device=cuda_device, dtype=torch.float32)
        camera_view.get_depth(out=depth_batched_out)
        depth_batched = camera_view.get_depth()
        # copy output to the same device so we can compare it
        depth_batched = depth_batched.to(cuda_device)
        self.assertEqual(depth_batched.dtype, depth_batched_out.dtype)
        self.assertEqual(depth_batched.shape, depth_batched_out.shape)
        self.assertTrue(torch.allclose(depth_batched, depth_batched_out, atol=1e-5))

    async def test_batched_rgb_images(self) -> None:
        """Compare batched RGB images from each camera against golden references."""
        camera_view = await self._setup_default_camera_view()
        golden_dir = os.path.join(self.GOLDEN_DIR, "batched_rgb")
        out_dir = os.path.join(self.TEST_OUTPUT_DIR, "batched_rgb")
        os.makedirs(out_dir, exist_ok=True)

        batch = camera_view.get_rgb()
        for i in range(batch.shape[0]):
            image = (batch[i]).to(dtype=torch.uint8).cpu().numpy()
            image = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
            cv2.imwrite(os.path.join(out_dir, f"camera_view_rgb_batched_{i}.png"), image)

        result = compare_images_in_directories(
            golden_dir=golden_dir,
            test_dir=out_dir,
            path_pattern=r"\.png$",
            allclose_rtol=None,
            allclose_atol=None,
            mean_tolerance=self.RGB_MEAN_DIFF_TOLERANCE,
            print_all_stats=False,
        )
        self.assertTrue(result["all_passed"], f"Image comparison failed. Output dir: {out_dir}")

    async def test_batched_depth_images(self) -> None:
        """Compare batched depth images from each camera against golden references."""
        camera_view = await self._setup_default_camera_view()
        golden_dir = os.path.join(self.GOLDEN_DIR, "batched_depth")
        out_dir = os.path.join(self.TEST_OUTPUT_DIR, "batched_depth")
        os.makedirs(out_dir, exist_ok=True)

        depth_batched = camera_view.get_depth()
        for i in range(depth_batched.shape[0]):
            image = (depth_batched[i] * 255).to(dtype=torch.uint8).cpu().numpy()
            cv2.imwrite(os.path.join(out_dir, f"camera_view_depth_batched_{i}.png"), image)

        result = compare_images_in_directories(
            golden_dir=golden_dir,
            test_dir=out_dir,
            path_pattern=r"\.png$",
            allclose_rtol=None,
            allclose_atol=None,
            mean_tolerance=self.DEPTH_MEAN_DIFF_TOLERANCE,
            print_all_stats=False,
        )
        self.assertTrue(result["all_passed"], f"Image comparison failed. Output dir: {out_dir}")

    async def test_data(self) -> None:
        """Verify batched data shape, dtype, and output buffer for all annotator types."""
        camera_view = await self._setup_default_camera_view()
        for annotator_type in sorted(self.EXPECTED_ANNOTATOR_SPEC.keys()):
            print(f"annotator type: {annotator_type}")
            spec = self.EXPECTED_ANNOTATOR_SPEC[annotator_type]
            data, info = camera_view.get_data(annotator_type)
            # check shape
            shape = (len(camera_view.prims), *self.CAMERA_VIEW_RESOLUTION, spec["channels"])
            self.assertEqual(data.shape, shape, f"{annotator_type} shape {data.shape} != {shape}")
            # check dtype
            dtype = spec["dtype"]
            self.assertEqual(data.dtype, dtype, f"{annotator_type} dtype {data.dtype} != {dtype}")
            # check out
            out = wp.zeros(shape, dtype=dtype, device=data.device)
            camera_view.get_data(annotator_type, out=out)
            # - convert to NumPy to check elements
            data = data.numpy()
            out = out.numpy()
            self.assertTrue(
                np.allclose(data, out),
                f"{annotator_type} data/out mean: {np.mean(data - out)}, std: {np.std(data - out)}",
            )

    async def test_tiled_data(self) -> None:
        """Verify tiled data shape, dtype, and output buffer for all annotator types."""
        camera_view = await self._setup_default_camera_view()
        for annotator_type in sorted(self.EXPECTED_ANNOTATOR_SPEC.keys()):
            print(f"annotator type: {annotator_type}")
            spec = self.EXPECTED_ANNOTATOR_SPEC[annotator_type]
            data, info = camera_view.get_data(annotator_type, tiled=True)
            # check shape
            shape = (*camera_view.tiled_resolution, spec["channels"])
            self.assertEqual(data.shape, shape, f"{annotator_type} shape {data.shape} != {shape}")
            # check dtype
            dtype = spec["dtype"]
            self.assertEqual(data.dtype, dtype, f"{annotator_type} dtype {data.dtype} != {dtype}")
            # check out
            out = wp.zeros(shape, dtype=dtype, device=data.device)
            camera_view.get_data(annotator_type, tiled=True, out=out)
            # - convert to NumPy to check elements
            data = data.numpy()
            out = out.numpy()
            self.assertTrue(
                np.allclose(data, out),
                f"{annotator_type} data/out mean: {np.mean(data - out)}, std: {np.std(data - out)}",
            )

    async def test_properties(self) -> None:
        """Verify camera count and test setting/getting camera properties."""
        camera_view = await self._setup_default_camera_view()
        num_cameras = len(camera_view.prims)
        self.assertEqual(num_cameras, self.NUM_CAMERAS)
        camera_view.set_focal_lengths([5.0] * num_cameras)
        self.assertTrue(np.isclose(camera_view.get_focal_lengths(), [5.0] * num_cameras, atol=1e-05).all())
        camera_view.set_focus_distances([0.01] * num_cameras)
        self.assertTrue(np.isclose(camera_view.get_focus_distances(), [0.01] * num_cameras, atol=1e-05).all())
        camera_view.set_lens_apertures([0.01] * num_cameras)
        self.assertTrue(np.isclose(camera_view.get_lens_apertures(), [0.01] * num_cameras, atol=1e-05).all())
        camera_view.set_horizontal_apertures([1.2] * num_cameras)
        self.assertTrue(np.isclose(camera_view.get_horizontal_apertures(), [1.2] * num_cameras, atol=1e-05).all())
        camera_view.set_vertical_apertures([1.2] * num_cameras)
        self.assertTrue(np.isclose(camera_view.get_vertical_apertures(), [1.2] * num_cameras, atol=1e-05).all())
        camera_view.set_projection_types(["fisheyeOrthographic"] * num_cameras)
        self.assertTrue(camera_view.get_projection_types() == ["fisheyeOrthographic"] * num_cameras)

    async def test_non_square_resolution(self) -> None:
        """Test that non-square resolutions are handled correctly (width != height)."""
        await self._create_test_environment()
        non_square_resolution = (320, 240)  # width=320, height=240
        width, height = non_square_resolution

        camera_view = CameraView(
            prim_paths_expr="/World/Camera*",
            name="camera_view_non_square",
            camera_resolution=non_square_resolution,
            output_annotators=["rgb", "depth"],
        )

        await self._start_data_capture(num_warm_up_frames=5)

        num_cameras = len(camera_view.prims)

        # Batched data should have shape (num_cameras, height, width, channels)
        rgb_data, _ = camera_view.get_data("rgb")
        expected_rgb_shape = (num_cameras, height, width, 3)
        self.assertEqual(
            rgb_data.shape,
            expected_rgb_shape,
            f"RGB batched shape {rgb_data.shape} != expected {expected_rgb_shape}",
        )

        depth_data, _ = camera_view.get_data("depth")
        expected_depth_shape = (num_cameras, height, width, 1)
        self.assertEqual(
            depth_data.shape,
            expected_depth_shape,
            f"Depth batched shape {depth_data.shape} != expected {expected_depth_shape}",
        )

        # Aspect ratio should be width/height
        expected_aspect_ratio = float(width) / float(height)
        actual_aspect_ratio = camera_view.get_aspect_ratios()
        self.assertAlmostEqual(
            actual_aspect_ratio,
            expected_aspect_ratio,
            places=5,
            msg=f"Aspect ratio {actual_aspect_ratio} != expected {expected_aspect_ratio}",
        )
