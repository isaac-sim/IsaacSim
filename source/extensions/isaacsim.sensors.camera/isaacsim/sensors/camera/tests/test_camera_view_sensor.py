# SPDX-FileCopyrightText: Copyright (c) 2024-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

import asyncio
import os

import carb
import cv2
import numpy as np
import omni.kit.test
import omni.replicator.core as rep
import torch
import warp as wp
from isaacsim.core.api import World
from isaacsim.core.api.objects import VisualCuboid
from isaacsim.core.utils.stage import create_new_stage_async, update_stage_async
from isaacsim.sensors.camera.camera_view import ANNOTATOR_SPEC, CameraView
from isaacsim.sensors.camera.tests.utils import compare_images, save_image

SAVE_IMAGE_AS_TEST = False
SAVE_IMAGE_AS_GOLDEN = False

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
assert sorted(ANNOTATOR_SPEC.keys()) == sorted(EXPECTED_ANNOTATOR_SPEC.keys())

IMG_COMPARISON_TOLERANCE = 0.92


class TestCameraViewSensor(omni.kit.test.AsyncTestCase):
    # Before running each test
    async def setUp(self):
        await create_new_stage_async()
        self.my_world = World(stage_units_in_meters=1.0)
        await self.my_world.initialize_simulation_context_async()
        self.my_world.scene.add_default_ground_plane()

        # Add a red and blue cube
        self.cube_1 = self.my_world.scene.add(
            VisualCuboid(
                prim_path="/new_cube_1",
                name="cube_1",
                position=np.array([0.25, 0.25, 0.25]),
                scale=np.array([0.5, 0.5, 0.5]),
                size=1.0,
                color=np.array([255, 0, 0]),
            )
        )
        self.cube_2 = self.my_world.scene.add(
            VisualCuboid(
                prim_path="/new_cube_2",
                name="cube_2",
                position=np.array([-0.25, -0.25, 0.0]),
                scale=np.array([0.5, 0.5, 0.5]),
                size=1.0,
                color=np.array([0, 0, 255]),
            )
        )
        # rep.create.plane(scale=(10, 10, 1))

        # All cameras will be looking down the -z axis
        camera_positions = [(0.5, 0, 2), (0, 0.5, 2), (-0.5, 0, 2), (0, -0.5, 2)]
        for pos in camera_positions:
            rep.create.camera(position=pos, look_at=(pos[0], pos[1], 0))
        await omni.kit.app.get_app().next_update_async()
        self.num_cameras = len(camera_positions)

        self.resolution = (256, 256)
        self.camera_view = CameraView(
            prim_paths_expr="/Replicator/Camera_Xform*/Camera",
            name="camera_prim_view",
            camera_resolution=self.resolution,
            output_annotators=sorted(list(EXPECTED_ANNOTATOR_SPEC.keys())),
        )

        await self.my_world.reset_async()
        # Warmup
        for _ in range(20):
            await update_stage_async()

        self.golden_dir = os.path.join(os.path.dirname(os.path.realpath(__file__)), "data", "golden")
        self.test_dir = carb.tokens.get_tokens_interface().resolve("${temp}/test_camera_view_sensor")

    # After running each test
    async def tearDown(self):
        self.camera_view = None
        self.my_world.clear_instance()
        await omni.kit.app.get_app().next_update_async()
        while omni.usd.get_context().get_stage_loading_status()[2] > 0:
            print("tearDown, assets still loading, waiting to finish...")
            await asyncio.sleep(0.25)
        await omni.kit.app.get_app().next_update_async()
        return

    async def test_tiled_rgb_data(self):
        # cpu / numpy
        rgb_np_tiled_out = np.zeros((*self.camera_view.tiled_resolution, 3), dtype=np.uint8)
        self.camera_view.get_rgb_tiled(out=rgb_np_tiled_out, device="cpu")
        rgb_tiled_np = self.camera_view.get_rgb_tiled(device="cpu")

        self.assertEqual(rgb_np_tiled_out.dtype, rgb_tiled_np.dtype)
        self.assertEqual(rgb_np_tiled_out.shape, rgb_tiled_np.shape)
        self.assertTrue(np.allclose(rgb_np_tiled_out, rgb_tiled_np, atol=1e-5))

        # cuda / torch
        rgb_tiled_torch_out = torch.zeros((*self.camera_view.tiled_resolution, 3), device="cuda", dtype=torch.uint8)
        self.camera_view.get_rgb_tiled(out=rgb_tiled_torch_out, device="cuda")
        rgb_tiled_torch = self.camera_view.get_rgb_tiled(device="cuda")
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

    async def test_tiled_depth_data(self):
        # cpu / numpy
        depth_np_tiled_out = np.zeros((*self.camera_view.tiled_resolution, 1), dtype=np.float32)
        self.camera_view.get_depth_tiled(out=depth_np_tiled_out, device="cpu")
        depth_tiled_np = self.camera_view.get_depth_tiled(device="cpu")

        self.assertEqual(depth_np_tiled_out.dtype, depth_tiled_np.dtype)
        self.assertEqual(depth_np_tiled_out.shape, depth_tiled_np.shape)
        self.assertTrue(np.allclose(depth_np_tiled_out, depth_tiled_np, atol=1e-5))

        # cuda / torch
        depth_tiled_torch_out = torch.zeros((*self.camera_view.tiled_resolution, 1), device="cuda", dtype=torch.float32)
        self.camera_view.get_depth_tiled(out=depth_tiled_torch_out, device="cuda")
        depth_tiled_torch = self.camera_view.get_depth_tiled(device="cuda")
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

    async def test_tiled_rgb_image(self):
        image = self.camera_view.get_rgb_tiled(device="cpu").astype(np.uint8)
        image = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
        save_image(
            image,
            f"camera_view_rgb_tiled.png",
            self.golden_dir,
            self.test_dir,
            save_as_golden=SAVE_IMAGE_AS_GOLDEN,
            save_as_test=SAVE_IMAGE_AS_TEST,
        )
        # load golden image
        golden_img_path = os.path.join(self.golden_dir, "camera_view_rgb_tiled.png")
        golden_img = cv2.imread(golden_img_path, cv2.IMREAD_UNCHANGED)
        # check
        score = compare_images(golden_img, image)
        self.assertTrue(
            score[0] > IMG_COMPARISON_TOLERANCE,
            f"comparison score ({score[0]}, ({score[1]}, {score[2]})) < {IMG_COMPARISON_TOLERANCE}",
        )

    async def test_tiled_depth_image(self):
        image = (self.camera_view.get_depth_tiled(device="cpu") * 255).astype(np.uint8)
        save_image(
            image,
            f"camera_view_depth_tiled.png",
            self.golden_dir,
            self.test_dir,
            save_as_golden=SAVE_IMAGE_AS_GOLDEN,
            save_as_test=SAVE_IMAGE_AS_TEST,
        )
        # load golden image
        golden_img_path = os.path.join(self.golden_dir, "camera_view_depth_tiled.png")
        golden_img = cv2.imread(golden_img_path, cv2.IMREAD_UNCHANGED)
        # check
        score = compare_images(golden_img, image, hist_div=3)
        self.assertTrue(
            score[0] > IMG_COMPARISON_TOLERANCE,
            f"comparison score ({score[0]}, ({score[1]}, {score[2]})) < {IMG_COMPARISON_TOLERANCE}",
        )

    async def test_batched_rgb_data(self):
        rgb_batched_shape = (self.num_cameras, *self.resolution, 3)
        # Make sure the pre-allocated output tensor is on the appropriate cuda device
        cuda_device = str(wp.get_cuda_device())
        print(f"Pre-allocating output tensor of shape {rgb_batched_shape} on cuda device {cuda_device}")
        rgb_batched_out = torch.zeros(rgb_batched_shape, device=cuda_device, dtype=torch.uint8)
        self.camera_view.get_rgb(out=rgb_batched_out)
        rgb_batched = self.camera_view.get_rgb()
        self.assertEqual(rgb_batched.dtype, rgb_batched_out.dtype)
        self.assertEqual(rgb_batched.shape, rgb_batched_out.shape)
        # copy output to the same device so we can compare it
        rgb_batched = rgb_batched.to(cuda_device)
        self.assertTrue(torch.allclose(rgb_batched.to(torch.float32), rgb_batched_out.to(torch.float32), atol=1e-5))

    async def test_batched_depth_data(self):
        depth_batched_shape = (self.num_cameras, *self.resolution, 1)
        # Make sure the pre-allocated output tensor is on the appropriate cuda device
        cuda_device = str(wp.get_cuda_device())
        print(f"Pre-allocating output tensor of shape {depth_batched_shape} on cuda device {cuda_device}")
        depth_batched_out = torch.zeros(depth_batched_shape, device=cuda_device, dtype=torch.float32)
        self.camera_view.get_depth(out=depth_batched_out)
        depth_batched = self.camera_view.get_depth()
        # copy output to the same device so we can compare it
        depth_batched = depth_batched.to(cuda_device)
        self.assertEqual(depth_batched.dtype, depth_batched_out.dtype)
        self.assertEqual(depth_batched.shape, depth_batched_out.shape)
        self.assertTrue(torch.allclose(depth_batched, depth_batched_out, atol=1e-5))

    async def test_batched_rgb_images(self):
        batch = self.camera_view.get_rgb()
        for i in range(batch.shape[0]):
            image = (batch[i]).to(dtype=torch.uint8).cpu().numpy()
            image = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
            save_image(
                image,
                f"camera_view_rgb_batched_{i}.png",
                self.golden_dir,
                self.test_dir,
                save_as_golden=SAVE_IMAGE_AS_GOLDEN,
                save_as_test=SAVE_IMAGE_AS_TEST,
            )
            # load golden image
            golden_img_path = os.path.join(self.golden_dir, f"camera_view_rgb_batched_{i}.png")
            golden_img = cv2.imread(golden_img_path, cv2.IMREAD_UNCHANGED)
            # check
            score = compare_images(golden_img, image)
            self.assertTrue(
                score[0] > IMG_COMPARISON_TOLERANCE,
                f"camera {i}: comparison score ({score[0]}, ({score[1]}, {score[2]})) < {IMG_COMPARISON_TOLERANCE}",
            )

    async def test_batched_depth_images(self):
        depth_batched = self.camera_view.get_depth()
        for i in range(depth_batched.shape[0]):
            image = (depth_batched[i] * 255).to(dtype=torch.uint8).cpu().numpy()
            save_image(
                image,
                f"camera_view_depth_batched_{i}.png",
                self.golden_dir,
                self.test_dir,
                save_as_golden=SAVE_IMAGE_AS_GOLDEN,
                save_as_test=SAVE_IMAGE_AS_TEST,
            )
            # load golden image
            golden_img_path = os.path.join(self.golden_dir, f"camera_view_depth_batched_{i}.png")
            golden_img = cv2.imread(golden_img_path, cv2.IMREAD_UNCHANGED)
            # check
            score = compare_images(golden_img, image, hist_div=3)
            self.assertTrue(
                score[0] > IMG_COMPARISON_TOLERANCE,
                f"camera {i}: comparison score ({score[0]}, ({score[1]}, {score[2]})) < {IMG_COMPARISON_TOLERANCE}",
            )

    async def test_data(self):
        for annotator_type in sorted(list(EXPECTED_ANNOTATOR_SPEC.keys())):
            print(f"annotator type: {annotator_type}")
            spec = EXPECTED_ANNOTATOR_SPEC[annotator_type]
            data, info = self.camera_view.get_data(annotator_type)
            # check shape
            shape = (self.num_cameras, *self.resolution, spec["channels"])
            self.assertEqual(data.shape, shape, f"{annotator_type} shape {data.shape} != {shape}")
            # check dtype
            dtype = spec["dtype"]
            self.assertEqual(data.dtype, dtype, f"{annotator_type} dtype {data.dtype} != {dtype}")
            # check out
            out = wp.zeros(shape, dtype=dtype, device=data.device)
            self.camera_view.get_data(annotator_type, out=out)
            # - convert to NumPy to check elements
            data = data.numpy()
            out = out.numpy()
            self.assertTrue(
                np.allclose(data, out),
                f"{annotator_type} data/out mean: {np.mean(data - out)}, std: {np.std(data - out)}",
            )

    async def test_tiled_data(self):
        for annotator_type in sorted(list(EXPECTED_ANNOTATOR_SPEC.keys())):
            print(f"annotator type: {annotator_type}")
            spec = EXPECTED_ANNOTATOR_SPEC[annotator_type]
            data, info = self.camera_view.get_data(annotator_type, tiled=True)
            # check shape
            shape = (*self.camera_view.tiled_resolution, spec["channels"])
            self.assertEqual(data.shape, shape, f"{annotator_type} shape {data.shape} != {shape}")
            # check dtype
            dtype = spec["dtype"]
            self.assertEqual(data.dtype, dtype, f"{annotator_type} dtype {data.dtype} != {dtype}")
            # check out
            out = wp.zeros(shape, dtype=dtype, device=data.device)
            self.camera_view.get_data(annotator_type, tiled=True, out=out)
            # - convert to NumPy to check elements
            data = data.numpy()
            out = out.numpy()
            self.assertTrue(
                np.allclose(data, out),
                f"{annotator_type} data/out mean: {np.mean(data - out)}, std: {np.std(data - out)}",
            )

    async def test_properties(self):
        self.assertTrue(self.num_cameras == len(self.camera_view.prims))
        self.camera_view.set_focal_lengths([5.0] * 4)
        self.assertTrue(np.isclose(self.camera_view.get_focal_lengths(), [5.0] * 4, atol=1e-05).all())
        self.camera_view.set_focus_distances([0.01] * 4)
        self.assertTrue(np.isclose(self.camera_view.get_focus_distances(), [0.01] * 4, atol=1e-05).all())
        self.camera_view.set_lens_apertures([0.01] * 4)
        self.assertTrue(np.isclose(self.camera_view.get_lens_apertures(), [0.01] * 4, atol=1e-05).all())
        self.camera_view.set_horizontal_apertures([1.2] * 4)
        self.assertTrue(np.isclose(self.camera_view.get_horizontal_apertures(), [1.2] * 4, atol=1e-05).all())
        self.camera_view.set_vertical_apertures([1.2] * 4)
        self.assertTrue(np.isclose(self.camera_view.get_vertical_apertures(), [1.2] * 4, atol=1e-05).all())
        self.camera_view.set_projection_types(["fisheyeOrthographic"] * 4)
        self.assertTrue(self.camera_view.get_projection_types() == ["fisheyeOrthographic"] * 4)
        return
