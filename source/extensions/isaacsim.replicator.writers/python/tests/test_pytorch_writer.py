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

"""Verifies PytorchWriter tensor output for single and multiple render products on CPU and CUDA. The tests cover in-memory listener delivery, optional PNG backend writing, tensor shapes, and multi-camera output organization."""

import os
import tempfile
import unittest
from pathlib import Path

import carb
import isaacsim.core.experimental.utils.stage as stage_utils
import numpy as np
import omni.kit
import omni.replicator.core as rep
import omni.usd
from isaacsim.replicator.writers import PytorchListener
from PIL import Image


class TestMultipleRenderProducts(omni.kit.test.AsyncTestCase):
    """Test suite for validating PytorchWriter functionality with single and multiple render products.

    Tests the PytorchWriter's ability to handle various configurations including single and multiple cameras,
    with and without backend file writing, and on both CPU and GPU devices. The test suite creates a synthetic
    scene with randomized geometric shapes (torus, sphere, cube) and multiple camera viewpoints to generate
    diverse rendering data for validation.

    Each test method verifies specific aspects of the PytorchWriter:
    - Data tensor shapes and device placement (CPU/GPU)
    - Backend file writing capabilities
    - Multi-camera batched processing
    - Cross-validation between in-memory tensors and saved image files

    The test environment includes three render products with cameras positioned at different viewpoints,
    ensuring comprehensive coverage of multi-camera scenarios commonly used in synthetic data generation
    workflows.
    """

    async def setUp(self) -> None:
        """Set up test fixtures with render products, cameras, and scene objects.

        Creates a new stage with three cameras at different positions, each with 512x512 render products.
        Populates the scene with randomized torus, sphere, and cube objects with semantic labels.
        Prepares output directory for backend writing tests.
        """
        # Create new stage
        await omni.kit.app.get_app().next_update_async()
        await stage_utils.create_new_stage_async()

        # Create camera and render products
        render_product1 = rep.create.render_product(rep.create.camera(position=(0, 0, 1000)), (512, 512))
        render_product2 = rep.create.render_product(rep.create.camera(position=(100, 100, 1000)), (512, 512))
        render_product3 = rep.create.render_product(rep.create.camera(position=(200, -100, 1000)), (512, 512))
        self.render_products = [render_product1, render_product2, render_product3]

        stage = omni.usd.get_context().get_stage()

        # Setup scene with randomized shapes
        torus = rep.create.torus(semantics=[("class", "torus"), ("alias", "donut")])
        sphere = rep.create.sphere(semantics=[("class", "sphere"), ("alias", "ball")])
        cube = rep.create.cube(semantics=[("class", "cube")])

        test_seed = 2134
        with rep.trigger.on_frame():
            with rep.create.group([torus, sphere, cube]):
                rep.modify.pose(
                    position=rep.distribution.uniform((-100, -100, -100), (200, 200, 200), seed=test_seed),
                    scale=rep.distribution.uniform(0.1, 2, seed=test_seed),
                )
                rep.randomizer.rotation(seed=test_seed)

        # Output directory for backend writing
        self.out_dir = tempfile.mkdtemp(prefix="test_pytorch_writer_")
        print(f"Output directory: {self.out_dir}")

        await omni.kit.app.get_app().next_update_async()

    async def tearDown(self) -> None:
        """Clean up test fixtures by destroying render products and closing the stage."""
        await omni.kit.app.get_app().next_update_async()
        for rp in self.render_products:
            rp.destroy()
            rp = None
            await omni.kit.app.get_app().next_update_async()
        self.render_products = None
        await omni.kit.app.get_app().next_update_async()
        omni.usd.get_context().close_stage()

    async def _run_until_stopped(self) -> None:
        """Run the replicator orchestrator for 10 frames until completion."""
        await rep.orchestrator.run_until_complete_async(num_frames=10)

    @unittest.skip("Skipping test as PytorchWriter is deprecated")
    async def test_single_camera_writer_without_backend(self) -> None:
        """Test PytorchWriter with a single camera using CPU device without file output.

        Verifies that the writer produces a tensor with correct shape [1, 3, 512, 512] on CPU device.
        """
        render_products = self.render_products[1:2]
        pytorch_listener = PytorchListener()
        pytorch_writer = rep.WriterRegistry.get("PytorchWriter")
        pytorch_writer.initialize(listener=pytorch_listener, device="cpu")
        pytorch_writer.attach(render_products)

        await self._run_until_stopped()

        image = pytorch_listener.get_rgb_data()
        self.assertTrue(image.device.type == "cpu")
        self.assertTrue([*image.shape] == [1, 3, 512, 512])
        pytorch_writer.detach()
        pytorch_listener = None

    @unittest.skip("Skipping test as PytorchWriter is deprecated")
    async def test_single_camera_writer_with_backend(self) -> None:
        """Test PytorchWriter with a single camera using CPU device and file output.

        Verifies that the writer produces both in-memory tensor data and saves PNG files to disk.
        Compares the tensor data with the saved image file to ensure consistency.
        """
        render_products = self.render_products[0:1]
        pytorch_listener = PytorchListener()
        pytorch_writer = rep.WriterRegistry.get("PytorchWriter")

        out_dir = os.path.join(self.out_dir, "single_cam")

        pytorch_writer.initialize(output_dir=out_dir, listener=pytorch_listener, device="cpu")
        pytorch_writer.attach(render_products)

        await self._run_until_stopped()

        image = pytorch_listener.get_rgb_data()
        file_path = os.path.join(out_dir, "rgb_9_LdrColor.png")
        self.assertTrue(Path(file_path).exists())

        # convert arrays/tensors to (W, H, R) format
        file_image = np.asarray(Image.open(file_path))[:, :, :3]
        torch_to_numpy = image.numpy().transpose(0, 2, 3, 1).squeeze()

        self.assertTrue((file_image == torch_to_numpy).all())
        pytorch_writer.detach()
        pytorch_listener = None

    @unittest.skip("Skipping test as PytorchWriter is deprecated")
    async def test_multiple_cameras_writer_without_backend(self) -> None:
        """Test PytorchWriter with multiple cameras using CPU device without file output.

        Verifies that the writer produces a tensor with correct shape [3, 3, 512, 512] on CPU device
        for three render products.
        """
        render_products = self.render_products
        pytorch_listener = PytorchListener()
        pytorch_writer = rep.WriterRegistry.get("PytorchWriter")
        pytorch_writer.initialize(listener=pytorch_listener, device="cpu")
        pytorch_writer.attach(render_products)

        await self._run_until_stopped()

        images = pytorch_listener.get_rgb_data()
        self.assertTrue(images.device.type == "cpu")
        self.assertTrue([*images.shape] == [3, 3, 512, 512])
        pytorch_writer.detach()
        pytorch_listener = None

    @unittest.skip("Skipping test as PytorchWriter is deprecated")
    async def test_multiple_cameras_writer_with_backend(self) -> None:
        """Test PytorchWriter with multiple cameras using CPU device and file output.

        Verifies that the writer produces both in-memory tensor data and saves PNG files for each camera.
        Compares the concatenated tensor data with the saved image files to ensure consistency.
        """
        render_products = self.render_products
        pytorch_listener = PytorchListener()
        pytorch_writer = rep.WriterRegistry.get("PytorchWriter")

        out_dir = os.path.join(self.out_dir, "multi_cam")

        pytorch_writer.initialize(output_dir=out_dir, listener=pytorch_listener, device="cpu")
        pytorch_writer.attach(render_products)

        await self._run_until_stopped()

        image = pytorch_listener.get_rgb_data()
        file_paths = []
        rp_prefix = carb.settings.get_settings().get_as_string("/exts/omni.kit.hydra_texture/renderProduct/path/prefix")
        for rp in render_products:
            name = rp.path.split(rp_prefix)[-1]
            path = os.path.join(out_dir, f"rgb_9_{name}.png")
            file_paths.append(path)
            self.assertTrue(Path(path).exists())

        # convert arrays/tensors to (Batches, W, H, R) format

        file_image_one = np.expand_dims(np.asarray(Image.open(file_paths[0]))[:, :, :3], axis=0)
        file_image_two = np.expand_dims(np.asarray(Image.open(file_paths[1]))[:, :, :3], axis=0)
        file_image_three = np.expand_dims(np.asarray(Image.open(file_paths[2]))[:, :, :3], axis=0)

        concatenated_images = np.concatenate((file_image_one, file_image_two, file_image_three), axis=0)
        torch_to_numpy = image.numpy().transpose(0, 2, 3, 1)

        self.assertTrue((concatenated_images == torch_to_numpy).all())
        pytorch_writer.detach()
        pytorch_listener = None

    @unittest.skip("Skipping test as PytorchWriter is deprecated")
    async def test_single_camera_writer_with_gpu(self) -> None:
        """Test PytorchWriter with a single camera using CUDA GPU device.

        Verifies that the writer produces a tensor with correct shape [1, 3, 512, 512] on CUDA device.
        Skipped if GPU is not available on the machine.
        """
        render_products = self.render_products[2:3]
        pytorch_listener = PytorchListener()
        pytorch_writer = rep.WriterRegistry.get("PytorchWriter")
        pytorch_writer.initialize(listener=pytorch_listener, device="cuda")
        pytorch_writer.attach(render_products)

        await self._run_until_stopped()

        image = pytorch_listener.get_rgb_data()
        self.assertTrue(image.device.type == "cuda")
        self.assertTrue([*image.shape] == [1, 3, 512, 512])
        pytorch_writer.detach()
        pytorch_listener = None

    @unittest.skip("Skipping test as PytorchWriter is deprecated")
    async def test_multiple_cameras_writer_with_gpu(self) -> None:
        """Test PytorchWriter with multiple cameras using CUDA GPU device.

        Verifies that the writer produces a tensor with correct shape [3, 3, 512, 512] on CUDA device
        for three render products. Skipped if GPU is not available on the machine.
        """
        render_products = self.render_products
        pytorch_listener = PytorchListener()
        pytorch_writer = rep.WriterRegistry.get("PytorchWriter")
        pytorch_writer.initialize(listener=pytorch_listener, device="cuda")
        pytorch_writer.attach(render_products)

        await self._run_until_stopped()

        images = pytorch_listener.get_rgb_data()
        self.assertTrue(images.device.type == "cuda")
        self.assertTrue([*images.shape] == [3, 3, 512, 512])
        pytorch_writer.detach()
        pytorch_listener = None
