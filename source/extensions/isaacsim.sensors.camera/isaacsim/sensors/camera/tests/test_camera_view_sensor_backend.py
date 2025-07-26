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

import numpy as np
import omni.kit.app
import omni.usd
from isaacsim.core.api import World
from isaacsim.core.api.objects import FixedCuboid
from isaacsim.core.utils.stage import create_new_stage_async, update_stage_async
from isaacsim.sensors.camera import CameraView
from pxr import UsdGeom


class TestCameraViewSensorBackend(omni.kit.test.AsyncTestCase):
    # Before running each test
    async def setUp(self):
        await create_new_stage_async()
        for _ in range(3):
            await update_stage_async()

    # After running each test
    async def tearDown(self):
        await omni.kit.app.get_app().next_update_async()
        while omni.usd.get_context().get_stage_loading_status()[2] > 0:
            print("tearDown, assets still loading, waiting to finish...")
            await asyncio.sleep(0.25)
        await omni.kit.app.get_app().next_update_async()
        return

    async def run_test_async(self, world_backend="numpy", world_device=None, gpu_dynamics=False, app_warmup=False):
        await omni.usd.get_context().new_stage_async()
        stage = omni.usd.get_context().get_stage()

        world = World(backend=world_backend, device=world_device)
        await world.initialize_simulation_context_async()

        if gpu_dynamics:
            world.get_physics_context().enable_gpu_dynamics(True)
            world.get_physics_context().set_broadphase_type("GPU")

        # Visuals are not important for this test, so we skip them
        # world.scene.add_default_ground_plane()
        # cube = FixedCuboid(prim_path="/World/cube", name="cube", position=[0, 0, 1], color=np.array([255, 0, 0]))

        num_cameras = 4
        for i in range(num_cameras):
            camera_prim = stage.DefinePrim(f"/World/Camera_{i+1}", "Camera")
            UsdGeom.Xformable(camera_prim).AddTranslateOp().Set((0, i * 0.5, 0.2))
            UsdGeom.Xformable(camera_prim).AddRotateXYZOp().Set((0, 0, 0))

        camera_view = CameraView(
            name="camera_view",
            camera_resolution=(300, 300),
            prim_paths_expr="/World/Camera_*",
        )
        self.assertIsNotNone(camera_view, "CameraView could not be created")
        self.assertEqual(
            len(camera_view.prims), num_cameras, f"Expected {num_cameras} cameras, got {len(camera_view.prims)}"
        )

        world.reset()

        if app_warmup:
            for i in range(5):
                await omni.kit.app.get_app().next_update_async()

        # Make sure there are no errors capturing the data
        for i in range(3):
            print(f"Capture step {i}")
            world.step_async()
            await omni.kit.app.get_app().next_update_async()

            rgb_tiled = camera_view.get_rgb_tiled()
            print(f"\tRGBA tiled shape: {rgb_tiled.shape}, dtype: {rgb_tiled.dtype}")
            self.assertIsNotNone(rgb_tiled, "RGBA tiled data could not be captured")

            rgb = camera_view.get_rgb()
            print(f"\tRGB shape: {rgb.shape}, dtype: {rgb.dtype}")
            self.assertIsNotNone(rgb, "RGB data could not be captured")

            depth_tiled = camera_view.get_depth_tiled()
            print(f"\tDepth tiled shape: {depth_tiled.shape}, dtype: {depth_tiled.dtype}")
            self.assertIsNotNone(depth_tiled, "Depth tiled data could not be captured")

            depth = camera_view.get_depth()
            print(f"\tDepth shape: {depth.shape}, dtype: {depth.dtype}")
            self.assertIsNotNone(depth, "Depth data could not be captured")

        world.clear_instance()

    # numpy backend
    async def test_numpy_cpu_nogpu_nowarmup(self):
        await self.run_test_async(world_backend="numpy", world_device=None, gpu_dynamics=False, app_warmup=False)

    async def test_numpy_cpu_nogpu_warmup(self):
        await self.run_test_async(world_backend="numpy", world_device=None, gpu_dynamics=False, app_warmup=True)

    async def test_numpy_cpu_gpu_nowarmup(self):
        await self.run_test_async(world_backend="numpy", world_device=None, gpu_dynamics=True, app_warmup=False)

    async def test_numpy_cpu_gpu_warmup(self):
        await self.run_test_async(world_backend="numpy", world_device=None, gpu_dynamics=True, app_warmup=True)

    async def test_numpy_cuda_nogpu_nowarmup(self):
        await self.run_test_async(world_backend="numpy", world_device="cuda", gpu_dynamics=False, app_warmup=False)

    async def test_numpy_cuda_nogpu_warmup(self):
        await self.run_test_async(world_backend="numpy", world_device="cuda", gpu_dynamics=False, app_warmup=True)

    async def test_numpy_cuda_gpu_nowarmup(self):
        await self.run_test_async(world_backend="numpy", world_device="cuda", gpu_dynamics=True, app_warmup=False)

    async def test_numpy_cuda_gpu_warmup(self):
        await self.run_test_async(world_backend="numpy", world_device="cuda", gpu_dynamics=True, app_warmup=True)

    # torch backend
    async def test_torch_cpu_nogpu_nowarmup(self):
        await self.run_test_async(world_backend="torch", world_device=None, gpu_dynamics=False, app_warmup=False)

    async def test_torch_cpu_nogpu_warmup(self):
        await self.run_test_async(world_backend="torch", world_device=None, gpu_dynamics=False, app_warmup=True)

    async def test_torch_cpu_gpu_nowarmup(self):
        await self.run_test_async(world_backend="torch", world_device=None, gpu_dynamics=True, app_warmup=False)

    async def test_torch_cpu_gpu_warmup(self):
        await self.run_test_async(world_backend="torch", world_device=None, gpu_dynamics=True, app_warmup=True)

    async def test_torch_cuda_nogpu_nowarmup(self):
        await self.run_test_async(world_backend="torch", world_device="cuda", gpu_dynamics=False, app_warmup=False)

    async def test_torch_cuda_nogpu_warmup(self):
        await self.run_test_async(world_backend="torch", world_device="cuda", gpu_dynamics=False, app_warmup=True)

    async def test_torch_cuda_gpu_nowarmup(self):
        await self.run_test_async(world_backend="torch", world_device="cuda", gpu_dynamics=True, app_warmup=False)

    async def test_torch_cuda_gpu_warmup(self):
        await self.run_test_async(world_backend="torch", world_device="cuda", gpu_dynamics=True, app_warmup=True)

    # warp backend
    # async def test_warp_cpu_nogpu_nowarmup(self):
    #     await self.run_test_async(world_backend="warp", world_device=None, gpu_dynamics=False, app_warmup=False)

    # async def test_warp_cpu_nogpu_warmup(self):
    #     await self.run_test_async(world_backend="warp", world_device=None, gpu_dynamics=False, app_warmup=True)

    # async def test_warp_cpu_gpu_nowarmup(self):
    #     await self.run_test_async(world_backend="warp", world_device=None, gpu_dynamics=True, app_warmup=False)

    # async def test_warp_cpu_gpu_warmup(self):
    #     await self.run_test_async(world_backend="warp", world_device=None, gpu_dynamics=True, app_warmup=True)

    # async def test_warp_cuda_nogpu_nowarmup(self):
    #     await self.run_test_async(world_backend="warp", world_device="cuda", gpu_dynamics=False, app_warmup=False)

    # async def test_warp_cuda_nogpu_warmup(self):
    #     await self.run_test_async(world_backend="warp", world_device="cuda", gpu_dynamics=False, app_warmup=True)

    # async def test_warp_cuda_gpu_nowarmup(self):
    #     await self.run_test_async(world_backend="warp", world_device="cuda", gpu_dynamics=True, app_warmup=False)

    # async def test_warp_cuda_gpu_warmup(self):
    #     await self.run_test_async(world_backend="warp", world_device="cuda", gpu_dynamics=True, app_warmup=True)
