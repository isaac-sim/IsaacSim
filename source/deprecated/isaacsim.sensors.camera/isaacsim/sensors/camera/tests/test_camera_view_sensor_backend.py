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

"""Tests for camera view sensor backend configurations."""

import omni.kit.app
import omni.kit.test
import omni.timeline
import omni.usd
from isaacsim.core.experimental.objects import DomeLight, GroundPlane
from isaacsim.core.experimental.utils.stage import create_new_stage_async
from isaacsim.core.simulation_manager import SimulationManager
from isaacsim.sensors.camera import CameraView
from pxr import UsdGeom


class TestCameraViewSensorBackend(omni.kit.test.AsyncTestCase):
    """Tests CameraView sensor with different SimulationManager backend configurations."""

    async def setUp(self) -> None:
        """Set up test fixtures."""
        await omni.kit.app.get_app().next_update_async()
        await create_new_stage_async()
        await omni.kit.app.get_app().next_update_async()

    async def tearDown(self) -> None:
        """Tear down test fixtures."""
        timeline = omni.timeline.get_timeline_interface()
        timeline.stop()
        SimulationManager.set_backend("numpy")
        SimulationManager.set_device("cpu")
        omni.usd.get_context().close_stage()
        await omni.kit.app.get_app().next_update_async()
        while omni.usd.get_context().get_stage_loading_status()[2] > 0:
            await omni.kit.app.get_app().next_update_async()

    async def run_test_async(
        self, backend: str = "numpy", device: object = None, gpu_dynamics: bool = False, app_warmup: bool = False
    ) -> None:
        """Run the camera capture test with specified backend configuration.

        Args:
            backend: Backend to use for simulation.
            device: Device to use for physics simulation.
            gpu_dynamics: Whether to enable GPU dynamics.
            app_warmup: Whether to warm up the application.

        """
        await omni.usd.get_context().new_stage_async()
        stage = omni.usd.get_context().get_stage()
        timeline = omni.timeline.get_timeline_interface()

        # Configure SimulationManager
        SimulationManager.set_backend(backend)
        if device:
            SimulationManager.set_physics_sim_device(device)

        if gpu_dynamics:
            SimulationManager.enable_gpu_dynamics(True)
            SimulationManager.set_broadphase_type("GPU")

        # Create a plane and dome light
        dome_light = DomeLight("/World/DomeLight")
        dome_light.set_intensities(500)
        GroundPlane("/World/defaultGroundPlane", sizes=100.0, templates=None)

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

        # Start simulation
        timeline.play()
        await omni.kit.app.get_app().next_update_async()
        SimulationManager.initialize_physics()

        if app_warmup:
            for i in range(5):
                await omni.kit.app.get_app().next_update_async()

        # Capture data over multiple steps
        for i in range(3):
            print(f"Capture step {i}")
            SimulationManager.step()
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

    # numpy backend
    async def test_numpy_cpu_nogpu_nowarmup(self) -> None:
        """Tests numpy backend on CPU without GPU dynamics and without app warmup."""
        await self.run_test_async(backend="numpy", device=None, gpu_dynamics=False, app_warmup=False)

    async def test_numpy_cpu_nogpu_warmup(self) -> None:
        """Tests numpy backend on CPU without GPU dynamics and with app warmup."""
        await self.run_test_async(backend="numpy", device=None, gpu_dynamics=False, app_warmup=True)

    async def test_numpy_cpu_gpu_nowarmup(self) -> None:
        """Tests numpy backend on CPU with GPU dynamics and without app warmup."""
        await self.run_test_async(backend="numpy", device=None, gpu_dynamics=True, app_warmup=False)

    async def test_numpy_cpu_gpu_warmup(self) -> None:
        """Tests numpy backend on CPU with GPU dynamics and with app warmup."""
        await self.run_test_async(backend="numpy", device=None, gpu_dynamics=True, app_warmup=True)

    async def test_numpy_cuda_nogpu_nowarmup(self) -> None:
        """Tests numpy backend on CUDA without GPU dynamics and without app warmup."""
        await self.run_test_async(backend="numpy", device="cuda", gpu_dynamics=False, app_warmup=False)

    async def test_numpy_cuda_nogpu_warmup(self) -> None:
        """Tests numpy backend on CUDA without GPU dynamics and with app warmup."""
        await self.run_test_async(backend="numpy", device="cuda", gpu_dynamics=False, app_warmup=True)

    async def test_numpy_cuda_gpu_nowarmup(self) -> None:
        """Tests numpy backend on CUDA with GPU dynamics and without app warmup."""
        await self.run_test_async(backend="numpy", device="cuda", gpu_dynamics=True, app_warmup=False)

    async def test_numpy_cuda_gpu_warmup(self) -> None:
        """Tests numpy backend on CUDA with GPU dynamics and with app warmup."""
        await self.run_test_async(backend="numpy", device="cuda", gpu_dynamics=True, app_warmup=True)

    # torch backend
    async def test_torch_cpu_nogpu_nowarmup(self) -> None:
        """Tests torch backend on CPU without GPU dynamics and without app warmup."""
        await self.run_test_async(backend="torch", device=None, gpu_dynamics=False, app_warmup=False)

    async def test_torch_cpu_nogpu_warmup(self) -> None:
        """Tests torch backend on CPU without GPU dynamics and with app warmup."""
        await self.run_test_async(backend="torch", device=None, gpu_dynamics=False, app_warmup=True)

    async def test_torch_cpu_gpu_nowarmup(self) -> None:
        """Tests torch backend on CPU with GPU dynamics and without app warmup."""
        await self.run_test_async(backend="torch", device=None, gpu_dynamics=True, app_warmup=False)

    async def test_torch_cpu_gpu_warmup(self) -> None:
        """Tests torch backend on CPU with GPU dynamics and with app warmup."""
        await self.run_test_async(backend="torch", device=None, gpu_dynamics=True, app_warmup=True)

    async def test_torch_cuda_nogpu_nowarmup(self) -> None:
        """Tests torch backend on CUDA without GPU dynamics and without app warmup."""
        await self.run_test_async(backend="torch", device="cuda", gpu_dynamics=False, app_warmup=False)

    async def test_torch_cuda_nogpu_warmup(self) -> None:
        """Tests torch backend on CUDA without GPU dynamics and with app warmup."""
        await self.run_test_async(backend="torch", device="cuda", gpu_dynamics=False, app_warmup=True)

    async def test_torch_cuda_gpu_nowarmup(self) -> None:
        """Tests torch backend on CUDA with GPU dynamics and without app warmup."""
        await self.run_test_async(backend="torch", device="cuda", gpu_dynamics=True, app_warmup=False)

    async def test_torch_cuda_gpu_warmup(self) -> None:
        """Tests torch backend on CUDA with GPU dynamics and with app warmup."""
        await self.run_test_async(backend="torch", device="cuda", gpu_dynamics=True, app_warmup=True)
