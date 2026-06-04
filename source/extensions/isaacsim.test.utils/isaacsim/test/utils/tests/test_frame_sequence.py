# SPDX-FileCopyrightText: Copyright (c) 2025-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

"""Test suite for capture_frame_sequence_async in Isaac Sim."""

from __future__ import annotations

import os
import shutil
import tempfile

import numpy as np
import omni.kit.app
import omni.usd
from isaacsim.core.experimental.utils import stage as stage_utils
from isaacsim.test.utils.image_capture import capture_frame_sequence_async
from isaacsim.test.utils.timed_async_test import TimedAsyncTestCase
from pxr import UsdGeom, UsdLux


class TestCaptureFrameSequenceApp(TimedAsyncTestCase):
    """Tests for capture_frame_sequence_async in 'app' mode."""

    async def setUp(self) -> None:
        """Set up test fixtures with a minimal scene."""
        await super().setUp()
        self.test_dir = tempfile.mkdtemp()
        await stage_utils.create_new_stage_async()
        await omni.kit.app.get_app().next_update_async()
        stage = omni.usd.get_context().get_stage()
        UsdGeom.Cube.Define(stage, "/World/Cube")
        UsdLux.DistantLight.Define(stage, "/DistantLight")
        for _ in range(5):
            await omni.kit.app.get_app().next_update_async()

    async def tearDown(self) -> None:
        """Clean up temporary files."""
        if os.path.exists(self.test_dir):
            shutil.rmtree(self.test_dir)
        await super().tearDown()

    async def test_app_mode_captures_frames(self) -> None:
        """Verify that app mode captures the requested number of PNG frames."""
        out = os.path.join(self.test_dir, "app_frames")
        paths = await capture_frame_sequence_async(out, num_frames=5, mode="app")
        self.assertEqual(len(paths), 5)
        for p in paths:
            self.assertTrue(os.path.isfile(p), f"Frame file not found: {p}")
            self.assertTrue(p.endswith(".png"))
            self.assertGreater(os.path.getsize(p), 0)

    async def test_app_mode_custom_prefix_and_start(self) -> None:
        """Verify custom prefix and start_index naming."""
        out = os.path.join(self.test_dir, "custom")
        paths = await capture_frame_sequence_async(out, num_frames=3, mode="app", prefix="shot", start_index=10)
        self.assertEqual(len(paths), 3)
        self.assertIn("shot_0010.png", os.path.basename(paths[0]))
        self.assertIn("shot_0011.png", os.path.basename(paths[1]))
        self.assertIn("shot_0012.png", os.path.basename(paths[2]))

    async def test_app_mode_creates_output_dir(self) -> None:
        """Verify that output directory is created if it does not exist."""
        out = os.path.join(self.test_dir, "new_dir", "nested")
        paths = await capture_frame_sequence_async(out, num_frames=2, mode="app")
        self.assertEqual(len(paths), 2)
        self.assertTrue(os.path.isdir(out))


class TestCaptureFrameSequenceViewport(TimedAsyncTestCase):
    """Tests for capture_frame_sequence_async in 'viewport' mode."""

    async def setUp(self) -> None:
        """Set up test fixtures with a minimal scene."""
        await super().setUp()
        self.test_dir = tempfile.mkdtemp()
        await stage_utils.create_new_stage_async()
        await omni.kit.app.get_app().next_update_async()
        stage = omni.usd.get_context().get_stage()
        UsdGeom.Cube.Define(stage, "/World/Cube")
        UsdLux.DistantLight.Define(stage, "/DistantLight")
        for _ in range(5):
            await omni.kit.app.get_app().next_update_async()

    async def tearDown(self) -> None:
        """Clean up temporary files."""
        if os.path.exists(self.test_dir):
            shutil.rmtree(self.test_dir)
        await super().tearDown()

    async def test_viewport_mode_captures_frames(self) -> None:
        """Verify that viewport mode captures the requested number of PNG frames."""
        out = os.path.join(self.test_dir, "viewport_frames")
        paths = await capture_frame_sequence_async(out, num_frames=5, mode="viewport")
        self.assertEqual(len(paths), 5)
        for p in paths:
            self.assertTrue(os.path.isfile(p), f"Frame file not found: {p}")
            self.assertTrue(p.endswith(".png"))
            self.assertGreater(os.path.getsize(p), 0)


class TestCaptureFrameSequenceReplicator(TimedAsyncTestCase):
    """Tests for capture_frame_sequence_async in 'replicator' mode."""

    async def setUp(self) -> None:
        """Set up test fixtures with a minimal scene."""
        await super().setUp()
        self.test_dir = tempfile.mkdtemp()
        await stage_utils.create_new_stage_async()
        await omni.kit.app.get_app().next_update_async()
        stage = omni.usd.get_context().get_stage()
        UsdGeom.Cube.Define(stage, "/World/Cube")
        UsdLux.DistantLight.Define(stage, "/DistantLight")
        for _ in range(5):
            await omni.kit.app.get_app().next_update_async()

    async def tearDown(self) -> None:
        """Clean up temporary files."""
        if os.path.exists(self.test_dir):
            shutil.rmtree(self.test_dir)
        await super().tearDown()

    async def test_replicator_rgb_captures_frames(self) -> None:
        """Verify that replicator mode captures RGB frames as PNG."""
        out = os.path.join(self.test_dir, "rep_rgb")
        paths = await capture_frame_sequence_async(
            out, num_frames=3, mode="replicator", annotator_name="rgb", resolution=(640, 480)
        )
        self.assertEqual(len(paths), 3)
        for p in paths:
            self.assertTrue(os.path.isfile(p), f"Frame file not found: {p}")
            self.assertTrue(p.endswith(".png"))
            self.assertGreater(os.path.getsize(p), 0)

    async def test_replicator_depth_captures_npy(self) -> None:
        """Verify that replicator mode captures depth frames as NPY."""
        out = os.path.join(self.test_dir, "rep_depth")
        paths = await capture_frame_sequence_async(
            out, num_frames=3, mode="replicator", annotator_name="distance_to_camera", resolution=(320, 240)
        )
        self.assertEqual(len(paths), 3)
        for p in paths:
            self.assertTrue(os.path.isfile(p), f"Frame file not found: {p}")
            self.assertTrue(p.endswith(".npy"))
            data = np.load(p)
            self.assertEqual(data.dtype, np.float32)

    async def test_replicator_custom_camera(self) -> None:
        """Verify capture from a user-created camera prim."""
        stage = omni.usd.get_context().get_stage()
        UsdGeom.Camera.Define(stage, "/World/TestCam")
        for _ in range(3):
            await omni.kit.app.get_app().next_update_async()

        out = os.path.join(self.test_dir, "rep_cam")
        paths = await capture_frame_sequence_async(
            out,
            num_frames=2,
            mode="replicator",
            annotator_name="rgb",
            resolution=(320, 240),
            camera_prim_path="/World/TestCam",
        )
        self.assertEqual(len(paths), 2)
        for p in paths:
            self.assertTrue(os.path.isfile(p))

        # Verify camera prim still exists
        self.assertTrue(stage.GetPrimAtPath("/World/TestCam").IsValid())

    async def test_replicator_with_existing_render_product(self) -> None:
        """Verify capture using a caller-provided render product."""
        import omni.replicator.core as rep

        rp = rep.create.render_product("/OmniverseKit_Persp", (320, 240))

        out = os.path.join(self.test_dir, "rep_existing_rp")
        paths = await capture_frame_sequence_async(
            out, num_frames=2, mode="replicator", annotator_name="rgb", render_product=rp
        )
        self.assertEqual(len(paths), 2)
        for p in paths:
            self.assertTrue(os.path.isfile(p))

        # Caller owns the render product — it should still be usable
        rp.destroy()

    async def test_replicator_default_resolution(self) -> None:
        """Verify that default resolution (1280x720) is used when not specified."""
        out = os.path.join(self.test_dir, "rep_default_res")
        paths = await capture_frame_sequence_async(out, num_frames=1, mode="replicator", annotator_name="rgb")
        self.assertEqual(len(paths), 1)
        self.assertTrue(os.path.isfile(paths[0]))

    async def test_replicator_default_camera(self) -> None:
        """Verify that the active viewport camera is used when camera_prim_path is not specified."""
        out = os.path.join(self.test_dir, "rep_default_cam")
        paths = await capture_frame_sequence_async(
            out, num_frames=1, mode="replicator", annotator_name="rgb", resolution=(320, 240)
        )
        self.assertEqual(len(paths), 1)
        self.assertTrue(os.path.isfile(paths[0]))


class TestCaptureFrameSequenceErrors(TimedAsyncTestCase):
    """Tests for error handling in capture_frame_sequence_async."""

    async def setUp(self) -> None:
        """Set up test fixtures."""
        await super().setUp()
        self.test_dir = tempfile.mkdtemp()

    async def tearDown(self) -> None:
        """Clean up temporary files."""
        if os.path.exists(self.test_dir):
            shutil.rmtree(self.test_dir)
        await super().tearDown()

    async def test_invalid_mode_raises(self) -> None:
        """Verify that an invalid mode raises ValueError."""
        out = os.path.join(self.test_dir, "invalid")
        with self.assertRaises(ValueError):
            await capture_frame_sequence_async(out, num_frames=1, mode="invalid")
