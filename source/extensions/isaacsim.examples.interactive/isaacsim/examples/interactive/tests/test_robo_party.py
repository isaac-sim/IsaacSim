"""Tests for the robo party interactive example."""

# SPDX-FileCopyrightText: Copyright (c) 2018-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

import isaacsim.core.experimental.utils.app as app_utils
import isaacsim.core.experimental.utils.stage as stage_utils
import omni.kit
import omni.kit.test
from isaacsim.core.simulation_manager import PhysicsScene, PhysxScene
from isaacsim.examples.interactive.robo_party import RoboParty


class TestRoboPartyExampleExtension(omni.kit.test.AsyncTestCase):
    """Test cases for the robo party example."""

    # Before running each test
    async def setUp(self) -> None:
        """Set up the robo party sample and load the world."""
        self._sample = RoboParty()
        self._sample.set_world_settings(physics_dt=1.0 / 60.0, stage_units_in_meters=1.0)
        await self._sample.load_world_async()
        await app_utils.update_app_async()
        while stage_utils.is_stage_loading():
            await app_utils.update_app_async()
        return

    # After running each test
    async def tearDown(self) -> None:
        """Tear down by waiting for assets and clearing the sample."""
        # In some cases the test will end before the asset is loaded, in this case wait for assets to load
        while stage_utils.is_stage_loading():
            print("tearDown, assets still loading, waiting to finish...")
            await asyncio.sleep(1.0)
        await self._sample.clear_async()
        await app_utils.update_app_async()
        self._sample = None

    # Run all functions with simulation enabled
    async def test_stacking(self) -> None:
        """Test the stacking and wheeled robot behaviors."""
        await self._sample.reset_async()
        await app_utils.update_app_async()
        await self._sample._on_start_party_event_async()
        await app_utils.update_app_async()
        # run for 2500 frames and print time
        for i in range(500):
            await app_utils.update_app_async()

    async def test_reset(self) -> None:
        """Test that resetting the sample twice works without errors."""
        await self._sample.reset_async()
        await app_utils.update_app_async()
        await app_utils.update_app_async()
        await self._sample.reset_async()
        await app_utils.update_app_async()
        await app_utils.update_app_async()

    async def test_cpu_device_preserved_after_reset(self) -> None:
        """After reset with device='cpu', all physics scenes must remain in CPU mode."""
        await self._sample.reset_async()
        await app_utils.update_app_async()

        scene_paths = PhysicsScene.get_physics_scene_paths()
        self.assertGreater(len(scene_paths), 0, "No physics scenes found after reset")

        for scene_path in scene_paths:
            physx_scene = PhysxScene(scene_path)
            self.assertFalse(
                physx_scene.get_enabled_gpu_dynamics(),
                f"GPU dynamics should be False (CPU mode) at {scene_path}, got True",
            )
            self.assertEqual(
                physx_scene.get_broadphase_type(),
                "MBP",
                f"Broadphase should be 'MBP' (CPU mode) at {scene_path}, " f"got '{physx_scene.get_broadphase_type()}'",
            )
