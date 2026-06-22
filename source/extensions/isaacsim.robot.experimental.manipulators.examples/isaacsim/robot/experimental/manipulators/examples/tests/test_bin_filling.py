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

"""Tests for the interactive bin filling example."""

from __future__ import annotations

import isaacsim.core.experimental.utils.app as app_utils
import numpy as np
import omni.kit.test
from isaacsim.core.experimental.utils.transform import euler_angles_to_quaternion
from isaacsim.robot.experimental.manipulators.examples.interactive.bin_filling import BinFilling


class TestBinFillingExampleExtension(omni.kit.test.AsyncTestCase):
    """Test suite for the bin filling interactive example."""

    async def setUp(self) -> None:
        """Set up test environment before each test."""
        self._sample = BinFilling()
        await self._sample.load_world_async()
        await app_utils.update_app_async()

    async def tearDown(self) -> None:
        """Clean up after each test."""
        if app_utils.is_playing():
            app_utils.stop()
        await self._sample.clear_async()
        await app_utils.update_app_async()
        self._sample = None

    async def test_bin_filling_task(self) -> None:
        """Test bin filling simulation runs and robot operates correctly."""
        await self._sample.reset_async()
        await app_utils.update_app_async()

        self.assertIsNotNone(self._sample._robot)
        self.assertIsNotNone(self._sample._bin_prim)

        await self._sample.on_fill_bin_event_async()
        await app_utils.update_app_async()

        for i in range(1500):
            await app_utils.update_app_async()

            if self._sample._event == 3:
                break

        self.assertGreaterEqual(self._sample._event, 1)

    async def test_reset(self) -> None:
        """Test reset functionality during bin filling."""
        await self._sample.reset_async()
        await app_utils.update_app_async()

        await self._sample.on_fill_bin_event_async()
        await app_utils.update_app_async()

        await app_utils.update_app_async(steps=500)

        await self._sample.reset_async()
        await app_utils.update_app_async()

        self.assertEqual(self._sample._event, 0)

        await self._sample.on_fill_bin_event_async()
        await app_utils.update_app_async()

        await app_utils.update_app_async(steps=500)

    async def test_smooth_target_generation_state(self) -> None:
        """Test cached pick position and smooth target generation."""

        class FakeTensor:
            def __init__(self, value: object) -> None:
                self._value = np.asarray(value)

            def numpy(self) -> np.ndarray:
                return self._value

        class FakeBin:
            def __init__(self, position: object) -> None:
                self.position = np.asarray(position, dtype=float)

            def get_world_poses(self) -> tuple[FakeTensor, None]:
                return FakeTensor([self.position.copy()]), None

        class FakeRobot:
            def close_gripper(self) -> None:
                pass

        targets = []
        self._sample._move_to_target = lambda position, dt: targets.append(np.asarray(position))
        fake_robot = FakeRobot()
        self._sample._robot = fake_robot
        self._sample._bin_prim = FakeBin([0.3, 0.5, -0.2])
        self._sample._ee_orientation = np.array([1.0, 0.0, 0.0, 0.0])
        self._sample._suction_offset_world = np.zeros(3)

        self._sample._event = 2
        self._sample._physics_step(1.0 / 60.0, None)
        np.testing.assert_allclose(self._sample._pick_position, [0.3, 0.5, -0.2])

        self._sample._bin_prim.position = np.array([0.35, 0.45, -0.18])
        self._sample._event = 3
        self._sample._physics_step(1.0 / 60.0, None)
        np.testing.assert_allclose(self._sample._pick_position, [0.35, 0.45, -0.18])

        self._sample._t = 0.5
        self._sample._event = 5
        self._sample._pick_position = np.array([0.2, 0.4, -0.1])
        self._sample._physics_step(1.0 / 60.0, None)
        np.testing.assert_allclose(targets[-1], [0.1, 0.475, 0.33])

        self._sample._reset_state_machine()
        self.assertIsNone(self._sample._pick_position)

    async def test_tool_orientation_uses_ee_link_target_orientation(self) -> None:
        """Test cuMotion tool0 orientation is derived from the desired ee_link orientation."""
        ee_orientation = euler_angles_to_quaternion([np.pi, 0.0, -np.pi / 2.0], extrinsic=False).numpy()
        tool_orientation = self._sample._get_tool_orientation_from_ee_orientation(ee_orientation)

        self.assertAlmostEqual(np.linalg.norm(tool_orientation), 1.0)
        self.assertFalse(
            np.allclose(tool_orientation, euler_angles_to_quaternion([0.0, np.pi, 0.0], extrinsic=False).numpy())
        )
