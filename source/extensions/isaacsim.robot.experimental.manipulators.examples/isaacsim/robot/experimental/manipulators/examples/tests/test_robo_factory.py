"""Tests for the robo factory interactive example."""

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

from __future__ import annotations

import isaacsim.core.experimental.utils.app as app_utils
import numpy as np
import omni.kit.test
from isaacsim.robot.experimental.manipulators.examples.interactive.robo_factory import RoboFactory


class TestRoboFactoryExampleExtension(omni.kit.test.AsyncTestCase):
    """Test cases for the robo factory example."""

    async def _wait_for_stacking_done(self, *, max_steps: int = 3200) -> bool:
        """Wait until all stacking tasks report completion."""
        is_done = False

        def check_done(_step: int, _steps: int) -> bool:
            nonlocal is_done
            is_done = bool(self._sample._stackings) and all(stacking.is_done() for stacking in self._sample._stackings)
            return not is_done

        await app_utils.update_app_async(steps=max_steps, callback=check_done)
        return is_done

    async def setUp(self) -> None:
        """Set up the robo factory sample and load the world."""
        self._sample = RoboFactory()
        await self._sample.load_world_async()
        await app_utils.update_app_async()

    async def tearDown(self) -> None:
        """Clean up after each test."""
        if app_utils.is_playing():
            app_utils.stop()
        await self._sample.clear_async()
        await app_utils.update_app_async()
        self._sample = None

    async def test_stacking(self) -> None:
        """Test that stacking task runs and completes successfully."""
        await self._sample.reset_async()
        await app_utils.update_app_async()

        await self._sample._on_start_stacking_event_async()
        await app_utils.update_app_async()

        self.assertTrue(await self._wait_for_stacking_done(), "Stacking did not complete within the allotted steps.")
        await app_utils.update_app_async(steps=60)

        stacking_task = self._sample._stackings[0]
        cube_names = stacking_task.get_cube_names()
        task_observations = stacking_task.get_observations()

        expected_stack_position_xy = np.array([0.5, -2.5])
        for cube_name in cube_names:
            cube_position = task_observations[cube_name]["position"]
            cube_position_xy = cube_position[0:2]

            is_close = np.isclose(cube_position_xy, expected_stack_position_xy, atol=0.02).all()
            self.assertTrue(
                is_close,
                f"Cube {cube_name} not at expected stack position. "
                f"Got xy: {cube_position_xy}, expected xy: {expected_stack_position_xy}",
            )

    async def test_reset(self) -> None:
        """Test that resetting the sample twice works without errors."""
        await self._sample.reset_async()
        await app_utils.update_app_async()
        await self._sample.reset_async()
        await app_utils.update_app_async()
