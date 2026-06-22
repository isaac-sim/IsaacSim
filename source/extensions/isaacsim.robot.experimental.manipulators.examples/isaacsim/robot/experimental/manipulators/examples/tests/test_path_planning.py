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

"""Tests for the interactive path planning example."""

from __future__ import annotations

import isaacsim.core.experimental.utils.app as app_utils
import omni.kit.test
from isaacsim.robot.experimental.manipulators.examples.interactive.path_planning import PathPlanning


class TestPathPlanningExampleExtension(omni.kit.test.AsyncTestCase):
    """Test suite for the path planning interactive example."""

    async def setUp(self) -> None:
        """Set up test environment before each test."""
        self._sample = PathPlanning()
        await self._sample.load_world_async()
        await app_utils.update_app_async()

    async def tearDown(self) -> None:
        """Clean up after each test."""
        if app_utils.is_playing():
            app_utils.stop()
        await self._sample.clear_async()
        await app_utils.update_app_async()
        self._sample = None

    async def test_follow_target(self) -> None:
        """Test planning and executing a trajectory to the target."""
        await self._sample.reset_async()
        await app_utils.update_app_async()

        await self._sample._on_plan_to_target_event_async()
        await app_utils.update_app_async()

        await app_utils.update_app_async(steps=500)

    async def test_add_obstacle(self) -> None:
        """Test adding wall obstacles and replanning."""
        await self._sample.reset_async()
        await app_utils.update_app_async()

        for i in range(500):
            await app_utils.update_app_async()
            if i % 50 == 0:
                self._sample._on_add_wall_event()
                await app_utils.update_app_async()
                await self._sample._on_plan_to_target_event_async()

        await app_utils.update_app_async()
        await self._sample.reset_async()
        await app_utils.update_app_async()

    async def test_reset(self) -> None:
        """Test reset functionality."""
        await self._sample.reset_async()
        await app_utils.update_app_async()
