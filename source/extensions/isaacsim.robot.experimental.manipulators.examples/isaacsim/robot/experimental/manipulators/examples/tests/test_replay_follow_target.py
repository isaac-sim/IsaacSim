"""Tests for the replay follow-target interactive example."""

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

import os

import isaacsim.core.experimental.utils.app as app_utils
import omni.kit.app
import omni.kit.test
from isaacsim.robot.experimental.manipulators.examples.interactive.replay_follow_target import ReplayFollowTarget

EXT_NAME = "isaacsim.robot.experimental.manipulators.examples"


def _get_data_file() -> str:
    """Get the path to the example data file in this extension's data/ folder.

    Returns:
        Absolute path to the example data file.
    """
    mgr = omni.kit.app.get_app().get_extension_manager()
    ext_root = mgr.get_extension_path(mgr.get_enabled_extension_id(EXT_NAME))
    return os.path.join(ext_root, "data", "example_data_file.json")


class TestReplayFollowTargetExampleExtension(omni.kit.test.AsyncTestCase):
    """Test cases for the replay follow-target example."""

    async def setUp(self) -> None:
        """Set up the replay follow-target sample and load the world."""
        self._sample = ReplayFollowTarget()
        await self._sample.load_world_async()
        await app_utils.update_app_async()

    async def tearDown(self) -> None:
        """Clean up after each test."""
        if app_utils.is_playing():
            app_utils.stop()
        await self._sample.clear_async()
        await app_utils.update_app_async()
        self._sample = None

    async def test_reset(self) -> None:
        """Test that resetting the sample twice works without errors."""
        await self._sample.reset_async()
        await app_utils.update_app_async()
        await self._sample.reset_async()
        await app_utils.update_app_async()

    async def test_replay_trajectory(self) -> None:
        """Test trajectory-only replay from a data file."""
        await self._sample.reset_async()
        await app_utils.update_app_async()

        await self._sample._on_replay_trajectory_event_async(_get_data_file())
        await app_utils.update_app_async(steps=500)

    async def test_replay_scene(self) -> None:
        """Test full scene replay from a data file."""
        await self._sample.reset_async()
        await app_utils.update_app_async()

        await self._sample._on_replay_scene_event_async(_get_data_file())
        await app_utils.update_app_async(steps=500)
