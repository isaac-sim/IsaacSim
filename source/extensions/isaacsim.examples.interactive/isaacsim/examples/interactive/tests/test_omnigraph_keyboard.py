"""Tests for the OmniGraph keyboard interactive example."""

# SPDX-FileCopyrightText: Copyright (c) 2022-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

# NOTE:
#   omni.kit.test - std python's unittest module with additional wrapping to add suport for async/await tests
#   For most things refer to unittest docs: https://docs.python.org/3/library/unittest.html
import omni.kit.test

# Import extension python module we are testing with absolute import path, as if we are external user (other extension)
from isaacsim.examples.interactive.omnigraph_keyboard import OmnigraphKeyboard


class TestOmnigraphKeyboardExampleExtension(omni.kit.test.AsyncTestCase):
    """Test cases for the OmniGraph keyboard example."""

    # Before running each test
    async def setUp(self) -> None:
        """Set up the OmniGraph keyboard sample and load the world."""
        self._sample = OmnigraphKeyboard()
        self._sample.set_world_settings(physics_dt=1.0 / 60.0, stage_units_in_meters=1.0)
        await self._sample.load_world_async()
        await app_utils.update_app_async()
        while stage_utils.is_stage_loading():
            await app_utils.update_app_async()

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

    async def test_reset(self) -> None:
        """Test that resetting the sample twice works without errors."""
        await self._sample.reset_async()
        await app_utils.update_app_async()
        await app_utils.update_app_async()
        await self._sample.reset_async()
        await app_utils.update_app_async()
        await app_utils.update_app_async()
