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

"""Tests for Robot Poser UI extension startup and UIBuilder construction."""

import isaacsim.core.experimental.utils.app as app_utils
import isaacsim.core.experimental.utils.stage as stage_utils
import omni.kit.test


class TestRobotPoserStartup(omni.kit.test.AsyncTestCase):
    """Async tests for Robot Poser UI extension."""

    async def setUp(self) -> None:
        """Create a fresh USD stage for each test."""
        await stage_utils.create_new_stage_async()
        await app_utils.update_app_async()

    async def test_import_and_construct(self) -> None:
        """Verify the package imports and the UIBuilder can be constructed."""
        from isaacsim.robot.poser.ui.ui.ui_builder import UIBuilder

        builder = UIBuilder()
        self.assertIsNotNone(builder)
