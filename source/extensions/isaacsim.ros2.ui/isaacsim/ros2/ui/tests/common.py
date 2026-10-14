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

"""Common utilities for ROS 2 UI tests."""

from isaacsim.core.experimental.utils import stage as stage_utils
from isaacsim.ros2.core.impl.ros2_test_case import ROS2TestCase
from isaacsim.test.utils import MenuUITestCase
from omni.ui.tests.test_base import OmniUiTest

__all__ = ["ROS2UITestCase"]


class ROS2UITestCase(MenuUITestCase, ROS2TestCase):
    """ROS 2 tests with shared menu UI readiness and automatic ROS resource cleanup."""

    async def setUp(self) -> None:
        """Initialize the UI and ROS fixtures and wait for the stage and menus."""
        # `OmniUiTest` does not forward setup, and `TimedAsyncTestCase` calls its
        # parent synchronously. Keep the UI base first and run both lifecycles explicitly.
        await OmniUiTest.setUp(self)
        await ROS2TestCase.setUp(self)
        # The ROS base creates the stage; calling `MenuUITestCase.setUp` would create another.
        self._stage = stage_utils.get_current_stage()
        await self.wait_for_stage_loading()

    async def tearDown(self) -> None:
        """Stop simulation, finish UI cleanup, and release ROS resources."""
        self._timeline.stop()
        try:
            await MenuUITestCase.tearDown(self)
        finally:
            await ROS2TestCase.tearDown(self)
