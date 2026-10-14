# SPDX-FileCopyrightText: Copyright (c) 2021-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

"""Smoke-test discovery of the experimental motion-generation extension tests.

This file intentionally keeps the assertion minimal; its purpose is to confirm
that Kit can collect and execute the extension's async Python test case.
"""

import isaacsim.robot_motion.experimental.motion_generation as motion_generation
import omni.kit.test
from isaacsim.robot_motion.experimental.motion_generation.impl._logging import get_logger


class TestExtension(omni.kit.test.AsyncTestCase):
    """Test case class for the isaacsim.robot_motion.experimental.motion_generation extension.

    This class provides unit tests to verify the functionality and behavior of the motion generation extension.
    It inherits from omni.kit.test.AsyncTestCase to support asynchronous test execution and integrates with
    the Omniverse Kit testing framework for extension validation.
    """

    async def setUp(self) -> None:
        """Method called to prepare the test fixture."""
        super().setUp()
        # ---------------
        # Do custom setUp
        # ---------------

    async def tearDown(self) -> None:
        """Method called immediately after the test method has been called."""
        # ------------------
        # Do custom tearDown
        # ------------------
        super().tearDown()

    # --------------------------------------------------------------------

    async def test_extension(self) -> None:
        """Test case for the extension functionality."""
        self.assertEqual(get_logger().channel, "isaacsim.robot_motion.experimental.motion_generation")
        module_path = getattr(motion_generation, "__file__", None)
        self.assertIsNotNone(module_path)
        print(f"Motion generation module: {module_path}")
        self.assertIn("pip_prebundle", module_path)
