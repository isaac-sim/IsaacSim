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

# NOTE:
#   omni.kit.test - std python's unittest module with additional wrapping to add suport for async/await tests
#   For most things refer to unittest docs: https://docs.python.org/3/library/unittest.html

"""Test module for verifying Physics API Editor extension startup functionality."""

import omni.kit.test


# Having a test class dervived from omni.kit.test.AsyncTestCase declared on the root of module will make it auto-discoverable by omni.kit.test
class TestStartup(omni.kit.test.AsyncTestCase):
    """Test case for verifying the Physics API Editor extension starts up correctly."""

    # Before running each test
    async def setUp(self) -> None:
        """Set up test fixtures before each test method.

        Waits for the next application update to ensure the environment is ready.
        """
        await omni.kit.app.get_app().next_update_async()

    # After running each test
    async def tearDown(self) -> None:
        """Clean up after each test method.

        Waits for the next application update to allow cleanup to complete.
        """
        await omni.kit.app.get_app().next_update_async()

    # Run for 60 frames and make sure there were no errors loading
    async def test_startup(self) -> None:
        """Verify the Physics API Editor window loads without errors.

        Opens the Physics API Editor window, runs for 60 frames to ensure stability,
        then closes the window. Asserts that the window was successfully created.
        """
        window = omni.ui.Workspace.get_window("Physics API Editor")
        self.assertIsNotNone(window)
        window.visible = True
        for frame in range(60):
            await omni.kit.app.get_app().next_update_async()
        window.visible = False
