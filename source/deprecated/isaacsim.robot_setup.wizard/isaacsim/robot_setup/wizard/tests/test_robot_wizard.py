# SPDX-FileCopyrightText: Copyright (c) 2024-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

"""The Kit extension system tests for Python has additional wrapping.

to make test auto-discoverable add support for async/await tests.
The easiest way to set up the test class is to have it derive from
the omni.kit.test.AsyncTestCase class that implements them.

"""

import omni.kit.test


class TestExtension(omni.kit.test.AsyncTestCase):
    """Test case class for the isaacsim.robot_setup.wizard extension.

    This class provides a testing framework for the Robot Setup Wizard extension using the Omniverse Kit SDK's
    asynchronous test capabilities. It inherits from omni.kit.test.AsyncTestCase to support async/await test
    methods and automatic test discovery.

    The class includes setUp and tearDown methods for test fixture preparation and cleanup, allowing for
    customized test environment initialization before each test and proper resource cleanup afterward.
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

    # async def test_extension(self):
    #     # Kit extension system test for Python is based on the unittest module.
    #     # Visit https://docs.python.org/3/library/unittest.html to see the
    #     # available assert methods to check for and report failures.
    #     print("Test case: test_extension")
    #     self.assertTrue(True)
