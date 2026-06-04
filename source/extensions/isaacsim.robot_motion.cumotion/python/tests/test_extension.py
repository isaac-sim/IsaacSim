# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

"""
The Kit extension system tests for Python has additional wrapping.

to make test auto-discoverable add support for async/await tests.
The easiest way to set up the test class is to have it derive from
the omni.kit.test.AsyncTestCase class that implements them.

Visit the next link for more details:
  https://docs.omniverse.nvidia.com/kit/docs/kit-manual/latest/guide/testing_exts_python.html
"""

import omni.kit.test


class TestExtension(omni.kit.test.AsyncTestCase):
    """Test class for the isaacsim.robot_motion.cumotion extension.

    This class provides automated testing for the cumotion extension functionality using the Omniverse Kit
    testing framework. It inherits from omni.kit.test.AsyncTestCase to support asynchronous test methods and
    auto-discovery by the Kit extension system.

    The test verifies basic functionality of the cumotion module, including world creation capabilities.
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
        """Tests the cumotion extension by creating a world instance and verifying it is not None."""
        # Kit extension system test for Python is based on the unittest module.
        # Visit https://docs.python.org/3/library/unittest.html to see the
        # available assert methods to check for and report failures.
        print("Test case: test_extension")
        import cumotion

        world = cumotion.create_world()
        self.assertIsNotNone(world)
