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

"""Test suite for Lula kinematics library import and basic functionality."""

# NOTE:
#   omni.kit.test - std python's unittest module with additional wrapping to add suport for async/await tests
#   For most things refer to unittest docs: https://docs.python.org/3/library/unittest.html
import omni.kit.test


# Having a test class dervived from omni.kit.test.AsyncTestCase declared on the root of module will make it auto-discoverable by omni.kit.test
class TestLula(omni.kit.test.AsyncTestCase):
    """Test basic Lula kinematics library functionality."""

    # Before running each test
    async def setUp(self) -> None:
        """Set up test fixtures."""

    # After running each test
    async def tearDown(self) -> None:
        """Clean up after each test."""

    async def test_lula(self) -> None:
        """Test that Lula can be imported and a world can be created."""
        import lula

        world = lula.create_world()
        self.assertIsNotNone(world)
