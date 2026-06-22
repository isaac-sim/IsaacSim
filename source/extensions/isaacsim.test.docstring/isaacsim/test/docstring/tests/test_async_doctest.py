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

"""Tests for validating asynchronous doctests in the isaacsim.test.docstring extension."""

import isaacsim.test.docstring


class TestAsyncDocTest(isaacsim.test.docstring.AsyncDocTestCase):
    """A test case for validating asynchronous doctests in the isaacsim.test.docstring extension.

    This class inherits from AsyncDocTestCase and provides automated testing functionality
    for doctest examples that require asynchronous execution. It validates that doctests
    within the AsyncDocTestCase class execute correctly in an async environment.
    """

    # Before running each test
    async def setUp(self) -> None:
        """Set up the test fixture before each test method is run."""

    # After running each test
    async def tearDown(self) -> None:
        """Clean up after each test method has run."""

    async def test_async_doctest_case(self) -> None:
        """Test AsyncDocTestCase docstring examples."""
        from isaacsim.test.docstring import AsyncDocTestCase

        await self.assertDocTests(AsyncDocTestCase)
