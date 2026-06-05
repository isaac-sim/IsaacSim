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

"""Test suite for verifying docstring examples in the storage native extension."""

import isaacsim.storage.native.nucleus as nucleus
import isaacsim.test.docstring

# import isaacsim.core.experimental.utils.impl.stage as stage_utils


class TestExtensionDocstrings(isaacsim.test.docstring.AsyncDocTestCase):
    """Test that docstring examples in the storage native extension execute correctly."""

    async def setUp(self):
        """Method called to prepare the test fixture."""
        super().setUp()
        # create new stage

    async def tearDown(self):
        """Method called immediately after the test method has been called."""
        super().tearDown()

    async def test_nucleus_docstrings(self):
        """Test docstring examples in the nucleus module."""
        await self.assertDocTests(nucleus)
