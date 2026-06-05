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

"""Docstring tests for MJCF importer UI APIs."""

import isaacsim.core.experimental.utils.impl.stage as stage_utils
import isaacsim.test.docstring
from isaacsim.asset.importer.mjcf import MJCFImporter, MJCFImporterConfig


class TestExtensionDocstrings(isaacsim.test.docstring.AsyncDocTestCase):
    """Run docstring tests for MJCF UI importer APIs.

    Example:

    .. code-block:: python

        >>> import isaacsim.test.docstring
        >>> issubclass(isaacsim.test.docstring.AsyncDocTestCase, object)
        True
    """

    async def setUp(self) -> None:
        """Prepare the test fixture and create a stage.

        Example:

        .. code-block:: python

            >>> import isaacsim.core.experimental.utils.impl.stage as stage_utils
            >>> stage_utils.create_new_stage_async()  # doctest: +SKIP
        """
        super().setUp()
        # create new stage
        await stage_utils.create_new_stage_async()

    async def tearDown(self) -> None:
        """Clean up the test fixture.

        Example:

        .. code-block:: python

            >>> import isaacsim.test.docstring
            >>> issubclass(isaacsim.test.docstring.AsyncDocTestCase, object)
            True
        """
        super().tearDown()

    async def test_mjcf_docstrings(self) -> None:
        """Validate docstring examples for MJCF importer classes.

        Example:

        .. code-block:: python

            >>> from isaacsim.asset.importer.mjcf import MJCFImporterConfig
            >>> MJCFImporterConfig()
            <...>
        """
        await self.assertDocTests(MJCFImporter)
        await self.assertDocTests(MJCFImporterConfig)
