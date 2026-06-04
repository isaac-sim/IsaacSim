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

"""Tests for MJCF importer command implementations."""

import asyncio
import os
import shutil

import carb
import isaacsim.core.experimental.utils.stage as stage_utils

# NOTE:
#   omni.kit.test - std python's unittest module with additional wrapping to add suport for async/await tests
#   For most things refer to unittest docs: https://docs.python.org/3/library/unittest.html
import omni.kit.commands
import omni.kit.test
from isaacsim.asset.importer.mjcf import MJCFImporterConfig


class TestMJCFCommands(omni.kit.test.AsyncTestCase):
    """Test MJCF importer command implementations.

    Example:

    .. code-block:: python

        >>> import omni.kit.test
        >>> class Example(omni.kit.test.AsyncTestCase):
        ...     pass
        ...
    """

    async def setUp(self) -> None:
        """Prepare shared test fixtures.

        Example:

        .. code-block:: python

            >>> import omni.usd
            >>> omni.usd.get_context()  # doctest: +SKIP
        """
        ext_manager = omni.kit.app.get_app().get_extension_manager()
        mjcf_ext_id = ext_manager.get_enabled_extension_id("isaacsim.asset.importer.mjcf")
        self._mjcf_extension_path = ext_manager.get_extension_path(mjcf_ext_id)
        self._mjcf_path = os.path.normpath(os.path.join(self._mjcf_extension_path, "data", "mjcf", "nv_ant.xml"))
        await omni.usd.get_context().new_stage_async()
        await omni.kit.app.get_app().next_update_async()

    async def tearDown(self) -> None:
        """Wait for stage loading to complete after tests.

        Example:

        .. code-block:: python

            >>> import asyncio
            >>> asyncio.sleep(0)  # doctest: +SKIP
        """
        while omni.usd.get_context().get_stage_loading_status()[2] > 0:
            carb.log_info("tearDown, assets still loading, waiting to finish...")
            await asyncio.sleep(1.0)
        await omni.kit.app.get_app().next_update_async()

    def test_mjcf_create_import_config(self) -> None:
        """Test MJCFCreateImportConfig command execution.

        Example:

        .. code-block:: python

            >>> import omni.kit.commands
            >>> from isaacsim.asset.importer.mjcf import MJCFImporterConfig

            >>> status, config = omni.kit.commands.execute("MJCFCreateImportConfig")
            >>> isinstance(config, MJCFImporterConfig)
            True
        """
        status, config = omni.kit.commands.execute("MJCFCreateImportConfig")

        self.assertTrue(status)
        self.assertIsInstance(config, MJCFImporterConfig)
        self.assertIsNone(config.mjcf_path)
        self.assertIsNone(config.usd_path)
        self.assertTrue(config.import_scene)
        self.assertFalse(config.merge_mesh)
        self.assertFalse(config.debug_mode)

    async def test_mjcf_create_asset(self) -> None:
        """Test MJCFCreateAsset command execution.

        Example:

        .. code-block:: python

            >>> import omni.kit.commands
            >>> from isaacsim.asset.importer.mjcf import MJCFImporterConfig

            >>> config = MJCFImporterConfig(mjcf_path="/path/to/ant.xml")
            >>> status, path = omni.kit.commands.execute(
            ...     "MJCFCreateAsset",
            ...     mjcf_path="/path/to/ant.xml",
            ...     import_config=config
            ... )  # doctest: +SKIP
            >>> isinstance(path, str)
            True
        """
        config = MJCFImporterConfig(mjcf_path=self._mjcf_path)
        status, output_path = omni.kit.commands.execute(
            "MJCFCreateAsset",
            mjcf_path=self._mjcf_path,
            import_config=config,
            prim_path="/World/Robot",
            dest_path="",
        )

        self.assertTrue(status)
        self.assertIsInstance(output_path, str)
        self.assertTrue(os.path.exists(output_path))
        self.assertTrue(output_path.endswith(".usda"))

        # Verify stage was opened
        status, stage = stage_utils.open_stage(output_path)
        self.assertTrue(status)
        self.assertIsNotNone(stage)

        prim = stage.GetDefaultPrim()
        self.assertIsNotNone(prim)
        self.assertEqual(prim.GetName(), "ant")
        stage = None

        # Clean up
        try:
            shutil.rmtree(os.path.normpath(os.path.dirname(output_path)))
        except OSError as e:
            carb.log_error(f"Error cleaning up {os.path.normpath(os.path.dirname(output_path))}: {e.strerror}")
