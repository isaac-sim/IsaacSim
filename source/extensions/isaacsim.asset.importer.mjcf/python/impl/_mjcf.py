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

"""Deprecated MJCF importer configuration utilities."""

import logging

from .config import MJCFImporterConfig
from .converter import MJCFImporter


class _mjcf:  # noqa: N801
    """Deprecated MJCF importer configuration utilities.

    Args:
        config: Optional MJCF importer configuration. If None, a default config is used.

    .. deprecated:: Use MJCFImporter() directly instead.
    """

    def __init__(self, config: MJCFImporterConfig | None = None) -> None:
        self._logger = logging.getLogger(__name__)
        self._logger.warning("_mjcf is deprecated and will be removed in a future version. Use MJCFImporter() instead.")
        self.importer: MJCFImporter | None = MJCFImporter(config)
        self.config = self.importer.config

    def acquire_mjcf_interface(self) -> MJCFImporter:
        """Acquire the MJCF interface.

        Returns:
            MJCFImporter instance.

        .. deprecated:: Use MJCFImporter() directly instead.
        """
        self._logger.warning(
            "acquire_mjcf_interface is deprecated and will be removed in a future version. Use MJCFImporter() instead."
        )
        if not self.importer:
            self.importer = MJCFImporter(self.config)
            self.importer.config = self.config
        return self.importer

    def release_mjcf_interface(self) -> None:
        """Release the MJCF interface.

        .. deprecated:: Use MJCFImporter() directly instead.
        """
        self._logger.warning(
            "release_mjcf_interface is deprecated and will be removed in a future version. Use MJCFImporter() instead."
        )
        self.importer = None

    def create_asset_mjcf(
        self, mjcf_path: str, prim_path: str, import_config: MJCFImporterConfig, dest_path: str
    ) -> str:
        """Create an asset from an MJCF file.

        Args:
            mjcf_path: Path to the MJCF file.
            prim_path: Path to the prim on the USD stage.
            import_config: Import configuration.
            dest_path: Destination path for the USD file.

        Returns:
            Path to the generated USD file.

        .. deprecated:: Use MJCFImporter() directly instead.
        """
        self._logger.warning(
            "create_asset_mjcf is deprecated and will be removed in a future version. Use MJCFImporter() instead."
        )
        if not self.importer:
            self.importer = MJCFImporter(self.config)
            self.importer.config = self.config
        self.importer.config.mjcf_path = mjcf_path
        self.importer.config.usd_path = dest_path
        return self.importer.import_mjcf()

    def import_config(self) -> MJCFImporterConfig:  # noqa: N802
        """Get the import configuration.

        Returns:
            Current import configuration.

        .. deprecated:: Use MJCFImporterConfig() directly instead.
        """
        self._logger.warning(
            "ImportConfig is deprecated and will be removed in a future version. Use MJCFImporterConfig() instead."
        )
        return self.config

    # Backward compatibility alias
    ImportConfig = import_config  # noqa: N802
