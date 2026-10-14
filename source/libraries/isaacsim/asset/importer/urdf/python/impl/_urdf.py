# SPDX-FileCopyrightText: Copyright (c) 2023-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

"""Deprecated URDF importer configuration utilities."""

from __future__ import annotations

import logging
from typing import NoReturn

from .config import URDFImporterConfig
from .converter import URDFImporter


class _urdf:  # noqa: N801
    """Deprecated URDF importer configuration utilities.

    Args:
        config: Optional URDF importer configuration. If None, a default config is used.

    .. deprecated:: Use URDFImporter() directly instead.
    """

    def __init__(self, config: URDFImporterConfig | None = None) -> None:
        self._logger = logging.getLogger(__name__)
        self._logger.warning("_urdf is deprecated and will be removed in a future version. Use URDFImporter() instead.")
        self.importer: URDFImporter | None = URDFImporter(config)
        self.config = self.importer.config

    def acquire_urdf_interface(self) -> URDFImporter:
        """Acquire the URDF interface.

        Returns:
            URDFImporter instance.

        .. deprecated:: Use URDFImporter() directly instead.
        """
        self._logger.warning(
            "acquire_urdf_interface is deprecated and will be removed in a future version. Use URDFImporter() instead."
        )
        if not self.importer:
            self.importer = URDFImporter(self.config)
            self.importer.config = self.config
        return self.importer

    def release_urdf_interface(self) -> None:
        """Release the URDF interface.

        .. deprecated:: Use URDFImporter() directly instead.
        """
        self._logger.warning(
            "release_urdf_interface is deprecated and will be removed in a future version. Use URDFImporter() instead."
        )
        self.importer = None

    def create_asset_urdf(
        self, urdf_path: str, prim_path: str, import_config: URDFImporterConfig, dest_path: str
    ) -> str:
        """Create an asset from a URDF file.

        Args:
            urdf_path: Path to the URDF file.
            prim_path: Path to the prim on the USD stage.
            import_config: Import configuration.
            dest_path: Destination path for the USD file.

        Returns:
            Path to the generated USD file.

        .. deprecated:: Use URDFImporter() directly instead.
        """
        self._logger.warning(
            "create_asset_urdf is deprecated and will be removed in a future version. Use URDFImporter() instead."
        )
        if not self.importer:
            self.importer = URDFImporter(self.config)
            self.importer.config = self.config
        self.importer.config.urdf_path = urdf_path
        self.importer.config.usd_path = dest_path
        return self.importer.import_urdf()

    def import_config(self) -> URDFImporterConfig:  # noqa: N802
        """Get the import configuration.

        Returns:
            Current import configuration.

        .. deprecated:: Use URDFImporterConfig() directly instead.
        """
        self._logger.warning(
            "ImportConfig is deprecated and will be removed in a future version. Use URDFImporterConfig() instead."
        )
        return self.config

    def parse_string_urdf(self, urdf_string: str) -> NoReturn:
        """Parse a URDF string and return the USD string.

        Args:
            urdf_string: The URDF string to parse.

        Raises:
            ValueError: Parsing urdf strings is not supported.
        """
        self._logger.warning(
            "parse_string_urdf is deprecated and will be removed in a future version. Parsing urdf strings is not supported."
        )
        raise ValueError("Parsing urdf strings is not supported.")

    def get_kinematic_chain(self, urdf: str) -> NoReturn:
        """Get the kinematic chain of the robot. Mostly used for graphic display of the kinematic tree.

        Args:
            urdf: The parsed URDF, the output from :obj:`parse_urdf`

        Raises:
            ValueError: Getting the kinematic chain is not supported.
        """
        self._logger.warning(
            "get_kinematic_chain is deprecated and will be removed in a future version. Getting the kinematic chain is not supported."
        )
        raise ValueError("Getting the kinematic chain is not supported.")

    def compute_natural_stiffness(self, urdf: str, joint_name: str, natural_frequency: float) -> NoReturn:
        """Compute the natural stiffness of the robot.

        Args:
            urdf: The parsed URDF, the output from :obj:`parse_urdf`
            joint_name: The name of the joint to compute the natural stiffness for.
            natural_frequency: The natural frequency to compute the natural stiffness for.

        Raises:
            ValueError: Computing the natural stiffness is not supported.
        """
        self._logger.warning(
            "compute_natural_stiffness is deprecated and will be removed in a future version. Computing the natural stiffness is not supported. Please use the gains tuner extension instead."
        )
        raise ValueError(
            "Computing the natural stiffness is not supported. Please use the gains tuner extension instead."
        )

    # Backward compatibility alias
    ImportConfig = import_config  # noqa: N802
