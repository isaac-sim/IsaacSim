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

"""Deprecated MJCF importer command implementations."""

import carb
import omni.client
import omni.kit.commands
from isaacsim.asset.importer.mjcf import MJCFImporter, MJCFImporterConfig
from isaacsim.asset.importer.utils import stage_utils


class MJCFCreateImportConfig(omni.kit.commands.Command):
    """Deprecated command to create an ImportConfig object.

    Should be used with the `MJCFCreateAsset` command.

    .. deprecated:: Use MJCFImporterConfig() directly instead.
    """

    def __init__(self) -> None:
        pass

    def do(self) -> MJCFImporterConfig:
        """Execute the command to create an import configuration.

        Returns:
            New MJCFImporterConfig instance.
        """
        carb.log_warn(
            "MJCFCreateImportConfig is deprecated and will be removed in a future version. Use MJCFImporterConfig() class instead."
        )
        return MJCFImporterConfig()

    def undo(self) -> None:
        """Undo the command (no-op)."""


class MJCFCreateAsset(omni.kit.commands.Command):
    """Deprecated command to parse and import an MJCF file.

    Args:
        mjcf_path: The absolute path to the MJCF file.
        import_config: Import configuration.
        prim_path: Path to the robot on the USD stage.
        dest_path: Destination path for robot USD. Default is "" which will load the robot in-memory on the open stage.

    .. deprecated:: Use MJCFImporter() directly instead.
    """

    def __init__(
        self,
        mjcf_path: str = "",
        import_config: MJCFImporterConfig = MJCFImporterConfig(),
        prim_path: str = "",
        dest_path: str = "",
    ) -> None:
        carb.log_warn(
            f"Warning: MJCFCreateAsset is deprecated and will be removed in a future version. Use MJCFImporter() instead."
        )

        self.importer = MJCFImporter()
        self.config = import_config
        self.config.mjcf_path = mjcf_path
        self.config.usd_path = dest_path
        self.importer.config = self.config

    def do(self) -> str:
        """Execute the command to import the MJCF file.

        Returns:
            Path to the imported USD file.
        """
        path = self.importer.import_mjcf()
        stage_utils.open_stage(path)
        return path

    def undo(self) -> None:
        """Undo the command (no-op)."""


omni.kit.commands.register_all_commands_in_module(__name__)
