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

"""Deprecated URDF importer command implementations."""

from __future__ import annotations

from typing import NoReturn

import carb
import omni.client
import omni.kit.commands
from isaacsim.asset.importer.urdf import URDFImporter, URDFImporterConfig
from omni.client import Result


class URDFCreateImportConfig(omni.kit.commands.Command):
    """Deprecated command to create an ImportConfig object.

    Should be used with the `URDFParseFile` and `URDFImportRobot` commands.

    .. deprecated:: Use URDFImporterConfig() directly instead.
    """

    def __init__(self) -> None:
        pass

    def do(self) -> URDFImporterConfig:
        """Execute the command to create an import configuration.

        Returns:
            New URDFImporterConfig instance.
        """
        carb.log_warn(
            "URDFCreateImportConfig is deprecated and will be removed in a future version. Use URDFImporterConfig() class instead."
        )
        return URDFImporterConfig()

    def undo(self) -> None:
        """Undo the command (no-op)."""


class URDFParseText(omni.kit.commands.Command):
    """Deprecated command to parse a URDF string.

    Args:
        urdf_text: The URDF string to parse.
        import_config: Import configuration.

    .. deprecated:: Parsing URDF strings is not supported. Use URDFImporter() with a file path instead.
    """

    def __init__(self, urdf_text: str = "", import_config: URDFImporterConfig = URDFImporterConfig()) -> None:
        carb.log_warn(
            "URDFParseText is deprecated and will be removed in a future version. Parsing URDF strings is not supported."
        )
        self.urdf_text = urdf_text
        self.import_config = import_config

    def do(self) -> NoReturn:
        """Execute the command to parse the URDF string.

        Raises:
            RuntimeError: Parsing URDF strings is no longer supported.
        """
        raise RuntimeError("Parsing URDF strings is no longer supported.")

    def undo(self) -> None:
        """Undo the command (no-op)."""


class URDFParseFile(omni.kit.commands.Command):
    """Deprecated command to parse a URDF file.

    Args:
        urdf_path: The absolute path to the URDF file.
        import_config: Import configuration.

    .. deprecated:: Use URDFImporter() directly instead.
    """

    def __init__(self, urdf_path: str = "", import_config: URDFImporterConfig = URDFImporterConfig()) -> None:
        carb.log_warn("URDFParseFile is deprecated and will be removed in a future version.")
        self.urdf_path = urdf_path
        self.import_config = import_config

    def do(self) -> NoReturn:
        """Execute the command to parse the URDF file.

        Raises:
            RuntimeError: Parsing URDF files is no longer supported.
        """
        raise RuntimeError("Parsing URDF files is no longer supported.")

    def undo(self) -> None:
        """Undo the command (no-op)."""


class URDFImportRobot(omni.kit.commands.Command):
    """Deprecated command to import a URDF robot.

    Args:
        urdf_path: The absolute path to the URDF file.
        urdf_robot: The robot model from URDFParseFile (optional, for backward compatibility).
        import_config: Import configuration.
        dest_path: Destination path for robot USD. Default is "" which will load the robot in-memory on the open stage.
        return_articulation_root_prim: Whether to return the articulation root prim instead of the robot USD path.

    .. deprecated:: Use URDFImporter() directly instead.
    """

    def __init__(
        self,
        urdf_path: str = "",
        urdf_robot: object | None = None,
        import_config: URDFImporterConfig = URDFImporterConfig(),
        dest_path: str = "",
        return_articulation_root_prim: bool = False,
    ) -> None:
        carb.log_warn(
            "URDFImportRobot is deprecated and will be removed in a future version. Use URDFImporter() directly instead."
        )

        self.urdf_path = urdf_path
        self.import_config = import_config
        self.import_config.urdf_path = urdf_path
        self.import_config.usd_path = dest_path
        self.return_articulation_root_prim = return_articulation_root_prim

    def do(self) -> tuple[Result, str]:
        """Execute the command to import the URDF file.

        Returns:
            Tuple of (Result, prim_path). Falls back to C++ command if urdf_robot is provided,
            otherwise uses URDFImporter.
        """
        importer = URDFImporter()
        importer.config = self.import_config
        path = importer.import_urdf()

        if self.return_articulation_root_prim:
            carb.log_warn("Return articulation root prim is not supported in the command.")
            carb.log_warn("Please use the URDFImporter() class directly instead.")
            carb.log_warn("Returning the USD path instead. Use the following code to get all articulation roots: ")
            carb.log_warn(
                "articulation_roots = [prim.GetPath().pathString for prim in stage.Traverse() if UsdPhysics.ArticulationRootAPI(prim).GetPrim()]"
            )
            return path
        else:
            return path


omni.kit.commands.register_all_commands_in_module(__name__)
