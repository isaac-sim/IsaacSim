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

"""URDF to USD conversion utilities."""

from __future__ import annotations

import gc
import importlib
import logging
import os
import shutil
import tempfile
from typing import Any

from isaacsim.asset.importer.utils.impl import (
    asset_utils,
    importer_utils,
    merge_mesh_utils,
    stage_utils,
    urdf_to_mjc_physx_conversion_utils,
)
from isaacsim.asset.transformer.rules import DEFAULT_PROFILE_PATH
from pxr import Sdf

from .config import URDFImporterConfig
from .drive_reconstruction import parse_source_drive_breadcrumbs, reconstruct_source_drives
from .geometry_reconstruction import parse_source_geometry_breadcrumbs, reconstruct_source_geometry
from .joint_reconstruction import parse_source_joint_breadcrumbs, reconstruct_source_joints
from .urdf_utils import _rewrite_relative_mesh_paths_to_absolute, merge_fixed_joints

_logger = logging.getLogger(__name__)


class URDFImporter:
    """URDF to USD importer.

    Uses urdf-usd-converter to convert URDF files to USD format.

    Args:
        config: Optional configuration for the import operation.

    Example:

    .. code-block:: python

        >>> from isaacsim.asset.importer.urdf import URDFImporter
        >>> URDFImporter()
        <...>
    """

    def __init__(self, config: URDFImporterConfig | None = None) -> None:
        self._config = config if config else URDFImporterConfig()
        self.converter: Any = None

    @property
    def config(self) -> URDFImporterConfig:
        """Get the importer configuration.

        Returns:
            Current importer configuration.

        Example:

        .. code-block:: python

            >>> from isaacsim.asset.importer.urdf import URDFImporter, URDFImporterConfig

            >>> importer = URDFImporter()
            >>> importer.config  # doctest: +ELLIPSIS
            URDFImporterConfig(...)
        """
        return self._config

    @config.setter
    def config(self, config: URDFImporterConfig) -> None:
        self._config = config

    def import_urdf(self, config: URDFImporterConfig | None = None) -> str:
        """Import a URDF file and convert it to USD.

        Args:
            config: Optional configuration for the import operation.
                If not provided, the stored importer configuration will be used.

        Returns:
            Path to the generated USD file.

        Raises:
            ValueError: If the URDF path is not configured or if the file
                does not have a ``.urdf`` extension.

        Example:

        .. code-block:: python

            >>> from isaacsim.asset.importer.urdf import URDFImporter, URDFImporterConfig

            >>> importer = URDFImporter()
            >>> config = URDFImporterConfig(urdf_path="/tmp/robot.urdf")
            >>> importer.config = config
            >>> # output_path = importer.import_urdf()
        """
        if config is not None:
            self.config = config

        if not self.config.urdf_path:
            raise ValueError("URDF path is not set in the importer configuration.")

        urdf_path = os.path.normpath(self.config.urdf_path)
        source_urdf_dir = os.path.dirname(urdf_path)
        robot_name = importer_utils.parse_robot_name(urdf_path, expected_extension=".urdf")

        if self.config.usd_path is None:
            self.config.usd_path = os.path.normpath(source_urdf_dir)

        usd_path = os.path.normpath(self.config.usd_path)

        if self.config.debug_mode:
            scratch_dir = os.path.normpath(os.path.join(usd_path, f"_debug_{robot_name}"))
            os.makedirs(scratch_dir, exist_ok=True)
        else:
            scratch_dir = tempfile.mkdtemp(prefix=f"urdf_import_{robot_name}_")

        if self.config.merge_fixed_joints:
            merged_urdf_path = os.path.normpath(os.path.join(scratch_dir, f"{robot_name}_merged.urdf"))
            urdf_path = merge_fixed_joints(urdf_path, merged_urdf_path)
            if os.path.dirname(urdf_path) != source_urdf_dir:
                _rewrite_relative_mesh_paths_to_absolute(urdf_path, source_urdf_dir)

        try:
            usdex_path = importer_utils.resolve_unique_path(
                os.path.normpath(os.path.join(scratch_dir, f"usdex_{robot_name}")), is_file=False
            )
            temp_dir = importer_utils.resolve_unique_path(
                os.path.normpath(os.path.join(scratch_dir, f"temp_{robot_name}")), is_file=False
            )
            intermediate_path = os.path.normpath(os.path.join(temp_dir, f"{robot_name}.usd"))
            if self.converter is None:
                urdf_usd_converter = importlib.import_module("urdf_usd_converter")
                self.converter = urdf_usd_converter.Converter(layer_structure=False, scene=False)
            self.converter.params.ros_packages = list(self.config.ros_package_paths)
            asset: Sdf.AssetPath = self.converter.convert(urdf_path, usdex_path)

            # Now open the flattened stage in the USD context
            self.stage = stage_utils.open_stage(asset.path)

            if not self.stage:
                raise ValueError(f"Failed to open flattened stage at path: {asset.path}")

            breadcrumbs = parse_source_geometry_breadcrumbs(urdf_path)
            if breadcrumbs:
                n = reconstruct_source_geometry(self.stage, breadcrumbs)
                if n:
                    _logger.info(f"Reconstructed {n} source geometry primitives from breadcrumbs")

            joint_breadcrumbs = parse_source_joint_breadcrumbs(urdf_path)
            if joint_breadcrumbs:
                n = reconstruct_source_joints(self.stage, joint_breadcrumbs)
                if n:
                    _logger.info(f"Reconstructed {n} source joint types from breadcrumbs")

            importer_utils.remove_custom_scopes(self.stage)
            importer_utils.add_joint_schemas(self.stage)

            drive_breadcrumbs = parse_source_drive_breadcrumbs(urdf_path)
            if drive_breadcrumbs:
                n = reconstruct_source_drives(self.stage, drive_breadcrumbs)
                if n:
                    _logger.info(f"Reconstructed {n} joint drive configurations from breadcrumbs")
            if self.config.fix_base is True:
                asset_utils.apply_fix_base(self.stage)
            elif self.config.fix_base is False:
                asset_utils.apply_floating_base(self.stage)

            if self.config.link_density:
                asset_utils.apply_link_density(self.stage, self.config.link_density)

            has_drive_overrides = (
                self.config.joint_drive_type is not None
                or self.config.joint_target_type is not None
                or self.config.override_joint_stiffness is not None
                or self.config.override_joint_damping is not None
            )
            if has_drive_overrides:
                asset_utils.apply_joint_drives(
                    self.stage,
                    drive_type=self.config.joint_drive_type,
                    target_type=self.config.joint_target_type,
                    stiffness=self.config.override_joint_stiffness,
                    damping=self.config.override_joint_damping,
                )

            if self.config.merge_mesh:
                merge_mesh_utils.clean_mesh_operation(self.stage)
                merge_mesh_utils.generate_mesh_uv_normals_operation(self.stage)
                merge_mesh_utils.merge_meshes_operation(self.stage)

            if self.config.collision_from_visuals:
                importer_utils.collision_from_visuals(self.stage, self.config.collision_type)

            importer_utils.enable_self_collision(self.stage, self.config.allow_self_collision)
            importer_utils.create_robot_schema(self.stage, robot_type=self.config.robot_type)

            if self.config.run_multi_physics_conversion:
                urdf_to_mjc_physx_conversion_utils.convert_joints_attributes(self.stage)

            output_dir = importer_utils.resolve_unique_path(
                os.path.normpath(os.path.join(usd_path, robot_name)), is_file=False
            )
            final_path = os.path.normpath(os.path.join(output_dir, f"{robot_name}.usda"))

            if self.config.run_asset_transformer:
                stage_utils.save_stage(self.stage, intermediate_path)
                if self.config.debug_mode:
                    log_path = os.path.normpath(
                        os.path.join(os.path.dirname(intermediate_path), f"isaacsim_structure_logs_{robot_name}.json")
                    )
                else:
                    log_path = None
                importer_utils.run_asset_transformer_profile(
                    input_stage_path=intermediate_path,
                    output_package_root=output_dir,
                    profile_json_path=DEFAULT_PROFILE_PATH,
                    log_path=log_path,
                )
            else:
                os.makedirs(output_dir, exist_ok=True)
                stage_utils.save_stage(self.stage, final_path)

            self.stage = None
            gc.collect()
        finally:
            if not self.config.debug_mode:
                shutil.rmtree(scratch_dir, ignore_errors=True)

        return final_path
