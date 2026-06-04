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

"""MJCF to USD conversion pipeline."""

from __future__ import annotations

import gc
import importlib
import os
import shutil
import tempfile
from typing import Any

from isaacsim.asset.importer.utils.impl import (
    asset_utils,
    importer_utils,
    merge_mesh_utils,
    mjc_to_physx_conversion_utils,
    stage_utils,
)
from isaacsim.asset.transformer.rules import DEFAULT_PROFILE_PATH
from pxr import Sdf

from .config import MJCFImporterConfig


class MJCFImporter:
    """MuJoCo MJCF to USD importer.

    Uses mujoco-usd-converter to convert MJCF files to USD format.

    Args:
        config: Optional configuration for the import operation.

    Example:

    .. code-block:: python

        >>> from isaacsim.asset.importer.mjcf import MJCFImporter
        >>> MJCFImporter()
        <...>
    """

    def __init__(self, config: MJCFImporterConfig | None = None) -> None:
        self._config = config if config else MJCFImporterConfig()
        self.converter: Any = None

    @property
    def config(self) -> MJCFImporterConfig:
        """Get the importer configuration.

        Returns:
            Current importer configuration.

        Example:

        .. code-block:: python

            >>> from isaacsim.asset.importer.mjcf import MJCFImporter, MJCFImporterConfig

            >>> importer = MJCFImporter()
            >>> importer.config  # doctest: +ELLIPSIS
            MJCFImporterConfig(...)
        """
        return self._config

    @config.setter
    def config(self, config: MJCFImporterConfig) -> None:
        self._config = config

    def import_mjcf(self, config: MJCFImporterConfig | None = None) -> str:
        """Import an MJCF file and convert it to USD.

        Args:
            config: Optional configuration for the import operation.
                If not provided, the stored importer configuration will be used.

        Returns:
            Path to the generated USD file.

        Raises:
            ValueError: If the MJCF path is not configured or if the file
                does not have a ``.xml`` extension.
            FileNotFoundError: If the MJCF file does not exist at the given path.

        Example:

        .. code-block:: python

            >>> from isaacsim.asset.importer.mjcf import MJCFImporter, MJCFImporterConfig

            >>> importer = MJCFImporter()
            >>> config = MJCFImporterConfig(mjcf_path="/tmp/robot.xml")
            >>> importer.config = config
            >>> # output_path = importer.import_mjcf()
        """
        if config is not None:
            self.config = config

        if not self.config.mjcf_path:
            raise ValueError("MJCF path is not set in the importer configuration.")

        mjcf_path = os.path.normpath(self.config.mjcf_path)
        if not os.path.exists(mjcf_path):
            raise FileNotFoundError(f"MJCF file does not exist at path: {mjcf_path}")

        robot_name = importer_utils.parse_robot_name(mjcf_path, expected_extension=".xml")

        if self.config.usd_path is None:
            self.config.usd_path = os.path.normpath(os.path.dirname(mjcf_path))

        usd_path = os.path.normpath(self.config.usd_path)

        if self.config.debug_mode:
            scratch_dir = os.path.normpath(os.path.join(usd_path, f"_debug_{robot_name}"))
            os.makedirs(scratch_dir, exist_ok=True)
        else:
            scratch_dir = tempfile.mkdtemp(prefix=f"mjcf_import_{robot_name}_")

        try:
            usdex_path = importer_utils.resolve_unique_path(
                os.path.normpath(os.path.join(scratch_dir, f"usdex_{robot_name}")), is_file=False
            )
            temp_dir = importer_utils.resolve_unique_path(
                os.path.normpath(os.path.join(scratch_dir, f"temp_{robot_name}")), is_file=False
            )
            intermediate_path = os.path.normpath(os.path.join(temp_dir, f"{robot_name}.usd"))

            if self.converter is None:
                mujoco_usd_converter = importlib.import_module("mujoco_usd_converter")
                self.converter = mujoco_usd_converter.Converter(layer_structure=False, scene=self.config.import_scene)

            asset: Sdf.AssetPath = self.converter.convert(mjcf_path, usdex_path)

            # Now open the usdex stage in the USD context
            self.stage = stage_utils.open_stage(asset.path)

            if not self.stage:
                raise ValueError(f"Failed to open usdex stage at path: {asset.path}")

            importer_utils.add_joint_schemas(self.stage)

            if self.config.fix_base is True:
                asset_utils.apply_fix_base(self.stage)
            elif self.config.fix_base is False:
                asset_utils.apply_floating_base(self.stage)

            if self.config.link_density:
                asset_utils.apply_link_density(self.stage, self.config.link_density)

            has_gain_overrides = (
                self.config.override_gain_type is not None
                or self.config.override_bias_type is not None
                or self.config.override_gain_prm is not None
                or self.config.override_bias_prm is not None
            )
            if has_gain_overrides:
                asset_utils.apply_mjc_actuator_gains(
                    self.stage,
                    gain_type=self.config.override_gain_type,
                    bias_type=self.config.override_bias_type,
                    gain_prm=self.config.override_gain_prm,
                    bias_prm=self.config.override_bias_prm,
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
                mjc_to_physx_conversion_utils.convert_mjc_to_physx(self.stage)

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
                if self.config.run_multi_physics_conversion:
                    physx_layer_path = os.path.normpath(os.path.join(output_dir, "payloads", "Physics", "physx.usda"))
                    mjc_to_physx_conversion_utils.combine_overconstrained_joints_in_physx_layer(physx_layer_path)
            else:
                os.makedirs(output_dir, exist_ok=True)
                stage_utils.save_stage(self.stage, final_path)

            self.stage = None
            gc.collect()
        finally:
            if not self.config.debug_mode:
                shutil.rmtree(scratch_dir, ignore_errors=True)

        return final_path
