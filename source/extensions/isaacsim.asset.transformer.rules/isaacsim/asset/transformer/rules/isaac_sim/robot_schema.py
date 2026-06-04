# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

"""Rule for applying the Isaac Sim robot schema to an asset prim."""

from __future__ import annotations

import os

import usd.schema.isaac.robot_schema as rs
from isaacsim.asset.transformer import RuleConfigurationParam, RuleInterface
from pxr import Sdf, Usd
from usd.schema.isaac.robot_schema import utils as robot_schema_utils

from .. import utils


class RobotSchemaRule(RuleInterface):
    """Apply Isaac Sim robot schema to a target prim.

    Uses the default prim when no explicit prim path is provided.
    """

    def _get_destination_layer(self, destination: str, stage_name: str) -> tuple[Sdf.Layer | None, str]:
        """Resolve the destination layer and label for robot schema output.

        Args:
            destination: Destination folder or label for the output.
            stage_name: Output stage filename.

        Returns:
            Tuple of (destination layer or None, destination label).

        """
        destination_label = os.path.join(destination, stage_name) if stage_name else destination
        if not destination_label:
            return None, ""

        destination_path = os.path.join(self.package_root, destination_label)
        layer = utils.find_or_create_layer(destination_path, self.source_stage)
        return layer, destination_label

    def _insert_sublayer(self, sublayer_path: str, root_layer: Sdf.Layer) -> bool:
        """Insert a sublayer into the provided root layer.

        Args:
            sublayer_path: Path to the sublayer to insert.
            root_layer: The layer to insert the sublayer into.

        Returns:
            True if the sublayer was inserted successfully.

        """
        if not sublayer_path or not root_layer:
            return False

        resolved_path = sublayer_path
        if not os.path.isabs(sublayer_path):
            resolved_path = os.path.join(os.path.dirname(root_layer.identifier), sublayer_path)

        if not os.path.exists(resolved_path):
            self.log_operation(f"Sublayer not found: {resolved_path}")
            return False

        explicit_path = sublayer_path if os.path.isabs(sublayer_path) else utils.make_explicit_relative(sublayer_path)
        if resolved_path not in root_layer.subLayerPaths and explicit_path not in root_layer.subLayerPaths:
            root_layer.subLayerPaths.insert(0, explicit_path)
            self.log_operation(f"Inserted sublayer: {explicit_path}")
            return True
        return False

    def get_configuration_parameters(self) -> list[RuleConfigurationParam]:
        """Return the configuration parameters for this rule.

        Returns:
            List of configuration parameters.

        Example:

        .. code-block:: python

            params = rule.get_configuration_parameters()

        """
        return [
            RuleConfigurationParam(
                name="prim_path",
                display_name="Prim Path",
                param_type=str,
                description="Prim path to apply the Robot schema to. Defaults to the stage default prim.",
                default_value=None,
            ),
            RuleConfigurationParam(
                name="stage_name",
                display_name="Stage Name",
                param_type=str,
                description="Name of the output USD file for robot schema opinions.",
                default_value="robot_schema.usda",
            ),
            RuleConfigurationParam(
                name="add_sites",
                display_name="Add Sites",
                param_type=bool,
                description="Add sites to the robot. Scans child Xforms with no children under Link prims.",
                default_value=True,
            ),
            RuleConfigurationParam(
                name="sites_last",
                display_name="Sites Last",
                param_type=bool,
                description="If False, sites are added after their parent link. If True, all sites are added at the end.",
                default_value=False,
            ),
            RuleConfigurationParam(
                name="sublayer",
                display_name="Sublayer",
                param_type=str,
                description="Optional sublayer to include on the input stage prior to applying the robot schema.",
                default_value=None,
            ),
        ]

    def process_rule(self) -> str | None:
        """Apply Robot, Link, and Joint schemas and populate robot relationships.

        Returns:
            None (this rule does not change the working stage).

        Example:

        .. code-block:: python

            rule.process_rule()

        """
        params = self.args.get("params", {}) or {}
        prim_path = params.get("prim_path") or ""
        stage_name = params.get("stage_name") or "robot_schema.usda"
        add_sites = params.get("add_sites", True)
        sites_last = params.get("sites_last", False)
        sublayer = params.get("sublayer") or ""
        destination = self.destination_path or ""

        root_layer = self.source_stage.GetRootLayer()

        # Determine prim path
        if not prim_path:
            prim_path = utils.get_default_prim_path(self.source_stage)

        # Set up destination layer
        destination_layer, destination_label = self._get_destination_layer(destination, stage_name)

        # Determine edit layer and whether we're editing a separate layer
        if destination_layer and destination_layer.identifier != root_layer.identifier:
            edit_layer = destination_layer
            editing_separate_layer = True
            self.log_operation(
                f"RobotSchemaRule start prim={prim_path} destination={destination_label} add_sites={add_sites} sites_last={sites_last}"
            )
        else:
            edit_layer = root_layer
            editing_separate_layer = False
            self.log_operation(
                f"RobotSchemaRule start prim={prim_path} destination=source add_sites={add_sites} sites_last={sites_last}"
            )

        # Insert sublayer if provided (only modify root layer if we're editing it directly)

        if editing_separate_layer:
            self._insert_sublayer(destination, root_layer)

        if sublayer:
            self._insert_sublayer(sublayer, root_layer)

        # Add destination layer as sublayer only if it's different from root layer
        if editing_separate_layer:
            rel_path = utils.make_explicit_relative(
                os.path.relpath(edit_layer.realPath, os.path.dirname(root_layer.realPath))
            ).replace("\\", "/")
            if rel_path not in root_layer.subLayerPaths and edit_layer.identifier not in root_layer.subLayerPaths:
                root_layer.subLayerPaths.append(rel_path)
                self.log_operation(f"Added robot schema layer as sublayer: {rel_path}")

        robot_prim = self.source_stage.GetPrimAtPath(prim_path)
        if not robot_prim or not robot_prim.IsValid():
            self.log_operation(f"RobotSchemaRule skipped: invalid prim path {prim_path}")
            return None

        with Usd.EditContext(self.source_stage, edit_layer):
            # Check for existing robot schema
            has_existing_schema = robot_prim.HasAPI(rs.Classes.ROBOT_API.value)

            if has_existing_schema:
                self.log_operation("RobotAPI already applied, recalculating schema while preserving order")
                robot_schema_utils.UpdateDeprecatedSchemas(robot_prim)
                self.log_operation("Updated deprecated schemas")

                # Recalculate schema - preserves existing order, removes invalid, appends new
                root_link, root_joint = robot_schema_utils.RecalculateRobotSchema(
                    self.source_stage,
                    robot_prim,
                    robot_prim,
                    detect_sites=add_sites,
                    sites_last=sites_last,
                )
            else:
                rs.ApplyRobotAPI(robot_prim)
                self.log_operation("Applied RobotAPI to target prim")

                # Populate robot schema from articulation (with site detection if enabled)
                root_link, root_joint = robot_schema_utils.PopulateRobotSchemaFromArticulation(
                    self.source_stage,
                    robot_prim,
                    robot_prim,
                    detect_sites=add_sites,
                    sites_last=sites_last,
                )

        if root_link:
            self.log_operation(f"Detected root link: {root_link.GetPath()}")
        else:
            self.log_operation("No articulation root link detected")

        if root_joint:
            self.log_operation(f"Detected root joint: {root_joint.GetPath()}")
        else:
            self.log_operation("No articulation root joint detected")

        # Ensure defaultPrim is set (may not have been set if earlier
        # schema/property routing rules found no matching content).
        if not edit_layer.defaultPrim:
            default_prim = self.source_stage.GetDefaultPrim()
            if default_prim and default_prim.IsValid():
                edit_layer.defaultPrim = default_prim.GetName()

        # Save the edit layer
        edit_layer.Save()
        self.add_affected_stage(edit_layer.identifier)

        # Discard any changes to root layer if we were editing a separate layer
        if editing_separate_layer:
            root_layer.Reload()
            self.log_operation("Discarded root layer changes (editing separate layer)")

        self.log_operation("RobotSchemaRule completed")

        return None
