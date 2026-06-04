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

"""Rule for deleting applied API schema opinions from a USD layer."""

from __future__ import annotations

import os
import re

from isaacsim.asset.transformer import RuleConfigurationParam, RuleInterface
from pxr import Sdf, Usd

from .. import utils


class RemoveSchemaRule(RuleInterface):
    """Delete schema opinions for matching prims in a USD layer."""

    def get_configuration_parameters(self) -> list[RuleConfigurationParam]:
        """Return the configuration parameters for this rule.

        Returns:
            List of configuration parameters for schema overwrite behavior.

        Example:

        .. code-block:: python

            params = rule.get_configuration_parameters()

        """
        return [
            RuleConfigurationParam(
                name="schema_patterns",
                display_name="Schema Patterns",
                param_type=list,
                description="List of regex patterns for applied API schemas to remove.",
                default_value=None,
            ),
            RuleConfigurationParam(
                name="prim_path_patterns",
                display_name="Prim Path Patterns",
                param_type=list,
                description="List of regex patterns for prim paths to update. Defaults to all prims.",
                default_value=None,
            ),
            RuleConfigurationParam(
                name="input_stage_path",
                display_name="Input Stage Path",
                param_type=str,
                description="Optional USD stage path to read from. Defaults to current stage.",
                default_value=None,
            ),
            RuleConfigurationParam(
                name="stage_name",
                display_name="Stage Name",
                param_type=str,
                description="Optional output USD file to delete schemas. Defaults to current stage.",
                default_value=None,
            ),
            RuleConfigurationParam(
                name="property_patterns",
                display_name="Property Patterns",
                param_type=list,
                description="List of regex patterns for properties to remove.",
                default_value=None,
            ),
            RuleConfigurationParam(
                name="clear_properties",
                display_name="Clear Properties",
                param_type=bool,
                description="If True, removes properties matching property patterns.",
                default_value=False,
            ),
        ]

    def process_rule(self) -> str | None:
        """Overwrite schema opinions on the destination layer.

        Returns:
            None (this rule does not change the working stage).

        Example:

        .. code-block:: python

            rule.process_rule()

        """
        params = self.args.get("params", {}) or {}
        schema_patterns = utils.compile_patterns(params.get("schema_patterns") or [])
        property_patterns = utils.compile_patterns(params.get("property_patterns") or [])
        if not schema_patterns and not property_patterns:
            self.log_operation("RemoveSchemaRule skipped: no schema or property patterns specified")
            return None

        prim_path_patterns = utils.compile_patterns(params.get("prim_path_patterns") or [])
        stage_name = params.get("stage_name")
        clear_properties = params.get("clear_properties", False)
        input_stage_path = params.get("input_stage_path") or ""

        destination_label = None
        if stage_name:
            destination_label = os.path.join(self.destination_path, stage_name)
        elif self.destination_path:
            _, ext = os.path.splitext(self.destination_path)
            if ext.lower() in utils.USD_EXTENSIONS:
                destination_label = self.destination_path

        destination_path = os.path.join(self.package_root, destination_label) if destination_label else ""

        source_stage = self.source_stage
        if input_stage_path:
            source_stage = self.args.get("input_stage") or Usd.Stage.Open(input_stage_path)
            if not source_stage:
                self.log_operation(f"Failed to open input stage: {input_stage_path}")
                return None

        self.log_operation(
            f"RemoveSchemaRule start destination={destination_label} "
            f"schemas={len(schema_patterns)} properties={len(property_patterns)}"
        )

        if destination_label:
            output_layer = utils.find_or_create_layer(destination_path, self.source_stage)
            if not output_layer:
                self.log_operation(f"Failed to open schema layer: {destination_path}")
                return None
        else:
            output_layer = source_stage.GetRootLayer()

        matching_prim_paths = self._collect_matching_prim_paths(source_stage, prim_path_patterns)
        if not matching_prim_paths:
            self.log_operation("No prims matched, skipping")
            return None

        removed_schema_count = 0
        removed_property_count = 0

        for prim_path in matching_prim_paths:
            prim_spec = utils.ensure_prim_spec_in_layer(output_layer, Sdf.Path(prim_path))
            if not prim_spec:
                continue
            source_prim = source_stage.GetPrimAtPath(prim_path)
            if not source_prim or not source_prim.IsValid():
                continue

            if schema_patterns:
                removed_schema_count += self._remove_matching_schemas(prim_spec, source_prim, schema_patterns)

            if clear_properties and property_patterns:
                removed_property_count += self._remove_matching_properties(prim_spec, property_patterns)

        if destination_label:
            output_layer.Save()
            self.add_affected_stage(destination_label)
        else:
            source_root = source_stage.GetRootLayer()
            source_root.Save()
            self.add_affected_stage(source_root.identifier)
        self.log_operation("RemoveSchemaRule completed")
        return None

    def _collect_matching_prim_paths(self, stage: Usd.Stage, prim_path_patterns: list[re.Pattern[str]]) -> list[str]:
        """Collect prim paths that match the provided regex patterns.

        Args:
            stage: Stage to traverse for prim paths.
            prim_path_patterns: Compiled regex patterns for prim paths.

        Returns:
            List of matching prim paths.

        """
        if not prim_path_patterns:
            return [prim.GetPath().pathString for prim in stage.Traverse()]

        matched = []
        for prim in stage.Traverse():
            prim_path = prim.GetPath().pathString
            if utils.matches_any_pattern(prim_path, prim_path_patterns):
                matched.append(prim_path)
        return matched

    def _remove_matching_schemas(
        self, prim_spec: Sdf.PrimSpec, source_prim: Usd.Prim, schema_patterns: list[re.Pattern[str]]
    ) -> int:
        """Delete applied API schemas matching the patterns from a prim spec.

        Args:
            prim_spec: Prim spec to update.
            source_prim: Source prim used to determine applied schemas.
            schema_patterns: Compiled regex patterns for schema tokens.

        Returns:
            Count of deleted schema tokens.

        """
        applied_schemas = [str(token) for token in source_prim.GetAppliedSchemas()]
        matching_schemas = [schema for schema in applied_schemas if utils.matches_any_pattern(schema, schema_patterns)]
        if not matching_schemas:
            return 0

        api_schemas = prim_spec.GetInfo("apiSchemas")
        if isinstance(api_schemas, Sdf.TokenListOp):
            deleted_items = list(api_schemas.deletedItems or [])
            explicit_items = list(api_schemas.explicitItems or [])
            prepended_items = list(api_schemas.prependedItems or [])
            appended_items = list(api_schemas.appendedItems or [])
            ordered_items = list(api_schemas.orderedItems or [])
        else:
            deleted_items = []
            explicit_items = []
            prepended_items = []
            appended_items = []
            ordered_items = []

        for schema in matching_schemas:
            if schema not in deleted_items:
                deleted_items.append(schema)

        new_list_op = Sdf.TokenListOp()
        if explicit_items:
            new_list_op.explicitItems = explicit_items
        if prepended_items:
            new_list_op.prependedItems = prepended_items
        if appended_items:
            new_list_op.appendedItems = appended_items
        if deleted_items:
            new_list_op.deletedItems = deleted_items
        if ordered_items:
            new_list_op.orderedItems = ordered_items
        prim_spec.SetInfo("apiSchemas", new_list_op)

        return len(matching_schemas)

    def _remove_matching_properties(self, prim_spec: Sdf.PrimSpec, property_patterns: list[re.Pattern[str]]) -> int:
        """Remove properties matching the patterns from a prim spec.

        Args:
            prim_spec: Prim spec to update.
            property_patterns: Compiled regex patterns for property names.

        Returns:
            Count of removed properties.

        """
        removed_count = 0
        for prop_name, _ in list(prim_spec.properties.items()):
            prop_str = str(prop_name)
            if utils.matches_any_pattern(prop_str, property_patterns):
                del prim_spec.properties[prop_name]
                removed_count += 1
        return removed_count
