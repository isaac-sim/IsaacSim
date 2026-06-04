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

"""Property routing rule for organizing USD properties by name pattern into separate layers."""

from __future__ import annotations

import os
from collections import defaultdict

from isaacsim.asset.transformer import RuleConfigurationParam, RuleInterface
from pxr import Sdf, Usd

from .. import utils


def copy_property_to_layer(
    src_prim: Usd.Prim,
    prop_name: str,
    dst_layer: Sdf.Layer,
) -> bool:
    """Copy a property spec from source prim to destination layer as an override.

    Copies the property from the strongest opinion in the prim stack to the
    destination layer. The destination prim is created as an over if it doesn't exist.

    Args:
        src_prim: Source prim from the composed stage.
        prop_name: Name of the property to copy.
        dst_layer: Destination layer to copy to.

    Returns:
        True if the copy succeeded, False otherwise.

    Example:

    .. code-block:: python

        success = copy_property_to_layer(prim, "physics:mass", layer)

    """
    prim_path = src_prim.GetPath()
    prop_path = prim_path.AppendProperty(prop_name)

    # Ensure destination prim spec exists as an over
    dst_prim_spec = utils.ensure_prim_spec_in_layer(dst_layer, prim_path)
    if not dst_prim_spec:
        return False

    # Skip if property already exists in destination
    if dst_layer.GetPropertyAtPath(prop_path):
        return True

    # Find the strongest opinion for this property in the prim stack
    for spec in src_prim.GetPrimStack():
        if spec.layer == dst_layer:
            continue

        src_prop_spec = spec.layer.GetPropertyAtPath(prop_path)
        if src_prop_spec:
            # Copy the spec to destination (strongest opinion)
            Sdf.CopySpec(spec.layer, prop_path, dst_layer, prop_path)
            return True

    return False


def remove_property_from_source_layers(
    prim: Usd.Prim,
    prop_name: str,
    exclude_layer: Sdf.Layer,
) -> tuple[int, set[Sdf.Layer]]:
    """Remove property specs from all layers in the prim stack except the excluded layer.

    Args:
        prim: The prim whose property specs should be removed.
        prop_name: Name of the property to remove.
        exclude_layer: Layer to exclude from removal (typically the destination).

    Returns:
        Tuple of (removed_count, set of modified layers).

    Example:

    .. code-block:: python

        removed_count, layers = remove_property_from_source_layers(prim, "physics:mass", layer)

    """
    prim_path = prim.GetPath()
    prop_path = prim_path.AppendProperty(prop_name)
    removed_count = 0
    modified_layers = set()

    for spec in prim.GetPrimStack():
        if spec.layer == exclude_layer:
            continue

        try:
            if prop_name in spec.properties:
                del spec.properties[prop_name]
                removed_count += 1
                modified_layers.add(spec.layer)
        except Exception:
            pass

    return removed_count, modified_layers


class PropertyRoutingRule(RuleInterface):
    """Route properties matching name patterns to a separate layer.

    This rule identifies properties with names matching specified regex patterns,
    copies the property specs to a dedicated layer as overrides, and removes the
    property specs from all source layers. This allows organizing specific properties
    (e.g., physics properties, custom attributes) into modular layers.
    """

    def get_configuration_parameters(self) -> list[RuleConfigurationParam]:
        """Return the configuration parameters for this rule.

        Returns:
            List of configuration parameters for property patterns and output file.

        Example:

        .. code-block:: python

            params = rule.get_configuration_parameters()

        """
        return [
            RuleConfigurationParam(
                name="properties",
                display_name="Property Patterns",
                param_type=list,
                description="List of regex patterns to match property names (e.g., 'physics:.*', 'custom:.*')",
                default_value=None,
            ),
            RuleConfigurationParam(
                name="ignore_properties",
                display_name="Ignore Property Patterns",
                param_type=list,
                description="List of regex patterns to ignore, overrides properties matches",
                default_value=None,
            ),
            utils.make_stage_name_param("properties.usda", "Name of the output USD file for the properties"),
            utils.make_scope_param("Root path to search for matching properties (default: '/')"),
            utils.make_prim_names_param(),
            utils.make_ignore_prim_names_param(),
        ]

    def process_rule(self) -> str | None:
        """Move property specs matching patterns to the destination layer as overrides.

        Configuration is provided via ``params`` for property patterns, scope,
        name filters, and output stage name.

        Returns:
            None (this rule does not change the working stage).

        Example:

        .. code-block:: python

            rule.process_rule()

        """
        params = self.args.get("params", {}) or {}
        property_patterns = params.get("properties") or []
        ignore_property_patterns = params.get("ignore_properties") or []

        if not property_patterns:
            self.log_operation("No property patterns specified, skipping")
            return None

        # Compile regex patterns
        compiled_patterns = utils.compile_patterns(property_patterns, self.log_operation)
        if not compiled_patterns:
            self.log_operation("No valid property patterns after compilation, skipping")
            return None

        compiled_ignore_patterns = utils.compile_patterns(ignore_property_patterns, self.log_operation)

        destination_path = self.destination_path
        stage_name = params.get("stage_name") or "properties.usda"
        scope = params.get("scope") or "/"
        prim_names = params.get("prim_names") or [".*"]
        ignore_prim_names = params.get("ignore_prim_names") or []
        destination_label = os.path.join(destination_path, stage_name)

        self.log_operation(f"PropertyRoutingRule start destination={destination_label}")
        self.log_operation(f"Property patterns: {', '.join(property_patterns)}")
        if ignore_property_patterns:
            self.log_operation(f"Ignoring properties: {', '.join(ignore_property_patterns)}")
        self.log_operation(f"Search scope: {scope}")
        if prim_names != ["*"]:
            self.log_operation(f"Prim name patterns: {', '.join(prim_names)}")
        if ignore_prim_names:
            self.log_operation(f"Ignoring prim names: {', '.join(ignore_prim_names)}")

        # Resolve output path relative to package root
        properties_output_path = os.path.join(self.package_root, destination_label)

        # Get the scope prim to search under
        scope_prim = utils.get_scope_root(self.source_stage, scope, fallback_to_pseudo_root=False)
        if not scope_prim:
            self.log_operation(f"Invalid scope path: {scope}")
            return None

        # First pass: collect all matching properties to determine if output file is needed
        matching_items: list[tuple[Usd.Prim, str]] = []

        for prim in Usd.PrimRange(scope_prim):
            if not prim.IsValid():
                continue

            # Check prim name filter
            if not utils.matches_prim_filter(prim.GetName(), prim_names, ignore_prim_names):
                continue

            for attr in prim.GetAttributes():
                attr_name = attr.GetName()
                if utils.matches_any_pattern(attr_name, compiled_patterns):
                    if utils.matches_any_pattern(attr_name, compiled_ignore_patterns):
                        continue
                    if attr.HasAuthoredValue() or attr.GetConnections():
                        matching_items.append((prim, attr_name))

            for rel in prim.GetRelationships():
                rel_name = rel.GetName()
                if utils.matches_any_pattern(rel_name, compiled_patterns):
                    if utils.matches_any_pattern(rel_name, compiled_ignore_patterns):
                        continue
                    if rel.HasAuthoredTargets():
                        matching_items.append((prim, rel_name))

        if not matching_items:
            self.log_operation("No matching properties found, skipping output file creation")
            return None

        # Open or create properties layer only if we have matches
        properties_layer = utils.find_or_create_layer(properties_output_path, self.source_stage)
        if not properties_layer:
            self.log_operation(f"Failed to create properties layer: {properties_output_path}")
            return None
        self.log_operation(f"Using properties layer: {properties_output_path}")

        # Process collected matching properties
        properties_processed = 0
        properties_removed = 0
        all_modified_layers: set[Sdf.Layer] = set()

        # Group by prim for logging
        prim_props = defaultdict(list)
        for prim, prop_name in matching_items:
            prim_props[prim.GetPath()].append((prim, prop_name))

        for prim_path, props in prim_props.items():
            self.log_operation(f"Processing prim: {prim_path} ({len(props)} matching properties)")

            for prim, prop_name in props:
                # Copy property to destination layer
                success = copy_property_to_layer(prim, prop_name, properties_layer)
                if success:
                    properties_processed += 1

                    # Remove property specs from source layers
                    removed, modified_layers = remove_property_from_source_layers(prim, prop_name, properties_layer)
                    properties_removed += removed
                    all_modified_layers.update(modified_layers)

                    if removed > 0:
                        self.log_operation(f"  Moved property: {prop_name} (removed from {removed} source layer(s))")
                    else:
                        self.log_operation(f"  Copied property: {prop_name} (no source removal needed)")
                else:
                    self.log_operation(f"  Failed to copy property: {prop_name}")

        # Set default prim from the composed stage (more reliable than the
        # root layer when re-transforming an already-transformed asset).
        default_prim = self.source_stage.GetDefaultPrim()
        if default_prim and default_prim.IsValid():
            properties_layer.defaultPrim = default_prim.GetName()

        # Export the properties layer
        properties_layer.Export(properties_layer.identifier)

        # Save all modified source layers
        for layer in all_modified_layers:
            layer.Save()
            self.log_operation(f"Saved modified layer: {layer.identifier}")

        self.log_operation(
            f"Processed {properties_processed} property(s), removed {properties_removed} property spec(s) from source"
        )

        self.add_affected_stage(destination_label)
        self.log_operation("PropertyRoutingRule completed")

        return None
