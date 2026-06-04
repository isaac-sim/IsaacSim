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

"""Prim routing rule for organizing USD prims by type into separate layers."""

from __future__ import annotations

import os

from isaacsim.asset.transformer import RuleConfigurationParam, RuleInterface
from pxr import Sdf, Usd

from .. import utils

# Re-export for backward compatibility with tests
merge_list_op = utils.merge_token_list_op
copy_composed_prim_to_layer = utils.copy_composed_prim_to_layer


def merge_path_list_op(existing_list: Sdf.PathListOp, new_paths: list) -> None:
    """Merge new paths into an existing path list op by prepending.

    Modifies the existing list in place. New paths are prepended, deletes are combined.

    Args:
        existing_list: Existing path list op to modify in place.
        new_paths: New paths from source to prepend.

    Example:

    .. code-block:: python

        merge_path_list_op(list_op, [Sdf.Path("/World")])

    """
    # Get existing paths to avoid duplicates
    existing_paths = set()
    for path in existing_list.GetAddedOrExplicitItems():
        existing_paths.add(str(path))

    # Prepend new paths (in reverse order since Prepend adds to front)
    for path in reversed(new_paths):
        if str(path) not in existing_paths:
            existing_list.Prepend(path)


def remove_prim_from_source_layers(prim: Usd.Prim, exclude_layer: Sdf.Layer) -> tuple:
    """Remove prim specs and all property specs from all layers in the prim stack except the excluded layer.

    This function ensures complete eradication of the prim by:
    1. Explicitly deleting all property specs (attributes and relationships) from each prim spec
    2. Clearing apiSchemas metadata from each prim spec
    3. Deleting the prim spec itself from the layer

    This is necessary because property specs can exist as overrides in layers (like robot_schema.usda)
    even when the prim is being moved elsewhere. Simply deleting the prim spec may leave orphaned
    property specs behind.

    Args:
        prim: The prim whose specs should be removed.
        exclude_layer: Layer to exclude from removal (typically the destination).

    Returns:
        Tuple of (removed_count, set of modified layers).

    Example:

    .. code-block:: python

        removed_count, layers = remove_prim_from_source_layers(prim, exclude_layer)

    """
    prim_path = prim.GetPath()
    removed_count = 0
    modified_layers = set()

    # Iterate through prim stack and remove specs
    for spec in prim.GetPrimStack():
        if spec.layer == exclude_layer:
            continue

        # Remove this prim spec from its layer
        try:
            layer = spec.layer

            # First, explicitly delete all property specs from this prim spec
            # This handles cases where properties were authored as overrides (e.g., from SchemaRoutingRule)
            for prop_name in list(spec.properties.keys()):  # noqa: SIM118
                del spec.properties[prop_name]

            # Then delete the prim spec itself (this also removes apiSchemas and other metadata)
            parent_spec = layer.GetPrimAtPath(prim_path.GetParentPath())
            if parent_spec and spec.name in parent_spec.nameChildren:
                del parent_spec.nameChildren[spec.name]
                removed_count += 1
                modified_layers.add(layer)
        except Exception:
            pass

    return removed_count, modified_layers


class PrimRoutingRule(RuleInterface):
    """Route prims matching type patterns to a separate layer.

    This rule identifies prims with types matching specified patterns (supporting wildcards),
    copies the complete composed prim definition to a dedicated layer, and removes the prim
    specs from all source layers. This allows organizing physics prims, render prims, or other
    typed prims into modular layers that can be selectively loaded.
    """

    def get_configuration_parameters(self) -> list[RuleConfigurationParam]:
        """Return the configuration parameters for this rule.

        Returns:
            List of configuration parameters for prim type patterns and output file.

        Example:

        .. code-block:: python

            params = rule.get_configuration_parameters()

        """
        return [
            RuleConfigurationParam(
                name="prim_types",
                display_name="Prim Types",
                param_type=list,
                description="List of regex patterns for prim types to route (e.g., 'Physics.*')",
                default_value=None,
            ),
            RuleConfigurationParam(
                name="ignore_prim_types",
                display_name="Ignore Prim Types",
                param_type=list,
                description="List of regex patterns for prim types to ignore, overrides prim_types matches",
                default_value=None,
            ),
            utils.make_stage_name_param("prims.usda", "Name of the output USD file for the prims"),
            utils.make_scope_param("Root path to search for matching prims (default: '/')"),
            utils.make_prim_names_param(),
            utils.make_ignore_prim_names_param(),
        ]

    def process_rule(self) -> str | None:
        """Move complete prim definitions to the destination layer.

        Configuration is provided via ``params`` for prim type patterns, scope,
        name filters, and output stage name.

        Returns:
            None (this rule does not change the working stage).

        Raises:
            ValueError: If ``prim_types`` is missing or empty. ``prim_types`` is
                the rule's primary selector; without it the rule has no defined
                work surface and the caller almost certainly misconfigured it.

        Example:

        .. code-block:: python

            rule.process_rule()

        """
        params = self.args.get("params", {}) or {}
        prim_types = params.get("prim_types") or []

        if not prim_types:
            raise ValueError(
                "PrimRoutingRule requires 'prim_types' to be configured with at least one type "
                "pattern. To disable a rule without removing its entry, set the rule spec's "
                "'enabled = false' flag instead -- the manager honors it and skips execution "
                "with no error reported."
            )

        destination_path = self.destination_path
        stage_name = params.get("stage_name") or "prims.usda"
        scope = params.get("scope") or "/"
        prim_names = params.get("prim_names") or [".*"]
        ignore_prim_names = params.get("ignore_prim_names") or []
        ignore_prim_types = params.get("ignore_prim_types") or []
        destination_label = os.path.join(destination_path, stage_name)

        self.log_operation(f"PrimRoutingRule start destination={destination_label}")
        self.log_operation(f"Prim type patterns: {', '.join(prim_types)}")
        if ignore_prim_types:
            self.log_operation(f"Ignoring prim types: {', '.join(ignore_prim_types)}")
        self.log_operation(f"Search scope: {scope}")
        if prim_names != ["*"]:
            self.log_operation(f"Prim name patterns: {', '.join(prim_names)}")
        if ignore_prim_names:
            self.log_operation(f"Ignoring prim names: {', '.join(ignore_prim_names)}")

        # Resolve output path relative to package root
        prims_output_path = os.path.join(self.package_root, destination_label)

        # Get the scope prim to search under
        scope_prim = utils.get_scope_root(self.source_stage, scope, fallback_to_pseudo_root=False)
        if not scope_prim:
            self.log_operation(f"Invalid scope path: {scope}")
            return None

        # Compile regex patterns
        compiled_types = utils.compile_patterns(prim_types, self.log_operation)
        compiled_ignore_types = utils.compile_patterns(ignore_prim_types, self.log_operation)
        if not compiled_types:
            self.log_operation("No valid prim type patterns after compilation, skipping")
            return None

        # Collect all matching prims first (to avoid iterator invalidation when removing)
        matching_prims = []
        for prim in Usd.PrimRange(scope_prim):
            # Check prim name filter
            if not utils.matches_prim_filter(prim.GetName(), prim_names, ignore_prim_names):
                continue

            # Check if this prim's type matches any pattern
            prim_type = prim.GetTypeName()
            if not prim_type:
                continue

            matched = utils.matches_any_pattern(str(prim_type), compiled_types)
            if not matched:
                continue

            ignored = utils.matches_any_pattern(str(prim_type), compiled_ignore_types)
            if not ignored:
                matching_prims.append((prim.GetPath(), prim_type))

        if not matching_prims:
            self.log_operation("No matching prims found, skipping output file creation")
            return None

        # Open or create prims layer only if we have matches
        prims_layer = utils.find_or_create_layer(prims_output_path, self.source_stage)
        if not prims_layer:
            self.log_operation(f"Failed to create prims layer: {prims_output_path}")
            return None
        self.log_operation(f"Using prims layer: {prims_output_path}")

        # Process collected prims
        prims_processed = 0
        prims_removed = 0
        all_modified_layers = set()

        for prim_path, prim_type in matching_prims:
            # Re-fetch the prim (in case stage has changed)
            prim = self.source_stage.GetPrimAtPath(prim_path)
            if not prim or not prim.IsValid():
                continue

            self.log_operation(f"Processing prim: {prim_path} (type: {prim_type})")

            # Copy the composed prim to destination layer
            success = copy_composed_prim_to_layer(prim, prims_layer, prim_path)
            if success:
                prims_processed += 1
                self.log_operation(f"Copied composed prim: {prim_path}")

                # Remove prim specs from source layers
                removed, modified_layers = remove_prim_from_source_layers(prim, prims_layer)
                prims_removed += removed
                all_modified_layers.update(modified_layers)
                if removed > 0:
                    self.log_operation(f"Removed {removed} prim spec(s) from source layers")
                else:
                    self.log_operation(
                        f"WARNING: Failed to remove prim specs for {prim_path} - may result in duplicates"
                    )
            else:
                self.log_operation(f"Failed to copy prim: {prim_path}")

        # Set default prim from the composed stage (more reliable than the
        # root layer when re-transforming an already-transformed asset).
        default_prim = self.source_stage.GetDefaultPrim()
        if default_prim and default_prim.IsValid():
            prims_layer.defaultPrim = default_prim.GetName()

        # Export the prims layer (Export does a clean serialization)
        prims_layer.Export(prims_layer.identifier)

        # Save all modified source layers (schemas layer, etc.)
        for layer in all_modified_layers:
            layer.Save()
            self.log_operation(f"Saved modified layer: {layer.identifier}")
        self.log_operation(f"Processed {prims_processed} prim(s), removed {prims_removed} prim spec(s) from source")

        self.add_affected_stage(destination_label)
        self.log_operation("PrimRoutingRule completed")

        return None
