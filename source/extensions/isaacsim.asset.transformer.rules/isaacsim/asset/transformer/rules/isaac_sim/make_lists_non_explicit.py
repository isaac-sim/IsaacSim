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

"""Rule for converting explicit list ops to non-explicit list ops."""

from __future__ import annotations

from isaacsim.asset.transformer import RuleConfigurationParam, RuleInterface
from pxr import Sdf, Usd

from .. import utils

_LIST_OP_PREPEND: str = "prepend"
_LIST_OP_APPEND: str = "append"
_LIST_OP_ALIASES: dict[str, str] = {
    "prepend": _LIST_OP_PREPEND,
    "prepended": _LIST_OP_PREPEND,
    "prepending": _LIST_OP_PREPEND,
    "append": _LIST_OP_APPEND,
    "appended": _LIST_OP_APPEND,
    "appending": _LIST_OP_APPEND,
}


def _normalize_list_op_type(list_op_type: str | None) -> str:
    """Normalize the list op type string to a supported value.

    Args:
        list_op_type: Raw list op type value from configuration.

    Returns:
        Normalized list op type string ("prepend" or "append").

    """
    if not list_op_type:
        return _LIST_OP_PREPEND
    normalized = list_op_type.strip().lower()
    return _LIST_OP_ALIASES.get(normalized, _LIST_OP_PREPEND)


def _matches_any(name: str, patterns: list[str]) -> bool:
    """Check if a name fully matches any regex pattern.

    Args:
        name: Name to check.
        patterns: List of regex pattern strings.

    Returns:
        True if name matches any pattern, False otherwise.

    """
    compiled = utils.compile_patterns(patterns)
    return utils.matches_any_pattern(name, compiled)


def _convert_list_op(
    list_op: object,
    list_op_type: str,
) -> tuple[object | None, list[object]]:
    """Convert an explicit list op to a prepended or appended list op.

    Args:
        list_op: The list op object to convert.
        list_op_type: Target list op type ("prepend" or "append").

    Returns:
        Tuple of (new list op or None, explicit items list).

    """
    if not list_op or not hasattr(list_op, "isExplicit") or not getattr(list_op, "isExplicit"):
        return None, []
    if hasattr(list_op, "GetAddedOrExplicitItems"):
        explicit_items = list(list_op.GetAddedOrExplicitItems() or [])
    else:
        explicit_items = list(getattr(list_op, "explicitItems", []) or [])
    if not explicit_items:
        return None, []
    if isinstance(list_op, Sdf.TokenListOp):
        if list_op_type == _LIST_OP_APPEND:
            new_list_op = Sdf.TokenListOp.Create(appendedItems=explicit_items)
        else:
            new_list_op = Sdf.TokenListOp.Create(prependedItems=explicit_items)
    elif isinstance(list_op, Sdf.PathListOp) or all(isinstance(item, Sdf.Path) for item in explicit_items):
        if list_op_type == _LIST_OP_APPEND:
            new_list_op = Sdf.PathListOp.Create(appendedItems=explicit_items)
        else:
            new_list_op = Sdf.PathListOp.Create(prependedItems=explicit_items)
    else:
        return None, []
    return new_list_op, explicit_items


def _apply_list_op_items(list_op_proxy: object, explicit_items: list[object], list_op_type: str) -> bool:
    """Apply list op items to a list op proxy as non-explicit edits.

    Args:
        list_op_proxy: The list op proxy to update (e.g., targetPathList).
        explicit_items: Explicit items to re-author as edits.
        list_op_type: Target list op type ("prepend" or "append").

    Returns:
        True if items were applied, False otherwise.

    """
    if not list_op_proxy or not explicit_items:
        return False
    clear_edits = getattr(list_op_proxy, "ClearEdits", None)
    if clear_edits is None:
        return False

    clear_edits()
    if list_op_type == _LIST_OP_APPEND and hasattr(list_op_proxy, "Append"):
        for item in explicit_items:
            list_op_proxy.Append(item)
    elif hasattr(list_op_proxy, "Prepend"):
        for item in reversed(explicit_items):
            list_op_proxy.Prepend(item)
    else:
        return False
    return True


def _recreate_relationship_with_targets(
    prim_spec: Sdf.PrimSpec,
    rel_name: str,
    explicit_items: list[Sdf.Path],
    list_op_type: str,
) -> Sdf.RelationshipSpec | None:
    """Recreate a relationship spec with non-explicit target edits.

    Args:
        prim_spec: Prim spec that owns the relationship.
        rel_name: Relationship name to recreate.
        explicit_items: Target paths to author as list edits.
        list_op_type: Target list op type ("prepend" or "append").

    Returns:
        The recreated relationship spec or None if creation failed.

    """
    if rel_name in prim_spec.relationships:
        old_rel = prim_spec.relationships[rel_name]
        info_to_restore = {}
        for key in old_rel.ListInfoKeys():
            if key == "targetPaths":
                continue
            try:
                info_to_restore[key] = old_rel.GetInfo(key)
            except Exception:
                pass
        prim_spec.RemoveProperty(old_rel)
    else:
        info_to_restore = {}

    new_rel = Sdf.RelationshipSpec(prim_spec, rel_name)
    if not new_rel:
        return None
    for key, value in info_to_restore.items():
        if value is not None:
            try:
                new_rel.SetInfo(key, value)
            except Exception:
                pass

    if list_op_type == _LIST_OP_APPEND:
        for item in explicit_items:
            new_rel.targetPathList.Append(item)
    else:
        for item in reversed(explicit_items):
            new_rel.targetPathList.Prepend(item)
    return new_rel


class MakeListsNonExplicitRule(RuleInterface):
    """Convert explicit list ops on prim metadata and properties to non-explicit list ops.

    Scans prim specs in the stage and converts explicit list ops to prepended or
    appended list ops for matching metadata names and property names. This is
    useful for converting authored explicit lists into non-explicit list edits.
    """

    def get_configuration_parameters(self) -> list[RuleConfigurationParam]:
        """Return the configuration parameters for this rule.

        Returns:
            List of configuration parameters for metadata and property filters.

        Example:

        .. code-block:: python

            params = rule.get_configuration_parameters()

        """
        return [
            RuleConfigurationParam(
                name="metadata_names",
                display_name="Metadata Names",
                param_type=list,
                description="List of regex patterns for prim metadata names to convert (e.g., 'api.*')",
                default_value=None,
            ),
            RuleConfigurationParam(
                name="property_names",
                display_name="Property Names",
                param_type=list,
                description="List of regex patterns for prim property names to convert (e.g., 'material:.*')",
                default_value=None,
            ),
            RuleConfigurationParam(
                name="list_op_type",
                display_name="List Op Type",
                param_type=str,
                description="Target list op type: 'prepend' (default) or 'append'",
                default_value=_LIST_OP_PREPEND,
            ),
        ]

    def process_rule(self) -> str | None:
        """Convert explicit list ops to non-explicit list ops for matching names.

        Returns:
            None (this rule does not change the working stage).

        Example:

        .. code-block:: python

            rule.process_rule()

        """
        params = self.args.get("params", {}) or {}
        metadata_names = params.get("metadata_names") or []
        property_names = params.get("property_names") or []

        if not metadata_names and not property_names:
            self.log_operation("No metadata or property names specified, skipping")
            return None

        list_op_type = _normalize_list_op_type(params.get("list_op_type"))
        if list_op_type not in (_LIST_OP_PREPEND, _LIST_OP_APPEND):
            self.log_operation(f"Invalid list_op_type '{list_op_type}', using '{_LIST_OP_PREPEND}'")
            list_op_type = _LIST_OP_PREPEND

        self.log_operation(
            f"MakeListsNonExplicitRule start metadata={metadata_names} properties={property_names} "
            f"list_op_type={list_op_type}"
        )

        metadata_converted = 0
        property_converted = 0
        modified_layers: set[Sdf.Layer] = set()

        root_layer = self.source_stage.GetRootLayer()
        root_prim = self.source_stage.GetPseudoRoot()
        for prim in Usd.PrimRange(root_prim):
            if not prim or not prim.IsValid():
                continue

            # Metadata branch stays root-layer-only. `apiSchemas` and similar
            # composed-metadata list-ops must be authored on the root layer to
            # stick across references/payloads, so do not widen scope here.
            if metadata_names:
                metadata_spec = root_layer.GetPrimAtPath(prim.GetPath())
                if metadata_spec:
                    for key in metadata_spec.ListInfoKeys():
                        if not _matches_any(key, metadata_names):
                            continue
                        list_op = metadata_spec.GetInfo(key)
                        new_list_op, explicit_items = _convert_list_op(list_op, list_op_type)
                        if new_list_op is None:
                            continue
                        metadata_spec.SetInfo(key, new_list_op)
                        modified_layers.add(root_layer)
                        metadata_converted += 1
                        self.log_operation(
                            f"Converted metadata '{key}' on {metadata_spec.path} ({len(explicit_items)} item(s))"
                        )

            # Property branch walks every contributing `PrimSpec` returned by
            # `prim.GetPrimStack()` (root layer, sublayers, payloads, etc.) and
            # converts matching relationship / attribute-connection list-ops on
            # each. Each touched layer is added to `modified_layers` so the Save
            # loop at the end of `process_rule` persists all edits. This closes
            # the gap where relationships authored in sublayers (e.g. a payload
            # authoring `isaac:physics:robotJoints`) were never converted because
            # the earlier implementation inspected only the root-layer `PrimSpec`.
            if property_names:
                for prim_spec in prim.GetPrimStack():
                    if not prim_spec:
                        continue
                    spec_layer = prim_spec.layer
                    if spec_layer is None:
                        continue
                    for prop_name, prop_spec in prim_spec.properties.items():
                        if not _matches_any(prop_name, property_names):
                            continue

                        if isinstance(prop_spec, Sdf.RelationshipSpec):
                            list_op = prop_spec.targetPathList
                            new_list_op, explicit_items = _convert_list_op(list_op, list_op_type)
                            if not explicit_items or new_list_op is None:
                                continue
                            recreated_rel = _recreate_relationship_with_targets(
                                prim_spec,
                                prop_name,
                                list(explicit_items),
                                list_op_type,
                            )
                            if not recreated_rel:
                                continue
                            modified_layers.add(spec_layer)
                            property_converted += 1
                            self.log_operation(
                                f"Converted relationship '{prop_name}' on {prim_spec.path} "
                                f"in {spec_layer.identifier} ({len(explicit_items)} item(s))"
                            )
                        elif isinstance(prop_spec, Sdf.AttributeSpec):
                            list_op = prop_spec.connectionPathList
                            new_list_op, explicit_items = _convert_list_op(list_op, list_op_type)
                            if not explicit_items or new_list_op is None:
                                continue
                            if not _apply_list_op_items(
                                prop_spec.connectionPathList,
                                list(explicit_items),
                                list_op_type,
                            ):
                                continue
                            modified_layers.add(spec_layer)
                            property_converted += 1
                            self.log_operation(
                                f"Converted attribute connections '{prop_name}' on {prim_spec.path} "
                                f"in {spec_layer.identifier} ({len(explicit_items)} item(s))"
                            )

        for layer in modified_layers:
            layer.Save()
            self.add_affected_stage(layer.identifier)
            self.log_operation(f"Saved modified layer: {layer.identifier}")

        self.log_operation(
            f"MakeListsNonExplicitRule completed: metadata={metadata_converted}, properties={property_converted}"
        )

        return None
