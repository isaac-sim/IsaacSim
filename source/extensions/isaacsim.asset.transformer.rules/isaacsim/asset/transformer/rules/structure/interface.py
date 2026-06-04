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

"""Interface composition rule for USD asset transformation."""

from __future__ import annotations

import os
from typing import Any

from isaacsim.asset.transformer import RuleConfigurationParam, RuleInterface
from pxr import Sdf

from .. import utils

# Connection types for composition arcs
CONNECTION_REFERENCE: str = "Reference"
CONNECTION_PAYLOAD: str = "Payload"
CONNECTION_SUBLAYER: str = "Sublayer"
CONNECTION_INHERIT: str = "Inherit"
VALID_CONNECTION_TYPES: tuple[str, ...] = (
    CONNECTION_REFERENCE,
    CONNECTION_PAYLOAD,
    CONNECTION_SUBLAYER,
    CONNECTION_INHERIT,
)

# Default configuration values
_DEFAULT_BASE_LAYER: str = "payloads/base.usda"
_DEFAULT_CONNECTION_TYPE: str = CONNECTION_REFERENCE
_DEFAULT_GENERATE_FOLDER_VARIANTS: bool = False
_DEFAULT_PAYLOADS_FOLDER: str = "payloads"

# Variant set option for no variant selected
_NONE_VARIANT_NAME: str = "none"


class InterfaceConnectionRule(RuleInterface):
    """Generate interface layer with composition arcs to organize USD assets.

    Creates the final interface layer that references or payloads the base asset
    and optionally generates variant sets from folder structure organization.
    The interface layer is named after the original asset and placed at the
    package root.
    """

    def get_configuration_parameters(self) -> list[RuleConfigurationParam]:
        """Return the configuration parameters for this rule.

        Returns:
            List of configuration parameters for base connection type, folder
            variants generation, custom connections, and payloads folder.

        Example:

        .. code-block:: python

            params = rule.get_configuration_parameters()

        """
        return [
            RuleConfigurationParam(
                name="base_layer",
                display_name="Base Layer",
                param_type=str,
                description="Relative path to the base USD layer to connect",
                default_value=_DEFAULT_BASE_LAYER,
            ),
            RuleConfigurationParam(
                name="base_connection_type",
                display_name="Base Connection Type",
                param_type=str,
                description="How to connect base.usda to interface: Reference (default), Payload, or Sublayer",
                default_value=_DEFAULT_CONNECTION_TYPE,
            ),
            RuleConfigurationParam(
                name="generate_folder_variants",
                display_name="Generate Folder Variants",
                param_type=bool,
                description="Generate variant sets from payloads folder structure. Folder name = variant set, asset name = variant",
                default_value=_DEFAULT_GENERATE_FOLDER_VARIANTS,
            ),
            RuleConfigurationParam(
                name="payloads_folder",
                display_name="Payloads Folder",
                param_type=str,
                description="Folder to scan for variant set organization",
                default_value=_DEFAULT_PAYLOADS_FOLDER,
            ),
            RuleConfigurationParam(
                name="connections",
                display_name="Custom Connections",
                param_type=list,
                description="List of connection specs: [{asset_path, target_path, connection_type}]. "
                "Adds target_path to asset_path. If asset_path is omitted, adds to the interface layer. "
                "Sublayer adds as sublayer; Reference/Payload/Inherit add to default prim.",
                default_value=[],
            ),
            RuleConfigurationParam(
                name="default_variant_selections",
                display_name="Default Variant Selections",
                param_type=dict,
                description="Dictionary mapping variant set names to default variant selection names. "
                "If a variant set is not specified, defaults to 'None'.",
                default_value={},
            ),
        ]

    def process_rule(self) -> str | None:
        """Generate interface layer with composition structure.

        Creates an interface layer at the package root with the same name as the
        original asset. Connects the base layer using the specified connection type
        and optionally generates variant sets from folder organization.

        Returns:
            None (this rule does not change the working stage).

        Example:

        .. code-block:: python

            rule.process_rule()

        """
        params = self.args.get("params", {}) or {}

        # Extract configuration
        base_layer_path = params.get("base_layer") or _DEFAULT_BASE_LAYER
        base_connection_type = params.get("base_connection_type") or _DEFAULT_CONNECTION_TYPE
        generate_folder_variants = params.get("generate_folder_variants", _DEFAULT_GENERATE_FOLDER_VARIANTS)
        payloads_folder = params.get("payloads_folder") or _DEFAULT_PAYLOADS_FOLDER
        custom_connections = params.get("connections") or []
        default_variant_selections = params.get("default_variant_selections") or {}

        # Get interface asset name from args (passed by manager)
        interface_asset_name = self.args.get("interface_asset_name")
        if not interface_asset_name:
            # Derive from original input stage path (not the working stage which is base.usda)
            input_stage_path = self.args.get("input_stage_path", "")
            if input_stage_path:
                base_name = os.path.splitext(os.path.basename(input_stage_path))[0]
                interface_asset_name = base_name + ".usda"
            else:
                interface_asset_name = "asset.usda"

        self.log_operation(
            f"InterfaceConnectionRule start interface={interface_asset_name} "
            f"base_connection={base_connection_type} folder_variants={generate_folder_variants}"
        )

        # Validate connection type
        if base_connection_type not in VALID_CONNECTION_TYPES:
            self.log_operation(f"Invalid connection type '{base_connection_type}', using Reference")
            base_connection_type = CONNECTION_REFERENCE

        # Check if any referenced files exist before creating interface layer
        base_layer_abs_path = os.path.join(self.package_root, base_layer_path)
        base_layer_exists = os.path.exists(base_layer_abs_path)

        # Check if payloads folder has valid variant subfolders
        payloads_abs_path = os.path.join(self.package_root, payloads_folder)
        has_variant_folders = False
        if generate_folder_variants and os.path.isdir(payloads_abs_path):
            has_variant_folders = self._has_valid_variant_folders(payloads_abs_path)

        # Check if any custom connections reference existing files
        has_valid_connections = False
        if custom_connections:
            for spec in custom_connections:
                if isinstance(spec, dict):
                    asset_path = spec.get("asset_path")
                    if asset_path:
                        asset_abs = os.path.join(self.package_root, asset_path)
                        if os.path.exists(asset_abs):
                            has_valid_connections = True
                            break

        if not base_layer_exists and not has_variant_folders and not has_valid_connections:
            self.log_operation("No referenced files exist, skipping interface layer creation")
            return None

        # Create interface layer path at package root
        interface_layer_path = os.path.join(self.package_root, interface_asset_name)
        interface_layer = utils.find_or_create_layer(interface_layer_path, self.source_stage)
        if not interface_layer:
            self.log_operation(f"Failed to create interface layer at {interface_layer_path}")
            return None
        self.log_operation("Created/opened interface layer with stage metadata")

        # Get or determine default prim path
        default_prim_path = utils.get_default_prim_path(self.source_stage)

        # Create the default prim in interface layer
        self._ensure_default_prim(interface_layer, default_prim_path)

        # Connect base layer to interface
        if base_layer_exists:
            base_rel_path = utils.get_relative_layer_path(interface_layer, base_layer_abs_path)
            self._add_connection(
                interface_layer,
                default_prim_path,
                base_rel_path,
                base_connection_type,
            )
            self.log_operation(f"Connected base layer via {base_connection_type}: {base_rel_path}")
        else:
            self.log_operation(f"Base layer not found: {base_layer_abs_path}")

        # Generate variant sets from folder structure
        if generate_folder_variants:
            self._generate_folder_variants(
                interface_layer, default_prim_path, payloads_folder, default_variant_selections
            )

        # Apply custom connections
        if custom_connections:
            self._apply_custom_connections(interface_layer, custom_connections)

        # Move extraneous root-level prims from the base layer into the
        # interface layer so they survive the reference round-trip.
        if base_layer_exists:
            self._recover_extraneous_root_prims(interface_layer, base_layer_abs_path)

        # Save the interface layer
        interface_layer.Save()
        self.add_affected_stage(interface_layer_path)
        self.log_operation(f"InterfaceConnectionRule completed: {interface_layer_path}")

        return interface_layer_path

    def _ensure_default_prim(self, layer: Sdf.Layer, prim_path: str) -> Sdf.PrimSpec:
        """Ensure the default prim exists in the layer.

        Args:
            layer: The layer to create the prim in.
            prim_path: The prim path to create.

        Returns:
            The prim spec for the default prim.

        """
        prim_spec = layer.GetPrimAtPath(prim_path)
        if not prim_spec:
            prim_spec = utils.create_prim_spec(
                layer,
                prim_path,
                specifier=Sdf.SpecifierDef,
                type_name="Xform",
            )
        # Set as default prim
        prim_name = Sdf.Path(prim_path).name
        layer.defaultPrim = prim_name
        return prim_spec

    def _add_connection(
        self,
        layer: Sdf.Layer,
        prim_path: str,
        asset_path: str,
        connection_type: str,
        prepend: bool = True,
    ) -> None:
        """Add a composition arc to a prim.

        Args:
            layer: The layer containing the prim.
            prim_path: Path to the prim to add the arc to.
            asset_path: Relative path to the asset layer.
            connection_type: One of Reference, Payload, or Sublayer.
            prepend: If True, prepend the arc; otherwise append.

        """
        if connection_type == CONNECTION_SUBLAYER:
            # Sublayers are on the layer, not the prim
            sublayers = list(layer.subLayerPaths)
            if asset_path not in sublayers:
                if prepend:
                    sublayers.insert(0, asset_path)
                else:
                    sublayers.append(asset_path)
                layer.subLayerPaths = sublayers
            return

        prim_spec = layer.GetPrimAtPath(prim_path)
        if not prim_spec:
            utils.ensure_prim_hierarchy(layer, prim_path)
            prim_spec = utils.create_prim_spec(layer, prim_path)

        if not prim_spec:
            return

        if connection_type == CONNECTION_REFERENCE:
            ref = Sdf.Reference(asset_path)
            if prepend:
                prim_spec.referenceList.Prepend(ref)
            else:
                prim_spec.referenceList.Append(ref)
        elif connection_type == CONNECTION_PAYLOAD:
            payload = Sdf.Payload(asset_path)
            if prepend:
                prim_spec.payloadList.Prepend(payload)
            else:
                prim_spec.payloadList.Append(payload)

    def _generate_folder_variants(
        self,
        interface_layer: Sdf.Layer,
        default_prim_path: str,
        payloads_folder: str,
        default_variant_selections: dict[str, str],
    ) -> None:
        """Generate variant sets from folder structure in payloads directory.

        Each subfolder in the payloads directory becomes a variant set.
        Each USD file in that subfolder becomes a variant option.
        A "None" variant is always included with no payload.

        Args:
            interface_layer: The interface layer to add variants to.
            default_prim_path: Path to the default prim.
            payloads_folder: Relative path to the payloads folder.
            default_variant_selections: Mapping of variant set names to default variant selection.

        """
        payloads_abs_path = os.path.join(self.package_root, payloads_folder)
        if not os.path.isdir(payloads_abs_path):
            self.log_operation(f"Payloads folder not found: {payloads_abs_path}")
            return

        prim_spec = interface_layer.GetPrimAtPath(default_prim_path)
        if not prim_spec:
            return

        # Scan subdirectories for variant sets
        for entry in os.listdir(payloads_abs_path):
            subfolder_path = os.path.join(payloads_abs_path, entry)
            if not os.path.isdir(subfolder_path) or subfolder_path == payloads_abs_path:
                continue

            # Skip folder if it contains any non-USD file
            if entry.startswith("."):
                continue
            subfolder_files = os.listdir(subfolder_path)
            allowed_usd_exts = utils.USD_EXTENSIONS
            if any(
                os.path.isfile(os.path.join(subfolder_path, fname))
                and os.path.splitext(fname)[1].lower() not in allowed_usd_exts
                for fname in subfolder_files
            ):
                continue

            # Collect USD files in this subfolder
            variant_assets = self._collect_usd_files(subfolder_path)
            if not variant_assets:
                continue

            # Folder name becomes variant set name
            variant_set_name = utils.sanitize_prim_name(entry, prefix="VariantSet_")

            # Create variant set on the prim
            variant_set_spec = prim_spec.variantSets.get(variant_set_name)
            if not variant_set_spec:
                variant_set_spec = Sdf.VariantSetSpec(prim_spec, variant_set_name)
                # Append variant set name to the prim's variantSetNames metadata
                prim_spec.variantSetNameList.Append(variant_set_name)

            # Add "None" variant first
            none_variant = variant_set_spec.variants.get(_NONE_VARIANT_NAME)
            if not none_variant:
                none_variant = Sdf.VariantSpec(variant_set_spec, _NONE_VARIANT_NAME)

            # Add variants for each USD file
            for asset_file in variant_assets:
                asset_name = os.path.splitext(asset_file)[0]
                variant_name = utils.sanitize_prim_name(asset_name, prefix="Variant_")

                variant_spec = variant_set_spec.variants.get(variant_name)
                if not variant_spec:
                    variant_spec = Sdf.VariantSpec(variant_set_spec, variant_name)

                # Get prim spec within variant
                variant_prim_spec = variant_spec.primSpec
                if not variant_prim_spec:
                    continue

                # Compute relative path from interface layer to variant asset
                asset_abs_path = os.path.join(subfolder_path, asset_file)
                asset_rel_path = utils.get_relative_layer_path(interface_layer, asset_abs_path)

                # Add payload to the variant (prepended)
                payload = Sdf.Payload(asset_rel_path)
                variant_prim_spec.payloadList.Prepend(payload)

            # Set default variant selection from configuration or fall back to "None"
            default_selection = default_variant_selections.get(variant_set_name, _NONE_VARIANT_NAME)
            prim_spec.variantSelections[variant_set_name] = default_selection

            self.log_operation(f"Created variant set '{variant_set_name}' with {len(variant_assets)} variants + None")

    def _collect_usd_files(self, folder_path: str) -> list[str]:
        """Collect USD files from a folder.

        Args:
            folder_path: Absolute path to the folder to scan.

        Returns:
            List of USD filenames (not full paths).

        """
        usd_extensions = utils.USD_EXTENSIONS
        files = []
        for entry in os.listdir(folder_path):
            if os.path.isfile(os.path.join(folder_path, entry)):
                _, ext = os.path.splitext(entry)
                if ext.lower() in usd_extensions:
                    files.append(entry)
        return sorted(files)

    def _has_valid_variant_folders(self, payloads_abs_path: str) -> bool:
        """Check if payloads folder contains valid variant subfolders with USD files.

        Args:
            payloads_abs_path: Absolute path to the payloads folder.

        Returns:
            True if at least one valid variant subfolder exists.

        """
        allowed_usd_exts = {".usd", ".usda", ".usdc", ".usdz"}
        for entry in os.listdir(payloads_abs_path):
            subfolder_path = os.path.join(payloads_abs_path, entry)
            if not os.path.isdir(subfolder_path) or entry.startswith("."):
                continue

            # Skip folder if it contains any non-USD file
            subfolder_files = os.listdir(subfolder_path)
            has_non_usd = any(
                os.path.isfile(os.path.join(subfolder_path, fname))
                and os.path.splitext(fname)[1].lower() not in allowed_usd_exts
                for fname in subfolder_files
            )
            if has_non_usd:
                continue

            # Check if folder has any USD files
            variant_assets = self._collect_usd_files(subfolder_path)
            if variant_assets:
                return True

        return False

    def _apply_custom_connections(
        self,
        interface_layer: Sdf.Layer,
        connections: list[dict[str, Any]],
    ) -> None:
        """Apply custom connection specifications.

        The connections list contains connection specifications:
        [
            {
                "asset_path": "payloads/Physics/physics.usda",
                "target_path": "payloads/Physics/physx.usda",
                "connection_type": "Sublayer"
            }
        ]

        The target_path is added to the asset_path layer using the specified connection type.
        If asset_path is omitted or empty, the connection is added to the interface layer.
        For Sublayer, adds target_path as a sublayer.
        For Reference/Payload/Inherit, adds target_path to the default prim.

        Args:
            interface_layer: The interface layer to use when asset_path is not specified.
            connections: List of connection specifications.

        """
        for spec in connections:
            if not isinstance(spec, dict):
                self.log_operation(f"Invalid connection spec (not a dict): {spec}, skipping")
                continue

            asset_path = spec.get("asset_path")
            target_path = spec.get("target_path")
            connection_type = spec.get("connection_type", CONNECTION_PAYLOAD)

            if not target_path:
                self.log_operation(f"Missing target_path in connection spec: {spec}, skipping")
                continue

            # Validate connection type
            if connection_type not in VALID_CONNECTION_TYPES:
                self.log_operation(f"Invalid connection type '{connection_type}', skipping")
                continue

            # Check if target path exists
            target_abs_path = (
                os.path.join(self.package_root, target_path) if not os.path.isabs(target_path) else target_path
            )

            if not os.path.exists(target_abs_path):
                self.log_operation(f"Target path not found: {target_path}, skipping connection")
                continue

            # Determine which layer to modify
            if asset_path:
                # Use the specified asset layer
                asset_layer_abs = os.path.join(self.package_root, asset_path)
                if not os.path.exists(asset_layer_abs):
                    self.log_operation(f"Asset layer not found: {asset_path}, skipping connection")
                    continue

                asset_layer = Sdf.Layer.FindOrOpen(asset_layer_abs)
                if not asset_layer:
                    self.log_operation(f"Failed to open asset layer: {asset_path}")
                    continue

                layer_needs_save = True
                layer_display_name = asset_path
            else:
                # Use the interface layer directly
                asset_layer = interface_layer
                asset_layer_abs = interface_layer.realPath
                layer_needs_save = False  # Interface layer is saved by the caller
                layer_display_name = "interface"

            # Get default prim path
            default_prim_path = utils.get_default_prim_path(self.source_stage)

            # Ensure default prim exists in asset layer (for non-sublayer connections)
            if connection_type != CONNECTION_SUBLAYER:
                self._ensure_default_prim(asset_layer, default_prim_path)

            # Compute relative path from asset layer to target
            rel_path = utils.get_relative_layer_path(asset_layer, target_abs_path)

            self._add_connection(
                asset_layer,
                default_prim_path,
                rel_path,
                connection_type,
            )

            if layer_needs_save:
                asset_layer.Save()
                self.add_affected_stage(asset_layer_abs)

            self.log_operation(f"Added connection: {target_path} -> {layer_display_name} via {connection_type}")

    def _recover_extraneous_root_prims(
        self,
        interface_layer: Sdf.Layer,
        base_layer_path: str,
    ) -> None:
        """Move root-level prims outside ``defaultPrim`` from the base layer into the interface layer.

        Prims such as ``/Render`` or ``/PhysicsScene`` may exist at the root
        of the base layer after flattening.  Because the interface layer
        connects to the base layer via a *reference* on the default prim,
        root-level siblings are not composed and would be lost on a
        subsequent re-transform.  Copying them into the interface layer at the
        same root level and removing them from the base layer keeps them
        reachable and makes the pipeline idempotent.

        Args:
            interface_layer: The interface layer to copy prims into.
            base_layer_path: Absolute path to the base layer on disk.

        """
        base_layer = Sdf.Layer.FindOrOpen(base_layer_path)
        if not base_layer or not base_layer.pseudoRoot:
            return

        default_prim_name = base_layer.defaultPrim
        if not default_prim_name:
            return

        extra_names = [child.name for child in base_layer.pseudoRoot.nameChildren if child.name != default_prim_name]
        if not extra_names:
            return

        for name in extra_names:
            src_path = Sdf.Path.absoluteRootPath.AppendChild(name)
            Sdf.CopySpec(base_layer, src_path, interface_layer, src_path)
            del base_layer.pseudoRoot.nameChildren[name]

        base_layer.Save()
        self.log_operation(
            f"Recovered {len(extra_names)} root-level prim(s) from base layer " f"into interface: {extra_names}"
        )
