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

"""Variant routing rule for extracting variant assets."""

from __future__ import annotations

import os

from isaacsim.asset.transformer import RuleConfigurationParam, RuleInterface
from pxr import Sdf, UsdUtils

from .. import utils
from ..utils import (
    COMPOSITION_SKIP_KEYS,
    asset_dirname,
    asset_exists,
    copy_attributes_to_prim_spec,
    copy_file_to_directory,
    copy_prim_metadata,
    copy_relationships_to_prim_spec,
    copy_stage_metadata,
    find_first_resolvable_arc,
    get_path_string,
    is_absolute_asset_path,
    is_builtin_mdl,
    join_asset_path,
    norm_path,
    remap_asset_path,
    resolve_asset_path,
    sanitize_prim_name,
)

# Default configuration
_DEFAULT_DESTINATION: str = "payloads"


class VariantRoutingRule(RuleInterface):
    """Route variant set contents to separate layer files.

    This rule extracts each variant from the default prim's variant sets into
    individual USDA files organized by variant set folder. For variants that contain
    payloads, references, or sublayers to other assets, the rule:

    1. Maps existing variant payloads/references to identify source assets
    2. Creates a new USDA file for each variant in the variant set folder
    3. Copies the source asset (preserving composition structure, not flattening)
    4. Applies remaining overrides from the source variant as strongest opinions
    5. Remaps inter-variant dependencies to point to new variant files
    6. Collects external dependencies (with ALL sub-dependencies) into a "dependencies" folder

    Configuration options:
        - case_insensitive (default True): Convert variant names to lowercase for
          output file names.
        - variant_sets: Optional list of variant set names to process.
        - collect_dependencies (default True): Collect external dependencies into
          a dependencies folder.
        - excluded_variants: Variant names to skip while still creating empty files.
    """

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
                name="variant_sets",
                display_name="Variant Sets",
                param_type=list,
                description="Optional list of variant set names to process. If empty, all variant sets are processed.",
                default_value=[],
            ),
            RuleConfigurationParam(
                name="case_insensitive",
                display_name="Case Insensitive",
                param_type=bool,
                description="If True, convert variant option names to lowercase for output files and references.",
                default_value=True,
            ),
            RuleConfigurationParam(
                name="collect_dependencies",
                display_name="Collect Dependencies",
                param_type=bool,
                description="If True, collect external dependencies into a dependencies folder.",
                default_value=True,
            ),
            RuleConfigurationParam(
                name="excluded_variants",
                display_name="Excluded Variants",
                param_type=list,
                description="List of variant names to exclude from processing. Excluded variants will have empty .usda files created but their contents will not be processed.",
                default_value=[],
            ),
        ]

    def _get_prepended_arcs(self, variant_spec: Sdf.VariantSpec) -> tuple[list[Sdf.Payload], list[Sdf.Reference]]:
        """Extract prepended payloads and references from a variant spec.

        Args:
            variant_spec: The variant spec to inspect.

        Returns:
            Tuple of (prepended_payloads, prepended_references).

        """
        prepended_payloads: list[Sdf.Payload] = []
        prepended_references: list[Sdf.Reference] = []

        prim_spec = variant_spec.primSpec
        if not prim_spec:
            return prepended_payloads, prepended_references

        # Check for prepended payloads
        if prim_spec.hasPayloads:
            payload_list = prim_spec.payloadList
            prepended_payloads = list(payload_list.prependedItems)

        # Check for prepended references
        if prim_spec.hasReferences:
            ref_list = prim_spec.referenceList
            prepended_references = list(ref_list.prependedItems)

        return prepended_payloads, prepended_references

    def _resolve_asset_path(self, arc_asset_path: str, base_layer: Sdf.Layer | None = None) -> str:
        """Resolve an asset path to absolute path.

        Args:
            arc_asset_path: The asset path from a composition arc.
            base_layer: Optional layer to resolve relative paths against.

        Returns:
            Resolved absolute path or empty string if not resolvable.

        """
        fallback_dirs = [
            asset_dirname(self.source_stage.GetRootLayer().realPath),
        ]
        input_stage_path = self.args.get("input_stage_path", "")
        if input_stage_path:
            fallback_dirs.append(asset_dirname(input_stage_path))

        return resolve_asset_path(arc_asset_path, base_layer, fallback_dirs)

    def _build_variant_asset_map(
        self,
        variant_set_spec: Sdf.VariantSetSpec,
        source_layer: Sdf.Layer,
    ) -> dict[str, str]:
        """Build a map of original variant asset paths to variant names.

        This identifies which assets are "variant files" that will be processed.

        Args:
            variant_set_spec: The variant set spec to analyze.
            source_layer: The source layer for resolving paths.

        Returns:
            Dict mapping normalized absolute paths to variant names.

        """
        asset_to_variant: dict[str, str] = {}

        for variant_name in variant_set_spec.variants.keys():  # noqa: SIM118
            variant_spec = variant_set_spec.variants[variant_name]
            payloads, references = self._get_prepended_arcs(variant_spec)

            for payload in payloads:
                if payload.assetPath:
                    resolved = self._resolve_asset_path(payload.assetPath)
                    if resolved:
                        asset_to_variant[norm_path(resolved)] = variant_name

            for ref in references:
                if ref.assetPath:
                    resolved = self._resolve_asset_path(ref.assetPath)
                    if resolved:
                        asset_to_variant[norm_path(resolved)] = variant_name

        return asset_to_variant

    def _collect_all_dependencies(
        self,
        source_layer_path: str,
        dependencies_dir: str,
        variant_file_map: dict[str, str],
    ) -> dict[str, str]:
        """Collect all dependencies of a layer recursively.

        Uses UsdUtils.ComputeAllDependencies to find all referenced assets
        (sublayers, references, payloads, textures, etc.) and copies them
        to the dependencies directory.

        Args:
            source_layer_path: Absolute path to the source layer.
            dependencies_dir: Directory to collect dependencies into.
            variant_file_map: Map of variant asset paths (to exclude from collection).

        Returns:
            Dict mapping original absolute paths to collected absolute paths.

        """
        collected: dict[str, str] = {}
        source_norm = norm_path(source_layer_path)

        try:
            layers, assets, unresolved = UsdUtils.ComputeAllDependencies(source_layer_path)
            self.log_operation(
                f"ComputeAllDependencies found {len(layers)} layers, {len(assets)} assets, {len(unresolved)} unresolved"
            )
        except Exception as e:
            self.log_operation(f"Failed to compute dependencies for {source_layer_path}: {e}")
            return collected

        def should_skip(normed: str) -> bool:
            """Check if a path should be skipped from collection.

            Args:
                normed: Normalized (case-folded) absolute path.

            Returns:
                True if the path should be skipped.

            """
            return normed == source_norm or normed in variant_file_map or normed in collected

        def collect_path(path_obj: object, is_layer: bool = False) -> None:
            """Collect a single dependency path.

            Args:
                path_obj: Dependency path object from USD dependency results.
                is_layer: True when the path represents a USD layer.

            """
            abs_path = get_path_string(path_obj)
            if not abs_path or not asset_exists(abs_path):
                if is_layer:
                    self.log_operation(f"Skipping non-existent layer: {abs_path}")
                return

            # Skip built-in MDL files (e.g. OmniPBR.mdl) -- they ship with
            # the runtime and must not be collected as dependencies.
            if is_builtin_mdl(abs_path):
                return

            normed = norm_path(abs_path)
            if should_skip(normed):
                if is_layer and normed in variant_file_map:
                    self.log_operation(f"Skipping variant file (will be remapped): {normed}")
                return

            dest_path = copy_file_to_directory(abs_path, dependencies_dir, collected)
            if dest_path:
                collected[normed] = dest_path
                log_type = "layer" if is_layer else "asset"
                self.log_operation(f"Collected {log_type} dependency: {os.path.basename(abs_path)}")

        # Collect USD layer dependencies
        for layer in layers:
            collect_path(layer, is_layer=True)

        # Collect non-USD asset dependencies (textures, meshes, etc.)
        for asset_path in assets:
            collect_path(asset_path, is_layer=False)

        self.log_operation(f"Total collected dependencies: {len(collected)}")
        return collected

    def _collect_variant_delta_dependencies(
        self,
        variant_spec: Sdf.VariantSpec,
        source_layer_dir: str,
        dependencies_dir: str,
        variant_file_map: dict[str, str],
        all_collected_deps: dict[str, str],
    ) -> None:
        """Collect dependencies from composition arcs within variant deltas.

        Walks the variant spec's prim tree and collects any referenced assets
        from references/payloads on child prims.

        Args:
            variant_spec: The variant spec to walk.
            source_layer_dir: Directory of the source layer (for resolving relative paths).
            dependencies_dir: Directory to collect dependencies into.
            variant_file_map: Map of variant file paths (to exclude).
            all_collected_deps: Shared dict to update with collected dependencies.

        """
        prim_spec = variant_spec.primSpec
        if not prim_spec:
            self.log_operation("No prim spec in variant for delta dependency collection")
            return

        def collect_prim_arcs(spec: Sdf.PrimSpec) -> None:
            """Recursively collect dependencies from a prim spec's composition arcs.

            Args:
                spec: Prim spec to traverse.

            """
            # Collect from references
            if spec.hasReferences:
                for ref in spec.referenceList.GetAddedOrExplicitItems():
                    if ref.assetPath:
                        self._collect_arc_dependency(
                            ref.assetPath, source_layer_dir, dependencies_dir, variant_file_map, all_collected_deps
                        )

            # Collect from payloads
            if spec.hasPayloads:
                for payload in spec.payloadList.GetAddedOrExplicitItems():
                    if payload.assetPath:
                        self._collect_arc_dependency(
                            payload.assetPath, source_layer_dir, dependencies_dir, variant_file_map, all_collected_deps
                        )

            # Recurse to children
            for child_name in spec.nameChildren.keys():  # noqa: SIM118
                collect_prim_arcs(spec.nameChildren[child_name])

            return

        collect_prim_arcs(prim_spec)

    def _collect_arc_dependency(
        self,
        asset_path: str,
        source_layer_dir: str,
        dependencies_dir: str,
        variant_file_map: dict[str, str],
        all_collected_deps: dict[str, str],
    ) -> None:
        """Collect a single composition arc's asset and all its dependencies.

        Args:
            asset_path: The asset path from a reference/payload arc.
            source_layer_dir: Directory of the source layer (for resolving
                relative paths). May be a local directory or a remote URL.
            dependencies_dir: Directory to collect dependencies into.
            variant_file_map: Map of variant file paths (to exclude).
            all_collected_deps: Shared dict to update with collected dependencies.

        """
        # Resolve to absolute path or remote URL. ``join_asset_path`` keeps
        # the ``scheme://`` of remote bases intact and resolves ``./`` /
        # ``../`` segments without invoking ``os.path.normpath`` (which
        # would collapse the ``//`` after the scheme).
        if is_absolute_asset_path(asset_path):
            abs_path = asset_path
        else:
            abs_path = join_asset_path(source_layer_dir, asset_path)

        self.log_operation(f"Resolving arc dependency: {asset_path} -> {abs_path}")

        # Try USD extension variations (.usd can resolve to .usda or .usdc).
        # ``asset_exists`` handles both local files and remote URLs via
        # omni.client.
        resolved_path = None
        if asset_exists(abs_path):
            resolved_path = abs_path
        else:
            base, ext = os.path.splitext(abs_path)
            if ext.lower() == ".usd":
                for alt_ext in [".usda", ".usdc"]:
                    alt_path = base + alt_ext
                    if asset_exists(alt_path):
                        resolved_path = alt_path
                        self.log_operation(f"Resolved with extension: {asset_path} -> {os.path.basename(alt_path)}")
                        break

        if not resolved_path:
            self.log_operation(f"Variant delta arc references non-existent file: {asset_path} (tried: {abs_path})")
            return

        normed = norm_path(resolved_path)

        if normed in all_collected_deps or normed in variant_file_map:
            return

        self.log_operation(f"Collecting variant delta dependency: {asset_path} -> {resolved_path}")

        deps = self._collect_all_dependencies(resolved_path, dependencies_dir, variant_file_map)
        all_collected_deps.update(deps)

        if normed not in all_collected_deps:
            dest_path = copy_file_to_directory(resolved_path, dependencies_dir, all_collected_deps)
            if dest_path:
                all_collected_deps[normed] = dest_path
                self.log_operation(f"Collected arc target: {os.path.basename(resolved_path)}")

    def _copy_layer_with_remapping(
        self,
        source_layer_path: str,
        dest_layer_path: str,
        variant_file_map: dict[str, str],
        collected_deps: dict[str, str],
    ) -> Sdf.Layer | None:
        """Copy a layer file, remapping all asset paths.

        Creates a copy of the source layer at the destination, updating all
        internal asset paths to point to:
        - New variant file paths (for inter-variant dependencies)
        - Collected dependency paths (for external assets)

        Args:
            source_layer_path: Absolute path to source layer.
            dest_layer_path: Absolute path for destination layer.
            variant_file_map: Map from original variant paths to new variant paths.
            collected_deps: Map from original dependency paths to collected paths.

        Returns:
            The created destination layer, or None on failure.

        """
        self.log_operation(f"Copying layer with remapping: {source_layer_path} -> {dest_layer_path}")
        self.log_operation(f"  variant_file_map has {len(variant_file_map)} entries")
        self.log_operation(f"  collected_deps has {len(collected_deps)} entries")

        source_layer = Sdf.Layer.FindOrOpen(source_layer_path)
        if not source_layer:
            self.log_operation(f"Failed to open source layer: {source_layer_path}")
            return None

        os.makedirs(os.path.dirname(dest_layer_path), exist_ok=True)

        if not source_layer.Export(dest_layer_path):
            self.log_operation(f"Failed to export layer to: {dest_layer_path}")
            return None

        dest_layer = Sdf.Layer.FindOrOpen(dest_layer_path)
        if not dest_layer:
            self.log_operation(f"Failed to open destination layer: {dest_layer_path}")
            return None

        source_dir = asset_dirname(source_layer_path)
        dest_dir = os.path.dirname(dest_layer_path)
        remapped_count = [0]

        def remap_fn(original_path: str) -> str:
            result = remap_asset_path(original_path, source_dir, dest_dir, variant_file_map, collected_deps)
            if result != original_path:
                remapped_count[0] += 1
            return result

        # Remap sublayer paths (not caught by ModifyAssetPaths)
        if dest_layer.subLayerPaths:
            dest_layer.subLayerPaths = [remap_fn(p) for p in dest_layer.subLayerPaths]

        UsdUtils.ModifyAssetPaths(dest_layer, remap_fn)

        self.log_operation(f"Remapped {remapped_count[0]} asset paths")
        dest_layer.Save()
        return dest_layer

    # Tolerance for identifying an Xformable opinion as "identity" before
    # treating it as a copy artifact safe to strip from the variant file.
    _XFORM_IDENTITY_EPS: float = 1e-6

    def _strip_identity_xformops_from_prim(
        self,
        prim_spec: Sdf.PrimSpec,
        preserve_attrs: set[str] | None = None,
    ) -> None:
        """Strip identity-only Xformable opinions from a single prim spec.

        Core of the variant-routing identity-strip logic, factored out so it
        can run on both the variant file's root prim (with the variant's
        authored attribute names as ``preserve_attrs``) and on each collected
        dependency's default prim (with ``preserve_attrs=None``).

        The strip only fires when *every* authored xformOp on the prim is
        identity. A non-identity op, a time-sampled value, a connection, or a
        ``!resetXformStack!`` directive in ``xformOpOrder`` leaves the entire
        stack untouched.

        Args:
            prim_spec: The prim spec to clean.
            preserve_attrs: Names of attributes that must be kept regardless
                of value (typically the names the variant author explicitly
                listed in their own delta). ``None`` means "preserve nothing
                special" — use this for dependency files where there is no
                variant context.

        """
        preserve = preserve_attrs or set()

        xform_attr_specs: list[Sdf.AttributeSpec] = []
        for attr_name in list(prim_spec.attributes.keys()):  # noqa: SIM118
            if attr_name in preserve:
                continue
            if attr_name.startswith("xformOp:") or attr_name == "xformOpOrder":
                xform_attr_specs.append(prim_spec.attributes[attr_name])

        if not xform_attr_specs:
            return

        for spec in xform_attr_specs:
            if not self._xform_attr_is_identity(spec):
                self.log_operation(f"Keeping non-identity xformOps on {prim_spec.path} (because of {spec.name})")
                return

        stripped_names = [spec.name for spec in xform_attr_specs]
        # ``del prim_spec.attributes[name]`` is a no-op on the Sdf proxy view;
        # ``RemoveProperty`` is the supported way to drop an attribute spec.
        for spec in xform_attr_specs:
            prim_spec.RemoveProperty(spec)
        self.log_operation(f"Stripped identity xformOps from {prim_spec.path}: {stripped_names}")

    def _strip_identity_root_xformops_in_collected_files(
        self,
        dependencies_dir: str,
    ) -> None:
        """Apply the identity-xformop strip to every collected USD dependency.

        When a variant payloads/references a collected dependency (e.g. the
        variant authors ``over "ee_link" (prepend payload = @dep.usd@)``),
        the dep file's default-prim xformOps are remapped onto the payload
        target prim. If those root xformOps are an identity artifact, they
        nonetheless out-rank the base layer's authoring on the same prim
        because variant-rooted arcs are stronger than the interface's
        top-level reference to the base. The result the user sees: switching
        from "no variant" (correct base pose) to a variant selection snaps
        the prim back to the dep's identity transform.

        The fix is to strip those identity opinions from each dep file's
        default prim. Non-identity poses are preserved exactly as before —
        they encode real authoring intent.

        Args:
            dependencies_dir: Per-variant-set dependencies directory to walk.

        """
        if not os.path.isdir(dependencies_dir):
            return

        usd_extensions = utils.USD_EXTENSIONS
        for filename in os.listdir(dependencies_dir):
            if os.path.splitext(filename)[1].lower() not in usd_extensions:
                continue
            filepath = os.path.join(dependencies_dir, filename)
            layer = Sdf.Layer.FindOrOpen(filepath)
            if not layer:
                continue
            default_prim_name = layer.defaultPrim
            if not default_prim_name:
                continue
            prim_spec = layer.GetPrimAtPath(Sdf.Path.absoluteRootPath.AppendChild(default_prim_name))
            if not prim_spec:
                continue
            before = list(prim_spec.attributes.keys())
            self._strip_identity_xformops_from_prim(prim_spec)
            after = list(prim_spec.attributes.keys())
            if before != after:
                layer.Save()

    def _strip_inherited_root_opinions(
        self,
        variant_spec: Sdf.VariantSpec,
        dest_prim_spec: Sdf.PrimSpec,
    ) -> None:
        """Strip identity-only Xformable root-prim attributes inherited from the source.

        A variant file is meant to overlay onto the consuming stage's existing
        default prim (it is referenced into ``/<default_prim>`` from the
        interface layer). Source assets routinely author a default pose
        (``xformOp:translate = (0, 0, 0)``, ``xformOp:scale = (1, 1, 1)``,
        ...) on their default prim as a structural artifact, and copying the
        source verbatim turns each of those identity opinions into a STRONG
        override against whatever pose the consumer's wrapping stage authors
        on the same prim. The consumer's prim snaps back to identity every
        time the variant is selected.

        To avoid that surprise while still preserving genuine variant intent
        the strip is bounded by two conditions, both required:

        * The attribute is Xformable (``xformOp:*`` or ``xformOpOrder``).
          Non-Xformable opinions are left alone — they may be load-bearing
          for downstream rules or consumer schemas.
        * Every authored Xformable opinion on the root encodes the identity
          transform (translates ~= 0, scales ~= 1, rotations ~= 0, orient is
          ``(w=1, x=y=z=0)``, ``xformOp:transform`` matches the identity
          matrix). A non-identity authored opinion is taken as intentional
          and the entire xform stack is preserved as-is.

        Anything the variant explicitly authored in its own ``primSpec``
        deltas is preserved regardless of value — the variant author asked
        for that opinion. Time-sampled and connected Xformable attributes
        are treated as non-identity (their effective value cannot be
        verified without examining every sample / resolving the connection).
        A ``!resetXformStack!`` directive in ``xformOpOrder`` also blocks
        the strip; it is a semantic operation, not a transform value.

        Children, composition arcs, variantSets, applied API schemas, and
        non-Xformable attributes are preserved — they carry the actual
        variant content.

        Args:
            variant_spec: Variant spec whose own authored deltas must be preserved.
            dest_prim_spec: Destination root prim spec to strip in place.

        """
        variant_prim_spec = variant_spec.primSpec
        delta_attr_names: set[str] = (
            set(variant_prim_spec.attributes.keys()) if variant_prim_spec is not None else set()
        )
        self._strip_identity_xformops_from_prim(dest_prim_spec, delta_attr_names)

    def _xform_attr_is_identity(self, attr_spec: Sdf.AttributeSpec) -> bool:
        """Return True when an Xformable attribute spec is safe to strip.

        Treats unauthored defaults, empty ``xformOpOrder`` lists, and
        identity-valued ops as safe. Refuses to classify time-sampled or
        connected attributes as identity, since the effective value depends
        on data not visible in the default. ``xformOpOrder`` entries
        containing ``!resetXformStack!`` are also refused: that directive
        resets the parent stack and is semantically meaningful even when no
        ops follow.

        Args:
            attr_spec: Xformable attribute spec to classify.

        Returns:
            True when the attribute is safe to strip (identity or unauthored).

        """
        if attr_spec.HasInfo("timeSamples"):
            return False
        if attr_spec.connectionPathList.GetAddedOrExplicitItems():
            return False

        name = attr_spec.name

        if name == "xformOpOrder":
            if not attr_spec.HasInfo("default"):
                return True
            order = attr_spec.default
            if not order:
                return True
            return all(str(entry) != "!resetXformStack!" for entry in order)

        if not attr_spec.HasInfo("default"):
            return True

        return self._xform_value_is_identity(name, attr_spec.default)

    @staticmethod
    def _vector_components(value: object) -> list[float] | None:
        """Return *value* as a list of float components, or ``None`` if scalar.

        Handles ``Gf.Vec*`` types, which expose ``__len__`` / ``__getitem__``
        but no ``__iter__`` method — ``hasattr(value, "__iter__")`` is False
        for those types, so callers cannot rely on it to detect vectors.

        Args:
            value: Authored attribute value to decompose.

        Returns:
            List of float components, or ``None`` when *value* is scalar or
            cannot be decomposed.

        """
        if isinstance(value, (int, float, bool)):
            return None
        if hasattr(value, "__len__") and hasattr(value, "__getitem__"):
            try:
                return [float(value[i]) for i in range(len(value))]
            except Exception:
                return None
        try:
            return [float(c) for c in value]  # generic iterable
        except TypeError:
            return None

    def _xform_value_is_identity(self, attr_name: str, value: object) -> bool:
        """Compare an authored Xformable value against its identity.

        Identity is op-type-specific: translate / rotation -> zero; scale ->
        one; orient quaternion -> ``(w=1, x=0, y=0, z=0)``; transform matrix
        -> 4x4 identity. Unknown op names are conservatively reported as
        non-identity so we never strip something we can't classify.

        Args:
            attr_name: Full Xformable attribute name (e.g. ``xformOp:translate``).
            value: Authored value to compare against the op's identity.

        Returns:
            True when *value* encodes the identity transform for the op.

        """
        if value is None:
            return True

        parts = attr_name.split(":")
        op_name = parts[1] if len(parts) >= 2 else ""
        eps = self._XFORM_IDENTITY_EPS

        try:
            if op_name == "translate":
                components = self._vector_components(value)
                if components is None:
                    return False
                return all(abs(c) < eps for c in components)

            if op_name.startswith("rotate"):
                components = self._vector_components(value)
                if components is None:
                    return abs(float(value)) < eps
                return all(abs(c) < eps for c in components)

            if op_name == "scale":
                components = self._vector_components(value)
                if components is None:
                    return False
                return all(abs(c - 1.0) < eps for c in components)

            if op_name == "orient":
                if hasattr(value, "GetReal") and hasattr(value, "GetImaginary"):
                    if abs(float(value.GetReal()) - 1.0) >= eps:
                        return False
                    imag_components = self._vector_components(value.GetImaginary())
                    if imag_components is None:
                        return False
                    return all(abs(c) < eps for c in imag_components)
                return False

            if op_name == "transform":
                for i in range(4):
                    row = value[i]
                    for j in range(4):
                        expected = 1.0 if i == j else 0.0
                        if abs(float(row[j]) - expected) > eps:
                            return False
                return True
        except Exception:
            return False

        return False

    def _apply_variant_deltas(
        self,
        variant_spec: Sdf.VariantSpec,
        dest_layer: Sdf.Layer,
        dest_prim_path: str,
        source_layer_dir: str,
        variant_file_map: dict[str, str],
        collected_deps: dict[str, str],
    ) -> None:
        """Apply the variant's own deltas on top of existing content.

        This applies any direct overrides from the variant spec (attributes,
        relationships, child prims) as the strongest opinion. Composition arcs
        are remapped to point to collected dependencies.

        Args:
            variant_spec: The variant spec with deltas to apply.
            dest_layer: The destination layer.
            dest_prim_path: The destination prim path.
            source_layer_dir: Directory of the original source layer (for path resolution).
            variant_file_map: Map from original variant paths to new variant paths.
            collected_deps: Map from original dependency paths to collected paths.

        """
        prim_spec = variant_spec.primSpec
        if not prim_spec:
            self.log_operation("No prim spec in variant, skipping deltas")
            return

        self.log_operation(f"Applying variant deltas to {dest_prim_path}")

        dest_prim_spec = dest_layer.GetPrimAtPath(dest_prim_path)
        if not dest_prim_spec:
            dest_prim_spec = Sdf.CreatePrimInLayer(dest_layer, dest_prim_path)
            if not dest_prim_spec:
                self.log_operation(f"Failed to create destination prim at {dest_prim_path}")
                return
            dest_prim_spec.specifier = Sdf.SpecifierDef

        dest_layer_dir = os.path.dirname(dest_layer.realPath)

        self.log_operation(
            f"  Variant deltas: {len(prim_spec.attributes)} attributes, "
            f"{len(prim_spec.relationships)} relationships, "
            f"{len(prim_spec.nameChildren)} child prims"
        )

        # Update type if authored
        if prim_spec.typeName:
            dest_prim_spec.typeName = prim_spec.typeName

        # Merge applied API schemas
        if prim_spec.HasInfo("apiSchemas"):
            src_schemas = prim_spec.GetInfo("apiSchemas")
            if dest_prim_spec.HasInfo("apiSchemas"):
                existing = set(dest_prim_spec.GetInfo("apiSchemas").GetAddedOrExplicitItems())
                new_items = set(src_schemas.GetAddedOrExplicitItems())
                merged = list(existing.union(new_items))
                dest_prim_spec.SetInfo("apiSchemas", Sdf.TokenListOp.CreateExplicit(merged))
            else:
                dest_prim_spec.SetInfo("apiSchemas", src_schemas)

        # Copy metadata (excluding composition-related)
        copy_prim_metadata(prim_spec, dest_prim_spec, COMPOSITION_SKIP_KEYS | {"apiSchemas"})

        # Apply attribute and relationship overrides
        copy_attributes_to_prim_spec(prim_spec, dest_prim_spec)
        copy_relationships_to_prim_spec(prim_spec, dest_prim_spec)

        # Recursively apply child deltas
        for child_name in prim_spec.nameChildren.keys():  # noqa: SIM118
            child_spec = prim_spec.nameChildren[child_name]
            child_dest_path = f"{dest_prim_path}/{child_name}"
            self._apply_child_prim_deltas(
                child_spec,
                dest_layer,
                child_dest_path,
                source_layer_dir,
                dest_layer_dir,
                variant_file_map,
                collected_deps,
            )

    def _apply_child_prim_deltas(
        self,
        src_spec: Sdf.PrimSpec,
        dest_layer: Sdf.Layer,
        dest_path: str,
        source_layer_dir: str,
        dest_layer_dir: str,
        variant_file_map: dict[str, str],
        collected_deps: dict[str, str],
    ) -> None:
        """Apply a child prim's deltas including composition arcs with path remapping.

        Args:
            src_spec: The source prim spec.
            dest_layer: The destination layer.
            dest_path: The destination prim path.
            source_layer_dir: Directory of the original source layer (for path resolution).
            dest_layer_dir: Directory of the destination layer.
            variant_file_map: Map from original variant paths to new variant paths.
            collected_deps: Map from original dependency paths to collected paths.

        """
        dest_prim_spec = dest_layer.GetPrimAtPath(dest_path)
        if not dest_prim_spec:
            dest_prim_spec = Sdf.CreatePrimInLayer(dest_layer, dest_path)
            if not dest_prim_spec:
                return
            dest_prim_spec.specifier = src_spec.specifier

        # Update specifier if source is stronger
        if src_spec.specifier == Sdf.SpecifierDef:
            dest_prim_spec.specifier = src_spec.specifier

        if src_spec.typeName:
            dest_prim_spec.typeName = src_spec.typeName

        # Copy metadata, attributes, and relationships using utilities
        copy_prim_metadata(src_spec, dest_prim_spec)
        copy_attributes_to_prim_spec(src_spec, dest_prim_spec)
        copy_relationships_to_prim_spec(src_spec, dest_prim_spec)

        # Helper for remapping composition arc paths
        def remap_fn(path: str) -> str:
            return remap_asset_path(path, source_layer_dir, dest_layer_dir, variant_file_map, collected_deps)

        def make_remapped_ref(ref: Sdf.Reference) -> Sdf.Reference:
            new_path = remap_fn(ref.assetPath) if ref.assetPath else ref.assetPath
            if new_path != ref.assetPath and ref.assetPath:
                self.log_operation(f"  Remapped child reference at {dest_path}: {ref.assetPath} -> {new_path}")
            return Sdf.Reference(new_path, ref.primPath, ref.layerOffset)

        def make_remapped_payload(payload: Sdf.Payload) -> Sdf.Payload:
            new_path = remap_fn(payload.assetPath) if payload.assetPath else payload.assetPath
            if new_path != payload.assetPath and payload.assetPath:
                self.log_operation(f"  Remapped child payload at {dest_path}: {payload.assetPath} -> {new_path}")
            return Sdf.Payload(new_path, payload.primPath, payload.layerOffset)

        # Copy composition arcs with path remapping (handle all list types)
        if src_spec.hasReferences:
            ref_list = src_spec.referenceList
            for ref in ref_list.prependedItems:
                dest_prim_spec.referenceList.Prepend(make_remapped_ref(ref))
            for ref in ref_list.appendedItems:
                dest_prim_spec.referenceList.Append(make_remapped_ref(ref))
            if ref_list.isExplicit:
                dest_prim_spec.referenceList.explicitItems = [make_remapped_ref(r) for r in ref_list.explicitItems]

        if src_spec.hasPayloads:
            payload_list = src_spec.payloadList
            for payload in payload_list.prependedItems:
                dest_prim_spec.payloadList.Prepend(make_remapped_payload(payload))
            for payload in payload_list.appendedItems:
                dest_prim_spec.payloadList.Append(make_remapped_payload(payload))
            if payload_list.isExplicit:
                dest_prim_spec.payloadList.explicitItems = [
                    make_remapped_payload(p) for p in payload_list.explicitItems
                ]

        # Recurse to children
        for child_name in src_spec.nameChildren.keys():  # noqa: SIM118
            child_spec = src_spec.nameChildren[child_name]
            self._apply_child_prim_deltas(
                child_spec,
                dest_layer,
                f"{dest_path}/{child_name}",
                source_layer_dir,
                dest_layer_dir,
                variant_file_map,
                collected_deps,
            )

    def _get_variant_file_path(
        self,
        variant_name: str,
        variant_set_output_dir: str,
        case_insensitive: bool = True,
    ) -> str:
        """Get the output file path for a variant.

        Args:
            variant_name: Name of the variant.
            variant_set_output_dir: Output directory for this variant set.
            case_insensitive: If True, convert variant name to lowercase for output.

        Returns:
            Path to the variant layer file.

        """
        sanitized_name = sanitize_prim_name(variant_name)
        if case_insensitive:
            sanitized_name = sanitized_name.lower()
        return os.path.join(variant_set_output_dir, f"{sanitized_name}.usda")

    def _create_variant_layer(
        self,
        variant_file_path: str,
        default_prim_path: str,
    ) -> tuple[Sdf.Layer, str] | None:
        """Create a new variant layer with standard setup.

        Args:
            variant_file_path: Path for the new layer file.
            default_prim_path: Path to the default prim.

        Returns:
            Tuple of (layer, dest_prim_path) or None if creation failed.

        """
        os.makedirs(os.path.dirname(variant_file_path), exist_ok=True)
        variant_layer = Sdf.Layer.CreateNew(variant_file_path)
        if not variant_layer:
            self.log_operation(f"Failed to create variant layer: {variant_file_path}")
            return None

        copy_stage_metadata(self.source_stage, variant_layer)
        default_prim_name = Sdf.Path(default_prim_path).name
        variant_layer.defaultPrim = default_prim_name
        return variant_layer, f"/{default_prim_name}"

    def _create_empty_variant_file(
        self,
        variant_name: str,
        variant_set_output_dir: str,
        default_prim_path: str,
        case_insensitive: bool = True,
    ) -> str | None:
        """Create an empty variant file for excluded variants.

        Creates a minimal USDA file with just the default prim defined but no content.
        This allows the variant to exist in the variant set without processing its contents.

        Args:
            variant_name: Name of the variant.
            variant_set_output_dir: Output directory for this variant set.
            default_prim_path: Path to the default prim.
            case_insensitive: If True, convert variant name to lowercase for output.

        Returns:
            Path to the created variant layer file, or None if creation failed.

        """
        variant_file_path = self._get_variant_file_path(variant_name, variant_set_output_dir, case_insensitive)
        self.log_operation(f"Creating empty variant file for excluded variant '{variant_name}' -> {variant_file_path}")

        result = self._create_variant_layer(variant_file_path, default_prim_path)
        if not result:
            return None

        variant_layer, dest_prim_path = result
        prim_spec = Sdf.CreatePrimInLayer(variant_layer, dest_prim_path)
        if prim_spec:
            prim_spec.specifier = Sdf.SpecifierDef
            self.log_operation(f"Created empty prim spec at '{dest_prim_path}' for excluded variant")
        else:
            self.log_operation(f"Warning: Failed to create prim spec at '{dest_prim_path}' for excluded variant")

        variant_layer.Save()
        return variant_file_path

    def _process_variant(
        self,
        variant_set_spec: Sdf.VariantSetSpec,
        variant_name: str,
        variant_set_output_dir: str,
        source_layer: Sdf.Layer,
        default_prim_path: str,
        variant_file_map: dict[str, str],
        all_collected_deps: dict[str, str],
        case_insensitive: bool = True,
        collect_dependencies: bool = True,
        excluded_variants: set[str] | None = None,
    ) -> str | None:
        """Process a single variant and write it to a USDA file.

        For variants that have payloads or references to assets, this method:
        1. Copies the variant's source asset file (preserving composition)
        2. Collects all dependencies of that asset
        3. Remaps paths to point to new variant files or collected dependencies
        4. Applies the variant's own deltas as the strongest opinion

        For excluded variants, creates an empty USDA file without processing contents.

        Args:
            variant_set_spec: The variant set spec containing the variant.
            variant_name: Name of the variant to process.
            variant_set_output_dir: Output directory for this variant set.
            source_layer: The source layer containing the variant.
            default_prim_path: Path to the default prim.
            variant_file_map: Map from original variant asset paths to new paths.
            all_collected_deps: Shared dict tracking all collected dependencies.
            case_insensitive: If True, convert variant name to lowercase for output.
            collect_dependencies: If True, collect external dependencies.
            excluded_variants: Set of variant names to exclude from full processing.

        Returns:
            Path to the created variant layer file, or None if processing failed.

        """
        # The interface rule adds an empty "none" option even when no such option
        # existed in the source. On a second transform, do not turn that
        # synthetic option into a new payload file. An authored None variant
        # with properties, children, arcs, or metadata still follows the usual
        # excluded-variant behavior and gets its empty placeholder file.
        variant_name_for_check = variant_name.lower() if case_insensitive else variant_name
        if excluded_variants and variant_name_for_check in excluded_variants:
            variant_spec = variant_set_spec.variants.get(variant_name)
            prim_spec = variant_spec.primSpec if variant_spec else None
            if (
                variant_name_for_check == "none"
                and prim_spec
                and prim_spec.specifier == Sdf.SpecifierOver
                and set(prim_spec.ListInfoKeys()) == {"specifier"}
                and not prim_spec.properties
                and not prim_spec.nameChildren
            ):
                self.log_operation(f"Skipping synthetic empty variant '{variant_name}'")
                return None
            self.log_operation(f"Variant '{variant_name}' is excluded, creating empty variant file")
            return self._create_empty_variant_file(
                variant_name, variant_set_output_dir, default_prim_path, case_insensitive
            )

        variant_spec = variant_set_spec.variants.get(variant_name)
        if not variant_spec:
            self.log_operation(f"Variant '{variant_name}' not found in variant set")
            return None

        variant_file_path = self._get_variant_file_path(variant_name, variant_set_output_dir, case_insensitive)
        self.log_operation(f"Processing variant '{variant_name}' -> {variant_file_path}")

        prepended_payloads, prepended_references = self._get_prepended_arcs(variant_spec)
        self.log_operation(
            f"Found {len(prepended_payloads)} prepended payloads, {len(prepended_references)} prepended references"
        )

        # Find the primary source asset for this variant
        primary_source_path = find_first_resolvable_arc(
            prepended_payloads, prepended_references, self._resolve_asset_path
        )

        # Get the source layer directory for resolving relative paths
        # Use the ORIGINAL input stage path (not the working stage) because
        # composition arcs are relative to the original source location
        original_input_path = self.args.get("input_stage_path", "")
        if original_input_path:
            source_layer_dir = asset_dirname(original_input_path)
        else:
            # Fall back to working stage path if original not available
            source_layer_path = source_layer.realPath or source_layer.identifier
            source_layer_dir = asset_dirname(source_layer_path) if source_layer_path else ""
        self.log_operation(f"Original input path: {original_input_path}, source dir: {source_layer_dir}")
        dependencies_dir = os.path.join(variant_set_output_dir, "dependencies") if collect_dependencies else ""

        if not primary_source_path:
            # No external asset - create minimal layer with deltas only
            self.log_operation(f"No external asset for variant '{variant_name}', creating minimal layer")

            # Collect dependencies from variant delta composition arcs
            if collect_dependencies and dependencies_dir:
                self._collect_variant_delta_dependencies(
                    variant_spec, source_layer_dir, dependencies_dir, variant_file_map, all_collected_deps
                )

            result = self._create_variant_layer(variant_file_path, default_prim_path)
            if not result:
                return None
            variant_layer, dest_prim_path = result
            self._apply_variant_deltas(
                variant_spec, variant_layer, dest_prim_path, source_layer_dir, variant_file_map, all_collected_deps
            )
            # Strip stale composition arcs (see note in the primary-source path below)
            dest_prim_spec = variant_layer.GetPrimAtPath(dest_prim_path)
            if dest_prim_spec:
                if dest_prim_spec.hasPayloads:
                    dest_prim_spec.payloadList.ClearEdits()
                if dest_prim_spec.hasReferences:
                    dest_prim_spec.referenceList.ClearEdits()
            variant_layer.Save()
            return variant_file_path

        self.log_operation(f"Primary source asset: {primary_source_path}")

        # Collect dependencies from primary source
        if collect_dependencies and dependencies_dir:
            deps = self._collect_all_dependencies(primary_source_path, dependencies_dir, variant_file_map)
            all_collected_deps.update(deps)
            self.log_operation(f"Collected {len(deps)} dependencies from primary source for variant '{variant_name}'")

            # Also collect dependencies from variant delta composition arcs
            self._collect_variant_delta_dependencies(
                variant_spec, source_layer_dir, dependencies_dir, variant_file_map, all_collected_deps
            )

        # Copy source layer with remapping
        variant_layer = self._copy_layer_with_remapping(
            primary_source_path, variant_file_path, variant_file_map, all_collected_deps
        )
        if not variant_layer:
            return None

        copy_stage_metadata(self.source_stage, variant_layer)
        default_prim_name = Sdf.Path(default_prim_path).name
        dest_prim_path = f"/{default_prim_name}"

        # Snapshot payload/reference state from the source copy before applying deltas
        dest_prim_spec = variant_layer.GetPrimAtPath(dest_prim_path)
        had_payloads = bool(dest_prim_spec and dest_prim_spec.hasPayloads)
        had_references = bool(dest_prim_spec and dest_prim_spec.hasReferences)

        # Apply variant deltas as strongest opinion
        self._apply_variant_deltas(
            variant_spec, variant_layer, dest_prim_path, source_layer_dir, variant_file_map, all_collected_deps
        )
        self.log_operation("Applied variant deltas as strongest opinion")

        # If applying deltas introduced payload/reference arcs that were not in
        # the source file, strip them.  These are the composition arcs from the
        # variant spec that route INTO this content — they must not remain on
        # the output prim or they create invalid / self-referencing arcs.
        dest_prim_spec = variant_layer.GetPrimAtPath(dest_prim_path)
        if dest_prim_spec:
            if dest_prim_spec.hasPayloads and not had_payloads:
                dest_prim_spec.payloadList.ClearEdits()
                self.log_operation(f"Cleared stale payload arc(s) from {dest_prim_path}")
            if dest_prim_spec.hasReferences and not had_references:
                dest_prim_spec.referenceList.ClearEdits()
                self.log_operation(f"Cleared stale reference arc(s) from {dest_prim_path}")

            # Strip root-prim opinions inherited from the copied source asset.
            # The variant file is meant to overlay the consumer's default prim;
            # carrying the source's root transforms or apiSchemas creates
            # strong opinions that conflict with upstream authoring whenever
            # this variant is selected.
            self._strip_inherited_root_opinions(variant_spec, dest_prim_spec)

        variant_layer.defaultPrim = default_prim_name
        variant_layer.Save()
        self.log_operation(f"Created variant layer: {variant_file_path}")
        return variant_file_path

    def _remap_collected_dependencies(
        self,
        dependencies_dir: str,
        variant_file_map: dict[str, str],
        all_collected_deps: dict[str, str],
    ) -> None:
        """Remap paths within collected dependency files.

        After all dependencies are collected, update any internal references
        within those files to point to the collected locations.

        The key insight is that paths INSIDE collected files are relative to
        their ORIGINAL location, not the new dependencies folder. We need to
        resolve paths against the original location, then remap to new paths.

        Args:
            dependencies_dir: Directory containing collected dependencies.
            variant_file_map: Map of variant file paths.
            all_collected_deps: Map from original absolute paths to new absolute paths.

        """
        if not os.path.isdir(dependencies_dir):
            return

        # Build reverse map: new_path -> original_path
        # This lets us find the original location of each copied file
        new_to_original: dict[str, str] = {}
        for orig_path, new_path in all_collected_deps.items():
            norm_new = norm_path(new_path)
            new_to_original[norm_new] = orig_path

        usd_extensions = utils.USD_EXTENSIONS
        files_processed = 0
        paths_remapped = 0

        self.log_operation(f"Remapping paths in collected dependencies: {dependencies_dir}")

        for filename in os.listdir(dependencies_dir):
            filepath = os.path.join(dependencies_dir, filename)
            ext = os.path.splitext(filename)[1]

            if ext.lower() not in usd_extensions:
                continue

            layer = Sdf.Layer.FindOrOpen(filepath)
            if not layer:
                self.log_operation(f"  Failed to open dependency layer: {filename}")
                continue

            # Find the ORIGINAL location of this file
            norm_filepath = norm_path(filepath)
            original_file_path = new_to_original.get(norm_filepath)

            if original_file_path:
                # Resolve paths relative to the ORIGINAL file location.
                # ``asset_dirname`` preserves remote URL scheme so deps
                # downloaded from omniverse:// re-resolve correctly.
                original_dir = asset_dirname(original_file_path)
            else:
                # Fallback to new location if not in map
                original_dir = os.path.dirname(filepath)
                self.log_operation(f"  Warning: No original path found for {filename}, using new location")

            new_dir = os.path.dirname(filepath)
            file_remap_count = [0]

            def remap_fn(original_path: str) -> str:
                # Resolve path relative to ORIGINAL location
                result = remap_asset_path(original_path, original_dir, new_dir, variant_file_map, all_collected_deps)
                if result != original_path:
                    file_remap_count[0] += 1
                return result

            if layer.subLayerPaths:
                layer.subLayerPaths = [remap_fn(p) for p in layer.subLayerPaths]

            UsdUtils.ModifyAssetPaths(layer, remap_fn)
            layer.Save()

            files_processed += 1
            paths_remapped += file_remap_count[0]
            if file_remap_count[0] > 0:
                self.log_operation(f"  Remapped {file_remap_count[0]} paths in {filename}")

        self.log_operation(
            f"Completed dependency remapping: {files_processed} files processed, {paths_remapped} total paths remapped"
        )

    def process_rule(self) -> str | None:
        """Process variant sets and extract each variant to separate layer files.

        For each variant set on the default prim, creates a folder and extracts
        each variant to a USDA file. Source assets are copied (not flattened),
        with paths remapped to point to new variant files or collected dependencies.
        Variant deltas are applied as the strongest opinion.

        Returns:
            None (this rule does not change the working stage).

        Example:

        .. code-block:: python

            rule.process_rule()

        """
        params = self.args.get("params", {}) or {}
        variant_sets_filter: list[str] = params.get("variant_sets") or []
        case_insensitive: bool = params.get("case_insensitive", True)
        collect_dependencies: bool = params.get("collect_dependencies", True)
        excluded_variants_list: list[str] = params.get("excluded_variants") or []
        # Normalize excluded variants to lowercase if case_insensitive is enabled
        excluded_variants: set[str]
        if case_insensitive:
            excluded_variants = {v.lower() for v in excluded_variants_list}
        else:
            excluded_variants = set(excluded_variants_list)
        destination = self.destination_path

        self.log_operation(
            f"VariantRoutingRule start destination={destination}, "
            f"case_insensitive={case_insensitive}, collect_dependencies={collect_dependencies}, "
            f"excluded_variants={excluded_variants}"
        )

        # Get the default prim
        default_prim = self.source_stage.GetDefaultPrim()
        if not default_prim or not default_prim.IsValid():
            self.log_operation("No valid default prim found, skipping")
            return None

        default_prim_path = default_prim.GetPath().pathString
        self.log_operation(f"Processing default prim: {default_prim_path}")

        # Get variant sets from the default prim
        variant_sets = default_prim.GetVariantSets()
        variant_set_names = variant_sets.GetNames() if variant_sets else []

        if not variant_set_names:
            self.log_operation("No variant sets found on default prim")
            return None

        self.log_operation(f"Found variant sets: {variant_set_names}")

        # Filter variant sets if specified
        if variant_sets_filter:
            variant_set_names = [name for name in variant_set_names if name in variant_sets_filter]
            self.log_operation(f"Filtered to variant sets: {variant_set_names}")

        # Get the source layer
        source_layer = self.source_stage.GetRootLayer()

        # Process each variant set
        output_base = os.path.join(self.package_root, destination)

        # Track all outputs and collected dependencies
        all_variant_outputs: list[str] = []
        all_collected_deps: dict[str, str] = {}

        for vs_name in variant_set_names:
            variant_set = variant_sets.GetVariantSet(vs_name)
            if not variant_set:
                continue

            # Create output directory for this variant set
            sanitized_vs_name = sanitize_prim_name(vs_name)
            variant_set_output_dir = os.path.join(output_base, sanitized_vs_name)
            os.makedirs(variant_set_output_dir, exist_ok=True)

            self.log_operation(f"Processing variant set '{vs_name}' -> {variant_set_output_dir}")

            # Get all variant names
            variant_names = variant_set.GetVariantNames()

            # Get the variant set spec
            prim_spec = source_layer.GetPrimAtPath(default_prim_path)
            if not prim_spec:
                self.log_operation(f"Could not find prim spec for {default_prim_path}")
                continue

            variant_set_spec = prim_spec.variantSets.get(vs_name)
            if not variant_set_spec:
                self.log_operation(f"Could not find variant set spec for {vs_name}")
                continue

            # Build map of original variant assets to new variant file paths
            variant_asset_map = self._build_variant_asset_map(variant_set_spec, source_layer)
            variant_file_map: dict[str, str] = {
                original_path: self._get_variant_file_path(v_name, variant_set_output_dir, case_insensitive)
                for original_path, v_name in variant_asset_map.items()
            }

            self.log_operation(f"Variant file mapping: {variant_file_map}")

            # Dependencies directory for this variant set
            dependencies_dir = os.path.join(variant_set_output_dir, "dependencies")

            # Process each variant
            for variant_name in variant_names:
                output_path = self._process_variant(
                    variant_set_spec,
                    variant_name,
                    variant_set_output_dir,
                    source_layer,
                    default_prim_path,
                    variant_file_map,
                    all_collected_deps,
                    case_insensitive=case_insensitive,
                    collect_dependencies=collect_dependencies,
                    excluded_variants=excluded_variants,
                )

                if output_path:
                    all_variant_outputs.append(output_path)
                    self.add_affected_stage(output_path)

            # Remap paths within collected dependencies for this variant set
            if collect_dependencies:
                self._remap_collected_dependencies(dependencies_dir, variant_file_map, all_collected_deps)
                # Strip identity-only root xformOps from each collected dep
                # file. When the variant payloads/references a dep, the dep's
                # default-prim transforms become the payload target's
                # strongest opinion; leaving an identity transform there
                # snaps the prim back on every variant switch.
                self._strip_identity_root_xformops_in_collected_files(dependencies_dir)

        # Log summary
        excluded_count = len(excluded_variants) if excluded_variants else 0
        self.log_operation(
            f"VariantRoutingRule completed: {len(all_variant_outputs)} variant files created, "
            f"{len(all_collected_deps)} dependencies collected, "
            f"{excluded_count} variants excluded from processing"
        )

        return None
