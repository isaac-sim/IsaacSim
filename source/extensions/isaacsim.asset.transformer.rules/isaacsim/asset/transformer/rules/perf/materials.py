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

"""Material routing rule for shared material layers."""

from __future__ import annotations

import hashlib
import os
import re
import shutil
from collections import defaultdict
from dataclasses import dataclass, field

from isaacsim.asset.transformer import RuleConfigurationParam, RuleInterface
from pxr import Sdf, Usd, UsdShade

from .. import utils

# Material binding purposes to check
_MATERIAL_PURPOSES: tuple[str, ...] = ("", "physics", "preview", "full")

# Default Configuration Parameters
_DEFAULT_SCOPE: str = "/"
_DEFAULT_MATERIALS_LAYER_PATH: str = "materials.usda"
_DEFAULT_TEXTURES_FOLDER: str = "textures"
_DEFAULT_MDL_FOLDER: str = "materials"
_DEFAULT_DEDUPLICATE: bool = True
_DEFAULT_DOWNLOAD_TEXTURES: bool = False

# Prim paths and scope names
_MATERIALS_SCOPE_PATH: str = "/Materials"

# Prim type names
_SCOPE_TYPE_NAME: str = "Scope"
_MATERIAL_TYPE_NAME: str = "Material"

# Truncation length for material content hashes
_MATERIAL_HASH_LENGTH: int = 32

# File extensions that should be transferred (textures, images, etc.)
_TEXTURE_ASSET_EXTENSIONS: frozenset[str] = frozenset(
    (
        ".png",
        ".jpg",
        ".jpeg",
        ".tga",
        ".bmp",
        ".gif",
        ".tiff",
        ".tif",
        ".exr",
        ".hdr",
        ".dds",
        ".ktx",
        ".ktx2",
        ".webp",
    )
)
_TRANSFERABLE_ASSET_EXTENSIONS: frozenset[str] = _TEXTURE_ASSET_EXTENSIONS.union({".mdl"})

_MDL_TEXTURE_PATH_PATTERN = re.compile(
    r'"([^"]+\.(?:' + "|".join(sorted(ext.lstrip(".") for ext in _TEXTURE_ASSET_EXTENSIONS)) + r'))"',
    re.IGNORECASE,
)

# Relative MDL module imports, e.g. ``import OmniPBR_ClearCoat::*;`` or
# ``using OmniPBRBase import *;``. These name a sibling ``<Module>.mdl`` file
# co-located with the importing module. Absolute/standard imports start with
# ``::`` (e.g. ``import ::df::*;``) and are intentionally excluded because they
# resolve to runtime-provided modules, not to co-located files on disk.
_MDL_MODULE_IMPORT_PATTERN = re.compile(r"\b(?:import|using)\s+([A-Za-z_]\w*)\s*(?:::|\bimport\b)")

# UDIM tile token as it appears in authored asset paths, matching both the
# canonical USD form (``<UDIM>``) and the URL-encoded form (``%3CUDIM%3E``)
# commonly found in Nucleus asset paths. A UDIM template never resolves to a
# single file: it stands in for a family of concrete tiles (``.1001.``,
# ``.1002.``, ...), each of which must be transferred individually.
_UDIM_TOKEN_PATTERN = re.compile(r"%3CUDIM%3E|<UDIM>", re.IGNORECASE)

# UDIM tile indices are always four digits (1001 and up).
_UDIM_TILE_REGEX: str = r"\d{4}"

# Canonical UDIM token written into transferred (local) asset paths so the USD
# resolver expands the family against the downloaded tiles.
_UDIM_CANONICAL_TOKEN: str = "<UDIM>"


@dataclass
class MaterialSource:
    """Tracks the source information for a material prim.

    Args:
        prim_path: Path of the source prim.
        defining_layer_id: Identifier of the defining layer.
        defining_prim_path: Prim path within the defining layer.
        material_hash: Hash of the material content.

    """

    prim_path: str
    defining_layer_id: str  # Layer where the material is defined
    defining_prim_path: str  # Prim path within the defining layer
    material_hash: str = ""


@dataclass
class MaterialEntry:
    """Represents a unique material stored in the materials layer.

    Args:
        name: Unique material name.
        material_layer_path: Prim path in the materials layer.
        sources: Source prims that contributed to this material.
        existing: Whether the material already existed in the layer.
        asset_paths: Asset paths used by the material.

    """

    name: str
    material_layer_path: str  # e.g., "/Materials/Material_001"
    sources: list[MaterialSource] = field(default_factory=list)
    existing: bool = False  # True if this material was already in the layer
    asset_paths: list[str] = field(default_factory=list)  # List of asset paths used by the material


@dataclass
class MaterialBinding:
    """Tracks a material binding on a prim.

    Args:
        prim_path: Path of the bound prim.
        material_path: Path to the material prim.
        purpose: Binding purpose name.

    """

    prim_path: str
    material_path: str
    purpose: str  # "" for allPurpose, or specific purpose like "physics", "preview", "full"


class MaterialsRoutingRule(RuleInterface):
    """Route material prims to a shared layer and create instanceable references at original locations.

    This rule identifies all material prims, tracks their sources including referenced materials,
    deduplicates identical materials based on their shader properties and attributes, moves their
    definitions to a dedicated materials layer under /Materials/{name}, creates instanceable
    references at original locations, and transfers texture/MDL assets to a designated folder.
    Materials that are not bound to any surface after processing are removed.
    """

    def get_configuration_parameters(self) -> list[RuleConfigurationParam]:
        """Return the configuration parameters for this rule.

        Returns:
            List of configuration parameters for scope, materials layer path,
            textures folder, and deduplication settings.

        Example:

        .. code-block:: python

            params = rule.get_configuration_parameters()

        """
        return [
            RuleConfigurationParam(
                name="scope",
                display_name="Scope",
                param_type=str,
                description="The scope to limit the search for material prims",
                default_value=_DEFAULT_SCOPE,
            ),
            RuleConfigurationParam(
                name="materials_layer",
                display_name="Materials Layer",
                param_type=str,
                description="The path to the materials layer",
                default_value=_DEFAULT_MATERIALS_LAYER_PATH,
            ),
            RuleConfigurationParam(
                name="textures_folder",
                display_name="Textures Folder",
                param_type=str,
                description="Folder name for texture assets at the root of the destination path",
                default_value=_DEFAULT_TEXTURES_FOLDER,
            ),
            RuleConfigurationParam(
                name="mdl_folder",
                display_name="MDL Folder",
                param_type=str,
                description=(
                    "Folder name for bundled MDL modules at the root of the destination path. "
                    "Kept separate from textures so an extracted material module is not shipped "
                    "inside the textures folder."
                ),
                default_value=_DEFAULT_MDL_FOLDER,
            ),
            RuleConfigurationParam(
                name="deduplicate",
                display_name="Deduplicate",
                param_type=bool,
                description="If True, deduplicate identical materials based on shader properties",
                default_value=_DEFAULT_DEDUPLICATE,
            ),
            RuleConfigurationParam(
                name="download_textures",
                display_name="Download Textures",
                param_type=bool,
                description="If True, download remote textures (e.g., from Nucleus) to local textures folder",
                default_value=_DEFAULT_DOWNLOAD_TEXTURES,
            ),
        ]

    def process_rule(self) -> str | None:
        """Move materials to a shared layer and create instanceable references.

        Order of operations:
        1. Find all material prims and their defining layers.
        2. Resolve file paths for supporting assets FIRST (including MDL references).
        3. Compute material hashes using resolved asset paths (before transferring).
        4. Transfer all supporting assets to the designated folder (globally deduplicated).
        5. Update MDL files to point to transferred textures.
        6. Create materials in materials layer with updated relative asset paths.
        7. Update materials in their defining layers with instanceable references.
        8. Update material bindings to point to deduplicated materials.
        9. Ensure MaterialBindingAPI is applied to all prims with material bindings.
        10. Remove material references that are not bound to any surface.

        Returns:
            None (this rule does not change the working stage).

        Example:

        .. code-block:: python

            rule.process_rule()

        """
        params = self.args.get("params", {}) or {}
        scope = params.get("scope") or "/"
        materials_layer_path = os.path.join(
            self.destination_path, params.get("materials_layer") or _DEFAULT_MATERIALS_LAYER_PATH
        )
        configured_textures_folder = params.get("textures_folder") or _DEFAULT_TEXTURES_FOLDER
        textures_folder = self._resolve_existing_folder_casing(self.package_root, configured_textures_folder)
        if textures_folder != configured_textures_folder:
            self.log_operation(f"Reusing existing textures folder casing: {textures_folder}")
        mdl_folder = params.get("mdl_folder") or _DEFAULT_MDL_FOLDER
        deduplicate = params.get("deduplicate", True)
        download_textures = params.get("download_textures", False)

        self.log_operation(
            f"MaterialsRoutingRule start scope={scope} deduplicate={deduplicate} "
            f"materials_layer={materials_layer_path} textures_folder={textures_folder} "
            f"mdl_folder={mdl_folder} download_textures={download_textures}"
        )

        # Resolve output paths relative to package root
        materials_output_path = os.path.join(self.package_root, materials_layer_path)
        textures_output_path = os.path.join(self.package_root, textures_folder)
        materials_layer_dir = os.path.dirname(materials_output_path)
        # Bundle MDL modules under the materials-layer directory (not the package
        # root) so the authored ``info:mdl:sourceAsset`` stays a ``./``-relative,
        # resolvable reference (VM.MDL.001) instead of a parent-relative ``../``
        # path when the materials layer lives in a subfolder such as ``payloads/``.
        mdl_output_path = os.path.join(materials_layer_dir, mdl_folder)

        # Ensure output directories exist
        os.makedirs(materials_layer_dir, exist_ok=True)
        os.makedirs(textures_output_path, exist_ok=True)

        # PHASE 1: Find all material prims and their sources (including defining layer info)
        material_sources = self._find_material_sources(scope)
        self.log_operation(f"Found {len(material_sources)} material prims")

        if not material_sources:
            self.log_operation("No material prims found, skipping")
            return None

        # PHASE 2: Collect all asset paths and compute hashes BEFORE transferring
        # This ensures we use resolved paths for proper deduplication
        source_asset_paths: dict[str, list[str]] = {}  # source.prim_path -> resolved asset paths
        source_hashes: dict[str, str] = {}  # source.prim_path -> hash
        all_mdl_paths: set[str] = set()

        for source in material_sources:
            prim = self.source_stage.GetPrimAtPath(source.prim_path)
            if not prim.IsValid():
                continue

            # Collect asset paths FIRST (using resolved paths)
            asset_paths, mdl_paths = self._collect_asset_paths(prim, include_remote=download_textures)
            source_asset_paths[source.prim_path] = asset_paths
            all_mdl_paths.update(mdl_paths)

            # Compute hash using resolved asset paths for proper deduplication
            mat_hash = self._compute_material_hash(prim, asset_paths)
            source.material_hash = mat_hash
            source_hashes[source.prim_path] = mat_hash

        # PHASE 3: Transfer all unique assets globally BEFORE creating materials
        # This deduplicates assets across all materials
        all_asset_paths: set[str] = set()
        for paths in source_asset_paths.values():
            all_asset_paths.update(paths)

        # Transfer assets and get mapping from original resolved path -> new relative path.
        # Bundled MDL modules are routed to their own folder so an extracted
        # material module is not shipped inside the textures folder; textures and
        # other supporting assets stay in the textures folder.
        mdl_asset_paths = {p for p in all_asset_paths if os.path.splitext(p)[1].lower() == ".mdl"}
        texture_asset_paths = all_asset_paths - mdl_asset_paths

        asset_path_mapping = self._transfer_all_assets(
            list(texture_asset_paths),
            textures_output_path,
            materials_layer_dir,
            download_remote=download_textures,
        )
        if mdl_asset_paths:
            os.makedirs(mdl_output_path, exist_ok=True)
            asset_path_mapping.update(
                self._transfer_all_assets(
                    list(mdl_asset_paths),
                    mdl_output_path,
                    materials_layer_dir,
                    download_remote=download_textures,
                )
            )
        self.log_operation(f"Transferred {len(asset_path_mapping)} unique assets")

        # Update MDL texture references after transfer
        if all_mdl_paths and asset_path_mapping:
            self._update_mdl_texture_paths(all_mdl_paths, asset_path_mapping, materials_layer_dir)

        # Open or create materials layer
        materials_layer = utils.find_or_create_layer(materials_output_path, self.source_stage)
        if not materials_layer:
            self.log_operation(f"Failed to create materials layer: {materials_output_path}")
            return None
        self.log_operation(f"Using materials layer: {materials_output_path}")
        mat_stage = Usd.Stage.Open(materials_layer)

        # Create /Materials scope in materials layer
        mat_stage.DefinePrim(_MATERIALS_SCOPE_PATH, _SCOPE_TYPE_NAME)

        # Group materials by their content hash for deduplication
        material_by_hash: dict[str, MaterialEntry] = {}
        material_name_counter: dict[str, int] = defaultdict(int)

        # If deduplication is enabled, scan existing materials in the layer first
        existing_count = 0
        if deduplicate:
            existing_materials = self._scan_existing_materials(mat_stage)
            for mat_hash, entry in existing_materials.items():
                material_by_hash[mat_hash] = entry
                material_name_counter[entry.name] += 1
            existing_count = len(existing_materials)
            if existing_count > 0:
                self.log_operation(f"Found {existing_count} existing materials in layer")

        new_count = 0
        reused_count = 0
        hash_occurrence_count: dict[str, int] = defaultdict(int)  # Track hash occurrences for duplicate reporting

        # PHASE 4: Group materials by hash and create entries in materials layer
        for source in material_sources:
            prim = self.source_stage.GetPrimAtPath(source.prim_path)
            if not prim.IsValid():
                continue

            source_hash = source_hashes.get(source.prim_path)
            if source_hash is None:
                continue

            # Track hash occurrences for duplicate reporting
            hash_occurrence_count[source_hash] += 1

            # When deduplication is disabled, use prim path as key to ensure uniqueness
            lookup_key = source_hash if deduplicate else source.prim_path

            if lookup_key in material_by_hash:
                # Material already exists (either from this run or existing in layer)
                material_by_hash[lookup_key].sources.append(source)
                if material_by_hash[lookup_key].existing:
                    reused_count += 1
            else:
                # New unique material, create entry
                base_name = utils.sanitize_prim_name(prim.GetName(), prefix="mat_")
                material_name_counter[base_name] += 1
                if material_name_counter[base_name] > 1:
                    unique_name = f"{base_name}_{material_name_counter[base_name] - 1}"
                else:
                    unique_name = base_name

                entry = MaterialEntry(
                    name=unique_name,
                    material_layer_path=f"{_MATERIALS_SCOPE_PATH}/{unique_name}",
                    sources=[source],
                    existing=False,
                )
                material_by_hash[lookup_key] = entry
                entry.asset_paths = source_asset_paths.get(source.prim_path, [])

                # Copy material to materials layer with updated asset paths
                self._copy_material_to_layer(prim, entry, mat_stage, asset_path_mapping, materials_layer_dir)
                new_count += 1

        # Calculate duplicates: materials that share the same hash
        unique_hashes = len(hash_occurrence_count)
        duplicate_count = len(material_sources) - unique_hashes

        if deduplicate:
            self.log_operation(
                f"Processed {len(material_sources)} materials: "
                f"{new_count} new, {reused_count} reused from existing layer, "
                f"{duplicate_count} deduplicated within batch"
            )
        else:
            self.log_operation(
                f"Processed {len(material_sources)} materials: "
                f"{new_count} new, {reused_count} reused from existing layer, "
                f"{duplicate_count} duplicates found but kept separate (deduplication disabled)"
            )

        # Export materials layer
        materials_layer.Export(materials_output_path)

        # Collect all material bindings in the stage
        all_bindings = self._collect_all_material_bindings(scope)
        self.log_operation(f"Found {len(all_bindings)} material bindings")

        # PHASE 5: Update materials in their defining layers with instanceable references
        self._update_defining_layers(material_by_hash, materials_output_path)

        # PHASE 6: Ensure MaterialBindingAPI is applied to all prims with material bindings
        self._ensure_material_binding_api(all_bindings)

        # PHASE 7: Check for unused materials and remove their instanceable references
        self._cleanup_unused_material_references(material_by_hash, all_bindings)

        self.add_affected_stage(materials_layer_path)

        total_unique = len(material_by_hash)
        new_in_layer = total_unique - existing_count
        self.log_operation(
            f"MaterialsRoutingRule completed: {total_unique} unique materials in layer "
            f"({new_in_layer} new, {existing_count} pre-existing), "
            f"{len(all_bindings)} bindings updated"
        )
        self.source_stage.Save()
        return None

    def _find_defining_layer(self, prim: Usd.Prim) -> tuple[str, str]:
        """Find the layer where a prim is defined (has SpecifierDef).

        Args:
            prim: The prim to find the defining layer for.

        Returns:
            Tuple of (layer_identifier, prim_path_in_layer).

        """
        prim_path = prim.GetPath().pathString
        prim_stack = prim.GetPrimStack()

        if prim_stack:
            # Find the layer where this prim is actually defined
            for spec in prim_stack:
                if spec.specifier == Sdf.SpecifierDef:
                    return spec.layer.identifier, spec.path.pathString

            # Fallback to strongest opinion layer
            return prim_stack[0].layer.identifier, prim_path

        return self.source_stage.GetRootLayer().identifier, prim_path

    def _find_material_sources(self, scope: str | None) -> list[MaterialSource]:
        """Find all material prims and track their source information.

        Physics materials (those with ``PhysicsMaterialAPI`` applied) are
        excluded because they belong in the physics layer, not the visual
        materials layer.

        Args:
            scope: Optional root scope path to limit the search.

        Returns:
            List of MaterialSource objects describing each material.

        """
        material_sources = []
        root_prim = utils.get_scope_root(self.source_stage, scope or _DEFAULT_SCOPE)

        for prim in Usd.PrimRange(root_prim, Usd.TraverseInstanceProxies()):
            if not prim.IsA(UsdShade.Material):
                continue

            if "PhysicsMaterialAPI" in prim.GetAppliedSchemas():
                self.log_operation(f"Skipping physics material: {prim.GetPath()}")
                continue

            prim_path = prim.GetPath().pathString
            defining_layer_id, defining_prim_path = self._find_defining_layer(prim)

            material_sources.append(
                MaterialSource(
                    prim_path=prim_path,
                    defining_layer_id=defining_layer_id,
                    defining_prim_path=defining_prim_path,
                )
            )

        return material_sources

    def _scan_existing_materials(self, mat_stage: Usd.Stage) -> dict[str, MaterialEntry]:
        """Scan existing materials in the materials layer and compute their hashes.

        This enables deduplication against previously imported materials.

        Args:
            mat_stage: The materials stage to scan.

        Returns:
            Dictionary mapping material hashes to MaterialEntry objects for existing materials.

        """
        existing: dict[str, MaterialEntry] = {}

        materials_scope = mat_stage.GetPrimAtPath(_MATERIALS_SCOPE_PATH)
        if not materials_scope.IsValid():
            return existing

        for material_prim in materials_scope.GetChildren():
            if not material_prim.IsValid() or not material_prim.IsA(UsdShade.Material):
                continue

            material_path = material_prim.GetPath().pathString
            material_name = material_prim.GetName()

            # Compute hash for this existing material
            mat_hash = self._compute_material_hash(material_prim)

            entry = MaterialEntry(
                name=material_name,
                material_layer_path=material_path,
                sources=[],
                existing=True,
            )
            existing[mat_hash] = entry

        return existing

    def _compute_material_hash(self, prim: Usd.Prim, resolved_asset_paths: list[str] | None = None) -> str:
        """Compute a hash of the material's properties for deduplication.

        Uses resolved (absolute) asset paths for hashing to ensure materials using
        identical assets hash the same regardless of how paths are written in USD.

        Args:
            prim: The material prim to hash.
            resolved_asset_paths: Optional list of resolved asset paths for consistent hashing.

        Returns:
            A hex string hash representing the material's content.

        """
        hasher = hashlib.sha256()
        material_root_path = prim.GetPath().pathString

        # Create a mapping from filename to resolved path for consistent hashing
        asset_by_filename: dict[str, str] = {}
        if resolved_asset_paths:
            for resolved_path in sorted(resolved_asset_paths):
                asset_by_filename[os.path.basename(resolved_path)] = resolved_path

        def make_relative_path(abs_path: str) -> str:
            """Convert absolute path to relative path from the material root.

            Args:
                abs_path: Absolute path to normalize.

            Returns:
                Relative path from the material root.

            """
            if abs_path.startswith(material_root_path):
                return abs_path[len(material_root_path) :]
            return Sdf.Path(abs_path).name

        def hash_prim_recursive(p: Usd.Prim) -> None:
            hasher.update(p.GetTypeName().encode())

            for attr in sorted(p.GetAttributes(), key=lambda a: a.GetName()):
                if not attr.HasAuthoredValue() and not attr.GetConnections():
                    continue

                hasher.update(attr.GetName().encode())
                value = attr.Get()
                if value is not None:
                    if isinstance(value, Sdf.AssetPath):
                        filename = os.path.basename(value.path) if value.path else ""
                        hasher.update(asset_by_filename.get(filename, filename).encode())
                    else:
                        hasher.update(str(value).encode())

                for conn in sorted(attr.GetConnections(), key=lambda c: c.pathString):
                    hasher.update(make_relative_path(conn.pathString).encode())

            for rel in sorted(p.GetRelationships(), key=lambda r: r.GetName()):
                if not rel.HasAuthoredTargets():
                    continue
                hasher.update(rel.GetName().encode())
                for target in sorted(rel.GetTargets(), key=lambda t: t.pathString):
                    hasher.update(make_relative_path(target.pathString).encode())

            for child in sorted(p.GetFilteredChildren(Usd.TraverseInstanceProxies()), key=lambda c: c.GetName()):
                hasher.update(child.GetName().encode())
                hash_prim_recursive(child)

        hash_prim_recursive(prim)
        return hasher.hexdigest()[:_MATERIAL_HASH_LENGTH]

    def _collect_asset_paths(self, prim: Usd.Prim, include_remote: bool = False) -> tuple[list[str], list[str]]:
        """Collect transferable asset paths used by a material and its shader children.

        Scans authored asset attributes, resolves local paths based on the authored
        layer stack, and includes MDL dependencies (plus textures referenced by MDL).

        Args:
            prim: The material prim to scan for asset paths.
            include_remote: If True, include remote paths (e.g., omniverse://) for download.

        Returns:
            Tuple of (asset_paths, mdl_paths) to transfer.

        """
        asset_paths: set[str] = set()
        mdl_paths: set[str] = set()

        def is_transferable_asset(path: str) -> bool:
            """Check if an asset should be transferred based on its extension.

            Args:
                path: Asset path to evaluate.

            Returns:
                True if the asset should be transferred.

            """
            if not path:
                return False
            ext = os.path.splitext(path)[1].lower()
            return ext in _TRANSFERABLE_ASSET_EXTENSIONS

        def add_asset(path: str) -> None:
            if not path:
                return
            # A built-in-named MDL (e.g. OmniPBR.mdl) that is NOT bundled as a
            # real file ships with the runtime and must not be extracted or have
            # its path rewritten -- Kit resolves it through the MDL search paths.
            # But when the asset bundles its own copy (the path resolves to a
            # real file), transfer it like any other asset so the composed
            # package stays self-contained and the reference resolves on disk.
            if utils.is_builtin_mdl(path) and not os.path.isfile(path):
                return
            asset_paths.add(path)
            if os.path.splitext(path)[1].lower() == ".mdl":
                mdl_paths.add(path)

        def layer_dirs_from_layer(layer: Sdf.Layer | None) -> list[str]:
            if not layer:
                return []
            dirs: list[str] = []
            if layer.realPath:
                dirs.append(os.path.dirname(layer.realPath))
            identifier = layer.identifier or ""
            if identifier and not identifier.startswith("anon:"):
                dirs.append(os.path.dirname(identifier))
                dirs.append(os.path.dirname(os.path.abspath(identifier)))
            return [d for d in dirs if d]

        def get_attr_resolution_dirs(attr: Usd.Attribute, stage_layer: Sdf.Layer) -> list[str]:
            dirs: list[str] = []
            for prop_spec in attr.GetPropertyStack():
                dirs.extend(layer_dirs_from_layer(prop_spec.layer))
            dirs.extend(layer_dirs_from_layer(stage_layer))
            # Deduplicate while preserving order
            seen: set[str] = set()
            unique_dirs: list[str] = []
            for d in dirs:
                norm = os.path.normpath(d)
                if norm not in seen:
                    seen.add(norm)
                    unique_dirs.append(norm)
            return unique_dirs

        def resolve_relative_path(raw_path: str, candidate_dirs: list[str]) -> str:
            for base_dir in candidate_dirs:
                abs_path = os.path.normpath(os.path.join(base_dir, raw_path))
                if os.path.exists(abs_path):
                    return abs_path
            return ""

        stage_layer = prim.GetStage().GetRootLayer()

        def collect_from_prim(p: Usd.Prim) -> None:
            for attr in p.GetAttributes():
                if not attr.HasAuthoredValue():
                    continue

                value = attr.Get()
                if value is None:
                    continue

                if isinstance(value, Sdf.AssetPath):
                    raw_path = value.path
                    resolved_path = value.resolvedPath

                    if not is_transferable_asset(raw_path) and not is_transferable_asset(resolved_path):
                        continue

                    # UDIM families never resolve to a single file. Collect the
                    # template path itself; concrete tiles are enumerated and
                    # transferred later by ``_transfer_udim_family``.
                    if self._is_udim_path(raw_path) or self._is_udim_path(resolved_path):
                        udim_template = self._collectable_udim_template(
                            raw_path,
                            resolved_path,
                            get_attr_resolution_dirs(attr, stage_layer),
                            include_remote,
                        )
                        if udim_template:
                            add_asset(udim_template)
                        continue

                    # Remote assets
                    if include_remote:
                        if resolved_path and utils.is_remote_path(resolved_path):
                            add_asset(resolved_path)
                            continue
                        if raw_path and utils.is_remote_path(raw_path):
                            add_asset(raw_path)
                            continue

                    # Local resolved path
                    if resolved_path and os.path.exists(resolved_path):
                        add_asset(resolved_path)
                        continue

                    # Resolve relative or absolute raw paths
                    if raw_path:
                        if os.path.isabs(raw_path) and os.path.exists(raw_path):
                            add_asset(raw_path)
                        else:
                            candidate_dirs = get_attr_resolution_dirs(attr, stage_layer)
                            abs_path = resolve_relative_path(raw_path, candidate_dirs)
                            if abs_path:
                                add_asset(abs_path)

            for child in p.GetFilteredChildren(Usd.TraverseInstanceProxies()):
                collect_from_prim(child)

        collect_from_prim(prim)

        # Bundle the local MDL module import closure (sibling ``.mdl`` modules
        # referenced via ``import Foo::*;`` / ``using Foo import *;``). Kit cannot
        # compile a relocated MDL unless the modules it imports travel with it and
        # stay co-located, so the whole local closure must be transferred
        # (VM.BIND.002). ``add_asset`` also adds them to ``mdl_paths``.
        if mdl_paths:
            for module_path in self._collect_mdl_module_imports(set(mdl_paths)):
                add_asset(module_path)

        # Parse all MDL modules (primary + imported closure) for texture deps.
        if mdl_paths:
            for dep_path in self._collect_mdl_texture_paths(set(mdl_paths), include_remote=include_remote):
                add_asset(dep_path)

        return list(asset_paths), list(mdl_paths)

    def _collect_mdl_module_imports(self, mdl_paths: set[str]) -> set[str]:
        """Resolve the local sibling MDL modules imported by the given MDL files.

        MDL modules import sibling modules by relative module name (e.g.
        ``import OmniPBR_ClearCoat::*;`` resolves to ``OmniPBR_ClearCoat.mdl`` in
        the same directory). A relocated/bundled MDL only compiles when the whole
        local import closure travels with it and stays co-located, so every
        reachable sibling ``.mdl`` file is collected transitively. Absolute or
        standard imports (``import ::df::*;``) and module names without a
        co-located file on disk are ignored -- those are runtime-provided.

        Args:
            mdl_paths: Set of local (or remote) MDL paths to scan.

        Returns:
            Set of resolved local sibling ``.mdl`` paths in the import closure.

        """
        closure: set[str] = set()
        seen: set[str] = set()
        worklist = [p for p in mdl_paths if p and not utils.is_remote_path(p) and os.path.isfile(p)]
        while worklist:
            mdl_path = worklist.pop()
            norm = os.path.normpath(mdl_path)
            if norm in seen:
                continue
            seen.add(norm)
            try:
                with open(mdl_path, encoding="utf-8", errors="ignore") as f:
                    text = f.read()
            except Exception as e:
                self.log_operation(f"Failed to read MDL for import scan {mdl_path}: {e}")
                continue
            # Strip comments so commented-out imports are not collected.
            text = re.sub(r"/\*.*?\*/", "", text, flags=re.DOTALL)
            text = re.sub(r"//.*", "", text)
            base_dir = os.path.dirname(mdl_path)
            for module_name in _MDL_MODULE_IMPORT_PATTERN.findall(text):
                sibling = os.path.normpath(os.path.join(base_dir, f"{module_name}.mdl"))
                if os.path.normpath(sibling) in seen:
                    continue
                if os.path.isfile(sibling):
                    closure.add(sibling)
                    worklist.append(sibling)
        return closure

    def _collect_mdl_texture_paths(self, mdl_paths: set[str], include_remote: bool = False) -> list[str]:
        """Collect texture dependencies referenced inside MDL files.

        Args:
            mdl_paths: Set of local or remote MDL paths to parse.
            include_remote: If True, allow remote MDL files to be read.

        Returns:
            List of resolved texture paths referenced by the MDL files.

        """
        deps: set[str] = set()
        for mdl_path in sorted(mdl_paths):
            if not mdl_path:
                continue
            if utils.is_remote_path(mdl_path):
                if not include_remote:
                    continue
                mdl_text = self._read_remote_text(mdl_path)
                if not mdl_text:
                    continue
                base_dir = self._remote_dir(mdl_path)
                deps.update(self._extract_mdl_texture_paths(mdl_text, base_dir, base_is_remote=True))
            else:
                if not os.path.isfile(mdl_path):
                    continue
                try:
                    with open(mdl_path, encoding="utf-8", errors="ignore") as f:
                        mdl_text = f.read()
                except Exception as e:
                    self.log_operation(f"Failed to read MDL file {mdl_path}: {e}")
                    continue
                base_dir = os.path.dirname(mdl_path)
                deps.update(self._extract_mdl_texture_paths(mdl_text, base_dir, base_is_remote=False))
        return list(deps)

    def _extract_mdl_texture_paths(self, mdl_text: str, base_dir: str, base_is_remote: bool) -> set[str]:
        """Extract and resolve texture paths from MDL content.

        Args:
            mdl_text: MDL source text.
            base_dir: Base directory or remote URL for relative resolution.
            base_is_remote: True if base_dir is a remote URL.

        Returns:
            Set of resolved dependency paths.

        """
        if not mdl_text:
            return set()
        # Strip comments to avoid collecting commented-out resources
        text = re.sub(r"/\*.*?\*/", "", mdl_text, flags=re.DOTALL)
        text = re.sub(r"//.*", "", text)

        deps: set[str] = set()
        for match in _MDL_TEXTURE_PATH_PATTERN.finditer(text):
            raw_path = match.group(1)
            resolved = self._resolve_mdl_dependency_path(raw_path, base_dir, base_is_remote)
            if resolved:
                deps.add(resolved)
        return deps

    def _resolve_mdl_dependency_path(self, raw_path: str, base_dir: str, base_is_remote: bool) -> str:
        """Resolve a texture path referenced from an MDL file.

        Args:
            raw_path: Raw path in MDL.
            base_dir: Base directory or remote URL for relative resolution.
            base_is_remote: True if base_dir is a remote URL.

        Returns:
            Resolved path string, or empty string if not resolvable.

        """
        if not raw_path:
            return ""
        if utils.is_remote_path(raw_path):
            return raw_path
        if os.path.isabs(raw_path):
            return raw_path
        if base_is_remote:
            return self._join_remote_path(base_dir, raw_path)
        return os.path.normpath(os.path.join(base_dir, raw_path))

    def _remote_dir(self, remote_path: str) -> str:
        """Return the directory portion of a remote URL.

        Args:
            remote_path: Remote URL string.

        Returns:
            Remote URL directory.

        """
        if "/" not in remote_path:
            return remote_path
        return remote_path.rsplit("/", 1)[0]

    def _join_remote_path(self, base: str, rel_path: str) -> str:
        """Join a remote base URL with a relative path.

        Args:
            base: Remote base URL.
            rel_path: Relative path to join.

        Returns:
            Joined remote URL.

        """
        rel = rel_path.lstrip("./")
        return f"{base.rstrip('/')}/{rel.lstrip('/')}"

    def _read_remote_text(self, src_url: str) -> str:
        """Read a remote text file using omni.client.

        Args:
            src_url: Remote URL to read.

        Returns:
            Decoded text content, or empty string if unavailable.

        """
        try:
            import omni.client

            result, _, content = omni.client.read_file(src_url)
            if result != omni.client.Result.OK:
                self.log_operation(f"Failed to read remote text {src_url}: {result}")
                return ""
            return bytes(memoryview(content)).decode("utf-8", errors="ignore")
        except ImportError:
            self.log_operation("omni.client not available, cannot read remote text")
            return ""
        except Exception as e:
            self.log_operation(f"Failed to read remote text {src_url}: {e}")
            return ""

    def _download_remote_asset(self, src_url: str, dst_path: str) -> bool:
        """Download a remote asset using omni.client.

        Args:
            src_url: The remote URL to download from.
            dst_path: The local destination path.

        Returns:
            True if download succeeded, False otherwise.

        """
        try:
            import omni.client

            result, _, content = omni.client.read_file(src_url)
            if result != omni.client.Result.OK:
                self.log_operation(f"Failed to read remote asset {src_url}: {result}")
                return False

            with open(dst_path, "wb") as f:
                f.write(memoryview(content))

            self.log_operation(f"Downloaded asset: {src_url} -> {dst_path}")
            return True
        except ImportError:
            self.log_operation("omni.client not available, cannot download remote assets")
            return False
        except Exception as e:
            self.log_operation(f"Failed to download asset {src_url}: {e}")
            return False

    def _transfer_all_assets(
        self,
        asset_paths: list[str],
        assets_output_path: str,
        materials_layer_dir: str,
        download_remote: bool = False,
    ) -> dict[str, str]:
        """Transfer all assets to the designated folder, deduplicating globally.

        This transfers assets BEFORE materials are created, ensuring all materials
        reference the same transferred files.

        Args:
            asset_paths: List of absolute/resolved asset paths to transfer.
            assets_output_path: Absolute path to the assets output folder.
            materials_layer_dir: Directory of the materials layer (for relative path computation).
            download_remote: If True, download remote assets using omni.client.

        Returns:
            Dictionary mapping original resolved path -> new relative path from materials layer.

        """
        path_mapping: dict[str, str] = {}
        used_filenames: dict[str, str] = {}

        for src_path in sorted(set(asset_paths)):
            if not src_path:
                continue

            # UDIM templates do not name a single file -- expand them into their
            # concrete tiles, transfer each, and map the template to a local
            # ``<UDIM>`` template so the resolver finds the downloaded tiles.
            if self._is_udim_path(src_path):
                template_rel = self._transfer_udim_family(
                    src_path,
                    assets_output_path,
                    materials_layer_dir,
                    used_filenames,
                    download_remote,
                )
                if template_rel:
                    path_mapping[src_path] = template_rel
                continue

            relative_path = self._transfer_single_asset(
                src_path,
                assets_output_path,
                materials_layer_dir,
                used_filenames,
                download_remote,
            )
            if relative_path:
                path_mapping[src_path] = relative_path

        return path_mapping

    def _transfer_single_asset(
        self,
        src_path: str,
        assets_output_path: str,
        materials_layer_dir: str,
        used_filenames: dict[str, str],
        download_remote: bool,
    ) -> str | None:
        """Transfer one concrete asset file to the assets folder.

        Handles filename-collision renaming and content-identity short-circuits.
        ``used_filenames`` maps a destination filename to the source path that
        claimed it and is updated in place.

        Args:
            src_path: Absolute/resolved local path or remote URL of a single file.
            assets_output_path: Absolute path to the assets output folder.
            materials_layer_dir: Directory of the materials layer (for relative path computation).
            used_filenames: Filename -> source-path map used for collision tracking.
            download_remote: If True, download remote assets using omni.client.

        Returns:
            Explicit relative path (from the materials layer) to the transferred
            file, or None when the asset was skipped.

        """
        is_remote = utils.is_remote_path(src_path)

        # Skip remote paths if download is disabled
        if is_remote and not download_remote:
            return None

        # Skip local paths that don't exist
        if not is_remote and not os.path.exists(src_path):
            return None

        filename = os.path.basename(src_path)
        dst_path = os.path.join(assets_output_path, filename)

        # Handle filename collisions
        if filename in used_filenames:
            existing_src = used_filenames[filename]
            if existing_src == src_path:
                return self._make_relative_asset_path(dst_path, materials_layer_dir)
            elif not is_remote and os.path.exists(dst_path) and utils.files_are_identical(src_path, dst_path):
                return self._make_relative_asset_path(dst_path, materials_layer_dir)
            else:
                base, ext = os.path.splitext(filename)
                counter = 1
                while filename in used_filenames or os.path.exists(dst_path):
                    if (
                        filename not in used_filenames
                        and not is_remote
                        and os.path.exists(dst_path)
                        and utils.files_are_identical(src_path, dst_path)
                    ):
                        break
                    filename = f"{base}_{counter}{ext}"
                    dst_path = os.path.join(assets_output_path, filename)
                    counter += 1

        # Transfer the file if not already there
        if not os.path.exists(dst_path):
            if is_remote:
                if not self._download_remote_asset(src_path, dst_path):
                    return None
            else:
                try:
                    shutil.copy2(src_path, dst_path)
                    self.log_operation(f"Copied asset: {src_path} -> {dst_path}")
                except Exception as e:
                    self.log_operation(f"Failed to copy asset {src_path}: {e}")
                    return None

        used_filenames[filename] = src_path
        return self._make_relative_asset_path(dst_path, materials_layer_dir)

    def _transfer_udim_family(
        self,
        template_path: str,
        assets_output_path: str,
        materials_layer_dir: str,
        used_filenames: dict[str, str],
        download_remote: bool,
    ) -> str | None:
        """Expand a UDIM template, transfer every concrete tile, and map the family.

        Each tile is transferred via :meth:`_transfer_single_asset`, so tiles
        share the same collision handling as ordinary assets. The returned
        relative path keeps the canonical ``<UDIM>`` token so the USD resolver
        expands it against the transferred tiles.

        Args:
            template_path: UDIM template path (remote URL or local absolute path).
            assets_output_path: Absolute path to the assets output folder.
            materials_layer_dir: Directory of the materials layer (for relative path computation).
            used_filenames: Filename -> source-path map used for collision tracking.
            download_remote: If True, download remote tiles using omni.client.

        Returns:
            Explicit relative ``<UDIM>`` template path (from the materials layer)
            when at least one tile was transferred, otherwise None.

        """
        is_remote = utils.is_remote_path(template_path)
        if is_remote and not download_remote:
            return None

        tiles = self._expand_udim_tiles(template_path, is_remote)
        if not tiles:
            self.log_operation(f"No UDIM tiles found for {template_path}")
            return None

        transferred = 0
        for tile_path in sorted(set(tiles)):
            if self._transfer_single_asset(
                tile_path,
                assets_output_path,
                materials_layer_dir,
                used_filenames,
                download_remote,
            ):
                transferred += 1

        if transferred == 0:
            return None

        template_name = self._canonical_udim_name(os.path.basename(template_path))
        template_dst = os.path.join(assets_output_path, template_name)
        self.log_operation(f"Transferred {transferred} UDIM tile(s) for {template_name}")
        return self._make_relative_asset_path(template_dst, materials_layer_dir)

    @staticmethod
    def _is_udim_path(path: str) -> bool:
        """Return True if ``path`` contains a UDIM tile token (``<UDIM>``).

        Args:
            path: Asset path to inspect.

        Returns:
            True if the path contains a UDIM tile token.

        """
        return bool(path) and _UDIM_TOKEN_PATTERN.search(path) is not None

    @staticmethod
    def _canonical_udim_name(name: str) -> str:
        """Rewrite any UDIM token form in ``name`` to the canonical ``<UDIM>``.

        Args:
            name: Filename that may contain a UDIM token in any form.

        Returns:
            The name with any UDIM token normalized to ``<UDIM>``.

        """
        return _UDIM_TOKEN_PATTERN.sub(_UDIM_CANONICAL_TOKEN, name)

    @staticmethod
    def _udim_filename_matcher(template_name: str) -> re.Pattern | None:
        """Build a regex matching concrete tile filenames for a UDIM template.

        The token is replaced by a four-digit group and every literal segment is
        escaped, so ``foo.<UDIM>.png`` matches ``foo.1001.png`` but not
        ``foo_extra.1001.png``.

        Args:
            template_name: UDIM template filename (any token form).

        Returns:
            Compiled, anchored, case-insensitive pattern, or None when
            ``template_name`` carries no UDIM token.

        """
        segments = _UDIM_TOKEN_PATTERN.split(template_name)
        if len(segments) < 2:
            return None
        pattern = _UDIM_TILE_REGEX.join(re.escape(segment) for segment in segments)
        return re.compile(rf"^{pattern}$", re.IGNORECASE)

    def _expand_udim_tiles(self, template_path: str, is_remote: bool) -> list[str]:
        """Enumerate concrete tile paths for a UDIM template.

        Args:
            template_path: UDIM template path (remote URL or local absolute path).
            is_remote: True when ``template_path`` is a remote URL.

        Returns:
            List of concrete tile paths (remote URLs or local paths).

        """
        if is_remote:
            return self._expand_remote_udim_tiles(template_path)
        return self._expand_local_udim_tiles(template_path)

    def _expand_local_udim_tiles(self, template_path: str) -> list[str]:
        """Enumerate concrete tiles for a local UDIM template by scanning its folder.

        Args:
            template_path: Local UDIM template path.

        Returns:
            Sorted list of absolute tile paths present on disk.

        """
        directory = os.path.dirname(template_path)
        matcher = self._udim_filename_matcher(os.path.basename(template_path))
        if matcher is None or not os.path.isdir(directory):
            return []
        return sorted(os.path.join(directory, name) for name in os.listdir(directory) if matcher.match(name))

    def _expand_remote_udim_tiles(self, template_url: str) -> list[str]:
        """Enumerate concrete tiles for a remote UDIM template via ``omni.client.list``.

        Args:
            template_url: Remote UDIM template URL.

        Returns:
            Sorted list of concrete tile URLs found on the server.

        """
        try:
            import omni.client
        except ImportError:
            self.log_operation("omni.client not available, cannot expand remote UDIM tiles")
            return []

        directory = self._remote_dir(template_url)
        matcher = self._udim_filename_matcher(template_url[len(directory) :].lstrip("/"))
        if matcher is None:
            return []

        try:
            result, entries = omni.client.list(directory)
        except Exception as e:
            self.log_operation(f"Failed to list remote UDIM folder {directory}: {e}")
            return []
        if result != omni.client.Result.OK:
            self.log_operation(f"Failed to list remote UDIM folder {directory}: {result}")
            return []

        tiles: list[str] = []
        for entry in entries:
            name = getattr(entry, "relative_path", "") or ""
            if name and matcher.match(name):
                tiles.append(self._join_remote_path(directory, name))
        return sorted(tiles)

    def _collectable_udim_template(
        self,
        raw_path: str,
        resolved_path: str,
        candidate_dirs: list[str],
        include_remote: bool,
    ) -> str:
        """Return the UDIM template path to transfer for a UDIM asset reference.

        The USD resolver may leave a UDIM reference unresolved, keep the
        template, or expand it to a single concrete tile. This normalizes any of
        those outcomes to a template path whose folder can be enumerated at
        transfer time:

        * a template that is already remote/absolute is returned directly;
        * a resolved concrete tile is turned back into a template in the tile's
          folder so the whole family is discovered;
        * a relative template is resolved against the attribute's layer stack,
          keeping only a folder that actually contains matching tiles.

        Args:
            raw_path: Authored asset path (``value.path``).
            resolved_path: Resolver output (``value.resolvedPath``); may be empty,
                a template, or a single expanded tile.
            candidate_dirs: Directories to resolve a relative template against.
            include_remote: If True, remote templates are eligible for download.

        Returns:
            A template path (remote URL or local absolute path), or "" when none
            could be determined.

        """
        candidate = ""

        if resolved_path and self._is_udim_path(resolved_path):
            # Resolver preserved the template.
            if utils.is_remote_path(resolved_path):
                return resolved_path if include_remote else ""
            if os.path.isabs(resolved_path):
                return resolved_path
            candidate = resolved_path
        elif resolved_path and self._is_udim_path(raw_path):
            # Resolver expanded the template to a single concrete tile; rebuild
            # the template in the tile's folder so the whole family is found.
            template_name = self._canonical_udim_name(os.path.basename(raw_path))
            if utils.is_remote_path(resolved_path):
                template = self._join_remote_path(self._remote_dir(resolved_path), template_name)
                return template if include_remote else ""
            return os.path.join(os.path.dirname(resolved_path), template_name)
        elif self._is_udim_path(raw_path):
            if utils.is_remote_path(raw_path):
                return raw_path if include_remote else ""
            if os.path.isabs(raw_path):
                return raw_path
            candidate = raw_path

        if not candidate:
            return ""

        # Relative template: resolve against the layer stack, keeping the first
        # folder that actually contains matching tiles.
        for base_dir in candidate_dirs:
            abs_template = os.path.normpath(os.path.join(base_dir, candidate))
            if self._expand_local_udim_tiles(abs_template):
                return abs_template
        return ""

    @staticmethod
    def _make_relative_asset_path(dst_path: str, materials_layer_dir: str) -> str:
        """Build an explicit, forward-slash relative path for an asset reference.

        Material references must always be relative to the materials layer so
        that the produced package is portable. ``make_explicit_relative``
        normalizes separators to forward slashes for us.

        Args:
            dst_path: Absolute destination path of the transferred asset.
            materials_layer_dir: Directory containing the materials layer.

        Returns:
            Explicit relative path with forward-slash separators (e.g. ``./Textures/foo.png``).

        """
        return utils.make_explicit_relative(os.path.relpath(dst_path, materials_layer_dir))

    @staticmethod
    def _resolve_existing_folder_casing(parent: str, name: str) -> str:
        """Return ``name`` with its on-disk casing if a matching folder already exists.

        This exists to keep in-place asset transformations portable across
        filesystems with different case-sensitivity rules. On a case-insensitive
        filesystem (Windows, default macOS), creating ``Textures/`` with
        ``os.makedirs`` when ``textures/`` already exists is a silent no-op:
        new files land in the pre-existing directory but the rule still rewrites
        every shader ``assetPath`` to the configured ``Textures`` casing. The
        package then loads on Windows (case-insensitive resolution) but fails on
        Linux. Preferring the existing casing avoids mutating any on-disk state
        while still producing a USD whose paths match what is actually on disk.

        Falls back to ``name`` unchanged when the parent is not a directory,
        when no matching subdirectory exists, or when ``name`` itself contains
        path separators (multi-level resolution is intentionally not attempted).

        Args:
            parent: Absolute path of the directory that will contain the folder.
            name: Configured folder name (e.g. ``"Textures"``).

        Returns:
            Either the existing folder's on-disk name (preferring an exact match,
            then any case-insensitive match) or ``name`` unchanged when no match
            is found.

        """
        if "/" in name or "\\" in name or not os.path.isdir(parent):
            return name

        target_lower = name.lower()
        insensitive_match: str | None = None

        try:
            for entry in sorted(os.listdir(parent)):
                if not os.path.isdir(os.path.join(parent, entry)):
                    continue
                if entry == name:
                    return entry
                if insensitive_match is None and entry.lower() == target_lower:
                    insensitive_match = entry
        except OSError:
            return name

        return insensitive_match or name

    def _update_mdl_texture_paths(
        self,
        mdl_paths: set[str],
        path_mapping: dict[str, str],
        materials_layer_dir: str,
    ) -> None:
        """Update texture references inside copied MDL files.

        Args:
            mdl_paths: Set of source MDL paths.
            path_mapping: Mapping from source path to new relative path.
            materials_layer_dir: Materials layer directory for absolute resolution.

        """
        if not mdl_paths or not path_mapping:
            return

        updated_files = 0
        replaced_total = 0

        for mdl_src_path in sorted(mdl_paths):
            dst_rel = path_mapping.get(mdl_src_path)
            if not dst_rel:
                continue

            mdl_dst_abs = os.path.normpath(os.path.join(materials_layer_dir, dst_rel))
            if not os.path.isfile(mdl_dst_abs):
                continue

            try:
                with open(mdl_dst_abs, encoding="utf-8", errors="ignore") as f:
                    content = f.read()
            except Exception as e:
                self.log_operation(f"Failed to read MDL for update {mdl_dst_abs}: {e}")
                continue

            new_content, replaced = self._remap_mdl_texture_paths(
                content,
                mdl_src_path,
                mdl_dst_abs,
                path_mapping,
                materials_layer_dir,
            )
            if replaced > 0 and new_content != content:
                try:
                    with open(mdl_dst_abs, "w", encoding="utf-8") as f:
                        f.write(new_content)
                    updated_files += 1
                    replaced_total += replaced
                except Exception as e:
                    self.log_operation(f"Failed to write MDL {mdl_dst_abs}: {e}")

        if updated_files:
            self.log_operation(f"Updated {replaced_total} texture references in {updated_files} MDL files")

    def _remap_mdl_texture_paths(
        self,
        mdl_text: str,
        mdl_src_path: str,
        mdl_dst_abs: str,
        path_mapping: dict[str, str],
        materials_layer_dir: str,
    ) -> tuple[str, int]:
        """Rewrite texture paths in an MDL file based on transferred assets.

        Args:
            mdl_text: MDL file contents.
            mdl_src_path: Source MDL path (local or remote).
            mdl_dst_abs: Destination MDL absolute path.
            path_mapping: Mapping from source path to new relative path.
            materials_layer_dir: Materials layer directory for absolute resolution.

        Returns:
            Tuple of (updated_text, replacements_count).

        """
        base_is_remote = utils.is_remote_path(mdl_src_path)
        base_dir = self._remote_dir(mdl_src_path) if base_is_remote else os.path.dirname(mdl_src_path)
        mdl_dst_dir = os.path.dirname(mdl_dst_abs)

        def replacer(match: re.Match) -> str:
            raw_path = match.group(1)
            resolved = self._resolve_mdl_dependency_path(raw_path, base_dir, base_is_remote)
            if not resolved:
                return match.group(0)
            mapped_rel = path_mapping.get(resolved)
            if not mapped_rel:
                return match.group(0)
            mapped_abs = os.path.normpath(os.path.join(materials_layer_dir, mapped_rel))
            new_rel = self._make_relative_asset_path(mapped_abs, mdl_dst_dir)
            return f'"{new_rel}"'

        return _MDL_TEXTURE_PATH_PATTERN.subn(replacer, mdl_text)

    def _copy_material_to_layer(
        self,
        prim: Usd.Prim,
        entry: MaterialEntry,
        mat_stage: Usd.Stage,
        asset_path_mapping: dict[str, str],
        materials_layer_dir: str,
    ) -> None:
        """Copy a material prim to the materials layer.

        Uses composed stage copy as primary method to handle instance proxies
        and materials from references properly. Falls back to Sdf.CopySpec
        if composed stage copy fails.

        Args:
            prim: The source material prim.
            entry: The MaterialEntry describing where to store it.
            mat_stage: The materials stage to copy to.
            asset_path_mapping: Mapping from original resolved path to new relative path.
            materials_layer_dir: Directory of the materials layer used to relativize
                any absolute asset paths that survive the copy.

        """
        dst_layer = mat_stage.GetRootLayer()
        src_prim_path = prim.GetPath().pathString

        # Primary: Use composed stage copy to ensure shader children are captured
        # This handles instance proxies and materials from references properly
        copy_succeeded = utils.copy_prim_from_composed_stage(
            prim,
            dst_layer,
            entry.material_layer_path,
            remap_connections_from=src_prim_path,
            remap_connections_to=entry.material_layer_path,
        )

        if not copy_succeeded:
            # Fallback: try Sdf.CopySpec if composed stage copy fails
            is_instance_proxy = prim.IsInstanceProxy()
            if not is_instance_proxy:
                prim_stack = prim.GetPrimStack()
                for prim_spec in prim_stack:
                    if prim_spec.specifier == Sdf.SpecifierDef:
                        src_material_layer = prim_spec.layer
                        src_material_path = prim_spec.path
                        Sdf.CopySpec(
                            src_material_layer, src_material_path, dst_layer, Sdf.Path(entry.material_layer_path)
                        )
                        copy_succeeded = True
                        break

        if not copy_succeeded:
            self.log_operation(f"Failed to copy material {src_prim_path}")
            return

        # Clear instanceable flags recursively to ensure shader children are accessible
        copied_spec = dst_layer.GetPrimAtPath(entry.material_layer_path)
        if copied_spec:
            utils.clear_instanceable_recursive(copied_spec)

        # Always normalize asset paths in the copied material so the materials layer never
        # contains absolute paths or backslash separators, even when an asset was not
        # transferred (and therefore is not present in ``asset_path_mapping``).
        self._update_asset_paths_in_material(
            entry.material_layer_path, dst_layer, asset_path_mapping, materials_layer_dir
        )

        self.log_operation(f"Copied material {prim.GetPath()} to {entry.material_layer_path}")

    def _update_asset_paths_in_material(
        self,
        material_path: str,
        layer: Sdf.Layer,
        path_mapping: dict[str, str],
        materials_layer_dir: str,
    ) -> None:
        """Update asset paths in a material spec.

        Asset paths in the materials layer must be portable: they must use
        forward-slash separators and must never be absolute. This method
        rewrites known transferred assets via ``path_mapping`` and normalizes
        any remaining asset paths so absolute paths are converted to relative
        paths against the materials layer directory.

        Args:
            material_path: Path to the material in the layer.
            layer: The layer containing the material.
            path_mapping: Dictionary mapping old resolved paths to new relative paths.
            materials_layer_dir: Directory of the materials layer, used to relativize
                absolute asset paths that are not present in ``path_mapping``.

        """
        # Build lookup tables for fast matching
        by_resolved: dict[str, str] = path_mapping.copy()
        by_filename: dict[str, str] = {}
        for src_path, new_rel_path in path_mapping.items():
            filename = os.path.basename(src_path)
            by_filename[filename] = new_rel_path

        def normalize_unmapped(asset_path: str) -> str | None:
            """Normalize an asset path that is not in ``path_mapping``.

            Args:
                asset_path: Asset path value authored on the material.

            Returns:
                A forward-slash, relative path, or ``None`` if the path does
                not need to be rewritten.

            """
            if not asset_path:
                return None
            # Leave remote paths alone -- they are already URL-form.
            if utils.is_remote_path(asset_path):
                return None
            normalized = asset_path.replace("\\", "/")
            if os.path.isabs(asset_path) or os.path.isabs(normalized):
                try:
                    rel = os.path.relpath(asset_path, materials_layer_dir)
                except ValueError:
                    # Different drives on Windows: cannot make relative.
                    return None
                return self._make_relative_asset_path(
                    os.path.normpath(os.path.join(materials_layer_dir, rel)),
                    materials_layer_dir,
                )
            if normalized != asset_path:
                return normalized
            return None

        def update_spec_recursive(prim_spec: Sdf.PrimSpec) -> None:
            if prim_spec is None:
                return

            for attr_name in list(prim_spec.attributes.keys()):  # noqa: SIM118
                attr_spec = prim_spec.attributes[attr_name]
                if attr_spec.default is None:
                    continue

                # Check if this is an asset attribute
                value = attr_spec.default
                if isinstance(value, Sdf.AssetPath):
                    old_path = value.path
                    old_resolved = value.resolvedPath
                    new_rel_path = None

                    # Determine whether the authored MDL is actually bundled as
                    # a real file in the package/source (as opposed to a bare
                    # reference to a runtime-provided module). A bundled copy is
                    # kept and referenced with a resolvable, self-contained path;
                    # only a non-bundled built-in reference is collapsed to the
                    # canonical Kit identifier.
                    bundled_on_disk = bool(old_resolved and os.path.isfile(old_resolved))
                    if not bundled_on_disk and old_path and not os.path.isabs(old_path):
                        candidate = os.path.normpath(os.path.join(materials_layer_dir, old_path))
                        bundled_on_disk = os.path.isfile(candidate)

                    # Built-in MDLs that are NOT bundled are resolved by Kit's
                    # MDL search paths and cannot safely be relocated (the MDL
                    # system ties module identity to filesystem location), so a
                    # bare or project-local reference is rewritten to the
                    # canonical bare/suffix form. A bundled copy of a
                    # built-in-named MDL (the authored path resolves to a real
                    # file) is instead kept and re-anchored like any other
                    # transferred asset, so the package stays self-contained.
                    if not bundled_on_disk and utils.is_builtin_mdl(old_path or old_resolved or ""):
                        canonical = utils.canonical_builtin_mdl_path(old_path or old_resolved or "")
                        if canonical and canonical != old_path:
                            attr_spec.default = Sdf.AssetPath(canonical)
                        continue

                    # Try matching by resolved path first
                    if old_resolved and old_resolved in by_resolved:
                        new_rel_path = by_resolved[old_resolved]
                    elif old_path and old_path in by_resolved:
                        new_rel_path = by_resolved[old_path]
                    else:
                        # Fall back to filename matching, then to absolute->relative normalization
                        filename = os.path.basename(old_path) if old_path else None
                        if filename and filename in by_filename:
                            new_rel_path = by_filename[filename]
                        else:
                            new_rel_path = normalize_unmapped(old_path)

                    if new_rel_path:
                        attr_spec.default = Sdf.AssetPath(new_rel_path)

            # Recurse into children
            for child_spec in prim_spec.nameChildren:
                update_spec_recursive(child_spec)

        material_spec = layer.GetPrimAtPath(material_path)
        if material_spec:
            update_spec_recursive(material_spec)

    def _collect_all_material_bindings(self, scope: str | None) -> list[MaterialBinding]:
        """Collect all material bindings in the stage.

        Args:
            scope: Optional root scope path to limit the search.

        Returns:
            List of MaterialBinding objects describing each binding.

        """
        bindings: list[MaterialBinding] = []
        root_prim = utils.get_scope_root(self.source_stage, scope or _DEFAULT_SCOPE)

        for prim in Usd.PrimRange(root_prim, Usd.TraverseInstanceProxies()):
            # Check for material bindings via API or relationships
            has_binding_api = prim.HasAPI(UsdShade.MaterialBindingAPI)
            has_binding_rel = any(r.GetName().startswith("material:binding") for r in prim.GetRelationships())

            if not has_binding_api and not has_binding_rel:
                continue

            binding_api = UsdShade.MaterialBindingAPI(prim)
            prim_path = prim.GetPath().pathString

            for purpose in _MATERIAL_PURPOSES:
                binding = binding_api.GetDirectBinding(materialPurpose=purpose)
                mat_path = binding.GetMaterialPath()
                if mat_path:
                    bindings.append(MaterialBinding(prim_path, mat_path.pathString, purpose))

        return bindings

    def _get_or_open_layer(
        self,
        layer_id: str,
        layer_cache: dict[str, Sdf.Layer],
    ) -> Sdf.Layer | None:
        """Get a layer from cache or open it.

        Args:
            layer_id: The layer identifier.
            layer_cache: Cache of opened layers (modified in place).

        Returns:
            The layer, or None if it could not be opened.

        """
        if layer_id in layer_cache:
            return layer_cache[layer_id]

        source_layer = self.source_stage.GetRootLayer()
        if layer_id == source_layer.identifier:
            layer_cache[layer_id] = source_layer
            return source_layer

        layer = Sdf.Layer.FindOrOpen(layer_id)
        if layer:
            layer_cache[layer_id] = layer
        return layer

    def _update_defining_layers(
        self,
        material_by_hash: dict[str, MaterialEntry],
        materials_layer_abs_path: str,
    ) -> None:
        """Update materials in their defining layers with instanceable references.

        Args:
            material_by_hash: Dictionary mapping material hashes to MaterialEntry objects.
            materials_layer_abs_path: Absolute path to the materials layer file.

        """
        source_layer = self.source_stage.GetRootLayer()
        layer_cache: dict[str, Sdf.Layer] = {}
        processed_paths: set[str] = set()
        processed_count = 0

        with Sdf.ChangeBlock():
            for entry in material_by_hash.values():
                for source in entry.sources:
                    if source.prim_path in processed_paths:
                        continue
                    processed_paths.add(source.prim_path)

                    target_layer = self._get_or_open_layer(source.defining_layer_id, layer_cache)
                    if not target_layer:
                        self.log_operation(f"Could not open layer {source.defining_layer_id}")
                        continue

                    # Get or create prim spec
                    prim_spec = target_layer.GetPrimAtPath(source.defining_prim_path)
                    if not prim_spec:
                        utils.ensure_prim_hierarchy(target_layer, source.defining_prim_path)
                        prim_spec = utils.create_prim_spec(
                            target_layer, source.defining_prim_path, type_name=_MATERIAL_TYPE_NAME
                        )

                    if prim_spec:
                        # Clear existing content
                        utils.clear_composition_arcs(prim_spec, make_explicit=True)
                        for child_name in [c.name for c in prim_spec.nameChildren]:
                            del prim_spec.nameChildren[child_name]
                        for prop_name in [p.name for p in prim_spec.properties]:
                            del prim_spec.properties[prop_name]

                        # Add instanceable reference to materials layer
                        prim_spec.referenceList.ClearEdits()
                        rel_path = utils.get_relative_layer_path(target_layer, materials_layer_abs_path)
                        prim_spec.referenceList.Prepend(Sdf.Reference(rel_path, entry.material_layer_path))
                        prim_spec.instanceable = True
                        processed_count += 1

        # Save modified layers (except source layer)
        for layer_id, layer in layer_cache.items():
            if layer != source_layer:
                layer.Save()
                self.add_affected_stage(layer_id)
                self.log_operation(f"Saved modified layer: {layer_id}")

        self.log_operation(f"Updated {processed_count} materials in their defining layers")

    def _ensure_material_binding_api(self, all_bindings: list[MaterialBinding]) -> None:
        """Ensure MaterialBindingAPI is applied to all prims with material bindings.

        Args:
            all_bindings: List of all material bindings in the stage.

        """
        source_layer = self.source_stage.GetRootLayer()
        applied_count = 0

        # Get unique prim paths with bindings
        prim_paths_with_bindings: set[str] = {binding.prim_path for binding in all_bindings}

        with Sdf.ChangeBlock():
            for prim_path in prim_paths_with_bindings:
                prim = self.source_stage.GetPrimAtPath(prim_path)
                if not prim.IsValid():
                    continue

                # Check if MaterialBindingAPI is already applied
                if prim.HasAPI(UsdShade.MaterialBindingAPI):
                    continue

                # Apply MaterialBindingAPI
                prim_spec = source_layer.GetPrimAtPath(prim_path)
                if not prim_spec:
                    prim_spec = Sdf.CreatePrimInLayer(source_layer, prim_path)
                    if prim_spec:
                        prim_spec.specifier = Sdf.SpecifierOver

                if prim_spec:
                    # Add MaterialBindingAPI to applied schemas
                    existing_schemas = prim_spec.GetInfo("apiSchemas")
                    if existing_schemas:
                        schema_list = list(existing_schemas.GetAppliedItems())
                    else:
                        schema_list = []

                    if "MaterialBindingAPI" not in schema_list:
                        schema_list.append("MaterialBindingAPI")
                        prim_spec.SetInfo("apiSchemas", Sdf.TokenListOp.CreateExplicit(schema_list))
                        applied_count += 1

        if applied_count > 0:
            self.log_operation(f"Applied MaterialBindingAPI to {applied_count} prims")

    def _cleanup_unused_material_references(
        self,
        material_by_hash: dict[str, MaterialEntry],
        all_bindings: list[MaterialBinding],
    ) -> None:
        """Remove instanceable references for materials that are not bound to any surface.

        Args:
            material_by_hash: Dictionary mapping material hashes to MaterialEntry objects.
            all_bindings: List of all material bindings in the stage.

        """
        source_layer = self.source_stage.GetRootLayer()
        bound_material_paths: set[str] = {b.material_path for b in all_bindings}
        layer_cache: dict[str, Sdf.Layer] = {}
        removed_count = 0

        with Sdf.ChangeBlock():
            for entry in material_by_hash.values():
                for source in entry.sources:
                    if source.prim_path in bound_material_paths:
                        continue

                    target_layer = self._get_or_open_layer(source.defining_layer_id, layer_cache)
                    if not target_layer:
                        continue

                    prim_spec = target_layer.GetPrimAtPath(source.defining_prim_path)
                    if prim_spec:
                        parent_spec = target_layer.GetPrimAtPath(Sdf.Path(source.defining_prim_path).GetParentPath())
                        if parent_spec and prim_spec.name in parent_spec.nameChildren:
                            del parent_spec.nameChildren[prim_spec.name]
                            removed_count += 1

        for layer_id, layer in layer_cache.items():
            if layer != source_layer:
                layer.Save()
                self.add_affected_stage(layer_id)

        if removed_count > 0:
            self.log_operation(f"Removed {removed_count} unused material references")
