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

"""Common utility functions for asset transformer rules."""

from __future__ import annotations

import logging
import os
import re
import shutil
from collections.abc import Callable

from isaacsim.asset.transformer import RuleConfigurationParam
from isaacsim.asset.transformer.utils import make_explicit_relative
from pxr import Sdf, Usd

_LOGGER = logging.getLogger(__name__)

# USD file extensions (including zipped)
USD_EXTENSIONS: frozenset[str] = frozenset({".usd", ".usda", ".usdc", ".usdz"})

# Token prefixes that mark a Kit-shipped MDL in the form returned by
# ``omni.kit.material.library.get_mdl_list_async(expand_paths=False)``. MDLs
# from custom ``lib_paths`` come back as plain absolute paths and are
# excluded; only entries living under the Kit install (or active app) tree
# are considered "built-in".
_KIT_TOKEN_PREFIXES: tuple[str, ...] = ("${kit}", "${app}")

# Hardcoded fallback set of Kit-shipped MDL basenames (lowercase). Used
# whenever live discovery via ``omni.kit.material.library`` is unavailable
# (e.g. when the asset transformer rules are imported in a non-Kit Python
# environment, when the extension is not enabled, or when the API call
# fails). Keep this in sync with the names the runtime actually ships.
_FALLBACK_BUILTIN_MDL_BASENAMES: frozenset[str] = frozenset(
    {
        "omnipbr.mdl",
        "omnipbr_base.mdl",
        "omnipbr_clearcoat.mdl",
        "omniglass.mdl",
        "omniemissive.mdl",
        "omnisurface.mdl",
        "omnisurfacebase.mdl",
        "omnisurfaceblend.mdl",
        "omnisurfacelite.mdl",
        "omnisurfacelitebase.mdl",
        "omnihair.mdl",
        "omnihairbase.mdl",
        "usdpreviewsurface.mdl",
        "omnisurfacepresets.mdl",
        "core_definitions.mdl",
    }
)

# Hardcoded fallback set of multi-component MDL reference suffixes
# (lowercase, forward-slash). USD authors sometimes use these prefixed
# forms instead of the bare filename when the MDL package's directory
# disambiguates same-named modules.
_FALLBACK_BUILTIN_MDL_SUFFIXES: frozenset[str] = frozenset({"nvidia/core_definitions.mdl"})

# Live cache of Kit-shipped MDL basenames (lowercase). Initialized to the
# hardcoded fallback at import time and upgraded by
# :func:`refresh_builtin_mdl_cache_async` when ``omni.kit.material.library``
# is available.
_BUILTIN_MDL_BASENAMES: frozenset[str] = _FALLBACK_BUILTIN_MDL_BASENAMES

# Live cache of multi-component MDL reference suffixes. Same lifecycle as
# ``_BUILTIN_MDL_BASENAMES``.
_BUILTIN_MDL_SUFFIXES: frozenset[str] = _FALLBACK_BUILTIN_MDL_SUFFIXES


async def refresh_builtin_mdl_cache_async() -> bool:
    """Best-effort upgrade of the built-in MDL cache from the live Kit runtime.

    Queries :func:`omni.kit.material.library.get_mdl_list_async` with
    ``expand_paths=False`` so Kit-shipped entries are returned in their token
    form (``${kit}/...`` or ``${app}/...``); entries from custom user
    ``lib_paths`` come back as plain absolute paths and are filtered out --
    only the token-prefixed entries are recorded. For every kept entry the
    basename and (when present) the last-two-component suffix are added to
    the cache, so authored references like ``OmniPBR.mdl`` and
    ``nvidia/core_definitions.mdl`` both classify as built-in.

    If ``omni.kit.material.library`` is not importable, the API call fails,
    or the filtered list is empty, the cache is left at the hardcoded
    fallback (see ``_FALLBACK_BUILTIN_MDL_BASENAMES`` /
    ``_FALLBACK_BUILTIN_MDL_SUFFIXES``) and a warning is logged. The function
    never raises.

    The hardcoded fallback is also what the cache contains immediately after
    module import, so callers can use :func:`is_builtin_mdl` and
    :func:`canonical_builtin_mdl_path` without awaiting this refresh first.

    Returns:
        ``True`` if the live runtime data was successfully merged into the
        cache, ``False`` if the fallback is in effect.

    Example:

    .. code-block:: python

        from isaacsim.asset.transformer.rules import utils

        await utils.refresh_builtin_mdl_cache_async()

    """
    global _BUILTIN_MDL_BASENAMES, _BUILTIN_MDL_SUFFIXES

    try:
        import omni.kit.material.library as material_library
    except ImportError:
        _LOGGER.warning(
            "omni.kit.material.library is not available; " "using hardcoded built-in MDL list (%d basename(s)).",
            len(_FALLBACK_BUILTIN_MDL_BASENAMES),
        )
        return False

    try:
        mdl_list = await material_library.get_mdl_list_async(
            use_hidden=True,
            wait_for_ready=True,
            get_private=True,
            expand_paths=False,
        )
    except Exception:
        _LOGGER.exception(
            "omni.kit.material.library.get_mdl_list_async failed; "
            "using hardcoded built-in MDL list (%d basename(s)).",
            len(_FALLBACK_BUILTIN_MDL_BASENAMES),
        )
        return False

    basenames: set[str] = set()
    suffixes: set[str] = set()

    for entry in mdl_list:
        # Kit's API returns either a string path or a tuple/dataclass; the
        # path is always the last element when iterable, but a string is the
        # documented common form. Normalize defensively.
        token_path: str | None
        if isinstance(entry, str):
            token_path = entry
        else:
            token_path = next((str(part) for part in reversed(entry) if isinstance(part, str)), None)
        if not token_path:
            continue
        if not token_path.startswith(_KIT_TOKEN_PREFIXES):
            continue

        normalized = token_path.replace("\\", "/").lower()
        parts = [p for p in normalized.split("/") if p]
        if not parts:
            continue

        basenames.add(parts[-1])
        if len(parts) >= 2:
            suffixes.add(f"{parts[-2]}/{parts[-1]}")

    if not basenames:
        _LOGGER.warning(
            "omni.kit.material.library.get_mdl_list_async returned no ${kit}/${app} entries; "
            "using hardcoded built-in MDL list (%d basename(s)).",
            len(_FALLBACK_BUILTIN_MDL_BASENAMES),
        )
        return False

    # Union the discovered set with the hardcoded fallback so any MDL the
    # team has historically shipped continues to classify as built-in even
    # if a future Kit removes it from the live enumeration.
    _BUILTIN_MDL_BASENAMES = frozenset(basenames | _FALLBACK_BUILTIN_MDL_BASENAMES)
    _BUILTIN_MDL_SUFFIXES = frozenset(suffixes | _FALLBACK_BUILTIN_MDL_SUFFIXES)
    _LOGGER.info(
        "Discovered %d Kit-shipped MDL basename(s) and %d suffix form(s) "
        "(merged with %d hardcoded fallback basename(s)).",
        len(basenames),
        len(suffixes),
        len(_FALLBACK_BUILTIN_MDL_BASENAMES),
    )
    return True


def is_builtin_mdl(path: str) -> bool:
    """Check whether *path* refers to a built-in MDL file shipped with the runtime.

    The classification consults a cache populated from
    :func:`omni.kit.material.library.get_mdl_list_async` (filtered to entries
    living under Kit's install via ``${kit}`` / ``${app}`` token paths) when
    that extension is available, and from a hardcoded fallback list otherwise.
    The check itself is name-based and case-insensitive: any path whose
    basename matches a known Kit MDL, or whose tail matches a known
    multi-component reference form (e.g. ``nvidia/core_definitions.mdl``), is
    considered built-in. This catches bare authored references
    (``OmniPBR.mdl``) and project-local copies (``C:/Dev/.../OmniPBR.mdl``)
    alike -- copies cannot safely be relocated because Kit's MDL system ties
    module identity to filesystem location and relies on its built-in search
    paths to find these modules.

    See :func:`canonical_builtin_mdl_path` for converting a built-in match to
    the canonical Kit-resolvable form, and
    :func:`refresh_builtin_mdl_cache_async` for upgrading the cache from the
    live Kit runtime.

    Args:
        path: File-system or asset path (absolute or relative).

    Returns:
        ``True`` when the basename matches a known built-in MDL, or when the
        path ends with a known built-in suffix form.

    """
    if not path:
        return False
    path_lower = path.lower().replace("\\", "/")
    if os.path.basename(path_lower) in _BUILTIN_MDL_BASENAMES:
        return True
    return any(path_lower.endswith(suffix) for suffix in _BUILTIN_MDL_SUFFIXES)


def canonical_builtin_mdl_path(path: str) -> str | None:
    """Return the canonical Kit-resolvable form for a built-in MDL path.

    Kit resolves built-in MDLs through its MDL search paths using either the
    bare filename (``OmniPBR.mdl``) or a known prefix form
    (``nvidia/core_definitions.mdl``). When a USD authors an absolute or
    explicit-relative path to a project-local copy of a built-in (e.g.
    ``C:/Dev/.../OmniPBR.mdl``), the path must be rewritten to its canonical
    form so the resulting package is portable: the file itself must NOT be
    transferred because Kit's MDL system ties module identity to filesystem
    location and the built-in search paths are the only safe resolver.

    Args:
        path: File-system or asset path (absolute or relative).

    Returns:
        The canonical asset-path string Kit will resolve from its built-in
        search paths, with original casing preserved. ``None`` if *path* is
        not a known built-in.

    """
    if not is_builtin_mdl(path):
        return None
    normalized = path.replace("\\", "/")
    normalized_lower = normalized.lower()
    # Suffix-prefixed form (e.g. "nvidia/core_definitions.mdl") takes
    # precedence: preserve the prefix so Kit's resolver finds the right module.
    for suffix in _BUILTIN_MDL_SUFFIXES:
        if normalized_lower.endswith(suffix):
            return normalized[-len(suffix) :]
    # Basename form -- the file is reachable purely by name from Kit's
    # built-in MDL search paths.
    return os.path.basename(normalized)


def norm_path(path: str) -> str:
    """Normalize a path for cross-platform comparison.

    Uses ``normpath`` for separator and dot normalization, then
    ``normcase`` so that look-ups in dicts keyed by path work on
    case-insensitive file systems (Windows).  On Linux ``normcase``
    is a no-op, so behaviour is unchanged.

    Args:
        path: The file-system path to normalize.

    Returns:
        Normalized path string suitable for dict keys and comparisons.

    """
    return os.path.normcase(os.path.normpath(path))


# Schema metadata keys to remove from copied attributes
SCHEMA_METADATA_TO_REMOVE: tuple[str, ...] = ("allowedTokens", "doc", "displayName")

# Keys to skip when copying prim metadata (composition arcs handled separately)
COMPOSITION_SKIP_KEYS: frozenset = frozenset(
    {
        "specifier",
        "typeName",
        "apiSchemas",
        "references",
        "payload",
        "payloads",
        "inherits",
        "specializes",
        "variantSets",
        "variantSelection",
    }
)


# =============================================================================
# Shared Configuration Parameters
# =============================================================================


def make_scope_param(description: str = "Root path to search (default: '/')") -> RuleConfigurationParam:
    """Create a scope configuration parameter.

    Args:
        description: Custom description for the parameter.

    Returns:
        RuleConfigurationParam for scope.

    Example:

    .. code-block:: python

        scope_param = make_scope_param()

    """
    return RuleConfigurationParam(
        name="scope",
        display_name="Scope",
        param_type=str,
        description=description,
        default_value="/",
    )


def make_stage_name_param(
    default_value: str = "output.usda",
    description: str = "Name of the output USD file",
) -> RuleConfigurationParam:
    """Create a stage_name configuration parameter.

    Args:
        default_value: Default filename for the output stage.
        description: Custom description for the parameter.

    Returns:
        RuleConfigurationParam for stage_name.

    Example:

    .. code-block:: python

        stage_param = make_stage_name_param(default_value="out.usda")

    """
    return RuleConfigurationParam(
        name="stage_name",
        display_name="Stage Name",
        param_type=str,
        description=description,
        default_value=default_value,
    )


def make_prim_names_param(
    description: str = "List of regex patterns to match prim names (e.g. 'Body.*')",
) -> RuleConfigurationParam:
    """Create a prim_names configuration parameter.

    Args:
        description: Custom description for the parameter.

    Returns:
        RuleConfigurationParam for prim_names.

    Example:

    .. code-block:: python

        prim_names_param = make_prim_names_param()

    """
    return RuleConfigurationParam(
        name="prim_names",
        display_name="Prim Names",
        param_type=list,
        description=description,
        default_value=[".*"],
    )


def make_ignore_prim_names_param(
    description: str = "List of regex patterns to match prim names to ignore",
) -> RuleConfigurationParam:
    """Create an ignore_prim_names configuration parameter.

    Args:
        description: Custom description for the parameter.

    Returns:
        RuleConfigurationParam for ignore_prim_names.

    Example:

    .. code-block:: python

        ignore_param = make_ignore_prim_names_param()

    """
    return RuleConfigurationParam(
        name="ignore_prim_names",
        display_name="Ignore Prim Names",
        param_type=list,
        description=description,
        default_value=None,
    )


# =============================================================================
# Layer and Prim Utilities
# =============================================================================


def find_or_create_layer(
    path: str,
    copy_metadata_from: Usd.Stage | None = None,
) -> Sdf.Layer | None:
    """Find an existing layer or create a new one at the given path.

    Args:
        path: Absolute path to the layer file.
        copy_metadata_from: Optional stage to copy metadata from (metersPerUnit, upAxis, etc.).

    Returns:
        The layer, or None if creation failed.

    Example:

    .. code-block:: python

        layer = find_or_create_layer("/tmp/out.usda", stage)

    """
    os.makedirs(os.path.dirname(path), exist_ok=True)
    layer = Sdf.Layer.FindOrOpen(path)
    if not layer:
        layer = Sdf.Layer.CreateNew(path)
        if layer and copy_metadata_from:
            copy_stage_metadata(copy_metadata_from, layer)
    return layer


def ensure_prim_spec_in_layer(
    layer: Sdf.Layer,
    path: Sdf.Path,
    specifier: Sdf.Specifier = Sdf.SpecifierOver,
) -> Sdf.PrimSpec | None:
    """Ensure a prim spec exists in a layer, creating it if necessary.

    Args:
        layer: The layer to create the prim spec in.
        path: Path to the prim.
        specifier: Specifier type for the prim (default: Over).

    Returns:
        The existing or newly created prim spec, or None if creation failed.

    Example:

    .. code-block:: python

        prim_spec = ensure_prim_spec_in_layer(layer, Sdf.Path("/World"))

    """
    prim_spec = layer.GetPrimAtPath(path)
    if prim_spec:
        return prim_spec
    prim_spec = Sdf.CreatePrimInLayer(layer, path)
    if prim_spec:
        prim_spec.specifier = specifier
    return prim_spec


def get_scope_root(stage: Usd.Stage, scope: str, fallback_to_pseudo_root: bool = True) -> Usd.Prim | None:
    """Get the root prim for a given scope path.

    Args:
        stage: The stage to search in.
        scope: Scope path to search under (e.g., "/" or "/World").
        fallback_to_pseudo_root: If True and scope is invalid, return pseudo root.

    Returns:
        The scope prim, or None/pseudo root if invalid.

    Example:

    .. code-block:: python

        scope_prim = get_scope_root(stage, "/World")

    """
    # "/" is always valid and maps to pseudo root
    if not scope or scope == "/":
        return stage.GetPseudoRoot()

    # Try to get the specific scope prim
    scope_prim = stage.GetPrimAtPath(scope)
    if scope_prim and scope_prim.IsValid():
        return scope_prim
    if fallback_to_pseudo_root:
        return stage.GetPseudoRoot()
    return None


def get_default_prim_path(stage: Usd.Stage, fallback: str = "/World") -> str:
    """Get the default prim path from a stage.

    Args:
        stage: The stage to query.
        fallback: Fallback path if no default prim exists.

    Returns:
        The default prim path string.

    Example:

    .. code-block:: python

        path = get_default_prim_path(stage)

    """
    default_prim = stage.GetDefaultPrim()
    if default_prim and default_prim.IsValid():
        return default_prim.GetPath().pathString
    return fallback


def compile_patterns(
    patterns: list[str] | None,
    logger: Callable[[str], None] | None = None,
) -> list[re.Pattern[str]]:
    """Compile a list of regex pattern strings into compiled pattern objects.

    Invalid patterns are silently skipped (with an optional log callback).

    Args:
        patterns: List of regex pattern strings. ``None`` or empty list returns ``[]``.
        logger: Optional callback invoked with a message when a pattern is invalid.

    Returns:
        List of compiled regex patterns.

    Example:

    .. code-block:: python

        compiled = compile_patterns(["Physics.*", "Newton.*"])

    """
    if not patterns:
        return []
    compiled: list[re.Pattern[str]] = []
    for pattern in patterns:
        if not pattern:
            continue
        try:
            compiled.append(re.compile(pattern))
        except re.error as exc:
            if logger:
                logger(f"Invalid regex pattern '{pattern}': {exc}, skipping")
    return compiled


def matches_any_pattern(value: str, compiled_patterns: list[re.Pattern[str]]) -> bool:
    """Check if a string fully matches any of the compiled regex patterns.

    Uses ``re.fullmatch`` so the entire string must match, consistent with
    how glob/fnmatch patterns were historically interpreted.

    Args:
        value: The string to test.
        compiled_patterns: Compiled regex patterns from :func:`compile_patterns`.

    Returns:
        True if *value* fully matches at least one pattern.

    Example:

    .. code-block:: python

        patterns = compile_patterns(["Physics.*"])
        matches_any_pattern("PhysicsRigidBodyAPI", patterns)  # True

    """
    return any(p.fullmatch(value) for p in compiled_patterns)


def matches_prim_filter(
    prim_name: str,
    include_patterns: list[str],
    exclude_patterns: list[str] | None = None,
) -> bool:
    """Check if a prim name matches include patterns and doesn't match exclude patterns.

    Args:
        prim_name: The prim name to check.
        include_patterns: List of regex patterns that must match (at least one).
        exclude_patterns: Optional list of regex patterns that must not match.

    Returns:
        True if prim name passes the filter.

    Example:

    .. code-block:: python

        matches = matches_prim_filter("Body", ["Body.*"], ["BodyIgnore.*"])

    """
    compiled_include = compile_patterns(include_patterns)
    if not compiled_include:
        return False

    if not matches_any_pattern(prim_name, compiled_include):
        return False

    if exclude_patterns:
        compiled_exclude = compile_patterns(exclude_patterns)
        if matches_any_pattern(prim_name, compiled_exclude):
            return False

    return True


def is_remote_path(path: str) -> bool:
    """Check if a path is a remote URL (Nucleus, HTTP, etc.).

    Args:
        path: The path to check.

    Returns:
        True if the path is a remote URL.

    Example:

    .. code-block:: python

        is_remote = is_remote_path("omniverse://server/asset.usd")

    """
    if not path:
        return False
    return path.startswith(("omniverse://", "http://", "https://"))


def is_usd_file(path: str) -> bool:
    """Check if a path has a USD file extension.

    Args:
        path: The file path to check.

    Returns:
        True if the path has a USD extension.

    Example:

    .. code-block:: python

        is_usd = is_usd_file("asset.usda")

    """
    if not path:
        return False
    ext = os.path.splitext(path)[1].lower()
    return ext in USD_EXTENSIONS


def clear_composition_arcs(prim_spec: Sdf.PrimSpec, make_explicit: bool = False) -> None:
    """Clear all composition arcs from a prim spec if they are authored.

    Args:
        prim_spec: The prim spec to clear arcs from.
        make_explicit: If True, use ClearEditsAndMakeExplicit to override sublayer arcs.

    Example:

    .. code-block:: python

        clear_composition_arcs(prim_spec)

    """
    arc_checks_and_lists = [
        (prim_spec.hasReferences, prim_spec.referenceList),
        (prim_spec.hasPayloads, prim_spec.payloadList),
        (prim_spec.hasInheritPaths, prim_spec.inheritPathList),
        (prim_spec.hasSpecializes, prim_spec.specializesList),
    ]
    for has_arcs, arc_list in arc_checks_and_lists:
        if has_arcs:
            if make_explicit:
                arc_list.ClearEditsAndMakeExplicit()
            else:
                arc_list.ClearEdits()


def create_prim_spec(
    layer: Sdf.Layer,
    path: str,
    specifier: Sdf.Specifier = Sdf.SpecifierDef,
    type_name: str = "",
    instanceable: bool = False,
) -> Sdf.PrimSpec | None:
    """Create a prim spec with common settings.

    Args:
        layer: The layer to create the prim in.
        path: The prim path.
        specifier: The specifier (Def, Over, Class).
        type_name: The prim type name.
        instanceable: Whether the prim should be instanceable.

    Returns:
        The created prim spec or None if creation failed.

    Example:

    .. code-block:: python

        prim_spec = create_prim_spec(layer, "/World", type_name="Xform")

    """
    prim_spec = Sdf.CreatePrimInLayer(layer, path)
    if prim_spec:
        prim_spec.specifier = specifier
        if type_name:
            prim_spec.typeName = type_name
        if instanceable:
            prim_spec.instanceable = True
    return prim_spec


def get_relative_layer_path(from_layer: Sdf.Layer, to_layer_path: str) -> str:
    """Compute relative path from one layer to another.

    The result always starts with ``./`` or ``../`` so that USD tooling
    treats it as a relative path rather than a search-path identifier.

    Args:
        from_layer: The layer to compute the path from.
        to_layer_path: Absolute path to the target layer.

    Returns:
        Relative path string with explicit ``./`` or ``../`` prefix.

    Example:

    .. code-block:: python

        rel_path = get_relative_layer_path(layer, "/tmp/other.usda")

    """
    from_dir = os.path.dirname(from_layer.identifier)
    return make_explicit_relative(os.path.relpath(to_layer_path, from_dir))


def clear_instanceable_recursive(prim_spec: Sdf.PrimSpec) -> None:
    """Recursively clear the instanceable flag on a prim spec and all its children.

    Args:
        prim_spec: The prim spec to clear instanceable on.

    Example:

    .. code-block:: python

        clear_instanceable_recursive(prim_spec)

    """
    if prim_spec.instanceable:
        prim_spec.instanceable = False
    for child_spec in prim_spec.nameChildren:
        clear_instanceable_recursive(child_spec)


def clean_schema_metadata(prim_spec: Sdf.PrimSpec) -> None:
    """Clean up schema-level metadata from attributes.

    These are copied by CopySpec but shouldn't be authored on instances.

    Args:
        prim_spec: The prim spec to clean metadata from.

    Example:

    .. code-block:: python

        clean_schema_metadata(prim_spec)

    """
    for attr_name in list(prim_spec.attributes.keys()):  # noqa: SIM118
        attr_spec = prim_spec.attributes[attr_name]
        for meta_key in SCHEMA_METADATA_TO_REMOVE:
            if attr_spec.HasInfo(meta_key):
                attr_spec.ClearInfo(meta_key)


def find_ancestor_matching(
    prim: Usd.Prim,
    predicate: Callable[[Usd.Prim], bool],
) -> Usd.Prim | None:
    """Find the first ancestor prim matching a predicate.

    Args:
        prim: The prim to start from (not included in search).
        predicate: Callable that takes a prim and returns True if it matches.

    Returns:
        The first matching ancestor prim, or None if not found.

    Example:

    .. code-block:: python

        ancestor = find_ancestor_matching(prim, lambda p: p.GetName() == "Root")

    """
    ancestor = prim.GetParent()
    while ancestor and ancestor.IsValid():
        if predicate(ancestor):
            return ancestor
        ancestor = ancestor.GetParent()
    return None


def merge_token_list_op(existing_list_op: Sdf.TokenListOp | None, new_items: list) -> Sdf.TokenListOp:
    """Merge new items into an existing TokenListOp by prepending.

    For non-delete operations, new items are prepended to existing items.
    For delete operations, both lists are combined.

    Args:
        existing_list_op: Existing list op from destination layer (may be None).
        new_items: New items from source prim to merge.

    Returns:
        Merged TokenListOp with new items prepended and deletes combined.

    Example:

    .. code-block:: python

        merged = merge_token_list_op(existing_list_op, ["PhysicsAPI"])

    """
    if (
        existing_list_op is None
        or existing_list_op.isExplicit is False
        and not any(
            [
                existing_list_op.prependedItems,
                existing_list_op.appendedItems,
                existing_list_op.deletedItems,
                existing_list_op.explicitItems,
            ]
        )
    ):
        return Sdf.TokenListOp.CreateExplicit(list(new_items))

    existing_explicit = list(existing_list_op.explicitItems) if existing_list_op.isExplicit else []
    existing_prepended = list(existing_list_op.prependedItems) if existing_list_op.prependedItems else []
    existing_appended = list(existing_list_op.appendedItems) if existing_list_op.appendedItems else []
    existing_deleted = list(existing_list_op.deletedItems) if existing_list_op.deletedItems else []

    new_items_list = list(new_items)

    if existing_explicit:
        merged_explicit = new_items_list + [item for item in existing_explicit if item not in new_items_list]
        return Sdf.TokenListOp.CreateExplicit(merged_explicit)

    result = Sdf.TokenListOp()
    merged_prepended = new_items_list + [item for item in existing_prepended if item not in new_items_list]
    if merged_prepended:
        result.prependedItems = merged_prepended
    if existing_appended:
        result.appendedItems = existing_appended
    if existing_deleted:
        result.deletedItems = existing_deleted

    return result


def copy_prim_from_composed_stage(
    src_prim: Usd.Prim,
    dst_layer: Sdf.Layer,
    dst_path: str,
    remap_connections_from: str | None = None,
    remap_connections_to: str | None = None,
) -> bool:
    """Copy prim content from composed stage to a layer, handling instance proxies.

    This function reads from the composed stage which can traverse instance proxies,
    unlike Sdf.CopySpec which only works with layer specs. It manually creates
    all prim specs, attributes, relationships, and children.

    Args:
        src_prim: The source prim from the composed stage.
        dst_layer: The destination layer to copy to.
        dst_path: The destination path in the layer.
        remap_connections_from: Optional path prefix to remap connections from.
        remap_connections_to: Optional path prefix to remap connections to.

    Returns:
        True if the copy succeeded, False otherwise.

    Example:

    .. code-block:: python

        success = copy_prim_from_composed_stage(src_prim, layer, "/World/Prim")

    """
    if not src_prim.IsValid():
        return False

    # Create the prim spec
    prim_spec = Sdf.CreatePrimInLayer(dst_layer, dst_path)
    if not prim_spec:
        return False

    prim_spec.specifier = Sdf.SpecifierDef
    prim_spec.typeName = src_prim.GetTypeName()

    # Copy applied API schemas
    applied_schemas = src_prim.GetAppliedSchemas()
    if applied_schemas:
        prim_spec.SetInfo("apiSchemas", Sdf.TokenListOp.CreateExplicit(list(applied_schemas)))

    # Copy all authored attributes
    for attr in src_prim.GetAttributes():
        attr_name = attr.GetName()
        # Include attributes that have authored values, connections, or are output ports
        is_output = attr_name.startswith("outputs:")
        if not attr.HasAuthoredValue() and not attr.GetConnections() and not is_output:
            continue

        try:
            attr_spec = Sdf.AttributeSpec(prim_spec, attr_name, attr.GetTypeName())
            if not attr_spec:
                continue

            # Copy value
            if attr.HasAuthoredValue():
                value = attr.Get()
                if value is not None:
                    attr_spec.default = value

            # Copy variability
            if attr.GetVariability() == Sdf.VariabilityUniform:
                attr_spec.variability = Sdf.VariabilityUniform

            # Copy connections with optional path remapping
            connections = attr.GetConnections()
            if connections:
                for conn in connections:
                    conn_path = conn.pathString
                    if remap_connections_from and remap_connections_to:
                        if conn_path.startswith(remap_connections_from):
                            conn_path = conn_path.replace(remap_connections_from, remap_connections_to, 1)
                    attr_spec.connectionPathList.Prepend(Sdf.Path(conn_path))

            # Copy relevant metadata (skip schema-level metadata)
            for key in attr.GetAllMetadata():
                if key in ("typeName", "default", "variability", "connectionPaths"):
                    continue
                if key in SCHEMA_METADATA_TO_REMOVE:
                    continue
                try:
                    metadata_value = attr.GetMetadata(key)
                    if metadata_value is not None:
                        attr_spec.SetInfo(key, metadata_value)
                except Exception:
                    pass

        except Exception:
            pass

    # Copy relationships
    for rel in src_prim.GetRelationships():
        if not rel.HasAuthoredTargets():
            continue

        try:
            rel_spec = Sdf.RelationshipSpec(prim_spec, rel.GetName())
            if rel_spec:
                for target in rel.GetTargets():
                    target_path = target.pathString
                    if remap_connections_from and remap_connections_to:
                        if target_path.startswith(remap_connections_from):
                            target_path = target_path.replace(remap_connections_from, remap_connections_to, 1)
                    rel_spec.targetPathList.Prepend(Sdf.Path(target_path))
        except Exception:
            pass

    # Recursively copy children (using TraverseInstanceProxies to access instance proxy children)
    for child in src_prim.GetFilteredChildren(Usd.TraverseInstanceProxies()):
        child_name = child.GetName()
        child_dst_path = f"{dst_path}/{child_name}"
        copy_prim_from_composed_stage(child, dst_layer, child_dst_path, remap_connections_from, remap_connections_to)

    return True


def copy_composed_prim_to_layer(
    src_prim: Usd.Prim,
    dst_layer: Sdf.Layer,
    dst_path: Sdf.Path,
    merge_existing: bool = True,
) -> bool:
    """Copy a composed prim from a stage to a layer, optionally merging with existing specs.

    This function reads composed values from the stage and writes them to the destination
    layer. If merge_existing is True and the destination already has a spec, properties
    and metadata are merged with source values taking precedence.

    Args:
        src_prim: Source prim from the composed stage.
        dst_layer: Destination layer to copy to.
        dst_path: Destination path in the layer.
        merge_existing: If True, merge with existing specs; otherwise overwrite.

    Returns:
        True if the copy succeeded, False otherwise.

    Example:

    .. code-block:: python

        success = copy_composed_prim_to_layer(src_prim, layer, Sdf.Path("/World/Prim"))

    """
    if not src_prim.IsValid():
        return False

    existing_spec = dst_layer.GetPrimAtPath(dst_path) if merge_existing else None
    existing_attrs = {}
    existing_rels = {}
    existing_variant_selections = {}

    if existing_spec:
        for attr_name in existing_spec.attributes.keys():  # noqa: SIM118
            attr_spec = existing_spec.attributes[attr_name]
            existing_attrs[attr_name] = {
                "type_name": attr_spec.typeName,
                "default": attr_spec.default if attr_spec.HasInfo("default") else None,
                "variability": attr_spec.variability,
                "connections": list(attr_spec.connectionPathList.GetAddedOrExplicitItems()),
            }
        for rel_name in existing_spec.relationships.keys():  # noqa: SIM118
            rel_spec = existing_spec.relationships[rel_name]
            existing_rels[rel_name] = list(rel_spec.targetPathList.GetAddedOrExplicitItems())
        existing_variant_selections = dict(existing_spec.variantSelections)

    prim_spec = Sdf.CreatePrimInLayer(dst_layer, dst_path)
    if not prim_spec:
        return False

    prim_spec.specifier = Sdf.SpecifierDef
    type_name = src_prim.GetTypeName()
    if type_name:
        prim_spec.typeName = type_name

    # Handle API schemas
    applied_schemas = src_prim.GetAppliedSchemas()
    if applied_schemas:
        if merge_existing and existing_spec and existing_spec.HasInfo("apiSchemas"):
            existing_api_schemas = existing_spec.GetInfo("apiSchemas")
            merged_schemas = merge_token_list_op(existing_api_schemas, applied_schemas)
            prim_spec.SetInfo("apiSchemas", merged_schemas)
        else:
            prim_spec.SetInfo("apiSchemas", Sdf.TokenListOp.CreateExplicit(list(applied_schemas)))

    # Copy attribute specs from prim stack (strongest opinions)
    for spec in src_prim.GetPrimStack():
        for attr_name in spec.attributes.keys():  # noqa: SIM118
            src_prop_path = spec.path.AppendProperty(attr_name)
            dst_prop_path = dst_path.AppendProperty(attr_name)
            if not dst_layer.GetPropertyAtPath(dst_prop_path):
                Sdf.CopySpec(spec.layer, src_prop_path, dst_layer, dst_prop_path)

    # Handle composed attribute values
    for attr in src_prim.GetAttributes():
        attr_name = attr.GetName()
        is_output = attr_name.startswith("outputs:")
        if not attr.HasAuthoredValue() and not attr.GetConnections() and not is_output:
            continue

        try:
            attr_spec = prim_spec.attributes.get(attr_name)
            if not attr_spec:
                attr_spec = Sdf.AttributeSpec(prim_spec, attr_name, attr.GetTypeName())
                if not attr_spec:
                    continue
                for meta_key in SCHEMA_METADATA_TO_REMOVE:
                    if attr_spec.HasInfo(meta_key):
                        attr_spec.ClearInfo(meta_key)

            value = attr.Get()
            if value is not None:
                attr_spec.default = value

            if attr.GetVariability() == Sdf.VariabilityUniform:
                attr_spec.variability = Sdf.VariabilityUniform

            connections = attr.GetConnections()
            if connections:
                if merge_existing and attr_name in existing_attrs and existing_attrs[attr_name]["connections"]:
                    attr_spec.connectionPathList.ClearEditsAndMakeExplicit()
                    for conn in connections:
                        attr_spec.connectionPathList.Prepend(conn)
                    for conn in existing_attrs[attr_name]["connections"]:
                        if str(conn) not in [str(c) for c in connections]:
                            attr_spec.connectionPathList.Prepend(conn)
                else:
                    attr_spec.connectionPathList.ClearEditsAndMakeExplicit()
                    for conn in connections:
                        attr_spec.connectionPathList.Prepend(conn)
        except Exception:
            pass

    # Restore existing-only attributes when merging
    if merge_existing:
        for attr_name, attr_data in existing_attrs.items():
            if attr_name not in prim_spec.attributes:
                try:
                    attr_spec = Sdf.AttributeSpec(prim_spec, attr_name, attr_data["type_name"])
                    if attr_spec:
                        if attr_data["default"] is not None:
                            attr_spec.default = attr_data["default"]
                        attr_spec.variability = attr_data["variability"]
                        for conn in attr_data["connections"]:
                            attr_spec.connectionPathList.Prepend(conn)
                except Exception:
                    pass

    # Copy relationships
    for rel in src_prim.GetRelationships():
        if not rel.HasAuthoredTargets():
            continue
        try:
            rel_name = rel.GetName()
            rel_spec = prim_spec.relationships.get(rel_name)
            if not rel_spec:
                rel_spec = Sdf.RelationshipSpec(prim_spec, rel_name)

            if rel_spec:
                source_targets = rel.GetTargets()
                if merge_existing and rel_name in existing_rels and existing_rels[rel_name]:
                    source_target_strs = {str(t) for t in source_targets}
                    rel_spec.targetPathList.ClearEditsAndMakeExplicit()
                    for target in source_targets:
                        rel_spec.targetPathList.Prepend(target)
                    for target in existing_rels[rel_name]:
                        if str(target) not in source_target_strs:
                            rel_spec.targetPathList.Prepend(target)
                else:
                    for target in source_targets:
                        rel_spec.targetPathList.Prepend(target)
        except Exception:
            pass

    # Restore existing-only relationships when merging
    if merge_existing:
        for rel_name, targets in existing_rels.items():
            if rel_name not in prim_spec.relationships:
                try:
                    rel_spec = Sdf.RelationshipSpec(prim_spec, rel_name)
                    if rel_spec:
                        for target in targets:
                            rel_spec.targetPathList.Prepend(target)
                except Exception:
                    pass

    # Copy prim metadata
    for key in ["kind", "instanceable", "active", "hidden"]:
        if src_prim.HasMetadata(key):
            try:
                value = src_prim.GetMetadata(key)
                if value is not None:
                    prim_spec.SetInfo(key, value)
            except Exception:
                pass

    # Handle variant selections
    if merge_existing:
        for vset_name, selection in existing_variant_selections.items():
            if vset_name not in prim_spec.variantSelections:
                prim_spec.variantSelections[vset_name] = selection

    variant_sets = src_prim.GetVariantSets()
    for vset_name in variant_sets.GetNames():
        vset = variant_sets.GetVariantSet(vset_name)
        if vset:
            selection = vset.GetVariantSelection()
            if selection:
                prim_spec.variantSelections[vset_name] = selection

    clean_schema_metadata(prim_spec)

    # Recursively copy children
    for child in src_prim.GetFilteredChildren(Usd.TraverseInstanceProxies()):
        child_name = child.GetName()
        child_dst_path = dst_path.AppendChild(child_name)
        copy_composed_prim_to_layer(child, dst_layer, child_dst_path, merge_existing)

    return True


def ensure_prim_hierarchy(
    layer: Sdf.Layer,
    prim_path: str,
    default_type: str = "Scope",
) -> None:
    """Ensure the parent hierarchy exists for a given prim path.

    Args:
        layer: The layer to create the hierarchy in.
        prim_path: The full prim path whose parents need to exist.
        default_type: The type to use for created parent prims.

    Example:

    .. code-block:: python

        ensure_prim_hierarchy(layer, "/World/Scope/Prim")

    """
    path = Sdf.Path(prim_path)
    parent_path = path.GetParentPath()

    if parent_path != Sdf.Path.absoluteRootPath:
        parent_spec = layer.GetPrimAtPath(parent_path)
        if not parent_spec:
            ensure_prim_hierarchy(layer, parent_path.pathString, default_type)
            create_prim_spec(layer, parent_path.pathString, type_name=default_type)


def files_are_identical(path1: str, path2: str) -> bool:
    """Check if two files have identical content.

    Args:
        path1: Path to first file.
        path2: Path to second file.

    Returns:
        True if files are identical, False otherwise.

    Example:

    .. code-block:: python

        same = files_are_identical("/tmp/a.usda", "/tmp/b.usda")

    """
    try:
        if os.path.getsize(path1) != os.path.getsize(path2):
            return False
        with open(path1, "rb") as f1, open(path2, "rb") as f2:
            while True:
                chunk1 = f1.read(8192)
                chunk2 = f2.read(8192)
                if chunk1 != chunk2:
                    return False
                if not chunk1:
                    return True
    except Exception:
        return False


def sanitize_prim_name(name: str, prefix: str = "prim_") -> str:
    """Sanitize a name for use as a USD prim name.

    Args:
        name: The name to sanitize.
        prefix: Prefix to use if name starts with a digit.

    Returns:
        A sanitized name suitable for USD prim paths.

    Example:

    .. code-block:: python

        sanitized = sanitize_prim_name("My Prim")

    """
    sanitized = "".join(c if c.isalnum() or c == "_" else "_" for c in name)
    if not sanitized or sanitized[0].isdigit():
        sanitized = prefix + sanitized
    return sanitized


def copy_stage_metadata_from_layer(source_layer: Sdf.Layer, target_layer: Sdf.Layer) -> None:
    """Copy stage-level metadata from source layer to target layer.

    Copies all layer-level metadata (metersPerUnit, upAxis, timeCodesPerSecond, etc.)
    from the source layer to the target layer.

    Args:
        source_layer: The source layer to copy metadata from.
        target_layer: The layer to copy metadata to.

    Example:

    .. code-block:: python

        copy_stage_metadata_from_layer(source_layer, target_layer)

    """
    source_pseudo_root = source_layer.pseudoRoot
    target_pseudo_root = target_layer.pseudoRoot

    # Keys to skip (handled separately or should not be copied)
    skip_keys = frozenset(("defaultPrim", "subLayers", "primChildren"))

    # Copy all metadata from source to target
    for key in source_pseudo_root.ListInfoKeys():
        if key in skip_keys:
            continue
        try:
            value = source_pseudo_root.GetInfo(key)
            if value is not None:
                target_pseudo_root.SetInfo(key, value)
        except Exception:
            pass


def copy_stage_metadata(source_stage: Usd.Stage, target_layer: Sdf.Layer) -> None:
    """Copy stage-level metadata from source stage to target layer.

    Copies all layer-level metadata (metersPerUnit, upAxis, timeCodesPerSecond, etc.)
    from the source stage root layer to the target layer.

    Args:
        source_stage: The source stage to copy metadata from.
        target_layer: The layer to copy metadata to.

    Example:

    .. code-block:: python

        copy_stage_metadata(stage, target_layer)

    """
    copy_stage_metadata_from_layer(source_stage.GetRootLayer(), target_layer)


def get_path_string(asset_path_obj: object) -> str:
    """Extract path string from various USD path object types.

    Handles ArResolvedPath, Sdf.AssetPath, Sdf.Layer, or string.

    Args:
        asset_path_obj: The path object to extract from.

    Returns:
        The path string, or empty string if extraction fails.

    Example:

    .. code-block:: python

        path_str = get_path_string(asset_path)

    """
    if hasattr(asset_path_obj, "GetPathString"):
        return asset_path_obj.GetPathString()
    if hasattr(asset_path_obj, "realPath"):
        return asset_path_obj.realPath
    if hasattr(asset_path_obj, "resolvedPath"):
        return asset_path_obj.resolvedPath
    if hasattr(asset_path_obj, "identifier"):
        return asset_path_obj.identifier
    if hasattr(asset_path_obj, "path"):
        return asset_path_obj.path
    return str(asset_path_obj) if asset_path_obj else ""


def remap_asset_path(
    original_path: str,
    source_dir: str,
    dest_dir: str,
    variant_file_map: dict[str, str],
    collected_deps: dict[str, str],
) -> str:
    """Remap an asset path to a new location.

    Resolves the original path to absolute, then checks if it should be
    remapped to a variant file or collected dependency location.

    Args:
        original_path: The original asset path to remap.
        source_dir: Directory to resolve relative paths against.
        dest_dir: Destination directory for computing relative output paths.
        variant_file_map: Map from original variant paths to new variant paths.
        collected_deps: Map from original dependency paths to collected paths.

    Returns:
        The remapped path (relative to dest_dir), or original if no remapping needed.

    Example:

    .. code-block:: python

        remapped = remap_asset_path("foo.usd", "/src", "/dst", {}, {})

    """
    if not original_path:
        return original_path

    # Resolve to absolute path
    if os.path.isabs(original_path):
        abs_path = original_path
    else:
        abs_path = os.path.normpath(os.path.join(source_dir, original_path))

    normed = norm_path(abs_path)

    # Build list of paths to try (include USD extension variations)
    paths_to_try = [normed]
    base, ext = os.path.splitext(normed)
    if ext.lower() == ".usd":
        paths_to_try.extend([norm_path(base + ".usda"), norm_path(base + ".usdc")])
    elif ext.lower() in (".usda", ".usdc"):
        paths_to_try.append(norm_path(base + ".usd"))

    # Check variant file map first (try all path variations)
    for try_path in paths_to_try:
        if try_path in variant_file_map:
            return make_explicit_relative(os.path.relpath(variant_file_map[try_path], dest_dir))

    # Check collected deps (try all path variations)
    for try_path in paths_to_try:
        if try_path in collected_deps:
            return make_explicit_relative(os.path.relpath(collected_deps[try_path], dest_dir))

    # For existing files not in maps, make relative to destination
    if os.path.isfile(abs_path):
        return make_explicit_relative(os.path.relpath(abs_path, dest_dir))

    return original_path


def copy_file_to_directory(
    src_path: str,
    dest_dir: str,
    existing_collected: dict[str, str] | None = None,
) -> str | None:
    """Copy a file to a directory, handling filename conflicts.

    Args:
        src_path: Absolute path to source file.
        dest_dir: Directory to copy the file into.
        existing_collected: Optional dict to check for already-collected files.

    Returns:
        The destination path, or None if copy failed.

    Example:

    .. code-block:: python

        dest_path = copy_file_to_directory("/tmp/a.usd", "/tmp/output")

    """
    if not os.path.isfile(src_path):
        return None

    normed = norm_path(src_path)

    # Skip if already collected
    if existing_collected and normed in existing_collected:
        return existing_collected[normed]

    filename = os.path.basename(src_path)
    dest_path = os.path.join(dest_dir, filename)

    # Handle filename conflicts
    if os.path.exists(dest_path) and not files_are_identical(src_path, dest_path):
        base, ext = os.path.splitext(filename)
        counter = 1
        while os.path.exists(dest_path):
            dest_path = os.path.join(dest_dir, f"{base}_{counter}{ext}")
            counter += 1

    if not os.path.exists(dest_path):
        os.makedirs(dest_dir, exist_ok=True)
        shutil.copy2(src_path, dest_path)

    return dest_path


def copy_attributes_to_prim_spec(
    src_spec: Sdf.PrimSpec,
    dest_spec: Sdf.PrimSpec,
) -> int:
    """Copy attributes from source prim spec to destination.

    Args:
        src_spec: Source prim spec with attributes to copy.
        dest_spec: Destination prim spec to copy attributes to.

    Returns:
        Number of attributes copied.

    Example:

    .. code-block:: python

        count = copy_attributes_to_prim_spec(src_spec, dst_spec)

    """
    count = 0
    for attr_name in src_spec.attributes.keys():  # noqa: SIM118
        attr_spec = src_spec.attributes[attr_name]
        dest_attr = dest_spec.attributes.get(attr_name)
        if not dest_attr:
            try:
                if attr_spec.variability != Sdf.VariabilityVarying:
                    dest_attr = Sdf.AttributeSpec(dest_spec, attr_name, attr_spec.typeName, attr_spec.variability)
                else:
                    dest_attr = Sdf.AttributeSpec(dest_spec, attr_name, attr_spec.typeName)
            except Exception:
                continue
        if dest_attr and attr_spec.HasDefaultValue():
            dest_attr.default = attr_spec.default
            count += 1
    return count


def copy_relationships_to_prim_spec(
    src_spec: Sdf.PrimSpec,
    dest_spec: Sdf.PrimSpec,
) -> int:
    """Copy relationships from source prim spec to destination.

    Args:
        src_spec: Source prim spec with relationships to copy.
        dest_spec: Destination prim spec to copy relationships to.

    Returns:
        Number of relationships copied.

    Example:

    .. code-block:: python

        count = copy_relationships_to_prim_spec(src_spec, dst_spec)

    """
    count = 0
    for rel_name in src_spec.relationships.keys():  # noqa: SIM118
        rel_spec = src_spec.relationships[rel_name]
        dest_rel = dest_spec.relationships.get(rel_name)
        if not dest_rel:
            try:
                dest_rel = Sdf.RelationshipSpec(dest_spec, rel_name)
            except Exception:
                continue
        if dest_rel:
            for target in rel_spec.targetPathList.GetAddedOrExplicitItems():
                dest_rel.targetPathList.Prepend(target)
            count += 1
    return count


def copy_prim_metadata(
    src_spec: Sdf.PrimSpec,
    dest_spec: Sdf.PrimSpec,
    skip_keys: frozenset | None = None,
) -> None:
    """Copy metadata from source prim spec to destination.

    Args:
        src_spec: Source prim spec with metadata to copy.
        dest_spec: Destination prim spec to copy metadata to.
        skip_keys: Set of metadata keys to skip.

    Example:

    .. code-block:: python

        copy_prim_metadata(src_spec, dst_spec)

    """
    if skip_keys is None:
        skip_keys = COMPOSITION_SKIP_KEYS

    for key in src_spec.ListInfoKeys():
        if key in skip_keys:
            continue
        try:
            value = src_spec.GetInfo(key)
            if value is not None:
                dest_spec.SetInfo(key, value)
        except Exception:
            pass


def _try_usd_extensions(base_path: str) -> str | None:
    """Try USD extension variations for a path.

    Args:
        base_path: Base path to try extensions on.

    Returns:
        The first existing path, or None if none exist.

    """
    if os.path.isfile(base_path):
        return base_path

    base, ext = os.path.splitext(base_path)
    if ext.lower() == ".usd":
        for alt_ext in [".usda", ".usdc"]:
            alt_path = base + alt_ext
            if os.path.isfile(alt_path):
                return alt_path
    return None


def resolve_asset_path(
    arc_asset_path: str,
    base_layer: Sdf.Layer | None = None,
    fallback_dirs: list[str] | None = None,
) -> str:
    """Resolve an asset path to an absolute path.

    Tries multiple resolution strategies in order:
    1. If already absolute and exists (with USD extension variations), return it
    2. Resolve relative to base_layer (with USD extension variations)
    3. Resolve relative to each fallback directory (with USD extension variations)

    Args:
        arc_asset_path: The asset path to resolve.
        base_layer: Optional layer to resolve relative paths against.
        fallback_dirs: Optional list of directories to try for resolution.

    Returns:
        Resolved absolute path, or empty string if not resolvable.

    Example:

    .. code-block:: python

        resolved = resolve_asset_path("asset.usd", base_layer=layer, fallback_dirs=["/tmp"])

    """
    if not arc_asset_path:
        return ""

    # If already absolute, check if it exists (try USD extension variations)
    if os.path.isabs(arc_asset_path):
        resolved = _try_usd_extensions(arc_asset_path)
        return resolved if resolved else ""

    # Try base layer first
    if base_layer:
        layer_dir = os.path.dirname(base_layer.realPath)
        candidate = os.path.normpath(os.path.join(layer_dir, arc_asset_path))
        resolved = _try_usd_extensions(candidate)
        if resolved:
            return resolved

    # Try fallback directories
    if fallback_dirs:
        for fallback_dir in fallback_dirs:
            if fallback_dir:
                candidate = os.path.normpath(os.path.join(fallback_dir, arc_asset_path))
                resolved = _try_usd_extensions(candidate)
                if resolved:
                    return resolved

    return ""


def find_first_resolvable_arc(
    payloads: list[Sdf.Payload | Sdf.Reference],
    references: list[Sdf.Payload | Sdf.Reference],
    resolve_func: Callable[[str], str],
) -> str | None:
    """Find the first resolvable asset path from payloads or references.

    Args:
        payloads: List of payload arcs to check.
        references: List of reference arcs to check.
        resolve_func: Function to resolve asset paths.

    Returns:
        First resolved absolute path, or None if none found.

    Example:

    .. code-block:: python

        resolved = find_first_resolvable_arc(payloads, references, resolve_asset_path)

    """
    for arc in payloads:
        if arc.assetPath:
            resolved = resolve_func(arc.assetPath)
            if resolved:
                return resolved

    for arc in references:
        if arc.assetPath:
            resolved = resolve_func(arc.assetPath)
            if resolved:
                return resolved

    return None
