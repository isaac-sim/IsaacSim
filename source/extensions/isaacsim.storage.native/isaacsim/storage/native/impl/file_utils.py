# SPDX-FileCopyrightText: Copyright (c) 2025-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

"""File utility functions for USD asset discovery, validation, and path operations."""

import asyncio
import concurrent.futures
import os
from typing import List

import carb
from pxr import Sdf, UsdUtils

from ..nucleus import get_assets_root_path, get_assets_root_path_async

_URL_SCHEMES = ("omniverse://", "https://", "http://", "ftp://", "sftp://")


def path_join(base: str, name: str) -> str:
    """Join two path components intelligently handling Omniverse URLs and remote URLs.

    Args:
        base: Base path, can be local or a URL (omniverse://, https://, file://, etc.).
        name: Path component to append to the base.

    Returns:
        Joined path string.

    Example:

    .. code-block:: python

        >>> path_join("omniverse://server/folder", "file.usd")
        'omniverse://server/folder/file.usd'
        >>> path_join("https://cdn.example.com/assets", "file.usd")
        'https://cdn.example.com/assets/file.usd'
        >>> path_join("/local/path", "file.usd")
        '/local/path/file.usd'
    """
    if "://" in base:
        name = name.replace("\\", "/")
        if name.startswith("./"):
            name = name[2:]
        while name.startswith("../"):
            base = base.rsplit("/", 1)[0]
            name = name[3:]
        if base.endswith("/"):
            base = base[:-1]
        return base + "/" + name
    else:
        return os.path.join(base, name)


def is_local_path(path: str) -> bool:
    """Check if a path is local vs online (omniverse://, https://, etc.).

    This function determines whether a given path points to an offline resource
    that can be accessed without network connectivity.

    Args:
        path: The file path to check.

    Returns:
        True if the path is local filesystem, False if online (omniverse://, https://, etc.).

    Example:

    .. code-block:: python

        >>> is_local_path("/home/user/file.usd")
        True
        >>> is_local_path("omniverse://server/path/file.usd")
        False
    """
    if not path:
        return True  # Empty paths are considered local

    path = path.strip()

    if any(path.startswith(scheme) for scheme in _URL_SCHEMES):
        return False

    # Local paths (absolute or relative) are considered local
    return True


def find_files_recursive(abs_path: list, filter_fn: object = lambda a: True):
    """Recursively list all files under given path(s) that match the filter function.

    Args:
        abs_path: List of absolute paths to search.
        filter_fn: Filter function that takes a path and returns boolean indicating
            if path should be included. Defaults to accepting all files.

    Returns:
        List of file paths that match the filter criteria.

    Example:

    .. code-block:: python

        >>> from isaacsim.storage.native import find_files_recursive
        >>>
        >>> # Find all USD files
        >>> usd_filter = lambda p: p.endswith('.usd')
        >>> files = find_files_recursive(["/path/to/assets"], usd_filter)
    """
    import omni.client
    from omni.client import Result

    sub_folders = []
    remaining_folders = []
    remaining_folders.extend(abs_path)
    files = []
    while remaining_folders:
        path = remaining_folders.pop()
        result, entries = omni.client.list(path)
        if result == Result.OK:
            files.extend(
                [path_join(path, e.relative_path) for e in entries if (e.flags & 4) == 0 and filter_fn(e.relative_path)]
            )
            remaining_folders.extend([path_join(path, e.relative_path) for e in entries if (e.flags & 4) > 0])
    return files


def find_filtered_files(
    abs_paths: List[str],
    max_depth: int = None,
    filepath_excludes: List[str] = [],
    filter_patterns: List[str] = [],
    match_all: bool = False,
) -> set:
    """Find and filter USD files recursively with optional depth and pattern constraints.

    Traverses directory trees starting from the provided absolute paths to discover valid USD files.
    Supports recursive search with configurable depth limits, filepath exclusion patterns,
    and regex-based filtering. Uses Omniverse client for robust file system operations
    across local and remote paths.

    Args:
        abs_paths: List of absolute directory or file paths to search. Supports local paths and omniverse:// URLs.
        max_depth: Maximum recursion depth for directory traversal. If None, searches
            without depth limit. Depth 0 means current directory only.
        filepath_excludes: List of strings that, if found in a filepath, will exclude
            that file from results. Commonly used to exclude directories like ".thumbs".
        filter_patterns: List of regex pattern strings to filter discovered filepaths.
            Patterns are applied to the full absolute filepath.
        match_all: Controls regex pattern matching behavior. If True, all patterns in
            filter_patterns must match for a file to be included. If False, any single
            pattern match includes the file.

    Returns:
        Set of absolute paths to valid USD files that match all filtering criteria.

    Example:

    .. code-block:: python

        # Find USD files in Isaac environments with depth limit
        >>> paths = ["/Isaac/Environments", "/Isaac/Samples"]
        >>> usd_files = find_filtered_files(
        ...     abs_paths=paths,
        ...     max_depth=2,
        ...     filepath_excludes=[".thumbs"],
        ... )

        # Find files matching specific patterns (any pattern)
        >>> filtered_files = find_filtered_files(
        ...     abs_paths=["/Isaac/Samples"],
        ...     filter_patterns=["carter", "navigation"],
        ...     match_all=False,
        ... )

        # Find files matching all patterns
        >>> strict_filtered = find_filtered_files(
        ...     abs_paths=["/Isaac/Samples"],
        ...     filter_patterns=["robot", "demo"],
        ...     match_all=True,
        ... )
    """
    import re

    import omni.client
    from omni.client import Result

    usd_files = set()
    # Track paths with their current depth: [(path, depth)]
    remaining_folders = [(path, 0) for path in abs_paths]

    # Compile regex patterns once for efficiency
    compiled_patterns = []
    if filter_patterns:
        for pattern_str in filter_patterns:
            try:
                compiled_patterns.append(re.compile(pattern_str))
            except re.error:
                carb.log_warn(f"Invalid regex pattern: {pattern_str}")

    while remaining_folders:
        current_path, current_depth = remaining_folders.pop()

        result, entries = omni.client.list(current_path)
        if result != Result.OK:
            carb.log_warn(f"Could not list path '{current_path}': {result}")
            continue
        for entry in entries:
            entry_path = path_join(current_path, entry.relative_path)

            # Check if it's a file (not a directory)
            if (entry.flags & 4) == 0:  # 4 is the directory flag
                # Apply USD file validation and filtering
                if is_valid_usd_file(entry_path, filepath_excludes):
                    # Apply pattern filters if provided
                    if filter_patterns:
                        if match_all:  # ALL patterns must match
                            if all(pattern.search(entry_path) for pattern in compiled_patterns):
                                usd_files.add(entry_path)
                        else:  # ANY pattern can match (default)
                            if any(pattern.search(entry_path) for pattern in compiled_patterns):
                                usd_files.add(entry_path)
                    else:
                        # No pattern filters, just add valid USD file
                        usd_files.add(entry_path)

            # If it's a directory and we haven't exceeded max depth, add to queue
            elif (entry.flags & 4) > 0:
                if max_depth is None or current_depth < max_depth:
                    remaining_folders.append((entry_path, current_depth + 1))

    return usd_files


def get_stage_references(stage_path: str, resolve_relatives: bool = True):
    """List all references in a USD stage.

    Args:
        stage_path: Path to the USD stage.
        resolve_relatives: If True, resolve all relative paths to absolute.

    Returns:
        List of path strings to referenced assets.

    Example:

    .. code-block:: python

        >>> from isaacsim.storage.native import get_stage_references
        >>>
        >>> refs = get_stage_references("/path/to/scene.usd")
        >>> refs
        ['/path/to/material.usd', '/path/to/mesh.usd']
    """
    all_layers, all_assets, unresolved_paths = UsdUtils.ComputeAllDependencies(stage_path)
    paths = []

    def add_path(path):
        paths.append(path)
        return path

    if resolve_relatives:
        for layer in all_layers:
            UsdUtils.ModifyAssetPaths(layer, add_path)
    else:
        paths = [str(layer).split("'")[1] for layer in all_layers]
    paths = list(set(paths))
    return paths


def is_absolute_path(path: str):
    """Check if a path is absolute, including Omniverse URLs.

    Handles standard filesystem absolute paths as well as omniverse://,
    file://, http://, and https:// URLs.

    Args:
        path: Path string to check.

    Returns:
        True if path is absolute.

    Example:

    .. code-block:: python

        >>> from isaacsim.storage.native import is_absolute_path
        >>>
        >>> is_absolute_path("/home/user/file.usd")
        True
        >>> is_absolute_path("omniverse://localhost/path/file.usd")
        True
        >>> is_absolute_path("relative/path/file.usd")
        False
    """
    if path.lower().startswith("omniverse://"):
        return True
    if path.lower().startswith("file://"):
        return True
    if path.lower().startswith("http://"):
        return True
    if path.lower().startswith("https://"):
        return True
    return os.path.isabs(path)


def is_valid_usd_file(item: str, excludes: list) -> bool:
    """Check if a path is a USD file and doesn't contain excluded substrings.

    Valid USD file extensions are: .usd, .usda, .usdc, .usdz.

    Args:
        item: Path to check.
        excludes: List of substrings that should not be present in the path.

    Returns:
        True if the path is a valid USD file.

    Example:

    .. code-block:: python

        >>> from isaacsim.storage.native import is_valid_usd_file
        >>>
        >>> is_valid_usd_file("/path/to/scene.usd", [".thumbs"])
        True
        >>> is_valid_usd_file("/path/.thumbs/scene.usd", [".thumbs"])
        False
        >>> is_valid_usd_file("/path/to/image.png", [])
        False
    """
    # remove any substrings we dont want
    for e in excludes:
        if e in item:
            return False
    _, ext = os.path.splitext(item)
    if ext in [".usd", ".usda", ".usdc", ".usdz"]:
        return True
    return False


def is_mdl_file(item: str) -> bool:
    """Check if a path is an MDL file.

    Args:
        item: Path to check.

    Returns:
        True if the path is an MDL file.

    Example:

    .. code-block:: python

        >>> from isaacsim.storage.native import is_mdl_file
        >>>
        >>> is_mdl_file("/path/to/material.mdl")
        True
        >>> is_mdl_file("/path/to/scene.usd")
        False
    """
    _, ext = os.path.splitext(item)
    return ext in [".mdl"]


async def find_absolute_paths_in_usds(base_path: str):
    """Check for absolute paths in USD files.

    Recursively searches for USD files and identifies any absolute path references
    within each file.

    Args:
        base_path: Base path to search for USD files.

    Returns:
        Dictionary mapping file paths to lists of absolute references they contain.

    Example:

    .. code-block:: python

        >>> import asyncio
        >>> from isaacsim.storage.native import find_absolute_paths_in_usds
        >>>
        >>> abs_paths = asyncio.get_event_loop().run_until_complete(
        ...     find_absolute_paths_in_usds("/path/to/assets")
        ... )
        >>> abs_paths
        {'/path/to/scene.usd': ['/absolute/ref.usd']}
    """
    abs_items = {}
    files = await find_files_recursive(base_path, lambda item: is_valid_usd_file(item, []))
    for i, item in enumerate(files):
        print(f"check {i}/{len(files)}")
        abs_refs = [i for i in get_stage_references(item) if is_absolute_path(i)]
        if abs_refs:
            abs_items[item] = abs_refs
    return abs_items


def is_path_external(path: str, base_path: str) -> bool:
    """Check if a path is external to a base path.

    A path is considered external if it does not contain the base_path as a substring.

    Args:
        path: Path to check.
        base_path: Base path to compare against.

    Returns:
        True if path is external to base_path.

    Raises:
        Exception: If there is an error comparing the paths.

    Example:

    .. code-block:: python

        >>> from isaacsim.storage.native import is_path_external
        >>>
        >>> is_path_external("/external/path/file.usd", "/project/assets")
        True
        >>> is_path_external("/project/assets/file.usd", "/project/assets")
        False
    """
    try:
        return base_path not in path
    except:
        print(path, base_path)
        raise Exception("Error comparing paths")


async def find_external_references(base_path: str):
    """Check for external references in USD files.

    Recursively searches for USD files and identifies references that point
    outside the base path.

    Args:
        base_path: Base path to search for USD files.

    Returns:
        Dictionary mapping file paths to lists of external references they contain.

    Example:

    .. code-block:: python

        >>> import asyncio
        >>> from isaacsim.storage.native import find_external_references
        >>>
        >>> ext_refs = asyncio.get_event_loop().run_until_complete(
        ...     find_external_references("/project/assets")
        ... )
        >>> ext_refs
        {'/project/assets/scene.usd': ['/shared/material.usd']}
    """
    abs_items = {}
    for item in await find_files_recursive(base_path, lambda item: is_valid_usd_file(item, [])):
        parent = os.path.dirname(item)
        abs_refs = [i for i in get_stage_references(item, resolve_relatives=False) if is_path_external(i, base_path)]
        if abs_refs:
            abs_items[item] = abs_refs
    return abs_items


async def count_asset_references(base_path: str):
    """Get reference counts for all assets in a base path.

    Recursively searches for all files and counts how many times each file
    is referenced by other USD files.

    Args:
        base_path: Base path to search for assets.

    Returns:
        Dictionary mapping asset paths to their reference counts, sorted by count.

    Example:

    .. code-block:: python

        >>> import asyncio
        >>> from isaacsim.storage.native import count_asset_references
        >>>
        >>> ref_counts = asyncio.get_event_loop().run_until_complete(
        ...     count_asset_references("/project/assets")
        ... )
        >>> ref_counts
        {'/project/assets/unused.usd': 0, '/project/assets/shared.usd': 5}
    """
    items = {item: 0 for item in await find_files_recursive(base_path)}
    for item in items.keys():
        print(item)
        for i in get_stage_references(item):
            base = os.path.dirname(item)
            name = path_join(base, i)
            print(" ", name)
            if name in items:
                items[name] += 1
    items = {k: v for k, v in sorted(items.items(), key=lambda item: item[1])}
    return items


def find_missing_references(base_path: str):
    """Check for missing references in USD files.

    Recursively searches for USD files and prints any files that have
    unresolved references.

    Args:
        base_path: Base path to search for USD files.

    Example:

    .. code-block:: python

        >>> from isaacsim.storage.native import find_missing_references
        >>>
        >>> find_missing_references("/project/assets")
        /project/assets/scene.usd ['missing_file.usd']
    """
    items = {item: 0 for item in find_files_recursive(base_path, lambda item: is_valid_usd_file(item, []))}
    for item in items.keys():
        all_layers, all_assets, unresolved_paths = UsdUtils.ComputeAllDependencies(item)
        if unresolved_paths:
            print(item, unresolved_paths)


async def path_exists(path: str) -> bool:
    """Check if a path exists.

    Uses the Omniverse client to check if the path exists on local
    filesystem or remote Nucleus server.

    Args:
        path: Path to check.

    Returns:
        True if the path exists.

    Example:

    .. code-block:: python

        >>> import asyncio
        >>> from isaacsim.storage.native import path_exists
        >>>
        >>> exists = asyncio.get_event_loop().run_until_complete(
        ...     path_exists("omniverse://localhost/NVIDIA/Assets/Isaac")
        ... )
        >>> exists
        True
    """
    import omni.client
    from omni.client import Result

    result, _ = await omni.client.stat_async(path)
    return result == Result.OK


def layer_has_missing_references(layer_identifier: str) -> bool:
    """Check if a layer has any missing references.

    Recursively checks the layer and all its external references to find
    any layers that cannot be opened.

    Args:
        layer_identifier: Identifier for the layer to check.

    Returns:
        True if the layer has missing references.

    Example:

    .. code-block:: python

        >>> from isaacsim.storage.native import layer_has_missing_references
        >>>
        >>> has_missing = layer_has_missing_references("/path/to/scene.usd")
        >>> has_missing
        False
    """
    queue = [layer_identifier]
    accessed_layers = []
    while queue:
        identifier = queue.pop(0)
        if identifier in accessed_layers:
            continue

        accessed_layers.append(identifier)
        layer = Sdf.Layer.FindOrOpen(identifier)
        if layer:
            for reference in layer.externalReferences:
                if reference:
                    absolute_path = layer.ComputeAbsolutePath(reference)
                    queue.append(absolute_path)
        else:
            return True

    return False


def prim_spec_has_missing_references(prim_spec: object) -> bool:
    """Check if a prim specification has any missing references.

    Checks all references in the prim specification's reference list and
    verifies that they can be resolved.

    Args:
        prim_spec: Prim specification to check.

    Returns:
        True if the prim specification has missing references.

    Example:

    .. code-block:: python

        >>> from pxr import Sdf
        >>> from isaacsim.storage.native import prim_spec_has_missing_references
        >>>
        >>> layer = Sdf.Layer.FindOrOpen("/path/to/scene.usd")
        >>> prim_spec = layer.GetPrimAtPath("/World/Mesh")
        >>> has_missing = prim_spec_has_missing_references(prim_spec)
        >>> has_missing
        False
    """
    from omni.kit.widget.stage.stage_model import AssetType

    reference_list = prim_spec.referenceList
    items = reference_list.GetAddedOrExplicitItems()
    for item in items:
        if item.assetPath:
            filename = item.assetPath
            if AssetType().is_usd(filename):
                filename = prim_spec.layer.ComputeAbsolutePath(filename)
                if layer_has_missing_references(filename):
                    return True

    return False


def prim_has_missing_references(prim: object) -> bool:
    """Check if a prim has any missing references.

    Checks all prim specs in the prim's stack for missing references.

    Args:
        prim: Prim to check.

    Returns:
        True if the prim has missing references.

    Example:

    .. code-block:: python

        >>> from pxr import Usd
        >>> from isaacsim.storage.native import prim_has_missing_references
        >>>
        >>> stage = Usd.Stage.Open("/path/to/scene.usd")
        >>> prim = stage.GetPrimAtPath("/World/Mesh")
        >>> has_missing = prim_has_missing_references(prim)
        >>> has_missing
        False
    """
    for prim_spec in prim.GetPrimStack():
        if prim_spec_has_missing_references(prim_spec):
            return True

    return False


def path_relative(path: str, start: str) -> str:
    """URL friendly version of os.path.relpath.

    Handles both local filesystem paths and Omniverse URLs.

    Args:
        path: Path to make relative.
        start: Start path to make the path relative to.

    Returns:
        Relative path string.

    Raises:
        ValueError: If URL scheme or domain does not match.

    Example:

    .. code-block:: python

        >>> from isaacsim.storage.native import path_relative
        >>>
        >>> path_relative("/project/assets/textures/wood.png", "/project/assets")
        'textures/wood.png'
        >>> path_relative("omniverse://localhost/A/B/file.usd", "omniverse://localhost/A")
        'B/file.usd'
    """
    from urllib.parse import urlparse

    # No trailing slash
    start = start.rstrip("/\\")

    # Determine if both are URLs
    parsed_path = urlparse(path)
    parsed_start = urlparse(start)

    if parsed_path.scheme and parsed_path.netloc:
        # Ensure same scheme and netloc
        if parsed_path.scheme != parsed_start.scheme or parsed_path.netloc != parsed_start.netloc:
            raise ValueError("URL scheme or domain mismatch.")

        return os.path.relpath(parsed_path.path, parsed_start.path)

    else:
        # Local file paths (Windows, Linux)
        return os.path.relpath(os.path.normpath(path), os.path.normpath(start))


def path_dirname(path: str) -> str:
    """URL friendly version of os.path.dirname.

    Handles both local filesystem paths and Omniverse URLs.

    Args:
        path: Path to get the directory name from.

    Returns:
        Directory path string.

    Example:

    .. code-block:: python

        >>> from isaacsim.storage.native import path_dirname
        >>>
        >>> path_dirname("/project/assets/scene.usd")
        '/project/assets/'
        >>> path_dirname("omniverse://localhost/A/B/file.usd")
        'omniverse://localhost/A/B/'
    """
    from urllib.parse import urlparse, urlunparse

    parsed = urlparse(path)
    # URL
    if parsed.scheme and parsed.netloc:
        dir_path = os.path.dirname(parsed.path)
        if not dir_path.endswith("/"):
            dir_path += "/"
        return urlunparse((parsed.scheme, parsed.netloc, dir_path, "", "", ""))
    else:
        return os.path.dirname(os.path.normpath(path)) + os.sep


async def resolve_asset_path_async(original_path: str) -> str | None:
    """Resolve asset path by checking original location and fallback to assets root.

    This async function attempts to resolve a given asset path by first checking if it exists
    in its original location. If not found, it constructs an alternate path using the
    Isaac Sim assets root path and checks if that exists instead.

    Args:
        original_path: The original asset path to resolve.

    Returns:
        The resolved path if found, None otherwise.
        Returns the path that exists (either original or assets root based), or None if neither exists.

    Raises:
        Exception: May raise exceptions from path_exists or get_assets_root_path_async calls.

    Example:

    .. code-block:: python

        >>> path = await resolve_asset_path_async("/Isaac/Environments/Grid/default_environment.usd")
        >>> print(path)
        /path/to/assets/Isaac/Environments/Grid/default_environment.usd
        >>>
        >>> invalid_path = await resolve_asset_path_async("/nonexistent/file.usd")
        >>> print(invalid_path)
        None
    """
    # Check if the original path exists
    original_exists = await path_exists(original_path)
    if original_exists:
        return original_path

    try:
        # Construct alternate path with asset root
        assets_root_path = await get_assets_root_path_async()
        if assets_root_path:
            alternate_path = (
                assets_root_path + original_path
                if original_path.startswith("/")
                else assets_root_path + "/" + original_path
            )

            if await path_exists(alternate_path):
                return alternate_path
    except Exception as e:
        carb.log_warn(f"Could not get assets root path: {e}")

    return None


def resolve_asset_path(original_path: str) -> str | None:
    """Resolve asset path by checking original location and fallback to assets root.

    This function attempts to resolve a given asset path by first checking if it exists
    in its original location. If not found, it constructs an alternate path using the
    Isaac Sim assets root path and checks if that exists instead.

    Args:
        original_path: The original asset path to resolve.

    Returns:
        The resolved path if found, None otherwise.
        Returns the path that exists (either original or assets root based), or None if neither exists.

    Raises:
        Exception: May raise exceptions from synchronous stat calls or get_assets_root_path.

    Example:

    .. code-block:: python

        >>> path = resolve_asset_path("/Isaac/Environments/Grid/default_environment.usd")
        >>> print(path)
        /path/to/assets/Isaac/Environments/Grid/default_environment.usd
        >>>
        >>> invalid_path = resolve_asset_path("/nonexistent/file.usd")
        >>> print(invalid_path)
        None
    """
    import omni.client
    from omni.client import Result

    # Check if the original path exists
    result, _ = omni.client.stat(original_path)
    if result == Result.OK:
        return original_path

    try:
        # Construct alternate path with asset root
        assets_root_path = get_assets_root_path()
        if assets_root_path:
            alternate_path = (
                assets_root_path + original_path
                if original_path.startswith("/")
                else assets_root_path + "/" + original_path
            )

            result_alt, _ = omni.client.stat(alternate_path)
            if result_alt == Result.OK:
                return alternate_path
    except Exception as e:
        carb.log_warn(f"Could not get assets root path: {e}")

    return None


async def find_filtered_files_async(
    root_path: str,
    filter_patterns: List[str] = [],
    match_all: bool = False,
    filepath_excludes: List[str] = [],
    max_depth: int = None,
) -> set:
    """Asynchronously find and filter USD files recursively with optional depth and pattern constraints.

    This is an async wrapper around find_filtered_files that uses a thread pool executor
    to avoid blocking the main thread. Results are returned as a set for automatic deduplication.

    Args:
        root_path: Root directory or file path to traverse. Can be local, file://, or omniverse://.
        filter_patterns: Optional list of regex patterns to filter filepaths.
        match_all: If True, all patterns must match. If False, any pattern can match.
        filepath_excludes: List of substrings that should not be present in filepaths.
        max_depth: Maximum directory depth to traverse. None means unlimited depth.

    Returns:
        A set of absolute paths to USD files discovered during traversal.
    """
    # Get filtered USD files with depth limit in one pass
    loop = asyncio.get_event_loop()
    with concurrent.futures.ThreadPoolExecutor() as executor:
        usd_files_set = await loop.run_in_executor(
            executor,
            find_filtered_files,
            [root_path],
            max_depth,
            filepath_excludes,
            filter_patterns,
            match_all,
        )

    return usd_files_set
