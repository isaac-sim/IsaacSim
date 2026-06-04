# SPDX-FileCopyrightText: Copyright (c) 2024-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

"""Utility functions for Newton tensor API."""

from __future__ import annotations

import re

from pxr import Sdf, Usd


def find_matching_paths(stage: Usd.Stage, pattern: str | list[str]) -> list[str]:
    """Find USD paths matching a pattern.

    Args:
        stage: USD stage to search.
        pattern: Path pattern with optional wildcards.

    Returns:
        List of matching path strings.
    """
    if not stage:
        return []

    # Handle list of patterns by recursively processing each
    if isinstance(pattern, list):
        all_paths = []
        for p in pattern:
            all_paths.extend(find_matching_paths(stage, p))
        return all_paths

    paths_ret = []

    # Check if pattern is a valid SdfPath and exists in the stage (no wildcards)
    if isinstance(pattern, str) and Sdf.Path.IsValidPathString(pattern) and "*" not in pattern and "[" not in pattern:
        prim = stage.GetPrimAtPath(Sdf.Path(pattern))
        if prim:
            paths_ret.append(str(prim.GetPath()))
            return paths_ret

    pattern = pattern.strip("/")
    tokens = pattern.split("/")

    # Convert wildcard patterns to regex and wrap in '^' and '$' to match full names
    tokens = [f"^{tok.replace('*', '.*')}$" for tok in tokens]

    roots = [stage.GetPseudoRoot()]
    matches = []  # type: ignore[var-annotated]

    num_tokens = len(tokens)
    for i in range(num_tokens):
        for prim in roots:
            _find_matching_children(prim, tokens[i], matches)

        if i < num_tokens - 1:
            roots, matches = matches, []

    result = [str(prim.GetPath()) for prim in matches]
    return result


def _find_matching_children(root: Usd.Prim, pattern: str, prims_ret: list[Usd.Prim]) -> None:
    """Find children of a prim matching a regex pattern.

    Args:
        root: Root prim to search.
        pattern: Regex pattern to match child names.
        prims_ret: Output list to append matching prims.
    """
    if not root:
        return

    matcher = re.compile(pattern)
    for child in root.GetAllChildren():
        if matcher.match(child.GetName()):
            prims_ret.append(child)
