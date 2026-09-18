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

"""Provide utility functions for the Newton tensor API."""

from __future__ import annotations

import re

from pxr import Sdf, Usd


def find_matching_paths(stage: Usd.Stage, pattern: str | list[str], recursive_leaf: bool = True) -> list[str]:
    """Find USD paths matching a pattern.

    Args:
        stage: USD stage to search.
        pattern: Slash-separated path pattern or patterns. ``*`` matches text within a segment, ``**`` descends
            recursively, and other regular-expression syntax is preserved.
        recursive_leaf: Whether a named final token may match descendants at any depth. If False, every named token
            matches direct children only.

    Returns:
        Matching path strings in pattern and stage-traversal order. When multiple input patterns select the same prim,
        its path can appear more than once.

    """
    if not stage:
        return []

    # Handle list of patterns by recursively processing each
    if isinstance(pattern, list):
        all_paths = []
        for p in pattern:
            all_paths.extend(find_matching_paths(stage, p, recursive_leaf))
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
    num_tokens = len(tokens)

    roots = [stage.GetPseudoRoot()]
    matches: list[Usd.Prim] = []

    # Per-segment matching contract:
    #   * ``**`` matches the current prim and all its descendants (recursive descent);
    #   * a named leaf token (not a bare ``*``) matches every descendant by name -- so a
    #     known link resolves regardless of how deeply the USD converter nested it -- unless
    #     ``recursive_leaf`` is False, which restricts it to direct children like every
    #     other named token;
    #   * a bare ``*`` leaf and every intermediate token match only direct children.
    for i, tok in enumerate(tokens):
        matches = []
        is_leaf = i == num_tokens - 1

        if tok == "**":
            for prim in roots:
                _collect_self_and_descendants(prim, matches)
        elif is_leaf and tok != "*" and recursive_leaf:
            matcher = _compile_token(tok)
            for prim in roots:
                _collect_matching_descendants(prim, matcher, matches)
        else:
            matcher = _compile_token(tok)
            for prim in roots:
                if prim:
                    for child in prim.GetAllChildren():
                        if matcher.match(child.GetName()):
                            matches.append(child)

        if i < num_tokens - 1:
            roots = matches

    # De-duplicate, preserving first-seen order: a recursive-leaf token following a
    # ``**`` finds the same descendant once per ancestor root, so the raw list can
    # repeat a path (e.g. ``Robot/**/link_2`` finds link_2 under each ancestor).
    seen: set[str] = set()
    result: list[str] = []
    for prim in matches:
        path = str(prim.GetPath())
        if path not in seen:
            seen.add(path)
            result.append(path)
    return result


def _compile_token(token: str) -> re.Pattern[str]:
    """Compile a path segment into an anchored regular expression.

    Args:
        token: Path segment whose asterisks represent arbitrary text.

    Returns:
        Anchored expression with alternation scoped to the complete segment.

    """
    return re.compile(f"^(?:{token.replace('*', '.*')})$")


def _collect_self_and_descendants(root: Usd.Prim, prims_ret: list[Usd.Prim]) -> None:
    """Append a prim and its descendants in preorder.

    The USD pseudo-root itself is omitted.

    Args:
        root: Root prim to traverse.
        prims_ret: Destination list to extend.

    """
    if not root:
        return
    if root.GetPath() != Sdf.Path.absoluteRootPath:
        prims_ret.append(root)
    for child in root.GetAllChildren():
        _collect_self_and_descendants(child, prims_ret)


def _collect_matching_descendants(
    root: Usd.Prim,
    matcher: re.Pattern[str],
    prims_ret: list[Usd.Prim],
    matched_ancestor_names: frozenset[str] = frozenset(),
) -> None:
    """Append matching descendants while suppressing repeated names along a branch.

    Args:
        root: Root prim whose descendants are searched.
        matcher: Expression applied to each descendant name.
        prims_ret: Destination list for matching prims.
        matched_ancestor_names: Matching names already encountered on the current ancestor path.

    """
    if not root:
        return
    for child in root.GetAllChildren():
        name = child.GetName()
        child_matched_names = matched_ancestor_names
        if matcher.match(name):
            if name not in matched_ancestor_names:
                prims_ret.append(child)
            child_matched_names = matched_ancestor_names | {name}
        _collect_matching_descendants(child, matcher, prims_ret, child_matched_names)
