# SPDX-FileCopyrightText: Copyright (c) 2021-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

"""Scene validation utilities for checking ancestor transform validity.

This module provides utilities to validate that prims in a USD stage have valid
ancestor transforms, specifically checking that no parent prims have non-identity
scaling that would cause issues with world-space operations.
"""

from __future__ import annotations

import isaacsim.core.experimental.utils.prim as prim_utils
import numpy as np


def _prim_scaling_is_valid(prim: Usd.Prim) -> bool:
    """Check if a prim has valid (identity) scaling.

    Validates that the prim has identity local scaling [1,1,1] and, if present,
    unity unitsResolve scaling. This ensures that local scales match world scales.

    Args:
        prim: The USD prim to validate.

    Returns:
        True if the prim has valid scaling, False otherwise.
    """
    # if it authors no scale parameter, that is valid:
    if prim.HasAttribute("xformOp:scale"):
        local_scale = np.array(prim.GetAttribute("xformOp:scale").Get())

        # Note: We require identity scaling [1,1,1] rather than allowing uniform
        # scaling (alpha * [1,1,1]) because the experimental core API does not
        # provide get_world_scales(). This ensures local_scale == world_scale.
        if not np.allclose(local_scale, 1.0):
            return False

    # If the prim authors no point scaling, that is valid:
    if prim.HasAttribute("xformOp:scale:unitsResolve"):
        units_resolve = prim.GetAttribute("xformOp:scale:unitsResolve").Get()
        if not np.allclose(np.array(units_resolve), 1.0):
            return False

    return True


def _invalid_ancestors_of_prim(prim_path: str, checked_prims: list[str]) -> list[str]:
    """Find all invalid ancestors of a single prim.

    Traverses up the prim hierarchy and checks each Xformable ancestor for non-identity
    local scaling. Caches checked prims to avoid redundant validation.

    Args:
        prim_path: Path to the prim to validate.
        checked_prims: List of already-validated prim paths for caching.

    Returns:
        List of ancestor prim paths that have invalid (non-identity) scaling.
    """
    prim = prim_utils.get_prim_at_path(prim_path)
    current_prim = prim.GetParent()
    current_prim_path = lambda: current_prim.GetPath().pathString

    invalid_ancestors = []

    while current_prim_path() != "/":
        # We have already checked this prim, and by extension all of its ancestors.
        if current_prim_path() in checked_prims:
            return invalid_ancestors

        if not _prim_scaling_is_valid(current_prim):
            invalid_ancestors.append(current_prim_path())

        checked_prims.append(current_prim_path())
        current_prim = current_prim.GetParent()

    return invalid_ancestors


def find_all_invalid_ancestors(prim_paths: list[str]) -> list[str]:
    """Find all invalid ancestors for a list of prims.

    Validates that all ancestor prims have identity local scaling [1,1,1] and have
    unity unitsResolve. This ensures that local scales match world scales and prevents
    shearing or mirroring in the transform hierarchy.

    The function efficiently caches checked ancestors to avoid redundant validation when
    multiple prims share common ancestors.

    Args:
        prim_paths: List of prim paths to validate.

    Returns:
        List of ancestor prim paths that have invalid (non-identity) scaling. Each invalid
        ancestor appears only once in the list, even if multiple queried prims share it.

    Example:

    .. code-block:: python

        from isaacsim.robot_motion.experimental.motion_generation.impl.utils import scene_validation
        invalid = scene_validation.find_all_invalid_ancestors(["/World/Parent/Child"])
        if len(invalid) > 0:
            print(f"Found invalid ancestors: {invalid}")
    """
    checked_prims = []
    invalid_prims = []
    for prim_path in prim_paths:
        invalid_prims.extend(_invalid_ancestors_of_prim(prim_path, checked_prims))
    return invalid_prims
