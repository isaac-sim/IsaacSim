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
from pxr import Gf, Usd, UsdGeom


def get_world_scale(prim_path: str) -> np.ndarray:
    """Get a prim's composed world scale without requiring Kit.

    Args:
        prim_path: Path to the USD prim.

    Returns:
        World scale as an ``xyz`` array, or identity for a non-transformable prim.

    Raises:
        ValueError: If the path does not identify a valid prim.
    """
    prim = prim_utils.get_prim_at_path(prim_path)
    if not prim.IsValid():
        raise ValueError(f"Prim path does not identify a valid prim: {prim_path}")
    xformable = UsdGeom.Xformable(prim)
    if not xformable:
        return np.ones(3, dtype=np.float32)
    world_transform = xformable.ComputeLocalToWorldTransform(Usd.TimeCode.Default())
    return np.asarray(Gf.Transform(world_transform).GetScale(), dtype=np.float32)


def _prim_scaling_is_valid(prim_path: object) -> bool:
    """Check if a prim has valid (identity) scaling.

    Validates that the prim has uniform global scaling and, if present,
    unity unitsResolve scaling.

    Args:
        prim_path: The USD prim path to validate.

    Returns:
        True if the prim has valid scaling, False otherwise.
    """
    world_scale = get_world_scale(prim_path)
    if (
        world_scale[0] <= 0
        or not np.isclose(world_scale, np.array([world_scale[0], world_scale[0], world_scale[0]])).all()
    ):
        return False

    prim = prim_utils.get_prim_at_path(prim_path)
    attribute_names = prim_utils.get_prim_attribute_names(prim)
    # If the prim authors no point scaling, that is valid:
    if "xformOp:scale:unitsResolve" in attribute_names:
        units_resolve = prim_utils.get_prim_attribute_value(prim, "xformOp:scale:unitsResolve")
        if not np.allclose(np.array(units_resolve), 1.0):
            return False

    return True


def _invalid_ancestors_of_prim(prim_path: str, checked_prims: list[str]) -> list[str]:
    """Find all invalid ancestors of a single prim.

    Traverses up the prim hierarchy and checks each Xformable ancestor for non-uniform
    world scaling. Caches checked prims to avoid redundant validation.

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

        if not _prim_scaling_is_valid(current_prim_path()):
            invalid_ancestors.append(current_prim_path())

        checked_prims.append(current_prim_path())
        current_prim = current_prim.GetParent()

    return invalid_ancestors


def find_all_invalid_ancestors(prim_paths: list[str]) -> list[str]:
    """Find all invalid ancestors for a list of prims.

    Validates that all ancestor prims have uniform scaling and have
    unity unitsResolve.

    The function efficiently caches checked ancestors to avoid redundant validation when
    multiple prims share common ancestors.

    Args:
        prim_paths: List of prim paths to validate.

    Returns:
        List of ancestor prim paths that have invalid (non-uniform) scaling. Each invalid
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
