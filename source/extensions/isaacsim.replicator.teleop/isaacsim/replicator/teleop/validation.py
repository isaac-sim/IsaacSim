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

"""Validation utilities for teleop controllers and UI."""

from __future__ import annotations

from dataclasses import dataclass, field

from isaacsim.core.experimental.utils.stage import get_current_stage
from pxr import Sdf, Usd, UsdGeom, UsdPhysics


@dataclass
class ValidationResult:
    """Structured validation result for a single prim."""

    errors: list[str] = field(default_factory=list)
    warnings: list[str] = field(default_factory=list)
    blocks_tracking: bool = False

    @property
    def is_valid(self) -> bool:
        """Return True when no errors are present."""
        return not self.errors


def _validate_prim_exists(prim_path: str) -> tuple[Usd.Prim | None, ValidationResult]:
    result = ValidationResult()
    stage = get_current_stage()
    if not stage:
        result.errors.append("No stage available")
        return None, result

    if not prim_path or not Sdf.Path.IsValidPathString(prim_path):
        result.errors.append(f"Invalid prim path: '{prim_path}'")
        return None, result

    prim = stage.GetPrimAtPath(prim_path)
    if not prim or not prim.IsValid():
        result.errors.append(f"Prim not found at '{prim_path}'")
        return None, result

    return prim, result


def validate_floating_end_effector(prim_path: str) -> ValidationResult:
    """Validate a prim for floating rigid-body controller usage."""
    prim, result = _validate_prim_exists(prim_path)
    if not prim:
        return result

    if not prim.HasAPI(UsdPhysics.RigidBodyAPI):
        result.errors.append("Missing RigidBodyAPI")

    if not prim.HasAPI(UsdPhysics.MassAPI):
        result.warnings.append("Missing MassAPI (angular dynamics may not work)")

    xformable = UsdGeom.Xformable(prim)
    ops = xformable.GetOrderedXformOps() if xformable else []
    if not any(op.GetOpType() == UsdGeom.XformOp.TypeTranslate for op in ops):
        result.warnings.append("Missing translate xform op")
    if not any(op.GetOpType() == UsdGeom.XformOp.TypeOrient for op in ops):
        result.warnings.append("Missing orient xform op")

    return result


def validate_marker_path(prim_path: str) -> ValidationResult:
    """Validate a marker path for live tracking."""
    prim, result = _validate_prim_exists(prim_path)
    if not prim:
        return result

    rigid = 0
    collision = 0
    for desc in Usd.PrimRange(prim):
        if desc.HasAPI(UsdPhysics.RigidBodyAPI):
            rigid += 1
        if desc.HasAPI(UsdPhysics.CollisionAPI):
            collision += 1

    if rigid > 0:
        result.warnings.append(f"{rigid} RigidBody descendant(s)")
    if collision > 0:
        result.warnings.append(f"{collision} Collider descendant(s)")

    if rigid > 0 or collision > 0:
        result.blocks_tracking = True

    return result
