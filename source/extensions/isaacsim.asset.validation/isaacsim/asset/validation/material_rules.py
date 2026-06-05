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

"""Material validation rules for Isaac Sim assets."""

from collections.abc import Iterator

from omni.asset_validator.core import registerRule
from pxr import Usd, UsdShade

from .util import DedupBaseRuleChecker


def traverse_without_references_payloads(prim: "Usd.Prim") -> Iterator:
    """Recursively traverse prim hierarchy excluding references and payloads.

    Args:
        prim: The USD prim to start traversal from.

    Yields:
        USD prims in the hierarchy that don't have references or payloads.

    Returns:
        Iterator over the same prims as yielded (depth-first, children with
        authored references or payloads skipped).
    """
    yield prim

    for child in prim.GetChildren():
        # Skip children that have references or payloads
        if child.HasAuthoredReferences() or child.HasAuthoredPayloads():
            continue

        # Recursively traverse children
        yield from traverse_without_references_payloads(child)


@registerRule("IsaacSim.SimReadyAssetRules")
class NoNestedMaterials(DedupBaseRuleChecker):
    """Validates that materials don't contain nested materials.

    This rule checks that UsdShade.Material prims don't have child prims that are also
    materials, which can cause unexpected rendering behavior.
    """

    def CheckPrim(self, prim: Usd.Prim) -> None:  # noqa: N802
        """Check if a material prim contains nested materials.

        Args:
            prim: The USD prim to validate.
        """
        if prim.IsA(UsdShade.Material):
            for child_prim in Usd.PrimRange(prim):
                if child_prim.GetPath() == prim.GetPath():
                    continue
                if child_prim.IsA(UsdShade.Material):
                    self._AddWarning(
                        message=f"Material {prim.GetPath()} has nested material {child_prim.GetPath()}",
                        at=child_prim,
                    )


@registerRule("IsaacSim.SimReadyAssetRules")
class MaterialsOnTopLevelOnly(DedupBaseRuleChecker):
    """Validates that materials are only defined in the top-level Looks prim.

    This rule checks that all UsdShade.Material prims are direct children of the
    top-level Looks prim, following USD best practices for material organization.
    """

    def CheckStage(self, stage: Usd.Stage) -> None:  # noqa: N802
        """Check if all materials are properly organized in the Looks prim.

        Args:
            stage: The USD stage to validate.
        """
        default_prim = stage.GetDefaultPrim()
        looks_prim = stage.GetPrimAtPath(f"{default_prim.GetPath()}/Looks")
        if not looks_prim:
            self._AddInfo(
                message="No Looks prim found in the stage, skipping MaterialsOnTopLevelOnlyRule",
            )
            return

        material_set = set()
        for prim in looks_prim.GetChildren():
            if prim.IsA(UsdShade.Material):
                material_set.add(prim)

        for prim in traverse_without_references_payloads(default_prim):
            if prim.IsA(UsdShade.Material):
                if prim not in material_set:
                    self._AddError(
                        message=f"Material {prim.GetPath()} is not in the top level Looks prim",
                        at=prim,
                    )
