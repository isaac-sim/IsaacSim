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

"""Integration test: dedup wiring in ValidationEngine suppresses variant fan-out duplicates."""

from __future__ import annotations

import omni.kit.test
from isaacsim.asset.validation.physics_rules import RigidBodyHasCollider
from omni.asset_validator.core import IssueSeverity, ValidationEngine
from pxr import Usd, UsdGeom, UsdPhysics


def _build_fanout_stage() -> Usd.Stage:
    """Build an in-memory stage with a 5-variant fan-out on a prim that lacks CollisionAPI.

    /Robot          — default prim, has variantSet 'pose' with 5 variants (v1..v5)
    /Robot/body     — child Xform with RigidBodyAPI enabled but no CollisionAPI

    When the ValidationEngine traverses with variants=True, it visits /Robot/body once
    per variant selection, producing 5 identical _AddError calls without dedup wiring and
    exactly 1 call with dedup wiring in place.
    """
    stage = Usd.Stage.CreateInMemory()

    robot = UsdGeom.Xform.Define(stage, "/Robot")
    stage.SetDefaultPrim(robot.GetPrim())

    vsets = robot.GetPrim().GetVariantSets()
    pose_vset = vsets.AddVariantSet("pose")
    for name in ("v1", "v2", "v3", "v4", "v5"):
        pose_vset.AddVariant(name)
    pose_vset.SetVariantSelection("v1")

    body = UsdGeom.Xform.Define(stage, "/Robot/body")
    rigid_body_api = UsdPhysics.RigidBodyAPI.Apply(body.GetPrim())
    rigid_body_api.GetRigidBodyEnabledAttr().Set(True)
    # Intentionally no UsdPhysics.CollisionAPI — triggers RigidBodyHasCollider

    return stage


class TestDedupIntegration(omni.kit.test.AsyncTestCase):
    """Proves that dedup wiring collapses 5 variant-fan-out duplicates into 1 event."""

    async def test_variant_fanout_deduped_to_one_event(self) -> None:
        """engine.validate() on a 5-variant stage with a missing CollisionAPI returns exactly 1 error."""
        stage = _build_fanout_stage()

        engine = ValidationEngine(init_rules=False, variants=True)
        engine.enable_rule(RigidBodyHasCollider)
        results = engine.validate(stage)

        # RigidBodyHasCollider._AddError maps to IssueSeverity.FAILURE in the engine;
        # filter both FAILURE and ERROR to be safe against future severity changes.
        errors = [i for i in results if i.severity in (IssueSeverity.FAILURE, IssueSeverity.ERROR)]
        self.assertEqual(len(errors), 1, f"Expected 1 deduped event, got {len(errors)}: {[i.message for i in errors]}")
