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

"""Apply ``PhysicsJointStateAPI`` to non-fixed prismatic/revolute joints missing it.

The validation rule ``JointHasJointStateAPI`` flags every non-fixed prismatic or
revolute joint that lacks a ``JointStateAPI`` with the correct actuator instance
(``"linear"`` for prismatic, ``"angular"`` for revolute). URDF/MJCF importers do
not author this API, so this transformer rule supplies it on the working stage.

The newly-applied schema token (``PhysicsJointStateAPI:<instance>``) is picked up
by the downstream ``Route Physics Schemas`` rule (matching ``Physics.*``) and
lands in ``payloads/Physics/physics.usda``; the existing
``Delete Physics Drive and Joint State APIs`` rule strips it from
``mujoco.usda`` unchanged.

This rule runs after joints have their final pose corrections (see
``PhysicsJointPoseFixRule``) and before any schema-routing rule, so applied APIs
are present on the working stage when routing decisions are made.
"""

from __future__ import annotations

import os

from isaacsim.asset.importer.utils.impl.physx_types import PhysxSchema as PhysxSchemaTokens
from isaacsim.asset.transformer import RuleConfigurationParam, RuleInterface
from pxr import Usd, UsdPhysics


def _has_joint_state_api(prim: Usd.Prim, instance: str) -> bool:
    return prim.HasAPI(PhysxSchemaTokens.JOINT_STATE_API, instance)


def _apply_joint_state_api(prim: Usd.Prim, instance: str) -> None:
    prim.ApplyAPI(PhysxSchemaTokens.JOINT_STATE_API, instance)


class JointStateAPIRule(RuleInterface):
    """Apply ``PhysicsJointStateAPI`` to non-fixed prismatic/revolute joints.

    Mirrors the validation rule's joint-type dispatch:

    * ``UsdPhysics.PrismaticJoint`` -> ``prim.ApplyAPI("PhysicsJointStateAPI", "linear")``
    * ``UsdPhysics.RevoluteJoint``  -> ``prim.ApplyAPI("PhysicsJointStateAPI", "angular")``
    * ``UsdPhysics.FixedJoint``      -> skip
    * Any other ``UsdPhysics.Joint`` subtype (D6, spherical) -> skip

    Idempotent: a joint that already has the relevant ``JointStateAPI``
    instance applied is left untouched.
    """

    def get_configuration_parameters(self) -> list[RuleConfigurationParam]:
        """Return configuration parameters for this rule.

        Returns:
            An empty list; this rule has no user-tunable parameters.
        """
        return []

    def process_rule(self) -> str | None:
        """Traverse joints on the working stage and apply missing ``JointStateAPI`` schemas.

        Returns:
            ``None`` on success. The rule never raises for ordinary joint configurations.
        """
        entry_stage = self.source_stage
        joint_prims = [prim for prim in entry_stage.Traverse() if prim.IsA(UsdPhysics.Joint)]
        if not joint_prims:
            self.log_operation("JointStateAPIRule: no physics joints found on entry stage")
            return None

        self.log_operation(f"JointStateAPIRule start joints={len(joint_prims)}")

        edit_layer = entry_stage.GetEditTarget().GetLayer()
        if not edit_layer:
            edit_layer = entry_stage.GetRootLayer()

        applied_count = 0
        skipped_count = 0

        for prim in joint_prims:
            if prim.IsA(UsdPhysics.FixedJoint):
                skipped_count += 1
                continue
            if prim.IsA(UsdPhysics.PrismaticJoint):
                instance = "linear"
            elif prim.IsA(UsdPhysics.RevoluteJoint):
                instance = "angular"
            else:
                # D6, spherical, and bare UsdPhysics.Joint -- matches validation rule short-circuit.
                skipped_count += 1
                continue

            if _has_joint_state_api(prim, instance):
                skipped_count += 1
                continue

            _apply_joint_state_api(prim, instance)
            applied_count += 1
            self.log_operation(f"JointStateAPIRule applied '{instance}' on {prim.GetPath()}")

        if edit_layer and edit_layer.dirty:
            edit_path = getattr(edit_layer, "realPath", None) or getattr(edit_layer, "identifier", None)
            if edit_path and os.path.isfile(edit_path):
                edit_layer.Save()
            self.add_affected_stage(edit_path or str(edit_layer.identifier))

        self.log_operation(f"JointStateAPIRule completed: {applied_count} applied, {skipped_count} skipped")
        return None
