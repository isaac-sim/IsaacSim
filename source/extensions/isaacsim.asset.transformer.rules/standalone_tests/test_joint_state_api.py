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

"""Joint-state authoring regressions for standalone USD without Kit."""

from __future__ import annotations

import tempfile
import unittest

from isaacsim.asset.transformer.rules.isaac_sim.joint_state_api import JointStateAPIRule
from pxr import Usd, UsdPhysics


class TestJointStateAPI(unittest.TestCase):
    """Exercise joint-state tokens without importing or registering PhysX schemas."""

    def test_authors_joint_state_and_preserves_existing_schemas(self) -> None:
        """Apply the correct instances and leave unrelated applied schemas intact."""
        stage = Usd.Stage.CreateInMemory()
        revolute = UsdPhysics.RevoluteJoint.Define(stage, "/revolute").GetPrim()
        prismatic = UsdPhysics.PrismaticJoint.Define(stage, "/prismatic").GetPrim()
        UsdPhysics.DriveAPI.Apply(revolute, "angular")

        with tempfile.TemporaryDirectory() as output_dir:
            rule = JointStateAPIRule(stage, output_dir, "", {"params": {}})
            self.assertIsNone(rule.process_rule())

        self.assertIn("PhysicsJointStateAPI:angular", revolute.GetMetadata("apiSchemas").GetAppliedItems())
        self.assertNotIn("PhysicsJointStateAPI:linear", revolute.GetMetadata("apiSchemas").GetAppliedItems())
        self.assertIn("PhysicsJointStateAPI:linear", prismatic.GetMetadata("apiSchemas").GetAppliedItems())
        self.assertNotIn("PhysicsJointStateAPI:angular", prismatic.GetMetadata("apiSchemas").GetAppliedItems())
        self.assertTrue(revolute.HasAPI(UsdPhysics.DriveAPI, "angular"))

    def test_existing_joint_state_is_idempotent(self) -> None:
        """Repeated runs preserve existing tokens without duplicate layer edits."""
        stage = Usd.Stage.CreateInMemory()
        joint = UsdPhysics.RevoluteJoint.Define(stage, "/joint").GetPrim()
        joint.AddAppliedSchema("PhysicsJointStateAPI:angular")
        original = stage.GetRootLayer().ExportToString()

        with tempfile.TemporaryDirectory() as output_dir:
            for _ in range(2):
                rule = JointStateAPIRule(stage, output_dir, "", {"params": {}})
                rule.process_rule()
                self.assertTrue(any("completed: 0 applied" in entry for entry in rule.get_operation_log()))
                self.assertEqual(stage.GetRootLayer().ExportToString(), original)

    def test_skips_fixed_and_unsupported_joints(self) -> None:
        """Fixed, spherical, and generic joints receive no joint-state tokens."""
        stage = Usd.Stage.CreateInMemory()
        joints = [
            schema.Define(stage, f"/joint{index}").GetPrim()
            for index, schema in enumerate((UsdPhysics.FixedJoint, UsdPhysics.SphericalJoint, UsdPhysics.Joint))
        ]
        with tempfile.TemporaryDirectory() as output_dir:
            rule = JointStateAPIRule(stage, output_dir, "", {"params": {}})
            rule.process_rule()

        for joint in joints:
            schemas = joint.GetMetadata("apiSchemas")
            tokens = schemas.GetAppliedItems() if schemas is not None else []
            self.assertFalse(any(token.startswith("PhysicsJointStateAPI:") for token in tokens))


if __name__ == "__main__":
    unittest.main()
