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

"""Tests for :class:`JointStateAPIRule`."""

from __future__ import annotations

import os
import shutil
import tempfile

import omni.kit.test
from isaacsim.asset.transformer.rules.isaac_sim.joint_state_api import JointStateAPIRule
from pxr import PhysxSchema, Sdf, Usd, UsdGeom, UsdPhysics


def _build_two_link_stage(path: str) -> Usd.Stage:
    """Build a stage with two rigid-body links and no joints (caller adds joints as needed).

    Args:
        path: Output ``.usda`` path for the stage.

    Returns:
        The newly-created ``Usd.Stage`` (already saved to ``path``).
    """
    stage = Usd.Stage.CreateNew(path)
    robot = UsdGeom.Xform.Define(stage, "/Robot")
    stage.SetDefaultPrim(robot.GetPrim())

    link0 = UsdGeom.Xform.Define(stage, "/Robot/link0")
    UsdPhysics.RigidBodyAPI.Apply(link0.GetPrim())
    link1 = UsdGeom.Xform.Define(stage, "/Robot/link1")
    UsdPhysics.RigidBodyAPI.Apply(link1.GetPrim())
    return stage


class TestJointStateAPIRule(omni.kit.test.AsyncTestCase):
    """Async tests for :class:`JointStateAPIRule`."""

    async def setUp(self) -> None:
        """Create a temporary directory for per-test stage files."""
        self._tmpdir = tempfile.mkdtemp()
        self._success = False

    async def tearDown(self) -> None:
        """Clean up after a successful test; leave artefacts on failure for debugging."""
        if self._success:
            shutil.rmtree(self._tmpdir, ignore_errors=True)

    def _create_rule(self, stage: Usd.Stage) -> JointStateAPIRule:
        """Instantiate the rule wired to ``stage``.

        Args:
            stage: Working stage to pass as ``source_stage``.

        Returns:
            A configured :class:`JointStateAPIRule` instance.
        """
        return JointStateAPIRule(
            source_stage=stage,
            package_root=self._tmpdir,
            destination_path="payloads",
            args={"params": {}},
        )

    async def test_get_configuration_parameters_empty(self) -> None:
        """``get_configuration_parameters()`` returns an empty list."""
        stage = Usd.Stage.CreateInMemory()
        rule = self._create_rule(stage)
        self.assertEqual(rule.get_configuration_parameters(), [])
        self._success = True

    async def test_applies_linear_to_prismatic(self) -> None:
        """The rule applies ``JointStateAPI("linear")`` to a prismatic joint."""
        stage_path = os.path.join(self._tmpdir, "prismatic.usda")
        stage = _build_two_link_stage(stage_path)
        prism = UsdPhysics.PrismaticJoint.Define(stage, "/Robot/joints/prism1")
        prism.CreateBody0Rel().SetTargets([Sdf.Path("/Robot/link0")])
        prism.CreateBody1Rel().SetTargets([Sdf.Path("/Robot/link1")])

        rule = self._create_rule(stage)
        rule.process_rule()

        prim = stage.GetPrimAtPath("/Robot/joints/prism1")
        self.assertTrue(prim.HasAPI(PhysxSchema.JointStateAPI, "linear"))
        self.assertFalse(prim.HasAPI(PhysxSchema.JointStateAPI, "angular"))
        self._success = True

    async def test_applies_angular_to_revolute(self) -> None:
        """The rule applies ``JointStateAPI("angular")`` to a revolute joint."""
        stage_path = os.path.join(self._tmpdir, "revolute.usda")
        stage = _build_two_link_stage(stage_path)
        rev = UsdPhysics.RevoluteJoint.Define(stage, "/Robot/joints/rev1")
        rev.CreateBody0Rel().SetTargets([Sdf.Path("/Robot/link0")])
        rev.CreateBody1Rel().SetTargets([Sdf.Path("/Robot/link1")])
        rev.CreateAxisAttr().Set("Z")

        rule = self._create_rule(stage)
        rule.process_rule()

        prim = stage.GetPrimAtPath("/Robot/joints/rev1")
        self.assertTrue(prim.HasAPI(PhysxSchema.JointStateAPI, "angular"))
        self.assertFalse(prim.HasAPI(PhysxSchema.JointStateAPI, "linear"))
        self._success = True

    async def test_skips_fixed(self) -> None:
        """Fixed joints receive no ``JointStateAPI``."""
        stage_path = os.path.join(self._tmpdir, "fixed.usda")
        stage = _build_two_link_stage(stage_path)
        fixed = UsdPhysics.FixedJoint.Define(stage, "/Robot/joints/fixed1")
        fixed.CreateBody0Rel().SetTargets([Sdf.Path("/Robot/link0")])
        fixed.CreateBody1Rel().SetTargets([Sdf.Path("/Robot/link1")])

        rule = self._create_rule(stage)
        rule.process_rule()

        prim = stage.GetPrimAtPath("/Robot/joints/fixed1")
        self.assertFalse(prim.HasAPI(PhysxSchema.JointStateAPI, "linear"))
        self.assertFalse(prim.HasAPI(PhysxSchema.JointStateAPI, "angular"))
        self._success = True

    async def test_skips_generic_joint(self) -> None:
        """A bare ``UsdPhysics.Joint`` (no specific subtype) receives no ``JointStateAPI``."""
        stage_path = os.path.join(self._tmpdir, "generic.usda")
        stage = _build_two_link_stage(stage_path)
        generic = UsdPhysics.Joint.Define(stage, "/Robot/joints/generic1")
        generic.CreateBody0Rel().SetTargets([Sdf.Path("/Robot/link0")])
        generic.CreateBody1Rel().SetTargets([Sdf.Path("/Robot/link1")])

        rule = self._create_rule(stage)
        rule.process_rule()

        prim = stage.GetPrimAtPath("/Robot/joints/generic1")
        self.assertFalse(prim.HasAPI(PhysxSchema.JointStateAPI, "linear"))
        self.assertFalse(prim.HasAPI(PhysxSchema.JointStateAPI, "angular"))
        self._success = True

    async def test_idempotent_on_existing_api(self) -> None:
        """A joint that already has the API is not re-applied; two runs produce the same result."""
        stage_path = os.path.join(self._tmpdir, "idempotent.usda")
        stage = _build_two_link_stage(stage_path)
        rev = UsdPhysics.RevoluteJoint.Define(stage, "/Robot/joints/rev1")
        rev.CreateBody0Rel().SetTargets([Sdf.Path("/Robot/link0")])
        rev.CreateBody1Rel().SetTargets([Sdf.Path("/Robot/link1")])
        PhysxSchema.JointStateAPI.Apply(rev.GetPrim(), "angular")

        # First run: should be a no-op on this prim (already applied).
        rule1 = self._create_rule(stage)
        rule1.process_rule()
        log1 = rule1.get_operation_log()
        self.assertTrue(
            any("completed: 0 applied" in m for m in log1),
            f"Expected '0 applied' in log after first run; got {log1!r}",
        )

        # Second run: same expectation (proves full idempotency).
        rule2 = self._create_rule(stage)
        rule2.process_rule()
        log2 = rule2.get_operation_log()
        self.assertTrue(
            any("completed: 0 applied" in m for m in log2),
            f"Expected '0 applied' in log after second run; got {log2!r}",
        )

        # Schema still present after both runs.
        prim = stage.GetPrimAtPath("/Robot/joints/rev1")
        self.assertTrue(prim.HasAPI(PhysxSchema.JointStateAPI, "angular"))
        self._success = True
