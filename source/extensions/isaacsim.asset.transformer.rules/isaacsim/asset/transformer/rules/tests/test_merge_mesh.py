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

"""Tests for MergeMeshRule."""

import os
import shutil
import tempfile

import omni.kit.test
from isaacsim.asset.transformer.rules.isaac_sim.merge_mesh import MergeMeshRule
from pxr import Gf, Usd, UsdGeom, UsdPhysics

from .common import _TEST_ADVANCED_USD


class TestMergeMeshRule(omni.kit.test.AsyncTestCase):
    """Async tests for MergeMeshRule."""

    async def setUp(self) -> None:
        """Create a temporary directory for test output."""
        self._tmpdir = tempfile.mkdtemp()
        self._success = False

    async def tearDown(self) -> None:
        """Remove temporary directory after successful tests."""
        if self._success:
            shutil.rmtree(self._tmpdir, ignore_errors=True)

    def _create_rule(self, stage: Usd.Stage) -> MergeMeshRule:
        return MergeMeshRule(
            source_stage=stage,
            package_root=self._tmpdir,
            destination_path="",
            args={"params": {}},
        )

    async def test_no_rigid_bodies_is_noop(self) -> None:
        """Rule should complete without error when stage has no rigid bodies."""
        stage = Usd.Stage.CreateInMemory()
        UsdGeom.Xform.Define(stage, "/World")
        stage.SetDefaultPrim(stage.GetPrimAtPath("/World"))

        rule = self._create_rule(stage)
        result = rule.process_rule()

        self.assertIsNone(result)
        log = rule.get_operation_log()
        self.assertTrue(
            any("merged 0 mesh group(s)" in entry for entry in log), f"Expected substring not found in log: {log}"
        )
        self._success = True

    async def test_rule_completes_on_stage_with_rigid_bodies(self) -> None:
        """Rule should run and report merged groups when rigid bodies with meshes exist."""
        stage = Usd.Stage.CreateInMemory()
        root = UsdGeom.Xform.Define(stage, "/World").GetPrim()
        stage.SetDefaultPrim(root)

        body = stage.DefinePrim("/World/body", "Xform")
        UsdPhysics.RigidBodyAPI.Apply(body)

        mesh_a = UsdGeom.Mesh.Define(stage, "/World/body/meshA")
        mesh_a.CreatePointsAttr().Set([Gf.Vec3f(0, 0, 0), Gf.Vec3f(1, 0, 0), Gf.Vec3f(0, 1, 0)])
        mesh_a.CreateFaceVertexCountsAttr().Set([3])
        mesh_a.CreateFaceVertexIndicesAttr().Set([0, 1, 2])

        mesh_b = UsdGeom.Mesh.Define(stage, "/World/body/meshB")
        mesh_b.CreatePointsAttr().Set([Gf.Vec3f(2, 0, 0), Gf.Vec3f(3, 0, 0), Gf.Vec3f(2, 1, 0)])
        mesh_b.CreateFaceVertexCountsAttr().Set([3])
        mesh_b.CreateFaceVertexIndicesAttr().Set([0, 1, 2])

        rule = self._create_rule(stage)
        result = rule.process_rule()

        self.assertIsNone(result)
        log = rule.get_operation_log()
        self.assertTrue(
            any("MergeMeshRule completed" in entry for entry in log), f"Expected substring not found in log: {log}"
        )
        self._success = True

    async def test_get_configuration_parameters_empty(self) -> None:
        """MergeMeshRule should have no configuration parameters."""
        stage = Usd.Stage.CreateInMemory()
        rule = self._create_rule(stage)
        self.assertEqual(rule.get_configuration_parameters(), [])
        self._success = True

    async def test_with_test_advanced_asset(self) -> None:
        """Rule should run on a real test asset without errors."""
        if not os.path.exists(_TEST_ADVANCED_USD):
            return

        temp_asset = os.path.join(self._tmpdir, "test_advanced.usda")
        shutil.copy(_TEST_ADVANCED_USD, temp_asset)
        stage = Usd.Stage.Open(temp_asset)

        rule = self._create_rule(stage)
        result = rule.process_rule()

        self.assertIsNone(result)
        log = rule.get_operation_log()
        self.assertTrue(
            any("MergeMeshRule completed" in entry for entry in log), f"Expected substring not found in log: {log}"
        )
        self._success = True
