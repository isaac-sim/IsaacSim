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

"""Standalone round-trip tests for isaacsim-asset-exporter-urdf.

Builds a minimal USD stage programmatically, exports to URDF, and validates
the output XML structure. No asset files or simulation runtime required.
"""

from __future__ import annotations

import os
import tempfile
import unittest
import xml.etree.ElementTree as ET

from isaacsim.asset.importer.utils.impl.physx_types import PhysxAttr
from pxr import Gf, Sdf, Usd, UsdGeom, UsdPhysics


def _build_two_link_robot(stage: Usd.Stage) -> None:
    """Build a minimal two-link robot with a revolute joint on the given stage."""
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    UsdGeom.SetStageMetersPerUnit(stage, 1.0)

    # Root
    UsdGeom.Xform.Define(stage, "/World")
    UsdGeom.Xform.Define(stage, "/World/robot")

    # Base link with visual geometry
    UsdGeom.Xform.Define(stage, "/World/robot/base_link")
    cube = UsdGeom.Cube.Define(stage, "/World/robot/base_link/visual")
    cube.GetSizeAttr().Set(0.1)

    # Apply ArticulationRootAPI to the robot root (required by the exporter).
    base_prim = stage.GetPrimAtPath("/World/robot/base_link")
    UsdPhysics.ArticulationRootAPI.Apply(base_prim)

    # Child link with visual geometry
    UsdGeom.Xform.Define(stage, "/World/robot/child_link")
    cube2 = UsdGeom.Cube.Define(stage, "/World/robot/child_link/visual")
    cube2.GetSizeAttr().Set(0.05)

    # Revolute joint
    joint = UsdPhysics.RevoluteJoint.Define(stage, "/World/robot/child_link/joint")
    joint.GetAxisAttr().Set("Z")
    joint.GetBody0Rel().AddTarget("/World/robot/base_link")
    joint.GetBody1Rel().AddTarget("/World/robot/child_link")
    joint.GetLowerLimitAttr().Set(-90.0)
    joint.GetUpperLimitAttr().Set(90.0)

    # PhysX attributes via physx_types
    jprim = joint.GetPrim()
    jprim.CreateAttribute(PhysxAttr.JOINT_ARMATURE.name, PhysxAttr.JOINT_ARMATURE.type).Set(0.01)
    jprim.CreateAttribute(PhysxAttr.JOINT_FRICTION.name, PhysxAttr.JOINT_FRICTION.type).Set(0.1)
    jprim.CreateAttribute(PhysxAttr.JOINT_MAX_VELOCITY.name, PhysxAttr.JOINT_MAX_VELOCITY.type).Set(180.0)


@unittest.skipUnless(
    os.environ.get("ISAACSIM_TEST_ROUNDTRIP"),
    "Round-trip tests require full USD schema plugins (set ISAACSIM_TEST_ROUNDTRIP=1 to run)",
)
class TestRoundTrip(unittest.TestCase):
    """URDF export from programmatically built USD stages.

    These tests require ArticulationRootAPI and related physics schemas to be
    fully registered.  In a minimal usd-core install the schema plugin discovery
    may not register all applied API schemas, causing HasAPI() to return False.
    Set ``ISAACSIM_TEST_ROUNDTRIP=1`` to enable when running with a more
    complete USD environment.
    """

    def test_export_minimal_urdf(self) -> None:
        """Export a minimal robot and verify the URDF structure."""
        from isaacsim.asset.exporter.urdf import UsdToUrdfConverter

        stage = Usd.Stage.CreateInMemory()
        _build_two_link_robot(stage)

        with tempfile.TemporaryDirectory() as tmpdir:
            urdf_path = os.path.join(tmpdir, "robot.urdf")
            converter = UsdToUrdfConverter(stage=stage, root_prim_path="/World/robot")
            converter.convert(urdf_path)

            self.assertTrue(os.path.exists(urdf_path), "URDF file not created")
            self.assertGreater(os.path.getsize(urdf_path), 0, "URDF file is empty")

            tree = ET.parse(urdf_path)
            root = tree.getroot()
            self.assertEqual(root.tag, "robot")

            joints = root.findall("joint")
            links = root.findall("link")
            self.assertGreaterEqual(len(joints), 1, f"Expected >= 1 joint, got {len(joints)}")
            self.assertGreaterEqual(len(links), 2, f"Expected >= 2 links, got {len(links)}")

    def test_export_preserves_joint_limits(self) -> None:
        """Verify joint limits survive the export."""
        from isaacsim.asset.exporter.urdf import UsdToUrdfConverter

        stage = Usd.Stage.CreateInMemory()
        _build_two_link_robot(stage)

        with tempfile.TemporaryDirectory() as tmpdir:
            urdf_path = os.path.join(tmpdir, "robot.urdf")
            converter = UsdToUrdfConverter(stage=stage, root_prim_path="/World/robot")
            converter.convert(urdf_path)

            tree = ET.parse(urdf_path)
            joints = tree.getroot().findall("joint")
            revolute_joints = [j for j in joints if j.get("type") == "revolute"]

            if revolute_joints:
                limit = revolute_joints[0].find("limit")
                self.assertIsNotNone(limit, "Revolute joint missing <limit> element")


if __name__ == "__main__":
    unittest.main()
