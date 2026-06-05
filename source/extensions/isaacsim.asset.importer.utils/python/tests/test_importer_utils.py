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

"""Test importer utils functionality."""

import omni.kit.test
import omni.usd
from isaacsim.asset.importer.utils.impl import importer_utils
from isaacsim.asset.importer.utils.impl.physx_types import PhysxAttr, PhysxSchema
from pxr import Usd, UsdGeom, UsdPhysics


class TestImporterUtils(omni.kit.test.AsyncTestCase):
    """Test helpers in :mod:`isaacsim.asset.importer.utils.impl.importer_utils`."""

    async def setUp(self) -> None:
        """Prepare a new USD stage for each test."""
        await omni.usd.get_context().new_stage_async()
        await omni.kit.app.get_app().next_update_async()

    async def test_delete_scope_reparents_children(self) -> None:
        """Reparent scope children to the parent prim before deleting the scope."""
        stage = Usd.Stage.CreateInMemory()
        UsdGeom.Xform.Define(stage, "/World")
        UsdGeom.Scope.Define(stage, "/World/Scope")
        UsdGeom.Xform.Define(stage, "/World/Scope/Child")

        importer_utils.delete_scope(stage, "/World/Scope")

        self.assertFalse(stage.GetPrimAtPath("/World/Scope").IsValid())
        self.assertTrue(stage.GetPrimAtPath("/World/Child").IsValid())

    async def test_add_joint_schemas(self) -> None:
        """Apply joint-related schemas to prismatic and revolute joints."""
        stage = Usd.Stage.CreateInMemory()
        UsdGeom.Xform.Define(stage, "/World")
        revolute = UsdPhysics.RevoluteJoint.Define(stage, "/World/Revolute").GetPrim()
        prismatic = UsdPhysics.PrismaticJoint.Define(stage, "/World/Prismatic").GetPrim()

        importer_utils.add_joint_schemas(stage)

        self.assertTrue(revolute.HasAPI(PhysxSchema.JOINT_API))
        self.assertTrue(revolute.HasAPI(UsdPhysics.DriveAPI, "angular"))
        self.assertTrue(revolute.HasAPI(PhysxSchema.JOINT_STATE_API, "angular"))

        self.assertTrue(prismatic.HasAPI(PhysxSchema.JOINT_API))
        self.assertTrue(prismatic.HasAPI(UsdPhysics.DriveAPI, "linear"))
        self.assertTrue(prismatic.HasAPI(PhysxSchema.JOINT_STATE_API, "linear"))

    async def test_add_rigid_body_schemas(self) -> None:
        """Apply MassAPI to rigid bodies discovered via stage traversal."""
        stage = Usd.Stage.CreateInMemory()
        UsdGeom.Xform.Define(stage, "/World")
        body_prim = stage.DefinePrim("/World/Body", "Xform")
        UsdPhysics.RigidBodyAPI.Apply(body_prim)
        importer_utils.add_rigid_body_schemas(stage)

        self.assertTrue(body_prim.HasAPI(UsdPhysics.RigidBodyAPI))
        self.assertTrue(body_prim.HasAPI(UsdPhysics.MassAPI))

    async def test_enable_self_collision_applies_articulation_api(self) -> None:
        """Enable self-collision on the default prim when missing roots.

        The PhysX-specific ``PhysxArticulationAPI`` is intentionally not
        authored here; the runtime consumes ``NewtonArticulationRootAPI``
        directly.
        """
        stage = Usd.Stage.CreateInMemory()
        default_prim = UsdGeom.Xform.Define(stage, "/World").GetPrim()
        stage.SetDefaultPrim(default_prim)

        updated = importer_utils.enable_self_collision(stage, enabled=True)

        self.assertEqual(updated, 1)
        self.assertTrue(default_prim.HasAPI("PhysicsArticulationRootAPI"))
        self.assertFalse(default_prim.HasAPI(PhysxSchema.ARTICULATION_API))
        self.assertTrue(default_prim.HasAPI("NewtonArticulationRootAPI"))

        newton_attr = default_prim.GetAttribute("newton:selfCollisionEnabled")
        self.assertTrue(newton_attr.IsValid())
        self.assertTrue(newton_attr.Get())

    async def test_parse_robot_name_basic(self) -> None:
        """Standard names are returned as the stem with the extension stripped."""
        self.assertEqual(importer_utils.parse_robot_name("/tmp/franka.urdf", expected_extension=".urdf"), "franka")
        self.assertEqual(
            importer_utils.parse_robot_name("/tmp/sub/dir/humanoid.xml", expected_extension=".xml"),
            "humanoid",
        )

    async def test_parse_robot_name_case_insensitive_extension(self) -> None:
        """Extension matching is case-insensitive."""
        self.assertEqual(importer_utils.parse_robot_name("/tmp/Franka.URDF", expected_extension=".urdf"), "Franka")

    async def test_parse_robot_name_hidden_file(self) -> None:
        """Hidden-style names (leading dot) still produce a non-empty robot name."""
        self.assertEqual(importer_utils.parse_robot_name("/tmp/.franka.urdf", expected_extension=".urdf"), "franka")
        self.assertEqual(importer_utils.parse_robot_name("/tmp/..humanoid.xml", expected_extension=".xml"), "humanoid")

    async def test_parse_robot_name_wrong_extension_raises(self) -> None:
        """Non-matching extensions raise ``ValueError``."""
        with self.assertRaises(ValueError):
            importer_utils.parse_robot_name("/tmp/franka.txt", expected_extension=".urdf")
        with self.assertRaises(ValueError):
            importer_utils.parse_robot_name("/tmp/franka", expected_extension=".urdf")
        with self.assertRaises(ValueError):
            importer_utils.parse_robot_name("/tmp/humanoid.urdf", expected_extension=".xml")

    async def test_parse_robot_name_dotfile_only_raises(self) -> None:
        """A bare dotfile basename (e.g. ``.urdf``) is treated by ``os.path.splitext`` as
        a hidden filename with no extension, so it fails the extension check."""
        with self.assertRaisesRegex(ValueError, "no extension"):
            importer_utils.parse_robot_name("/tmp/.urdf", expected_extension=".urdf")

    async def test_parse_robot_name_warns_on_dotted_stem(self) -> None:
        """A stem with embedded ``.`` characters logs a warning but still returns the stem."""
        with self.assertLogs("isaacsim.asset.importer.utils.impl.importer_utils", level="WARNING") as cm:
            result = importer_utils.parse_robot_name("/tmp/franka.v2.urdf", expected_extension=".urdf")
        self.assertEqual(result, "franka.v2")
        self.assertTrue(any("'.' characters" in msg for msg in cm.output))
