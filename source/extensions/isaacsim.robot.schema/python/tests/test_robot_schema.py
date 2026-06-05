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

"""Tests for Isaac robot schema helpers and utilities."""

from __future__ import annotations

import os
import tempfile
from unittest import mock

import omni.kit.app
import omni.kit.test
import omni.usd
import usd.schema.isaac as isaac_schema
from pxr import Gf, Sdf, Usd, UsdGeom, UsdPhysics
from usd.schema.isaac import robot_schema
from usd.schema.isaac.robot_schema import utils as robot_utils


class TestRobotSchemaUtils(omni.kit.test.AsyncTestCase):
    """Async tests for robot schema helper utilities."""

    async def setUp(self) -> None:
        """Set up a fresh USD stage for each test."""
        await omni.usd.get_context().new_stage_async()
        await omni.kit.app.get_app().next_update_async()
        self._stage = omni.usd.get_context().get_stage()

    def _create_robot_with_joint(self) -> tuple[UsdGeom.Xform, UsdGeom.Xform, UsdPhysics.Joint]:
        robot_prim = UsdGeom.Xform.Define(self._stage, "/World/Robot")
        UsdPhysics.RigidBodyAPI.Apply(robot_prim.GetPrim())
        UsdPhysics.ArticulationRootAPI.Apply(robot_prim.GetPrim())

        link_prim = UsdGeom.Xform.Define(self._stage, "/World/Robot/Link1")
        UsdPhysics.RigidBodyAPI.Apply(link_prim.GetPrim())

        joint = UsdPhysics.Joint.Define(self._stage, "/World/Robot/joint1")
        joint.CreateBody0Rel().SetTargets([robot_prim.GetPrim().GetPath()])
        joint.CreateBody1Rel().SetTargets([link_prim.GetPrim().GetPath()])
        joint.CreateLocalPos0Attr().Set(Gf.Vec3f(1.0, 0.0, 0.0))
        joint.CreateLocalRot0Attr().Set(Gf.Quatf(1.0, 0.0, 0.0, 0.0))
        joint.CreateLocalPos1Attr().Set(Gf.Vec3f(0.0, 0.0, 0.0))
        joint.CreateLocalRot1Attr().Set(Gf.Quatf(1.0, 0.0, 0.0, 0.0))

        return robot_prim, link_prim, joint

    def _apply_limit(self, joint_prim: Usd.Prim, axis_token: str, low: float = -1.0, high: float = 1.0) -> None:
        limit_api = UsdPhysics.LimitAPI.Apply(joint_prim, axis_token)
        limit_api.CreateLowAttr().Set(low)
        limit_api.CreateHighAttr().Set(high)

    async def test_apply_robot_api_populates_relationships(self) -> None:
        """Verify ApplyRobotAPI creates links and joints relationships."""
        robot_xform, link_xform, joint = self._create_robot_with_joint()
        robot_prim = robot_xform.GetPrim()
        link_prim = link_xform.GetPrim()
        joint_prim = joint.GetPrim()

        robot_schema.ApplyRobotAPI(robot_prim)

        self.assertTrue(robot_prim.HasAPI(robot_schema.Classes.ROBOT_API.value))
        self.assertTrue(robot_prim.GetAttribute(robot_schema.Attributes.DESCRIPTION.name))
        self.assertTrue(robot_prim.GetAttribute(robot_schema.Attributes.NAMESPACE.name))

        robot_links = robot_prim.GetRelationship(robot_schema.Relations.ROBOT_LINKS.name).GetTargets()
        robot_joints = robot_prim.GetRelationship(robot_schema.Relations.ROBOT_JOINTS.name).GetTargets()

        self.assertIn(robot_prim.GetPath(), robot_links)
        self.assertIn(link_prim.GetPath(), robot_links)
        self.assertIn(joint_prim.GetPath(), robot_joints)

        self.assertTrue(link_prim.HasAPI(robot_schema.Classes.LINK_API.value))
        self.assertTrue(joint_prim.HasAPI(robot_schema.Classes.JOINT_API.value))

    async def test_update_deprecated_joint_dof_order(self) -> None:
        """Verify deprecated DOF offset attributes are migrated."""
        _, _, joint = self._create_robot_with_joint()
        joint_prim = joint.GetPrim()

        self._apply_limit(joint_prim, UsdPhysics.Tokens.transX, low=-1.0, high=1.0)
        self._apply_limit(joint_prim, UsdPhysics.Tokens.rotX, low=-1.0, high=1.0)
        self._apply_limit(joint_prim, UsdPhysics.Tokens.transY, low=-1.0, high=1.0)
        self._apply_limit(joint_prim, UsdPhysics.Tokens.rotY, low=1.0, high=-1.0)

        joint_prim.CreateAttribute("isaac:physics:Tr_X:DoFOffset", Sdf.ValueTypeNames.Int).Set(0)
        joint_prim.CreateAttribute("isaac:physics:Rot_X:DoFOffset", Sdf.ValueTypeNames.Int).Set(0)
        joint_prim.CreateAttribute("isaac:physics:Tr_Y:DoFOffset", Sdf.ValueTypeNames.Int).Set(1)

        updated = robot_utils.UpdateDeprecatedJointDofOrder(joint_prim)
        self.assertTrue(updated)

        dof_attr = joint_prim.GetAttribute("isaac:physics:DofOffsetOpOrder")
        self.assertEqual(list(dof_attr.Get()), ["TransX", "RotX", "TransY"])

        self.assertFalse(robot_utils.UpdateDeprecatedJointDofOrder(joint_prim))

    async def test_get_all_robot_links_and_joints_warns_on_missing(self) -> None:
        """Warn when links or joints are missing from schema relationships."""
        robot_xform, link_xform, joint = self._create_robot_with_joint()
        robot_prim = robot_xform.GetPrim()

        robot_prim.AddAppliedSchema(robot_schema.Classes.ROBOT_API.value)

        robot_utils._warned_missing_schema_links.discard("/World/Robot")
        robot_utils._warned_missing_schema_joints.discard("/World/Robot")

        with mock.patch("usd.schema.isaac.robot_schema.utils.logger") as mock_logger:
            links = robot_utils.GetAllRobotLinks(self._stage, robot_prim)
            joints = robot_utils.GetAllRobotJoints(self._stage, robot_prim)

            self.assertEqual({str(prim.GetPath()) for prim in links}, {"/World/Robot", "/World/Robot/Link1"})
            self.assertEqual({str(prim.GetPath()) for prim in joints}, {"/World/Robot/joint1"})

            warnings = [call.args[0] for call in mock_logger.warning.call_args_list]
            self.assertTrue(any("links missing from schema relationship" in msg for msg in warnings))
            self.assertTrue(any("joints missing from schema relationship" in msg for msg in warnings))

    async def test_generate_robot_link_tree_and_get_links_from_joint(self) -> None:
        """Verify link tree generation and joint link retrieval."""
        robot_xform, link_xform, joint = self._create_robot_with_joint()
        robot_prim = robot_xform.GetPrim()
        joint_prim = joint.GetPrim()

        robot_schema.ApplyRobotAPI(robot_prim)

        tree_root = robot_utils.GenerateRobotLinkTree(self._stage, robot_prim)
        self.assertIsNotNone(tree_root)

        before_links, after_links = robot_utils.GetLinksFromJoint(tree_root, joint_prim)
        before_paths = {str(prim.GetPath()) for prim in before_links}
        after_paths = {str(prim.GetPath()) for prim in after_links}

        self.assertIn("/World/Robot", before_paths)
        self.assertIn("/World/Robot/Link1", after_paths)

    async def test_get_joint_pose_and_body_relationship(self) -> None:
        """Verify joint pose extraction and body relationship lookup."""
        robot_xform, _, joint = self._create_robot_with_joint()
        robot_prim = robot_xform.GetPrim()
        joint_prim = joint.GetPrim()

        body_path = robot_utils.GetJointBodyRelationship(joint_prim, 0)
        self.assertEqual(body_path, robot_prim.GetPath())

        joint.GetExcludeFromArticulationAttr().Set(True)
        self.assertIsNone(robot_utils.GetJointBodyRelationship(joint_prim, 0))

        joint.GetExcludeFromArticulationAttr().Set(False)
        pose = robot_utils.GetJointPose(robot_prim, joint_prim)
        self.assertIsNotNone(pose)
        translation = pose.ExtractTranslation()
        self.assertAlmostEqual(translation[0], 1.0)
        self.assertAlmostEqual(translation[1], 0.0)
        self.assertAlmostEqual(translation[2], 0.0)

    async def test_update_deprecated_schemas(self) -> None:
        """Verify deprecated schema migration to current versions."""
        robot_prim = UsdGeom.Xform.Define(self._stage, "/World/Robot").GetPrim()
        reference_prim = UsdGeom.Xform.Define(self._stage, "/World/Robot/Ref").GetPrim()
        reference_prim.AddAppliedSchema(robot_schema.Classes.REFERENCE_POINT_API.value)

        joint = UsdPhysics.Joint.Define(self._stage, "/World/Robot/joint1")
        joint_prim = joint.GetPrim()
        joint_prim.AddAppliedSchema(robot_schema.Classes.JOINT_API.value)

        self._apply_limit(joint_prim, UsdPhysics.Tokens.transX, low=-1.0, high=1.0)
        joint_prim.CreateAttribute("isaac:physics:Tr_X:DoFOffset", Sdf.ValueTypeNames.Int).Set(0)

        robot_utils.UpdateDeprecatedSchemas(robot_prim)

        self.assertFalse(reference_prim.HasAPI(robot_schema.Classes.REFERENCE_POINT_API.value))
        self.assertTrue(reference_prim.HasAPI(robot_schema.Classes.SITE_API.value))

        dof_attr = joint_prim.GetAttribute("isaac:physics:DofOffsetOpOrder")
        self.assertTrue(dof_attr and dof_attr.HasAuthoredValueOpinion())

    async def test_register_plugin_path_skips_already_registered(self) -> None:
        """Skip registration when plugins are already registered."""
        with tempfile.TemporaryDirectory() as temp_dir:
            plug_info = os.path.join(temp_dir, "plugInfo.json")
            with open(plug_info, "w") as handle:
                handle.write('# comment\n{"Plugins": [{"Name": "TestPlugin"}]}')

            with mock.patch.object(isaac_schema.Plug, "Registry") as registry_cls:
                registry = registry_cls.return_value
                plugin = mock.Mock()
                plugin.name = "TestPlugin"
                registry.GetAllPlugins.return_value = [plugin]

                isaac_schema._register_plugin_path(temp_dir)

                registry.RegisterPlugins.assert_not_called()

    async def test_register_plugin_path_registers_when_missing(self) -> None:
        """Register plugins when not yet present in the registry."""
        with tempfile.TemporaryDirectory() as temp_dir:
            plug_info = os.path.join(temp_dir, "plugInfo.json")
            with open(plug_info, "w") as handle:
                handle.write('# comment\n{"Plugins": [{"Name": "NewPlugin"}]}')

            with mock.patch.object(isaac_schema.Plug, "Registry") as registry_cls:
                registry = registry_cls.return_value
                registry.GetAllPlugins.return_value = []
                registry.RegisterPlugins.return_value = True

                isaac_schema._register_plugin_path(temp_dir)

                registry.RegisterPlugins.assert_called_once_with(temp_dir)


class TestSchemaEnums(omni.kit.test.AsyncTestCase):
    """Verify the values exposed by the schema enum classes."""

    async def test_classes_enum_values(self) -> None:
        """Classes enum contains the expected USD schema token strings."""
        self.assertEqual(robot_schema.Classes.ROBOT_API.value, "IsaacRobotAPI")
        self.assertEqual(robot_schema.Classes.LINK_API.value, "IsaacLinkAPI")
        self.assertEqual(robot_schema.Classes.JOINT_API.value, "IsaacJointAPI")
        self.assertEqual(robot_schema.Classes.SITE_API.value, "IsaacSiteAPI")
        self.assertEqual(robot_schema.Classes.REFERENCE_POINT_API.value, "IsaacReferencePointAPI")

    async def test_attributes_name_property(self) -> None:
        """Attributes.name returns the USD attribute path, not the enum member name."""
        self.assertEqual(robot_schema.Attributes.DESCRIPTION.name, "isaac:description")
        self.assertEqual(robot_schema.Attributes.NAMESPACE.name, "isaac:namespace")
        self.assertEqual(robot_schema.Attributes.DOF_OFFSET_OP_ORDER.name, "isaac:physics:DofOffsetOpOrder")

    async def test_attributes_display_name_property(self) -> None:
        """Attributes.display_name returns the human-readable label."""
        self.assertEqual(robot_schema.Attributes.DESCRIPTION.display_name, "Description")
        self.assertEqual(robot_schema.Attributes.NAMESPACE.display_name, "Namespace")

    async def test_relations_name_property(self) -> None:
        """Relations.name returns the USD relationship path, not the enum member name."""
        self.assertEqual(robot_schema.Relations.ROBOT_LINKS.name, "isaac:physics:robotLinks")
        self.assertEqual(robot_schema.Relations.ROBOT_JOINTS.name, "isaac:physics:robotJoints")

    async def test_dof_offset_op_order_values(self) -> None:
        """DofOffsetOpOrder enum values match the migration token strings."""
        self.assertEqual(robot_schema.DofOffsetOpOrder.TRANS_X.value, "TransX")
        self.assertEqual(robot_schema.DofOffsetOpOrder.ROT_Z.value, "RotZ")


class TestInternalHelpers(omni.kit.test.AsyncTestCase):
    """Tests for internal utility helpers."""

    async def setUp(self) -> None:
        """Set up a fresh USD stage for each test."""
        await omni.usd.get_context().new_stage_async()
        await omni.kit.app.get_app().next_update_async()
        self._stage = omni.usd.get_context().get_stage()

    async def test_is_single_dof_joint_revolute(self) -> None:
        """RevoluteJoint is a single-DOF joint."""
        joint = UsdPhysics.RevoluteJoint.Define(self._stage, "/World/RevJoint")
        self.assertTrue(robot_utils._is_single_dof_joint(joint.GetPrim()))

    async def test_is_single_dof_joint_prismatic(self) -> None:
        """PrismaticJoint is a single-DOF joint."""
        joint = UsdPhysics.PrismaticJoint.Define(self._stage, "/World/PrisJoint")
        self.assertTrue(robot_utils._is_single_dof_joint(joint.GetPrim()))

    async def test_is_single_dof_joint_fixed(self) -> None:
        """FixedJoint is a zero-DOF joint and treated as single-DOF."""
        joint = UsdPhysics.FixedJoint.Define(self._stage, "/World/FixedJoint")
        self.assertTrue(robot_utils._is_single_dof_joint(joint.GetPrim()))

    async def test_is_single_dof_joint_generic(self) -> None:
        """A plain Joint (multi-DOF) is not treated as single-DOF."""
        joint = UsdPhysics.Joint.Define(self._stage, "/World/GenericJoint")
        self.assertFalse(robot_utils._is_single_dof_joint(joint.GetPrim()))

    async def test_collect_deprecated_dof_values_authored(self) -> None:
        """Collect returns only attributes that have authored values."""
        joint = UsdPhysics.Joint.Define(self._stage, "/World/j1")
        prim = joint.GetPrim()
        prim.CreateAttribute("isaac:physics:Tr_X:DoFOffset", Sdf.ValueTypeNames.Int).Set(0)
        prim.CreateAttribute("isaac:physics:Rot_Z:DoFOffset", Sdf.ValueTypeNames.Int).Set(1)

        result = robot_utils._collect_deprecated_dof_values(prim)
        self.assertIn("TransX", result)
        self.assertIn("RotZ", result)
        self.assertNotIn("TransY", result)
        self.assertEqual(result["TransX"], 0)
        self.assertEqual(result["RotZ"], 1)

    async def test_collect_deprecated_dof_values_empty(self) -> None:
        """Return empty dict when no deprecated attributes are authored."""
        joint = UsdPhysics.Joint.Define(self._stage, "/World/j2")
        result = robot_utils._collect_deprecated_dof_values(joint.GetPrim())
        self.assertEqual(result, {})

    async def test_detect_sites_for_link_finds_empty_xform(self) -> None:
        """Child Xform with no sub-children qualifies as a site."""
        link = UsdGeom.Xform.Define(self._stage, "/World/Link").GetPrim()
        UsdGeom.Xform.Define(self._stage, "/World/Link/Site1")
        sites = robot_utils._detect_sites_for_link(link)
        self.assertEqual(len(sites), 1)
        self.assertEqual(sites[0].GetName(), "Site1")

    async def test_detect_sites_for_link_skips_xform_with_children(self) -> None:
        """Child Xform that has its own children is not a site."""
        link = UsdGeom.Xform.Define(self._stage, "/World/LinkB").GetPrim()
        UsdGeom.Xform.Define(self._stage, "/World/LinkB/Parent")
        UsdGeom.Xform.Define(self._stage, "/World/LinkB/Parent/Child")
        sites = robot_utils._detect_sites_for_link(link)
        self.assertEqual(len(sites), 0)

    async def test_detect_sites_for_link_skips_existing_site(self) -> None:
        """Child Xform already carrying SiteAPI is skipped."""
        link = UsdGeom.Xform.Define(self._stage, "/World/LinkC").GetPrim()
        child = UsdGeom.Xform.Define(self._stage, "/World/LinkC/AlreadySite").GetPrim()
        child.AddAppliedSchema(robot_schema.Classes.SITE_API.value)
        sites = robot_utils._detect_sites_for_link(link)
        self.assertEqual(len(sites), 0)

    async def test_detect_sites_for_link_skips_rigid_body(self) -> None:
        """Child Xform with RigidBodyAPI is skipped."""
        link = UsdGeom.Xform.Define(self._stage, "/World/LinkD").GetPrim()
        child = UsdGeom.Xform.Define(self._stage, "/World/LinkD/RigidChild").GetPrim()
        UsdPhysics.RigidBodyAPI.Apply(child)
        sites = robot_utils._detect_sites_for_link(link)
        self.assertEqual(len(sites), 0)

    async def test_print_robot_tree_does_not_raise(self) -> None:
        """PrintRobotTree completes without raising for a valid tree."""
        node = robot_utils.RobotLinkNode(None)
        node.path = None
        node.name = "Root"
        child = robot_utils.RobotLinkNode(None)
        child.path = None
        child.name = "Child"
        node.add_child(child)
        # Should not raise
        robot_utils.PrintRobotTree(node)


class TestValidateAndRebuild(omni.kit.test.AsyncTestCase):
    """Tests for ValidateRobotSchemaRelationships, RebuildRelationshipAsPrepend, and EnsurePrependListForRobotRelationships."""

    async def setUp(self) -> None:
        """Set up a fresh USD stage for each test."""
        await omni.usd.get_context().new_stage_async()
        await omni.kit.app.get_app().next_update_async()
        self._stage = omni.usd.get_context().get_stage()

    def _create_robot_with_joint(self) -> tuple[UsdGeom.Xform, UsdGeom.Xform, UsdPhysics.Joint]:
        robot_prim = UsdGeom.Xform.Define(self._stage, "/World/Robot")
        UsdPhysics.RigidBodyAPI.Apply(robot_prim.GetPrim())
        UsdPhysics.ArticulationRootAPI.Apply(robot_prim.GetPrim())
        link_prim = UsdGeom.Xform.Define(self._stage, "/World/Robot/Link1")
        UsdPhysics.RigidBodyAPI.Apply(link_prim.GetPrim())
        joint = UsdPhysics.Joint.Define(self._stage, "/World/Robot/joint1")
        joint.CreateBody0Rel().SetTargets([robot_prim.GetPrim().GetPath()])
        joint.CreateBody1Rel().SetTargets([link_prim.GetPrim().GetPath()])
        joint.CreateLocalPos0Attr().Set(Gf.Vec3f(1.0, 0.0, 0.0))
        joint.CreateLocalRot0Attr().Set(Gf.Quatf(1.0, 0.0, 0.0, 0.0))
        joint.CreateLocalPos1Attr().Set(Gf.Vec3f(0.0, 0.0, 0.0))
        joint.CreateLocalRot1Attr().Set(Gf.Quatf(1.0, 0.0, 0.0, 0.0))
        return robot_prim, link_prim, joint

    async def test_validate_relationships_all_valid(self) -> None:
        """ValidateRobotSchemaRelationships returns all targets as valid after ApplyRobotAPI."""
        robot_xform, link_xform, joint = self._create_robot_with_joint()
        robot_prim = robot_xform.GetPrim()
        robot_schema.ApplyRobotAPI(robot_prim)

        valid_links, invalid_links, valid_joints, invalid_joints = robot_utils.ValidateRobotSchemaRelationships(
            robot_prim
        )

        self.assertEqual(len(invalid_links), 0)
        self.assertEqual(len(invalid_joints), 0)
        self.assertGreater(len(valid_links), 0)
        self.assertGreater(len(valid_joints), 0)

    async def test_validate_relationships_with_stale_target(self) -> None:
        """Targets pointing to removed prims appear in the invalid lists."""
        robot_xform, link_xform, joint = self._create_robot_with_joint()
        robot_prim = robot_xform.GetPrim()
        robot_schema.ApplyRobotAPI(robot_prim)

        # Manually add a non-existent path to links relationship
        links_rel = robot_prim.GetRelationship(robot_schema.Relations.ROBOT_LINKS.name)
        links_rel.AddTarget(Sdf.Path("/World/Robot/GhostLink"))

        valid_links, invalid_links, valid_joints, invalid_joints = robot_utils.ValidateRobotSchemaRelationships(
            robot_prim
        )
        ghost_paths = [str(p) for p in invalid_links]
        self.assertIn("/World/Robot/GhostLink", ghost_paths)

    async def test_validate_relationships_invalid_prim(self) -> None:
        """Return four empty lists for an invalid prim."""
        invalid_prim = self._stage.GetPrimAtPath("/World/DoesNotExist")
        result = robot_utils.ValidateRobotSchemaRelationships(invalid_prim)
        for lst in result:
            self.assertEqual(len(lst), 0)

    async def test_rebuild_relationship_as_prepend(self) -> None:
        """RebuildRelationshipAsPrepend populates the relationship in the correct order."""
        robot_xform, _, _ = self._create_robot_with_joint()
        robot_prim = robot_xform.GetPrim()
        targets = [
            Sdf.Path("/World/Robot"),
            Sdf.Path("/World/Robot/Link1"),
        ]
        robot_utils.RebuildRelationshipAsPrepend(robot_prim, "myCustomRel", targets)
        rel = robot_prim.GetRelationship("myCustomRel")
        self.assertEqual(rel.GetTargets(), targets)

    async def test_ensure_prepend_list_for_robot_relationships(self) -> None:
        """EnsurePrependListForRobotRelationships preserves existing targets."""
        robot_xform, link_xform, joint = self._create_robot_with_joint()
        robot_prim = robot_xform.GetPrim()
        robot_schema.ApplyRobotAPI(robot_prim)

        before_links = robot_prim.GetRelationship(robot_schema.Relations.ROBOT_LINKS.name).GetTargets()
        before_joints = robot_prim.GetRelationship(robot_schema.Relations.ROBOT_JOINTS.name).GetTargets()

        robot_utils.EnsurePrependListForRobotRelationships(robot_prim)

        after_links = robot_prim.GetRelationship(robot_schema.Relations.ROBOT_LINKS.name).GetTargets()
        after_joints = robot_prim.GetRelationship(robot_schema.Relations.ROBOT_JOINTS.name).GetTargets()
        self.assertEqual({str(p) for p in before_links}, {str(p) for p in after_links})
        self.assertEqual({str(p) for p in before_joints}, {str(p) for p in after_joints})


class TestPopulateWithSites(omni.kit.test.AsyncTestCase):
    """Tests for PopulateRobotSchemaFromArticulation with site detection enabled."""

    async def setUp(self) -> None:
        """Set up a fresh USD stage for each test."""
        await omni.usd.get_context().new_stage_async()
        await omni.kit.app.get_app().next_update_async()
        self._stage = omni.usd.get_context().get_stage()

    def _build_robot_with_site(self) -> tuple[Usd.Prim, Usd.Prim]:
        robot = UsdGeom.Xform.Define(self._stage, "/World/Robot")
        UsdPhysics.RigidBodyAPI.Apply(robot.GetPrim())
        UsdPhysics.ArticulationRootAPI.Apply(robot.GetPrim())
        link = UsdGeom.Xform.Define(self._stage, "/World/Robot/Link1")
        UsdPhysics.RigidBodyAPI.Apply(link.GetPrim())
        # Empty Xform child of link → qualifies as a site
        UsdGeom.Xform.Define(self._stage, "/World/Robot/Link1/EEF")
        joint = UsdPhysics.Joint.Define(self._stage, "/World/Robot/joint1")
        joint.CreateBody0Rel().SetTargets([robot.GetPrim().GetPath()])
        joint.CreateBody1Rel().SetTargets([link.GetPrim().GetPath()])
        joint.CreateLocalPos0Attr().Set(Gf.Vec3f(1.0, 0.0, 0.0))
        joint.CreateLocalRot0Attr().Set(Gf.Quatf(1.0, 0.0, 0.0, 0.0))
        joint.CreateLocalPos1Attr().Set(Gf.Vec3f(0.0, 0.0, 0.0))
        joint.CreateLocalRot1Attr().Set(Gf.Quatf(1.0, 0.0, 0.0, 0.0))
        robot_schema.ApplyRobotAPI(robot.GetPrim())
        return robot.GetPrim(), link.GetPrim()

    async def test_populate_with_detect_sites(self) -> None:
        """With detect_sites=True, EEF child Xform gets SiteAPI and is in robotLinks."""
        robot_prim, link_prim = self._build_robot_with_site()
        eef_prim = self._stage.GetPrimAtPath("/World/Robot/Link1/EEF")

        robot_utils.PopulateRobotSchemaFromArticulation(self._stage, robot_prim, detect_sites=True)

        self.assertTrue(eef_prim.HasAPI(robot_schema.Classes.SITE_API.value))
        links = robot_prim.GetRelationship(robot_schema.Relations.ROBOT_LINKS.name).GetTargets()
        link_strs = [str(p) for p in links]
        self.assertIn("/World/Robot/Link1/EEF", link_strs)

    async def test_populate_with_detect_sites_last(self) -> None:
        """With sites_last=True, the site appears after all regular links."""
        robot_prim, link_prim = self._build_robot_with_site()
        robot_utils.PopulateRobotSchemaFromArticulation(self._stage, robot_prim, detect_sites=True, sites_last=True)
        links = robot_prim.GetRelationship(robot_schema.Relations.ROBOT_LINKS.name).GetTargets()
        link_strs = [str(p) for p in links]
        self.assertIn("/World/Robot/Link1/EEF", link_strs)
        # Site must come after the regular links
        eef_idx = link_strs.index("/World/Robot/Link1/EEF")
        link1_idx = link_strs.index("/World/Robot/Link1")
        self.assertGreater(eef_idx, link1_idx)


class TestPopulateSubRobotBoundary(omni.kit.test.AsyncTestCase):
    """Tests that PopulateRobotSchemaFromArticulation respects sub-robot boundaries."""

    async def setUp(self) -> None:
        """Create a fresh stage for each test case."""
        await omni.usd.get_context().new_stage_async()
        await omni.kit.app.get_app().next_update_async()
        self._stage = omni.usd.get_context().get_stage()

    def _build_robot_with_sub_robot(self) -> None:
        """Create parent robot -> arm link -> sub-robot -> finger link."""
        robot = UsdGeom.Xform.Define(self._stage, "/World/Robot")
        UsdPhysics.ArticulationRootAPI.Apply(robot.GetPrim())
        UsdPhysics.RigidBodyAPI.Apply(robot.GetPrim())

        arm = UsdGeom.Xform.Define(self._stage, "/World/Robot/Arm")
        UsdPhysics.RigidBodyAPI.Apply(arm.GetPrim())
        j_arm = UsdPhysics.Joint.Define(self._stage, "/World/Robot/j_arm")
        j_arm.CreateBody0Rel().SetTargets([robot.GetPrim().GetPath()])
        j_arm.CreateBody1Rel().SetTargets([arm.GetPrim().GetPath()])

        sub = UsdGeom.Xform.Define(self._stage, "/World/Robot/Sub")
        UsdPhysics.RigidBodyAPI.Apply(sub.GetPrim())
        robot_schema.ApplyRobotAPI(sub.GetPrim())

        finger = UsdGeom.Xform.Define(self._stage, "/World/Robot/Sub/Finger")
        UsdPhysics.RigidBodyAPI.Apply(finger.GetPrim())
        sub_joint = UsdPhysics.Joint.Define(self._stage, "/World/Robot/Sub/j_finger")
        sub_joint.CreateBody0Rel().SetTargets([sub.GetPrim().GetPath()])
        sub_joint.CreateBody1Rel().SetTargets([finger.GetPrim().GetPath()])

        j_sub = UsdPhysics.Joint.Define(self._stage, "/World/Robot/j_sub")
        j_sub.CreateBody0Rel().SetTargets([arm.GetPrim().GetPath()])
        j_sub.CreateBody1Rel().SetTargets([sub.GetPrim().GetPath()])

        return robot.GetPrim()

    async def test_populate_adds_sub_robot_prim_to_links(self) -> None:
        """The sub-robot prim itself appears in the parent's robotLinks."""
        robot_prim = self._build_robot_with_sub_robot()
        robot_utils.PopulateRobotSchemaFromArticulation(self._stage, robot_prim)

        link_paths = {str(p) for p in robot_prim.GetRelationship(robot_schema.Relations.ROBOT_LINKS.name).GetTargets()}
        self.assertIn("/World/Robot/Sub", link_paths)

    async def test_populate_excludes_sub_robot_internals(self) -> None:
        """Sub-robot internal links and joints are excluded from the parent."""
        robot_prim = self._build_robot_with_sub_robot()
        robot_utils.PopulateRobotSchemaFromArticulation(self._stage, robot_prim)

        link_paths = {str(p) for p in robot_prim.GetRelationship(robot_schema.Relations.ROBOT_LINKS.name).GetTargets()}
        joint_paths = {
            str(p) for p in robot_prim.GetRelationship(robot_schema.Relations.ROBOT_JOINTS.name).GetTargets()
        }

        self.assertNotIn("/World/Robot/Sub/Finger", link_paths)
        self.assertNotIn("/World/Robot/Sub/j_finger", joint_paths)

    async def test_populate_keeps_parent_joints(self) -> None:
        """The bridging joint and parent-only joints remain in the parent."""
        robot_prim = self._build_robot_with_sub_robot()
        robot_utils.PopulateRobotSchemaFromArticulation(self._stage, robot_prim)

        joint_paths = {
            str(p) for p in robot_prim.GetRelationship(robot_schema.Relations.ROBOT_JOINTS.name).GetTargets()
        }
        self.assertIn("/World/Robot/j_arm", joint_paths)
        self.assertIn("/World/Robot/j_sub", joint_paths)

    async def test_populate_adds_sub_robot_when_joint_targets_internal_link(self) -> None:
        """When a bridging joint body points inside the sub-robot, the sub-robot prim is still added."""
        robot = UsdGeom.Xform.Define(self._stage, "/World/Robot")
        UsdPhysics.ArticulationRootAPI.Apply(robot.GetPrim())
        UsdPhysics.RigidBodyAPI.Apply(robot.GetPrim())

        arm = UsdGeom.Xform.Define(self._stage, "/World/Robot/Arm")
        UsdPhysics.RigidBodyAPI.Apply(arm.GetPrim())
        j_arm = UsdPhysics.Joint.Define(self._stage, "/World/Robot/j_arm")
        j_arm.CreateBody0Rel().SetTargets([robot.GetPrim().GetPath()])
        j_arm.CreateBody1Rel().SetTargets([arm.GetPrim().GetPath()])

        sub = UsdGeom.Xform.Define(self._stage, "/World/Robot/Sub")
        robot_schema.ApplyRobotAPI(sub.GetPrim())

        palm = UsdGeom.Xform.Define(self._stage, "/World/Robot/Sub/Palm")
        UsdPhysics.RigidBodyAPI.Apply(palm.GetPrim())

        j_bridge = UsdPhysics.Joint.Define(self._stage, "/World/Robot/j_bridge")
        j_bridge.CreateBody0Rel().SetTargets([arm.GetPrim().GetPath()])
        j_bridge.CreateBody1Rel().SetTargets([palm.GetPrim().GetPath()])

        robot_utils.PopulateRobotSchemaFromArticulation(self._stage, robot.GetPrim())

        link_paths = {
            str(p) for p in robot.GetPrim().GetRelationship(robot_schema.Relations.ROBOT_LINKS.name).GetTargets()
        }
        self.assertIn("/World/Robot/Sub", link_paths)
        self.assertNotIn("/World/Robot/Sub/Palm", link_paths)

    async def test_populate_skips_non_rigid_body(self) -> None:
        """Prims without RigidBodyAPI are not added as links."""
        robot = UsdGeom.Xform.Define(self._stage, "/World/Robot")
        UsdPhysics.ArticulationRootAPI.Apply(robot.GetPrim())
        UsdPhysics.RigidBodyAPI.Apply(robot.GetPrim())
        robot_schema.ApplyRobotAPI(robot.GetPrim())

        link = UsdGeom.Xform.Define(self._stage, "/World/Robot/Link1")
        UsdPhysics.RigidBodyAPI.Apply(link.GetPrim())
        no_body = UsdGeom.Xform.Define(self._stage, "/World/Robot/NoBody")

        j1 = UsdPhysics.Joint.Define(self._stage, "/World/Robot/j1")
        j1.CreateBody0Rel().SetTargets([robot.GetPrim().GetPath()])
        j1.CreateBody1Rel().SetTargets([link.GetPrim().GetPath()])
        j2 = UsdPhysics.Joint.Define(self._stage, "/World/Robot/j2")
        j2.CreateBody0Rel().SetTargets([link.GetPrim().GetPath()])
        j2.CreateBody1Rel().SetTargets([no_body.GetPrim().GetPath()])

        robot_utils.PopulateRobotSchemaFromArticulation(self._stage, robot.GetPrim())

        link_paths = {
            str(p) for p in robot.GetPrim().GetRelationship(robot_schema.Relations.ROBOT_LINKS.name).GetTargets()
        }
        self.assertIn("/World/Robot/Link1", link_paths)
        self.assertNotIn("/World/Robot/NoBody", link_paths)
        self.assertFalse(no_body.GetPrim().HasAPI(robot_schema.Classes.LINK_API.value))


class TestRecalculateRobotSchema(omni.kit.test.AsyncTestCase):
    """Tests for RecalculateRobotSchema."""

    async def setUp(self) -> None:
        """Set up a fresh USD stage for each test."""
        await omni.usd.get_context().new_stage_async()
        await omni.kit.app.get_app().next_update_async()
        self._stage = omni.usd.get_context().get_stage()

    def _create_robot_with_joint(self) -> tuple[UsdGeom.Xform, UsdGeom.Xform, UsdPhysics.Joint]:
        robot_prim = UsdGeom.Xform.Define(self._stage, "/World/Robot")
        UsdPhysics.RigidBodyAPI.Apply(robot_prim.GetPrim())
        UsdPhysics.ArticulationRootAPI.Apply(robot_prim.GetPrim())
        link_prim = UsdGeom.Xform.Define(self._stage, "/World/Robot/Link1")
        UsdPhysics.RigidBodyAPI.Apply(link_prim.GetPrim())
        joint = UsdPhysics.Joint.Define(self._stage, "/World/Robot/joint1")
        joint.CreateBody0Rel().SetTargets([robot_prim.GetPrim().GetPath()])
        joint.CreateBody1Rel().SetTargets([link_prim.GetPrim().GetPath()])
        joint.CreateLocalPos0Attr().Set(Gf.Vec3f(1.0, 0.0, 0.0))
        joint.CreateLocalRot0Attr().Set(Gf.Quatf(1.0, 0.0, 0.0, 0.0))
        joint.CreateLocalPos1Attr().Set(Gf.Vec3f(0.0, 0.0, 0.0))
        joint.CreateLocalRot1Attr().Set(Gf.Quatf(1.0, 0.0, 0.0, 0.0))
        return robot_prim, link_prim, joint

    async def test_raises_on_invalid_stage(self) -> None:
        """Raise ValueError when stage is None."""
        robot_xform, _, _ = self._create_robot_with_joint()
        robot_schema.ApplyRobotAPI(robot_xform.GetPrim())
        with self.assertRaises(ValueError):
            robot_utils.RecalculateRobotSchema(None, robot_xform.GetPrim())

    async def test_raises_on_invalid_robot_prim(self) -> None:
        """Raise ValueError when robot_prim is None."""
        with self.assertRaises(ValueError):
            robot_utils.RecalculateRobotSchema(self._stage, None)

    async def test_basic_recalculate_returns_root_link(self) -> None:
        """RecalculateRobotSchema returns a valid root_link prim."""
        robot_xform, link_xform, joint = self._create_robot_with_joint()
        robot_prim = robot_xform.GetPrim()
        robot_schema.ApplyRobotAPI(robot_prim)

        root_link, root_joint = robot_utils.RecalculateRobotSchema(self._stage, robot_prim)
        self.assertIsNotNone(root_link)
        self.assertTrue(root_link.IsValid())

    async def test_recalculate_preserves_existing_links_and_adds_new(self) -> None:
        """Existing valid links are preserved; newly discovered links are appended."""
        robot_xform, link_xform, joint = self._create_robot_with_joint()
        robot_prim = robot_xform.GetPrim()
        link_prim = link_xform.GetPrim()
        robot_schema.ApplyRobotAPI(robot_prim)

        initial_links = {
            str(p) for p in robot_prim.GetRelationship(robot_schema.Relations.ROBOT_LINKS.name).GetTargets()
        }

        # Add a second link connected via a new joint
        link2 = UsdGeom.Xform.Define(self._stage, "/World/Robot/Link2")
        UsdPhysics.RigidBodyAPI.Apply(link2.GetPrim())
        joint2 = UsdPhysics.Joint.Define(self._stage, "/World/Robot/joint2")
        joint2.CreateBody0Rel().SetTargets([link_prim.GetPath()])
        joint2.CreateBody1Rel().SetTargets([link2.GetPrim().GetPath()])
        joint2.CreateLocalPos0Attr().Set(Gf.Vec3f(1.0, 0.0, 0.0))
        joint2.CreateLocalRot0Attr().Set(Gf.Quatf(1.0, 0.0, 0.0, 0.0))
        joint2.CreateLocalPos1Attr().Set(Gf.Vec3f(0.0, 0.0, 0.0))
        joint2.CreateLocalRot1Attr().Set(Gf.Quatf(1.0, 0.0, 0.0, 0.0))

        robot_utils.RecalculateRobotSchema(self._stage, robot_prim)

        new_links = {str(p) for p in robot_prim.GetRelationship(robot_schema.Relations.ROBOT_LINKS.name).GetTargets()}
        # Original links preserved
        for path in initial_links:
            self.assertIn(path, new_links)
        # New link added
        self.assertIn("/World/Robot/Link2", new_links)

    async def test_recalculate_skips_prims_without_rigid_body(self) -> None:
        """Prims reached via BFS that lack RigidBodyAPI are not added as links."""
        robot_xform = UsdGeom.Xform.Define(self._stage, "/World/Robot")
        UsdPhysics.ArticulationRootAPI.Apply(robot_xform.GetPrim())
        UsdPhysics.RigidBodyAPI.Apply(robot_xform.GetPrim())
        robot_schema.ApplyRobotAPI(robot_xform.GetPrim())

        link = UsdGeom.Xform.Define(self._stage, "/World/Robot/Link1")
        UsdPhysics.RigidBodyAPI.Apply(link.GetPrim())

        no_body = UsdGeom.Xform.Define(self._stage, "/World/Robot/NoBody")

        j1 = UsdPhysics.Joint.Define(self._stage, "/World/Robot/j1")
        j1.CreateBody0Rel().SetTargets([robot_xform.GetPrim().GetPath()])
        j1.CreateBody1Rel().SetTargets([link.GetPrim().GetPath()])

        j2 = UsdPhysics.Joint.Define(self._stage, "/World/Robot/j2")
        j2.CreateBody0Rel().SetTargets([link.GetPrim().GetPath()])
        j2.CreateBody1Rel().SetTargets([no_body.GetPrim().GetPath()])

        robot_utils.RecalculateRobotSchema(self._stage, robot_xform.GetPrim())

        link_paths = {
            str(p) for p in robot_xform.GetPrim().GetRelationship(robot_schema.Relations.ROBOT_LINKS.name).GetTargets()
        }
        self.assertIn("/World/Robot/Link1", link_paths)
        self.assertNotIn("/World/Robot/NoBody", link_paths)
        self.assertFalse(no_body.GetPrim().HasAPI(robot_schema.Classes.LINK_API.value))

    async def test_recalculate_does_not_descend_into_sub_robot(self) -> None:
        """Sub-robot internals are excluded; the sub-robot prim itself is discovered."""
        robot_xform = UsdGeom.Xform.Define(self._stage, "/World/Robot")
        UsdPhysics.ArticulationRootAPI.Apply(robot_xform.GetPrim())
        UsdPhysics.RigidBodyAPI.Apply(robot_xform.GetPrim())
        robot_schema.ApplyRobotAPI(robot_xform.GetPrim())

        arm = UsdGeom.Xform.Define(self._stage, "/World/Robot/Arm")
        UsdPhysics.RigidBodyAPI.Apply(arm.GetPrim())
        j_arm = UsdPhysics.Joint.Define(self._stage, "/World/Robot/j_arm")
        j_arm.CreateBody0Rel().SetTargets([robot_xform.GetPrim().GetPath()])
        j_arm.CreateBody1Rel().SetTargets([arm.GetPrim().GetPath()])

        sub = UsdGeom.Xform.Define(self._stage, "/World/Robot/Sub")
        UsdPhysics.RigidBodyAPI.Apply(sub.GetPrim())
        robot_schema.ApplyRobotAPI(sub.GetPrim())

        sub_link = UsdGeom.Xform.Define(self._stage, "/World/Robot/Sub/Finger")
        UsdPhysics.RigidBodyAPI.Apply(sub_link.GetPrim())
        sub_joint = UsdPhysics.Joint.Define(self._stage, "/World/Robot/Sub/j_finger")
        sub_joint.CreateBody0Rel().SetTargets([sub.GetPrim().GetPath()])
        sub_joint.CreateBody1Rel().SetTargets([sub_link.GetPrim().GetPath()])

        j_sub = UsdPhysics.Joint.Define(self._stage, "/World/Robot/j_sub")
        j_sub.CreateBody0Rel().SetTargets([arm.GetPrim().GetPath()])
        j_sub.CreateBody1Rel().SetTargets([sub.GetPrim().GetPath()])

        robot_utils.RecalculateRobotSchema(self._stage, robot_xform.GetPrim())

        link_paths = {
            str(p) for p in robot_xform.GetPrim().GetRelationship(robot_schema.Relations.ROBOT_LINKS.name).GetTargets()
        }
        joint_paths = {
            str(p) for p in robot_xform.GetPrim().GetRelationship(robot_schema.Relations.ROBOT_JOINTS.name).GetTargets()
        }

        self.assertIn("/World/Robot/Arm", link_paths)
        self.assertIn("/World/Robot/Sub", link_paths)
        self.assertNotIn("/World/Robot/Sub/Finger", link_paths)
        self.assertIn("/World/Robot/j_arm", joint_paths)
        self.assertIn("/World/Robot/j_sub", joint_paths)
        self.assertNotIn("/World/Robot/Sub/j_finger", joint_paths)


class TestDetectAndApplySites(omni.kit.test.AsyncTestCase):
    """Tests for DetectAndApplySites and AddSitesToRobotLinks."""

    async def setUp(self) -> None:
        """Set up a fresh USD stage for each test."""
        await omni.usd.get_context().new_stage_async()
        await omni.kit.app.get_app().next_update_async()
        self._stage = omni.usd.get_context().get_stage()

    def _create_robot_with_link_and_site_candidate(self) -> tuple[Usd.Prim, Usd.Prim]:
        robot = UsdGeom.Xform.Define(self._stage, "/World/Robot")
        UsdPhysics.RigidBodyAPI.Apply(robot.GetPrim())
        UsdPhysics.ArticulationRootAPI.Apply(robot.GetPrim())
        link = UsdGeom.Xform.Define(self._stage, "/World/Robot/Link1")
        UsdPhysics.RigidBodyAPI.Apply(link.GetPrim())
        UsdGeom.Xform.Define(self._stage, "/World/Robot/Link1/EEF")
        joint = UsdPhysics.Joint.Define(self._stage, "/World/Robot/joint1")
        joint.CreateBody0Rel().SetTargets([robot.GetPrim().GetPath()])
        joint.CreateBody1Rel().SetTargets([link.GetPrim().GetPath()])
        joint.CreateLocalPos0Attr().Set(Gf.Vec3f(1.0, 0.0, 0.0))
        joint.CreateLocalRot0Attr().Set(Gf.Quatf(1.0, 0.0, 0.0, 0.0))
        joint.CreateLocalPos1Attr().Set(Gf.Vec3f(0.0, 0.0, 0.0))
        joint.CreateLocalRot1Attr().Set(Gf.Quatf(1.0, 0.0, 0.0, 0.0))
        robot_schema.ApplyRobotAPI(robot.GetPrim())
        return robot.GetPrim(), link.GetPrim()

    async def test_detect_and_apply_sites_applies_site_api(self) -> None:
        """DetectAndApplySites applies SiteAPI to qualifying child Xforms."""
        robot_prim, link_prim = self._create_robot_with_link_and_site_candidate()
        eef_prim = self._stage.GetPrimAtPath("/World/Robot/Link1/EEF")
        self.assertFalse(eef_prim.HasAPI(robot_schema.Classes.SITE_API.value))

        all_sites, sites_by_parent = robot_utils.DetectAndApplySites(self._stage, robot_prim)

        self.assertTrue(eef_prim.HasAPI(robot_schema.Classes.SITE_API.value))
        self.assertEqual(len(all_sites), 1)
        self.assertEqual(all_sites[0].GetPath(), eef_prim.GetPath())
        self.assertIn("/World/Robot/Link1", sites_by_parent)

    async def test_detect_and_apply_sites_empty_when_no_candidates(self) -> None:
        """Returns empty lists when no site candidates exist."""
        robot = UsdGeom.Xform.Define(self._stage, "/World/EmptyRobot")
        UsdPhysics.RigidBodyAPI.Apply(robot.GetPrim())
        UsdPhysics.ArticulationRootAPI.Apply(robot.GetPrim())
        robot_schema.ApplyRobotAPI(robot.GetPrim())
        all_sites, sites_by_parent = robot_utils.DetectAndApplySites(self._stage, robot.GetPrim())
        self.assertEqual(len(all_sites), 0)
        self.assertEqual(len(sites_by_parent), 0)

    def _create_robot_with_two_links_and_sites(self) -> Usd.Prim:
        """Build a robot with two links each having a site candidate child.

        Returns:
            The robot prim with applied RobotAPI.

        """
        robot = UsdGeom.Xform.Define(self._stage, "/World/Robot2")
        UsdPhysics.RigidBodyAPI.Apply(robot.GetPrim())
        UsdPhysics.ArticulationRootAPI.Apply(robot.GetPrim())

        link1 = UsdGeom.Xform.Define(self._stage, "/World/Robot2/Link1")
        UsdPhysics.RigidBodyAPI.Apply(link1.GetPrim())
        UsdGeom.Xform.Define(self._stage, "/World/Robot2/Link1/EEF1")

        link2 = UsdGeom.Xform.Define(self._stage, "/World/Robot2/Link2")
        UsdPhysics.RigidBodyAPI.Apply(link2.GetPrim())
        UsdGeom.Xform.Define(self._stage, "/World/Robot2/Link2/EEF2")

        joint1 = UsdPhysics.Joint.Define(self._stage, "/World/Robot2/joint1")
        joint1.CreateBody0Rel().SetTargets([robot.GetPrim().GetPath()])
        joint1.CreateBody1Rel().SetTargets([link1.GetPrim().GetPath()])
        joint1.CreateLocalPos0Attr().Set(Gf.Vec3f(1.0, 0.0, 0.0))
        joint1.CreateLocalRot0Attr().Set(Gf.Quatf(1.0, 0.0, 0.0, 0.0))
        joint1.CreateLocalPos1Attr().Set(Gf.Vec3f(0.0, 0.0, 0.0))
        joint1.CreateLocalRot1Attr().Set(Gf.Quatf(1.0, 0.0, 0.0, 0.0))

        joint2 = UsdPhysics.Joint.Define(self._stage, "/World/Robot2/joint2")
        joint2.CreateBody0Rel().SetTargets([link1.GetPrim().GetPath()])
        joint2.CreateBody1Rel().SetTargets([link2.GetPrim().GetPath()])
        joint2.CreateLocalPos0Attr().Set(Gf.Vec3f(1.0, 0.0, 0.0))
        joint2.CreateLocalRot0Attr().Set(Gf.Quatf(1.0, 0.0, 0.0, 0.0))
        joint2.CreateLocalPos1Attr().Set(Gf.Vec3f(0.0, 0.0, 0.0))
        joint2.CreateLocalRot1Attr().Set(Gf.Quatf(1.0, 0.0, 0.0, 0.0))

        robot_schema.ApplyRobotAPI(robot.GetPrim())
        return robot.GetPrim()

    async def test_add_sites_to_robot_links_sites_last(self) -> None:
        """AddSitesToRobotLinks appends sites to the end of robotLinks when sites_last=True.

        Expected order: [Robot2, Link1, Link2, EEF1, EEF2]
        """
        robot_prim = self._create_robot_with_two_links_and_sites()
        all_sites, sites_by_parent = robot_utils.DetectAndApplySites(self._stage, robot_prim)

        robot_utils.AddSitesToRobotLinks(robot_prim, sites=all_sites, sites_last=True)

        links = robot_prim.GetRelationship(robot_schema.Relations.ROBOT_LINKS.name).GetTargets()
        link_strs = [str(p) for p in links]

        link1_idx = link_strs.index("/World/Robot2/Link1")
        link2_idx = link_strs.index("/World/Robot2/Link2")
        eef1_idx = link_strs.index("/World/Robot2/Link1/EEF1")
        eef2_idx = link_strs.index("/World/Robot2/Link2/EEF2")

        # Both regular links appear before both sites
        self.assertLess(link1_idx, eef1_idx)
        self.assertLess(link2_idx, eef1_idx)
        self.assertLess(link1_idx, eef2_idx)
        self.assertLess(link2_idx, eef2_idx)
        # A regular link sits between Link1 and the first site
        self.assertLess(link1_idx, link2_idx)
        self.assertLess(link2_idx, eef1_idx)

    async def test_add_sites_to_robot_links_inline(self) -> None:
        """AddSitesToRobotLinks inserts sites after their parent link when sites_last=False.

        Expected order: [Robot2, Link1, EEF1, Link2, EEF2]
        """
        robot_prim = self._create_robot_with_two_links_and_sites()
        all_sites, sites_by_parent = robot_utils.DetectAndApplySites(self._stage, robot_prim)

        robot_utils.AddSitesToRobotLinks(robot_prim, sites_by_parent=sites_by_parent, sites_last=False)

        links = robot_prim.GetRelationship(robot_schema.Relations.ROBOT_LINKS.name).GetTargets()
        link_strs = [str(p) for p in links]

        link1_idx = link_strs.index("/World/Robot2/Link1")
        link2_idx = link_strs.index("/World/Robot2/Link2")
        eef1_idx = link_strs.index("/World/Robot2/Link1/EEF1")
        eef2_idx = link_strs.index("/World/Robot2/Link2/EEF2")

        # Each site appears directly after its parent link
        self.assertEqual(eef1_idx, link1_idx + 1)
        self.assertEqual(eef2_idx, link2_idx + 1)
        # Link2 appears after EEF1 (interleaved, not grouped)
        self.assertLess(eef1_idx, link2_idx)


class TestPopulateRobotSchemaFallback(omni.kit.test.AsyncTestCase):
    """Regression tests for the robot_prim-fallback + PrimRange-sweep behavior.

    When ``_find_articulation_root`` returns None, the populate function must
    fall back to ``robot_prim`` as a synthetic articulation root. When BFS
    yields empty joint/link sets, a ``Usd.PrimRange`` sweep must apply
    ``JointAPI`` / ``LinkAPI`` to every joint / rigid body so downstream
    relationships are populated on rigid-body-only and instanceable assets.
    """

    async def setUp(self) -> None:
        """Set up a fresh USD stage for each test."""
        await omni.usd.get_context().new_stage_async()
        await omni.kit.app.get_app().next_update_async()
        self._stage = omni.usd.get_context().get_stage()

    async def test_populate_falls_back_to_robot_prim_when_no_articulation_root(self) -> None:
        """Missing articulation root → robot_prim fallback; BFS + PrimRange find the joint."""
        # Stage with `/Robot` + rigid-body links + a RevoluteJoint, but NO ArticulationRootAPI anywhere.
        robot_xform = UsdGeom.Xform.Define(self._stage, "/World/Robot")
        robot_prim = robot_xform.GetPrim()

        link0 = UsdGeom.Xform.Define(self._stage, "/World/Robot/link0")
        UsdPhysics.RigidBodyAPI.Apply(link0.GetPrim())
        link1 = UsdGeom.Xform.Define(self._stage, "/World/Robot/link1")
        UsdPhysics.RigidBodyAPI.Apply(link1.GetPrim())

        joint = UsdPhysics.RevoluteJoint.Define(self._stage, "/World/Robot/joints/j0")
        joint.CreateBody0Rel().SetTargets([link0.GetPrim().GetPath()])
        joint.CreateBody1Rel().SetTargets([link1.GetPrim().GetPath()])
        joint.CreateLocalPos0Attr().Set(Gf.Vec3f(0.0, 0.0, 0.0))
        joint.CreateLocalRot0Attr().Set(Gf.Quatf(1.0, 0.0, 0.0, 0.0))
        joint.CreateLocalPos1Attr().Set(Gf.Vec3f(0.0, 0.0, 0.0))
        joint.CreateLocalRot1Attr().Set(Gf.Quatf(1.0, 0.0, 0.0, 0.0))
        # IMPORTANT: no ArticulationRootAPI.Apply anywhere — exercises the robot_prim fallback.

        result_root, _ = robot_utils.PopulateRobotSchemaFromArticulation(self._stage, robot_prim)

        self.assertIsNotNone(
            result_root,
            "Expected fallback to robot_prim, got early-return (None, None).",
        )
        # Verify JointAPI was applied via the PrimRange sweep.
        self.assertTrue(
            joint.GetPrim().HasAPI(robot_schema.Classes.JOINT_API.value),
            "Expected sweep to apply JointAPI to the RevoluteJoint.",
        )
        # Verify the `isaac:physics:robotJoints` relationship was populated.
        rel = robot_prim.GetRelationship(robot_schema.Relations.ROBOT_JOINTS.name)
        self.assertTrue(rel and rel.IsValid())
        targets = rel.GetTargets()
        self.assertGreaterEqual(
            len(targets),
            1,
            f"Expected robotJoints to have >= 1 target, got {list(targets)}.",
        )

    async def test_populate_sweeps_links_for_rigid_body_only_robot(self) -> None:
        """ordered_links empty → sweep RigidBodyAPI prims, populate robotLinks."""
        # Vehicle-style stage: rigid-body links only, no joints, no ArticulationRootAPI.
        vehicle_xform = UsdGeom.Xform.Define(self._stage, "/World/Vehicle")
        robot_prim = vehicle_xform.GetPrim()
        for i in range(3):
            link = UsdGeom.Xform.Define(self._stage, f"/World/Vehicle/link_{i}")
            UsdPhysics.RigidBodyAPI.Apply(link.GetPrim())
        # No joints, no ArticulationRootAPI — pure rigid-body-only robot.

        result_root, _ = robot_utils.PopulateRobotSchemaFromArticulation(self._stage, robot_prim)
        self.assertIsNotNone(result_root)

        for i in range(3):
            link_prim = self._stage.GetPrimAtPath(f"/World/Vehicle/link_{i}")
            self.assertTrue(
                link_prim.HasAPI(robot_schema.Classes.LINK_API.value),
                f"Expected link_{i} to have LinkAPI via sweep.",
            )
        rel = robot_prim.GetRelationship(robot_schema.Relations.ROBOT_LINKS.name)
        self.assertTrue(rel and rel.IsValid())
        # BFS unconditionally applies LinkAPI to the synthetic root (`/World/Vehicle`)
        # in addition to the 3 child rigid bodies picked up by the sweep; total >= 3.
        targets = [str(p) for p in rel.GetTargets()]
        self.assertGreaterEqual(
            len(targets),
            3,
            f"Expected robotLinks to have >= 3 targets, got {targets}.",
        )
        for i in range(3):
            self.assertIn(
                f"/World/Vehicle/link_{i}",
                targets,
                f"Expected /World/Vehicle/link_{i} to appear in robotLinks, got {targets}.",
            )

    async def test_populate_unchanged_when_articulation_root_present(self) -> None:
        """Regression: well-formed articulated robots still get normal BFS treatment."""
        robot_xform = UsdGeom.Xform.Define(self._stage, "/World/Robot")
        robot_prim = robot_xform.GetPrim()
        UsdPhysics.RigidBodyAPI.Apply(robot_prim)
        UsdPhysics.ArticulationRootAPI.Apply(robot_prim)

        link1 = UsdGeom.Xform.Define(self._stage, "/World/Robot/link1")
        UsdPhysics.RigidBodyAPI.Apply(link1.GetPrim())

        joint = UsdPhysics.Joint.Define(self._stage, "/World/Robot/joint1")
        joint.CreateBody0Rel().SetTargets([robot_prim.GetPath()])
        joint.CreateBody1Rel().SetTargets([link1.GetPrim().GetPath()])
        joint.CreateLocalPos0Attr().Set(Gf.Vec3f(1.0, 0.0, 0.0))
        joint.CreateLocalRot0Attr().Set(Gf.Quatf(1.0, 0.0, 0.0, 0.0))
        joint.CreateLocalPos1Attr().Set(Gf.Vec3f(0.0, 0.0, 0.0))
        joint.CreateLocalRot1Attr().Set(Gf.Quatf(1.0, 0.0, 0.0, 0.0))

        result_root, _ = robot_utils.PopulateRobotSchemaFromArticulation(self._stage, robot_prim)
        self.assertIsNotNone(result_root)

        rel_links = robot_prim.GetRelationship(robot_schema.Relations.ROBOT_LINKS.name)
        self.assertTrue(rel_links and rel_links.IsValid())
        self.assertGreaterEqual(
            len(rel_links.GetTargets()),
            1,
            "Well-formed articulated stage should still yield at least one robotLinks target via BFS.",
        )
        rel_joints = robot_prim.GetRelationship(robot_schema.Relations.ROBOT_JOINTS.name)
        self.assertTrue(rel_joints and rel_joints.IsValid())
        self.assertGreaterEqual(
            len(rel_joints.GetTargets()),
            1,
            "Well-formed articulated stage should still yield at least one robotJoints target via BFS.",
        )
