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

"""Tests for Robot Poser UI helper utilities (fk_helpers, named-pose table model)."""

import numpy as np
import omni.kit.app
import omni.kit.test
import omni.usd
from isaacsim.robot.poser.ui.utils.fk_helpers import (
    build_pose_from_current_joints,
    find_robot_ancestor,
    get_site_candidates,
    read_joint_limits_native,
    write_transform_to_prim,
)
from pxr import Gf, UsdGeom, UsdPhysics
from usd.schema.isaac import robot_schema
from usd.schema.isaac.robot_schema import utils as robot_utils
from usd.schema.isaac.robot_schema.math import Transform


class TestFkHelpers(omni.kit.test.AsyncTestCase):
    """Async tests for fk_helpers utility functions."""

    async def setUp(self) -> None:
        """Create a fresh USD stage before each test."""
        await omni.usd.get_context().new_stage_async()
        await omni.kit.app.get_app().next_update_async()
        self._stage = omni.usd.get_context().get_stage()

    def _create_robot(self) -> tuple:
        """Create a two-link robot with a revolute joint and RobotAPI.

        Returns:
            Tuple of (robot_prim, link_prim, joint_prim).
        """
        robot_xform = UsdGeom.Xform.Define(self._stage, "/World/Robot")
        UsdPhysics.RigidBodyAPI.Apply(robot_xform.GetPrim())
        UsdPhysics.ArticulationRootAPI.Apply(robot_xform.GetPrim())

        link1 = UsdGeom.Xform.Define(self._stage, "/World/Robot/Link1")
        UsdPhysics.RigidBodyAPI.Apply(link1.GetPrim())

        joint = UsdPhysics.RevoluteJoint.Define(self._stage, "/World/Robot/joint1")
        joint.CreateBody0Rel().SetTargets([robot_xform.GetPrim().GetPath()])
        joint.CreateBody1Rel().SetTargets([link1.GetPrim().GetPath()])
        joint.CreateAxisAttr("X")
        joint.GetLowerLimitAttr().Set(-90.0)
        joint.GetUpperLimitAttr().Set(90.0)
        joint.CreateLocalPos0Attr().Set(Gf.Vec3f(1.0, 0.0, 0.0))
        joint.CreateLocalRot0Attr().Set(Gf.Quatf(1.0, 0.0, 0.0, 0.0))
        joint.CreateLocalPos1Attr().Set(Gf.Vec3f(0.0, 0.0, 0.0))
        joint.CreateLocalRot1Attr().Set(Gf.Quatf(1.0, 0.0, 0.0, 0.0))

        robot_schema.ApplyRobotAPI(robot_xform.GetPrim())
        return robot_xform.GetPrim(), link1.GetPrim(), joint.GetPrim()

    # -- find_robot_ancestor -----------------------------------------------

    async def test_find_robot_ancestor(self) -> None:
        """Verify that find_robot_ancestor returns the robot prim for a named pose under it."""
        robot_prim, _, _ = self._create_robot()
        scope = self._stage.DefinePrim(robot_prim.GetPath().AppendChild("Named_Poses"), "Scope")
        child = robot_schema.CreateNamedPose(self._stage, str(scope.GetPath().AppendChild("MyPose")))

        ancestor = find_robot_ancestor(self._stage, child)
        self.assertIsNotNone(ancestor)
        self.assertEqual(ancestor.GetPath(), robot_prim.GetPath())

    async def test_find_robot_ancestor_returns_none(self) -> None:
        """Verify that find_robot_ancestor returns None when prim has no robot ancestor."""
        orphan = UsdGeom.Xform.Define(self._stage, "/World/Orphan").GetPrim()
        self.assertIsNone(find_robot_ancestor(self._stage, orphan))

    async def test_find_robot_ancestor_deep_hierarchy(self) -> None:
        """Verify that find_robot_ancestor finds the robot when the prim is several levels deep."""
        robot_prim, _, _ = self._create_robot()
        scope = self._stage.DefinePrim(robot_prim.GetPath().AppendChild("Named_Poses"), "Scope")
        sub = self._stage.DefinePrim(scope.GetPath().AppendChild("Sub"), "Scope")
        deep = robot_schema.CreateNamedPose(self._stage, str(sub.GetPath().AppendChild("Deep")))

        ancestor = find_robot_ancestor(self._stage, deep)
        self.assertIsNotNone(ancestor)
        self.assertEqual(ancestor.GetPath(), robot_prim.GetPath())

    # -- get_site_candidates -----------------------------------------------

    async def test_get_site_candidates_includes_links(self) -> None:
        """Verify that get_site_candidates includes robot and link paths."""
        robot_prim, link_prim, _ = self._create_robot()
        candidates = get_site_candidates(self._stage, robot_prim)
        self.assertIn(str(robot_prim.GetPath()), candidates)
        self.assertIn(str(link_prim.GetPath()), candidates)

    async def test_get_site_candidates_includes_sites(self) -> None:
        """Verify that get_site_candidates includes IsaacSite prim paths."""
        robot_prim, link_prim, _ = self._create_robot()
        site = UsdGeom.Xform.Define(self._stage, "/World/Robot/Link1/ee_site")
        robot_schema.ApplySiteAPI(site.GetPrim())
        robot_utils.PopulateRobotSchemaFromArticulation(self._stage, robot_prim, robot_prim, detect_sites=True)
        candidates = get_site_candidates(self._stage, robot_prim)
        self.assertIn(str(site.GetPrim().GetPath()), candidates)

    async def test_get_site_candidates_sorted(self) -> None:
        """Verify that get_site_candidates returns paths in sorted order."""
        robot_prim, _, _ = self._create_robot()
        candidates = get_site_candidates(self._stage, robot_prim)
        self.assertEqual(candidates, sorted(candidates))

    async def test_get_site_candidates_excludes_named_poses_scope(self) -> None:
        """Verify that Xform fallback path does not include Named_Poses children."""
        plain = UsdGeom.Xform.Define(self._stage, "/World/PlainBot").GetPrim()
        UsdGeom.Xform.Define(self._stage, "/World/PlainBot/Link1")
        UsdGeom.Xform.Define(self._stage, "/World/PlainBot/NamedPoses")
        UsdGeom.Xform.Define(self._stage, "/World/PlainBot/NamedPoses/Pose1")

        candidates = get_site_candidates(self._stage, plain)
        for c in candidates:
            self.assertNotIn("NamedPoses", c)

    # -- write_transform_to_prim -------------------------------------------

    async def test_write_transform_to_prim(self) -> None:
        """Verify that write_transform_to_prim sets translate and orient ops on the prim."""
        prim = UsdGeom.Xform.Define(self._stage, "/World/Target").GetPrim()
        t = Transform(t=[1.0, 2.0, 3.0], q=[1.0, 0.0, 0.0, 0.0])
        write_transform_to_prim(prim, t)

        xformable = UsdGeom.Xformable(prim)
        ops = xformable.GetOrderedXformOps()
        translate_found = False
        orient_found = False
        for op in ops:
            if op.GetOpType() == UsdGeom.XformOp.TypeTranslate:
                v = op.Get()
                self.assertAlmostEqual(float(v[0]), 1.0)
                self.assertAlmostEqual(float(v[1]), 2.0)
                self.assertAlmostEqual(float(v[2]), 3.0)
                translate_found = True
            elif op.GetOpType() == UsdGeom.XformOp.TypeOrient:
                orient_found = True
        self.assertTrue(translate_found)
        self.assertTrue(orient_found)

    async def test_write_transform_preserves_existing_ops(self) -> None:
        """Verify that calling write_transform_to_prim twice reuses existing ops without error."""
        prim = UsdGeom.Xform.Define(self._stage, "/World/Target2").GetPrim()
        t1 = Transform(t=[1.0, 0.0, 0.0], q=[1.0, 0.0, 0.0, 0.0])
        t2 = Transform(t=[2.0, 3.0, 4.0], q=[1.0, 0.0, 0.0, 0.0])
        write_transform_to_prim(prim, t1)
        write_transform_to_prim(prim, t2)

        xformable = UsdGeom.Xformable(prim)
        for op in xformable.GetOrderedXformOps():
            if op.GetOpType() == UsdGeom.XformOp.TypeTranslate:
                v = op.Get()
                self.assertAlmostEqual(float(v[0]), 2.0)

    async def test_write_transform_respects_precision(self) -> None:
        """Verify that pre-existing float-precision ops keep float precision after write."""
        prim = UsdGeom.Xform.Define(self._stage, "/World/Float").GetPrim()
        xformable = UsdGeom.Xformable(prim)
        xformable.AddTranslateOp(UsdGeom.XformOp.PrecisionFloat).Set(Gf.Vec3f(0, 0, 0))
        xformable.AddOrientOp(UsdGeom.XformOp.PrecisionFloat).Set(Gf.Quatf(1, 0, 0, 0))

        t = Transform(t=[5.0, 6.0, 7.0], q=[1.0, 0.0, 0.0, 0.0])
        write_transform_to_prim(prim, t)

        for op in xformable.GetOrderedXformOps():
            if op.GetOpType() == UsdGeom.XformOp.TypeTranslate:
                self.assertEqual(op.GetPrecision(), UsdGeom.XformOp.PrecisionFloat)
                v = op.Get()
                self.assertAlmostEqual(float(v[0]), 5.0, places=4)

    # -- read_joint_limits_native ------------------------------------------

    async def test_read_joint_limits_revolute(self) -> None:
        """Verify that read_joint_limits_native returns name, is_revolute, and limits for a revolute joint."""
        _, _, joint_prim = self._create_robot()
        name, is_rev, lo, hi = read_joint_limits_native(self._stage, str(joint_prim.GetPath()))
        self.assertEqual(name, "joint1")
        self.assertTrue(is_rev)
        self.assertAlmostEqual(lo, -90.0)
        self.assertAlmostEqual(hi, 90.0)

    async def test_read_joint_limits_prismatic(self) -> None:
        """Verify that read_joint_limits_native returns limits for a prismatic joint."""
        robot_prim, link_prim, _ = self._create_robot()
        prism = UsdPhysics.PrismaticJoint.Define(self._stage, "/World/Robot/prism1")
        prism.CreateBody0Rel().SetTargets([robot_prim.GetPath()])
        prism.CreateBody1Rel().SetTargets([link_prim.GetPath()])
        prism.CreateAxisAttr("Z")
        prism.GetLowerLimitAttr().Set(0.0)
        prism.GetUpperLimitAttr().Set(1.5)

        name, is_rev, lo, hi = read_joint_limits_native(self._stage, "/World/Robot/prism1")
        self.assertEqual(name, "prism1")
        self.assertFalse(is_rev)
        self.assertAlmostEqual(lo, 0.0)
        self.assertAlmostEqual(hi, 1.5)

    async def test_read_joint_limits_invalid_path(self) -> None:
        """Verify that read_joint_limits_native returns infinite range for non-existent joint path."""
        name, is_rev, lo, hi = read_joint_limits_native(self._stage, "/World/NonExistent")
        self.assertEqual(name, "NonExistent")
        self.assertFalse(is_rev)
        self.assertTrue(np.isinf(lo))
        self.assertTrue(np.isinf(hi))

    async def test_read_joint_limits_unlimited(self) -> None:
        """Verify that joints with lower > upper are treated as unlimited (infinite range)."""
        self._create_robot()
        rev = UsdPhysics.RevoluteJoint.Define(self._stage, "/World/Robot/unlim")
        rev.GetLowerLimitAttr().Set(180.0)
        rev.GetUpperLimitAttr().Set(-180.0)
        _, _, lo, hi = read_joint_limits_native(self._stage, "/World/Robot/unlim")
        self.assertTrue(np.isinf(lo))
        self.assertTrue(np.isinf(hi))

    # -- build_pose_from_current_joints ------------------------------------

    async def test_build_pose_from_current_joints(self) -> None:
        """Verify that build_pose_from_current_joints returns PoseResult with joints and target."""
        robot_prim, link_prim, joint_prim = self._create_robot()

        result = build_pose_from_current_joints(
            self._stage,
            robot_prim,
            str(robot_prim.GetPath()),
            str(link_prim.GetPath()),
        )
        self.assertIsNotNone(result)
        self.assertTrue(result.success)
        self.assertIn(str(joint_prim.GetPath()), result.joints)
        self.assertIsNotNone(result.target_position)
        self.assertIsNotNone(result.target_orientation)
        self.assertEqual(len(result.target_position), 3)
        self.assertEqual(len(result.target_orientation), 4)

    async def test_build_pose_from_current_joints_invalid_paths(self) -> None:
        """Verify that build_pose_from_current_joints returns None for invalid start/end paths."""
        robot_prim, _, _ = self._create_robot()
        result = build_pose_from_current_joints(self._stage, robot_prim, "/World/Invalid", "/World/AlsoInvalid")
        self.assertIsNone(result)

    async def test_build_pose_joint_fixed_all_false(self) -> None:
        """Verify that pose built from current joints has all joint_fixed set to False."""
        robot_prim, link_prim, _ = self._create_robot()
        result = build_pose_from_current_joints(
            self._stage,
            robot_prim,
            str(robot_prim.GetPath()),
            str(link_prim.GetPath()),
        )
        self.assertIsNotNone(result)
        for val in result.joint_fixed.values():
            self.assertFalse(val)


class TestNamedPoseTableModel(omni.kit.test.AsyncTestCase):
    """Async tests for NamedPosesModel and NamedPoseItem data model."""

    async def setUp(self) -> None:
        """Create a fresh USD stage before each test."""
        await omni.usd.get_context().new_stage_async()
        await omni.kit.app.get_app().next_update_async()

    async def test_named_pose_item_construction(self) -> None:
        """Verify that NamedPoseItem stores name, sites, and prim path."""
        from isaacsim.robot.poser.ui.ui.named_pose_table import NamedPoseItem

        item = NamedPoseItem(
            name="Test",
            start_site="/World/Robot",
            end_site="/World/Robot/Link1",
            prim_path="/World/Robot/Named_Poses/Test",
        )
        self.assertEqual(item.name.get_value_as_string(), "Test")
        self.assertEqual(item.start_site.get_value_as_string(), "/World/Robot")
        self.assertEqual(item.end_site.get_value_as_string(), "/World/Robot/Link1")
        self.assertEqual(item.prim_path, "/World/Robot/Named_Poses/Test")
        self.assertFalse(item.tracking)

    async def test_named_pose_item_refresh_text(self) -> None:
        """Verify that refresh_text syncs the text field from the name model."""
        from isaacsim.robot.poser.ui.ui.named_pose_table import NamedPoseItem

        item = NamedPoseItem(name="Original", start_site="", end_site="")
        item.name.set_value("Renamed")
        item.refresh_text()
        self.assertEqual(item.text, "Renamed")

    async def test_named_poses_model_empty(self) -> None:
        """Verify that empty model starts with no searchable items."""
        from isaacsim.robot.poser.ui.ui.named_pose_table import (
            NamedPoseItem,
            NamedPosesModel,
        )

        model = NamedPosesModel([])
        children = model.get_item_children(None)
        pose_items = [c for c in children if isinstance(c, NamedPoseItem)]
        self.assertEqual(len(pose_items), 0)

    async def test_named_poses_model_add_item(self) -> None:
        """Verify that add_item appends a NamedPoseItem into the model."""
        from isaacsim.robot.poser.ui.ui.named_pose_table import (
            NamedPoseItem,
            NamedPosesModel,
        )

        model = NamedPosesModel([])
        item = NamedPoseItem(name="P1", start_site="/A", end_site="/B", prim_path="/P1")
        model.add_item(item)

        children = model.get_item_children(None)
        pose_items = [c for c in children if isinstance(c, NamedPoseItem)]
        self.assertEqual(len(pose_items), 1)
        self.assertEqual(pose_items[0].name.get_value_as_string(), "P1")

    async def test_named_poses_model_add_multiple(self) -> None:
        """Verify that add_item can be called multiple times; all items appear as children."""
        from isaacsim.robot.poser.ui.ui.named_pose_table import (
            NamedPoseItem,
            NamedPosesModel,
        )

        model = NamedPosesModel([])
        for n in ("A", "B", "C"):
            model.add_item(NamedPoseItem(name=n, start_site="", end_site="", prim_path=f"/{n}"))

        children = model.get_item_children(None)
        pose_items = [c for c in children if isinstance(c, NamedPoseItem)]
        self.assertEqual(len(pose_items), 3)

    async def test_named_poses_model_remove_item(self) -> None:
        """Verify that remove_item removes the given item from the model."""
        from isaacsim.robot.poser.ui.ui.named_pose_table import (
            NamedPoseItem,
            NamedPosesModel,
        )

        model = NamedPosesModel([])
        item = NamedPoseItem(name="P1", start_site="/A", end_site="/B", prim_path="/P1")
        model.add_item(item)
        model.remove_item(item)

        children = model.get_item_children(None)
        pose_items = [c for c in children if isinstance(c, NamedPoseItem)]
        self.assertEqual(len(pose_items), 0)

    async def test_named_poses_model_filter_by_text(self) -> None:
        """Verify that filter_by_text restricts visible NamedPoseItems by substring."""
        from isaacsim.robot.poser.ui.ui.named_pose_table import (
            NamedPoseItem,
            NamedPosesModel,
        )

        model = NamedPosesModel([])
        for name in ("alpha", "beta", "gamma"):
            model.add_item(NamedPoseItem(name=name, start_site="", end_site="", prim_path=f"/P/{name}"))

        model.filter_by_text(["al"])
        children = model.get_item_children(None)
        pose_items = [c for c in children if isinstance(c, NamedPoseItem)]
        names = [i.name.get_value_as_string() for i in pose_items]
        self.assertIn("alpha", names)
        self.assertNotIn("beta", names)
        self.assertNotIn("gamma", names)

    async def test_named_poses_model_clear_filter(self) -> None:
        """Verify that clearing filter text restores all items."""
        from isaacsim.robot.poser.ui.ui.named_pose_table import (
            NamedPoseItem,
            NamedPosesModel,
        )

        model = NamedPosesModel([])
        for name in ("alpha", "beta", "gamma"):
            model.add_item(NamedPoseItem(name=name, start_site="", end_site="", prim_path=f"/P/{name}"))

        model.filter_by_text(["xyz"])
        model.filter_by_text([])

        children = model.get_item_children(None)
        pose_items = [c for c in children if isinstance(c, NamedPoseItem)]
        self.assertEqual(len(pose_items), 3)

    async def test_named_poses_model_set_items(self) -> None:
        """Verify that set_items replaces all items in the model."""
        from isaacsim.robot.poser.ui.ui.named_pose_table import (
            NamedPoseItem,
            NamedPosesModel,
        )

        model = NamedPosesModel([])
        model.add_item(NamedPoseItem(name="old", start_site="", end_site="", prim_path="/old"))

        new_items = [
            NamedPoseItem(name="new1", start_site="", end_site="", prim_path="/new1"),
            NamedPoseItem(name="new2", start_site="", end_site="", prim_path="/new2"),
        ]
        model.set_items(new_items)

        children = model.get_item_children(None)
        pose_items = [c for c in children if isinstance(c, NamedPoseItem)]
        names = {i.name.get_value_as_string() for i in pose_items}
        self.assertEqual(names, {"new1", "new2"})

    async def test_named_poses_model_find_unique_name(self) -> None:
        """Verify that find_unique_name returns a name not already used by an item."""
        from isaacsim.robot.poser.ui.ui.named_pose_table import (
            NamedPoseItem,
            NamedPosesModel,
        )

        model = NamedPosesModel([])
        model.add_item(NamedPoseItem(name="pose1", start_site="", end_site="", prim_path="/p1"))

        unique = model.find_unique_name("pose")
        self.assertNotEqual(unique, "pose1")
        self.assertTrue(unique.startswith("pose"))

    async def test_named_poses_model_get_item_value_model(self) -> None:
        """Verify that column 1 returns name, column 2 returns start_site, column 3 returns end_site."""
        from isaacsim.robot.poser.ui.ui.named_pose_table import (
            NamedPoseItem,
            NamedPosesModel,
        )

        model = NamedPosesModel([])
        item = NamedPoseItem(name="Test", start_site="/S", end_site="/E", prim_path="/p")
        model.add_item(item)

        self.assertEqual(model.get_item_value_model(item, 1).get_value_as_string(), "Test")
        self.assertEqual(model.get_item_value_model(item, 2).get_value_as_string(), "/S")
        self.assertEqual(model.get_item_value_model(item, 3).get_value_as_string(), "/E")
        self.assertIsNone(model.get_item_value_model(item, 99))

    async def test_named_poses_model_drop_accepted(self) -> None:
        """Verify that drop is accepted between two NamedPoseItems."""
        from isaacsim.robot.poser.ui.ui.named_pose_table import (
            NamedPoseItem,
            NamedPosesModel,
        )

        model = NamedPosesModel([])
        a = NamedPoseItem(name="A", start_site="", end_site="", prim_path="/A")
        b = NamedPoseItem(name="B", start_site="", end_site="", prim_path="/B")
        model.add_item(a)
        model.add_item(b)

        self.assertTrue(model.drop_accepted(a, b))
        self.assertFalse(model.drop_accepted(a, None))
