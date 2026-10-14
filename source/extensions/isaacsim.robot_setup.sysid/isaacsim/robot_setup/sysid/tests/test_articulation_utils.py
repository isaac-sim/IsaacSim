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

"""Tests for articulation-root discovery."""

from __future__ import annotations

from types import SimpleNamespace
from unittest.mock import patch

import omni.kit.test
from isaacsim.robot_setup.sysid.articulation_utils import (
    collect_robot_link_and_joint_paths,
    find_articulation_root,
    order_joint_paths_for_trajectory,
)
from pxr import Usd, UsdPhysics


class ArticulationUtilsTests(omni.kit.test.AsyncTestCase):
    """Keep articulation discovery within the selected robot hierarchy."""

    async def test_selected_sibling_does_not_resolve_to_unrelated_robot(self) -> None:
        """Do not escape from an environment prim into a sibling articulation."""
        stage = Usd.Stage.CreateInMemory()
        stage.DefinePrim("/World/Table")
        robot = stage.DefinePrim("/World/Robot")
        UsdPhysics.ArticulationRootAPI.Apply(robot)

        self.assertIsNone(find_articulation_root(stage, "/World/Table"))

    async def test_selected_link_resolves_to_ancestor_articulation(self) -> None:
        """Resolve a link selection by inspecting its ancestor chain."""
        stage = Usd.Stage.CreateInMemory()
        robot = stage.DefinePrim("/World/Robot")
        UsdPhysics.ArticulationRootAPI.Apply(robot)
        stage.DefinePrim("/World/Robot/Links/Arm")

        self.assertEqual(find_articulation_root(stage, "/World/Robot/Links/Arm"), "/World/Robot")

    async def test_ambiguous_peer_articulations_require_explicit_selection(self) -> None:
        """Reject a container with two equally close articulation roots."""
        stage = Usd.Stage.CreateInMemory()
        stage.DefinePrim("/World/Robots")
        left = stage.DefinePrim("/World/Robots/Left")
        right = stage.DefinePrim("/World/Robots/Right")
        UsdPhysics.ArticulationRootAPI.Apply(left)
        UsdPhysics.ArticulationRootAPI.Apply(right)

        self.assertIsNone(find_articulation_root(stage, "/World/Robots"))

    async def test_geometry_named_ancestor_does_not_demote_nearest_root(self) -> None:
        """Choose by hierarchy depth without assigning semantics to prim names."""
        stage = Usd.Stage.CreateInMemory()
        stage.DefinePrim("/World/Container")
        nearest = stage.DefinePrim("/World/Container/Geometry/Robot")
        farther = stage.DefinePrim("/World/Container/Assembly/Nested/Robot")
        UsdPhysics.ArticulationRootAPI.Apply(nearest)
        UsdPhysics.ArticulationRootAPI.Apply(farther)

        self.assertEqual(
            find_articulation_root(stage, "/World/Container"),
            "/World/Container/Geometry/Robot",
        )

    async def test_collect_robot_paths_finds_rigid_links_and_scalar_joints(self) -> None:
        """Collect supported links and joints below the resolved articulation root."""
        stage = Usd.Stage.CreateInMemory()
        robot = stage.DefinePrim("/World/Robot")
        UsdPhysics.ArticulationRootAPI.Apply(robot)
        link = stage.DefinePrim("/World/Robot/Links/Arm")
        UsdPhysics.RigidBodyAPI.Apply(link)
        UsdPhysics.RevoluteJoint.Define(stage, "/World/Robot/Joints/Shoulder")
        UsdPhysics.PrismaticJoint.Define(stage, "/World/Robot/Joints/Slider")

        links, joints = collect_robot_link_and_joint_paths(stage, "/World/Robot")

        self.assertEqual(links, ["/World/Robot/Links/Arm"])
        self.assertEqual(
            joints,
            ["/World/Robot/Joints/Shoulder", "/World/Robot/Joints/Slider"],
        )

    async def test_collect_robot_paths_finds_joints_sibling_to_nested_articulation_root(self) -> None:
        """Find joints in a sibling Physics scope of a base-link articulation root."""
        stage = Usd.Stage.CreateInMemory()
        stage.DefinePrim("/World/Robot", "Xform")
        base = stage.DefinePrim("/World/Robot/Geometry/Base", "Xform")
        UsdPhysics.ArticulationRootAPI.Apply(base)
        UsdPhysics.RigidBodyAPI.Apply(base)
        UsdPhysics.RevoluteJoint.Define(stage, "/World/Robot/Physics/Shoulder")

        links, joints = collect_robot_link_and_joint_paths(stage, "/World/Robot")

        self.assertEqual(links, ["/World/Robot/Geometry/Base"])
        self.assertEqual(joints, ["/World/Robot/Physics/Shoulder"])

    async def test_collect_robot_paths_uses_body_wiring_to_exclude_foreign_joints(self) -> None:
        """Associate joints by body relationships so a shared scope's foreign joints are excluded."""
        stage = Usd.Stage.CreateInMemory()
        stage.DefinePrim("/World/Cell", "Xform")
        selected = stage.DefinePrim("/World/Cell/RobotA", "Xform")
        UsdPhysics.ArticulationRootAPI.Apply(selected)
        UsdPhysics.RigidBodyAPI.Apply(stage.DefinePrim("/World/Cell/RobotA/Base", "Xform"))
        UsdPhysics.RigidBodyAPI.Apply(stage.DefinePrim("/World/Cell/RobotB/Base", "Xform"))
        # Both robots author their joints in a shared sibling scope, wired via body rels.
        own = UsdPhysics.RevoluteJoint.Define(stage, "/World/Cell/Physics/A_Shoulder")
        own.CreateBody1Rel().SetTargets(["/World/Cell/RobotA/Base"])
        foreign = UsdPhysics.RevoluteJoint.Define(stage, "/World/Cell/Physics/B_Shoulder")
        foreign.CreateBody1Rel().SetTargets(["/World/Cell/RobotB/Base"])

        links, joints = collect_robot_link_and_joint_paths(stage, "/World/Cell/RobotA")

        self.assertEqual(links, ["/World/Cell/RobotA/Base"])
        self.assertEqual(joints, ["/World/Cell/Physics/A_Shoulder"])

    async def test_joint_order_uses_telemetry_names_and_preserves_remainder(self) -> None:
        """Order matched joints by telemetry metadata and retain unmentioned joints."""
        paths = ["/Robot/Joints/A", "/Robot/Joints/B", "/Robot/Joints/C"]
        trajectory = SimpleNamespace(metadata=SimpleNamespace(extra={"joint_names": ["B", "A"]}))

        ordered = order_joint_paths_for_trajectory(paths, trajectory)

        self.assertEqual(ordered, ["/Robot/Joints/B", "/Robot/Joints/A", "/Robot/Joints/C"])

    async def test_joint_order_without_names_keeps_traversal_order(self) -> None:
        """Keep the input ordering when telemetry has no joint-name metadata."""
        paths = ["/Robot/Joints/A", "/Robot/Joints/B"]
        trajectory = SimpleNamespace(metadata=SimpleNamespace(extra={}))

        self.assertIs(order_joint_paths_for_trajectory(paths, trajectory), paths)

    async def test_duplicate_joint_leaf_names_warn_and_keep_traversal_order(self) -> None:
        """Reject ambiguous telemetry ordering when USD joint leaf names collide."""
        paths = ["/Robot/Arm/Joint", "/Robot/Leg/Joint"]
        trajectory = SimpleNamespace(metadata=SimpleNamespace(extra={"joint_names": ["Joint"]}))

        with patch("carb.log_warn") as log_warn:
            ordered = order_joint_paths_for_trajectory(paths, trajectory)

        self.assertEqual(ordered, paths)
        messages = [call.args[0] for call in log_warn.call_args_list]
        self.assertTrue(any("multiple joint prims share leaf name" in message for message in messages))
