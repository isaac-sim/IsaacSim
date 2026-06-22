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

"""Tests for the Joint Inspector column catalogue and resolution logic."""

from __future__ import annotations

import omni.kit.app
import omni.kit.test
import omni.usd
from isaacsim.gui.property.joint_inspector import (
    _COLUMN_BY_ID,
    _COLUMNS,
    _DEFAULT_VISIBLE_COLUMN_IDS,
    _column_available_for,
    _column_axes_for_joints,
    _resolve_columns,
)
from pxr import PhysxSchema, UsdPhysics


class TestJointInspectorColumnResolution(omni.kit.test.AsyncTestCase):
    """Validates per-axis coalescing and catalogue resolution for the inspector."""

    async def setUp(self) -> None:
        """Create a fresh empty USD stage for each test case."""
        await omni.usd.get_context().new_stage_async()
        await omni.kit.app.get_app().next_update_async()
        self._stage = omni.usd.get_context().get_stage()

    def _revolute(self, path: str) -> UsdPhysics.RevoluteJoint:
        """Create a revolute joint at `path` with a DriveAPI on its `angular` axis.

        Args:
            path: Joint path to create.

        Returns:
            Created joint.
        """
        joint = UsdPhysics.RevoluteJoint.Define(self._stage, path)
        UsdPhysics.DriveAPI.Apply(joint.GetPrim(), "angular")
        return joint

    def _prismatic(self, path: str) -> UsdPhysics.PrismaticJoint:
        """Create a prismatic joint at `path` with a DriveAPI on its `linear` axis.

        Args:
            path: Joint path to create.

        Returns:
            Created joint.
        """
        joint = UsdPhysics.PrismaticJoint.Define(self._stage, path)
        UsdPhysics.DriveAPI.Apply(joint.GetPrim(), "linear")
        return joint

    def _d6_with_axes(self, path: str, axes: tuple[str, ...]) -> UsdPhysics.Joint:
        """Create a generic joint with DriveAPI applied on `axes`.

        Args:
            path: Joint path to create.
            axes: Drive API axes to apply.

        Returns:
            Created joint.
        """
        joint = UsdPhysics.Joint.Define(self._stage, path)
        for axis in axes:
            UsdPhysics.DriveAPI.Apply(joint.GetPrim(), axis)
        return joint

    async def test_catalogue_is_consistent(self) -> None:
        """Every catalogue entry is reachable via `_COLUMN_BY_ID` and has a unique id."""
        ids = [c.id for c in _COLUMNS]
        self.assertEqual(len(ids), len(set(ids)), msg="Column ids must be unique")
        for spec in _COLUMNS:
            self.assertIs(_COLUMN_BY_ID[spec.id], spec)

    async def test_defaults_are_present_in_catalogue(self) -> None:
        """`_DEFAULT_VISIBLE_COLUMN_IDS` entries all exist in the catalogue."""
        for cid in _DEFAULT_VISIBLE_COLUMN_IDS:
            self.assertIn(cid, _COLUMN_BY_ID)

    async def test_single_dof_joints_collapse_drive_columns(self) -> None:
        """Revolute + prismatic mixes produce a single coalesced axis slot per drive column."""
        joints = [
            self._revolute("/World/r0").GetPrim(),
            self._revolute("/World/r1").GetPrim(),
            self._prismatic("/World/p0").GetPrim(),
        ]
        drive_spec = _COLUMN_BY_ID["drive_stiffness"]
        axes = _column_axes_for_joints(drive_spec, joints)
        self.assertEqual(axes, [""], msg="All joints are single-DOF; drive should collapse to one column")

    async def test_multi_dof_joint_fans_out_axes(self) -> None:
        """A D6 joint authoring two axes forces the column to fan out per-axis."""
        joints = [
            self._revolute("/World/r0").GetPrim(),
            self._d6_with_axes("/World/d6", ("transX", "rotZ")).GetPrim(),
        ]
        drive_spec = _COLUMN_BY_ID["drive_stiffness"]
        axes = _column_axes_for_joints(drive_spec, joints)
        # Expect distinct authored axes across the joint set, ordered by
        # `_AXIS_PRIORITY`: angular, linear, transX..Z, rotX..Z.
        self.assertEqual(set(axes), {"angular", "transX", "rotZ"})
        self.assertGreater(len(axes), 1, msg="Multi-DOF joint should force fan-out")

    async def test_non_axis_column_returns_single_slot(self) -> None:
        """Columns without `axis_api` always return the empty-axis slot."""
        limit_spec = _COLUMN_BY_ID["limit_lower"]
        self.assertEqual(_column_axes_for_joints(limit_spec, []), [""])
        joints = [self._revolute("/World/r0").GetPrim()]
        self.assertEqual(_column_axes_for_joints(limit_spec, joints), [""])

    async def test_resolve_columns_drops_axis_columns_without_api(self) -> None:
        """Axis-API columns are dropped from `_resolve_columns` when no joint authors them.

        `_resolve_columns` preserves non-axis columns regardless (they are
        disabled at cell-render time via `_ColumnSpec.api_test`), but axis-API
        columns collapse to an empty axis list when no joint in the set
        authors the API, so they drop out of the resolved list entirely.
        """
        # A plain joint with no DriveAPI / JointStateAPI / PerfEnvelope / MjcJointAPI.
        bare_joint = UsdPhysics.Joint.Define(self._stage, "/World/bare").GetPrim()
        visible = {"drive_stiffness", "perf_max_actuator_velocity", "state_position", "limit_lower"}
        resolved = _resolve_columns(visible, [bare_joint])
        resolved_ids = {rc.spec.id for rc in resolved}
        self.assertNotIn("drive_stiffness", resolved_ids)
        self.assertNotIn("perf_max_actuator_velocity", resolved_ids)
        self.assertNotIn("state_position", resolved_ids)
        # Non-axis columns survive resolution and are disabled per-cell.
        self.assertIn("limit_lower", resolved_ids)

    async def test_column_available_reflects_applied_apis(self) -> None:
        """`_column_available_for` distinguishes authored vs unauthored API columns.

        This is the predicate the columns popup uses to dim entries for
        unavailable APIs.
        """
        bare_joint = UsdPhysics.Joint.Define(self._stage, "/World/bare").GetPrim()
        rev_with_state = self._revolute("/World/r0").GetPrim()
        PhysxSchema.JointStateAPI.Apply(rev_with_state, "angular")

        drive_spec = _COLUMN_BY_ID["drive_stiffness"]
        state_spec = _COLUMN_BY_ID["state_position"]
        limit_spec = _COLUMN_BY_ID["limit_lower"]

        self.assertFalse(_column_available_for(drive_spec, [bare_joint]))
        self.assertTrue(_column_available_for(drive_spec, [rev_with_state]))
        self.assertTrue(_column_available_for(state_spec, [rev_with_state]))
        self.assertTrue(_column_available_for(limit_spec, [rev_with_state]))
        self.assertFalse(_column_available_for(limit_spec, [bare_joint]))

    async def test_resolve_columns_preserves_catalogue_order(self) -> None:
        """Resolved columns follow `_COLUMNS` ordering even if `visible_ids` is unsorted."""
        rev = self._revolute("/World/r0").GetPrim()
        PhysxSchema.JointStateAPI.Apply(rev, "angular")
        visible = {"state_position", "limit_lower", "drive_stiffness"}
        resolved = _resolve_columns(visible, [rev])
        order = [rc.spec.id for rc in resolved]
        # `limit_lower` appears before `drive_stiffness` before `state_position` in `_COLUMNS`.
        self.assertLess(order.index("limit_lower"), order.index("drive_stiffness"))
        self.assertLess(order.index("drive_stiffness"), order.index("state_position"))

    async def test_resolved_column_label_includes_axis_suffix(self) -> None:
        """Multi-DOF fan-out decorates labels with the axis token."""
        joints = [
            self._revolute("/World/r0").GetPrim(),
            self._d6_with_axes("/World/d6", ("transX", "rotZ")).GetPrim(),
        ]
        resolved = _resolve_columns({"drive_stiffness"}, joints)
        labels = [rc.label for rc in resolved]
        # At least one per-axis decorated label should be present (the exact set
        # depends on `_AXIS_PRIORITY` ordering; assert the pattern rather than
        # exact membership).
        self.assertTrue(any("[" in lbl and "]" in lbl for lbl in labels))
