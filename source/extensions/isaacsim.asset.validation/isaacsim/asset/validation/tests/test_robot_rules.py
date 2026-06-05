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

"""Tests for robot validation rules.

Regression coverage for ``JointsExist`` and ``LinksExist``: rigid-body-only
assets (e.g. vehicles, non-articulated props) must not be warned for missing
``isaac:physics:JointAPI`` / ``LinkAPI``.
"""

from __future__ import annotations

import omni.kit.test
from isaacsim.asset.validation.robot_rules import JointsExist, LinksExist
from pxr import Usd, UsdPhysics


class _WarningCapturingJointsExist(JointsExist):
    """Subclass that captures ``_AddWarning`` emissions."""

    def __init__(self) -> None:
        super().__init__()
        self.warnings: list[str] = []

    def _AddWarning(self, message: str, **kwargs: object) -> None:  # type: ignore[override]  # noqa: N802
        self.warnings.append(message)


class _WarningCapturingLinksExist(LinksExist):
    """Subclass that captures ``_AddWarning`` emissions."""

    def __init__(self) -> None:
        super().__init__()
        self.warnings: list[str] = []

    def _AddWarning(self, message: str, **kwargs: object) -> None:  # type: ignore[override]  # noqa: N802
        self.warnings.append(message)


class TestJointsExist(omni.kit.test.AsyncTestCase):
    """Regression tests for the ``JointsExist`` joint-presence gate."""

    async def test_rigid_body_only_stage_silent(self) -> None:
        """basic_vehicle_m / Ant_colored shape: rigid bodies, no joints, no warning."""
        stage = Usd.Stage.CreateInMemory()
        prim = stage.DefinePrim("/Vehicle/chassis", "Xform")
        UsdPhysics.RigidBodyAPI.Apply(prim)
        checker = _WarningCapturingJointsExist()
        checker.CheckStage(stage)
        self.assertEqual(checker.warnings, [])

    async def test_joints_without_joint_api_still_warns(self) -> None:
        """Regression: joints authored but isaac:physics:JointAPI missing — warning preserved."""
        stage = Usd.Stage.CreateInMemory()
        UsdPhysics.RevoluteJoint.Define(stage, "/Robot/joints/j0")
        # No robot_schema JointAPI applied intentionally.
        checker = _WarningCapturingJointsExist()
        checker.CheckStage(stage)
        self.assertEqual(len(checker.warnings), 1)
        self.assertIn("No joints found", checker.warnings[0])


class TestLinksExist(omni.kit.test.AsyncTestCase):
    """Regression tests for the ``LinksExist`` joint-presence gate."""

    async def test_rigid_body_only_stage_silent(self) -> None:
        """Rigid bodies, no joints → no warning (gate active)."""
        stage = Usd.Stage.CreateInMemory()
        prim = stage.DefinePrim("/Vehicle/chassis", "Xform")
        UsdPhysics.RigidBodyAPI.Apply(prim)
        checker = _WarningCapturingLinksExist()
        checker.CheckStage(stage)
        self.assertEqual(checker.warnings, [])

    async def test_joints_without_link_api_still_warns(self) -> None:
        """Regression: joints authored but isaac:physics:LinkAPI missing — warning preserved."""
        stage = Usd.Stage.CreateInMemory()
        UsdPhysics.RevoluteJoint.Define(stage, "/Robot/joints/j0")
        checker = _WarningCapturingLinksExist()
        checker.CheckStage(stage)
        self.assertEqual(len(checker.warnings), 1)
        self.assertIn("No links found", checker.warnings[0])
