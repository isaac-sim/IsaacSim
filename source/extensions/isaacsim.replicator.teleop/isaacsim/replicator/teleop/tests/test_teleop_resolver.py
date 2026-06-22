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

"""Tests for teleop profile validation against stage state."""

from __future__ import annotations

import omni.kit.app
import omni.kit.test
import omni.usd
from isaacsim.replicator.teleop import (
    STAGE_STATE_NO_STAGE,
    STAGE_STATE_READY,
    BimanualControllerProfile,
    ControllerSideProfile,
    LocomotionProfile,
    TeleopProfile,
    TeleopSettingsProfile,
    resolve_teleop_profile,
)


class TestTeleopResolver(omni.kit.test.AsyncTestCase):
    """Verify teleop profile validation against current stage state."""

    async def setUp(self) -> None:
        """Set up the test fixture."""
        await omni.kit.app.get_app().next_update_async()
        omni.usd.get_context().new_stage()
        await omni.kit.app.get_app().next_update_async()

    async def tearDown(self) -> None:
        """Tear down the test fixture."""
        usd_context = omni.usd.get_context()
        if usd_context.get_stage() is not None:
            usd_context.close_stage()
            await omni.kit.app.get_app().next_update_async()
        while usd_context.get_stage_loading_status()[2] > 0:
            await omni.kit.app.get_app().next_update_async()

    async def test_resolver_reports_no_stage(self) -> None:
        """Verify the resolver reports no-stage when no USD stage is open."""
        usd_context = omni.usd.get_context()
        usd_context.close_stage()
        await omni.kit.app.get_app().next_update_async()
        while usd_context.get_stage_loading_status()[2] > 0:
            await omni.kit.app.get_app().next_update_async()

        report = resolve_teleop_profile(TeleopProfile())

        self.assertEqual(report.stage_state, STAGE_STATE_NO_STAGE)
        self.assertFalse(report.ready)
        self.assertEqual(report.error_count, 0)

    async def test_resolver_reports_missing_stage_references(self) -> None:
        """Verify the resolver surfaces errors for prims that do not exist on stage."""
        profile = TeleopProfile(
            session=TeleopSettingsProfile(
                tracking_space_enabled=True,
                tracking_space_path="/World/MissingTrackingSpace",
            ),
            floating=BimanualControllerProfile(
                left=ControllerSideProfile(
                    enabled=True,
                    settings={"prim_path": "/World/MissingRigidBody"},
                )
            ),
            locomotion=LocomotionProfile(
                enabled=True,
                settings={"prim_path": "/World/MissingBase"},
            ),
        )

        report = resolve_teleop_profile(profile)

        self.assertEqual(report.stage_state, STAGE_STATE_READY)
        self.assertFalse(report.ready)
        self.assertGreaterEqual(report.error_count, 2)
        self.assertGreaterEqual(report.warning_count, 1)
        issue_sources = {issue.source for issue in report.issues}
        self.assertIn("Session Tracking Space", issue_sources)
        self.assertIn("Floating Left", issue_sources)
        self.assertIn("Locomotion", issue_sources)
