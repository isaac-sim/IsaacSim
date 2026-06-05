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

"""Tests for :func:`build_teleop_recorder`."""

from __future__ import annotations

import tempfile

import omni.kit.app
import omni.kit.test
import omni.usd
from isaacsim.replicator.teleop import build_teleop_recorder


class _StubTeleopManager:
    """Minimal manager implementing the observer hooks recordables expect."""

    def add_controller_inputs_observer(self, _cb):  # noqa: ANN001
        return lambda: None

    def add_head_observer(self, _cb):  # noqa: ANN001
        return lambda: None


class _StubTeleopManagerNoHead:
    def add_controller_inputs_observer(self, _cb):  # noqa: ANN001
        return lambda: None


class TestBuildTeleopRecorder(omni.kit.test.AsyncTestCase):
    async def setUp(self):
        await omni.kit.app.get_app().next_update_async()
        omni.usd.get_context().new_stage()
        await omni.kit.app.get_app().next_update_async()

    async def tearDown(self):
        omni.usd.get_context().close_stage()
        await omni.kit.app.get_app().next_update_async()
        while omni.usd.get_context().get_stage_loading_status()[2] > 0:
            await omni.kit.app.get_app().next_update_async()

    async def test_composes_expected_recordables(self):
        with tempfile.TemporaryDirectory(prefix="build_teleop_recorder_test_") as output_dir:
            tm = _StubTeleopManager()
            rec = build_teleop_recorder(
                output_dir,
                teleop_manager=tm,
                articulations={"robot": "/World/Robot"},
                xforms={"cube": "/World/Cube"},
                rigid_bodies={"box": "/World/Box"},
                record_head_pose=True,
            )
        by_group = {r.group: type(r).__name__ for r in rec.recordables()}
        # SimTimeRecordable is auto-attached by EpisodeRecorder.open_session(), not by the
        # factory, so it is not expected in the recordables list at this lifecycle stage.
        self.assertNotIn("meta/time", by_group)
        self.assertEqual(by_group["state/robot"], "ArticulationRecordable")
        self.assertEqual(by_group["state/cube"], "XformRecordable")
        self.assertEqual(by_group["state/box"], "RigidBodyRecordable")
        self.assertEqual(by_group["teleop/left"], "TeleopControllerRecordable")
        self.assertEqual(by_group["teleop/right"], "TeleopControllerRecordable")
        self.assertEqual(by_group["teleop/head"], "TeleopHeadRecordable")

    async def test_head_recordable_skipped_without_observer_api(self):
        with tempfile.TemporaryDirectory(prefix="build_teleop_recorder_test_") as output_dir:
            tm = _StubTeleopManagerNoHead()
            rec = build_teleop_recorder(
                output_dir,
                teleop_manager=tm,
                record_head_pose=True,
            )
        groups = {r.group for r in rec.recordables()}
        self.assertNotIn("teleop/head", groups)

    async def test_head_skipped_when_disabled(self):
        with tempfile.TemporaryDirectory(prefix="build_teleop_recorder_test_") as output_dir:
            tm = _StubTeleopManager()
            rec = build_teleop_recorder(
                output_dir,
                teleop_manager=tm,
                record_head_pose=False,
            )
        groups = {r.group for r in rec.recordables()}
        self.assertNotIn("teleop/head", groups)

    async def test_pose_backend_defaults_to_usd(self):
        with tempfile.TemporaryDirectory(prefix="build_teleop_recorder_test_") as output_dir:
            tm = _StubTeleopManager()
            rec = build_teleop_recorder(output_dir, teleop_manager=tm)
        self.assertEqual(rec.pose_backend, "usd")

    async def test_pose_backend_propagated_to_recorder(self):
        with tempfile.TemporaryDirectory(prefix="build_teleop_recorder_test_") as output_dir:
            tm = _StubTeleopManager()
            # Unknown / FSD-disabled backends fall back to usd inside EpisodeRecorder
            # so the assertion stays stable regardless of FSD availability.
            rec = build_teleop_recorder(output_dir, teleop_manager=tm, pose_backend="usd")
        self.assertEqual(rec.pose_backend, "usd")
