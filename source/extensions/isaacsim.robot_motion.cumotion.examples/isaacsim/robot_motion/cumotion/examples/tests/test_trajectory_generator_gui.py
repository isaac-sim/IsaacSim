# SPDX-FileCopyrightText: Copyright (c) 2022-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

"""Tests for the UR10 trajectory generator example GUI."""

from unittest.mock import patch

import omni.kit.app
import omni.kit.test
from isaacsim.core.experimental.utils import app as app_utils
from isaacsim.core.experimental.utils import stage as stage_utils
from isaacsim.robot_motion.cumotion.examples.trajectory_generator.ui_builder import UIBuilder

from .gui_test_support import (
    TEST_LOAD_TIMEOUT_SEC,
    WARMUP_LOAD_TIMEOUT_SEC,
    ensure_gui_class_warmup_once,
    wait_until,
)

_ROBOT_PATH = "/ur10"
_PHYSICS_SCENE_PATH = "/World/PhysicsScene"


class TestTrajectoryGeneratorGui(omni.kit.test.AsyncTestCase):
    """Test suite for trajectory generator GUI."""

    async def setUp(self) -> None:
        """Set up the UI builder before each test."""
        await omni.kit.app.get_app().next_update_async()
        await ensure_gui_class_warmup_once(
            type(self),
            ui_builder_cls=UIBuilder,
            wait_for_load=lambda wb: self._load_until_articulation_on(wb, timeout_sec=WARMUP_LOAD_TIMEOUT_SEC),
        )
        self.ui_builder = UIBuilder()
        self.ui_builder.build_ui()
        await omni.kit.app.get_app().next_update_async()

    async def tearDown(self) -> None:
        """Clean up the UI builder after each test."""
        self.ui_builder.cleanup()
        await omni.kit.app.get_app().next_update_async()
        await omni.kit.app.get_app().next_update_async()

    @classmethod
    async def _load_until_articulation_on(cls, ui_builder: UIBuilder, *, timeout_sec: float) -> None:
        ui_builder._load_btn.trigger_click()
        ok = await wait_until(
            lambda: ui_builder._scenario._articulation is not None,
            timeout_sec=timeout_sec,
        )
        if not ok:
            raise AssertionError("Timed out waiting for UR10 articulation after load")

    async def _load_until_articulation(self) -> None:
        await self._load_until_articulation_on(self.ui_builder, timeout_sec=TEST_LOAD_TIMEOUT_SEC)

    async def test_widgets_built(self) -> None:
        """Verify that all expected widgets are created."""
        self.assertIsNotNone(self.ui_builder._load_btn)
        self.assertIsNotNone(self.ui_builder._reset_btn)
        self.assertIsNotNone(self.ui_builder._cspace_trajectory_btn)
        self.assertIsNotNone(self.ui_builder._taskspace_trajectory_btn)
        self.assertIsNotNone(self.ui_builder._hybrid_trajectory_btn)

    async def test_load_enables_articulation(self) -> None:
        """Test that loading creates the articulation."""
        await self._load_until_articulation()

    async def test_load_creates_all_expected_assets(self) -> None:
        """LOAD populates every expected scenario object and prim on the stage."""
        self.ui_builder._load_btn.trigger_click()

        ok = await wait_until(
            lambda: self.ui_builder._load_task is not None and self.ui_builder._load_task.done(),
            timeout_sec=120.0,
        )
        self.assertTrue(ok, "Timed out waiting for load to complete")

        # Surface any exception raised inside the load coroutine.
        load_exc = self.ui_builder._load_task.exception()
        if load_exc is not None:
            raise load_exc

        # All expected scenario state must be populated.
        s = self.ui_builder._scenario
        self.assertIsNotNone(s._articulation, "scenario._articulation should be set after LOAD")
        self.assertIsNotNone(s._robot_config, "scenario._robot_config should be set after LOAD")
        self.assertIsNotNone(s._generator, "scenario._generator should be set after LOAD")
        self.assertIsNotNone(s._robot_joint_space, "scenario._robot_joint_space should be set after LOAD")
        self.assertIsNotNone(s._controlled_joint_names, "scenario._controlled_joint_names should be set after LOAD")

        # All expected prims must be on the stage.
        stage = stage_utils.get_current_stage()
        self.assertIsNotNone(stage, "Stage should exist after LOAD")
        for path in (_ROBOT_PATH, _PHYSICS_SCENE_PATH):
            self.assertTrue(stage.GetPrimAtPath(path).IsValid(), f"Expected prim {path!r} on stage after LOAD")

    async def test_reset_triggers_scenario_reset(self) -> None:
        """Test that the reset button triggers a scenario reset."""
        await self._load_until_articulation()
        with patch.object(self.ui_builder._scenario, "reset") as mock_reset:
            self.ui_builder._reset_btn.trigger_click()
            ok = await wait_until(lambda: mock_reset.call_count >= 1, timeout_sec=30.0)
            self.assertTrue(ok, "Timed out waiting for scenario reset after RESET")

    async def test_run_cspace_button_calls_setup(self) -> None:
        """Test that the C-space trajectory button calls setup."""
        await self._load_until_articulation()
        with patch.object(self.ui_builder._scenario, "setup_cspace_trajectory") as mock_setup:
            self.ui_builder._cspace_trajectory_btn.trigger_click_if_a_state()
            mock_setup.assert_called_once()

    async def test_run_taskspace_button_calls_setup(self) -> None:
        """Test that the task-space trajectory button calls setup."""
        await self._load_until_articulation()
        with patch.object(self.ui_builder._scenario, "setup_taskspace_trajectory") as mock_setup:
            self.ui_builder._taskspace_trajectory_btn.trigger_click_if_a_state()
            mock_setup.assert_called_once()

    async def test_run_hybrid_button_calls_setup(self) -> None:
        """Test that the hybrid trajectory button calls setup."""
        await self._load_until_articulation()
        with patch.object(self.ui_builder._scenario, "setup_hybrid_trajectory") as mock_setup:
            self.ui_builder._hybrid_trajectory_btn.trigger_click_if_a_state()
            mock_setup.assert_called_once()

    async def test_stop_button_pauses_timeline(self) -> None:
        """STOP (state B click) on a trajectory button should pause the timeline."""
        await self._load_until_articulation()
        with patch.object(self.ui_builder._scenario, "setup_cspace_trajectory"):
            self.ui_builder._cspace_trajectory_btn.trigger_click_if_a_state()
            await omni.kit.app.get_app().next_update_async()
            self.assertTrue(app_utils.is_playing())
            self.assertFalse(self.ui_builder._cspace_trajectory_btn.is_in_a_state())

            self.ui_builder._cspace_trajectory_btn.trigger_click_if_b_state()
            ok_pause = await wait_until(lambda: not app_utils.is_playing(), timeout_sec=10.0)
            self.assertTrue(ok_pause, "Timed out waiting for timeline to pause")
