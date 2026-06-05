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

"""Tests for the RMPflow example GUI."""

from unittest.mock import patch

import omni.kit.app
import omni.kit.test
from isaacsim.core.experimental.utils import stage as stage_utils
from isaacsim.robot_motion.cumotion.examples.rmp_flow.scenario import FrankaRmpFlowExample
from isaacsim.robot_motion.cumotion.examples.rmp_flow.ui_builder import UIBuilder

from .gui_test_support import (
    TEST_LOAD_TIMEOUT_SEC,
    WARMUP_LOAD_TIMEOUT_SEC,
    assert_xyz_and_unit_quaternion_wxyz,
    ensure_gui_class_warmup_once,
    wait_until,
)

_ROBOT_PATH = "/panda"
_TARGET_PATH = "/World/target"
_OBSTACLE_PATH = "/World/obstacle"
_PHYSICS_SCENE_PATH = "/World/PhysicsScene"


def _scenario_loaded(ui_builder: UIBuilder) -> bool:
    s = ui_builder._scenario
    return s._target is not None and s._articulation is not None and s._controller is not None


class TestRmpFlowGui(omni.kit.test.AsyncTestCase):
    """Test suite for the RmpFlow GUI."""

    async def setUp(self):
        """Set up the UI builder before each test."""
        await omni.kit.app.get_app().next_update_async()
        await ensure_gui_class_warmup_once(
            type(self),
            ui_builder_cls=UIBuilder,
            wait_for_load=lambda wb: self._load_until_scenario_ready_on(wb, timeout_sec=WARMUP_LOAD_TIMEOUT_SEC),
        )
        self.ui_builder = UIBuilder()
        self.ui_builder.build_ui()
        await omni.kit.app.get_app().next_update_async()

    async def tearDown(self):
        """Clean up the UI builder after each test."""
        self.ui_builder.cleanup()
        await omni.kit.app.get_app().next_update_async()
        await omni.kit.app.get_app().next_update_async()

    @classmethod
    async def _load_until_scenario_ready_on(cls, ui_builder: UIBuilder, *, timeout_sec: float) -> None:
        ui_builder._load_btn.trigger_click()
        ok = await wait_until(lambda: _scenario_loaded(ui_builder), timeout_sec=timeout_sec)
        if not ok:
            raise AssertionError("Timed out waiting for scenario load (target/articulation/controller)")

    async def test_gui_layout(self):
        """Checks that we have the correct buttons available."""
        self.assertIsNotNone(self.ui_builder._load_btn)
        self.assertIsNotNone(self.ui_builder._reset_btn)
        self.assertIsNotNone(self.ui_builder._scenario_state_btn)

        # The scenario state button should be disabled initially
        self.assertTrue(self.ui_builder._scenario_state_btn.is_in_a_state())

    async def test_load_creates_all_expected_assets(self):
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
        self.assertIsNotNone(s._target, "scenario._target should be set after LOAD")
        self.assertIsNotNone(s._controller, "scenario._controller should be set after LOAD")
        self.assertIsNotNone(s._world_binding, "scenario._world_binding should be set after LOAD")

        # All expected prims must be on the stage.
        stage = stage_utils.get_current_stage()
        self.assertIsNotNone(stage, "Stage should exist after LOAD")
        for path in (_ROBOT_PATH, _TARGET_PATH, _OBSTACLE_PATH, _PHYSICS_SCENE_PATH):
            self.assertTrue(stage.GetPrimAtPath(path).IsValid(), f"Expected prim {path!r} on stage after LOAD")

    async def test_load_button_creates_cube_and_articulation(self):
        """After calling load button, there should be a cube and an Articulation added to the stage."""
        # There is initially no controller:
        self.assertIsNone(self.ui_builder._scenario._controller)

        self.ui_builder._load_btn.trigger_click()
        await omni.kit.app.get_app().next_update_async()

        ok = await wait_until(lambda: _scenario_loaded(self.ui_builder), timeout_sec=TEST_LOAD_TIMEOUT_SEC)
        self.assertTrue(ok, "Timed out waiting for scenario load (target/articulation/controller)")

        self.assertIsNotNone(self.ui_builder._scenario._target)
        self.assertIsNotNone(self.ui_builder._scenario._articulation)
        self.assertIsNotNone(self.ui_builder._scenario._controller)

    async def test_physics_step_reads_valid_target_cube_pose(self):
        """During scenario update (physics), target cube world pose is XYZ + unit quaternion wxyz."""
        self.ui_builder._load_btn.trigger_click()
        await omni.kit.app.get_app().next_update_async()

        ok = await wait_until(lambda: _scenario_loaded(self.ui_builder), timeout_sec=TEST_LOAD_TIMEOUT_SEC)
        self.assertTrue(ok, "Timed out waiting for scenario load (target/articulation/controller)")

        _orig_update = FrankaRmpFlowExample.update
        verified: dict[str, bool] = {"done": False}

        def _update_check_target_pose(self, step: float) -> None:
            if self._controller is not None and self._target is not None:
                target_positions, target_orientations = self._target.get_world_poses()
                assert_xyz_and_unit_quaternion_wxyz(
                    target_positions.numpy()[0],
                    target_orientations.numpy()[0],
                )
                verified["done"] = True
            return _orig_update(self, step)

        with patch.object(FrankaRmpFlowExample, "update", _update_check_target_pose):
            self.ui_builder._scenario_state_btn.trigger_click_if_a_state()
            await omni.kit.app.get_app().next_update_async()

            saw = await wait_until(lambda: verified["done"], timeout_sec=60.0)
            self.assertTrue(saw, "Expected a physics step to run scenario.update with controller and target")

    async def test_button_clicks(self):
        """After calling reset button, the scenario should be reset."""
        # First, load the scenario
        self.ui_builder._load_btn.trigger_click()
        await omni.kit.app.get_app().next_update_async()

        ok = await wait_until(lambda: _scenario_loaded(self.ui_builder), timeout_sec=TEST_LOAD_TIMEOUT_SEC)
        self.assertTrue(ok, "Timed out waiting for scenario load (target/articulation/controller)")

        await omni.kit.app.get_app().next_update_async()

        # Then, start running the scenario:
        self.ui_builder._scenario_state_btn.trigger_click_if_a_state()
        await omni.kit.app.get_app().next_update_async()

        # The state button (run scenario button) should not be in state A anymore:
        self.assertFalse(self.ui_builder._scenario_state_btn.is_in_a_state())

        # The timeline should be playing:
        self.assertTrue(omni.timeline.get_timeline_interface().is_playing())

        self.ui_builder._reset_btn.trigger_click()
        for _ in range(20):
            await omni.kit.app.get_app().next_update_async()

        # The timeline should be paused:
        self.assertFalse(omni.timeline.get_timeline_interface().is_playing())

        # The state button (run scenario button) should be in state A again:
        self.assertTrue(self.ui_builder._scenario_state_btn.is_in_a_state())

        ####################################################################################
        # If we hit the state button twice, we should first start running, then be stopped:
        ####################################################################################
        self.ui_builder._scenario_state_btn.trigger_click_if_a_state()  # FIRST CLICK
        await omni.kit.app.get_app().next_update_async()

        # The timeline should be playing:
        self.assertTrue(omni.timeline.get_timeline_interface().is_playing())

        # The state button (run scenario button) should not be in state A anymore:
        self.assertFalse(self.ui_builder._scenario_state_btn.is_in_a_state())

        self.ui_builder._scenario_state_btn.trigger_click_if_b_state()  # SECOND CLICK
        await omni.kit.app.get_app().next_update_async()

        # The timeline should be paused:
        self.assertFalse(omni.timeline.get_timeline_interface().is_playing())

        # The state button (run scenario button) should be in state A again:
        self.assertTrue(self.ui_builder._scenario_state_btn.is_in_a_state())
