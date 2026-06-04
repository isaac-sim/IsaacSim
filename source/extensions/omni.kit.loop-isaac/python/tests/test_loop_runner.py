# SPDX-FileCopyrightText: Copyright (c) 2021-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

"""Test loop runner functionality."""

from typing import Any

import carb
import omni.kit.test


class TestLoopRunner(omni.kit.test.AsyncTestCase):
    """Test cases for the Isaac run loop runner functionality."""

    async def test_manual_mode(self) -> None:
        """Test enabling and disabling manual stepping mode and verifying dt behavior."""
        from omni.kit.loop import _loop as omni_loop

        _loop_runner = omni_loop.acquire_loop_interface()
        # Manual mode is enabled via carb setting in test settings
        self.assertTrue(_loop_runner.get_manual_mode())

        _loop_runner.set_manual_step_size(1.0 / 30.0)
        # There should be no delay between setting the manual mode and getting the manual mode
        _loop_runner.set_manual_mode(True)
        self.assertTrue(_loop_runner.get_manual_mode())
        _loop_runner.set_manual_mode(False)
        self.assertFalse(_loop_runner.get_manual_mode())

        carb.settings.get_settings().set_bool("/app/runLoops/main/manualModeEnabled", True)
        await omni.kit.app.get_app().next_update_async()
        self.assertTrue(_loop_runner.get_manual_mode())

        # Test that the manual mode is working
        current_dt = 0

        def _render_callback(event: Any) -> None:
            nonlocal current_dt
            current_dt = event.payload["dt"]
            print("render callback", event.payload["dt"])

        subscription = carb.eventdispatcher.get_eventdispatcher().observe_event(
            event_name=omni.kit.app.GLOBAL_EVENT_UPDATE,
            on_event=_render_callback,
            observer_name="loop_runner_test",
        )

        # Test that the manual mode is working
        await omni.kit.app.get_app().next_update_async()
        self.assertEqual(current_dt, 1.0 / 30.0)
        _loop_runner.set_manual_mode(False)
        await omni.kit.app.get_app().next_update_async()
        self.assertLess(current_dt, 1.0 / 30.0)
        # Using carb setting requires two frames to take effect
        carb.settings.get_settings().set_bool("/app/runLoops/main/manualModeEnabled", True)
        await omni.kit.app.get_app().next_update_async()
        await omni.kit.app.get_app().next_update_async()
        self.assertEqual(current_dt, 1.0 / 30.0)

        carb.settings.get_settings().set_bool("/app/runLoops/main/manualModeEnabled", False)
        await omni.kit.app.get_app().next_update_async()
        await omni.kit.app.get_app().next_update_async()
        self.assertLess(current_dt, 1.0 / 30.0)

        subscription = None
