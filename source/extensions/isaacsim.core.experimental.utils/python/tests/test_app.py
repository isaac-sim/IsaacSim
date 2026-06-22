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

"""Verifies app utility helpers for update ticks, extension lookup, extension enablement, and timeline access. Covers both synchronous and asynchronous app update paths."""

import os
import unittest
from typing import Any

import isaacsim.core.experimental.utils.app as app_utils
import omni.kit.test
import omni.timeline

EXTENSION_NAME = "omni.pip.cloud"


class TestApp(omni.kit.test.AsyncTestCase):
    """Test app."""

    async def setUp(self) -> None:
        """Method called to prepare the test fixture."""
        super().setUp()

    async def tearDown(self) -> None:
        """Method called immediately after the test method has been called."""
        super().tearDown()

    # --------------------------------------------------------------------
    @unittest.skip("Skipping update_app tests in async environment")
    async def test_update_app(self) -> None:
        """Test update app."""
        # perform 1 update step
        app_utils.update_app()
        # perform 5 update steps
        app_utils.update_app(steps=5)
        # perform 5 update steps
        counter = [0]
        app_utils.update_app(steps=5, callback=lambda step, steps: counter.__setitem__(0, counter[0] + 1))
        self.assertEqual(counter[0], 5)
        # perform 5 update steps but break after 2 steps
        counter = [0]
        app_utils.update_app(
            steps=5, callback=lambda step, steps: counter.__setitem__(0, counter[0] + 1) if step <= 2 else False
        )
        self.assertEqual(counter[0], 2)

    async def test_update_app_async(self) -> None:
        """Test update app async."""
        # perform 1 update step
        await app_utils.update_app_async()
        # perform 7 update steps
        await app_utils.update_app_async(steps=7)
        # perform 7 update steps
        counter = [0]
        await app_utils.update_app_async(steps=7, callback=lambda step, steps: counter.__setitem__(0, counter[0] + 1))
        self.assertEqual(counter[0], 7)
        # perform 7 update steps but break after 5 steps
        counter = [0]
        await app_utils.update_app_async(
            steps=7, callback=lambda step, steps: counter.__setitem__(0, counter[0] + 1) if step <= 5 else False
        )
        self.assertEqual(counter[0], 5)

    async def test_enable_extension(self) -> None:
        """Test enable extension."""
        for _ in range(3):
            # - enable extension
            self.assertTrue(app_utils.enable_extension(EXTENSION_NAME))
            self.assertTrue(app_utils.is_extension_enabled(EXTENSION_NAME), f"{EXTENSION_NAME} should be enabled")
            self.assertTrue(app_utils.enable_extension(EXTENSION_NAME))
            self.assertTrue(app_utils.is_extension_enabled(EXTENSION_NAME), f"{EXTENSION_NAME} should be enabled")
            # - disable extension
            self.assertTrue(app_utils.enable_extension(EXTENSION_NAME, enabled=False))
            self.assertFalse(app_utils.is_extension_enabled(EXTENSION_NAME), f"{EXTENSION_NAME} should be disabled")
            self.assertTrue(app_utils.enable_extension(EXTENSION_NAME, enabled=False))
            self.assertFalse(app_utils.is_extension_enabled(EXTENSION_NAME), f"{EXTENSION_NAME} should be disabled")

    async def test_extension_id(self) -> None:
        """Test extension id."""

        def _check_id(uid: Any) -> None:
            self.assertEqual(len(uid.split("-")), 2)
            self.assertEqual(uid.split("-")[0], EXTENSION_NAME)
            self.assertTrue(uid.split("-")[1] != "")

        self.assertTrue(app_utils.enable_extension(EXTENSION_NAME))  # ensure extension is enabled
        # test cases
        # - get from name
        uid = app_utils.get_extension_id(EXTENSION_NAME)
        version = "-".join(uid.split("-")[1:])
        _check_id(uid)
        # - get from id
        _check_id(app_utils.get_extension_id(uid))
        # - get from clipped id (clip last 4 version characters)
        _check_id(app_utils.get_extension_id(f"{EXTENSION_NAME}-{version[:-4]}"))
        # - get from wrong name
        self.assertIsNone(app_utils.get_extension_id(f"{EXTENSION_NAME}.wrong.name"))
        # - get from wrong version
        self.assertIsNone(app_utils.get_extension_id(f"{EXTENSION_NAME}-0.0.1"))
        # - disable extension and check return value
        self.assertTrue(app_utils.enable_extension(EXTENSION_NAME, enabled=False))
        self.assertIsNone(app_utils.get_extension_id(EXTENSION_NAME))

    async def test_extension_path(self) -> None:
        """Test extension path."""
        self.assertTrue(app_utils.enable_extension(EXTENSION_NAME))  # ensure extension is enabled
        # test cases
        # - get from name
        path = app_utils.get_extension_path(EXTENSION_NAME)
        self.assertTrue(os.path.exists(path), f"{path} should exist")
        # - get from id
        path = app_utils.get_extension_path(app_utils.get_extension_id(EXTENSION_NAME))
        self.assertTrue(os.path.exists(path), f"{path} should exist")
        # - disable extension and check return value
        self.assertTrue(app_utils.enable_extension(EXTENSION_NAME, enabled=False))
        path = app_utils.get_extension_path(EXTENSION_NAME)
        self.assertEqual(path, "")

    async def test_extension_dict(self) -> None:
        """Test extension dict."""

        def _check_dict(data: Any) -> None:
            self.assertTrue(isinstance(data, dict), f"{data} should be a dictionary")
            self.assertIn("path", data)
            self.assertIn("package", data)
            self.assertIn("name", data["package"])
            self.assertIn("version", data["package"])

        self.assertTrue(app_utils.enable_extension(EXTENSION_NAME))  # ensure extension is enabled
        # test cases
        # - get from name
        _check_dict(app_utils.get_extension_dict(EXTENSION_NAME))
        # - get from id
        _check_dict(app_utils.get_extension_dict(app_utils.get_extension_id(EXTENSION_NAME)))
        # - get from wrong name
        self.assertIsNone(app_utils.get_extension_dict(f"{EXTENSION_NAME}.wrong.name"))
        # - disable extension and check return value
        self.assertTrue(app_utils.enable_extension(EXTENSION_NAME, enabled=False))
        self.assertIsNone(app_utils.get_extension_dict(EXTENSION_NAME))

    async def test_timeline(self) -> None:
        """Test timeline."""

        def check_timeline_state(*, is_playing: Any, is_paused: Any, is_stopped: Any) -> None:
            self.assertEqual(app_utils.is_playing(), is_playing, "Timeline should be 'playing'")
            self.assertEqual(app_utils.is_paused(), is_paused, "Timeline should be 'paused'")
            self.assertEqual(app_utils.is_stopped(), is_stopped, "Timeline should be 'stopped'")

        def _on_play(event: Any) -> None:
            events[0] += 1

        def _on_pause(event: Any) -> None:
            events[1] += 1

        def _on_stop(event: Any) -> None:
            events[2] += 1

        timeline_event_stream = omni.timeline.get_timeline_interface().get_timeline_event_stream()
        subscriptions = [
            timeline_event_stream.create_subscription_to_pop_by_type(omni.timeline.TimelineEventType.PLAY, _on_play),
            timeline_event_stream.create_subscription_to_pop_by_type(omni.timeline.TimelineEventType.PAUSE, _on_pause),
            timeline_event_stream.create_subscription_to_pop_by_type(omni.timeline.TimelineEventType.STOP, _on_stop),
        ]

        # setup timeline
        omni.timeline.get_timeline_interface().set_end_time(1.0)
        omni.timeline.get_timeline_interface().set_time_codes_per_second(60.0)
        # test cases
        # - commit state
        commit = True
        events = [0, 0, 0]
        check_timeline_state(is_playing=False, is_paused=False, is_stopped=True)
        # -- play
        app_utils.play(commit=commit)
        self.assertListEqual(events, [1, 0, 0])
        check_timeline_state(is_playing=True, is_paused=False, is_stopped=False)
        await app_utils.update_app_async()
        self.assertListEqual(events, [1, 0, 0])
        # -- pause
        app_utils.pause(commit=commit)
        self.assertListEqual(events, [1, 1, 0])
        check_timeline_state(is_playing=False, is_paused=True, is_stopped=False)
        await app_utils.update_app_async()
        self.assertListEqual(events, [1, 1, 0])
        # -- stop
        app_utils.stop(commit=commit)
        self.assertListEqual(events, [1, 1, 1])
        check_timeline_state(is_playing=False, is_paused=False, is_stopped=True)
        await app_utils.update_app_async()
        self.assertListEqual(events, [1, 1, 1])
        # - commit state (silently)
        commit = None
        events = [0, 0, 0]
        check_timeline_state(is_playing=False, is_paused=False, is_stopped=True)
        # -- play
        app_utils.play(commit=commit)
        self.assertListEqual(events, [0, 0, 0])
        check_timeline_state(is_playing=True, is_paused=False, is_stopped=False)
        await app_utils.update_app_async()
        self.assertListEqual(events, [0, 0, 0])
        # -- pause
        app_utils.pause(commit=commit)
        self.assertListEqual(events, [0, 0, 0])
        check_timeline_state(is_playing=False, is_paused=True, is_stopped=False)
        await app_utils.update_app_async()
        self.assertListEqual(events, [0, 0, 0])
        # -- stop
        app_utils.stop(commit=commit)
        self.assertListEqual(events, [0, 0, 0])
        check_timeline_state(is_playing=False, is_paused=False, is_stopped=True)
        await app_utils.update_app_async()
        self.assertListEqual(events, [0, 0, 0])
        # - don't commit state
        commit = False
        events = [0, 0, 0]
        check_timeline_state(is_playing=False, is_paused=False, is_stopped=True)
        # -- play
        app_utils.play(commit=commit)
        self.assertListEqual(events, [0, 0, 0])
        check_timeline_state(is_playing=False, is_paused=False, is_stopped=True)
        await app_utils.update_app_async()
        self.assertListEqual(events, [1, 0, 0])
        check_timeline_state(is_playing=True, is_paused=False, is_stopped=False)
        # -- pause
        app_utils.pause(commit=commit)
        self.assertListEqual(events, [1, 0, 0])
        check_timeline_state(is_playing=True, is_paused=False, is_stopped=False)
        await app_utils.update_app_async()
        self.assertListEqual(events, [1, 1, 0])
        check_timeline_state(is_playing=False, is_paused=True, is_stopped=False)
        # -- stop
        app_utils.stop(commit=commit)
        self.assertListEqual(events, [1, 1, 0])
        check_timeline_state(is_playing=False, is_paused=True, is_stopped=False)
        await app_utils.update_app_async()
        self.assertListEqual(events, [1, 1, 1])
        check_timeline_state(is_playing=False, is_paused=False, is_stopped=True)
