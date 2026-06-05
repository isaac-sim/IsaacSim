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

import carb
import omni.kit.test
from isaacsim.replicator.experimental.mobility_gen.impl.inputs import KeyboardButton


class _FakeKeyboardEvent:
    # Lightweight stand-in for `carb.input.KeyboardEvent`. The real type cannot be
    # constructed from Python, but `KeyboardButton._event_callback` only reads
    # `.input` and `.type`, so duck typing is sufficient here.
    def __init__(self, input_key: carb.input.KeyboardInput, event_type: carb.input.KeyboardEventType) -> None:
        self.input = input_key
        self.type = event_type


# Regression tests: After driving with WASD, clicking a UI text field showed
# buffered keystrokes because `_event_callback` did not return True for events
# it consumed, so Kit kept propagating them to UI widgets.
class TestKeyboardButton(omni.kit.test.AsyncTestCase):
    async def test_event_callback_consumes_press_for_tracked_key(self):
        button = KeyboardButton(carb.input.KeyboardInput.W)
        event = _FakeKeyboardEvent(carb.input.KeyboardInput.W, carb.input.KeyboardEventType.KEY_PRESS)
        self.assertTrue(button._event_callback(event))
        self.assertTrue(button.value)

    async def test_event_callback_consumes_repeat_for_tracked_key(self):
        button = KeyboardButton(carb.input.KeyboardInput.A)
        event = _FakeKeyboardEvent(carb.input.KeyboardInput.A, carb.input.KeyboardEventType.KEY_REPEAT)
        self.assertTrue(button._event_callback(event))
        self.assertTrue(button.value)

    async def test_event_callback_consumes_release_for_tracked_key(self):
        button = KeyboardButton(carb.input.KeyboardInput.S)
        # Pre-set the button so we can confirm release flips it back to False.
        button._event_callback(_FakeKeyboardEvent(carb.input.KeyboardInput.S, carb.input.KeyboardEventType.KEY_PRESS))
        event = _FakeKeyboardEvent(carb.input.KeyboardInput.S, carb.input.KeyboardEventType.KEY_RELEASE)
        self.assertTrue(button._event_callback(event))
        self.assertFalse(button.value)

    async def test_event_callback_does_not_consume_other_keys(self):
        # A WASD button must let other keys (e.g. Q) fall through so UI widgets
        # can still receive them. Returning True here is the bug.
        button = KeyboardButton(carb.input.KeyboardInput.W)
        event = _FakeKeyboardEvent(carb.input.KeyboardInput.Q, carb.input.KeyboardEventType.KEY_PRESS)
        self.assertFalse(button._event_callback(event))
        self.assertFalse(button.value)

    async def test_event_callback_does_not_consume_unhandled_event_type(self):
        # `CHAR` events are produced for text input; we should never consume those
        # or text fields will be starved.
        button = KeyboardButton(carb.input.KeyboardInput.D)
        event = _FakeKeyboardEvent(carb.input.KeyboardInput.D, carb.input.KeyboardEventType.CHAR)
        self.assertFalse(button._event_callback(event))
        self.assertFalse(button.value)
