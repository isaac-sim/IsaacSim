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

"""Unit tests for :class:`VRRecordingButton` rising-edge detection and event dispatch.

The tests do not require a live VR session — they exercise the button by driving a
:class:`_FakeTeleopManager` that implements only the observer-registration API, plus a
:class:`_FakeController` with a mutable ``inputs`` dataclass matching the shape of a real controller
snapshot.
"""

from dataclasses import dataclass

import carb.eventdispatcher
import omni.kit.app
import omni.kit.test
import omni.usd
from isaacsim.replicator.episode_recorder import EPISODE_BINDING_EVENT
from isaacsim.replicator.teleop import VRButton, VRRecordingButton
from isaacsim.replicator.teleop.vr_recording_button import EPISODE_CMD_EVENT


@dataclass
class _FakeInputs:
    """Mirrors the subset of controller input fields read by :class:`VRRecordingButton`."""

    primary_click: bool = False
    secondary_click: bool = False
    thumbstick_click: bool = False


@dataclass
class _FakeController:
    inputs: _FakeInputs


class _FakeTeleopManager:
    """Minimal stand-in for :class:`TeleopManager` providing the observer registration API."""

    def __init__(self) -> None:
        self._observers: list = []

    def add_controller_inputs_observer(self, observer: object) -> None:
        self._observers.append(observer)

    def remove_controller_inputs_observer(self, observer: object) -> None:
        try:
            self._observers.remove(observer)
        except ValueError:
            pass

    def tick(self, left: object, right: object) -> None:
        """Drive one frame of the observer chain with the provided controller snapshots.

        Args:
            left: Value for left.
            right: Value for right.
        """
        for obs in list(self._observers):
            obs(left, right)


class TestVRRecordingButton(omni.kit.test.AsyncTestCase):
    """Rising-edge, event-dispatch, and detach semantics for :class:`VRRecordingButton`."""

    async def setUp(self) -> None:
        """Set up the test fixture."""
        await omni.kit.app.get_app().next_update_async()
        omni.usd.get_context().new_stage()
        await omni.kit.app.get_app().next_update_async()

    async def tearDown(self) -> None:
        """Tear down the test fixture."""
        omni.usd.get_context().close_stage()
        await omni.kit.app.get_app().next_update_async()
        while omni.usd.get_context().get_stage_loading_status()[2] > 0:
            await omni.kit.app.get_app().next_update_async()

    async def test_rising_edge_dispatches_toggle_once_per_press(self) -> None:
        """Holding the button across frames only dispatches once; release + press dispatches again."""
        received_events: list[dict] = []

        def capture_event(event: object) -> None:
            received_events.append(dict(event.payload) if event.payload is not None else {})

        dispatcher = carb.eventdispatcher.get_eventdispatcher()
        event_sub = dispatcher.observe_event(
            event_name=EPISODE_CMD_EVENT,
            on_event=capture_event,
            observer_name="TestVRRecordingButton.rising_edge",
        )
        try:
            await omni.kit.app.get_app().next_update_async()
            teleop = _FakeTeleopManager()
            button = VRRecordingButton(teleop, button=VRButton.LEFT_SECONDARY, command="toggle")
            button.attach()
            self.assertTrue(button.is_attached)

            left = _FakeController(inputs=_FakeInputs())
            right = _FakeController(inputs=_FakeInputs())

            teleop.tick(left, right)
            await omni.kit.app.get_app().next_update_async()
            self.assertEqual(len(received_events), 0)

            left.inputs.secondary_click = True
            teleop.tick(left, right)
            await omni.kit.app.get_app().next_update_async()
            self.assertEqual(len(received_events), 1)
            self.assertEqual(received_events[-1].get("command"), "toggle")

            teleop.tick(left, right)
            teleop.tick(left, right)
            await omni.kit.app.get_app().next_update_async()
            self.assertEqual(len(received_events), 1, "Held button must not re-dispatch.")

            left.inputs.secondary_click = False
            teleop.tick(left, right)
            left.inputs.secondary_click = True
            teleop.tick(left, right)
            await omni.kit.app.get_app().next_update_async()
            self.assertEqual(len(received_events), 2)

            button.destroy()
        finally:
            event_sub = None

    async def test_right_primary_does_not_trigger_left_button(self) -> None:
        """A right-controller button press must not fire a left-bound :class:`VRRecordingButton`."""
        received_events: list[dict] = []

        def capture_event(event: object) -> None:
            received_events.append(dict(event.payload) if event.payload is not None else {})

        dispatcher = carb.eventdispatcher.get_eventdispatcher()
        event_sub = dispatcher.observe_event(
            event_name=EPISODE_CMD_EVENT,
            on_event=capture_event,
            observer_name="TestVRRecordingButton.right_primary",
        )
        try:
            await omni.kit.app.get_app().next_update_async()
            teleop = _FakeTeleopManager()
            button = VRRecordingButton(teleop, button=VRButton.LEFT_SECONDARY, command="toggle")
            button.attach()

            left = _FakeController(inputs=_FakeInputs())
            right = _FakeController(inputs=_FakeInputs())

            right.inputs.secondary_click = True
            teleop.tick(left, right)
            right.inputs.secondary_click = False
            teleop.tick(left, right)
            await omni.kit.app.get_app().next_update_async()
            self.assertEqual(len(received_events), 0)

            button.destroy()
        finally:
            event_sub = None

    async def test_detach_stops_dispatch(self) -> None:
        """After :meth:`detach`, further button presses do not dispatch events."""
        received_events: list[dict] = []

        def capture_event(event: object) -> None:
            received_events.append(dict(event.payload) if event.payload is not None else {})

        dispatcher = carb.eventdispatcher.get_eventdispatcher()
        event_sub = dispatcher.observe_event(
            event_name=EPISODE_CMD_EVENT,
            on_event=capture_event,
            observer_name="TestVRRecordingButton.detach",
        )
        try:
            await omni.kit.app.get_app().next_update_async()
            teleop = _FakeTeleopManager()
            button = VRRecordingButton(teleop, button=VRButton.LEFT_SECONDARY)
            button.attach()

            left = _FakeController(inputs=_FakeInputs(secondary_click=True))
            right = _FakeController(inputs=_FakeInputs())
            teleop.tick(left, right)
            await omni.kit.app.get_app().next_update_async()
            self.assertEqual(len(received_events), 1)

            button.detach()
            self.assertFalse(button.is_attached)

            left.inputs.secondary_click = False
            teleop.tick(left, right)
            left.inputs.secondary_click = True
            teleop.tick(left, right)
            await omni.kit.app.get_app().next_update_async()
            self.assertEqual(len(received_events), 1)
        finally:
            event_sub = None

    async def test_custom_command_and_payload(self) -> None:
        """Custom ``command`` and ``command_payload`` are forwarded verbatim in the event payload."""
        received_events: list[dict] = []

        def capture_event(event: object) -> None:
            received_events.append(dict(event.payload) if event.payload is not None else {})

        dispatcher = carb.eventdispatcher.get_eventdispatcher()
        event_sub = dispatcher.observe_event(
            event_name=EPISODE_CMD_EVENT,
            on_event=capture_event,
            observer_name="TestVRRecordingButton.custom_payload",
        )
        try:
            await omni.kit.app.get_app().next_update_async()
            teleop = _FakeTeleopManager()
            button = VRRecordingButton(
                teleop,
                button=VRButton.RIGHT_PRIMARY,
                command="start",
                command_payload={"metadata": {"source": "right_a"}},
            )
            button.attach()

            left = _FakeController(inputs=_FakeInputs())
            right = _FakeController(inputs=_FakeInputs())
            right.inputs.primary_click = True
            teleop.tick(left, right)
            await omni.kit.app.get_app().next_update_async()

            self.assertEqual(len(received_events), 1)
            payload = received_events[-1]
            self.assertEqual(payload.get("command"), "start")
            self.assertEqual(payload.get("metadata"), {"source": "right_a"})

            button.destroy()
        finally:
            event_sub = None

    async def test_session_id_getter_none_suppresses_dispatch(self) -> None:
        """A session-scoped button should stay quiet until a recorder session is available."""
        received_events: list[dict] = []

        def capture_event(event: object) -> None:
            received_events.append(dict(event.payload) if event.payload is not None else {})

        dispatcher = carb.eventdispatcher.get_eventdispatcher()
        event_sub = dispatcher.observe_event(
            event_name=EPISODE_CMD_EVENT,
            on_event=capture_event,
            observer_name="TestVRRecordingButton.no_session",
        )
        try:
            await omni.kit.app.get_app().next_update_async()
            teleop = _FakeTeleopManager()
            button = VRRecordingButton(
                teleop,
                button=VRButton.LEFT_SECONDARY,
                session_id_getter=lambda: None,
            )
            button.attach()

            left = _FakeController(inputs=_FakeInputs(secondary_click=True))
            right = _FakeController(inputs=_FakeInputs())
            teleop.tick(left, right)
            await omni.kit.app.get_app().next_update_async()

            self.assertEqual(received_events, [])
            button.destroy()
        finally:
            event_sub = None

    async def test_attach_detach_dispatches_binding_lifecycle_events(self) -> None:
        """Attach must broadcast an ``attach`` binding event; detach must broadcast ``detach``."""
        received_events: list[dict] = []

        def capture_event(event: object) -> None:
            received_events.append(dict(event.payload) if event.payload is not None else {})

        dispatcher = carb.eventdispatcher.get_eventdispatcher()
        event_sub = dispatcher.observe_event(
            event_name=EPISODE_BINDING_EVENT,
            on_event=capture_event,
            observer_name="TestVRRecordingButton.binding_lifecycle",
        )
        try:
            await omni.kit.app.get_app().next_update_async()
            teleop = _FakeTeleopManager()
            button = VRRecordingButton(teleop, button=VRButton.LEFT_SECONDARY, command="toggle")

            self.assertEqual(received_events, [], "No binding event should fire before attach()")

            button.attach()
            await omni.kit.app.get_app().next_update_async()
            self.assertEqual(len(received_events), 1, "Attach must dispatch one binding event")
            attach_payload = received_events[-1]
            self.assertEqual(attach_payload.get("action"), "attach")
            self.assertEqual(attach_payload.get("source"), "vr_button")
            self.assertEqual(attach_payload.get("binding_id"), "vr_left_secondary")
            self.assertEqual(attach_payload.get("command"), "toggle")
            self.assertIn("Left Secondary", attach_payload.get("label") or "")

            button.detach()
            await omni.kit.app.get_app().next_update_async()
            self.assertEqual(len(received_events), 2, "Detach must dispatch a follow-up binding event")
            detach_payload = received_events[-1]
            self.assertEqual(detach_payload.get("action"), "detach")
            self.assertEqual(detach_payload.get("binding_id"), "vr_left_secondary")
        finally:
            event_sub = None
