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

"""VR controller binding that toggles episode recording on a button press.

This module provides a thin glue layer between :class:`TeleopManager` and the recorder command bus
in ``isaacsim.replicator.episode_recorder``. It detects a rising edge on a configurable VR controller
button and dispatches an ``EPISODE_CMD_EVENT`` command on the Kit event bus. Callers can optionally
scope events to the active recorder ``session_id``.

Typical usage:

.. code-block:: python

    from isaacsim.replicator.teleop import TeleopManager, VRRecordingButton, VRButton

    teleop = TeleopManager()
    record_button = VRRecordingButton(teleop, button=VRButton.LEFT_SECONDARY)  # Meta Quest "Y"
    record_button.attach()
    # ... on Y-button press the recorder will toggle.
    record_button.destroy()
"""

from __future__ import annotations

from collections.abc import Callable
from dataclasses import dataclass
from enum import Enum
from typing import Any

import carb
import carb.eventdispatcher
from isaacsim.replicator.episode_recorder import EPISODE_CMD_EVENT, dispatch_episode_binding


class VRButton(str, Enum):
    """VR controller buttons exposed by ``TeleopManager`` controller snapshots.

    The underlying ``inputs`` fields on each snapshot are:

        * ``primary_click``   — typically ``A``/``X`` on Meta Quest.
        * ``secondary_click`` — typically ``B``/``Y`` on Meta Quest.
        * ``thumbstick_click`` — thumbstick press.

    For the standard Meta Quest mapping, the left-hand "Y" button is ``LEFT_SECONDARY``.
    """

    LEFT_PRIMARY = "left_primary"
    LEFT_SECONDARY = "left_secondary"
    LEFT_THUMBSTICK = "left_thumbstick"
    RIGHT_PRIMARY = "right_primary"
    RIGHT_SECONDARY = "right_secondary"
    RIGHT_THUMBSTICK = "right_thumbstick"


_BUTTON_FIELD = {
    VRButton.LEFT_PRIMARY: ("left", "primary_click"),
    VRButton.LEFT_SECONDARY: ("left", "secondary_click"),
    VRButton.LEFT_THUMBSTICK: ("left", "thumbstick_click"),
    VRButton.RIGHT_PRIMARY: ("right", "primary_click"),
    VRButton.RIGHT_SECONDARY: ("right", "secondary_click"),
    VRButton.RIGHT_THUMBSTICK: ("right", "thumbstick_click"),
}


@dataclass
class _ButtonSpec:
    """Resolved mapping from a :class:`VRButton` enum to the snapshot field to read."""

    side: str
    field: str


class VRRecordingButton:
    """Binds a VR controller button press to an :class:`EpisodeRecorder` ``toggle`` event.

    The button detects a rising edge (``False`` → ``True``) on the configured controller button and
    dispatches an ``EPISODE_CMD_EVENT`` with the ``toggle`` command. If no recorder has an open
    session at the time of dispatch, the event is silently ignored.

    The binding subscribes to :meth:`TeleopManager.add_controller_inputs_observer` on
    :meth:`attach` and unsubscribes on :meth:`detach` (or :meth:`destroy`). It does not hold physics
    or USD state, so it is safe to construct early and attach/detach repeatedly.

    Args:
        teleop_manager: The :class:`TeleopManager` whose per-frame controller snapshots will drive
            the button edge detection. Must live for the lifetime of this binding.
        button: Which VR controller button to observe. Defaults to ``LEFT_SECONDARY`` (Meta Quest
            "Y"), matching the "record-from-left-Y" convention documented in the teleop workflow.
        command: Which :class:`EpisodeRecorder` command to dispatch on a rising edge. Defaults to
            ``"toggle"``, which starts a new episode if none is active and ends the active one
            otherwise. Use ``"start"``/``"end"`` for two-button start/stop wiring.
        command_payload: Extra payload fields forwarded with every dispatched event (e.g.
            ``{"metadata": {"trigger": "left_y"}}``). Useful for tagging episodes with the input
            source without adding per-button code to the recorder.
        session_id_getter: Optional callable returning the recorder ``session_id`` that should
            receive button events. When provided and it returns ``None``, the button press is
            ignored until a session is available.

    Example — bimanual start/stop on two separate buttons:

    .. code-block:: python

        start = VRRecordingButton(teleop, button=VRButton.LEFT_SECONDARY, command="start")
        stop = VRRecordingButton(teleop, button=VRButton.RIGHT_SECONDARY, command="end")
        start.attach()
        stop.attach()
    """

    def __init__(
        self,
        teleop_manager: Any,
        *,
        button: VRButton = VRButton.LEFT_SECONDARY,
        command: str = "toggle",
        command_payload: dict[str, Any] | None = None,
        session_id_getter: Callable[[], str | None] | None = None,
    ) -> None:
        if button not in _BUTTON_FIELD:
            raise ValueError(f"Unsupported VRButton: {button!r}")
        side, field = _BUTTON_FIELD[button]
        self._teleop_manager = teleop_manager
        self._button = button
        self._spec = _ButtonSpec(side=side, field=field)
        self._command = command
        self._command_payload = dict(command_payload or {})
        self._session_id_getter = session_id_getter
        self._attached = False
        self._prev_pressed = False

    @property
    def is_attached(self) -> bool:
        """Whether this binding is currently subscribed to the teleop manager."""
        return self._attached

    def attach(self) -> None:
        """Subscribe to the teleop manager's controller-input stream. Safe to call multiple times."""
        if self._attached:
            return
        if not hasattr(self._teleop_manager, "add_controller_inputs_observer"):
            raise RuntimeError(
                "TeleopManager does not expose add_controller_inputs_observer; "
                "VRRecordingButton requires a teleop build with the controller-observer API."
            )
        self._teleop_manager.add_controller_inputs_observer(self._on_controller_inputs)
        self._attached = True
        self._prev_pressed = False
        self._dispatch_binding("attach")

    def detach(self) -> None:
        """Unsubscribe from the teleop manager. Safe to call multiple times."""
        if not self._attached:
            return
        remove = getattr(self._teleop_manager, "remove_controller_inputs_observer", None)
        if remove is not None:
            remove(self._on_controller_inputs)
        self._attached = False
        self._prev_pressed = False
        self._dispatch_binding("detach")

    def _dispatch_binding(self, action: str) -> None:
        """Advertise this binding's lifecycle on the recorder binding event bus."""
        try:
            session_id = self._session_id_getter() if self._session_id_getter is not None else None
            dispatch_episode_binding(
                action,
                binding_id=f"vr_{self._button.value}",
                source="vr_button",
                label=f"VR {self._button.value.replace('_', ' ').title()}",
                command=self._command,
                session_id=session_id,
            )
        except Exception as exc:  # noqa: BLE001
            carb.log_warn(f"[VRRecordingButton] Failed to dispatch binding {action} event: {exc}")

    def destroy(self) -> None:
        """Alias for :meth:`detach`. Safe to call during shutdown."""
        self.detach()

    def _read_button(self, left_ctrl, right_ctrl) -> bool:
        """Return the boolean state of the bound button on the current snapshot."""
        snapshot = left_ctrl if self._spec.side == "left" else right_ctrl
        if snapshot is None:
            return False
        inputs = getattr(snapshot, "inputs", None)
        if inputs is None:
            return False
        value = getattr(inputs, self._spec.field, False)
        return bool(value)

    def _on_controller_inputs(self, left_ctrl, right_ctrl) -> None:
        """Edge-detect the bound button and dispatch the configured command on rising edges."""
        pressed = self._read_button(left_ctrl, right_ctrl)
        if pressed and not self._prev_pressed:
            self._dispatch_command()
        self._prev_pressed = pressed

    def _dispatch_command(self) -> None:
        """Dispatch ``EPISODE_CMD_EVENT`` with the configured command and payload."""
        session_id = self._session_id_getter() if self._session_id_getter is not None else None
        if self._session_id_getter is not None and session_id is None:
            carb.log_info(f"[VRRecordingButton] Ignored '{self._command}' from {self._button.value}: no session.")
            return

        payload = {"command": self._command, "session_id": session_id, **self._command_payload}
        carb.eventdispatcher.get_eventdispatcher().dispatch_event(
            event_name=EPISODE_CMD_EVENT,
            payload=payload,
        )
        carb.log_info(
            f"[VRRecordingButton] Dispatched '{self._command}' from {self._button.value} (session_id={session_id})."
        )
