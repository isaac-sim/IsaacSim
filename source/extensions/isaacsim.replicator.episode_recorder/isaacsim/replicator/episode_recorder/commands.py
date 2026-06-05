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

"""Carb event bus used to drive :class:`EpisodeRecorder` lifecycle remotely.

Supports per-session filtering: recorders with a ``session_id`` only react to events
carrying a matching ``session_id`` (or no session_id, i.e. broadcast). This keeps
multi-recorder scenarios (multi-robot SDG, multiple teleop operators) isolated.
"""

from __future__ import annotations

from typing import Any

import carb
import carb.eventdispatcher

EPISODE_CMD_EVENT = "isaacsim.replicator.episode_recorder.command"
"""Carb event name used to drive recorder lifecycle externally."""

EPISODE_BINDING_EVENT = "isaacsim.replicator.episode_recorder.binding"
"""Carb event name used to advertise external bindings (e.g. VR controller buttons).

External agents that wire physical inputs to :data:`EPISODE_CMD_EVENT` can dispatch
this event with ``action="attach"`` / ``action="detach"`` so UI listeners (such as the
Episode Recorder window) can surface a "VR-bound" badge without taking a hard
dependency on the publisher's extension.
"""

VALID_COMMANDS = frozenset({"start", "end", "toggle", "pause", "resume", "open_session", "close_session"})

VALID_BINDING_ACTIONS = frozenset({"attach", "detach"})


def dispatch_episode_command(
    command: str,
    *,
    session_id: str | None = None,
    **payload: Any,
) -> None:
    """Dispatch an :data:`EPISODE_CMD_EVENT` on the Kit event bus.

    Args:
        command: One of ``start``, ``end``, ``toggle``, ``pause``, ``resume``,
            ``open_session``, ``close_session``. Unknown commands are still dispatched on
            the event bus; recorder listeners log a warning and ignore them.
        session_id: When set, only recorders registered with this id react. When
            ``None`` (default), all listening recorders react.
        **payload: Additional fields forwarded to the recorder (e.g. ``metadata=...``,
            ``success=...``).
    """
    full_payload = {"command": command, "session_id": session_id, **payload}
    carb.eventdispatcher.get_eventdispatcher().dispatch_event(
        event_name=EPISODE_CMD_EVENT,
        payload=full_payload,
    )


def dispatch_episode_binding(
    action: str,
    *,
    binding_id: str,
    source: str,
    label: str | None = None,
    command: str | None = None,
    session_id: str | None = None,
) -> None:
    """Advertise an external binding on :data:`EPISODE_BINDING_EVENT`.

    Use this from publishers (e.g. :class:`VRRecordingButton`) so listeners (e.g. the
    Episode Recorder window) can show a badge whenever an input is bound to the
    recorder command bus.

    Args:
        action: ``"attach"`` when the binding becomes active, ``"detach"`` when it goes
            away. Values outside :data:`VALID_BINDING_ACTIONS` are forwarded as-is so
            new event types can roll out without breaking older subscribers, but a
            :func:`carb.log_warn` is emitted to flag accidental typos.
        binding_id: Stable identifier for this binding (e.g. ``"left_secondary"``).
            Listeners may use it to deduplicate multiple attaches from the same source.
        source: Short tag describing the publisher (e.g. ``"vr_button"``,
            ``"keyboard"``).
        label: Optional human-readable text for UI display (e.g. ``"VR Y button"``).
        command: Optional command this binding will dispatch (e.g. ``"toggle"``).
        session_id: Optional ``session_id`` this binding targets; ``None`` means
            broadcast.
    """
    if action not in VALID_BINDING_ACTIONS:
        carb.log_warn(
            f"[EpisodeRecorder] dispatch_episode_binding received non-standard action {action!r}; "
            f"expected one of {sorted(VALID_BINDING_ACTIONS)}. Forwarding as-is."
        )
    payload = {
        "action": action,
        "binding_id": binding_id,
        "source": source,
        "label": label,
        "command": command,
        "session_id": session_id,
    }
    carb.eventdispatcher.get_eventdispatcher().dispatch_event(
        event_name=EPISODE_BINDING_EVENT,
        payload=payload,
    )
