# SPDX-FileCopyrightText: Copyright (c) 2025-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Bridges a live :class:`TeleopManager` into the Episode Recorder session pipeline.

Installed automatically by :class:`TeleopManager` on construction, this module
registers a session injector with
``isaacsim.replicator.episode_recorder`` so the standalone Episode Recorder
window (or any other UI that calls :func:`apply_session_injectors`) attaches
teleop controller and head-pose recordables to its sessions without needing
any direct knowledge of teleop.

The aim / head-pose capture flags are read from carb settings so they can be
toggled without running UI code:

- ``/persistent/exts/isaacsim.replicator.teleop/record/record_aim_pose``
  (default ``True``)
- ``/persistent/exts/isaacsim.replicator.teleop/record/record_head_pose``
  (default ``True``)
"""

from __future__ import annotations

from collections.abc import Callable
from typing import TYPE_CHECKING, Any

import carb
import carb.settings

if TYPE_CHECKING:
    from isaacsim.replicator.episode_recorder import EpisodeRecorder, Recordable

_RECORD_AIM_KEY = "/persistent/exts/isaacsim.replicator.teleop/record/record_aim_pose"
_RECORD_HEAD_KEY = "/persistent/exts/isaacsim.replicator.teleop/record/record_head_pose"


def _add_if_absent(recorder: "EpisodeRecorder", recordable: "Recordable") -> None:
    """Add ``recordable`` to ``recorder`` unless its group is already claimed.

    Keeps the teleop injector idempotent across multiple ``TeleopManager``
    instances and defends against callers that pre-added a ``teleop/*`` group
    manually.

    Args:
        recorder: Value for recorder.
        recordable: Value for recordable.
    """
    group = recordable.group
    for existing in recorder.recordables():
        if existing.group == group:
            carb.log_info(f"[TeleopSessionInjector] group {group!r} already registered; skipping teleop-side add.")
            return
    recorder.add(recordable)


def install_teleop_session_injector(teleop_manager: Any) -> Callable[[], None]:
    """Register a session injector bound to ``teleop_manager``.

    Returns a zero-arg handle that unregisters the injector when called;
    :class:`TeleopManager` invokes this in ``destroy`` so no dangling
    injectors survive after teardown. Safe to call when the episode_recorder
    extension is not loaded — import failure is silently swallowed and a
    no-op handle is returned so teleop continues to function.

    Args:
        teleop_manager: Value for teleop manager.

    Returns:
        The requested value.
    """
    try:
        from isaacsim.replicator.episode_recorder import register_session_injector
    except ImportError:
        return lambda: None

    settings = carb.settings.get_settings()
    settings.set_default_bool(_RECORD_AIM_KEY, True)
    settings.set_default_bool(_RECORD_HEAD_KEY, True)

    def _inject(recorder: "EpisodeRecorder") -> None:
        from .recordables import TeleopControllerRecordable, TeleopHeadRecordable

        record_aim = bool(settings.get_as_bool(_RECORD_AIM_KEY))
        record_head = bool(settings.get_as_bool(_RECORD_HEAD_KEY))

        for side in ("left", "right"):
            _add_if_absent(
                recorder,
                TeleopControllerRecordable(
                    group=f"teleop/{side}",
                    side=side,
                    record_aim_pose=record_aim,
                    teleop_manager=teleop_manager,
                ),
            )
        if record_head and hasattr(teleop_manager, "add_head_observer"):
            _add_if_absent(recorder, TeleopHeadRecordable(group="teleop/head", teleop_manager=teleop_manager))

    return register_session_injector(_inject)
