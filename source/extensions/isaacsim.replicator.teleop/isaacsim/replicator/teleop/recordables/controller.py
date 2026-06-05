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

"""Per-side VR controller snapshot recordable (inputs + aim pose)."""

from __future__ import annotations

from collections.abc import Mapping
from typing import Any

import numpy as np
from isaacsim.replicator.episode_recorder import (
    ChannelDescriptor,
    Recordable,
    ReplayPolicy,
    register_recordable,
)

_SCALAR_FIELDS = ("trigger_value", "squeeze_value", "thumbstick_x", "thumbstick_y")
_BOOL_FIELDS = ("primary_click", "secondary_click", "thumbstick_click")


def _extract_aim(snapshot: Any) -> Any:
    if snapshot is None:
        return None
    aim = getattr(snapshot, "aim_pose", None)
    if aim is None or not getattr(aim, "is_valid", False) or getattr(aim, "pose", None) is None:
        return None
    return aim


@register_recordable
class TeleopControllerRecordable(Recordable):
    """Records one VR controller's inputs + aim pose per frame.

    Binds to a :class:`TeleopManager` via ``add_controller_inputs_observer``. The
    observer caches the latest left/right snapshots; this recordable samples the cache.

    One instance records one side. Create two for bimanual setups.

    Args:
        group: HDF5 group path (e.g. ``"teleop/left"``).
        side: ``"left"`` or ``"right"``.
        record_aim_pose: Whether to record aim_pose channels.
        teleop_manager: Live :class:`TeleopManager`. Passed here for binding; NOT
            persisted in the manifest (replay ignores teleop anyway).
    """

    TYPE_ID = "teleop_controller"

    def __init__(
        self,
        *,
        group: str,
        side: str,
        record_aim_pose: bool = True,
        teleop_manager: Any = None,
    ) -> None:
        super().__init__(group=group)
        if side not in ("left", "right"):
            raise ValueError(f"TeleopControllerRecordable.side must be 'left' or 'right', got {side!r}.")
        self.side = side
        self.record_aim_pose = bool(record_aim_pose)
        self._tm = teleop_manager
        self._observer_attached: bool = False
        self._last_left: Any = None
        self._last_right: Any = None

    def describe_channels(self) -> dict[str, ChannelDescriptor]:
        channels: dict[str, ChannelDescriptor] = {
            "trigger": ChannelDescriptor(shape=(), dtype="f4", units="normalized"),
            "squeeze": ChannelDescriptor(shape=(), dtype="f4", units="normalized"),
            "thumbstick_x": ChannelDescriptor(shape=(), dtype="f4", units="normalized"),
            "thumbstick_y": ChannelDescriptor(shape=(), dtype="f4", units="normalized"),
            "primary_click": ChannelDescriptor(shape=(), dtype="u1"),
            "secondary_click": ChannelDescriptor(shape=(), dtype="u1"),
            "thumbstick_click": ChannelDescriptor(shape=(), dtype="u1"),
        }
        if self.record_aim_pose:
            channels["aim_position"] = ChannelDescriptor(shape=(3,), dtype="f4")
            channels["aim_orientation"] = ChannelDescriptor(shape=(4,), dtype="f4", quaternion_order="wxyz")
        return channels

    def on_session_open(self, stage: Any) -> None:
        if self._tm is None:
            return
        if not hasattr(self._tm, "add_controller_inputs_observer"):
            return
        self._tm.add_controller_inputs_observer(self._on_inputs)
        self._observer_attached = True

    def on_session_close(self) -> None:
        if self._observer_attached and self._tm is not None:
            remove = getattr(self._tm, "remove_controller_inputs_observer", None)
            if remove is not None:
                remove(self._on_inputs)
        self._observer_attached = False

    def _on_inputs(self, left: Any, right: Any) -> None:
        self._last_left = left
        self._last_right = right

    def _current(self) -> Any:
        return self._last_left if self.side == "left" else self._last_right

    def sample(self) -> dict[str, Any]:
        snapshot = self._current()
        inputs = getattr(snapshot, "inputs", None) if snapshot is not None else None
        frame: dict[str, Any] = {}
        for field_name in _SCALAR_FIELDS:
            channel = field_name.replace("_value", "") if field_name.endswith("_value") else field_name
            value = float(getattr(inputs, field_name, 0.0) or 0.0) if inputs is not None else 0.0
            frame[channel] = value
        for field_name in _BOOL_FIELDS:
            value = 1 if (inputs is not None and bool(getattr(inputs, field_name, False))) else 0
            frame[field_name] = value
        if self.record_aim_pose:
            aim = _extract_aim(snapshot)
            if aim is None:
                frame["aim_position"] = np.zeros(3, dtype=np.float32)
                frame["aim_orientation"] = np.zeros(4, dtype=np.float32)
            else:
                p = aim.pose.position
                o = aim.pose.orientation
                frame["aim_position"] = np.asarray([p.x, p.y, p.z], dtype=np.float32)
                frame["aim_orientation"] = np.asarray([o.w, o.x, o.y, o.z], dtype=np.float32)
        return frame

    def apply(self, frame: Mapping[str, Any], *, policy: ReplayPolicy) -> None:
        return

    def to_manifest(self) -> dict[str, Any]:
        return {
            "type": self.TYPE_ID,
            "group": self.group,
            "side": self.side,
            "record_aim_pose": self.record_aim_pose,
        }

    @classmethod
    def from_manifest(cls, entry: Mapping[str, Any]) -> TeleopControllerRecordable:
        return cls(
            group=entry["group"],
            side=entry["side"],
            record_aim_pose=bool(entry.get("record_aim_pose", True)),
            teleop_manager=None,
        )
