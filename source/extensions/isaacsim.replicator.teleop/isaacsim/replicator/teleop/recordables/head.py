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

"""Headset pose recordable."""

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


def _extract_head(snapshot: Any) -> Any:
    if snapshot is None:
        return None
    if not getattr(snapshot, "is_valid", False) or getattr(snapshot, "pose", None) is None:
        return None
    return snapshot


@register_recordable
class TeleopHeadRecordable(Recordable):
    """Records the VR headset pose per frame. No-op on replay.

    Args:
        group: HDF5 group path (e.g. ``"teleop/head"``).
        teleop_manager: Live :class:`TeleopManager` exposing ``add_head_observer``.
    """

    TYPE_ID = "teleop_head"

    def __init__(self, *, group: str = "teleop/head", teleop_manager: Any = None) -> None:
        super().__init__(group=group)
        self._tm = teleop_manager
        self._observer_attached: bool = False
        self._last_head: Any = None

    def describe_channels(self) -> dict[str, ChannelDescriptor]:
        return {
            "position": ChannelDescriptor(shape=(3,), dtype="f4"),
            "orientation": ChannelDescriptor(shape=(4,), dtype="f4", quaternion_order="wxyz"),
        }

    def on_session_open(self, stage: Any) -> None:
        if self._tm is None or not hasattr(self._tm, "add_head_observer"):
            return
        self._tm.add_head_observer(self._on_head)
        self._observer_attached = True

    def on_session_close(self) -> None:
        if self._observer_attached and self._tm is not None:
            remove = getattr(self._tm, "remove_head_observer", None)
            if remove is not None:
                remove(self._on_head)
        self._observer_attached = False

    def _on_head(self, head: Any) -> None:
        self._last_head = head

    def sample(self) -> dict[str, np.ndarray]:
        head = _extract_head(self._last_head)
        if head is None:
            return {
                "position": np.zeros(3, dtype=np.float32),
                "orientation": np.zeros(4, dtype=np.float32),
            }
        p = head.pose.position
        o = head.pose.orientation
        return {
            "position": np.asarray([p.x, p.y, p.z], dtype=np.float32),
            "orientation": np.asarray([o.w, o.x, o.y, o.z], dtype=np.float32),
        }

    def apply(self, frame: Mapping[str, Any], *, policy: ReplayPolicy) -> None:
        return

    def to_manifest(self) -> dict[str, Any]:
        return {"type": self.TYPE_ID, "group": self.group}

    @classmethod
    def from_manifest(cls, entry: Mapping[str, Any]) -> TeleopHeadRecordable:
        return cls(group=entry.get("group", "teleop/head"), teleop_manager=None)
