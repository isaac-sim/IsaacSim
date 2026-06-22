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

"""Sim clock recordable. Always attached by :class:`EpisodeRecorder`."""

from __future__ import annotations

import time
from collections.abc import Mapping
from typing import Any

from ..base import ChannelDescriptor, Recordable, ReplayPolicy
from ..registry import register_recordable

DEFAULT_GROUP = "meta/time"


@register_recordable
class SimTimeRecordable(Recordable):
    """Records ``sim_time``, ``physics_step``, and ``wall_time`` per frame.

    Apply is a no-op: sim time channels exist to drive the replayer's timeline sync,
    not to mutate the live stage.

    Args:
        group: Recordable HDF5 group path.
    """

    TYPE_ID = "sim_time"

    def __init__(self, *, group: str = DEFAULT_GROUP) -> None:
        super().__init__(group=group)

    def describe_channels(self) -> dict[str, ChannelDescriptor]:
        """Describe the recorded channels.

        Returns:
            Channel descriptors keyed by channel name.
        """
        return {
            "sim_time": ChannelDescriptor(shape=(), dtype="f8", units="seconds"),
            "physics_step": ChannelDescriptor(shape=(), dtype="i8"),
            "wall_time": ChannelDescriptor(shape=(), dtype="f8", units="seconds"),
        }

    def sample(self) -> dict[str, Any]:
        """Sample one frame of data.

        Returns:
            Sampled frame data keyed by channel name.
        """
        from isaacsim.core.simulation_manager import SimulationManager

        return {
            "sim_time": float(SimulationManager.get_simulation_time()),
            "physics_step": int(SimulationManager.get_num_physics_steps()),
            "wall_time": time.time(),
        }

    def apply(self, frame: Mapping[str, Any], *, policy: ReplayPolicy) -> None:
        """Apply one recorded frame.

        Args:
            frame: Frame data keyed by channel name.
            policy: Replay policy controlling error handling.
        """
        return

    def to_manifest(self) -> dict[str, Any]:
        """Serialize this object to a manifest entry.

        Returns:
            JSON-friendly manifest entry.
        """
        return {"type": self.TYPE_ID, "group": self.group}

    @classmethod
    def from_manifest(cls, entry: Mapping[str, Any]) -> SimTimeRecordable:
        """Create an instance from a manifest entry.

        Args:
            entry: Manifest entry used to reconstruct the recordable.

        Returns:
            Recordable reconstructed from the manifest entry.
        """
        return cls(group=entry.get("group", DEFAULT_GROUP))
