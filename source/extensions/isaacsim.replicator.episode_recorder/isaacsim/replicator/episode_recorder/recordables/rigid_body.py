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

"""World-pose recordable for rigid bodies.

Thin alias over :class:`_PoseBase` fixed to ``space="world"``. Reads / writes go
through :class:`isaacsim.core.experimental.prims.XformPrim` — no physics-tensor
backend — and participate in the recorder's shared pose batch so every tick
incurs a single Fabric traversal across all pose recordables.
"""

from __future__ import annotations

from collections.abc import Mapping
from typing import Any

from ..registry import register_recordable
from ._pose_base import _PoseBase


@register_recordable
class RigidBodyRecordable(_PoseBase):
    """Records the world ``position`` + ``wxyz orientation`` of a single rigid body."""

    TYPE_ID = "rigid_body"

    def __init__(self, *, group: str, prim_path: str) -> None:
        super().__init__(group=group, prim_path=prim_path, space="world")

    def to_manifest(self) -> dict[str, Any]:
        return {"type": self.TYPE_ID, "group": self.group, "prim_path": self.prim_path}

    @classmethod
    def from_manifest(cls, entry: Mapping[str, Any]) -> RigidBodyRecordable:
        return cls(group=entry["group"], prim_path=entry["prim_path"])
