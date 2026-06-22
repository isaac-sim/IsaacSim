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

"""Shared implementation for single-prim pose recordables.

:class:`RigidBodyRecordable` and :class:`XformRecordable` differ only in their
``TYPE_ID`` and default ``space``; this module holds the behavior they share:
``XformPrim`` binding, pose-batch participation, ``wxyz`` frame shaping, and
the self-healing ``xformOp`` retry logic used during replay.
"""

from __future__ import annotations

from collections.abc import Mapping
from typing import Any

import carb
import numpy as np

from ..base import ChannelDescriptor, Recordable, ReplayPolicy
from ._utils import is_missing_xform_ops_error, to_numpy_f32


class _PoseBase(Recordable):
    """Common logic for a single-prim ``position`` + ``orientation`` recordable.

    Participates in the recorder's shared pose batch via :meth:`pose_paths` /
    :meth:`consume_pose_batch`, so every-tick pose reads fold into a single
    ``XformPrim.get_world_poses()`` across all batch participants rather than one
    call per recordable.

    Args:
        group: Recordable HDF5 group path.
        prim_path: USD prim path bound to the recordable.
        space: Coordinate space tag for sampled poses.
    """

    def __init__(self, *, group: str, prim_path: str, space: str = "world") -> None:
        super().__init__(group=group)
        if space not in ("world", "local"):
            raise ValueError(f"{type(self).__name__}.space must be 'world' or 'local', got {space!r}.")
        self.prim_path = str(prim_path)
        self.space = space
        self._wrapper: Any = None
        self._xform_ops_reset: bool = False

    def describe_channels(self) -> dict[str, ChannelDescriptor]:
        return {
            "position": ChannelDescriptor(shape=(3,), dtype="f4", units="meters", space=self.space),
            "orientation": ChannelDescriptor(shape=(4,), dtype="f4", quaternion_order="wxyz", space=self.space),
        }

    def on_session_open(self, stage: Any) -> None:
        prim = stage.GetPrimAtPath(self.prim_path) if stage is not None else None
        if prim is None or not prim.IsValid():
            raise RuntimeError(f"{type(self).__name__}: prim {self.prim_path} not found on stage.")
        from isaacsim.core.experimental.prims import XformPrim

        self._wrapper = XformPrim(self.prim_path)

    def on_session_close(self) -> None:
        self._wrapper = None
        self._xform_ops_reset = False

    # ---- pose batch participation -----------------------------------------
    def pose_paths(self) -> list[str] | None:
        """Single-prim recordables only batch world-space reads. Local-space opts out.

        Local-space sampling goes through ``XformPrim.get_local_poses()``, which has
        different semantics than the world-space batch; keeping it on the per-recordable
        path avoids invalidating the batch invariants.

        Returns:
            Prim paths for shared pose batching, or None when disabled.
        """
        if self.space != "world" or not self.prim_path:
            return None
        return [self.prim_path]

    def consume_pose_batch(self, positions: np.ndarray, orientations: np.ndarray) -> dict[str, np.ndarray]:
        return {"position": positions[0], "orientation": orientations[0]}

    # ---- sample / apply ---------------------------------------------------
    def sample(self) -> dict[str, np.ndarray]:
        if self._wrapper is None:
            raise RuntimeError(f"{type(self).__name__} {self.prim_path}: not bound. Call on_session_open first.")
        if self.space == "world":
            pos_wp, quat_wp = self._wrapper.get_world_poses()
        else:
            pos_wp, quat_wp = self._wrapper.get_local_poses()
        pos = to_numpy_f32(pos_wp)[0]
        quat = to_numpy_f32(quat_wp)[0]
        return {"position": pos, "orientation": quat}

    def apply(self, frame: Mapping[str, np.ndarray], *, policy: ReplayPolicy) -> None:
        if self._wrapper is None:
            if policy.strictness == "strict":
                raise RuntimeError(f"{type(self).__name__} {self.prim_path}: not bound.")
            return
        try:
            pos = np.asarray(frame["position"], dtype=np.float32).reshape(1, 3)
            quat = np.asarray(frame["orientation"], dtype=np.float32).reshape(1, 4)
        except (KeyError, ValueError) as exc:
            if policy.strictness == "strict":
                raise
            carb.log_warn(f"[{type(self).__name__} {self.prim_path}] malformed frame: {exc}")
            return

        def _write() -> None:
            if self.space == "world":
                self._wrapper.set_world_poses(positions=pos, orientations=quat)
            else:
                self._wrapper.set_local_poses(translations=pos, orientations=quat)

        try:
            _write()
        except Exception as exc:
            if self._xform_ops_reset or not is_missing_xform_ops_error(exc):
                if policy.strictness == "strict":
                    raise
                carb.log_warn(f"[{type(self).__name__} {self.prim_path}] apply failed: {exc}")
                return
            try:
                self._wrapper.reset_xform_op_properties()
                self._xform_ops_reset = True
                _write()
            except Exception as retry_exc:
                if policy.strictness == "strict":
                    raise
                carb.log_warn(
                    f"[{type(self).__name__} {self.prim_path}] "
                    f"apply failed after reset_xform_op_properties: {retry_exc}"
                )
