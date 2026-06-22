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

"""USD camera recordable: world pose + intrinsics per frame.

Replay targets the "record a simulation, replay visuals via Replicator" workflow:
downstream render products attached to the same camera prim pick up the recorded
trajectory authored into the anonymous replay sublayer.
"""

from __future__ import annotations

from collections.abc import Mapping
from typing import Any

import carb
import numpy as np

from ..base import ChannelDescriptor, ReplayPolicy
from ..registry import register_recordable
from ._pose_base import _PoseBase
from ._utils import to_numpy


def _camera_intrinsics(prim: Any) -> dict[str, np.ndarray]:
    from pxr import UsdGeom

    cam = UsdGeom.Camera(prim)
    focal = float(cam.GetFocalLengthAttr().Get() or 0.0)
    h_ap = float(cam.GetHorizontalApertureAttr().Get() or 0.0)
    v_ap = float(cam.GetVerticalApertureAttr().Get() or 0.0)
    clip = cam.GetClippingRangeAttr().Get()
    clip_arr = np.asarray([clip[0], clip[1]], dtype=np.float32) if clip is not None else np.zeros(2, dtype=np.float32)
    return {
        "focal_length": np.float32(focal),
        "horizontal_aperture": np.float32(h_ap),
        "vertical_aperture": np.float32(v_ap),
        "clipping_range": clip_arr,
    }


def _set_camera_intrinsics(prim: Any, frame: Mapping[str, Any]) -> None:
    from pxr import UsdGeom

    cam = UsdGeom.Camera(prim)
    if "focal_length" in frame:
        cam.GetFocalLengthAttr().Set(float(frame["focal_length"]))
    if "horizontal_aperture" in frame:
        cam.GetHorizontalApertureAttr().Set(float(frame["horizontal_aperture"]))
    if "vertical_aperture" in frame:
        cam.GetVerticalApertureAttr().Set(float(frame["vertical_aperture"]))
    if "clipping_range" in frame:
        clip = frame["clipping_range"]
        cam.GetClippingRangeAttr().Set((float(clip[0]), float(clip[1])))


@register_recordable
class CameraRecordable(_PoseBase):
    """Records a camera prim's world pose + intrinsics per frame.

    Static fields (``resolution``) are stored in the manifest rather than per-frame.

    Args:
        group: Recordable HDF5 group path.
        prim_path: USD prim path bound to the recordable.
        resolution: Camera resolution override, or None to use the render product resolution.
    """

    TYPE_ID = "camera"

    def __init__(
        self,
        *,
        group: str,
        prim_path: str,
        resolution: tuple[int, int] | None = None,
    ) -> None:
        super().__init__(group=group, prim_path=prim_path)
        self.resolution: tuple[int, int] | None = tuple(resolution) if resolution is not None else None
        self._stage: Any = None

    def describe_channels(self) -> dict[str, ChannelDescriptor]:
        """Describe the recorded channels.

        Returns:
            Channel descriptors keyed by channel name.
        """
        return {
            "position": ChannelDescriptor(shape=(3,), dtype="f4", units="meters", space="world"),
            "orientation": ChannelDescriptor(shape=(4,), dtype="f4", quaternion_order="wxyz", space="world"),
            "focal_length": ChannelDescriptor(shape=(), dtype="f4"),
            "horizontal_aperture": ChannelDescriptor(shape=(), dtype="f4"),
            "vertical_aperture": ChannelDescriptor(shape=(), dtype="f4"),
            "clipping_range": ChannelDescriptor(shape=(2,), dtype="f4"),
        }

    def on_session_open(self, stage: Any) -> None:
        """Open the recordable session.

        Args:
            stage: USD stage to use.
        """
        super().on_session_open(stage)
        self._stage = stage

    def on_session_close(self) -> None:
        """Close the recordable session."""
        super().on_session_close()
        self._stage = None

    def _prim(self) -> Any:
        return self._stage.GetPrimAtPath(self.prim_path)

    def pose_paths(self) -> list[str] | None:
        """Return the pose paths consumed by this recordable.

        Returns:
            Prim paths for shared pose batching, or None when disabled.
        """
        return [self.prim_path]

    def consume_pose_batch(self, positions: np.ndarray, orientations: np.ndarray) -> dict[str, np.ndarray]:
        """Consume a batched pose sample.

        Args:
            positions: Batched positions for this recordable.
            orientations: Batched orientations for this recordable.

        Returns:
            Sampled frame data keyed by channel name.
        """
        intrin = _camera_intrinsics(self._prim())
        return {
            "position": positions[0],
            "orientation": orientations[0],
            **intrin,
        }

    def sample(self) -> dict[str, np.ndarray]:
        """Sample one frame of data.

        Returns:
            Sampled frame data keyed by channel name.
        """
        if self._wrapper is None:
            raise RuntimeError(f"CameraRecordable {self.prim_path}: not bound.")
        pos_wp, quat_wp = self._wrapper.get_world_poses()
        pos = to_numpy(pos_wp)[0].astype(np.float32, copy=False)
        quat = to_numpy(quat_wp)[0].astype(np.float32, copy=False)
        intrin = _camera_intrinsics(self._prim())
        return {
            "position": pos,
            "orientation": quat,
            **intrin,
        }

    def apply(self, frame: Mapping[str, np.ndarray], *, policy: ReplayPolicy) -> None:
        """Apply one recorded frame.

        Args:
            frame: Frame data keyed by channel name.
            policy: Replay policy controlling error handling.
        """
        if self._wrapper is None:
            if policy.strictness == "strict":
                raise RuntimeError(f"CameraRecordable {self.prim_path}: not bound.")
            return
        try:
            pos = np.asarray(frame["position"], dtype=np.float32).reshape(1, 3)
            quat = np.asarray(frame["orientation"], dtype=np.float32).reshape(1, 4)
        except (KeyError, ValueError) as exc:
            if policy.strictness == "strict":
                raise
            carb.log_warn(f"[CameraRecordable {self.prim_path}] malformed frame: {exc}")
            return
        try:
            self._wrapper.set_world_poses(positions=pos, orientations=quat)
            _set_camera_intrinsics(self._prim(), frame)
        except Exception as exc:
            if policy.strictness == "strict":
                raise
            carb.log_warn(f"[CameraRecordable {self.prim_path}] apply failed: {exc}")

    def to_manifest(self) -> dict[str, Any]:
        """Serialize this object to a manifest entry.

        Returns:
            JSON-friendly manifest entry.
        """
        entry: dict[str, Any] = {
            "type": self.TYPE_ID,
            "group": self.group,
            "prim_path": self.prim_path,
        }
        if self.resolution is not None:
            entry["resolution"] = [int(self.resolution[0]), int(self.resolution[1])]
        return entry

    @classmethod
    def from_manifest(cls, entry: Mapping[str, Any]) -> CameraRecordable:
        """Create an instance from a manifest entry.

        Args:
            entry: Manifest entry used to reconstruct the recordable.

        Returns:
            Recordable reconstructed from the manifest entry.
        """
        res = entry.get("resolution")
        resolution = tuple(res) if res is not None else None
        return cls(group=entry["group"], prim_path=entry["prim_path"], resolution=resolution)
