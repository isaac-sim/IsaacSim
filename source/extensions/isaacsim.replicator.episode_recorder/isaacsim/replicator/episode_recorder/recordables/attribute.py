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

"""Generic USD attribute recordable. Works for any scalar / vector typed attribute."""

from __future__ import annotations

from collections.abc import Mapping
from typing import Any

import carb
import numpy as np

from ..base import ChannelDescriptor, Recordable, ReplayPolicy
from ..registry import register_recordable

_CHANNEL_NAME = "value"


def _infer_shape_dtype(value: Any) -> tuple[tuple[int, ...], str]:
    """Return ``(shape, dtype_code)`` for a USD attribute probe value.

    Args:
        value: Value to process.

    Returns:
        Shape and dtype inferred from the value.
    """
    if value is None:
        return ((), "f4")
    if isinstance(value, bool):
        return ((), "bool")
    if isinstance(value, int):
        return ((), "i8")
    if isinstance(value, float):
        return ((), "f4")
    arr = np.asarray(value)
    dtype_code = arr.dtype.str.lstrip("<>|=")
    # Normalize common dtypes to the short codes our ChannelDescriptor accepts.
    mapping = {"f4": "f4", "f8": "f8", "i4": "i4", "i8": "i8", "u1": "u1"}
    dtype_code = mapping.get(dtype_code, dtype_code)
    return (arr.shape, dtype_code)


@register_recordable
class AttributeRecordable(Recordable):
    """Records one USD attribute per frame.

    The attribute's shape and dtype are probed on :meth:`on_session_open`; both are
    persisted in the manifest so the replayer can construct a matching descriptor
    without live access to the stage.

    Args:
        group: HDF5 group path.
        prim_path: Absolute USD prim path.
        attribute_name: Attribute name on the prim (e.g. ``"intensity"``).
        shape: Optional explicit per-frame shape (otherwise probed).
        dtype: Optional explicit dtype code (otherwise probed).
    """

    TYPE_ID = "usd_attribute"

    def __init__(
        self,
        *,
        group: str,
        prim_path: str,
        attribute_name: str,
        shape: tuple[int, ...] | None = None,
        dtype: str | None = None,
    ) -> None:
        super().__init__(group=group)
        self.prim_path = str(prim_path)
        self.attribute_name = str(attribute_name)
        self._shape: tuple[int, ...] | None = tuple(shape) if shape is not None else None
        self._dtype: str | None = dtype
        self._stage: Any = None

    def describe_channels(self) -> dict[str, ChannelDescriptor]:
        """Describe the recorded channels.

        Returns:
            Channel descriptors keyed by channel name.
        """
        shape = self._shape if self._shape is not None else ()
        dtype = self._dtype if self._dtype is not None else "f4"
        return {_CHANNEL_NAME: ChannelDescriptor(shape=shape, dtype=dtype)}

    def _get_attr(self) -> Any:
        prim = self._stage.GetPrimAtPath(self.prim_path) if self._stage is not None else None
        if prim is None or not prim.IsValid():
            raise RuntimeError(f"AttributeRecordable: prim {self.prim_path} not found.")
        attr = prim.GetAttribute(self.attribute_name)
        if not attr:
            raise RuntimeError(f"AttributeRecordable: attribute {self.prim_path}.{self.attribute_name} not found.")
        return attr

    def on_session_open(self, stage: Any) -> None:
        """Open the recordable session.

        Args:
            stage: USD stage to use.
        """
        self._stage = stage
        attr = self._get_attr()
        if self._shape is None or self._dtype is None:
            probe = attr.Get()
            shape, dtype = _infer_shape_dtype(probe)
            if self._shape is None:
                self._shape = shape
            if self._dtype is None:
                self._dtype = dtype

    def on_session_close(self) -> None:
        """Close the recordable session."""
        self._stage = None

    def sample(self) -> dict[str, np.ndarray]:
        """Sample one frame of data.

        Returns:
            Sampled frame data keyed by channel name.
        """
        attr = self._get_attr()
        value = attr.Get()
        if value is None:
            shape = self._shape or ()
            dtype = self._dtype or "f4"
            return {_CHANNEL_NAME: np.zeros(shape, dtype=dtype)}
        arr = np.asarray(value)
        return {_CHANNEL_NAME: arr.astype(self._dtype or arr.dtype.str.lstrip("<>|="), copy=False)}

    def apply(self, frame: Mapping[str, np.ndarray], *, policy: ReplayPolicy) -> None:
        """Apply one recorded frame.

        Args:
            frame: Frame data keyed by channel name.
            policy: Replay policy controlling error handling.
        """
        if self._stage is None:
            if policy.strictness == "strict":
                raise RuntimeError(f"AttributeRecordable {self.prim_path}.{self.attribute_name}: not bound.")
            return
        try:
            attr = self._get_attr()
        except RuntimeError as exc:
            if policy.strictness == "strict":
                raise
            carb.log_warn(str(exc))
            return
        if _CHANNEL_NAME not in frame:
            if policy.strictness == "strict":
                raise KeyError(f"AttributeRecordable frame missing '{_CHANNEL_NAME}'.")
            return
        raw = frame[_CHANNEL_NAME]
        try:
            if self._shape == () or (isinstance(self._shape, tuple) and len(self._shape) == 0):
                attr.Set(raw.item() if hasattr(raw, "item") else raw)
            else:
                attr.Set(raw.tolist() if hasattr(raw, "tolist") else raw)
        except Exception as exc:
            if policy.strictness == "strict":
                raise
            carb.log_warn(f"[AttributeRecordable {self.prim_path}.{self.attribute_name}] apply failed: {exc}")

    def to_manifest(self) -> dict[str, Any]:
        """Serialize this object to a manifest entry.

        Returns:
            JSON-friendly manifest entry.
        """
        return {
            "type": self.TYPE_ID,
            "group": self.group,
            "prim_path": self.prim_path,
            "attribute_name": self.attribute_name,
            "shape": list(self._shape) if self._shape is not None else None,
            "dtype": self._dtype,
        }

    @classmethod
    def from_manifest(cls, entry: Mapping[str, Any]) -> AttributeRecordable:
        """Create an instance from a manifest entry.

        Args:
            entry: Manifest entry used to reconstruct the recordable.

        Returns:
            Recordable reconstructed from the manifest entry.
        """
        shape = entry.get("shape")
        shape_t = tuple(shape) if shape is not None else None
        return cls(
            group=entry["group"],
            prim_path=entry["prim_path"],
            attribute_name=entry["attribute_name"],
            shape=shape_t,
            dtype=entry.get("dtype"),
        )
