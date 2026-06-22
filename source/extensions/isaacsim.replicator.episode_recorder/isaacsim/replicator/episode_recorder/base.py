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

"""Define core recorder abstractions.

This module provides :class:`ChannelDescriptor`, :class:`Recordable`,
:class:`SamplingConfig`, and :class:`ReplayPolicy`. These types are backend-agnostic.
``Recordable`` plugins own a channel schema, sample one frame of data, apply one frame
back to a live stage, and serialize / deserialize their binding through the manifest.
"""

from __future__ import annotations

from abc import ABC, abstractmethod
from collections.abc import Mapping
from dataclasses import dataclass, field
from typing import Any

import numpy as np

_NUMPY_DTYPES = {"f4", "f8", "i1", "i2", "i4", "i8", "u1", "u2", "u4", "u8", "bool"}


@dataclass(frozen=True)
class ChannelDescriptor:
    """One dense per-frame signal produced by a :class:`Recordable`.

    Attributes:
        shape: Per-frame shape (no frames axis). ``()`` for scalars.
        dtype: NumPy dtype code (``"f4"``, ``"f8"``, ``"i8"``, ``"u1"``, ``"bool"``, ...).
        units: Optional physical unit string (e.g. ``"meters"``, ``"radians"``).
        space: Optional space tag (e.g. ``"world"``, ``"local"``, ``"stage_meters"``).
        quaternion_order: When the channel is a quaternion, the order of components
            (``"wxyz"`` or ``"xyzw"``). Omit for non-quaternion channels.
        attrs: Free-form attribute dict written on the backing HDF5 dataset.
    """

    shape: tuple[int, ...]
    dtype: str
    units: str | None = None
    space: str | None = None
    quaternion_order: str | None = None
    attrs: Mapping[str, Any] = field(default_factory=dict)

    def __post_init__(self) -> None:
        """Validate normalized instance state."""
        if not isinstance(self.shape, tuple):
            object.__setattr__(self, "shape", tuple(self.shape))
        for dim in self.shape:
            if not isinstance(dim, int) or dim < 0:
                raise ValueError(f"ChannelDescriptor.shape contains invalid dim {dim!r}; expected non-negative ints.")
        if not isinstance(self.dtype, str):
            raise TypeError(f"ChannelDescriptor.dtype must be a dtype code string, got {type(self.dtype).__name__}.")
        if self.dtype not in _NUMPY_DTYPES:
            try:
                np.dtype(self.dtype)
            except TypeError as exc:
                raise ValueError(f"ChannelDescriptor.dtype {self.dtype!r} is not a recognized NumPy dtype.") from exc
        if self.quaternion_order is not None and self.quaternion_order not in ("wxyz", "xyzw"):
            raise ValueError(
                f"ChannelDescriptor.quaternion_order must be 'wxyz' or 'xyzw', got {self.quaternion_order!r}."
            )


@dataclass
class SamplingConfig:
    """How frequently :meth:`Recordable.sample` is invoked by the recorder.

    Attributes:
        mode: ``"physics_post_step"`` (default) samples on every post-step physics callback;
            ``"app_update"`` samples on every ``omni.kit.app`` update tick.
        decimation: Only every N-th tick triggers a sample (``1`` = every tick).
    """

    mode: str = "physics_post_step"
    decimation: int = 1

    def __post_init__(self) -> None:
        """Validate normalized instance state."""
        if self.mode not in ("physics_post_step", "app_update"):
            raise ValueError(f"SamplingConfig.mode must be 'physics_post_step' or 'app_update', got {self.mode!r}.")
        if self.decimation < 1:
            raise ValueError(f"SamplingConfig.decimation must be >= 1, got {self.decimation}.")


@dataclass
class ReplayPolicy:
    """How the replayer treats binding failures at replay time.

    Replay is always pure-USD and visuals-only — every write lands in the replayer's
    anonymous sublayer and the physics tensor backend is never engaged — so there is
    no per-policy kinematic / dynamics switch. Only error-handling strictness is
    configurable.

    Attributes:
        strictness: ``"best_effort"`` (default) logs and skips; ``"strict"`` raises on
            the first missing binding or apply error.
    """

    strictness: str = "best_effort"

    def __post_init__(self) -> None:
        """Validate normalized instance state."""
        if self.strictness not in ("best_effort", "strict"):
            raise ValueError(f"ReplayPolicy.strictness must be 'best_effort' or 'strict', got {self.strictness!r}.")


class Recordable(ABC):
    """Plugin interface: schema + sampling + applying + manifest round-trip.

    One :class:`Recordable` instance owns one HDF5 group. Its :meth:`describe_channels`
    declares the per-frame dtype / shape; its :meth:`sample` produces one frame;
    its :meth:`apply` writes one frame back to a live stage for visual replay; and
    :meth:`to_manifest` / :meth:`from_manifest` round-trip its binding so the replayer
    can reconstruct it with no caller-side wiring.

    Subclasses must set a unique :attr:`TYPE_ID`. Register with
    :func:`register_recordable` so :meth:`from_manifest` can be invoked during replay.

    Args:
        group: Recordable HDF5 group path.
    """

    #: Stable string id used for registry lookup. Must be unique across all recordables.
    TYPE_ID: str = ""

    def __init__(self, *, group: str) -> None:
        if not isinstance(group, str):
            raise TypeError(f"Recordable.group must be a str, got {type(group).__name__}.")
        normalized = group.strip().strip("/")
        if not normalized:
            raise ValueError("Recordable.group must be a non-empty slash-separated path.")
        if "//" in normalized or any(not seg for seg in normalized.split("/")):
            raise ValueError(f"Recordable.group {group!r} has empty path segments.")
        self.group: str = normalized

    @abstractmethod
    def describe_channels(self) -> dict[str, ChannelDescriptor]:
        """Return per-channel descriptors keyed by channel name (no slashes).

        Returns:
            Channel descriptors keyed by channel name.
        """

    def on_session_open(self, stage: Any) -> None:
        """Called once per :meth:`EpisodeRecorder.open_session`. Bind live handles here.

        Args:
            stage: USD stage to use.
        """

    def on_session_close(self) -> None:
        """Called once per :meth:`EpisodeRecorder.close_session`. Release handles here."""

    def on_episode_start(self) -> None:
        """Called at the start of every episode (optional)."""

    def on_episode_end(self) -> None:
        """Called at the end of every episode (optional)."""

    @abstractmethod
    def sample(self) -> dict[str, np.ndarray | float | int]:
        """Return one frame of data. Keys must exactly match :meth:`describe_channels`.

        Returns:
            Sampled frame data keyed by channel name.
        """

    def pose_paths(self) -> list[str] | None:
        """Prim paths this recordable wants sampled via the shared pose batch.

        Recordables that only need world position + ``wxyz`` orientation (``XformPrim``
        reads) can opt into a session-wide batched read by returning a non-empty list
        here. The recorder then allocates one shared ``XformPrim`` over the concatenation
        of every participant's paths and calls :meth:`consume_pose_batch` each tick with
        the relevant slice of the pre-sampled arrays — avoiding per-recordable Fabric
        traversals.

        Return ``None`` (default) or an empty list to opt out; the recorder will call
        :meth:`sample` directly each tick instead.

        Returns:
            Prim paths for shared pose batching, or None when disabled.
        """
        return None

    def consume_pose_batch(
        self,
        positions: np.ndarray,
        orientations: np.ndarray,
    ) -> dict[str, np.ndarray | float | int]:
        """Build a frame from pre-sampled batched poses.

        Only invoked when :meth:`pose_paths` returned a non-empty list. ``positions``
        has shape ``(len(pose_paths()), 3)`` and ``orientations`` has shape
        ``(len(pose_paths()), 4)`` (``wxyz``). Both are ``float32`` NumPy **views**
        into the shared batch — do not hold references across ticks.

        Default implementation raises :class:`NotImplementedError`; opt-in recordables
        must override.

        Args:
            positions: Batched positions for this recordable.
            orientations: Batched orientations for this recordable.

        Returns:
            Sampled frame data keyed by channel name.
        """
        raise NotImplementedError(
            f"{type(self).__name__} opted into pose batching via pose_paths() but did "
            "not implement consume_pose_batch()."
        )

    @abstractmethod
    def apply(self, frame: Mapping[str, np.ndarray], *, policy: ReplayPolicy) -> None:
        """Apply one frame back to the live stage honoring the replay policy.

        Implementations should treat missing / malformed channels defensively in
        ``best_effort`` mode and raise in ``strict`` mode.

        Args:
            frame: Frame data keyed by channel name.
            policy: Replay policy controlling error handling.
        """

    @abstractmethod
    def to_manifest(self) -> dict[str, Any]:
        """Serialize this recordable's binding as a JSON-friendly dict.

        Must include ``"type"`` (= :attr:`TYPE_ID`) and ``"group"``, plus any extra
        fields :meth:`from_manifest` needs to reconstruct the instance.

        Returns:
            JSON-friendly manifest entry.
        """

    @classmethod
    @abstractmethod
    def from_manifest(cls, entry: Mapping[str, Any]) -> Recordable:
        """Inverse of :meth:`to_manifest`: construct a recordable from the manifest entry.

        Args:
            entry: Manifest entry used to reconstruct the recordable.

        Returns:
            Recordable reconstructed from the manifest entry.
        """
