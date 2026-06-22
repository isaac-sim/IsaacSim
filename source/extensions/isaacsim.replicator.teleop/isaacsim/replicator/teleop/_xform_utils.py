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

"""Shared helpers for teleop world-pose reads and conversions."""

from __future__ import annotations

from typing import Literal

import isaacsim.core.experimental.utils.stage as stage_utils
import isaacsim.core.experimental.utils.xform as core_xform_utils
import numpy as np
from pxr import Gf

from ._backend import get_teleop_backend


class WorldPosePrimCache:
    """Cache backend-specific prim handles for repeated world-pose reads.

    The teleop backend can switch between USD and FSD-backed reads at runtime.
    This cache keeps one prim handle for the `usd` backend and one for the
    `usdrt`/`fabric` backend so callers can reuse upstream
    `isaacsim.core.experimental.utils.xform.get_world_pose()` without resolving
    the prim path on every read.

    Args:
        prim_path: Value for prim path.
    """

    def __init__(self, prim_path: str = "") -> None:
        self._prim_path = prim_path
        self._cached_prims: dict[str, object | None] = {"usd": None, "usdrt": None}

    @property
    def prim_path(self) -> str:
        """Return the cached prim path.

        Returns:
            The requested value.
        """
        return self._prim_path

    def set_prim_path(self, prim_path: str) -> None:
        """Update the cached prim path and invalidate stored prim handles.

        Args:
            prim_path: Value for prim path.
        """
        if prim_path != self._prim_path:
            self._prim_path = prim_path
            self.clear()

    def clear(self) -> None:
        """Invalidate all cached prim handles."""
        self._cached_prims["usd"] = None
        self._cached_prims["usdrt"] = None

    def get_current_prim(self) -> object | None:
        """Return the cached prim for the active teleop backend.

        Returns:
            The requested value.
        """
        if not self._prim_path:
            return None

        backend = get_world_pose_backend_key()
        prim = self._cached_prims[backend]
        if _is_valid_prim(prim):
            return prim

        stage = stage_utils.get_current_stage(backend=backend)
        prim = stage.GetPrimAtPath(self._prim_path)
        if not _is_valid_prim(prim):
            return None

        self._cached_prims[backend] = prim
        return prim


def get_world_pose_backend_key() -> Literal["usd", "usdrt"]:
    """Return the backend key used for teleop world-pose reads.

    Returns:
        The requested value.
    """
    return "usd" if get_teleop_backend() == "usd" else "usdrt"


def _is_valid_prim(prim: object | None) -> bool:
    """Check whether a cached prim handle is still valid.

    Args:
        prim: Value for prim.

    Returns:
        The requested value.
    """
    return bool(prim is not None and hasattr(prim, "IsValid") and prim.IsValid())


def _resolve_world_pose_target(target: str | WorldPosePrimCache | object) -> object:
    """Resolve a world-pose read target to a backend-specific prim handle.

    Args:
        target: Value for target.

    Returns:
        The requested value.
    """
    if isinstance(target, WorldPosePrimCache):
        prim = target.get_current_prim()
    elif isinstance(target, str):
        prim = WorldPosePrimCache(target).get_current_prim()
    else:
        prim = target

    if not _is_valid_prim(prim):
        raise ValueError("Unable to resolve a valid prim for world-pose read.")
    return prim


def to_numpy_array(values: object, *, copy: bool = False) -> np.ndarray:
    """Convert backend-backed data to a NumPy array.

    Args:
        values: Backend-backed array or array-like object.
        copy: Return an owned copy instead of a view.

    Returns:
        The requested value.
    """
    array = values.numpy() if hasattr(values, "numpy") else np.asarray(values)
    return array.copy() if copy else array


def unpack_world_pose(position_array: np.ndarray, orientation_array: np.ndarray) -> tuple[Gf.Vec3d, Gf.Quatd]:
    """Convert single-pose arrays to `Gf` position and quaternion values.

    Args:
        position_array: Position array in `[x, y, z]` layout.
        orientation_array: Quaternion array in `[w, x, y, z]` layout.

    Returns:
        The requested value.
    """
    position = position_array.reshape(-1, 3)[0]
    orientation = orientation_array.reshape(-1, 4)[0]
    return (
        Gf.Vec3d(float(position[0]), float(position[1]), float(position[2])),
        Gf.Quatd(
            float(orientation[0]),
            float(orientation[1]),
            float(orientation[2]),
            float(orientation[3]),
        ),
    )


def read_world_pose_arrays(
    target: str | WorldPosePrimCache | object, *, copy: bool = False
) -> tuple[np.ndarray, np.ndarray]:
    """Read world-pose arrays through the active teleop backend.

    Args:
        target: Prim path, cached prim resolver, or backend-specific prim handle.
        copy: Return owned NumPy copies instead of backend views.

    Returns:
        The requested value.
    """
    positions, orientations = core_xform_utils.get_world_pose(_resolve_world_pose_target(target))
    return to_numpy_array(positions, copy=copy), to_numpy_array(orientations, copy=copy)


def read_world_pose_gf(target: str | WorldPosePrimCache | object) -> tuple[Gf.Vec3d, Gf.Quatd]:
    """Read a world pose and convert it to `Gf` types.

    Args:
        target: Prim path, cached prim resolver, or backend-specific prim handle.

    Returns:
        The requested value.
    """
    return unpack_world_pose(*read_world_pose_arrays(target))
