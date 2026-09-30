# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

"""Pose-method overrides that keep the deprecated scalar-first quaternion contract.

The Kit-independent :class:`isaacsim.foundation.objects.Xform` orders quaternions ``xyzw``, matching the physics
engine tensor buffers, and takes a ``rotation_format`` argument to select the order. The deprecated
``isaacsim.core.experimental`` wrappers it replaces documented ``wxyz`` on every pose method and had no such
argument, so this module pins them to ``wxyz`` and hides the argument.

The overrides are applied by a decorator rather than inherited from a mixin because nanobind-bound types accept
exactly one base class, so a wrapper cannot inherit from both a mixin and its foundation base.
"""

from __future__ import annotations

from typing import Any, TypeVar

__all__ = ["preserve_scalar_first_poses"]

_ROTATION_FORMAT = "wxyz"

_T = TypeVar("_T", bound=type)


def preserve_scalar_first_poses(cls: _T) -> _T:
    """Override a wrapper's pose methods to read and write ``wxyz`` quaternions.

    Apply this to a class that derives, directly or indirectly, from
    :class:`isaacsim.foundation.objects.Xform`. Apply it exactly once per hierarchy: decorating a class whose
    base is already decorated makes the override delegate to an override that no longer accepts a rotation
    format.

    Args:
        cls: Deprecated wrapper class to modify in place.

    Returns:
        The same class, with its four pose methods replaced.
    """

    def get_world_poses(self: Any, *, indices: Any = None) -> Any:  # noqa: D417
        """Get the world-frame poses.

        Args:
            indices: Indices of the prims to process. If ``None``, all wrapped prims are processed.

        Returns:
            The positions (shape ``(N, 3)``) and the orientations (shape ``(N, 4)``, quaternion ``wxyz``).
        """  # noqa: DOC101, DOC103
        return super(cls, self).get_world_poses(indices=indices, rotation_format=_ROTATION_FORMAT)

    def get_local_poses(self: Any, *, indices: Any = None) -> Any:  # noqa: D417
        """Get the local-frame poses.

        Args:
            indices: Indices of the prims to process. If ``None``, all wrapped prims are processed.

        Returns:
            The translations (shape ``(N, 3)``) and the orientations (shape ``(N, 4)``, quaternion ``wxyz``).
        """  # noqa: DOC101, DOC103
        return super(cls, self).get_local_poses(indices=indices, rotation_format=_ROTATION_FORMAT)

    def set_world_poses(  # noqa: D417
        self: Any, positions: Any = None, orientations: Any = None, *, indices: Any = None
    ) -> None:
        """Set the world-frame poses.

        Args:
            positions: Positions in the world frame (shape ``(N, 3)``).
            orientations: Orientations in the world frame (shape ``(N, 4)``, quaternion ``wxyz``).
            indices: Indices of the prims to process. If ``None``, all wrapped prims are processed.
        """  # noqa: DOC101, DOC103
        super(cls, self).set_world_poses(positions, orientations, indices=indices, rotation_format=_ROTATION_FORMAT)

    def set_local_poses(  # noqa: D417
        self: Any, translations: Any = None, orientations: Any = None, *, indices: Any = None
    ) -> None:
        """Set the local-frame poses.

        Args:
            translations: Translations in the local frame (shape ``(N, 3)``).
            orientations: Orientations in the local frame (shape ``(N, 4)``, quaternion ``wxyz``).
            indices: Indices of the prims to process. If ``None``, all wrapped prims are processed.
        """  # noqa: DOC101, DOC103
        super(cls, self).set_local_poses(translations, orientations, indices=indices, rotation_format=_ROTATION_FORMAT)

    cls.get_world_poses = get_world_poses
    cls.get_local_poses = get_local_poses
    cls.set_world_poses = set_world_poses
    cls.set_local_poses = set_local_poses
    return cls
