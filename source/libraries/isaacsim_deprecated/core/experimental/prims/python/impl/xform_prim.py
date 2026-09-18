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

"""Deprecated ``XformPrim`` wrapper that preserves experimental construction and quaternion semantics."""

from __future__ import annotations

from isaacsim.foundation.objects import Xform

from ._scalar_first_poses import preserve_scalar_first_poses
from ._transform_arguments import preserve_transform_arguments

__all__ = ["XformPrim"]


@preserve_transform_arguments
@preserve_scalar_first_poses
class XformPrim(Xform):
    """Deprecated transform wrapper with non-destructive construction semantics.

    Quaternions are ordered ``wxyz`` on every pose method, as the deprecated experimental wrapper documented,
    rather than the ``xyzw`` of :class:`isaacsim.foundation.objects.Xform`.

    Args:
        paths: Prim path or paths to wrap.
        resolve_paths: Whether to resolve path expressions.
        positions: Positions in the world frame. Mutually exclusive with ``translations``.
        translations: Translations in the local frame. Mutually exclusive with ``positions``.
        orientations: Orientations in the world frame (shape ``(N, 4)``, quaternion ``wxyz``).
        scales: Scales to apply to the prims.
        reset_xform_op_properties: Whether to replace existing transform operations with the standard operation set.

    Raises:
        ValueError: If both ``positions`` and ``translations`` are specified.
    """

    def __init__(
        self,
        paths: str | list[str],
        *,
        resolve_paths: bool = True,
        reset_xform_op_properties: bool = False,
    ) -> None:
        """Initialize the underlying Foundation wrapper.

        The placement arguments documented on the class are injected by
        :func:`preserve_transform_arguments` before this implementation runs.

        Args:
            paths: Prim path or paths to wrap.
            resolve_paths: Whether to resolve path expressions.
            reset_xform_op_properties: Whether to replace existing transform operations with the standard set.
        """  # noqa: DOC102, DOC103, DOC301
        # The placement arguments are added by the decorator, which forwards the rest here.
        super().__init__(
            paths,
            resolve_paths=resolve_paths,
            reset_xform_op_properties=reset_xform_op_properties,
        )
