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

"""Deprecated ``GeomPrim`` wrapper that preserves the experimental construction semantics.

The Kit-independent :class:`isaacsim.foundation.prims.ColliderBody` applies the USD/PhysX collision APIs on
construction by default (``apply_collision_apis=True``). The deprecated
``isaacsim.core.experimental.prims.GeomPrim`` it replaces defaulted to ``apply_collision_apis=False`` - it wrapped
*existing* prims read-only and did not author collision schemas. This subclass restores that default so existing
code (for example motion generation, which only reads
:meth:`isaacsim.foundation.prims.ColliderBody.get_enabled_collisions`) keeps working without
authoring collision APIs, and without requiring the PhysX USD schemas to be registered in a Kit-free runtime.
"""

from __future__ import annotations

from typing import Any

from isaacsim.foundation.prims import ColliderBody

from ._scalar_first_poses import preserve_scalar_first_poses
from ._transform_arguments import preserve_transform_arguments

__all__ = ["GeomPrim"]


@preserve_transform_arguments
@preserve_scalar_first_poses
class GeomPrim(ColliderBody):
    """Deprecated collider wrapper with non-destructive construction semantics.

    Quaternions are ordered ``wxyz`` on every pose method, as the deprecated experimental wrapper documented,
    rather than the ``xyzw`` of :class:`isaacsim.foundation.objects.Xform`.

    All other behavior, including
    :meth:`isaacsim.foundation.prims.ColliderBody.get_enabled_collisions`, is inherited from
    :class:`isaacsim.foundation.prims.ColliderBody`.

    Args:
        paths: Prim path or paths to wrap.
        apply_collision_apis: Whether to apply collision APIs during construction.
        reset_xform_op_properties: Whether to replace existing transform operations with the standard operation set.
        **kwargs: Additional keyword arguments accepted by :class:`isaacsim.foundation.prims.ColliderBody`.
    """

    def __init__(
        self,
        paths: str | list[str],
        *,
        apply_collision_apis: bool = False,
        reset_xform_op_properties: bool = False,
        **kwargs: Any,
    ) -> None:
        super().__init__(
            paths,
            apply_collision_apis=apply_collision_apis,
            reset_xform_op_properties=reset_xform_op_properties,
            **kwargs,
        )
