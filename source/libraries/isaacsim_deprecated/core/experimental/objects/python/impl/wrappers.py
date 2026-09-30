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

"""Deprecated object wrappers that preserve the experimental scalar-first quaternion contract.

Each class derives from its ``isaacsim.foundation`` counterpart and differs from it only in the component
order of the pose methods: ``wxyz``, as the deprecated ``isaacsim.core.experimental.objects`` classes documented
through the ``XformPrim`` they inherited, rather than the foundation ``xyzw``.

Every class here derives from :class:`isaacsim.foundation.objects.Xform`, so every one of them has pose methods
to pin. ``Stage`` is the one name the deprecated module exports that does not, so it stays a plain alias in the
package ``__init__``.
"""

from __future__ import annotations

from isaacsim.core.experimental.prims.impl._scalar_first_poses import preserve_scalar_first_poses
from isaacsim.core.experimental.prims.impl._transform_arguments import preserve_transform_arguments
from isaacsim.foundation import objects as _objects
from isaacsim.foundation import prims as _prims

__all__ = [
    "Camera",
    "Capsule",
    "Cone",
    "Cube",
    "Cylinder",
    "CylinderLight",
    "DiskLight",
    "DistantLight",
    "DomeLight",
    "GroundPlane",
    "Light",
    "Mesh",
    "Plane",
    "RectLight",
    "Shape",
    "Sphere",
    "SphereLight",
]


@preserve_transform_arguments
@preserve_scalar_first_poses
class Camera(_objects.Camera):
    """Deprecated camera wrapper whose pose methods order quaternions ``wxyz``."""


@preserve_transform_arguments
@preserve_scalar_first_poses
class Capsule(_objects.Capsule):
    """Deprecated capsule wrapper whose pose methods order quaternions ``wxyz``."""


@preserve_transform_arguments
@preserve_scalar_first_poses
class Cone(_objects.Cone):
    """Deprecated cone wrapper whose pose methods order quaternions ``wxyz``."""


@preserve_transform_arguments
@preserve_scalar_first_poses
class Cube(_objects.Cube):
    """Deprecated cube wrapper whose pose methods order quaternions ``wxyz``."""


@preserve_transform_arguments
@preserve_scalar_first_poses
class Cylinder(_objects.Cylinder):
    """Deprecated cylinder wrapper whose pose methods order quaternions ``wxyz``."""


@preserve_transform_arguments
@preserve_scalar_first_poses
class CylinderLight(_objects.CylinderLight):
    """Deprecated cylinder light wrapper whose pose methods order quaternions ``wxyz``."""


@preserve_transform_arguments
@preserve_scalar_first_poses
class DiskLight(_objects.DiskLight):
    """Deprecated disk light wrapper whose pose methods order quaternions ``wxyz``."""


@preserve_transform_arguments
@preserve_scalar_first_poses
class DistantLight(_objects.DistantLight):
    """Deprecated distant light wrapper whose pose methods order quaternions ``wxyz``."""


@preserve_transform_arguments
@preserve_scalar_first_poses
class DomeLight(_objects.DomeLight):
    """Deprecated dome light wrapper whose pose methods order quaternions ``wxyz``."""


@preserve_transform_arguments
@preserve_scalar_first_poses
class GroundPlane(_prims.GroundPlane):
    """Deprecated ground plane wrapper whose pose methods order quaternions ``wxyz``."""


@preserve_transform_arguments
@preserve_scalar_first_poses
class Light(_objects.Light):
    """Deprecated light base wrapper whose pose methods order quaternions ``wxyz``."""


@preserve_transform_arguments
@preserve_scalar_first_poses
class Mesh(_objects.Mesh):
    """Deprecated mesh wrapper whose pose methods order quaternions ``wxyz``."""


@preserve_transform_arguments
@preserve_scalar_first_poses
class Plane(_objects.Plane):
    """Deprecated plane wrapper whose pose methods order quaternions ``wxyz``."""


@preserve_transform_arguments
@preserve_scalar_first_poses
class RectLight(_objects.RectLight):
    """Deprecated rect light wrapper whose pose methods order quaternions ``wxyz``."""


@preserve_transform_arguments
@preserve_scalar_first_poses
class Shape(_objects.Shape):
    """Deprecated shape base wrapper whose pose methods order quaternions ``wxyz``."""


@preserve_transform_arguments
@preserve_scalar_first_poses
class Sphere(_objects.Sphere):
    """Deprecated sphere wrapper whose pose methods order quaternions ``wxyz``."""


@preserve_transform_arguments
@preserve_scalar_first_poses
class SphereLight(_objects.SphereLight):
    """Deprecated sphere light wrapper whose pose methods order quaternions ``wxyz``."""
