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

"""Deprecated compatibility alias for the object primitives.

.. deprecated::
    This module preserves the ``isaacsim.core.experimental.objects`` import path for existing code.
    New code should import the object primitives from :mod:`isaacsim.foundation.objects` instead.

The exported classes are thin compatibility subclasses that keep the experimental ``wxyz`` quaternion order on
their pose methods; the foundation classes they derive from order quaternions ``xyzw``.
"""

# Bind the bundled Kit-free OpenUSD into the process before importing anything that uses ``pxr``.
import isaacsim.foundation.usd.openusd  # noqa: F401

# ``Stage`` is not transformable, so it has no pose methods to pin and is aliased unchanged.
from isaacsim.foundation.objects import Stage

from .wrappers import (
    Camera,
    Capsule,
    Cone,
    Cube,
    Cylinder,
    CylinderLight,
    DiskLight,
    DistantLight,
    DomeLight,
    GroundPlane,
    Light,
    Mesh,
    Plane,
    RectLight,
    Shape,
    Sphere,
    SphereLight,
)

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
    "Stage",
]
