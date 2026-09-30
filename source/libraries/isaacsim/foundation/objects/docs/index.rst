..
   SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
   SPDX-License-Identifier: Apache-2.0

   Licensed under the Apache License, Version 2.0 (the "License");
   you may not use this file except in compliance with the License.
   You may obtain a copy of the License at

   http://www.apache.org/licenses/LICENSE-2.0

   Unless required by applicable law or agreed to in writing, software
   distributed under the License is distributed on an "AS IS" BASIS,
   WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
   See the License for the specific language governing permissions and
   limitations under the License.

.. _isaacsim-foundation-objects:
.. _isaacsim-foundation-objects-overview:

===========================
isaacsim.foundation.objects
===========================

``isaacsim.foundation.objects`` provides object-oriented wrappers around
common USD scene elements used across Isaac Sim.

.. _isaacsim-foundation-objects-stage-and-prims:

Stage and prims
---------------

:py:class:`~isaacsim.foundation.objects.Stage` manages a USD stage lifecycle (open, save, close) and exposes
its stage ID for interop with lower-level USD APIs. A backend string
(``"openusd"`` or ``"ovstage"``) selects the underlying USD-compatible runtime.

:py:class:`~isaacsim.foundation.objects.Prim` wraps one or more USD prim paths on the default stage.

.. _isaacsim-foundation-objects-xform:

Xform
-----

:py:class:`~isaacsim.foundation.objects.Xform` extends it with transform read/write helpers.

.. _isaacsim-foundation-objects-camera:

Camera
------

:py:class:`~isaacsim.foundation.objects.Camera` wraps a ``UsdGeom.Camera`` prim and provides helpers for
reading and writing intrinsic parameters.

.. _isaacsim-foundation-objects-mesh:

Mesh
----

:py:class:`~isaacsim.foundation.objects.Mesh` wraps a ``UsdGeom.Mesh`` prim and exposes its points, normals,
and face, crease, corner and subdivision specifications. It can also generate
the geometry of a set of built-in primitives (cone, cube, cylinder, disk,
plane, sphere and torus) when creating new prims.

.. _isaacsim-foundation-objects-shapes:

Shapes
------

Concrete geometry classes (:py:class:`~isaacsim.foundation.objects.Capsule`,
:py:class:`~isaacsim.foundation.objects.Cone`, :py:class:`~isaacsim.foundation.objects.Cube`,
:py:class:`~isaacsim.foundation.objects.Cylinder`, :py:class:`~isaacsim.foundation.objects.Plane`, and
:py:class:`~isaacsim.foundation.objects.Sphere`) inherit from the abstract
:py:class:`~isaacsim.foundation.objects.Shape` base and handle USD
mesh authoring for each geometry type.

.. _isaacsim-foundation-objects-lights:

Lights
------

Light classes (:py:class:`~isaacsim.foundation.objects.CylinderLight`,
:py:class:`~isaacsim.foundation.objects.DiskLight`, :py:class:`~isaacsim.foundation.objects.DistantLight`,
:py:class:`~isaacsim.foundation.objects.DomeLight`, :py:class:`~isaacsim.foundation.objects.RectLight`, and
:py:class:`~isaacsim.foundation.objects.SphereLight`) inherit from
:py:class:`~isaacsim.foundation.objects.Light` and wrap the corresponding
UsdLux light prims.

.. toctree::
    :maxdepth: 2

    api_cpp
    api_python
