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

.. _isaacsim-foundation-prims:
.. _isaacsim-foundation-prims-overview:

=========================
isaacsim.foundation.prims
=========================

``isaacsim.foundation.prims`` provides physics-aware prim wrappers built on top
of the base classes in ``isaacsim.foundation.objects``.  All classes in this
module extend :py:class:`~isaacsim.foundation.objects.Xform` and operate on one
or more USD prims resolved from the active stage.

.. _isaacsim-foundation-prims-physics-prims:

Physics prims
-------------

:py:class:`~isaacsim.foundation.prims.Articulation` wraps prims that carry
the ``PhysicsArticulationRootAPI`` and provides a unified interface for
articulation structure (links, joints, DOFs) and their physical properties
(gains, limits, friction, drive types, targets).

:py:class:`~isaacsim.foundation.prims.ColliderBody` applies the
``PhysicsCollisionAPI`` and exposes collision approximation, contact/rest
offsets, torsional patch radii, and enable/disable controls.

:py:class:`~isaacsim.foundation.prims.RigidBody` applies the
``PhysicsRigidBodyAPI`` and ``PhysicsMassAPI`` schemas and exposes mass,
density, linear/angular velocity, gravity, and sleep properties.

:py:class:`~isaacsim.foundation.prims.GroundPlane` wraps a composite prim
made of a ``Plane`` (collision) and a ``Mesh`` (rendering), creating or
wrapping them as needed, and exposes collision offsets, torsional patch
radii, and enable/disable controls.

.. toctree::
    :maxdepth: 2

    api_cpp
    api_python
