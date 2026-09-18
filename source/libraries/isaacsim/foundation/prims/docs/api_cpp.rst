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

.. _isaacsim-foundation-prims-api-cpp:

=========
C++ guide
=========

.. isaacsim-libraries-api-guide-start

All declarations live in the ``isaacsim::foundation::prims::physics`` namespace.

.. code-block:: cpp

    #include <isaacsim/foundation/prims/physics/Articulation.hpp>
    #include <isaacsim/foundation/prims/physics/ColliderBody.hpp>
    #include <isaacsim/foundation/prims/physics/GroundPlane.hpp>
    #include <isaacsim/foundation/prims/physics/RigidBody.hpp>

    using namespace isaacsim::foundation::prims::physics;

.. _isaacsim-foundation-prims-api-cpp-articulation:

Articulation
============

``Articulation`` extends ``Xform`` and wraps prims that carry the
``PhysicsArticulationRootAPI``.  Structural metadata (DOF/joint/link names,
counts, paths, and types) is parsed once at construction and cached:

.. code-block:: cpp

    Articulation robot("/World/Robot");

    // Inspect structure
    int   dofs  = robot.numDofs();
    auto  names = robot.dofNames();

    // Drive gains and targets
    robot.setDofGains(/*stiffnesses=*/Array{800.f}, /*dampings=*/Array{40.f});
    robot.setDofPositionTargets(Array{0.f, 1.57f, 0.f});

.. _isaacsim-foundation-prims-api-cpp-colliderbody:

ColliderBody
============

``ColliderBody`` extends ``Xform`` and applies the ``PhysicsCollisionAPI``.
The collision mesh approximation can be set at construction or updated later:

.. code-block:: cpp

    ColliderBody mesh("/World/Mesh", /*approximations=*/"convexHull");
    mesh.setCollisionApproximations("convexDecomposition");
    auto [contactOffsets, restOffsets] = mesh.getOffsets();

.. _isaacsim-foundation-prims-api-cpp-groundplane:

GroundPlane
===========

``GroundPlane`` extends ``Xform`` and wraps a composite prim made of a
``Plane`` (collision) and a ``Mesh`` (rendering) child. It creates the prims
if they do not exist, or wraps them if they do:

.. code-block:: cpp

    GroundPlane ground("/World/GroundPlane", /*sizes=*/Array{100.f});
    ground.setOffsets(/*contactOffsets=*/Array{0.02f});
    ground.setEnabledCollisions(Array{true});

.. _isaacsim-foundation-prims-api-cpp-rigidbody:

RigidBody
=========

``RigidBody`` extends ``Xform`` and applies the ``PhysicsRigidBodyAPI`` and
``PhysicsMassAPI`` schemas to one or more prims.  Optional constructor
parameters set initial mass, density, and transform in one call:

.. code-block:: cpp

    RigidBody box("/World/Box", /*masses=*/Array{1.5f});
    box.setVelocities(/*linear=*/Array{{0.f, 0.f, 1.f}});
    auto [linear, angular] = box.getVelocities();
