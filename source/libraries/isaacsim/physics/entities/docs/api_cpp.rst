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

.. _isaacsim-physics-entities-api-cpp:

=========
C++ guide
=========

.. isaacsim-libraries-api-guide-start

All declarations live in the ``isaacsim::physics::entities`` namespace.

.. code-block:: cpp

    #include <isaacsim/physics/entities/PhysicsEntity.hpp>
    #include <isaacsim/physics/entities/RigidBodyEntity.hpp>
    #include <isaacsim/physics/entities/ArticulationEntity.hpp>

    using namespace isaacsim::physics::entities;

Every wrapper is constructed from a physics engine name and one or more USD
prim paths.  Paths may be a single string or a ``std::vector<std::string>``,
and may contain regular expressions.  Methods act on the whole batch; the
optional ``indices`` argument restricts them to a subset of the wrapped prims.

Batched quantities use ``N`` for the number of selected prims, ``D`` for the
number of degrees of freedom, ``L`` for the number of links, and ``T`` for the
number of fixed tendons.

.. note::

    The wrapped prims must be part of a started simulation for data access to
    succeed.

.. _isaacsim-physics-entities-api-cpp-physicsentity:

PhysicsEntity
=============

``PhysicsEntity`` is the base class.  It binds the prim paths to a physics
engine entity view and exposes named data buffers as batched arrays.  The set
of valid buffer names depends on the entity kind and the engine backing it:

.. code-block:: cpp

    PhysicsEntity entity("physx", "rigid-body", "/World/Cube.*");

    size_t count = entity.numPrims();
    array::Array transforms = entity.getData("transforms");
    entity.setData("transforms", transforms);

Include: ``<isaacsim/physics/entities/PhysicsEntity.hpp>``

.. _isaacsim-physics-entities-api-cpp-rigidbodyentity:

RigidBodyEntity
===============

``RigidBodyEntity`` extends ``PhysicsEntity`` with rigid body state and
properties: world poses, linear and angular velocities, external force and
torque application, masses, inertia tensors, centers of mass, and per-body
simulation and gravity toggles:

.. code-block:: cpp

    RigidBodyEntity bodies("physx", { "/World/Cube0", "/World/Cube1" });

    auto [positions, orientations] = bodies.getWorldPoses();
    bodies.setVelocities(/*linearVelocities=*/linear);
    bodies.applyForces(forces);
    bodies.setEnabledGravities(flags, /*indices=*/only0);

Setters that take several optional components require at least one of them to
be defined, and throw ``std::invalid_argument`` otherwise.

Include: ``<isaacsim/physics/entities/RigidBodyEntity.hpp>``

.. _isaacsim-physics-entities-api-cpp-articulationentity:

ArticulationEntity
==================

``ArticulationEntity`` extends ``PhysicsEntity`` with articulation state and
properties.  Topology properties (``numDofs``, ``dofNames``, ``numLinks``,
``linkNames``, ...) describe the kinematic tree, and the name-to-index helpers
resolve DOF, joint, and link names into the ``dofIndices`` / ``linkIndices``
arguments taken by the batched accessors:

.. code-block:: cpp

    ArticulationEntity robots("physx", "/World/Robot.*");

    array::Array arm = robots.getDofIndices({ "panda_joint1", "panda_joint2" });
    robots.setDofPositionTargets(targets, /*indices=*/std::nullopt, arm);
    robots.switchDofControlMode("effort", std::nullopt, arm);

The remaining methods cover root pose and velocity, DOF state and drive
configuration, link mass properties, dynamics quantities, and fixed tendon
properties.  ``jacobianMatrixShape`` and ``massMatrixShape`` report the
per-articulation shapes returned by ``getJacobianMatrices`` and
``getMassMatrices``; both depend on whether the articulation base is fixed.

.. note::

    DOF, joint, and link USD paths and types (``dofPaths``, ``dofTypes``,
    ``jointPaths``, ``jointTypes``, ``linkPaths``) are not exposed by the
    tensor API and throw ``std::runtime_error``.

Include: ``<isaacsim/physics/entities/ArticulationEntity.hpp>``
