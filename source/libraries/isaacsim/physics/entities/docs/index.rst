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

.. _isaacsim-physics-entities:
.. _isaacsim-physics-entities-overview:

=========================
isaacsim.physics.entities
=========================

``isaacsim.physics.entities`` provides object-oriented wrappers around batches
of simulated physics prims, exposing their state and properties as batched
arrays through the physics tensor API.

Each wrapper binds a set of USD prim paths to a physics engine entity view.
Methods operate on the whole batch at once; an optional ``indices`` argument
restricts them to a subset of the wrapped prims.

.. _isaacsim-physics-entities-physics-entity:

Physics entity
--------------

:py:class:`~isaacsim.physics.entities.PhysicsEntity` is the base class. It resolves the prim paths against a
named physics engine and exposes named data buffers through generic
``get_data`` / ``set_data`` accessors.

.. _isaacsim-physics-entities-rigid-body:

Rigid body
----------

:py:class:`~isaacsim.physics.entities.RigidBodyEntity` extends it with rigid body state and properties: world
poses, linear and angular velocities, external force and torque application,
masses, inertia tensors, centers of mass, and per-body simulation and gravity
toggles.

.. _isaacsim-physics-entities-articulation:

Articulation
------------

:py:class:`~isaacsim.physics.entities.ArticulationEntity` extends it with articulation state and properties:
kinematic tree topology (links, joints, degrees of freedom), root pose and
velocity, DOF state and drive configuration, link mass properties, dynamics
quantities (Jacobian and mass matrices, compensation forces), and fixed tendon
properties.

Batched quantities use ``N`` for the number of selected prims, ``D`` for the
number of degrees of freedom, ``L`` for the number of links, and ``T`` for the
number of fixed tendons.

.. toctree::
    :maxdepth: 2

    api_cpp
    api_python
