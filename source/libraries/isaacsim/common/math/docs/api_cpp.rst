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

.. _isaacsim-common-math-api-cpp:

=========
C++ guide
=========

.. isaacsim-libraries-api-guide-start

Include ``isaacsim/common/math/Rotation.hpp`` and link ``isaacsim::common-math``.

All examples on this page assume the following namespace aliases:

.. code-block:: cpp

    #include <isaacsim/common/array/Array.hpp>
    #include <isaacsim/common/math/Rotation.hpp>

    namespace array = isaacsim::common::array;
    namespace math = isaacsim::common::math;

.. _isaacsim-common-math-api-cpp-quaternion-layout:

Quaternion layout
=================

A quaternion is four contiguous elements. Every function accepts a ``rotationFormat``
argument selecting their order: ``"xyzw"`` (scalar last, the default -- the order Warp
uses, the order the physics engine tensor buffers use, and the order the USD foundation
APIs report, so a batch crosses those boundaries without being reordered) or ``"wxyz"``
(scalar first).

Every function accepts an ``Array`` shaped ``(4,)`` for one quaternion or ``(N, 4)`` for
a batch, and ``(3,)`` or ``(N, 3)`` for the vectors a rotation applies to. A single
quaternion broadcasts against a batch of vectors, and a single vector against a batch of
quaternions. Results carry the operand's device and are ``float64``.

.. _isaacsim-common-math-api-cpp-quaternion-ops:

Quaternion operations
=====================

.. code-block:: cpp

    array::Array q(std::vector<double>{ 0.0, 0.0, 0.7071067811865476, 0.7071067811865475 });
    array::Array v(std::vector<double>{ 1.0, 0.0, 0.0 });

    math::normalizeQuaternion(q);          // unit length; a zero quaternion yields the identity
    math::conjugateQuaternion(q);          // negates the vector part
    math::inverseQuaternion(q);            // conjugate over the squared norm
    math::rotateByQuaternion(q, v);        // {0, 1, 0}: a quarter turn about Z takes +X onto +Y
    math::rotateByQuaternionInverse(q, v); // {0, -1, 0}: the same turn, undone
    math::dotQuaternions(q, q);            // 1.0, the squared norm of a unit quaternion

``multiplyQuaternions()`` composes two rotations, applying the right operand first:

.. code-block:: cpp

    math::multiplyQuaternions(body, mount);  // the mount offset, then the body rotation

``averageQuaternions()`` reduces a batch to one mean rotation, aligning signs before
summing because a quaternion and its negation describe the same rotation:

.. code-block:: cpp

    array::Array window(std::vector<std::vector<double>>{ { 0, 0, 0.7071067811865476, 0.7071067811865475 },
                                                            { 0, 0, -0.7071067811865476, -0.7071067811865475 } });
    math::averageQuaternions(window);  // the shared rotation, not the identity the raw sum would give

A quaternion ordered scalar-first is accepted the same way, by name:

.. code-block:: cpp

    array::Array qWxyz(std::vector<double>{ 0.7071067811865475, 0.0, 0.0, 0.7071067811865476 });
    math::rotateByQuaternion(qWxyz, v, "wxyz");  // {0, 1, 0}, the same rotation as q above

Element-wise arithmetic on quaternion batches is already ``add()`` and ``negative()`` in
``isaacsim/common/array/Functions.hpp``, so it is not repeated here.

.. _isaacsim-common-math-api-cpp-devices:

Devices
=======

An ``Array`` on a CUDA device is accepted, and the result comes back on that device.
Quaternion arithmetic mixes components across a small, fixed-size row (Hamilton products,
cross products, per-row norms), which the array element-wise and broadcast operations do
not express, so the elements are read on the host and the result is copied back. Batches
large enough for that round trip to matter want a device kernel instead.
