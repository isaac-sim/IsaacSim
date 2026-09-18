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

.. _isaacsim-ovsim-api-api-cpp:

=========
C++ guide
=========

.. isaacsim-libraries-api-guide-start

Link against ``isaacsim::ovsim-api``.

``#include <isaacsim/ovsim/api/Factory.hpp>``

Namespace: ``isaacsim::ovsim::api``

.. _isaacsim-ovsim-api-cpp-client-factory:

Client factory
==============

.. code-block:: cpp

    // name is "in-process" / "local", or "grpc".
    types::Implementation client = makeClient("in-process");

    // gRPC clients accept an optional configuration map (e.g. connection target).
    types::Implementation remote = makeClient("grpc", { { "endpoint", "127.0.0.1:50051" } });

.. _isaacsim-ovsim-api-cpp-implementation:

Implementation
==============

``#include <isaacsim/ovsim/api/Types.hpp>``

``types::Implementation`` bundles the ``control`` (``authoring`` + ``simulation``) and ``data``
interfaces returned by :cpp:func:`isaacsim::ovsim::api::makeClient`. Each member is a struct of ``std::function``
fields with the same signatures as the free functions in ``isaacsim::foundation::ovsim`` and
``isaacsim::physics::ovsim``.

.. code-block:: cpp

    types::Implementation client = makeClient("in-process");

    bool created = client.control.authoring.createStage();
    OutputValueType values = client.data.read(paths, "attributeName");
