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

.. _isaacsim-ovsim-api-api-python:

==============================
Python guide and API reference
==============================

.. currentmodule:: isaacsim.ovsim.api

.. autofunction:: make_client(name, configuration=None)

``make_client`` returns an ``Implementation`` object exposing ``control.authoring``,
``control.simulation``, and ``data``, each with the same methods as the corresponding module in
:ref:`isaacsim.foundation.ovsim <isaacsim-foundation-ovsim-api-python>` /
:ref:`isaacsim.physics.ovsim <isaacsim-physics-ovsim-api-python>`.

.. code-block:: python

    from isaacsim.ovsim.api import make_client

    # name is "in-process" / "local", or "grpc".
    client = make_client("in-process")

    # gRPC clients require an explicit standard channel target.
    remote = make_client("grpc", {"endpoint": "127.0.0.1:50051"})

    client.control.authoring.create_stage()
    values = client.data.read(paths, "attributeName")
