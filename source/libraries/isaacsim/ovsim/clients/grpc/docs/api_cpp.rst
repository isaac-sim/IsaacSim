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

.. _isaacsim-ovsim-clients-grpc-api-cpp:

=========
C++ guide
=========

.. isaacsim-libraries-api-guide-start

Include ``isaacsim/ovsim/clients/grpc/GrpcSession.hpp`` and link
``isaacsim::ovsim-clients-grpc``.

``GrpcSession`` exposes the OV SIM authoring, simulation, and data interfaces through one session object. Construct it
with a configuration map containing a non-empty ``endpoint`` channel target, such as ``127.0.0.1:50051``.

The client supports these remote operations:

- ``importStageFromString`` and ``closeStage`` to create and delete a hosted simulation.
- ``initialize`` to wait for the hosted simulation to become ready.
- ``step`` for drift-free manual 60 Hz stepping.
- ``read`` for latest-value reads using the protocol value codec.
- ``write`` for immediate CPU float32 or float64 position arrays.

Other authoring and automatic-playback methods currently throw ``std::logic_error``. Explicit or historical
timestamps are not supported. Consult each method's Doxygen contract for its validation and transport errors.

.. code-block:: cpp

    using isaacsim::ovsim::clients::grpc::GrpcSession;

    GrpcSession client({ { "endpoint", "127.0.0.1:50051" } });
    client.importStageFromString(usdContent);
    client.initialize();
    client.step();

The declarations and Doxygen comments in the public header are the authoritative C++ reference.
