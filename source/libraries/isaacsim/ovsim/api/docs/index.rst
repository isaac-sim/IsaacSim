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

.. _isaacsim-ovsim-api:
.. _isaacsim-ovsim-api-overview:

=====================================
Isaac Sim OV SIM API
=====================================

``isaacsim.ovsim.api`` builds an OV SIM client from a client name: ``"in-process"`` or ``"local"`` for the
in-process implementation, or ``"grpc"`` for a remote session over gRPC. Both clients expose the same ``authoring``,
``simulation``, and ``data`` interfaces, so calling code does not need to know which client it uses.

.. toctree::
    :maxdepth: 2

    api_cpp
    api_python
