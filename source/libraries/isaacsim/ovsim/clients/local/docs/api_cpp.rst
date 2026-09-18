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

.. _isaacsim-ovsim-clients-local-api-cpp:

=========
C++ guide
=========

.. isaacsim-libraries-api-guide-start

Link ``isaacsim::ovsim-clients-local`` and include the public headers under
``isaacsim/ovsim/clients/local``:

- ``control/authoring/Authoring.hpp`` provides stage and prim authoring operations.
- ``control/simulation/Simulation.hpp`` provides simulation lifecycle and parameter operations.
- ``data/Data.hpp`` provides attribute read and write operations.

The local client forwards these calls to the in-process Foundation and Physics OV SIM implementations. The declarations
and Doxygen comments in the public headers are the authoritative C++ reference.
