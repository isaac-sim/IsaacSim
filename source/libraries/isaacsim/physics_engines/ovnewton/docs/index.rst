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

.. _isaacsim-physics-engines-ovnewton:

=================================
isaacsim.physics_engines.ovnewton
=================================

``isaacsim.physics_engines.ovnewton`` implements the physics manager and tensor
interfaces with Newton. Importing the module registers and activates the
backend; applications can then select ``"newton"`` through
``isaacsim.physics.manager.PhysicsManager``.

The backend requires the optional Newton runtime dependencies supplied by the
physics-engines distribution. See the Python API reference for explicit
configuration and lifecycle helpers.

.. toctree::
    :maxdepth: 2

    api_python
