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

.. _isaacsim-physics-engines-library:

========================
isaacsim-physics-engines
========================

``isaacsim-physics-engines`` supplies engine implementations for the
backend-neutral registration and manager APIs:

* ``isaacsim.physics_engines.ovphysx`` uses the OvPhysX runtime.
* ``isaacsim.physics_engines.ovnewton`` uses Newton and Warp.
* ``isaacsim.physics_engines.ovstage`` prepares OVStage's private runtime without modifying ``pxr``.

Activate or register an engine before selecting it through
``isaacsim.physics.manager``.

The corresponding CMake package group is named ``isaacsim_physics_engines``.
