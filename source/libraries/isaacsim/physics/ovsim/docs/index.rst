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

.. _isaacsim-physics-ovsim:
.. _isaacsim-physics-ovsim-overview:

========================
Isaac Sim physics OV SIM
========================

``isaacsim.physics.ovsim`` implements OV SIM data access and manual simulation
control on top of the ``isaacsim-physics`` distribution. Select a registered
physics engine and supply an OVStage native handle before initializing manual
simulation.

Stage authoring operations are not available from this physics-only adapter and
raise ``RuntimeError``. Use an OV SIM client with stage-authoring support
when you need to create, open, save, or edit a stage.

.. toctree::
    :maxdepth: 2

    api_cpp
    api_python
