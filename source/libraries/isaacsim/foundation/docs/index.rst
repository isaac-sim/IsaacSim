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

.. _isaacsim-foundation-library:

===================
isaacsim-foundation
===================

The ``isaacsim-foundation`` distribution (CMake group ``isaacsim_foundation``) provides
the shared Universal Scene Description (USD) layer for standalone Isaac Sim libraries.
It includes OpenUSD and OVStage adapters, typed scene-object wrappers, physics prim
wrappers, and common stage, prim, and semantics utilities.

Use the adapter that matches the USD runtime owned by your application. Higher-level
libraries build on these modules instead of depending directly on runtime-specific
scene access throughout their implementations.
