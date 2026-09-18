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

.. _isaacsim-ovgl-viewport-library:

=======================
isaacsim-ovgl-viewport
=======================

The ``isaacsim-ovgl-viewport`` distribution (CMake group ``isaacsim_ovgl_viewport``) provides an interactive OpenGL
viewport for testing and debugging standalone Isaac Sim applications. The ``isaacsim.ovgl_viewport.debug`` module
renders an OVStage scene and provides camera navigation, headless frame rendering, and frame capture. Its Python
interface can author the standard render target on a Foundation stage and manages interactive camera updates and
viewport lifetime.

Use this library when you need to inspect standalone scene or simulation state. It is a concrete debugging tool, not a
generic rendering abstraction.
