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

.. _isaacsim-deprecated-library:

===================
isaacsim-deprecated
===================

The ``isaacsim-deprecated`` Python distribution contains compatibility APIs
retained while applications migrate from legacy ``isaacsim.core`` imports to
the standalone Isaac Sim libraries. It currently provides
``isaacsim.core.experimental.objects``,
``isaacsim.core.experimental.prims``, and
``isaacsim.core.experimental.utils`` from one platform-independent Python wheel.

Use this distribution only when existing code still depends on these legacy
imports. New code should use the corresponding APIs from the public standalone
library distributions instead.

The corresponding CMake package group is named ``isaacsim_deprecated``.
