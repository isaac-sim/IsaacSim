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

.. _isaacsim-robot-motion-schema-library:

============================
isaacsim-robot-motion-schema
============================

The ``isaacsim-robot-motion-schema`` Python distribution provides the Universal
Scene Description (USD) schema used to annotate robot motion-planning data. Its
applied API schema records whether collision checking is enabled for motion
planning on a prim.

Use this distribution when motion-planning metadata must travel with a USD asset. The
package supplies the schema resources and Python accessors through
``isaacsim.robot_motion.schema`` for the supported OpenUSD versions on Python 3.12.

The corresponding CMake package group is named ``isaacsim_robot_motion_schema``.
