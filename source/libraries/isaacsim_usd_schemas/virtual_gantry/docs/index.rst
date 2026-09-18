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

.. _isaacsim-virtual-gantry-schema-library:

==============================
isaacsim-virtual-gantry-schema
==============================

The ``isaacsim-virtual-gantry-schema`` Python distribution provides the
Universal Scene Description (USD) schema for a virtual gantry. The
``IsaacVirtualGantry`` typed prim uses its world transform as the anchor for a
one-sided spring-damper rope attached to an articulation link.

Use this distribution to author and inspect virtual-gantry attributes and
relationships without adding gantry-specific concepts to the general robot
schema. The package supplies the schema resources and Python accessors through
``isaacsim.robot_setup.virtual_gantry_schema`` for the supported OpenUSD versions
on Python 3.12.

The corresponding CMake package group is named ``isaacsim_virtual_gantry_schema``.
