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

.. _isaacsim-robot-schema-library:

=====================
isaacsim-robot-schema
=====================

The ``isaacsim-robot-schema`` Python distribution provides Universal Scene
Description (USD) schemas for describing robots, links, joints, named poses,
sensors, and surface grippers. It also provides C++ and Python helpers for
inspecting and authoring the schema data.

Python consumers import the primary API from ``usd.schema.isaac``; the package
also preserves the legacy ``omni.isaac.IsaacSensorSchema`` and
``omni.isaac.RangeSensorSchema`` imports. C++ consumers include
``isaacsim/robot/schema/robot_schema.hpp``,
``isaacsim/robot/schema/sensor_tokens.hpp``, or
``isaacsim/robot/schema/utils.hpp``.

Apply these schemas when a USD asset needs Isaac Sim robot or sensor metadata. Use
the schema variant packaged for the OpenUSD version in your Python 3.12 target
environment.

The corresponding CMake package group is named ``isaacsim_robot_schema``.
