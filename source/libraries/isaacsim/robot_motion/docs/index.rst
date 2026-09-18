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

.. _isaacsim-robot-motion-library:

=====================
isaacsim-robot-motion
=====================

The ``isaacsim-robot-motion`` Python distribution provides Kit-independent
robot motion APIs:

* ``isaacsim.robot_motion.controllers`` provides Ackermann, differential-drive,
  and holonomic controllers.
* ``isaacsim.robot_motion.experimental.motion_generation`` provides trajectory
  following, obstacle handling, scene queries, and motion-planning world bindings.
* ``isaacsim.robot_motion.cumotion`` provides cuMotion planners, trajectory tools,
  and packaged Franka and UR10 configurations.

Use this distribution to construct motion-generation workflows without requiring
an Isaac Sim application. APIs under the ``experimental`` namespace may evolve as
legacy motion-generation functionality moves into the standalone libraries.

The wheel remains platform-specific because it bundles the native cuMotion runtime.

The corresponding CMake package group is named ``isaacsim_robot_motion``.
