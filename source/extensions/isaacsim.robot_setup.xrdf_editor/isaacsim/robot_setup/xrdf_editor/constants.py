# SPDX-FileCopyrightText: Copyright (c) 2022-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
# http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Shared constants for the XRDF editor package."""

from __future__ import annotations

EXTENSION_NAME = "cuMotion/Lula Robot Description Editor"

DEFAULT_JERK_LIMIT = 10000
DEFAULT_ACCELERATION_LIMIT = 10

# XRDF format constants.
XRDF_FORMAT = "xrdf"
XRDF_VERSION_1 = 1.0
XRDF_VERSION_2 = 2.0
SUPPORTED_XRDF_VERSIONS = (XRDF_VERSION_1, XRDF_VERSION_2)

# Per-version key under which world-collision geometry is stored.
COLLISION_KEY_V1 = "collision"
COLLISION_KEY_V2 = "world_collision"

# Default name for an auto-generated collision sphere group when no merge
# source supplies one.
DEFAULT_GEOMETRY_GROUP_NAME = "auto_generated_collision_sphere_group"

# Operation type tags used by CollisionSphereEditor's undo/redo stacks.
OP_ADD = "ADD"
OP_DEL = "DEL"
OP_SCALE = "SCALE"
