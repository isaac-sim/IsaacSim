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

"""Utility functions for creating surface gripper prims in Isaac Sim."""

from __future__ import annotations

__all__ = ["create_surface_gripper"]

import omni.usd
from pxr import Usd
from usd.schema.isaac import robot_schema


def create_surface_gripper(
    stage: Usd.Stage,
    prim_path: str,
) -> Usd.Prim:
    """Create a Surface Gripper prim at the given path.

    Generates a unique child path by appending ``/SurfaceGripper`` to
    *prim_path* (incrementing to ``SurfaceGripper_01``, etc. if needed)
    and then creates the gripper prim via the robot schema API.

    Args:
        stage: The USD stage.
        prim_path: Parent path under which the gripper prim will be created.

    Returns:
        The created Surface Gripper prim.
    """
    gripper_path = omni.usd.get_stage_next_free_path(stage, prim_path + "/SurfaceGripper", False)
    return robot_schema.CreateSurfaceGripper(stage, gripper_path)
