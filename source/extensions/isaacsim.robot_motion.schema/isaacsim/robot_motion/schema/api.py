# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

"""Utilities for applying the Isaac motion planning schema API."""

from __future__ import annotations

from pxr import Sdf, Usd

MOTION_PLANNING_API_NAME = "IsaacMotionPlanningAPI"
MOTION_PLANNING_ENABLED_ATTR = "isaac:motionPlanning:collisionEnabled"


def apply_motion_planning_api(prim: Usd.Prim, enabled: bool | None = None) -> Usd.Prim:
    """Apply the IsaacMotionPlanningAPI to a prim.

    Args:
        prim: Prim to apply the API to.
        enabled: Optional value to author on the collision-enabled attribute.

    Returns:
        The prim with the API applied.

    Raises:
        ValueError: If prim is invalid.

    Example:

    .. code-block:: python

        >>> from isaacsim.robot_motion.schema import apply_motion_planning_api
        >>> stage = omni.usd.get_context().get_stage()
        >>> prim = stage.GetPrimAtPath("/World/Robot")
        >>> apply_motion_planning_api(prim, enabled=True)
    """
    if not prim or not prim.IsValid():
        raise ValueError("Prim is invalid.")

    prim.AddAppliedSchema(MOTION_PLANNING_API_NAME)
    attribute = prim.GetAttribute(MOTION_PLANNING_ENABLED_ATTR)
    if not attribute:
        attribute = prim.CreateAttribute(MOTION_PLANNING_ENABLED_ATTR, Sdf.ValueTypeNames.Bool, True)
    if enabled is None:
        attribute.Set(True)
    else:
        attribute.Set(bool(enabled))
    return prim
