# SPDX-FileCopyrightText: Copyright (c) 2021-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

"""Provides enumeration of USD APIs supported by SceneQuery and WorldBinding for motion generation."""

from enum import StrEnum


class TrackableApi(StrEnum):
    """Enumerate USD APIs supported by SceneQuery and WorldBinding.

    Example:

    .. code-block:: python

        >>> from isaacsim.robot_motion.experimental.motion_generation import TrackableApi
        >>>
        >>> TrackableApi.PHYSICS_COLLISION.value
        'PhysicsCollisionAPI'
    """

    PHYSICS_COLLISION = "PhysicsCollisionAPI"
    """USD PhysicsCollisionAPI identifier."""
    PHYSICS_RIGID_BODY = "PhysicsRigidBodyAPI"
    """USD PhysicsRigidBodyAPI identifier."""
    MOTION_GENERATION_COLLISION = "IsaacMotionPlanningAPI"
    """IsaacMotionPlanningAPI identifier."""
