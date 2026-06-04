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

"""Cortex rigid prim wrapper providing access to RigidBodyAPI physics properties."""

from __future__ import annotations

from collections.abc import Sequence

import numpy as np
from isaacsim.core.prims import SingleXFormPrim
from pxr import Gf, UsdPhysics


class CortexRigidPrim(SingleXFormPrim):
    """A simple API access to the RigidBodyAPI USD schema of an object.

    Args:
        *args: Positional arguments passed to SingleXFormPrim.
        **kwargs: Keyword arguments passed to SingleXFormPrim.
    """

    def __init__(self, *args: object, **kwargs: object) -> None:
        super().__init__(*args, **kwargs)
        if not self.prim.HasAPI(UsdPhysics.RigidBodyAPI):
            raise RuntimeError("Prim does not have the UsdPhysics.RigidBodyAPI schema.")
        self.rigid_api = UsdPhysics.RigidBodyAPI(self.prim)

    def enable_rigid_body_physics(self) -> None:
        """Enable physics on this object."""
        self.rigid_api.GetRigidBodyEnabledAttr().Set(True)

    def disable_rigid_body_physics(self) -> None:
        """Disable physics on this object."""
        self.rigid_api.GetRigidBodyEnabledAttr().Set(False)

    def get_linear_velocity(self) -> np.ndarray:
        """Retrieve the linear velocity of this object.

        Returns:
            Linear velocity as a 3d vector.
        """
        gf_velocity = self.rigid_api.GetVelocityAttr().Get()
        return np.array([gf_velocity[0], gf_velocity[1], gf_velocity[2]])

    def set_linear_velocity(self, velocity: Sequence[float]) -> None:
        """Set the linear velocity of this object.

        Args:
            velocity: The 3d linear velocity to set the linear velocity to.
        """
        gf_velocity = Gf.Vec3d(velocity[0], velocity[1], velocity[2])
        self.rigid_api.GetVelocityAttr().Set(gf_velocity)

    def get_angular_velocity(self) -> np.ndarray:
        """Retrieve the angular velocity of this object.

        Returns:
            Angular velocity as a 3d vector.
        """
        gf_ang_vel = self.rigid_api.GetAngularVelocityAttr().Get()
        return np.array([gf_ang_vel[0], gf_ang_vel[1], gf_ang_vel[2]])

    def set_angular_velocity(self, ang_vel: Sequence[float]) -> None:
        """Set the angular velocity of this object.

        Args:
            ang_vel: The 3d angular velocity vector to set the angular velocity to.
        """
        gf_ang_vel = Gf.Vec3d(ang_vel[0], ang_vel[1], ang_vel[2])
        self.rigid_api.GetAngularVelocityAttr().Set(gf_ang_vel)
