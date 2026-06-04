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

"""Holonomic robot USD setup using core.experimental.utils."""

from __future__ import annotations

import numpy as np
import omni.usd
from isaacsim.core.experimental.utils import stage as stage_utils
from pxr import Gf, Usd, UsdGeom, UsdPhysics


class HolonomicRobotUsdSetup:
    """Read holonomic (mecanum) robot parameters from USD for use with HolonomicController.

    Args:
        robot_prim_path: USD path to the robot articulation root prim.
        com_prim_path: USD path to the center-of-mass reference prim.
    """

    def __init__(self, robot_prim_path: str, com_prim_path: str) -> None:
        self._robot_prim_path = robot_prim_path
        self._com_prim_path = com_prim_path
        self._from_usd(robot_prim_path, com_prim_path)

    def _from_usd(self, robot_prim_path: str, com_prim_path: str) -> None:
        """Parse mecanum wheel joint data from the USD stage.

        Args:
            robot_prim_path: Path to the robot prim on the USD stage.
            com_prim_path: Path to the center-of-mass prim (or empty to use robot prim).

        Raises:
            ValueError: If robot_prim_path is invalid.
        """
        stage = stage_utils.get_current_stage(backend="usd")
        robot_prim = stage.GetPrimAtPath(robot_prim_path)
        if not robot_prim.IsValid():
            raise ValueError(f"Invalid robot prim path: {robot_prim_path}")
        com_prim = stage.GetPrimAtPath(com_prim_path) if com_prim_path else robot_prim
        if not com_prim.IsValid():
            com_prim = robot_prim

        self._mecanum_joints = [j for j in Usd.PrimRange(robot_prim) if j.GetAttribute("isaacmecanumwheel:angle")]
        self._num_wheels = len(self._mecanum_joints)
        self._wheel_radius = [j.GetAttribute("isaacmecanumwheel:radius").Get() for j in self._mecanum_joints]
        self._mecanum_angles = [j.GetAttribute("isaacmecanumwheel:angle").Get() for j in self._mecanum_joints]
        self._wheel_dof_names = [j.GetName() for j in self._mecanum_joints]
        self._wheel_positions = np.zeros((self._num_wheels, 3), dtype=float)
        self._wheel_orientations = np.zeros((self._num_wheels, 4), dtype=float)
        com_pose = Gf.Matrix4f(omni.usd.get_world_transform_matrix(com_prim))
        for i, j in enumerate(self._mecanum_joints):
            joint = UsdPhysics.RevoluteJoint(j)
            chassis_prim = stage.GetPrimAtPath(joint.GetBody0Rel().GetTargets()[0])
            chassis_pose = Gf.Matrix4f(omni.usd.get_world_transform_matrix(chassis_prim))
            p_0 = joint.GetLocalPos0Attr().Get()
            r_0 = joint.GetLocalRot0Attr().Get()
            local_0 = Gf.Matrix4f()
            local_0.SetTranslate(p_0)
            local_0.SetRotateOnly(r_0)
            joint_pose = local_0 * chassis_pose
            self._wheel_positions[i, :] = joint_pose.ExtractTranslation() - com_pose.ExtractTranslation()
            rel_rot = joint_pose.ExtractRotation() * (com_pose.ExtractRotation().GetInverse())
            q = rel_rot.GetQuat()
            self._wheel_orientations[i, :] = np.array([q.GetReal(), *q.GetImaginary()])
        axis = {"X": np.array([1, 0, 0]), "Y": np.array([0, 1, 0]), "Z": np.array([0, 0, 1])}
        self._up_axis = axis[UsdGeom.GetStageUpAxis(stage)]
        joint = UsdPhysics.RevoluteJoint(self._mecanum_joints[-1])
        self._wheel_axis = axis[joint.GetAxisAttr().Get()]

    def get_holonomic_controller_params(
        self,
    ) -> tuple[list, np.ndarray, np.ndarray, list, np.ndarray, np.ndarray]:
        """Return parameters needed to construct a HolonomicController.

        Returns:
            Tuple of (wheel_radius, wheel_positions, wheel_orientations,
            mecanum_angles, wheel_axis, up_axis).
        """
        return (
            self._wheel_radius,
            self._wheel_positions,
            self._wheel_orientations,
            self._mecanum_angles,
            self._wheel_axis,
            self._up_axis,
        )

    def get_articulation_controller_params(self) -> list[str]:
        """Return the DOF names for the mecanum wheel joints.

        Returns:
            List of wheel joint DOF name strings.
        """
        return self._wheel_dof_names

    @property
    def wheel_radius(self) -> list:
        """Radius values for each wheel."""
        return self._wheel_radius

    @property
    def wheel_positions(self) -> np.ndarray:
        """Position coordinates for each wheel relative to the center of mass."""
        return self._wheel_positions

    @property
    def wheel_orientations(self) -> np.ndarray:
        """Orientation quaternions for each wheel relative to the center of mass."""
        return self._wheel_orientations

    @property
    def mecanum_angles(self) -> list:
        """Mecanum wheel angles for each wheel joint."""
        return self._mecanum_angles

    @property
    def wheel_dof_names(self) -> list[str]:
        """Degree of freedom names for each wheel joint."""
        return self._wheel_dof_names

    @property
    def wheel_axis(self) -> np.ndarray:
        """Axis of rotation for the wheels."""
        return self._wheel_axis

    @property
    def up_axis(self) -> np.ndarray:
        """Up axis of the USD stage."""
        return self._up_axis
