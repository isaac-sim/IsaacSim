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

"""Module for setting up USD attributes and parameters for holonomic robots with mecanum wheels."""

from typing import Any

import numpy as np
import omni
from isaacsim.core.utils.prims import get_prim_at_path
from isaacsim.core.utils.rotations import gf_rotation_to_np_array
from isaacsim.core.utils.stage import get_current_stage
from pxr import Gf, Usd, UsdGeom, UsdPhysics


# TODO: Why isn't this a "HolonomicRobot" that inherits robot?
class HolonomicRobotUsdSetup:
    """Set up the attributes on the prims of a holonomic robot. Specifically adds the `isaacmecanumwheel:radius` and `isaacmecanumwheel:angle` attributes to the wheel joints of the robot prim.

    Args:
        robot_prim_path: Path of the robot articulation.
        com_prim_path: Path of the xform representing the center of mass of the vehicle.
    """

    def __init__(self, robot_prim_path: str, com_prim_path: str) -> None:
        self._robot_prim_path = robot_prim_path
        self._com_prim_path = com_prim_path  # TODO: make this one of USD attribute
        self.from_usd(self._robot_prim_path, self._com_prim_path)

    def from_usd(self, robot_prim_path: str, com_prim_path: str) -> None:
        """Extract and compile necessary information from USD when available.

        Args:
            robot_prim_path: Path of the robot articulation.
            com_prim_path: Path of the xform representing the center of mass of the vehicle.
        """
        stage = get_current_stage()
        robot_prim = get_prim_at_path(robot_prim_path)
        if self._com_prim_path == "":
            com_prim = robot_prim  # if no com prim given, assume robot root prim is also com prim
        else:
            com_prim = get_prim_at_path(com_prim_path)

        self._mecanum_joints = [j for j in Usd.PrimRange(robot_prim) if j.GetAttribute("isaacmecanumwheel:angle")]
        self._num_wheels = len(self._mecanum_joints)
        self._wheel_radius = [j.GetAttribute("isaacmecanumwheel:radius").Get() for j in self._mecanum_joints]
        self._mecanum_angles = [j.GetAttribute("isaacmecanumwheel:angle").Get() for j in self._mecanum_joints]
        self._wheel_dof_names = [j.GetName() for j in self._mecanum_joints]
        self._wheel_positions = np.zeros((self._num_wheels, 3), dtype=float)  ## xyz for position
        self._wheel_orientations = np.zeros((self._num_wheels, 4), dtype=float)  ## quaternion for orientation
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
            self._wheel_orientations[i, :] = gf_rotation_to_np_array(
                joint_pose.ExtractRotation() * ((com_pose.ExtractRotation()).GetInverse())
            )

        axis = {"X": np.array([1, 0, 0]), "Y": np.array([0, 1, 0]), "Z": np.array([0, 0, 1])}
        self._up_axis = axis[UsdGeom.GetStageUpAxis(stage)]
        self._wheel_axis = axis[joint.GetAxisAttr().Get()]

    def get_holonomic_controller_params(self) -> tuple:
        """Parameters needed for holonomic robot control.

        Returns:
            A tuple containing (wheel_radius, wheel_positions, wheel_orientations, mecanum_angles, wheel_axis, up_axis).
        """
        return (
            self._wheel_radius,
            self._wheel_positions,
            self._wheel_orientations,
            self._mecanum_angles,
            self._wheel_axis,
            self._up_axis,
        )

    def get_articulation_controller_params(self) -> list:
        """Parameters needed for articulation control.

        Returns:
            List of wheel degree of freedom names.
        """
        return self._wheel_dof_names

    @property
    def wheel_radius(self) -> list:
        """Radius values for each wheel.

        Returns:
            List of wheel radius values.
        """
        return self._wheel_radius

    @property
    def wheel_positions(self) -> Any:
        """Position coordinates for each wheel relative to the center of mass.

        Returns:
            Array of wheel positions with shape (num_wheels, 3).
        """
        return self._wheel_positions

    @property
    def wheel_orientations(self) -> Any:
        """Orientation quaternions for each wheel relative to the center of mass.

        Returns:
            Array of wheel orientations with shape (num_wheels, 4).
        """
        return self._wheel_orientations

    @property
    def mecanum_angles(self) -> list:
        """Mecanum wheel angles for each wheel joint.

        Returns:
            List of mecanum wheel angles.
        """
        return self._mecanum_angles

    @property
    def wheel_dof_names(self) -> list:
        """Degree of freedom names for each wheel joint.

        Returns:
            List of wheel degree of freedom names.
        """
        return self._wheel_dof_names

    @property
    def wheel_axis(self) -> Any:
        """Axis of rotation for the wheels.

        Returns:
            Array representing the wheel rotation axis.
        """
        return self._wheel_axis

    @property
    def up_axis(self) -> Any:
        """Up axis of the USD stage.

        Returns:
            Array representing the stage up axis.
        """
        return self._up_axis
