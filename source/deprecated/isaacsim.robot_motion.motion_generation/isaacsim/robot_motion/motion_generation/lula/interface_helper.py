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

"""Provides helper utilities for interfacing with Lula robot motion generation algorithms."""

from __future__ import annotations

from typing import Optional

import lula
import numpy as np
from isaacsim.core.api import objects
from isaacsim.core.utils.numpy.rotations import quats_to_rot_matrices
from isaacsim.core.utils.stage import get_stage_units

from . import utils as lula_utils
from .world import LulaWorld


class LulaInterfaceHelper(LulaWorld):
    """Class containing functions common in Lula based algorithms. The main utility of this class is handling the tracking of the robot base and returning basic robot information.

    Args:
        robot_description: The Lula robot description containing kinematic information.
    """

    def __init__(self, robot_description: lula.RobotDescription) -> None:
        LulaWorld.__init__(self)

        self._robot_description = robot_description
        self._kinematics = self._robot_description.kinematics()

        self._robot_base_moved = False
        self._robot_pos, self._robot_rot = np.zeros(3), np.eye(3)

        self._meters_per_unit = get_stage_units()

    def set_robot_base_pose(self, robot_position: np.array, robot_orientation: np.array) -> None:
        """Update position of the robot base. Until this function is called, Lula will assume the base pose.

        to be at the origin with identity rotation.

        Args:
            robot_position: (3 x 1) translation vector describing the translation of the robot base relative to the USD stage origin.
                The translation vector should be specified in the units of the USD stage
            robot_orientation: (4 x 1) quaternion describing the orientation of the robot base relative to the USD stage global frame
        """
        # all object poses are relative to the position of the robot base
        robot_position = robot_position * self._meters_per_unit
        robot_rot = quats_to_rot_matrices(robot_orientation)

        if np.any(self._robot_pos - robot_position) or np.any(self._robot_rot - robot_rot):
            self._robot_base_moved = True
        else:
            self._robot_base_moved = False

        self._robot_pos = robot_position
        self._robot_rot = robot_rot

    def get_active_joints(self) -> list[str]:
        """Names of the active joints in the robot.

        Returns:
            List of active joint names from the robot description.
        """
        return [
            self._robot_description.c_space_coord_name(i) for i in range(self._robot_description.num_c_space_coords())
        ]

    def get_watched_joints(self) -> list:
        """Lula does not currently support watching joint states that are not controllable.

        Returns:
            Always returns an empty list.
        """
        return []

    def get_end_effector_pose(self, active_joint_positions: np.array, frame_name: str) -> tuple[np.array, np.array]:
        """Return pose of robot end effector given current joint positions.

        The end effector position will be transformed into world coordinates based
        on the believed position of the robot base.  See set_robot_base_pose().

        Args:
            active_joint_positions: positions of the active joints in the robot
            frame_name: Name of the frame to compute pose for

        Returns:
            A tuple containing (end_effector_translation, end_effector_rotation) where:

            end_effector_translation: (3x1) translation vector for the robot end effector
                 relative to the USD stage origin

            end_effector_rotation: (3x3) rotation matrix describing the orientation of the
                robot end effector relative to the USD global frame
        """
        # returns pose of end effector in world coordinates
        pose = self._kinematics.pose(np.expand_dims(active_joint_positions, 1), frame_name)

        translation = self._robot_rot @ (pose.translation) + self._robot_pos
        rotation = self._robot_rot @ pose.rotation.matrix()
        return translation / self._meters_per_unit, rotation

    def update_world(self, updated_obstacles: Optional[list] = None) -> None:
        """Update the world state with current robot base position and any obstacle changes.

        Args:
            updated_obstacles: List of obstacles that have been updated
        """
        LulaWorld.update_world(self, updated_obstacles, self._robot_pos, self._robot_rot, self._robot_base_moved)
        self._robot_base_moved = False

    def add_cuboid(
        self,
        cuboid: objects.cuboid.DynamicCuboid | objects.cuboid.FixedCuboid | objects.cuboid.VisualCuboid,
        static: Optional[bool] = False,
    ) -> bool:
        """Add a cuboid obstacle to the Lula world.

        Args:
            cuboid: The cuboid object to add
            static: Whether the cuboid should be treated as static

        Returns:
            Result from adding the cuboid to the Lula world.
        """
        return LulaWorld.add_cuboid(self, cuboid, static, self._robot_pos, self._robot_rot)

    def add_sphere(
        self, sphere: objects.sphere.DynamicSphere | objects.sphere.VisualSphere, static: bool = False
    ) -> bool:
        """Add a sphere obstacle to the Lula world.

        Args:
            sphere: The sphere object to add
            static: Whether the sphere should be treated as static

        Returns:
            Result from adding the sphere to the Lula world.
        """
        return LulaWorld.add_sphere(self, sphere, static, self._robot_pos, self._robot_rot)

    def add_capsule(
        self, capsule: objects.capsule.DynamicCapsule | objects.capsule.VisualCapsule, static: bool = False
    ) -> bool:
        """Add a capsule obstacle to the Lula world.

        Args:
            capsule: The capsule object to add
            static: Whether the capsule should be treated as static

        Returns:
            Result from adding the capsule to the Lula world.
        """
        return LulaWorld.add_capsule(self, capsule, static, self._robot_pos, self._robot_rot)

    def reset(self) -> None:
        """Reset the Lula world and robot base pose to initial state."""
        LulaWorld.reset(self)

        self._robot_base_moved = False
        self._robot_pos, self._robot_rot = np.zeros(3), np.eye(3)

    def _get_prim_pose_rel_robot_base(self, prim: object) -> tuple:
        """Get the pose of a prim relative to the robot base position.

        Args:
            prim: The prim to get pose for

        Returns:
            The position of the prim relative to the robot base position.
        """
        # returns the position of a prim relative to the position of the robot
        return lula_utils.get_prim_pose_in_meters_rel_robot_base(
            prim, self._meters_per_unit, self._robot_pos, self._robot_rot
        )

    def _get_pose_rel_robot_base(self, trans: object, rot: object) -> tuple:
        """Transforms a pose from world coordinates to robot base relative coordinates.

        Args:
            trans: Translation vector in world coordinates.
            rot: Rotation matrix in world coordinates.

        Returns:
            The pose transformed to be relative to the robot base position and orientation.
        """
        return lula_utils.get_pose_rel_robot_base(trans, rot, self._robot_pos, self._robot_rot)
