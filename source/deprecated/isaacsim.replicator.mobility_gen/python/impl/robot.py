# SPDX-FileCopyrightText: Copyright (c) 2025-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

"""Provides robot implementations and utilities for the Mobility Generation replicator module."""

import math
from typing import NoReturn

import isaacsim.core.utils.numpy.rotations as rot_utils

# Standard imports
import numpy as np
from isaacsim.core.api.robots.robot import Robot as _Robot

# Isaac Sim Imports
from isaacsim.core.prims import Articulation as _ArticulationView
from isaacsim.core.prims import SingleXFormPrim as XFormPrim
from isaacsim.core.utils.prims import get_prim_at_path
from isaacsim.core.utils.stage import get_current_stage

# Extension imports
from .common import Buffer, Module
from .types import Pose2d
from .utils.global_utils import join_sdf_paths
from .utils.prim_utils import prim_rotate_x, prim_rotate_y, prim_rotate_z, prim_translate
from .utils.registry import Registry
from .utils.stage_utils import stage_add_camera

# =========================================================
#  BASE CLASSES
# =========================================================


class MobilityGenRobot(Module):
    """Abstract base class for robots.

    This class defines an abstract base class for robots.

    Robot implementations must subclass this class, define the
    required class parameters and abstract methods.

    The two main abstract methods subclasses must define are the build() and write_action()
    methods.

    Args:
        prim_path: The USD path where the robot is located.
        robot: The Isaac Sim robot instance.
        articulation_view: The articulation view for the robot.
        front_camera: The front camera module attached to the robot.
    """

    physics_dt: float
    """The physics timestep to use for simulating the robot."""

    z_offset: float
    """The Z-axis offset height to spawn the robot. """

    chase_camera_base_path: str
    """The relative USD path which will be used to spawn the third person view camera.  This is typically set
    to the robot base frame."""

    chase_camera_x_offset: float
    """The relative X-axis offset to spawn the third person view camera."""

    chase_camera_z_offset: float
    """The relative Z-axis offset to spawn the third person view camera."""

    chase_camera_tilt_angle: float
    """The tilt angle to apply to the third person view camera."""

    occupancy_map_radius: float
    """The robot footprint radius to use for spawning and path planning."""

    occupancy_map_collision_radius: float
    """The robot footprint radius to use for collision based episode termination."""

    front_camera_type: type[Module]
    """The static class representing the front camera.  It should define a build() and attach() method. """

    front_camera_base_path: str
    """The relative USD path to spawn the front camera."""

    front_camera_rotation: tuple[float, float, float]
    """The relative XYZ rotation used when spawning the front camera. """

    front_camera_translation: tuple[float, float, float]
    """The relative XYZ translation used when spawning the front camera. """

    keyboard_linear_velocity_gain: float
    """The gain used to map keyboard button presses to the robot's linear velocity.  A larger
    gain results in faster movement."""

    keyboard_angular_velocity_gain: float
    """The gain used to map keyboard button presses to the robot's angular velocity.  A larger
    gain results in faster movement."""

    gamepad_linear_velocity_gain: float
    """The gain used to map gamepad axis movement to the robot's linear velocity.  A larger
    gain results in faster movement."""

    gamepad_angular_velocity_gain: float
    """The gain used to map gamepad axis movement to the robot's angular velocity.  A larger
    gain results in faster movement."""

    random_action_linear_velocity_range: tuple[float, float]
    """The robot linear velocity limits for the random acceleration scenario."""

    random_action_angular_velocity_range: tuple[float, float]
    """The robot angular velocity limits for the random acceleration scenario."""

    random_action_linear_acceleration_std: float
    """The standard deviation used for sampling the robot linear acceleration each timestep during
    the random acceleration scenario."""

    random_action_angular_acceleration_std: float
    """The standard deviation used for sampling the robot angular acceleration each timestep during
    the random acceleration scenario."""

    random_action_grid_pose_sampler_grid_size: float
    """The grid size to use for spawning the robot during the random acceleration scenario."""

    path_following_speed: float
    """The constant linear speed to use for the path following scenario."""

    path_following_angular_gain: float
    """The gain used for the proportional steering control in the path following scenario.
    A larger gain results in quicker turning, but potential overshoot and wobbling."""

    path_following_stop_distance_threshold: float
    """The distance threshold at which point the robot will stop.  Applies to the path following scenario."""

    path_following_forward_angle_threshold = math.pi
    """The angle threshold at which point the robot will move forward.  Applies to the path following scenario."""

    path_following_target_point_offset_meters: float
    """The offset distance used to generate the 'target point' that the robot will follow in the path following scenario.
    A larger offset results in smoother motion, but too large may cause the robot to cut corners during turns."""

    def __init__(
        self, prim_path: str, robot: _Robot, articulation_view: _ArticulationView, front_camera: Module
    ) -> None:
        self.prim_path = prim_path
        self.robot = robot
        self.articulation_view = articulation_view

        self.action = Buffer(np.zeros(2))
        self.position = Buffer()
        self.orientation = Buffer()
        self.joint_positions = Buffer()
        self.joint_velocities = Buffer()
        self.linear_velocity = Buffer()
        self.angular_velocity = Buffer()
        self.front_camera = front_camera

    @classmethod
    def build_front_camera(cls, prim_path: str) -> object:
        """Builds and configures the front camera for the robot.

        Creates a camera at the specified prim path with the configured rotation and translation offsets.
        The camera is positioned and oriented according to the class-defined front camera parameters.

        Args:
            prim_path: The USD prim path where the camera should be built.

        Returns:
            The built front camera module instance.
        """
        # Add camera
        camera_path = join_sdf_paths(prim_path, cls.front_camera_base_path)
        front_camera_xform = XFormPrim(camera_path)

        stage = get_current_stage()
        front_camera_prim = get_prim_at_path(camera_path)
        prim_rotate_x(front_camera_prim, cls.front_camera_rotation[0])
        prim_rotate_y(front_camera_prim, cls.front_camera_rotation[1])
        prim_rotate_z(front_camera_prim, cls.front_camera_rotation[2])
        prim_translate(front_camera_prim, cls.front_camera_translation)

        return cls.front_camera_type.build(prim_path=camera_path)

    def build_chase_camera(self) -> str:
        """Builds and configures the third-person chase camera for the robot.

        Creates a camera with specified focal length and aperture settings, positioned behind and above
        the robot with the configured offset and tilt angle.

        Returns:
            The USD prim path of the created chase camera.
        """
        stage = get_current_stage()

        camera_path = join_sdf_paths(self.prim_path, self.chase_camera_base_path, "chase_camera")

        stage_add_camera(stage, camera_path, focal_length=10, horizontal_aperature=30, vertical_aperature=30)
        camera_prim = get_prim_at_path(camera_path)
        prim_rotate_x(camera_prim, self.chase_camera_tilt_angle)
        prim_rotate_y(camera_prim, 0)
        prim_rotate_z(camera_prim, -90)
        prim_translate(camera_prim, (self.chase_camera_x_offset, 0.0, self.chase_camera_z_offset))

        return camera_path

    @classmethod
    def build(cls, prim_path: str) -> "Robot":
        """Builds a robot instance at the specified prim path.

        This is an abstract method that must be implemented by robot subclasses to define
        how the robot is constructed and configured.

        Args:
            prim_path: The USD prim path where the robot should be built.

        Returns:
            The built robot instance.

        Raises:
            NotImplementedError: This method must be implemented by subclasses.
        """
        raise NotImplementedError

    def write_action(self, step_size: float) -> NoReturn:
        """Writes the robot's action to the simulation.

        This is an abstract method that must be implemented by robot subclasses to define
        how the robot's action buffer is applied to the simulation.

        Args:
            step_size: The timestep size for the action execution.

        Raises:
            NotImplementedError: This method must be implemented by subclasses.
        """
        raise NotImplementedError

    def update_state(self) -> None:
        """Updates the robot's state buffers with current simulation data.

        Retrieves and stores the robot's current world pose, joint positions, joint velocities,
        linear velocity, and angular velocity in the respective buffers.
        """
        pos, ori = self.robot.get_world_pose()
        self.position.set_value(pos)
        self.orientation.set_value(ori)
        self.joint_positions.set_value(self.robot.get_joint_positions())
        self.joint_velocities.set_value(self.robot.get_joint_velocities())
        self.linear_velocity.set_value(self.robot.get_linear_velocity())
        self.angular_velocity.set_value(self.robot.get_angular_velocity())
        super().update_state()

    def write_replay_data(self) -> None:
        """Writes buffered state data to the robot for replay purposes.

        Applies the stored position, orientation, and joint positions from the buffers
        to the robot's current state in the simulation.
        """
        self.robot.set_local_pose(self.position.get_value(), self.orientation.get_value())
        self.articulation_view.set_joint_positions(self.joint_positions.get_value())
        super().write_replay_data()

    def set_pose_2d(self, pose: Pose2d) -> None:
        """Sets the robot's 2D pose in the world.

        Initializes the articulation, stops all motion, and positions the robot at the specified
        2D pose with the configured Z-axis offset.

        Args:
            pose: The 2D pose containing x, y coordinates and theta orientation.
        """
        self.articulation_view.initialize()
        self.robot.set_world_velocity(np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0]))
        self.robot.post_reset()
        position, orientation = self.robot.get_world_pose()
        position[0] = pose.x
        position[1] = pose.y
        position[2] = self.z_offset
        orientation = rot_utils.euler_angles_to_quats(np.array([0.0, 0.0, pose.theta]))
        self.robot.set_local_pose(position, orientation)

    def get_pose_2d(self) -> Pose2d:
        """Current 2D pose of the robot.

        Extracts the x, y coordinates and theta orientation from the robot's world pose.

        Returns:
            The robot's current 2D pose containing x, y coordinates and theta orientation.
        """
        position, orientation = self.robot.get_world_pose()
        theta = rot_utils.quats_to_euler_angles(orientation)[2]
        return Pose2d(x=position[0], y=position[1], theta=theta)


ROBOTS = Registry[MobilityGenRobot]()
