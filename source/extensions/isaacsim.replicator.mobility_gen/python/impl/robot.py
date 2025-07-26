# SPDX-FileCopyrightText: Copyright (c) 2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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


import math
import os
from typing import Tuple, Type

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
    """Abstract base class for robots

    This class defines an abstract base class for robots.

    Robot implementations must subclass this class, define the
    required class parameters and abstract methods.

    The two main abstract methods subclasses must define are the build() and write_action()
    methods.

    Parameters:
        physics_dt (float): The physics time step the robot requires (in seconds).
            This may need to be modified depending on the underlying controller's

        z_offset (float): A z offset to use when spawning the robot to ensure it
            drops at an appropriate height when initializing.
        chase_camera_base_path (str):  A path (in the USD stage) relative to the
            base path to use as a parent when defining the "chase" camera.  Typically,
            this is the same as the path to the transform used for determining
            the 2D pose of the robot.
        chase_camera_x_offset (str):  The x offset of the chase camera.  Typically this is
            negative, to locate the camera behind the robot.
        chase_camera_z_offset (str): The z offset of the chase camera.  Typically this is
            positive, to locate the camera above the robot.
        chase_camera_tile_angle (float):  The tilt angle of the chase camera.  Typically
            this does not need to be modified.
        front_camera_type (Type[Sensor]):  The configurable sensor class to attach
            at the front camera for the robot.  This should be a final sensor class (like HawkCamera)
            that can be built using the class method HawkCamera.build(prim_path).
        front_camera_base_path (str):  The relative path (in the USD stage) relative to the
            robot prim path to use as the basis for creating the front camera XForm.
        front_camera_rotation (Tuple[float, float, float]):  The (x, y, z) rotation to apply
            when building the XForm for the front camera.
        front_camera_translation (Tuple[float, float, float]):  The (x, y, z) rotation to apply
            when building the XForm for the front camera.

    """

    physics_dt: float

    z_offset: float

    chase_camera_base_path: str
    chase_camera_x_offset: float
    chase_camera_z_offset: float
    chase_camera_tilt_angle: float

    occupancy_map_radius: float
    occupancy_map_z_min: float
    occupancy_map_z_max: float
    occupancy_map_cell_size: float
    occupancy_map_collision_radius: float

    front_camera_type: Type[Module]
    front_camera_base_path: str
    front_camera_rotation: Tuple[float, float, float]
    front_camera_translation: Tuple[float, float, float]

    keyboard_linear_velocity_gain: float
    keyboard_angular_velocity_gain: float

    gamepad_linear_velocity_gain: float
    gamepad_angular_velocity_gain: float

    random_action_linear_velocity_range: Tuple[float, float]
    random_action_angular_velocity_range: Tuple[float, float]
    random_action_linear_acceleration_std: float
    random_action_angular_acceleration_std: float
    random_action_grid_pose_sampler_grid_size: float

    path_following_speed: float
    path_following_angular_gain: float
    path_following_stop_distance_threshold: float
    path_following_forward_angle_threshold = math.pi
    path_following_target_point_offset_meters: float

    def __init__(self, prim_path: str, robot: _Robot, articulation_view: _ArticulationView, front_camera: Module):
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
    def build_front_camera(cls, prim_path):

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
        raise NotImplementedError

    def write_action(self, step_size: float):
        raise NotImplementedError

    def update_state(self):
        pos, ori = self.robot.get_world_pose()
        self.position.set_value(pos)
        self.orientation.set_value(ori)
        self.joint_positions.set_value(self.robot.get_joint_positions())
        self.joint_velocities.set_value(self.robot.get_joint_velocities())
        self.linear_velocity.set_value(self.robot.get_linear_velocity())
        self.angular_velocity.set_value(self.robot.get_angular_velocity())
        super().update_state()

    def write_replay_data(self):
        self.robot.set_local_pose(self.position.get_value(), self.orientation.get_value())
        self.articulation_view.set_joint_positions(self.joint_positions.get_value())
        super().write_replay_data()

    def set_pose_2d(self, pose: Pose2d):
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
        position, orientation = self.robot.get_world_pose()
        theta = rot_utils.quats_to_euler_angles(orientation)[2]
        return Pose2d(x=position[0], y=position[1], theta=theta)


ROBOTS = Registry[MobilityGenRobot]()
