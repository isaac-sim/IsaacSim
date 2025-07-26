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
from typing import List, Tuple, Union

import numpy as np

# isaacsim.core.api
from isaacsim.core.api.robots.robot import Robot as _Robot

# isaacsim.core.prims
from isaacsim.core.prims import Articulation as _ArticulationView
from isaacsim.core.prims import SingleXFormPrim as XFormPrim
from isaacsim.core.utils.prims import get_prim_at_path

# isaacsim.core.utils
from isaacsim.core.utils.stage import add_reference_to_stage, get_current_stage
from isaacsim.replicator.mobility_gen.impl.camera import MobilityGenCamera
from isaacsim.replicator.mobility_gen.impl.common import Buffer, Module

# isaacsim.replicator.mobility_gen.examples
from isaacsim.replicator.mobility_gen.impl.robot import ROBOTS, MobilityGenRobot
from isaacsim.replicator.mobility_gen.impl.types import Pose2d
from isaacsim.replicator.mobility_gen.impl.utils.global_utils import get_world, join_sdf_paths
from isaacsim.replicator.mobility_gen.impl.utils.prim_utils import (
    prim_rotate_x,
    prim_rotate_y,
    prim_rotate_z,
    prim_translate,
)
from isaacsim.replicator.mobility_gen.impl.utils.registry import Registry
from isaacsim.replicator.mobility_gen.impl.utils.stage_utils import stage_add_camera
from isaacsim.robot.policy.examples.robots import SpotFlatTerrainPolicy

# isaacsim.robot.policy.examples
from isaacsim.robot.policy.examples.robots.h1 import H1FlatTerrainPolicy
from isaacsim.robot.wheeled_robots.controllers.differential_controller import DifferentialController

# isaacsim.robot.wheeled_robots
from isaacsim.robot.wheeled_robots.robots import WheeledRobot as _WheeledRobot
from isaacsim.storage.native import get_assets_root_path

# this package
from .misc import HawkCamera


class WheeledMobilityGenRobot(MobilityGenRobot):

    # Wheeled robot parameters
    wheel_dof_names: List[str]
    usd_url: str
    chassis_subpath: str
    wheel_radius: float
    wheel_base: float

    def __init__(
        self,
        prim_path: str,
        robot: _WheeledRobot,
        articulation_view: _ArticulationView,
        controller: DifferentialController,
        front_camera: Module | None = None,
    ):
        super().__init__(
            prim_path=prim_path, robot=robot, articulation_view=articulation_view, front_camera=front_camera
        )
        self.controller = controller
        self.robot = robot

    @classmethod
    def build(cls, prim_path: str) -> "WheeledRobot":

        world = get_world()

        robot = world.scene.add(
            _WheeledRobot(prim_path, wheel_dof_names=cls.wheel_dof_names, create_robot=True, usd_path=cls.usd_url)
        )

        view = _ArticulationView(join_sdf_paths(prim_path, cls.chassis_subpath))

        world.scene.add(view)

        controller = DifferentialController(name="controller", wheel_radius=cls.wheel_radius, wheel_base=cls.wheel_base)

        camera = cls.build_front_camera(prim_path)

        return cls(prim_path=prim_path, robot=robot, articulation_view=view, controller=controller, front_camera=camera)

    def write_action(self, step_size: float):
        self.robot.apply_wheel_actions(self.controller.forward(command=self.action.get_value()))


class PolicyMobilityGenRobot(MobilityGenRobot):

    usd_url: str
    articulation_path: str

    def __init__(
        self,
        prim_path: str,
        robot: _Robot,
        articulation_view: _ArticulationView,
        controller: Union[H1FlatTerrainPolicy, SpotFlatTerrainPolicy],
        front_camera: Module | None = None,
    ):
        super().__init__(prim_path, robot, articulation_view, front_camera)
        self.controller = controller

    @classmethod
    def build_policy(cls, prim_path: str):
        raise NotImplementedError

    @classmethod
    def build(cls, prim_path: str):
        stage = get_current_stage()
        world = get_world()

        add_reference_to_stage(usd_path=cls.usd_url, prim_path=prim_path)

        robot = _Robot(prim_path=prim_path)

        world.scene.add(robot)

        # Articulation
        view = _ArticulationView(join_sdf_paths(prim_path, cls.articulation_path))

        world.scene.add(view)

        # Controller
        controller = cls.build_policy(prim_path)

        prim = get_prim_at_path(prim_path)
        prim_translate(prim, (0, 0, cls.z_offset))

        camera = cls.build_front_camera(prim_path)

        return cls(prim_path=prim_path, robot=robot, articulation_view=view, controller=controller, front_camera=camera)

    def write_action(self, step_size):
        action = self.action.get_value()
        command = np.array([action[0], 0.0, action[1]])
        self.controller.forward(step_size, command)

    def set_pose_2d(self, pose):
        super().set_pose_2d(pose)
        self.controller.initialize()


@ROBOTS.register()
class JetbotRobot(WheeledMobilityGenRobot):

    physics_dt: float = 0.005

    z_offset: float = 0.1

    chase_camera_base_path = "chassis"
    chase_camera_x_offset: float = -0.5
    chase_camera_z_offset: float = 0.5
    chase_camera_tilt_angle: float = 60.0

    occupancy_map_radius: float = 0.25
    occupancy_map_z_min: float = 0.05
    occupancy_map_z_max: float = 0.5
    occupancy_map_cell_size: float = 0.05
    occupancy_map_collision_radius: float = 0.25

    front_camera_base_path = "chassis/rgb_camera/front_hawk"
    front_camera_rotation = (0.0, 0.0, 0.0)
    front_camera_translation = (0.0, 0.0, 0.0)
    front_camera_type = HawkCamera

    keyboard_linear_velocity_gain: float = 0.25
    keyboard_angular_velocity_gain: float = 1.0

    gamepad_linear_velocity_gain: float = 0.25
    gamepad_angular_velocity_gain: float = 1.0

    random_action_linear_velocity_range: Tuple[float, float] = (-0.3, 0.25)
    random_action_angular_velocity_range: Tuple[float, float] = (-0.75, 0.75)
    random_action_linear_acceleration_std: float = 1.0
    random_action_angular_acceleration_std: float = 5.0
    random_action_grid_pose_sampler_grid_size: float = 5.0

    path_following_speed: float = 0.25
    path_following_angular_gain: float = 1.0
    path_following_stop_distance_threshold: float = 0.5
    path_following_forward_angle_threshold = math.pi / 4
    path_following_target_point_offset_meters: float = 1.0

    wheel_dof_names: List[str] = ["left_wheel_joint", "right_wheel_joint"]
    usd_url: str = get_assets_root_path() + "/Isaac/Robots/NVIDIA/Jetbot/jetbot.usd"

    chassis_subpath: str = "chassis"
    wheel_base: float = 0.1125
    wheel_radius: float = 0.03


@ROBOTS.register()
class CarterRobot(WheeledMobilityGenRobot):

    physics_dt: float = 0.005

    z_offset: float = 0.25

    chase_camera_base_path = "chassis_link"
    chase_camera_x_offset: float = -1.5
    chase_camera_z_offset: float = 0.8
    chase_camera_tilt_angle: float = 60.0

    occupancy_map_radius: float = 1.0
    occupancy_map_z_min: float = 0.1
    occupancy_map_z_max: float = 0.62
    occupancy_map_cell_size: float = 0.05
    occupancy_map_collision_radius: float = 0.5

    front_camera_base_path = "chassis_link/front_hawk/front_hawk"
    front_camera_rotation = (0.0, 0.0, 0.0)
    front_camera_translation = (0.0, 0.0, 0.0)
    front_camera_type = HawkCamera

    keyboard_linear_velocity_gain: float = 1.0
    keyboard_angular_velocity_gain: float = 1.0

    gamepad_linear_velocity_gain: float = 1.0
    gamepad_angular_velocity_gain: float = 1.0

    random_action_linear_velocity_range: Tuple[float, float] = (-0.3, 1.0)
    random_action_angular_velocity_range: Tuple[float, float] = (-0.75, 0.75)
    random_action_linear_acceleration_std: float = 5.0
    random_action_angular_acceleration_std: float = 5.0
    random_action_grid_pose_sampler_grid_size: float = 5.0

    path_following_speed: float = 1.0
    path_following_angular_gain: float = 1.0
    path_following_stop_distance_threshold: float = 0.5
    path_following_forward_angle_threshold = math.pi / 4
    path_following_target_point_offset_meters: float = 1.0

    wheel_dof_names: List[str] = ["joint_wheel_left", "joint_wheel_right"]
    usd_url: str = get_assets_root_path() + "/Isaac/Robots/NVIDIA/NovaCarter/nova_carter.usd"
    chassis_subpath: str = "chassis_link"
    wheel_base = 0.413
    wheel_radius = 0.14


@ROBOTS.register()
class H1Robot(PolicyMobilityGenRobot):

    physics_dt: float = 0.005

    z_offset: float = 1.05

    chase_camera_base_path = "pelvis"
    chase_camera_x_offset: float = -1.5
    chase_camera_z_offset: float = 0.8
    chase_camera_tilt_angle: float = 60.0

    occupancy_map_radius: float = 1.0
    occupancy_map_z_min: float = 0.1
    occupancy_map_z_max: float = 2.0
    occupancy_map_cell_size: float = 0.05
    occupancy_map_collision_radius: float = 0.5

    front_camera_base_path = "d435_left_imager_link/front_camera/front"
    front_camera_rotation = (0.0, 250.0, 90.0)
    front_camera_translation = (-0.06, 0.0, 0.0)
    front_camera_type = HawkCamera

    keyboard_linear_velocity_gain: float = 1.0
    keyboard_angular_velocity_gain: float = 1.0

    gamepad_linear_velocity_gain: float = 1.0
    gamepad_angular_velocity_gain: float = 1.0

    random_action_linear_velocity_range: Tuple[float, float] = (-0.3, 1.0)
    random_action_angular_velocity_range: Tuple[float, float] = (-0.75, 0.75)
    random_action_linear_acceleration_std: float = 5.0
    random_action_angular_acceleration_std: float = 5.0
    random_action_grid_pose_sampler_grid_size: float = 5.0

    path_following_speed: float = 1.0
    path_following_angular_gain: float = 1.0
    path_following_stop_distance_threshold: float = 0.5
    path_following_forward_angle_threshold = math.pi / 4
    path_following_target_point_offset_meters: float = 1.0

    usd_url = get_assets_root_path() + "/Isaac/Robots/Unitree/H1/h1.usd"
    articulation_path = "pelvis"
    controller_z_offset: float = 1.05

    @classmethod
    def build_policy(cls, prim_path: str):
        return H1FlatTerrainPolicy(prim_path=prim_path, position=np.array([0.0, 0.0, cls.controller_z_offset]))


@ROBOTS.register()
class SpotRobot(PolicyMobilityGenRobot):

    physics_dt: float = 0.005
    z_offset: float = 0.7

    chase_camera_base_path = "body"
    chase_camera_x_offset: float = -1.5
    chase_camera_z_offset: float = 0.8
    chase_camera_tilt_angle: float = 60.0

    occupancy_map_radius: float = 1.0
    occupancy_map_z_min: float = 0.1
    occupancy_map_z_max: float = 0.62
    occupancy_map_cell_size: float = 0.05
    occupancy_map_collision_radius: float = 0.5

    front_camera_base_path = "body/front_camera"
    front_camera_rotation = (180, 180, 180)
    front_camera_translation = (0.44, 0.075, 0.01)
    front_camera_type = HawkCamera

    keyboard_linear_velocity_gain: float = 1.0
    keyboard_angular_velocity_gain: float = 1.0

    gamepad_linear_velocity_gain: float = 1.0
    gamepad_angular_velocity_gain: float = 1.0

    random_action_linear_velocity_range: Tuple[float, float] = (-0.3, 1.0)
    random_action_angular_velocity_range: Tuple[float, float] = (-0.75, 0.75)
    random_action_linear_acceleration_std: float = 5.0
    random_action_angular_acceleration_std: float = 5.0
    random_action_grid_pose_sampler_grid_size: float = 5.0

    path_following_speed: float = 1.0
    path_following_angular_gain: float = 1.0
    path_following_stop_distance_threshold: float = 0.5
    path_following_forward_angle_threshold = math.pi / 4
    path_following_target_point_offset_meters: float = 1.0

    usd_url = get_assets_root_path() + "/Isaac/Robots/BostonDynamics/spot/spot.usd"
    articulation_path = "/"
    controller_z_offset: float = 0.7

    @classmethod
    def build_policy(cls, prim_path: str):
        return SpotFlatTerrainPolicy(prim_path=prim_path, position=np.array([0.0, 0.0, cls.controller_z_offset]))
