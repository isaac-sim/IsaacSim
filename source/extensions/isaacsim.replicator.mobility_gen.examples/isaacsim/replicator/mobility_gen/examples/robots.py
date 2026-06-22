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

"""Provides pre-configured robot implementations for mobility generation scenarios in Isaac Sim."""

from __future__ import annotations

import math
from abc import abstractmethod

import numpy as np
from isaacsim.core.deprecation_manager import import_module

# isaacsim.replicator.mobility_gen.examples
# isaacsim.core.experimental.*
from isaacsim.core.experimental.prims import Articulation
from isaacsim.core.experimental.utils.stage import add_reference_to_stage, get_current_stage
from isaacsim.replicator.experimental.mobility_gen import (
    ROBOTS,
    MobilityGenMultiSensorRobot,
    MobilityGenRobot,
    Module,
    Pose2d,
)
from isaacsim.robot.experimental.wheeled_robots import DifferentialController
from isaacsim.robot.policy.examples.robots import H1FlatTerrainPolicy, SpotFlatTerrainPolicy
from isaacsim.storage.native import get_assets_root_path

# this package
from .misc import HawkCamera


class WheeledMobilityGenRobot(MobilityGenRobot):
    """A wheeled robot implementation for mobility generation scenarios.

    This class extends MobilityGenRobot to provide specific functionality for wheeled robots using differential
    steering control. It integrates the robot's physical representation, articulation system, and control logic
    to enable autonomous navigation and movement in simulation environments.

    The class manages the robot's wheel actions through a DifferentialController, allowing for precise control
    of linear and angular velocities. It supports optional front camera integration for visual perception
    capabilities during navigation tasks.

    Args:
        prim_path: USD prim path where the robot is located in the stage.
        articulation: Articulation for managing the robot's joint system.
        controller: Differential controller for wheel-based movement control.
        front_camera: Optional camera module for front-facing visual perception.
    """

    # Wheeled robot parameters
    wheel_dof_names: list[str]
    usd_url: str
    chassis_subpath: str
    wheel_radius: float
    wheel_base: float

    def __init__(
        self,
        prim_path: str,
        articulation: Articulation,
        controller: DifferentialController,
        front_camera: Module | None = None,
    ) -> None:
        super().__init__(prim_path=prim_path, articulation=articulation, front_camera=front_camera)
        self.controller = controller
        self._wheel_indices = None  # cached after first physics-ready call

    @classmethod
    def build(cls, prim_path: str) -> "WheeledRobot":
        """Create and configures a wheeled robot instance with necessary components.

        Constructs a wheeled robot by creating the robot prim, articulation view, differential controller,
        and front camera, then adds them to the world scene.

        Args:
            prim_path: USD prim path where the robot will be created.

        Returns:
            The configured wheeled robot instance.
        """
        add_reference_to_stage(usd_path=cls.usd_url, path=prim_path)
        stage = get_current_stage(backend="usd")
        stage.Load(prim_path)
        articulation = Articulation(prim_path)

        controller = DifferentialController(wheel_radius=cls.wheel_radius, wheel_base=cls.wheel_base)

        camera = cls.build_front_camera(prim_path)

        return cls(prim_path=prim_path, articulation=articulation, controller=controller, front_camera=camera)

    def write_action(self, step_size: float) -> None:
        """Apply wheel actions to the robot based on current controller commands.

        Uses the differential controller to convert the stored action into wheel commands
        and applies them to the robot's wheels.

        Args:
            step_size: Time step size for the action application.
        """
        if not self.is_physics_ready():
            return
        if self._wheel_indices is None:
            self._wheel_indices = self.articulation.get_joint_indices(self.wheel_dof_names)
        action = self.controller.forward(command=self.action.get_value())
        self.articulation.set_dof_velocities(action[np.newaxis], dof_indices=self._wheel_indices)


class PolicyMobilityGenRobot(MobilityGenRobot):
    """Base class for policy-driven mobility generation robots.

    This class provides a foundation for robots that use reinforcement learning policies for locomotion control,
    such as humanoid and quadruped robots. It integrates with Isaac Sim's articulation system and supports
    various robot policies for terrain navigation and movement control.

    The class handles the setup and management of policy-controlled robots within the mobility generation
    framework, including articulation management, policy integration, and camera attachment for perception.

    Args:
        prim_path: USD path where the robot prim is located in the stage.
        articulation: Articulation for managing the robot's joints and degrees of freedom.
        controller: Policy controller that determines robot movement and behavior. Supports both H1 humanoid
            and Spot quadruped terrain navigation policies.
        front_camera: Optional camera module attached to the robot for perception and data collection.
    """

    usd_url: str
    articulation_path: str

    def __init__(
        self,
        prim_path: str,
        articulation: Articulation,
        controller: H1FlatTerrainPolicy | SpotFlatTerrainPolicy,
        front_camera: Module | None = None,
    ) -> None:
        super().__init__(prim_path, articulation, front_camera)
        self.controller = controller
        self._controller_initialized = False

    @classmethod
    @abstractmethod
    def build_policy(cls, prim_path: str) -> H1FlatTerrainPolicy | SpotFlatTerrainPolicy:
        """Build the policy controller for the robot.

        Args:
            prim_path: USD prim path for the robot.

        Returns:
            Policy controller for the robot.
        """
        ...

    @classmethod
    def build(cls, prim_path: str) -> PolicyMobilityGenRobot:
        """Create and configure a policy-controlled mobility generation robot instance.

        Args:
            prim_path: USD prim path for the robot.

        Returns:
            The configured robot instance.
        """
        add_reference_to_stage(usd_path=cls.usd_url, path=prim_path)
        stage = get_current_stage(backend="usd")

        controller = cls.build_policy(prim_path)

        stage.Load(prim_path)

        camera = cls.build_front_camera(prim_path)

        return cls(prim_path=prim_path, articulation=controller.robot, controller=controller, front_camera=camera)

    def write_action(self, step_size: float) -> None:
        """Apply the current action to the robot using the policy controller.

        Args:
            step_size: Time step size for the action.
        """
        if not self.is_physics_ready():
            return
        if not self._controller_initialized:
            self.controller.initialize()
            self._controller_initialized = True
        torch = import_module("torch")
        action = self.action.get_value()
        device = torch.device(str(self.controller.robot._device))
        command = torch.tensor([action[0], 0.0, action[1]], dtype=torch.float32, device=device)
        self.controller.forward(step_size, command)

    def set_pose_2d(self, pose: Pose2d) -> None:
        """Set the robot's 2D pose and reinitialize the controller.

        Args:
            pose: The 2D pose to set for the robot.
        """
        super().set_pose_2d(pose)
        if self.is_physics_ready():
            self.controller.initialize()
            self._controller_initialized = True


@ROBOTS.register()
class JetbotRobot(WheeledMobilityGenRobot):
    """A wheeled robot implementation for the Jetbot platform in mobility generation scenarios.

    This class provides a complete implementation of the NVIDIA Jetbot robot for use in mobility generation
    workflows. It inherits from WheeledMobilityGenRobot and includes pre-configured parameters for physics
    simulation, camera setup, occupancy mapping, and control behaviors.

    The robot features differential drive control with two wheels, a front-facing Hawk camera for perception,
    and support for various control modes including keyboard, gamepad, random actions, and path following.
    It includes built-in collision detection and occupancy mapping capabilities optimized for the Jetbot's
    compact form factor.

    Key characteristics:
    - Wheel base: 0.1125 meters
    - Wheel radius: 0.03 meters
    - Physics timestep: 0.005 seconds
    - Occupancy map radius: 0.25 meters
    - Maximum linear velocity: 0.25 m/s (keyboard/gamepad)
    - Maximum angular velocity: 1.0 rad/s (keyboard/gamepad)

    The robot automatically configures its USD asset path, joint names, camera positioning, and control
    parameters. It supports chase camera functionality with configurable offset and tilt angle for
    visualization purposes.
    """

    physics_dt: float = 0.005
    """Time step for physics simulation in seconds."""

    z_offset: float = 0.1
    """Vertical offset applied to the robot's initial position."""

    chase_camera_base_path = "chassis"
    """Base path for the chase camera attachment point."""
    chase_camera_x_offset: float = -0.5
    """X-axis offset for the chase camera position relative to the base path."""
    chase_camera_z_offset: float = 0.5
    """Z-axis offset for the chase camera position relative to the base path."""
    chase_camera_tilt_angle: float = 60.0
    """Tilt angle for the chase camera in degrees."""

    occupancy_map_radius: float = 0.25
    """Radius for occupancy map detection around the robot."""
    occupancy_map_z_min: float = 0.05
    """Minimum height for occupancy map detection."""
    occupancy_map_z_max: float = 0.5
    """Maximum height for occupancy map detection."""
    occupancy_map_cell_size: float = 0.05
    """Cell size for the occupancy map grid."""
    occupancy_map_collision_radius: float = 0.25
    """Collision radius used for occupancy map calculations."""

    front_camera_base_path = "chassis/rgb_camera/front_hawk"
    """Path to the front camera's base attachment point."""
    front_camera_rotation = (0.0, 0.0, 0.0)
    """Rotation values (x, y, z) for the front camera orientation."""
    front_camera_translation = (0.0, 0.0, 0.0)
    """Translation values (x, y, z) for the front camera position."""
    front_camera_type = HawkCamera

    keyboard_linear_velocity_gain: float = 0.25
    """Gain multiplier for linear velocity when using keyboard controls."""
    keyboard_angular_velocity_gain: float = 1.0
    """Gain multiplier for angular velocity when using keyboard controls."""

    gamepad_linear_velocity_gain: float = 0.25
    """Gain multiplier for linear velocity when using gamepad controls."""
    gamepad_angular_velocity_gain: float = 1.0
    """Gain multiplier for angular velocity when using gamepad controls."""

    random_action_linear_velocity_range: tuple[float, float] = (-0.3, 0.25)
    """Range (min, max) for random linear velocity actions."""
    random_action_angular_velocity_range: tuple[float, float] = (-0.75, 0.75)
    """Range (min, max) for random angular velocity actions."""
    random_action_linear_acceleration_std: float = 1.0
    """Standard deviation for random linear acceleration noise."""
    random_action_angular_acceleration_std: float = 5.0
    """Standard deviation for random angular acceleration noise."""
    random_action_grid_pose_sampler_grid_size: float = 5.0
    """Grid size for random pose sampling."""

    path_following_speed: float = 0.25
    """Target speed for path following behavior."""
    path_following_angular_gain: float = 1.0
    """Gain for angular corrections during path following."""
    path_following_stop_distance_threshold: float = 0.5
    """Distance threshold for stopping during path following."""
    path_following_forward_angle_threshold = math.pi / 4
    """Angle threshold for forward direction during path following."""
    path_following_target_point_offset_meters: float = 1.0
    """Distance offset for target point selection during path following."""

    wheel_dof_names: list[str] = ["left_wheel_joint", "right_wheel_joint"]
    """Names of the wheel degree-of-freedom joints."""
    usd_url: str = get_assets_root_path() + "/Isaac/Robots/NVIDIA/Jetbot/jetbot.usd"
    """URL path to the USD file containing the robot's 3D model."""

    chassis_subpath: str = "chassis"
    """Subpath to the chassis component within the robot's USD structure."""
    wheel_base: float = 0.1125
    """Distance between the front and rear axles of the robot."""
    wheel_radius: float = 0.03
    """Radius of the robot's wheels."""


@ROBOTS.register()
class CarterRobot(WheeledMobilityGenRobot):
    """A mobility generation robot implementation for the NVIDIA Nova Carter wheeled robot.

    This class provides a specialized configuration of the Nova Carter robot for mobility generation
    scenarios. It inherits from WheeledMobilityGenRobot and includes pre-configured parameters for
    physics simulation, camera setup, occupancy mapping, and various control modes including keyboard,
    gamepad, random actions, and path following.

    The robot features differential drive control with a wheelbase of 0.413 meters and wheel radius
    of 0.14 meters. It includes a front-facing Hawk camera for perception and supports chase camera
    views for visualization. The configuration includes collision detection parameters and occupancy
    mapping capabilities for navigation planning.

    Key features include support for multiple input modalities (keyboard, gamepad), autonomous
    behaviors (random exploration, path following), and configurable velocity limits and acceleration
    parameters for different operational scenarios.
    """

    physics_dt: float = 0.005
    """Physics simulation timestep in seconds."""

    z_offset: float = 0.25
    """Vertical offset applied to the robot's initial position."""

    chase_camera_base_path = "chassis_link"
    """Base path for the chase camera attachment point."""
    chase_camera_x_offset: float = -1.5
    """X-axis offset for the chase camera position."""
    chase_camera_z_offset: float = 0.8
    """Z-axis offset for the chase camera position."""
    chase_camera_tilt_angle: float = 60.0
    """Tilt angle for the chase camera in degrees."""

    occupancy_map_radius: float = 0.55
    """Radius for occupancy map generation around the robot."""
    occupancy_map_z_min: float = 0.1
    """Minimum Z height for occupancy map detection."""
    occupancy_map_z_max: float = 0.62
    """Maximum Z height for occupancy map detection."""
    occupancy_map_cell_size: float = 0.05
    """Cell size for the occupancy map grid."""
    occupancy_map_collision_radius: float = 0.5
    """Collision radius used for occupancy map calculations."""

    front_camera_base_path = "chassis_link/sensors/front_hawk/front_hawk"
    """Base path for the front camera attachment point."""
    front_camera_rotation = (0.0, 0.0, 0.0)
    """Rotation values for the front camera in degrees (x, y, z)."""
    front_camera_translation = (0.0, 0.0, 0.0)
    """Translation offset for the front camera position (x, y, z)."""
    front_camera_type = HawkCamera

    keyboard_linear_velocity_gain: float = 1.0
    """Gain multiplier for keyboard linear velocity input."""
    keyboard_angular_velocity_gain: float = 1.0
    """Gain multiplier for keyboard angular velocity input."""

    gamepad_linear_velocity_gain: float = 1.0
    """Gain multiplier for gamepad linear velocity input."""
    gamepad_angular_velocity_gain: float = 1.0
    """Gain multiplier for gamepad angular velocity input."""

    random_action_linear_velocity_range: tuple[float, float] = (-0.3, 1.0)
    """Range of linear velocities for random action generation (min, max)."""
    random_action_angular_velocity_range: tuple[float, float] = (-0.75, 0.75)
    """Range of angular velocities for random action generation (min, max)."""
    random_action_linear_acceleration_std: float = 5.0
    """Standard deviation for linear acceleration noise in random actions."""
    random_action_angular_acceleration_std: float = 5.0
    """Standard deviation for angular acceleration noise in random actions."""
    random_action_grid_pose_sampler_grid_size: float = 5.0
    """Grid size for random pose sampling."""

    path_following_speed: float = 1.0
    """Target speed for path following behavior."""
    path_following_angular_gain: float = 1.0
    """Angular gain for path following control."""
    path_following_stop_distance_threshold: float = 0.5
    """Distance threshold for stopping during path following."""
    path_following_forward_angle_threshold = math.pi / 4
    """Angle threshold for forward direction during path following."""
    path_following_target_point_offset_meters: float = 1.0
    """Offset distance for target point selection in path following."""

    wheel_dof_names: list[str] = ["joint_wheel_left", "joint_wheel_right"]
    """Names of the wheel degree-of-freedom joints."""
    usd_url: str = get_assets_root_path() + "/Isaac/Robots/NVIDIA/NovaCarter/nova_carter.usd"
    """USD file path for the robot asset."""
    chassis_subpath: str = "chassis_link"
    """Subpath to the chassis component within the robot USD."""
    wheel_base = 0.413
    """Distance between the robot's wheel axles."""
    wheel_radius = 0.14
    """Radius of the robot's wheels."""


@ROBOTS.register()
class H1Robot(PolicyMobilityGenRobot):
    """A humanoid robot implementation for mobility generation and navigation tasks.

    This class represents the Unitree H1 humanoid robot configured for autonomous navigation and mobility generation scenarios. It inherits from PolicyMobilityGenRobot and uses the H1FlatTerrainPolicy for locomotion control. The robot is equipped with front-facing cameras, occupancy mapping capabilities, and supports multiple control modes including keyboard, gamepad, random actions, and path following.

    The H1Robot features a bipedal locomotion system with sophisticated terrain navigation capabilities. It includes configurable camera systems for visual navigation, occupancy mapping for obstacle avoidance, and various control parameters for different operational modes. The robot can operate in environments with varying terrain conditions and supports both manual and autonomous navigation modes.
    """

    physics_dt: float = 0.005
    """Physics simulation timestep in seconds."""

    z_offset: float = 1.05
    """Vertical offset applied to the robot's initial position."""

    chase_camera_base_path = "pelvis"
    """Base path for attaching the chase camera to the robot."""
    chase_camera_x_offset: float = -1.5
    """Horizontal distance of the chase camera behind the robot."""
    chase_camera_z_offset: float = 0.8
    """Vertical distance of the chase camera above the robot."""
    chase_camera_tilt_angle: float = 60.0
    """Downward tilt angle of the chase camera in degrees."""

    occupancy_map_radius: float = 1.0
    """Radius around the robot for occupancy map generation."""
    occupancy_map_z_min: float = 0.1
    """Minimum height for occupancy map collision detection."""
    occupancy_map_z_max: float = 2.0
    """Maximum height for occupancy map collision detection."""
    occupancy_map_cell_size: float = 0.05
    """Grid cell size for occupancy map discretization."""
    occupancy_map_collision_radius: float = 0.5
    """Collision radius used for occupancy map calculations."""

    front_camera_base_path = "d435_left_imager_link/front_camera/front"
    """Base path for attaching the front camera to the robot."""
    front_camera_rotation = (0.0, 250.0, 90.0)
    """Rotation angles (x, y, z) applied to the front camera."""
    front_camera_translation = (-0.06, 0.0, 0.0)
    """Translation offset (x, y, z) applied to the front camera."""
    front_camera_type = HawkCamera

    keyboard_linear_velocity_gain: float = 1.0
    """Multiplier for keyboard input linear velocity commands."""
    keyboard_angular_velocity_gain: float = 1.0
    """Multiplier for keyboard input angular velocity commands."""

    gamepad_linear_velocity_gain: float = 1.0
    """Multiplier for gamepad input linear velocity commands."""
    gamepad_angular_velocity_gain: float = 1.0
    """Multiplier for gamepad input angular velocity commands."""

    random_action_linear_velocity_range: tuple[float, float] = (-0.3, 1.0)
    """Range (min, max) for random linear velocity actions."""
    random_action_angular_velocity_range: tuple[float, float] = (-0.75, 0.75)
    """Range (min, max) for random angular velocity actions."""
    random_action_linear_acceleration_std: float = 5.0
    """Standard deviation for random linear acceleration noise."""
    random_action_angular_acceleration_std: float = 5.0
    """Standard deviation for random angular acceleration noise."""
    random_action_grid_pose_sampler_grid_size: float = 5.0
    """Grid size for random pose sampling."""

    path_following_speed: float = 1.0
    """Target linear velocity for path following behavior."""
    path_following_angular_gain: float = 1.0
    """Proportional gain for angular control during path following."""
    path_following_stop_distance_threshold: float = 0.5
    """Distance threshold for stopping near path targets."""
    path_following_forward_angle_threshold = math.pi / 4
    """Angle threshold for determining forward motion direction."""
    path_following_target_point_offset_meters: float = 1.0
    """Distance ahead on the path to target for following."""

    usd_url = get_assets_root_path() + "/Isaac/Robots/Unitree/H1/h1.usd"
    """File path to the robot's USD asset."""
    articulation_path = "pelvis"
    """Path to the articulation root within the robot hierarchy."""
    controller_z_offset: float = 1.05
    """Vertical offset applied to the robot controller's position."""

    @classmethod
    def build_policy(cls, prim_path: str) -> H1FlatTerrainPolicy:
        """Create and configure a flat terrain policy for the H1 robot.

        Args:
            prim_path: USD prim path where the robot is located.

        Returns:
            Configured H1FlatTerrainPolicy instance for robot control.
        """
        return H1FlatTerrainPolicy(prim_path=prim_path, position=np.array([0.0, 0.0, cls.controller_z_offset]))


@ROBOTS.register()
class SpotRobot(PolicyMobilityGenRobot):
    """Boston Dynamics Spot quadruped robot for mobility generation in Isaac Sim.

    This class provides a complete implementation of the Spot robot with legged locomotion capabilities
    for use in robotic mobility scenarios. The robot uses a policy-based controller for navigation and
    features configurable parameters for physics simulation, camera systems, and motion control.

    The robot is equipped with a front-facing camera for perception, chase camera for visualization,
    and occupancy mapping capabilities for navigation. It supports multiple control modes including
    keyboard, gamepad, random actions, and path following behaviors.
    """

    physics_dt: float = 0.005
    """Physics simulation time step in seconds."""
    z_offset: float = 0.7
    """Vertical offset applied to the robot's position in meters."""

    chase_camera_base_path = "body"
    """Base prim path for positioning the chase camera relative to the robot."""
    chase_camera_x_offset: float = -1.5
    """Forward/backward offset of the chase camera from the base position in meters."""
    chase_camera_z_offset: float = 0.8
    """Vertical offset of the chase camera from the base position in meters."""
    chase_camera_tilt_angle: float = 60.0
    """Downward tilt angle of the chase camera in degrees."""

    occupancy_map_radius: float = 1.0
    """Radius around the robot for occupancy map generation in meters."""
    occupancy_map_z_min: float = 0.1
    """Minimum height for occupancy map obstacle detection in meters."""
    occupancy_map_z_max: float = 0.62
    """Maximum height for occupancy map obstacle detection in meters."""
    occupancy_map_cell_size: float = 0.05
    """Size of each cell in the occupancy map grid in meters."""
    occupancy_map_collision_radius: float = 0.5
    """Collision radius used for occupancy map calculations in meters."""

    front_camera_base_path = "body/front_camera"
    """Base prim path for positioning the front camera relative to the robot."""
    front_camera_rotation = (180, 180, 180)
    """Rotation angles (x, y, z) applied to the front camera in degrees."""
    front_camera_translation = (0.44, 0.075, 0.01)
    """Translation offset (x, y, z) applied to the front camera in meters."""
    front_camera_type = HawkCamera

    keyboard_linear_velocity_gain: float = 1.0
    """Scaling factor for linear velocity commands from keyboard input."""
    keyboard_angular_velocity_gain: float = 1.0
    """Scaling factor for angular velocity commands from keyboard input."""

    gamepad_linear_velocity_gain: float = 1.0
    """Scaling factor for linear velocity commands from gamepad input."""
    gamepad_angular_velocity_gain: float = 1.0
    """Scaling factor for angular velocity commands from gamepad input."""

    random_action_linear_velocity_range: tuple[float, float] = (-0.3, 1.0)
    """Range (min, max) for random linear velocity commands in meters per second."""
    random_action_angular_velocity_range: tuple[float, float] = (-0.75, 0.75)
    """Range (min, max) for random angular velocity commands in radians per second."""
    random_action_linear_acceleration_std: float = 5.0
    """Standard deviation for random linear acceleration noise."""
    random_action_angular_acceleration_std: float = 5.0
    """Standard deviation for random angular acceleration noise."""
    random_action_grid_pose_sampler_grid_size: float = 5.0
    """Grid size for random pose sampling in meters."""

    path_following_speed: float = 1.0
    """Target linear speed for path following mode in meters per second."""
    path_following_angular_gain: float = 1.0
    """Proportional gain for angular velocity control during path following."""
    path_following_stop_distance_threshold: float = 0.5
    """Distance threshold for stopping at path waypoints in meters."""
    path_following_forward_angle_threshold = math.pi / 4
    """Angular threshold for considering a target point as forward-facing in radians."""
    path_following_target_point_offset_meters: float = 1.0
    """Look-ahead distance for selecting target points along the path in meters."""

    usd_url = get_assets_root_path() + "/Isaac/Robots/BostonDynamics/spot/spot.usd"
    """File path to the USD asset for the Spot robot model."""
    articulation_path = "/"
    """Relative path to the articulation root within the robot's prim hierarchy."""
    controller_z_offset: float = 0.7
    """Vertical offset for the controller's reference position in meters."""

    @classmethod
    def build_policy(cls, prim_path: str) -> SpotFlatTerrainPolicy:
        """Create and initializes a SpotFlatTerrainPolicy controller for the Spot robot.

        Args:
            prim_path: USD prim path where the Spot robot is located in the stage.

        Returns:
            Configured SpotFlatTerrainPolicy instance with the robot's position and z-offset.
        """
        return SpotFlatTerrainPolicy(prim_path=prim_path, position=np.array([0.0, 0.0, cls.controller_z_offset]))


# =========================================================
#  V2: YAML-driven multi-sensor robots
# =========================================================


class WheeledMultiSensorRobot(MobilityGenMultiSensorRobot):
    """Wheeled differential-drive robot using YAML config and a multi-sensor rig.

    Concrete subclasses set ``robot_config_path`` to a YAML file (relative to the subclass
    module file).  The YAML ``wheel`` section provides DOF names, radius, and wheelbase.

    Args:
        prim_path: USD prim path of the robot root.
        articulation: Isaac Sim articulation for the robot.
        controller: Differential drive controller.
        sensor_rig: Optional pre-built sensor rig module.
    """

    def __init__(
        self,
        prim_path: str,
        articulation: Articulation,
        controller: DifferentialController,
        sensor_rig: Module | None = None,
    ) -> None:
        super().__init__(prim_path=prim_path, articulation=articulation, sensor_rig=sensor_rig)
        self.controller = controller
        self._wheel_indices = None

    @classmethod
    def build(cls, prim_path: str) -> "WheeledMultiSensorRobot":
        """Build the wheeled robot and its sensor rig at *prim_path*.

        Args:
            prim_path: USD prim path where the robot will be created.

        Returns:
            Configured :class:`WheeledMultiSensorRobot` instance.
        """
        full_url = get_assets_root_path() + cls.usd_url
        add_reference_to_stage(usd_path=full_url, path=prim_path)
        stage = get_current_stage(backend="usd")
        stage.Load(prim_path)
        articulation = Articulation(prim_path)
        controller = DifferentialController(wheel_radius=cls.wheel_radius, wheel_base=cls.wheel_base)
        sensor_rig = cls.build_sensor_rig(prim_path)
        return cls(prim_path=prim_path, articulation=articulation, controller=controller, sensor_rig=sensor_rig)

    def write_action(self, step_size: float) -> None:
        """Apply wheel velocities based on the current action.

        Args:
            step_size: Physics timestep size in seconds.
        """
        if not self.is_physics_ready():
            return
        if self._wheel_indices is None:
            self._wheel_indices = self.articulation.get_joint_indices(self.wheel_dof_names)
        action = self.controller.forward(command=self.action.get_value())
        self.articulation.set_dof_velocities(action[np.newaxis], dof_indices=self._wheel_indices)


class PolicyMultiSensorRobot(MobilityGenMultiSensorRobot):
    """Policy-driven robot (humanoid/quadruped) using YAML config and a multi-sensor rig.

    Concrete subclasses set ``robot_config_path`` and implement :meth:`build_policy`.

    Args:
        prim_path: USD prim path of the robot root.
        articulation: Isaac Sim articulation for the robot.
        controller: Locomotion policy controller (H1 or Spot).
        sensor_rig: Optional pre-built sensor rig module.
    """

    def __init__(
        self,
        prim_path: str,
        articulation: Articulation,
        controller: H1FlatTerrainPolicy | SpotFlatTerrainPolicy,
        sensor_rig: Module | None = None,
    ) -> None:
        super().__init__(prim_path=prim_path, articulation=articulation, sensor_rig=sensor_rig)
        self.controller = controller
        self._controller_initialized = False

    @classmethod
    @abstractmethod
    def build_policy(cls, prim_path: str) -> H1FlatTerrainPolicy | SpotFlatTerrainPolicy:
        """Create the locomotion policy controller for the robot.

        Args:
            prim_path: USD prim path of the robot.

        Returns:
            Locomotion policy controller for the robot.
        """
        ...

    @classmethod
    def build(cls, prim_path: str) -> "PolicyMultiSensorRobot":
        """Build the policy robot and its sensor rig at *prim_path*.

        Args:
            prim_path: USD prim path where the robot will be created.

        Returns:
            Configured :class:`PolicyMultiSensorRobot` instance.
        """
        full_url = get_assets_root_path() + cls.usd_url
        add_reference_to_stage(usd_path=full_url, path=prim_path)
        stage = get_current_stage(backend="usd")
        controller = cls.build_policy(prim_path)
        stage.Load(prim_path)
        sensor_rig = cls.build_sensor_rig(prim_path)
        return cls(prim_path=prim_path, articulation=controller.robot, controller=controller, sensor_rig=sensor_rig)

    def write_action(self, step_size: float) -> None:
        """Apply the current action via the locomotion policy controller.

        Args:
            step_size: Physics timestep size in seconds.
        """
        if not self.is_physics_ready():
            return
        if not self._controller_initialized:
            self.controller.initialize()
            self._controller_initialized = True
        torch = import_module("torch")
        action = self.action.get_value()
        device = torch.device(str(self.controller.robot._device))
        command = torch.tensor([action[0], 0.0, action[1]], dtype=torch.float32, device=device)
        self.controller.forward(step_size, command)

    def set_pose_2d(self, pose: Pose2d) -> None:
        """Set the robot's 2D pose and reinitialize the policy controller.

        Args:
            pose: The target 2D pose (x, y, theta).
        """
        super().set_pose_2d(pose)
        if self.is_physics_ready():
            self.controller.initialize()
            self._controller_initialized = True


@ROBOTS.register()
class CarterMultiSensorRobot(WheeledMultiSensorRobot):
    """Nova Carter with full multi-sensor rig (8 Hawk cameras + 4 Owl cameras), driven by YAML config."""

    robot_config_path = "data/robots/carter.yaml"


@ROBOTS.register()
class JetbotMultiSensorRobot(WheeledMultiSensorRobot):
    """Jetbot with front Hawk camera, driven by YAML config."""

    robot_config_path = "data/robots/jetbot.yaml"


@ROBOTS.register()
class H1MultiSensorRobot(PolicyMultiSensorRobot):
    """Unitree H1 humanoid with front camera, driven by YAML config."""

    robot_config_path = "data/robots/h1.yaml"

    @classmethod
    def build_policy(cls, prim_path: str) -> H1FlatTerrainPolicy:
        """Build an H1 policy controller.

        Args:
            prim_path: USD prim path of the robot.

        Returns:
            H1 locomotion policy controller.
        """
        return H1FlatTerrainPolicy(prim_path=prim_path, position=np.array([0.0, 0.0, cls.controller_z_offset]))


@ROBOTS.register()
class SpotMultiSensorRobot(PolicyMultiSensorRobot):
    """Boston Dynamics Spot quadruped with front camera, driven by YAML config."""

    robot_config_path = "data/robots/spot.yaml"

    @classmethod
    def build_policy(cls, prim_path: str) -> SpotFlatTerrainPolicy:
        """Build a Spot policy controller.

        Args:
            prim_path: USD prim path of the robot.

        Returns:
            Spot locomotion policy controller.
        """
        return SpotFlatTerrainPolicy(prim_path=prim_path, position=np.array([0.0, 0.0, cls.controller_z_offset]))
