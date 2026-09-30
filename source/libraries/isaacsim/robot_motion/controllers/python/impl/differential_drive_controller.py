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

"""DifferentialDriveController — velocity controller for two-wheeled differential drive robots."""

from __future__ import annotations

import math
from typing import Any

import numpy as np
import warp as wp
from isaacsim.robot_motion.experimental.motion_generation import BaseController, JointState, RobotState

from ._impl.controller_differential_drive import ControllerDifferentialDrive
from ._impl.utils import _normalize
from ._logging import get_logger

_MIN_SEPARATION_DEG = 89.9
_MIN_SEPARATION_COS = math.cos(math.radians(_MIN_SEPARATION_DEG))


@wp.kernel
def _project_velocities_kernel(
    linear_velocity: wp.array[wp.float32],
    angular_velocity: wp.array[wp.float32],
    forward_direction: wp.array[wp.float32],
    rotation_direction: wp.array[wp.float32],
    linear_speed_out: wp.array[wp.float32],
    angular_speed_out: wp.array[wp.float32],
) -> None:
    """Project control point velocities onto forward/rotation axes (single-thread).

    Args:
        linear_velocity: Linear velocity command.
        angular_velocity: Angular velocity command.
        forward_direction: Forward direction vector.
        rotation_direction: Rotation direction vector.
        linear_speed_out: Output linear speed.
        angular_speed_out: Output angular speed.
    """
    lin = wp.vec3(linear_velocity[0], linear_velocity[1], linear_velocity[2])
    ang = wp.vec3(angular_velocity[0], angular_velocity[1], angular_velocity[2])
    fwd = wp.vec3(forward_direction[0], forward_direction[1], forward_direction[2])
    rot = wp.vec3(rotation_direction[0], rotation_direction[1], rotation_direction[2])
    linear_speed_out[0] = wp.dot(lin, fwd)
    angular_speed_out[0] = wp.dot(ang, rot)


class DifferentialDriveController(BaseController):
    """Single-robot differential drive controller.

    Converts a control-point velocity setpoint into left / right wheel velocity
    commands using the differential drive kinematic model::

        omega_L = (2 * v - omega * wheel_base) / (2 * wheel_radius)
        omega_R = (2 * v + omega * wheel_base) / (2 * wheel_radius)

    The scalar commands are extracted from the named site in
    ``setpoint_state.sites`` by projection:

    - ``v     = dot(forward_direction,  site.linear_velocity)``
    - ``omega = dot(rotation_direction, site.angular_velocity)``

    The control point is interpreted as the point midway between the two front
    wheels, which may differ from the robot's root (e.g. center of mass).

    Both direction vectors are normalized at construction time and must be at
    least 89.9 degrees apart.

    Args:
        robot_joint_space: Ordered list of all joint names in the robot.
        left_wheel_joint: Name of the left wheel joint in ``robot_joint_space``.
        right_wheel_joint: Name of the right wheel joint in ``robot_joint_space``.
        wheel_radius: Wheel radius [m].
        wheel_base: Lateral wheel-to-wheel distance [m].
        control_point_name: Name of the site in ``setpoint_state.sites`` from
            which linear and angular velocities are read.
        forward_direction: Robot forward axis in the body frame. Normalized at
            construction.
        rotation_direction: Yaw axis in the body frame. Normalized at construction.
            Must be at least 89.9° from ``forward_direction``.
        max_linear_speed: Forward/backward speed limit [m/s]. When ``None``, no
            limit is applied.
        max_angular_speed: Yaw-rate limit [rad/s]. When ``None``, no limit is applied.
        max_wheel_speed: Per-wheel angular velocity limit [rad/s]. When ``None``, no
            limit is applied.
        device: Warp device for internal buffers. When a CUDA device is used, a
            CUDA graph is captured at construction.

    Raises:
        ValueError: If either direction vector is zero, not 3-element, or the two
            directions are less than 89.9 degrees apart.
        ValueError: If ``wheel_radius`` or ``wheel_base`` is not strictly positive.
        ValueError: If any of ``max_linear_speed``, ``max_angular_speed``, or
            ``max_wheel_speed`` is provided and not strictly positive.
        ValueError: If ``left_wheel_joint`` or ``right_wheel_joint`` is not in
            ``robot_joint_space``.
    """

    def __init__(
        self,
        *,
        robot_joint_space: list[str],
        left_wheel_joint: str,
        right_wheel_joint: str,
        wheel_radius: float,
        wheel_base: float,
        control_point_name: str = "control_point",
        forward_direction: list[float] | np.ndarray | wp.array = (1.0, 0.0, 0.0),
        rotation_direction: list[float] | np.ndarray | wp.array = (0.0, 0.0, 1.0),
        max_linear_speed: float | None = None,
        max_angular_speed: float | None = None,
        max_wheel_speed: float | None = None,
        device: Any = None,
    ) -> None:
        fwd = _normalize(forward_direction, "forward_direction")
        rot = _normalize(rotation_direction, "rotation_direction")
        cos_angle = abs(np.dot(fwd, rot))
        if cos_angle > _MIN_SEPARATION_COS:
            raise ValueError(
                f"forward_direction and rotation_direction must be at least {_MIN_SEPARATION_DEG}° apart "
                f"(effective axis angle {math.degrees(math.acos(min(1.0, cos_angle))):.1f}°; "
                f"anti-parallel axes are treated as equivalent to parallel)."
            )

        if left_wheel_joint == right_wheel_joint:
            raise ValueError(
                f"""left and right wheel joints must be unique. Got left joint: {left_wheel_joint}, right joint:
                {right_wheel_joint}"""
            )

        if wheel_radius <= 0.0:
            raise ValueError(f"wheel_radius must be positive, got {wheel_radius}.")
        if wheel_base <= 0.0:
            raise ValueError(f"wheel_base must be positive, got {wheel_base}.")
        for name, value in (
            ("max_linear_speed", max_linear_speed),
            ("max_angular_speed", max_angular_speed),
            ("max_wheel_speed", max_wheel_speed),
        ):
            if value is not None and value <= 0.0:
                raise ValueError(f"{name} must be positive when provided, got {value}.")

        # Validate wheel joint names up front rather than failing on first forward() call.
        joint_name_set = set(robot_joint_space)
        missing = [j for j in (left_wheel_joint, right_wheel_joint) if j not in joint_name_set]
        if missing:
            raise ValueError(f"Wheel joint(s) {missing} not found in robot_joint_space.")
        device = wp.get_device(device)
        self._device = device

        self._forward_direction = wp.array(fwd.astype(np.float32), dtype=wp.float32, device=device)
        self._rotation_direction = wp.array(rot.astype(np.float32), dtype=wp.float32, device=device)

        def _opt_clamp(value: Any) -> Any:
            return wp.array([value], dtype=wp.float32, device=device) if value is not None else None

        self._controller = ControllerDifferentialDrive(
            num_robots=1,
            wheel_radius=wp.array([wheel_radius], dtype=wp.float32, device=device),
            wheel_base=wp.array([wheel_base], dtype=wp.float32, device=device),
            default_dof_indices=wp.array([0, 1], dtype=wp.uint32, device=device),
            max_linear_speed=_opt_clamp(max_linear_speed),
            max_angular_speed=_opt_clamp(max_angular_speed),
            max_wheel_speed=_opt_clamp(max_wheel_speed),
            device=device,
        )
        self._input = self._controller.input()
        self._output = self._controller.output()
        self._robot_joint_space = robot_joint_space
        self._wheel_joints = [left_wheel_joint, right_wheel_joint]
        self._control_point_name = control_point_name

        self._graph = None
        if device.is_cuda:
            # Capture a warp graph against fixed staging buffers; forward() copies
            # the live setpoint into them before each graph launch.
            self._linear_velocity_buf = wp.zeros(3, dtype=wp.float32, device=device)
            self._angular_velocity_buf = wp.zeros(3, dtype=wp.float32, device=device)
            with wp.ScopedCapture() as capture:
                self._run_kernels(self._linear_velocity_buf, self._angular_velocity_buf)
            self._graph = capture.graph

    def _run_kernels(self, linear_velocity: wp.array, angular_velocity: wp.array) -> None:
        """Launch the projection kernel then the diff-drive compute kernel.

        Args:
            linear_velocity: Device array of shape ``(3,)`` containing the
                body-frame linear velocity [m/s].
            angular_velocity: Device array of shape ``(3,)`` containing the
                body-frame angular velocity [rad/s].
        """
        wp.launch(
            _project_velocities_kernel,
            dim=1,
            inputs=[linear_velocity, angular_velocity, self._forward_direction, self._rotation_direction],
            outputs=[self._input.linear_speed_command, self._input.angular_speed_command],
            device=self._device,
        )
        self._controller.compute(inputs=self._input, outputs=self._output, dt=0.0)

    def reset(
        self,
        estimated_state: RobotState,
        setpoint_state: RobotState | None,
        t: float,
        **kwargs: object,
    ) -> bool:
        """Reset the controller.

        This controller is stateless, so reset always succeeds immediately.

        Args:
            estimated_state: Current estimated state of the robot.
            setpoint_state: Optional desired state of the robot.
            t: Current clock time [s].
            **kwargs: Unused; accepted for interface compatibility.

        Returns:
            Always ``True``.
        """
        return True

    def forward(
        self,
        estimated_state: RobotState,
        setpoint_state: RobotState | None,
        t: float,
        **kwargs: object,
    ) -> RobotState | None:
        """Compute left and right wheel velocity targets from a control-point setpoint.

        Args:
            estimated_state: Current estimated state of the robot. Not used by
                this controller but required by the ``BaseController`` interface.
            setpoint_state: Desired robot state. Must have a ``sites`` field
                containing the named control point with both linear and angular
                velocities.
            t: Current clock time [s].
            **kwargs: Unused; accepted for interface compatibility.

        Returns:
            ``RobotState`` with ``JointState`` velocity targets for the two wheel
            joints, or ``None`` if the control point site or its velocities are
            absent.
        """
        name = self._control_point_name
        sites = None if setpoint_state is None else setpoint_state.sites
        if sites is None:
            get_logger().warning(
                f"DifferentialDriveController.forward: setpoint_state.sites is None;"
                f" expected a '{name}' site; skipping."
            )
            return None

        if name not in sites.linear_velocity_names or name not in sites.angular_velocity_names:
            get_logger().warning(
                f"DifferentialDriveController.forward: control point site '{name}'"
                " not found in setpoint_state.sites linear or angular velocities; skipping."
            )
            return None

        lin_vel = sites.linear_velocities[sites.linear_velocity_names.index(name)]
        ang_vel = sites.angular_velocities[sites.angular_velocity_names.index(name)]

        if self._graph is not None:
            wp.copy(self._linear_velocity_buf, lin_vel)
            wp.copy(self._angular_velocity_buf, ang_vel)
            wp.capture_launch(self._graph)
        else:
            self._run_kernels(lin_vel, ang_vel)

        return RobotState(
            joints=JointState.from_name(
                robot_joint_space=self._robot_joint_space,
                velocities=(self._wheel_joints, self._output.joint_target_qd),
            )
        )
