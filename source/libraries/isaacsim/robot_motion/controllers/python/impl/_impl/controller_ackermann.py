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

# NOTE: Temporary vendor copy — this implementation is in the process of moving to
# ``newton.controllers`` in upcoming releases of ``isaacsim.pip.newton``.

"""ControllerAckermann — vectorized control for car/forklift-like vehicles."""

from __future__ import annotations

from typing import Any

import numpy as np
import warp as wp

from .controller import Controller
from .utils import (
    _allocate_namespace,
    _idx_max,
    _normalize_indices,
    _normalize_parameter_port,
)


@wp.kernel
def _ackermann_kernel(
    steerable_wheel_radius: wp.array[float],
    non_steerable_wheel_radius: wp.array[float],
    wheel_base: wp.array[float],
    track_width: wp.array[float],
    max_linear_speed: wp.array[float],
    max_turning_angle: wp.array[float],
    non_steerable_wheel_offsets: wp.array2d[float],
    max_n_non_steerable: int,
    n_non_steerable_per_robot: wp.array[wp.int32],
    wheel_velocity_offsets: wp.array[wp.int32],
    steerable_wheels_at_rear: wp.array[wp.bool],
    linear_speed: wp.array[float],
    linear_speed_indices: wp.array[wp.uint32],
    turning_angle: wp.array[float],
    turning_angle_indices: wp.array[wp.uint32],
    output_wheel_velocities: wp.array[float],
    output_wheel_velocity_indices: wp.array[wp.uint32],
    output_steering_angles: wp.array[float],
    output_steering_angle_indices: wp.array[wp.uint32],
) -> None:
    """Per-robot: convert (v, turning_angle) to wheel angular velocities and steering angles.

    Wheel velocity index layout: packed flat — robot r's outputs start at
    ``wheel_velocity_offsets[r]``: left steerable, right steerable, then
    ``n_non_steerable_per_robot[r]`` non-steerable wheels in order.

    Steering angle index layout: ``[2*r, 2*r+1]`` for left and right.

    Args:
        steerable_wheel_radius: Steerable wheel radius values.
        non_steerable_wheel_radius: Non steerable wheel radius values.
        wheel_base: Distance between the front and rear axles.
        track_width: Distance between the left and right wheels.
        max_linear_speed: Maximum allowed linear speed.
        max_turning_angle: Maximum allowed steering angle.
        non_steerable_wheel_offsets: Non steerable wheel offsets values.
        max_n_non_steerable: Maximum non-steerable wheel count.
        n_non_steerable_per_robot: N non steerable per robot values.
        wheel_velocity_offsets: Wheel velocity offsets values.
        steerable_wheels_at_rear: Whether each vehicle steers with its rear wheels.
        linear_speed: Commanded linear speed.
        linear_speed_indices: Indices selecting linear speed values.
        turning_angle: Commanded steering angle.
        turning_angle_indices: Indices selecting turning angle values.
        output_wheel_velocities: Output wheel velocities values.
        output_wheel_velocity_indices: Indices selecting output wheel velocity values.
        output_steering_angles: Output buffer for steering angles.
        output_steering_angle_indices: Indices selecting output steering angle values.
    """
    r = wp.tid()
    v_max = max_linear_speed[r]
    theta_max = max_turning_angle[r]
    v = wp.clamp(linear_speed[linear_speed_indices[r]], -v_max, v_max)

    # SAFETY: we must assume that theta_max does not go beyond pi/2, otherwise this
    # complicates the geometry. This should be checked before this kernel is called.
    theta = wp.clamp(turning_angle[turning_angle_indices[r]], -theta_max, theta_max)

    steerable_radius = steerable_wheel_radius[r]
    ns_radius = non_steerable_wheel_radius[r]
    l_wb = wheel_base[r]
    l_tw = track_width[r]

    s_theta = wp.sin(theta)
    c_theta = wp.cos(theta)

    omega = v * s_theta / l_wb

    # Velocity vector of the left steerable wheel:
    v_lx = v * c_theta - 0.5 * omega * l_tw
    v_ly = omega * l_wb

    # Velocity vector of the right steerable wheel:
    v_rx = v * c_theta + 0.5 * omega * l_tw
    v_ry = v_ly

    # Steering angle: use the velocity *direction* (v_x / v, v_y / v) so the
    # wheel angle is independent of whether the car moves forward or in reverse.
    # v_lx / v = c_theta - 0.5 * s_theta * l_tw / l_wb  (no v term)
    # v_ly / v = s_theta                                  (no v term)
    theta_l = wp.atan2(s_theta, c_theta - 0.5 * s_theta * l_tw / l_wb)
    theta_r = wp.atan2(s_theta, c_theta + 0.5 * s_theta * l_tw / l_wb)

    # Rear-steering vehicles invert the steer angles:
    if steerable_wheels_at_rear[r]:
        theta_l = -theta_l
        theta_r = -theta_r

    output_steering_angles[output_steering_angle_indices[2 * r + 0]] = theta_l
    output_steering_angles[output_steering_angle_indices[2 * r + 1]] = theta_r

    # Angular velocities of the steerable wheels — use the full wheel-velocity
    # vector magnitude so the expression is well-defined at all body angles,
    # including theta_crit = atan(2*L/tw) where cos(theta_l) = 0 and v_lx = 0
    # simultaneously, which would otherwise produce 0/0 = NaN.
    # v_ly is identical for both wheels (determined by yaw rate and wheelbase).
    omega_l = wp.sign(v) * wp.sqrt(v_lx * v_lx + v_ly * v_ly) / steerable_radius
    omega_r = wp.sign(v) * wp.sqrt(v_rx * v_rx + v_ry * v_ry) / steerable_radius

    base = wheel_velocity_offsets[r]
    output_wheel_velocities[output_wheel_velocity_indices[base + 0]] = omega_l
    output_wheel_velocities[output_wheel_velocity_indices[base + 1]] = omega_r

    # Non-steerable wheels: always point forward, so speed = (v*cos θ - ω*d) / ns_radius
    # where d is the signed lateral offset from the centerline (positive = left).
    # Loop bound is the fleet-wide maximum; guard skips padded slots for this robot.
    n = n_non_steerable_per_robot[r]
    for i in range(max_n_non_steerable):
        if i < n:
            d = non_steerable_wheel_offsets[r, i]
            output_wheel_velocities[output_wheel_velocity_indices[base + 2 + i]] = (v * c_theta - omega * d) / ns_radius


class ControllerAckermann(Controller):
    """Vectorized Ackermann steering controller for car-like vehicles.

    For each of N robots, converts a per-robot ``(linear_speed,
    turning_angle)`` command into the steerable-wheel angles and all wheel
    angular velocities under the Ackermann kinematic model.

    The steerable wheels produce left/right steer angles computed from the
    instantaneous turning radius; each wheel's angular velocity is the scalar
    speed along its own heading divided by the wheel radius. Non-steerable
    wheels always point forward and have their speed computed from the
    instantaneous body velocity at their lateral offset.

    **Parameter ports** — ``steerable_wheel_radius``, ``non_steerable_wheel_radius``,
    ``wheel_base``, ``track_width``, ``max_linear_speed``, ``max_turning_angle`` —
    accept either a length-N ``wp.array`` (baked at construction) or a ``str``
    (live; resolved from the input struct each step).

    Args:
        num_robots: Number of Ackermann vehicles managed.
        steerable_wheel_radius: Per-robot radius of the steerable wheels [m].
        wheel_base: Per-robot axle-to-axle distance (front to rear) [m].
        track_width: Per-robot lateral wheel-to-wheel distance [m].
        default_wheel_velocity_indices: Output indices for each robot's wheel
            velocity DOFs, total length ``sum(2 + k_i)`` where ``k_i`` is the
            non-steerable count for robot i, laid out
            ``[r0_left, r0_right, r0_ns0, ..., r1_left, r1_right, r1_ns0, ...]``.
        default_steering_angle_indices: Output indices for each robot's two
            steerable-wheel position DOFs, length ``2 * num_robots``, laid out
            ``[r0_left, r0_right, r1_left, r1_right, ...]``.
        non_steerable_wheel_radius: Per-robot radius of the non-steerable wheels
            [m]. Defaults to ``steerable_wheel_radius`` when ``None``.
        max_linear_speed: Per-robot forward/backward speed limit [m/s].
            Defaults to ``+inf``.
        max_turning_angle: Per-robot steering angle clamp [rad]. Must stay
            below π/2. Defaults to π/2.
        steerable_wheels_at_rear: Per-robot bool; ``True`` for rear-wheel-
            steering vehicles (e.g. forklifts). Defaults to all ``False``.
        non_steerable_wheel_offsets: Signed lateral offsets [m] of each
            non-steerable wheel from the vehicle centreline (positive = left),
            shape ``(num_robots, max_n_non_steerable)``. ``None`` means no
            non-steerable wheels.
        num_non_steerable_wheels_per_robot: ``wp.array[int32]`` of length
            ``num_robots`` giving the actual non-steerable count for each
            robot. Rows beyond that count in ``non_steerable_wheel_offsets``
            are ignored. ``None`` means all robots share the same count
            (inferred from ``non_steerable_wheel_offsets.shape[1]``).
        linear_speed_attr: Input port name for the body-frame forward speed
            command [m/s].
        linear_speed_idx: Per-robot index override, or ``None`` for natural
            order.
        turning_angle_attr: Input port name for the desired turning angle
            [rad].
        turning_angle_idx: Per-robot index override, or ``None`` for natural
            order.
        joint_target_qd_attr: Output port name for wheel angular velocities
            [rad/s].
        joint_target_qd_idx: Per-DOF index override for velocities, or
            ``None`` to use ``default_wheel_velocity_indices``.
        joint_target_q_attr: Output port name for steerable-wheel position
            targets [rad].
        joint_target_q_idx: Per-DOF index override for steering angles, or
            ``None`` to use ``default_steering_angle_indices``.
        device: Warp device for internal buffers.
        requires_grad: Allocate buffers with gradient support.
    """

    def __init__(
        self,
        *,
        num_robots: int,
        steerable_wheel_radius: wp.array | str,
        wheel_base: wp.array | str,
        track_width: wp.array | str,
        default_wheel_velocity_indices: wp.array,
        default_steering_angle_indices: wp.array,
        non_steerable_wheel_radius: wp.array | str | None = None,
        max_linear_speed: wp.array | str | None = None,
        max_turning_angle: wp.array | str | None = None,
        steerable_wheels_at_rear: wp.array | None = None,
        non_steerable_wheel_offsets: wp.array | None = None,
        num_non_steerable_wheels_per_robot: wp.array | None = None,
        linear_speed_attr: str = "linear_speed_command",
        linear_speed_idx: wp.array | None = None,
        turning_angle_attr: str = "turning_angle_command",
        turning_angle_idx: wp.array | None = None,
        joint_target_qd_attr: str = "joint_target_qd",
        joint_target_qd_idx: wp.array | None = None,
        joint_target_q_attr: str = "joint_target_q",
        joint_target_q_idx: wp.array | None = None,
        device: Any = None,
        requires_grad: bool = False,
    ) -> None:
        if num_robots < 1:
            raise ValueError(f"num_robots must be >= 1, got {num_robots}.")

        self._device = device if device is not None else wp.get_device()
        self._requires_grad = requires_grad
        self._num_robots = num_robots

        # Resolve per-robot non-steerable counts and the padded offset matrix.
        if non_steerable_wheel_offsets is None:
            if num_non_steerable_wheels_per_robot is not None:
                raise ValueError("num_non_steerable_wheels_per_robot requires non_steerable_wheel_offsets.")
            max_n_non_steerable = 0
            n_ns_per_robot_np = np.zeros(num_robots, dtype=np.int32)
            self._non_steerable_offsets = wp.zeros((num_robots, 1), dtype=wp.float32, device=self._device)
        else:
            if not isinstance(non_steerable_wheel_offsets, wp.array):
                raise TypeError("non_steerable_wheel_offsets must be a wp.array or None.")
            if non_steerable_wheel_offsets.ndim != 2 or non_steerable_wheel_offsets.shape[0] != num_robots:
                raise ValueError(
                    f"non_steerable_wheel_offsets must have shape (num_robots, max_n_non_steerable), "
                    f"got {non_steerable_wheel_offsets.shape}."
                )
            max_n_non_steerable = non_steerable_wheel_offsets.shape[1]
            self._non_steerable_offsets = non_steerable_wheel_offsets

            if num_non_steerable_wheels_per_robot is None:
                # Homogeneous: every robot has the same count (backward-compatible).
                n_ns_per_robot_np = np.full(num_robots, max_n_non_steerable, dtype=np.int32)
            else:
                if (
                    not isinstance(num_non_steerable_wheels_per_robot, wp.array)
                    or num_non_steerable_wheels_per_robot.dtype != wp.int32
                ):
                    raise TypeError("num_non_steerable_wheels_per_robot must be wp.array[int32].")
                if int(num_non_steerable_wheels_per_robot.size) != num_robots:
                    raise ValueError(
                        f"num_non_steerable_wheels_per_robot length "
                        f"{num_non_steerable_wheels_per_robot.size} must equal num_robots={num_robots}."
                    )
                n_ns_per_robot_np = num_non_steerable_wheels_per_robot.numpy().astype(np.int32)
                if int(np.max(n_ns_per_robot_np)) > max_n_non_steerable:
                    raise ValueError(
                        f"num_non_steerable_wheels_per_robot max value {np.max(n_ns_per_robot_np)} "
                        f"exceeds non_steerable_wheel_offsets column count {max_n_non_steerable}."
                    )

        # Prefix-sum of (2 + k_i) gives each robot's base offset into the
        # flat wheel-velocity index array.
        wheel_velocity_offsets_np = np.zeros(num_robots, dtype=np.int32)
        total_wheel_outputs = 0
        for i in range(num_robots):
            wheel_velocity_offsets_np[i] = total_wheel_outputs
            total_wheel_outputs += 2 + int(n_ns_per_robot_np[i])

        self._max_n_non_steerable = max_n_non_steerable
        self._n_non_steerable_per_robot = wp.array(n_ns_per_robot_np, dtype=wp.int32, device=self._device)
        self._wheel_velocity_offsets = wp.array(wheel_velocity_offsets_np, dtype=wp.int32, device=self._device)

        # Validate index arrays.
        for arr, expected, label in (
            (default_wheel_velocity_indices, total_wheel_outputs, "default_wheel_velocity_indices"),
            (default_steering_angle_indices, 2 * num_robots, "default_steering_angle_indices"),
        ):
            if not isinstance(arr, wp.array) or arr.dtype != wp.uint32:
                raise TypeError(f"{label} must be wp.array[uint32].")
            if int(arr.size) != expected:
                raise ValueError(f"{label} length {arr.size} must equal {expected}.")
            arr_np = arr.numpy()
            if len(np.unique(arr_np)) != len(arr_np):
                raise ValueError(f"{label} contains duplicate indices — two outputs would write to the same slot.")

        # Per-robot boolean: steer with rear wheels?
        if steerable_wheels_at_rear is None:
            self._steerable_at_rear = wp.zeros(num_robots, dtype=wp.bool, device=self._device)
        else:
            if not isinstance(steerable_wheels_at_rear, wp.array) or steerable_wheels_at_rear.dtype != wp.bool:
                raise TypeError("steerable_wheels_at_rear must be wp.array[bool] or None.")
            if int(steerable_wheels_at_rear.size) != num_robots:
                raise ValueError(
                    f"steerable_wheels_at_rear length {steerable_wheels_at_rear.size} must equal "
                    f"num_robots={num_robots}."
                )
            self._steerable_at_rear = steerable_wheels_at_rear

        self._num_wheel_outputs = total_wheel_outputs
        self._num_steering_outputs = 2 * num_robots

        # Output ports.
        default_robot_idx = wp.array(np.arange(num_robots, dtype=np.uint32), device=self._device)
        self._default_robot_idx = default_robot_idx

        self._joint_target_qd_attr = joint_target_qd_attr
        self._wheel_velocity_idx = _normalize_indices(
            joint_target_qd_idx, default_wheel_velocity_indices, name="joint_target_qd"
        )
        self._joint_target_q_attr = joint_target_q_attr
        self._steering_angle_idx = _normalize_indices(
            joint_target_q_idx, default_steering_angle_indices, name="joint_target_q"
        )

        # Live command input ports.
        self._linear_speed_attr = linear_speed_attr
        self._linear_speed_idx = _normalize_indices(linear_speed_idx, default_robot_idx, name="linear_speed")
        self._turning_angle_attr = turning_angle_attr
        self._turning_angle_idx = _normalize_indices(turning_angle_idx, default_robot_idx, name="turning_angle")

        # Parameter ports: defaults for the clamps.
        if max_linear_speed is None:
            max_linear_speed = wp.full(num_robots, value=wp.inf, dtype=wp.float32, device=self._device)
        if max_turning_angle is None:
            max_turning_angle = wp.full(num_robots, value=np.pi / 2.0, dtype=wp.float32, device=self._device)

        if non_steerable_wheel_radius is None:
            non_steerable_wheel_radius = steerable_wheel_radius
        self._steerable_wheel_radius_attr, self._steerable_wheel_radius_baked = _normalize_parameter_port(
            steerable_wheel_radius, num_robots, wp.float32, self._device, requires_grad, name="steerable_wheel_radius"
        )
        self._non_steerable_wheel_radius_attr, self._non_steerable_wheel_radius_baked = _normalize_parameter_port(
            non_steerable_wheel_radius,
            num_robots,
            wp.float32,
            self._device,
            requires_grad,
            name="non_steerable_wheel_radius",
        )
        self._wheel_base_attr, self._wheel_base_baked = _normalize_parameter_port(
            wheel_base, num_robots, wp.float32, self._device, requires_grad, name="wheel_base"
        )
        self._track_width_attr, self._track_width_baked = _normalize_parameter_port(
            track_width, num_robots, wp.float32, self._device, requires_grad, name="track_width"
        )
        self._max_linear_speed_attr, self._max_linear_speed_baked = _normalize_parameter_port(
            max_linear_speed, num_robots, wp.float32, self._device, requires_grad, name="max_linear_speed"
        )
        self._max_turning_angle_attr, self._max_turning_angle_baked = _normalize_parameter_port(
            max_turning_angle, num_robots, wp.float32, self._device, requires_grad, name="max_turning_angle"
        )
        if self._max_turning_angle_baked is not None:

            if float(np.min(self._max_turning_angle_baked.numpy())) <= 0.0:
                raise ValueError("max_turning_angle must all be positive numbers; ")

            # Compare in float32 so the float32 representation of π/2 is not
            # rejected by float64 rounding when users pass the default limit.
            half_pi_f32 = float(np.float32(np.pi / 2.0))
            if float(np.max(self._max_turning_angle_baked.numpy())) > half_pi_f32:
                raise ValueError(
                    "max_turning_angle must not exceed π/2 (~1.5708 rad); "
                    "values beyond π/2 produce undefined Ackermann geometry."
                )

        if self._max_linear_speed_baked is not None:

            if float(np.min(self._max_linear_speed_baked.numpy())) <= 0.0:
                raise ValueError("max_linear_speed must all be positive numbers; ")

        self._input_specs: list[tuple[str, Any, int]] = [
            (self._linear_speed_attr, wp.float32, _idx_max(self._linear_speed_idx)),
            (self._turning_angle_attr, wp.float32, _idx_max(self._turning_angle_idx)),
        ]
        for attr in (
            self._steerable_wheel_radius_attr,
            self._non_steerable_wheel_radius_attr,
            self._wheel_base_attr,
            self._track_width_attr,
            self._max_linear_speed_attr,
            self._max_turning_angle_attr,
        ):
            if attr is not None:
                self._input_specs.append((attr, wp.float32, num_robots))

        self._output_specs: list[tuple[str, Any, int]] = [
            (self._joint_target_qd_attr, wp.float32, _idx_max(self._wheel_velocity_idx)),
            (self._joint_target_q_attr, wp.float32, _idx_max(self._steering_angle_idx)),
        ]

    @property
    def num_robots(self) -> int:
        return self._num_robots

    @property
    def device(self) -> Any:
        return self._device

    @property
    def requires_grad(self) -> bool:
        return self._requires_grad

    def is_stateful(self) -> bool:
        return False

    def is_graphable(self) -> bool:
        return True

    def input(self) -> Any:
        return _allocate_namespace(self._input_specs, self._device, self._requires_grad)

    def output(self) -> Any:
        return _allocate_namespace(self._output_specs, self._device, self._requires_grad)

    def compute(
        self,
        *,
        inputs: Any,
        outputs: Any,
        dt: float,
    ) -> None:
        linear_speed = getattr(inputs, self._linear_speed_attr)
        turning_angle = getattr(inputs, self._turning_angle_attr)
        steerable_wheel_radius = (
            self._steerable_wheel_radius_baked
            if self._steerable_wheel_radius_baked is not None
            else getattr(inputs, self._steerable_wheel_radius_attr)
        )
        non_steerable_wheel_radius = (
            self._non_steerable_wheel_radius_baked
            if self._non_steerable_wheel_radius_baked is not None
            else getattr(inputs, self._non_steerable_wheel_radius_attr)
        )
        wheel_base = (
            self._wheel_base_baked if self._wheel_base_baked is not None else getattr(inputs, self._wheel_base_attr)
        )
        track_width = (
            self._track_width_baked if self._track_width_baked is not None else getattr(inputs, self._track_width_attr)
        )
        max_linear_speed = (
            self._max_linear_speed_baked
            if self._max_linear_speed_baked is not None
            else getattr(inputs, self._max_linear_speed_attr)
        )
        max_turning_angle = (
            self._max_turning_angle_baked
            if self._max_turning_angle_baked is not None
            else getattr(inputs, self._max_turning_angle_attr)
        )
        wheel_velocities = getattr(outputs, self._joint_target_qd_attr)
        steering_angles = getattr(outputs, self._joint_target_q_attr)

        wp.launch(
            _ackermann_kernel,
            dim=self._num_robots,
            inputs=[
                steerable_wheel_radius,
                non_steerable_wheel_radius,
                wheel_base,
                track_width,
                max_linear_speed,
                max_turning_angle,
                self._non_steerable_offsets,
                self._max_n_non_steerable,
                self._n_non_steerable_per_robot,
                self._wheel_velocity_offsets,
                self._steerable_at_rear,
                linear_speed,
                self._linear_speed_idx,
                turning_angle,
                self._turning_angle_idx,
            ],
            outputs=[
                wheel_velocities,
                self._wheel_velocity_idx,
                steering_angles,
                self._steering_angle_idx,
            ],
            device=self._device,
        )
