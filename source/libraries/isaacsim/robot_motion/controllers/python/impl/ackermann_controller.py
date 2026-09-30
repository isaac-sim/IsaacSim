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

"""AckermannController — velocity and steering controller for Ackermann-geometry vehicles."""

from __future__ import annotations

import math
from typing import Any

import numpy as np
import warp as wp
from isaacsim.robot_motion.experimental.motion_generation import BaseController, JointState, RobotState

from ._impl.controller_ackermann import ControllerAckermann
from ._impl.utils import _normalize
from ._logging import get_logger

_MIN_SEPARATION_DEG = 89.9
_MIN_SEPARATION_COS = math.cos(math.radians(_MIN_SEPARATION_DEG))

# Below this total speed [m/s] the projection kernel holds the previous theta
# rather than recomputing it, avoiding a snap to zero during forward↔reverse
# transitions where atan2(≈0, ≈0) is numerically undefined.
_SPEED_STATIONARY_THRESHOLD = 1e-5


@wp.kernel
def _project_ackermann_inputs_stateful_kernel(
    linear_velocity: wp.array[wp.float32],
    forward_direction: wp.array[wp.float32],
    lateral_direction: wp.array[wp.float32],
    prev_theta: wp.array[wp.float32],
    linear_speed_out: wp.array[wp.float32],
    turning_angle_out: wp.array[wp.float32],
) -> None:
    """Project a velocity vector with stateful turning-angle hold near zero speed.

    When ``‖v‖ ≤ threshold`` the turning angle from the previous step is held
    rather than recomputed, preventing a snap to zero during forward↔reverse
    transitions.  When speed is above the threshold the computed angle is written
    back to ``prev_theta``.

    Args:
        linear_velocity: Linear velocity command.
        forward_direction: Forward direction vector.
        lateral_direction: Lateral direction vector.
        prev_theta: Previous steering angle.
        linear_speed_out: Output linear speed.
        turning_angle_out: Output steering angle.
    """
    lin = wp.vec3(linear_velocity[0], linear_velocity[1], linear_velocity[2])
    fwd = wp.vec3(forward_direction[0], forward_direction[1], forward_direction[2])
    lat = wp.vec3(lateral_direction[0], lateral_direction[1], lateral_direction[2])
    v_x = wp.dot(lin, fwd)
    v_y = wp.dot(lin, lat)
    speed = wp.sqrt(v_x * v_x + v_y * v_y)
    if speed > float(_SPEED_STATIONARY_THRESHOLD):
        if v_x >= 0.0:
            theta = wp.atan2(v_y, v_x)
        else:
            theta = wp.atan2(-v_y, -v_x)
        prev_theta[0] = theta
    else:
        theta = prev_theta[0]
    if v_x >= 0.0:
        linear_speed_out[0] = speed
    else:
        linear_speed_out[0] = -speed
    turning_angle_out[0] = theta


class AckermannController(BaseController):
    """Single-robot Ackermann steering controller.

    Converts a control-point linear-velocity setpoint into:

    - Steerable-wheel angular velocities and steering-angle position targets.
    - Non-steerable-wheel angular velocities (if any).

    The setpoint must be a real velocity vector at the control point.  For a vehicle
    traveling at total speed ``v`` with body turning angle ``θ``, the site's
    ``linear_velocity`` should be ``[v·cos θ, v·sin θ, 0]`` in the body frame.

    The controller recovers speed and angle by projection onto the body axes:

    - ``v_x           = dot(linear_velocity, forward_direction)``
    - ``v_y           = dot(linear_velocity, lateral_direction)``
    - ``linear_speed  = sign(v_x) · ‖[v_x, v_y]‖``
    - ``turning_angle = atan2(v_y, v_x)``   (forward, ``v_x > 0``)
    - ``turning_angle = atan2(-v_y, -v_x)`` (reversing, ``v_x ≤ 0``)

    where ``lateral_direction = normalize(cross(rotation_direction, forward_direction))``.

    Negating both components for reversing keeps ``turning_angle`` in ``(-π/2, π/2)``.
    Positive turning angle steers to the left (counterclockwise from above with the
    default ``rotation_direction = [0, 0, 1]``).

    Both direction vectors are normalized at construction time and must be at least
    89.9 degrees apart.

    Args:
        robot_joint_space: Ordered list of all joint names in the robot.
        left_steerable_wheel_joint: Name of the left steerable wheel joint in
            ``robot_joint_space`` (velocity target).
        right_steerable_wheel_joint: Name of the right steerable wheel joint in
            ``robot_joint_space`` (velocity target).
        left_steering_joint: Name of the left steering joint in ``robot_joint_space``
            (position target — the physical steering angle of the wheel).
        right_steering_joint: Name of the right steering joint in ``robot_joint_space``
            (position target).
        steerable_wheel_radius: Radius of the steerable wheels [m].
        wheel_base: Axle-to-axle distance, front to rear [m].
        track_width: Lateral wheel-to-wheel distance [m].
        left_non_steerable_wheel_joint: Name of the left non-steerable wheel joint in
            ``robot_joint_space`` (velocity target). Must be provided together with
            ``right_non_steerable_wheel_joint``. Its lateral offset is derived
            automatically as ``+non_steerable_track_width / 2``.
        right_non_steerable_wheel_joint: Name of the right non-steerable wheel joint in
            ``robot_joint_space`` (velocity target). Must be provided together with
            ``left_non_steerable_wheel_joint``. Its lateral offset is derived
            automatically as ``-non_steerable_track_width / 2``.
        non_steerable_wheel_radius: Radius of the non-steerable wheels [m]. When
            ``None``, falls back to ``steerable_wheel_radius``.
        non_steerable_track_width: Lateral distance between the non-steerable wheels
            [m]. When ``None``, falls back to ``track_width``.
        max_linear_speed: Forward/backward speed limit [m/s]. When ``None``, no
            limit is applied.
        max_turning_angle: Steering angle clamp [rad]. Must be in ``(0, π/2]``.
        steerable_wheels_at_rear: Pass ``True`` for rear-wheel-steering vehicles
            (e.g. forklifts).
        direct_command: Pass ``True`` to ignore ``setpoint_state`` in
            :meth:`forward` and read speed and angle directly from kwargs instead.
        linear_speed_kwarg: Name of the :meth:`forward` kwarg that carries the
            signed total speed [m/s] in direct mode.
        turning_angle_kwarg: Name of the :meth:`forward` kwarg that carries the
            body turning angle [rad] in direct mode.
        control_point_name: Name of the site in ``setpoint_state.sites`` from which
            ``linear_velocity`` is read. Unused when ``direct_command=True``.
        forward_direction: Robot forward axis in the body frame. Normalized at
            construction.
        rotation_direction: Yaw axis in the body frame. Normalized at construction.
            Must be at least 89.9° from ``forward_direction``. Used to derive
            ``lateral_direction`` as
            ``normalize(cross(rotation_direction, forward_direction))``.
        device: Warp device for internal buffers. Defaults to ``wp.get_device()``.

    Raises:
        ValueError: If either direction vector is zero, not 3-element, or the two
            directions are less than 89.9 degrees apart.
        ValueError: If ``steerable_wheel_radius``, ``wheel_base``, or ``track_width``
            is not strictly positive.
        ValueError: If ``non_steerable_wheel_radius`` is provided and not strictly
            positive.
        ValueError: If ``non_steerable_track_width`` is provided and not strictly
            positive.
        ValueError: If ``max_linear_speed`` is provided and not strictly positive.
        ValueError: If ``max_turning_angle`` is provided and not in ``(0, π/2]``.
        ValueError: If exactly one of ``left_non_steerable_wheel_joint`` /
            ``right_non_steerable_wheel_joint`` is provided (both or neither required).
        ValueError: If any two of the required joint names are identical.
        ValueError: If any required joint name is not in ``robot_joint_space``.
    """

    def __init__(
        self,
        *,
        robot_joint_space: list[str],
        left_steerable_wheel_joint: str,
        right_steerable_wheel_joint: str,
        left_steering_joint: str,
        right_steering_joint: str,
        steerable_wheel_radius: float,
        wheel_base: float,
        track_width: float,
        left_non_steerable_wheel_joint: str | None = None,
        right_non_steerable_wheel_joint: str | None = None,
        non_steerable_wheel_radius: float | None = None,
        non_steerable_track_width: float | None = None,
        max_linear_speed: float | None = None,
        max_turning_angle: float | None = None,
        steerable_wheels_at_rear: bool = False,
        direct_command: bool = False,
        linear_speed_kwarg: str = "linear_speed",
        turning_angle_kwarg: str = "turning_angle",
        control_point_name: str = "control_point",
        forward_direction: list[float] | np.ndarray | wp.array = (1.0, 0.0, 0.0),
        rotation_direction: list[float] | np.ndarray | wp.array = (0.0, 0.0, 1.0),
        device: wp.DeviceLike = None,
    ) -> None:
        # --- All guards first ---

        fwd = _normalize(forward_direction, "forward_direction")
        rot = _normalize(rotation_direction, "rotation_direction")
        cos_angle = abs(np.dot(fwd, rot))
        if cos_angle > _MIN_SEPARATION_COS:
            raise ValueError(
                f"forward_direction and rotation_direction must be at least {_MIN_SEPARATION_DEG}° apart "
                f"(effective axis angle {math.degrees(math.acos(min(1.0, cos_angle))):.1f}°; "
                f"anti-parallel axes are treated as equivalent to parallel)."
            )

        for param_name, value in (
            ("steerable_wheel_radius", steerable_wheel_radius),
            ("wheel_base", wheel_base),
            ("track_width", track_width),
        ):
            if value <= 0.0:
                raise ValueError(f"{param_name} must be positive, got {value}.")
        if non_steerable_wheel_radius is not None and non_steerable_wheel_radius <= 0.0:
            raise ValueError(f"non_steerable_wheel_radius must be positive, got {non_steerable_wheel_radius}.")
        if non_steerable_track_width is not None and non_steerable_track_width <= 0.0:
            raise ValueError(f"non_steerable_track_width must be positive, got {non_steerable_track_width}.")
        if max_linear_speed is not None and max_linear_speed <= 0.0:
            raise ValueError(f"max_linear_speed must be positive when provided, got {max_linear_speed}.")
        if max_turning_angle is not None:
            if max_turning_angle <= 0.0:
                raise ValueError(f"max_turning_angle must be positive when provided, got {max_turning_angle}.")
            if max_turning_angle > math.pi / 2.0:
                raise ValueError(f"max_turning_angle must not exceed π/2 (~1.5708 rad); got {max_turning_angle:.4f}.")

        if (left_non_steerable_wheel_joint is None) != (right_non_steerable_wheel_joint is None):
            raise ValueError(
                "left_non_steerable_wheel_joint and right_non_steerable_wheel_joint must both be "
                "provided or both omitted."
            )
        has_non_steerable = left_non_steerable_wheel_joint is not None
        non_steerable_wheel_joints = (
            [left_non_steerable_wheel_joint, right_non_steerable_wheel_joint] if has_non_steerable else []
        )

        all_required = [
            left_steerable_wheel_joint,
            right_steerable_wheel_joint,
            left_steering_joint,
            right_steering_joint,
        ] + non_steerable_wheel_joints
        if len(all_required) != len(set(all_required)):
            raise ValueError(f"All joint names must be unique; duplicates found in: {all_required}.")

        joint_name_set = set(robot_joint_space)
        missing = [j for j in all_required if j not in joint_name_set]
        if missing:
            raise ValueError(f"Joint(s) {missing} not found in robot_joint_space.")

        ns_tw = non_steerable_track_width if non_steerable_track_width is not None else track_width
        non_steerable_wheel_offsets = [ns_tw / 2.0, -ns_tw / 2.0] if has_non_steerable else []

        lat = np.cross(rot, fwd)
        lat = lat / np.linalg.norm(lat)

        device = wp.get_device(device)
        self._device = device
        self._forward_direction = wp.array(fwd.astype(np.float32), dtype=wp.float32, device=device)
        self._lateral_direction = wp.array(lat.astype(np.float32), dtype=wp.float32, device=device)

        self._direct_command = direct_command
        self._linear_speed_kwarg = linear_speed_kwarg
        self._turning_angle_kwarg = turning_angle_kwarg
        self._control_point_name = control_point_name
        self._robot_joint_space = robot_joint_space
        # Velocity targets: [left_steer_wheel, right_steer_wheel, left_ns, right_ns]
        self._all_wheel_joints = [
            left_steerable_wheel_joint,
            right_steerable_wheel_joint,
        ] + non_steerable_wheel_joints
        # Position targets: [left_steering, right_steering]
        self._steering_joints = [left_steering_joint, right_steering_joint]

        # --- Build ControllerAckermann ---
        n_ns = len(non_steerable_wheel_joints)
        n_wheel_outputs = 2 + n_ns

        def _opt(value: Any) -> Any:
            return wp.array([value], dtype=wp.float32, device=device) if value is not None else None

        self._controller = ControllerAckermann(
            num_robots=1,
            steerable_wheel_radius=wp.array([steerable_wheel_radius], dtype=wp.float32, device=device),
            wheel_base=wp.array([wheel_base], dtype=wp.float32, device=device),
            track_width=wp.array([track_width], dtype=wp.float32, device=device),
            default_wheel_velocity_indices=wp.array(
                np.arange(n_wheel_outputs, dtype=np.uint32), dtype=wp.uint32, device=device
            ),
            default_steering_angle_indices=wp.array([0, 1], dtype=wp.uint32, device=device),
            non_steerable_wheel_radius=(
                wp.array([non_steerable_wheel_radius], dtype=wp.float32, device=device)
                if non_steerable_wheel_radius is not None
                else None
            ),
            max_linear_speed=_opt(max_linear_speed),
            max_turning_angle=_opt(max_turning_angle),
            steerable_wheels_at_rear=(
                wp.array([steerable_wheels_at_rear], dtype=wp.bool, device=device) if steerable_wheels_at_rear else None
            ),
            non_steerable_wheel_offsets=(
                wp.array(
                    np.array([non_steerable_wheel_offsets], dtype=np.float32),
                    dtype=wp.float32,
                    device=device,
                )
                if n_ns > 0
                else None
            ),
            device=device,
        )
        self._input = self._controller.input()
        self._output = self._controller.output()

        # Stateful theta buffer: holds the previous turning angle so near-zero
        # speed transitions don't snap theta to 0.  Only used in non-direct mode.
        self._prev_theta = wp.zeros(1, dtype=wp.float32, device=device) if not direct_command else None
        self._prev_theta_zero = wp.zeros(1, dtype=wp.float32, device=device) if not direct_command else None

        # --- CUDA graphs ---
        self._graph = None
        self._direct_graph = None
        if device.is_cuda:
            if direct_command:
                # Lighter graph: copy pre-filled staging buffers into the input
                # arrays then run compute — no projection kernel.
                self._direct_speed_buf = wp.zeros(1, dtype=wp.float32, device=device)
                self._direct_angle_buf = wp.zeros(1, dtype=wp.float32, device=device)
                # Host-side buffers for the H2D copy; pre-allocated to avoid
                # per-step allocation in _forward_direct.
                self._h_direct_speed_buf = wp.zeros(1, dtype=wp.float32, device="cpu")
                self._h_direct_angle_buf = wp.zeros(1, dtype=wp.float32, device="cpu")
                with wp.ScopedCapture() as capture:
                    self._run_direct(self._direct_speed_buf, self._direct_angle_buf)
                self._direct_graph = capture.graph
            else:
                # Full graph: stateful projection (holds `_prev_theta` near zero
                # speed) → speed + angle → compute.
                self._linear_velocity_buf = wp.zeros(3, dtype=wp.float32, device=device)
                with wp.ScopedCapture() as capture:
                    self._run_kernels(self._linear_velocity_buf)
                self._graph = capture.graph

    def _run_kernels(self, linear_velocity: wp.array) -> None:
        """Launch the stateful projection kernel then the Ackermann compute kernel.

        Args:
            linear_velocity: Body-frame linear velocity of shape ``(3,)`` [m/s].
        """
        wp.launch(
            _project_ackermann_inputs_stateful_kernel,
            dim=1,
            inputs=[
                linear_velocity,
                self._forward_direction,
                self._lateral_direction,
                self._prev_theta,
            ],
            outputs=[self._input.linear_speed_command, self._input.turning_angle_command],
            device=self._device,
        )
        self._controller.compute(inputs=self._input, outputs=self._output, dt=0.0)

    def _run_direct(self, speed_buf: wp.array, angle_buf: wp.array) -> None:
        """Copy pre-filled speed/angle buffers into the input struct then compute.

        Args:
            speed_buf: Signed total speed to write into the controller input [m/s].
            angle_buf: Body turning angle to write into the controller input [rad].
        """
        wp.copy(self._input.linear_speed_command, speed_buf)
        wp.copy(self._input.turning_angle_command, angle_buf)
        self._controller.compute(inputs=self._input, outputs=self._output, dt=0.0)

    def reset(
        self,
        estimated_state: RobotState,
        setpoint_state: RobotState | None,
        t: float,
        **kwargs: object,
    ) -> bool:
        """Reset the controller.

        In non-direct mode the stored ``prev_theta`` is zeroed so the next
        command starts with a clean steering history.

        Args:
            estimated_state: Current estimated state of the robot.
            setpoint_state: Optional desired state of the robot.
            t: Current clock time [s].
            **kwargs: Unused; accepted for interface compatibility.

        Returns:
            Always ``True``.
        """
        if self._prev_theta is not None:
            wp.copy(self._prev_theta, self._prev_theta_zero)
        return True

    def forward(
        self,
        estimated_state: RobotState,
        setpoint_state: RobotState | None,
        t: float,
        **kwargs: object,
    ) -> RobotState | None:
        """Compute wheel velocity and steering targets.

        Dispatches to ``_forward_direct`` or ``_forward_site`` depending
        on the ``direct_command`` flag set at construction.

        Args:
            estimated_state: Current estimated state of the robot. Not used by
                this controller but required by the ``BaseController`` interface.
            setpoint_state: Desired robot state containing the control-point site.
                Ignored when ``direct_command=True``.
            t: Current clock time [s].
            **kwargs: In direct mode, must supply the kwargs named by
                ``linear_speed_kwarg`` (signed total speed [m/s]) and
                ``turning_angle_kwarg`` (body turning angle [rad]).

        Returns:
            ``RobotState`` whose ``JointState`` holds velocity targets for all
            wheel joints (steerable and non-steerable) in the order
            ``[left_steer, right_steer, left_ns, right_ns]``, and position
            targets for the two steering joints ``[left_steering, right_steering]``.
            ``None`` if required inputs are absent.
        """
        if self._direct_command:
            return self._forward_direct(**kwargs)
        return self._forward_site(setpoint_state)

    def _forward_direct(self, **kwargs: object) -> RobotState | None:
        """Run one step in direct-command mode.

        Args:
            **kwargs: Must contain the keys named by ``linear_speed_kwarg`` and
                ``turning_angle_kwarg``.

        Returns:
            Computed ``RobotState``, or ``None`` if either kwarg is missing.
        """
        linear_speed = kwargs.get(self._linear_speed_kwarg)
        turning_angle = kwargs.get(self._turning_angle_kwarg)
        if linear_speed is None or turning_angle is None:
            get_logger().warning(
                f"AckermannController.forward: direct_command=True requires"
                f" '{self._linear_speed_kwarg}' and '{self._turning_angle_kwarg}' kwargs; skipping."
            )
            return None
        if self._direct_graph is not None:
            self._h_direct_speed_buf.numpy()[0] = float(linear_speed)
            self._h_direct_angle_buf.numpy()[0] = float(turning_angle)
            wp.copy(self._direct_speed_buf, self._h_direct_speed_buf)
            wp.copy(self._direct_angle_buf, self._h_direct_angle_buf)
            wp.capture_launch(self._direct_graph)
        else:
            self._run_direct(
                wp.array([linear_speed], dtype=wp.float32, device=self._device),
                wp.array([turning_angle], dtype=wp.float32, device=self._device),
            )
        return self._build_result()

    def _forward_site(self, setpoint_state: RobotState | None) -> RobotState | None:
        """Run one step in site-setpoint mode.

        Args:
            setpoint_state: Desired robot state containing the control-point site.

        Returns:
            Computed ``RobotState``, or ``None`` if the site or its linear
            velocity is absent.
        """
        name = self._control_point_name
        sites = None if setpoint_state is None else setpoint_state.sites
        if sites is None:
            get_logger().warning(
                f"AckermannController.forward: setpoint_state.sites is None;" f" expected a '{name}' site; skipping."
            )
            return None
        if name not in sites.linear_velocity_names:
            get_logger().warning(
                f"AckermannController.forward: control point site '{name}'"
                " not found in setpoint_state.sites linear velocities; skipping."
            )
            return None
        lin_vel = sites.linear_velocities[sites.linear_velocity_names.index(name)]
        if self._graph is not None:
            wp.copy(self._linear_velocity_buf, lin_vel)
            wp.capture_launch(self._graph)
        else:
            self._run_kernels(lin_vel)
        return self._build_result()

    def _build_result(self) -> RobotState:
        """Assemble the output ``RobotState`` from the current kernel outputs.

        Returns:
            The resulting value.
        """
        return RobotState(
            joints=JointState.from_name(
                robot_joint_space=self._robot_joint_space,
                velocities=(self._all_wheel_joints, self._output.joint_target_qd),
                positions=(self._steering_joints, self._output.joint_target_q),
            )
        )
