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

"""HolonomicController — velocity controller for omnidirectional holonomic drive robots."""

from __future__ import annotations

from typing import Any

import numpy as np
import warp as wp
from isaacsim.core.experimental.utils import transform as transform_utils
from isaacsim.robot_motion.experimental.motion_generation import BaseController, JointState, RobotState, SpatialState

from ._impl.controller_holonomic_drive import ControllerHolonomicDrive
from ._logging import get_logger


def _build_plane_basis(normal: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    """Build an arbitrary orthonormal basis of the plane perpendicular to ``normal``.

    The twist is expressed as ``[v_u, v_v, w]``, so two in-plane axes are needed to
    decompose the commanded linear velocity.  Any orthonormal pair spanning the plane
    yields identical wheel speeds — the two in-plane terms of ``M @ twist`` sum to the
    projection of the velocity onto the plane, which is basis-independent — so the
    basis is chosen internally rather than being asked of the caller.

    Args:
        normal: Unit-length plane normal.

    Returns:
        Tuple of two orthonormal in-plane axes.
    """
    seed = np.zeros(3, dtype=np.float64)
    seed[int(np.argmin(np.abs(normal)))] = 1.0
    axis_u = seed - np.dot(seed, normal) * normal
    axis_u = axis_u / np.linalg.norm(axis_u)
    axis_v = np.cross(normal, axis_u)
    return axis_u, axis_v / np.linalg.norm(axis_v)


def _rotation_about_axis(axis: np.ndarray, angle: float) -> np.ndarray:
    """Return the 3×3 rotation matrix for rotating ``angle`` radians about ``axis`` (Rodrigues).

    Args:
        axis: Rotation axis.
        angle: Rotation angle in radians.

    Returns:
        The resulting value.
    """
    axis = axis / np.linalg.norm(axis)
    skew = np.array(
        [[0.0, -axis[2], axis[1]], [axis[2], 0.0, -axis[0]], [-axis[1], axis[0], 0.0]],
        dtype=np.float64,
    )
    return np.eye(3) + np.sin(angle) * skew + (1.0 - np.cos(angle)) * (skew @ skew)


def _build_kinematics(
    wheel_positions: np.ndarray,
    wheel_orientations: np.ndarray,
    wheel_radius: np.ndarray,
    mecanum_angles: np.ndarray,
    wheel_axis: np.ndarray,
    rotation_direction: np.ndarray,
    plane_axis_u: np.ndarray,
    plane_axis_v: np.ndarray,
) -> tuple[np.ndarray, np.ndarray]:
    """Precompute the kinematic matrix M (N×3) and conversion diagonal K (N,).

    ``M`` maps a command-site twist ``[v_u, v_v, w]`` to per-wheel contact speeds via the
    rigid-body relation ``u_i = a_i · (v_c + ω × r_i)``, where the twist components are
    projected onto ``plane_axis_u``, ``plane_axis_v``, and ``rotation_direction``.
    All geometry is expected in the command-site frame, so ``r_i`` is the moment arm
    about the command site.  ``K`` converts each contact speed to a wheel joint angular
    velocity: ``φ̇_i = u_i / (r_i · cos γ_i)``.

    Args:
        wheel_positions: Wheel positions relative to the robot.
        wheel_orientations: Wheel orientations relative to the robot.
        wheel_radius: Wheel radius.
        mecanum_angles: Mecanum roller angles.
        wheel_axis: Wheel rotation axis.
        rotation_direction: Rotation direction vector.
        plane_axis_u: First axis spanning the wheel plane.
        plane_axis_v: Second axis spanning the wheel plane.

    Returns:
        The resulting value.

    Raises:
        ValueError: If any wheel's axle is parallel to ``rotation_direction`` or its mecanum
            angle corresponds to ``cos γ = 0`` (0 or 180 degrees).
    """
    num_wheels = len(wheel_positions)
    m = np.zeros((num_wheels, 3), dtype=np.float64)
    k = np.zeros(num_wheels, dtype=np.float64)

    for i in range(num_wheels):
        rotation = transform_utils.quaternion_to_rotation_matrix(wheel_orientations[i]).numpy()
        axle = rotation @ wheel_axis
        axle = axle / np.linalg.norm(axle)

        cross = np.cross(axle, rotation_direction)
        cos_alpha = float(np.linalg.norm(cross))
        if cos_alpha < 1e-9:
            raise ValueError(f"Wheel {i}: axle is parallel to rotation_direction; rolling direction is undefined.")
        rolling_dir = cross / cos_alpha

        gamma = np.deg2rad(float(mecanum_angles[i]) - 90.0)
        cos_gamma = float(np.cos(gamma))
        if abs(cos_gamma) < 1e-9:
            raise ValueError(
                f"Wheel {i}: mecanum angle is 0 or 180 deg; the wheel cannot drive along its no-slip axis."
            )
        a = _rotation_about_axis(rotation_direction, gamma) @ rolling_dir

        r_i = wheel_positions[i]
        m[i, :] = [
            float(np.dot(a, plane_axis_u)),
            float(np.dot(a, plane_axis_v)),
            float(np.dot(a, np.cross(rotation_direction, r_i))),
        ]
        k[i] = 1.0 / (wheel_radius[i] * cos_gamma)

    return m, k


@wp.kernel
def _extract_twist_kernel(
    linear_velocity: wp.array[wp.float32],
    angular_velocity: wp.array[wp.float32],
    plane_axis_u: wp.array[wp.float32],
    plane_axis_v: wp.array[wp.float32],
    rotation_direction: wp.array[wp.float32],
    twist_out: wp.array[wp.float32],
) -> None:
    """Project command-site velocities onto the command-site twist axes (single thread).

    ``twist_out[0] = dot(linear_velocity, plane_axis_u)``
    ``twist_out[1] = dot(linear_velocity, plane_axis_v)``
    ``twist_out[2] = dot(angular_velocity, rotation_direction)``

    Args:
        linear_velocity: Linear velocity command.
        angular_velocity: Angular velocity command.
        plane_axis_u: First axis spanning the wheel plane.
        plane_axis_v: Second axis spanning the wheel plane.
        rotation_direction: Rotation direction vector.
        twist_out: Clamped output twist.
    """
    lin = wp.vec3(linear_velocity[0], linear_velocity[1], linear_velocity[2])
    ang = wp.vec3(angular_velocity[0], angular_velocity[1], angular_velocity[2])
    axis_u = wp.vec3(plane_axis_u[0], plane_axis_u[1], plane_axis_u[2])
    axis_v = wp.vec3(plane_axis_v[0], plane_axis_v[1], plane_axis_v[2])
    up = wp.vec3(rotation_direction[0], rotation_direction[1], rotation_direction[2])
    twist_out[0] = wp.dot(lin, axis_u)
    twist_out[1] = wp.dot(lin, axis_v)
    twist_out[2] = wp.dot(ang, up)


class HolonomicController(BaseController):
    """Holonomic (omni / mecanum) drive controller.

    Converts a planar twist setpoint at a named command site into per-wheel
    angular velocities using the closed-form inverse kinematics of the wheel base,
    and returns them as a ``RobotState`` whose joint state holds velocity targets
    for the wheel joints.

    The command is read from the named site in ``setpoint_state.sites`` and is
    interpreted as the twist of the **command site**: ``linear_velocity`` is the
    velocity of the site's origin and ``angular_velocity`` is taken about
    ``rotation_direction``.

    The command site is placed by ``command_site_position`` and
    ``command_site_quaternion``, given in the same frame as ``wheel_positions`` and
    ``wheel_orientations``.  The wheel geometry is transformed into the site frame at
    construction, so the wheels may be measured in whatever frame is convenient (a
    robot root, a USD center-of-mass prim) and the site moved independently of them.
    ``rotation_direction`` is then expressed in the command-site frame, so the default
    ``[0, 0, 1]`` means "the site frame's own +Z".

    The kinematics are precomputed once at construction into two constant operators:

    * ``M`` (N×3) maps the command-site twist ``[v_u, v_v, w]`` to per-wheel
      no-slip-axis contact speeds ``u`` via ``u_i = a_iᵀ (v_c + ω × r_i)``.
    * ``K`` (diagonal, length N) converts each contact speed to a wheel joint angular
      velocity: ``φ̇_i = u_i / (r_i · cos γ_i)``, where ``γ_i`` is the roller offset
      angle (``mecanum_angle_i − 90°``).

    On a CUDA device the full forward pass (project twist → clamp → matrix multiply)
    is captured into a CUDA graph at construction time.  Each :meth:`forward` call
    then copies the live setpoint into staging buffers and launches the graph with
    no CPU–GPU round-trips.

    Args:
        robot_joint_space: The ordered list of joint names defining the joint space
            of the controlled robot (for example, ``Articulation.dof_names``).
        wheel_joint_names: Names of the wheel joints, one per wheel, ordered to
            match ``wheel_positions[i]`` / ``wheel_orientations[i]`` /
            ``mecanum_angles[i]``.  Each name must be unique and present in
            ``robot_joint_space``.
        wheel_radius: Radius of each wheel (scalar broadcast to all wheels, or
            per-wheel array).
        wheel_positions: Positions of each wheel, in the same frame as
            ``command_site_position``.
        wheel_orientations: Quaternion orientations of each wheel, in the same frame
            as ``command_site_quaternion``, in ``[w, x, y, z]`` order.
        mecanum_angles: Mecanum roller angle of each wheel in degrees, measured
            from the wheel axle (90 = omni / plain wheel, 45 or 135 = standard
            mecanum).  This is the legacy ``isaacmecanumwheel:angle`` convention
            returned by ``HolonomicRobotUsdSetup``, so it can be passed straight
            through.  Scalar broadcast or per-wheel array.
        wheel_axis: Local rotation (spin) axis of the wheel joint.
        command_site_position: Position of the command site, in the same frame as
            ``wheel_positions``. Defaults to the origin of that frame.
        command_site_quaternion: Orientation of the command site, in the same frame as
            ``wheel_orientations``, in ``[w, x, y, z]`` order. Defaults to identity,
            in which case the wheel frame is used as the command-site frame directly.
        rotation_direction: Yaw axis, expressed in the command-site frame. Defaults to
            that frame's ``[0, 0, 1]``.
        max_linear_speed: Maximum planar linear speed [m/s]. ``None`` means no
            limit.
        max_angular_speed: Maximum yaw rate [rad/s]. ``None`` means no limit.
        max_wheel_speed: Maximum individual wheel angular velocity [rad/s].
            ``None`` means no limit.  Applied independently per wheel after the
            matrix multiply; this can distort the commanded direction.
        control_point_name: Name of the site in ``setpoint_state.sites`` from
            which the twist command is read.
        device: Warp device for internal buffers. If the device is a CUDA device,
            a CUDA graph is captured at construction time.

    Raises:
        ValueError: If ``wheel_radius``, ``wheel_positions``, or
            ``wheel_orientations`` is ``None``.
        ValueError: If ``wheel_joint_names`` contains duplicates, has a name not
            in ``robot_joint_space``, or does not have one entry per wheel.
        ValueError: If ``rotation_direction`` is zero, ``command_site_position`` does
            not have shape ``(3,)``, or ``command_site_quaternion`` does not have
            shape ``(4,)`` or is zero.
        ValueError: If any wheel's axle is parallel to ``rotation_direction`` (undefined
            rolling direction) or its mecanum angle is 0 or 180 degrees.
        ValueError: If any provided speed limit is not strictly positive.
    """

    def __init__(
        self,
        *,
        robot_joint_space: list[str],
        wheel_joint_names: list[str],
        wheel_radius: list | np.ndarray | None = None,
        wheel_positions: list | np.ndarray | None = None,
        wheel_orientations: list | np.ndarray | None = None,
        mecanum_angles: list | np.ndarray | None = None,
        wheel_axis: list | np.ndarray | None = None,
        command_site_position: list | np.ndarray | None = None,
        command_site_quaternion: list | np.ndarray | None = None,
        rotation_direction: list | np.ndarray | None = None,
        max_linear_speed: float | None = None,
        max_angular_speed: float | None = None,
        max_wheel_speed: float | None = None,
        control_point_name: str = "control_point",
        device: wp.DeviceLike = None,
    ) -> None:
        for param_name, param_val in (
            ("wheel_radius", wheel_radius),
            ("wheel_positions", wheel_positions),
            ("wheel_orientations", wheel_orientations),
        ):
            if param_val is None:
                raise ValueError(f"{param_name} is required (received None). Pass an array with one entry per wheel.")

        for name, value in (
            ("max_linear_speed", max_linear_speed),
            ("max_angular_speed", max_angular_speed),
            ("max_wheel_speed", max_wheel_speed),
        ):
            if value is not None and value <= 0.0:
                raise ValueError(f"{name} must be positive when provided, got {value}.")

        if wheel_axis is None:
            wheel_axis = np.array([1.0, 0.0, 0.0])
        if rotation_direction is None:
            rotation_direction = np.array([0.0, 0.0, 1.0])
        if command_site_position is None:
            command_site_position = np.zeros(3)
        if command_site_quaternion is None:
            command_site_quaternion = np.array([1.0, 0.0, 0.0, 0.0])

        rot_arr = np.asarray(rotation_direction, dtype=np.float64)
        up_norm = np.linalg.norm(rot_arr)
        if up_norm == 0.0:
            raise ValueError("rotation_direction must be non-zero.")
        rot_arr = rot_arr / up_norm

        site_position_arr = np.asarray(command_site_position, dtype=np.float64)
        if site_position_arr.shape != (3,):
            raise ValueError(f"command_site_position must have shape (3,), got {site_position_arr.shape}.")

        site_quaternion_arr = np.asarray(command_site_quaternion, dtype=np.float64)
        if site_quaternion_arr.shape != (4,):
            raise ValueError(f"command_site_quaternion must have shape (4,), got {site_quaternion_arr.shape}.")
        site_quaternion_norm = np.linalg.norm(site_quaternion_arr)
        if site_quaternion_norm == 0.0:
            raise ValueError("command_site_quaternion must be non-zero.")
        site_quaternion_arr = site_quaternion_arr / site_quaternion_norm

        axis_u_arr, axis_v_arr = _build_plane_basis(rot_arr)

        wheel_axis_arr = np.asarray(wheel_axis, dtype=np.float64)
        if np.linalg.norm(wheel_axis_arr) == 0.0:
            raise ValueError("wheel_axis must be non-zero.")

        wheel_positions = np.asarray(wheel_positions, dtype=np.float64)
        num_wheels = len(wheel_positions)

        if len(set(wheel_joint_names)) != len(wheel_joint_names):
            raise ValueError("wheel_joint_names must be unique.")
        for joint in wheel_joint_names:
            if joint not in robot_joint_space:
                raise ValueError(f"Wheel joint '{joint}' is not in robot_joint_space.")
        if len(wheel_joint_names) != num_wheels:
            raise ValueError(
                f"wheel_joint_names must have one entry per wheel ({num_wheels}), got {len(wheel_joint_names)}."
            )

        wheel_radius_arr = np.asarray(wheel_radius, dtype=np.float64)
        wheel_radius_arr = (
            np.full(num_wheels, float(wheel_radius_arr)) if wheel_radius_arr.size == 1 else wheel_radius_arr
        )
        if wheel_radius_arr.size != num_wheels:
            raise ValueError(f"wheel_radius length {wheel_radius_arr.size} does not match num_wheels={num_wheels}.")
        if np.any(wheel_radius_arr <= 0.0):
            raise ValueError(f"All wheel radii must be positive; got {wheel_radius_arr.tolist()}.")

        wheel_orientations_arr = np.asarray(wheel_orientations, dtype=np.float64)
        if wheel_orientations_arr.shape != (num_wheels, 4):
            raise ValueError(
                f"wheel_orientations must have shape ({num_wheels}, 4), got {wheel_orientations_arr.shape}."
            )

        mecanum_angles_arr = (
            np.full(num_wheels, 90.0) if mecanum_angles is None else np.asarray(mecanum_angles, dtype=np.float64)
        )
        mecanum_angles_arr = (
            np.full(num_wheels, float(mecanum_angles_arr)) if mecanum_angles_arr.size == 1 else mecanum_angles_arr
        )
        if mecanum_angles_arr.size != num_wheels:
            raise ValueError(f"mecanum_angles length {mecanum_angles_arr.size} does not match num_wheels={num_wheels}.")

        # Re-express the wheel geometry in the command-site frame, so that the kinematics
        # take the site as the origin of the moment arms and as the frame of the command.
        site_quaternion_inv = transform_utils.quaternion_conjugate(site_quaternion_arr).numpy()
        wheel_positions = transform_utils.rotate_vectors_by_quaternion(
            wheel_positions - site_position_arr, site_quaternion_inv
        ).numpy()
        wheel_orientations_arr = transform_utils.quaternion_multiplication(
            np.tile(site_quaternion_inv, (num_wheels, 1)), wheel_orientations_arr
        ).numpy()

        m, k = _build_kinematics(
            wheel_positions=wheel_positions,
            wheel_orientations=wheel_orientations_arr,
            wheel_radius=wheel_radius_arr,
            mecanum_angles=mecanum_angles_arr,
            wheel_axis=wheel_axis_arr,
            rotation_direction=rot_arr,
            plane_axis_u=axis_u_arr,
            plane_axis_v=axis_v_arr,
        )

        self._m = m
        self._k = k
        self._robot_joint_space = list(robot_joint_space)
        self._wheel_joint_names = list(wheel_joint_names)
        self._control_point_name = control_point_name

        device = wp.get_device(device)
        self._device = device

        self._plane_axis_u = wp.array(axis_u_arr.astype(np.float32), dtype=wp.float32, device=device)
        self._plane_axis_v = wp.array(axis_v_arr.astype(np.float32), dtype=wp.float32, device=device)
        self._rotation_direction = wp.array(rot_arr.astype(np.float32), dtype=wp.float32, device=device)

        def _opt_speed(value: Any) -> Any:
            return wp.array([value], dtype=wp.float32, device=device) if value is not None else None

        self._controller = ControllerHolonomicDrive(
            num_wheels=num_wheels,
            kinematics_m=m,
            kinematics_k=k,
            default_dof_indices=wp.array(np.arange(num_wheels, dtype=np.uint32), dtype=wp.uint32, device=device),
            max_linear_speed=_opt_speed(max_linear_speed),
            max_angular_speed=_opt_speed(max_angular_speed),
            max_wheel_speed=_opt_speed(max_wheel_speed),
            device=device,
        )
        self._input = self._controller.input()
        self._output = self._controller.output()

        self._graph = None
        if device.is_cuda:
            self._linear_velocity_buf = wp.zeros(3, dtype=wp.float32, device=device)
            self._angular_velocity_buf = wp.zeros(3, dtype=wp.float32, device=device)
            with wp.ScopedCapture() as capture:
                self._run_kernels(self._linear_velocity_buf, self._angular_velocity_buf)
            self._graph = capture.graph

    def _run_kernels(self, linear_velocity: wp.array, angular_velocity: wp.array) -> None:
        """Launch the extraction kernel then the holonomic compute kernels.

        Args:
            linear_velocity: Device array of shape ``(3,)`` — command-site linear
                velocity [m/s].
            angular_velocity: Device array of shape ``(3,)`` — command-site angular
                velocity [rad/s].
        """
        wp.launch(
            _extract_twist_kernel,
            dim=1,
            inputs=[
                linear_velocity,
                angular_velocity,
                self._plane_axis_u,
                self._plane_axis_v,
                self._rotation_direction,
            ],
            outputs=[self._input.twist_command],
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

        The holonomic controller is stateless, so this always succeeds.

        Args:
            estimated_state: Current estimated state of the robot (unused).
            setpoint_state: Desired setpoint state (unused).
            t: Current clock time (unused).
            **kwargs: Additional keyword arguments (unused).

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
        """Convert a command-site twist setpoint into per-wheel velocity targets.

        Args:
            estimated_state: Current estimated state of the robot (unused).
            setpoint_state: Desired robot state containing the named command-site
                with both linear and angular velocities.
            t: Current clock time (unused).
            **kwargs: Additional keyword arguments (unused).

        Returns:
            ``RobotState`` whose joint state contains velocity targets for
            the wheel joints, or ``None`` if the command-site or its velocities
            are absent.
        """
        name = self._control_point_name
        sites: SpatialState | None = None if setpoint_state is None else setpoint_state.sites
        if sites is None:
            get_logger().warning(
                f"HolonomicController.forward: setpoint_state.sites is None;" f" expected a '{name}' site; skipping."
            )
            return None

        if name not in sites.linear_velocity_names or name not in sites.angular_velocity_names:
            get_logger().warning(
                f"HolonomicController.forward: control point site '{name}'"
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
                velocities=(
                    self._wheel_joint_names,
                    self._output.joint_target_qd,
                ),
            )
        )
