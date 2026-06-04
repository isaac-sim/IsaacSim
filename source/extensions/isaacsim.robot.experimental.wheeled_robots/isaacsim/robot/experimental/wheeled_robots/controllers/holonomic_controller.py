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

"""Holonomic (e.g. mecanum) controller for wheeled robots."""

from __future__ import annotations

import carb
import numpy as np
import osqp
from isaacsim.core.experimental.utils import transform as transform_utils
from pxr import Gf
from scipy import sparse


class HolonomicController:
    """QP-based holonomic controller for mecanum-wheeled robots.

    Convert [forward, lateral, yaw] velocity commands into per-wheel angular
    velocities by solving a quadratic program.

    Args:
        wheel_radius: Radius of each wheel (scalar broadcast to all wheels, or per-wheel array).
        wheel_positions: Positions of each wheel relative to the robot center, shape (N, 3).
        wheel_orientations: Quaternion orientations of each wheel, shape (N, 4) as [w, x, y, z].
        mecanum_angles: Mecanum roller angles in degrees for each wheel.
        wheel_axis: Local rotation axis of the wheel joint.
        up_axis: Up direction of the robot frame.
        max_linear_speed: Maximum linear speed in m/s.
        max_angular_speed: Maximum angular speed in rad/s.
        max_wheel_speed: Maximum individual wheel speed in rad/s.
        linear_gain: Gain applied to the linear velocity command.
        angular_gain: Gain applied to the angular velocity command.

    Raises:
        ValueError: If `wheel_radius`, `wheel_positions`, `wheel_orientations`, or `mecanum_angles` is None.
    """

    def __init__(
        self,
        *,
        wheel_radius: list | np.ndarray | None = None,
        wheel_positions: list | np.ndarray | None = None,
        wheel_orientations: list | np.ndarray | None = None,
        mecanum_angles: list | np.ndarray | None = None,
        wheel_axis: list | np.ndarray | None = None,
        up_axis: list | np.ndarray | None = None,
        max_linear_speed: float = 1.0e20,
        max_angular_speed: float = 1.0e20,
        max_wheel_speed: float = 1.0e20,
        linear_gain: float = 1.0,
        angular_gain: float = 1.0,
    ) -> None:
        if wheel_axis is None:
            wheel_axis = np.array([1.0, 0.0, 0.0])
        if up_axis is None:
            up_axis = np.array([0.0, 0.0, 1.0])
        for param_name, param_val in (
            ("wheel_radius", wheel_radius),
            ("wheel_positions", wheel_positions),
            ("wheel_orientations", wheel_orientations),
            ("mecanum_angles", mecanum_angles),
        ):
            if param_val is None:
                raise ValueError(
                    f"{param_name} is required (received None). " f"Pass an array with one entry per wheel."
                )
        wheel_positions = np.asarray(wheel_positions)
        self.num_wheels = len(wheel_positions)
        wheel_radius = np.asarray(wheel_radius)
        if wheel_radius.size == 1:
            self.wheel_radius = [float(wheel_radius)] * self.num_wheels
        else:
            self.wheel_radius = list(wheel_radius)
        self.wheel_positions = np.asarray(wheel_positions)
        self.wheel_orientations = np.asarray(wheel_orientations)
        mecanum_angles = np.asarray(mecanum_angles)
        if mecanum_angles.size == 1:
            self.mecanum_angles = [float(mecanum_angles)] * self.num_wheels
        else:
            self.mecanum_angles = list(mecanum_angles)
        self.wheel_axis = np.asarray(wheel_axis, dtype=np.float64)
        self.up_axis = np.asarray(up_axis, dtype=np.float64)
        self.max_linear_speed = max_linear_speed
        self.max_angular_speed = max_angular_speed
        self.max_wheel_speed = max_wheel_speed
        self.linear_gain = linear_gain
        self.angular_gain = angular_gain
        self.joint_commands = np.zeros(self.num_wheels, dtype=np.float64)
        self._build_base()

    def _build_base(self) -> None:
        """Build the wheel direction matrix and initialize the OSQP solver."""
        self.base_dir = np.zeros((3, self.num_wheels), dtype=float)
        self.wheel_dists_inv = np.zeros((3, self.num_wheels), dtype=float)
        for i in range(self.num_wheels):
            p_0 = self.wheel_positions[i]
            r_0 = transform_utils.quaternion_to_rotation_matrix(self.wheel_orientations[i]).numpy()
            joint_pose = np.zeros((4, 4))
            joint_pose[:3, :3] = r_0.T
            joint_pose[3, :3] = p_0
            joint_pose[3, 3] = 1
            mecanum_angle = self.mecanum_angles[i]
            mecanum_radius = self.wheel_radius[i]
            euler_vec = np.array(self.up_axis * mecanum_angle, dtype=np.float64)
            quat = transform_utils.euler_angles_to_quaternion(euler_vec, degrees=True, extrinsic=True).numpy()
            m_rot = Gf.Rotation(Gf.Quatf(*quat.tolist()))
            j_axis = Gf.Vec3f(
                m_rot.TransformDir(Gf.Matrix4f(joint_pose).TransformDir(Gf.Vec3d(*self.wheel_axis.tolist())))
            ).GetNormalized()
            self.base_dir[0, i] = j_axis[0] * mecanum_radius
            self.base_dir[1, i] = j_axis[1] * mecanum_radius
            for k in range(2):
                self.wheel_dists_inv[k, i] = p_0[k]
        wheel_radius_arr = np.asarray(self.wheel_radius, dtype=np.float64)
        norm_r = float(np.linalg.norm(wheel_radius_arr))
        if norm_r > 0:
            diag_P = (wheel_radius_arr / norm_r).copy()
        else:
            diag_P = np.ones(self.num_wheels, dtype=np.float64)
        # Ensure plain numpy array for scipy (avoids coverage/sentinel issues)
        diag_P = np.asarray(diag_P, dtype=np.float64).copy()
        P = sparse.diags(diag_P, format="csc")
        q = np.zeros(self.num_wheels)
        V = self.base_dir
        W = np.cross(V, self.wheel_dists_inv, axis=0)
        concat_vw = np.asarray(np.concatenate((V, W), axis=0), dtype=np.float64).copy()
        A = sparse.csc_matrix(concat_vw)
        l = np.array([0.0, 0.0, -np.inf, -np.inf, -np.inf, 0.0])
        u = np.array([0.0, 0.0, np.inf, np.inf, np.inf, 0.0])
        self.prob = osqp.OSQP()
        self.prob.setup(P, q=q, A=A, l=l, u=u, verbose=False)
        self.prob.solve()
        self._l = l.copy()
        self._u = u.copy()

    def forward(self, command: np.ndarray) -> np.ndarray:
        """Compute wheel velocities from [forward, lateral, yaw] command.

        Args:
            command: Shape (3,) — [forward speed, lateral speed, yaw speed].

        Returns:
            Shape (num_wheels,) — wheel joint velocities.

        Raises:
            ValueError: If command does not have length 3.
        """
        if isinstance(command, list):
            command = np.array(command, dtype=np.float64)
        if command.shape[0] != 3:
            raise ValueError("command must have length 3")
        if np.allclose(command, 0.0):
            return np.zeros(self.num_wheels, dtype=np.float64)
        v = np.array([command[0], command[1], 0.0], dtype=np.float64) * self.linear_gain
        w = np.array([command[2]], dtype=np.float64) * self.angular_gain
        if np.linalg.norm(v) > 0:
            v_norm = v / np.linalg.norm(v)
        else:
            v_norm = v
        if np.linalg.norm(v) > self.max_linear_speed:
            v = v_norm * self.max_linear_speed
        if np.linalg.norm(w) > self.max_angular_speed:
            w = w / np.abs(w) * np.array([self.max_angular_speed])
        self._l[0:2] = self._u[0:2] = v[0:2] / self.max_linear_speed
        self._l[-1] = self._u[-1] = w[0] / self.max_linear_speed
        self.prob.update(l=self._l, u=self._u)
        res = None
        try:
            res = self.prob.solve()
        except Exception as e:
            carb.log_error(f"HolonomicController error: {e}")
        if res is not None:
            values = res.x.reshape(-1) * self.max_linear_speed
            if np.max(np.abs(values)) > self.max_wheel_speed:
                scale = self.max_wheel_speed / np.max(np.abs(values))
                values = values * scale
            self.joint_commands = values.astype(np.float64)
        return self.joint_commands.copy()
