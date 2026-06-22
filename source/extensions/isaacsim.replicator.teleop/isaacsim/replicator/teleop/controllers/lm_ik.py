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

"""Levenberg-Marquardt IK controller - multi-iteration damped least-squares.

Runs multiple LM iterations per ``compute()`` call on a single Jacobian read,
converging closer to the target within one frame than a single-step solver.
Uses the Articulation API for Jacobians and joint state - same data pipeline
as ``PositionBasedIKController`` and ``VelocityBasedIKController``.

Drop-in compatible - matches the same three-method interface::

    set_target(position, orientation)
    compute() -> np.ndarray | None
    reset()
"""

from __future__ import annotations

import numpy as np
from isaacsim.core.experimental.prims import Articulation, RigidPrim

from ._utils import ema_blend, quat_conjugate_wxyz, quat_mul_wxyz, xyzw_to_wxyz


class LMIKController:
    """Per-arm Levenberg-Marquardt IK solver.

    Each ``compute()`` call reads the Jacobian and EE pose once from
    PhysX, then runs up to ``max_iters`` damped least-squares steps on
    the joint-space residual.  The current joint state serves as warm
    start, giving faster per-frame convergence than single-step methods.

    Built-in safeguards for VR teleop:
    - **VR target filtering**: EMA low-pass filter on the raw VR target.
    - **Joint-step clamp**: caps total joint change per frame.
    - **Joint limits**: hard clamp every iteration via cached DOF limits.
    - **Convergence check**: early exit when pose error drops below threshold.

    Args:
        robot: Value for robot.
        ee_link: Value for ee link.
        ee_link_index: Value for ee link index.
        num_arm_dofs: Value for num arm dofs.
        damping: Value for damping.
        max_iters: Value for max iters.
        convergence_threshold: Value for convergence threshold.
        vr_target_filter: Value for vr target filter.
        max_joint_step_rad: Value for max joint step rad.
    """

    def __init__(
        self,
        robot: Articulation,
        ee_link: RigidPrim,
        ee_link_index: int,
        num_arm_dofs: int,
        damping: float = 1e-2,
        max_iters: int = 20,
        convergence_threshold: float = 1e-4,
        vr_target_filter: float = 0.0,
        max_joint_step_rad: float = 0.0,
    ) -> None:
        self._robot = robot
        self._ee_link = ee_link
        self._ee_link_index = ee_link_index
        self._num_arm_dofs = num_arm_dofs
        self._damping = max(1e-6, damping)
        self._max_iters = max(1, max_iters)
        self._convergence_threshold = max(1e-8, convergence_threshold)
        self._vr_target_filter = np.clip(vr_target_filter, 0.0, 0.99)
        self._max_joint_step_rad = max(0.0, max_joint_step_rad)

        # Cache joint limits (lower, upper) as 1-D arrays for the arm DOFs
        try:
            limits = robot.get_dof_limits().numpy()  # (N, num_dofs, 2)
            self._lo = limits[0, :num_arm_dofs, 0].astype(np.float64)
            self._hi = limits[0, :num_arm_dofs, 1].astype(np.float64)
        except Exception:
            self._lo = np.full(num_arm_dofs, -2 * np.pi)
            self._hi = np.full(num_arm_dofs, 2 * np.pi)

        self._raw_position: np.ndarray | None = None
        self._raw_orientation: np.ndarray | None = None  # wxyz (1, 4)
        self._filtered_position: np.ndarray | None = None
        self._filtered_orientation: np.ndarray | None = None
        self._reachable: bool = True

    # ------------------------------------------------------------------
    # Properties
    # ------------------------------------------------------------------

    @property
    def reachable(self) -> bool:
        """Whether the last compute() produced a valid solution.

        Returns:
            The requested value.
        """
        return self._reachable

    @property
    def vr_target_filter(self) -> float:
        """Return the EMA low-pass filter strength for VR targets.

        Returns:
            The requested value.
        """
        return float(self._vr_target_filter)

    @vr_target_filter.setter
    def vr_target_filter(self, value: float) -> None:
        """Set the EMA low-pass filter strength for VR targets.

        Args:
            value: Value for value.
        """
        self._vr_target_filter = np.clip(value, 0.0, 0.99)

    @property
    def max_joint_step_rad(self) -> float:
        """Return the maximum allowed joint change per step in radians.

        Returns:
            The requested value.
        """
        return float(self._max_joint_step_rad)

    @max_joint_step_rad.setter
    def max_joint_step_rad(self, value: float) -> None:
        """Set the maximum allowed joint change per step in radians.

        Args:
            value: Value for value.
        """
        self._max_joint_step_rad = max(0.0, value)

    # ------------------------------------------------------------------
    # Interface
    # ------------------------------------------------------------------

    def set_target(
        self,
        position: tuple[float, float, float],
        orientation: tuple[float, float, float, float] | None,
    ) -> None:
        """Set the 6DOF goal pose (sim coordinates, xyzw quaternion).

        Args:
            position: Value for position.
            orientation: Value for orientation.
        """
        self._raw_position = np.array([list(position)], dtype=np.float64)
        if orientation is not None:
            self._raw_orientation = xyzw_to_wxyz(orientation)
        else:
            self._raw_orientation = None

        self._filtered_position = ema_blend(
            self._filtered_position,
            self._raw_position,
            self._vr_target_filter,
        )
        self._filtered_orientation = (
            ema_blend(
                self._filtered_orientation,
                self._raw_orientation,
                self._vr_target_filter,
                normalize=True,
            )
            if self._raw_orientation is not None
            else None
        )

    def compute(self) -> np.ndarray | None:
        """Run multi-iteration LM solve and return absolute joint positions.

        Returns:
            The requested value.
        """
        if self._filtered_position is None:
            return None

        try:
            current_dofs = self._robot.get_dof_positions().numpy()
            ee_pos, ee_orient = (arr.numpy() for arr in self._ee_link.get_world_poses())
            jacobians = self._robot.get_jacobian_matrices().numpy()
            jacobian_ee = jacobians[:, self._ee_link_index - 1, :, : self._num_arm_dofs]
        except (AssertionError, RuntimeError):
            return None

        q = current_dofs[0, : self._num_arm_dofs].astype(np.float64).copy()
        J = jacobian_ee[0]  # (6, num_arm_dofs)
        lam = self._damping
        eye_n = np.eye(self._num_arm_dofs)

        goal_pos = self._filtered_position
        goal_orient = self._filtered_orientation if self._filtered_orientation is not None else ee_orient

        last_error_norm = float("inf")
        for i in range(self._max_iters):
            # 6D pose error: [orientation_error (3), position_error (3)]
            # Position component
            pos_err = goal_pos - ee_pos  # (1, 3)

            # Orientation component via relative quaternion
            q_rel = quat_mul_wxyz(goal_orient, quat_conjugate_wxyz(ee_orient))
            orient_err = q_rel[:, 1:] * np.sign(q_rel[:, [0]])  # (1, 3)

            e = np.concatenate([pos_err, orient_err], axis=-1).squeeze(0)  # (6,)
            error_norm = np.linalg.norm(e)
            last_error_norm = error_norm

            if error_norm < self._convergence_threshold:
                break

            # LM step: (J^T J + lambda * I) dq = J^T e
            JtJ = J.T @ J
            Jte = J.T @ e
            dq = np.linalg.solve(JtJ + lam * eye_n, Jte)

            q = np.clip(q + dq, self._lo, self._hi)

            # Update EE pose estimate for next iteration using linearization:
            # approximate new EE state by applying the Jacobian delta
            ee_delta = J @ dq
            ee_pos = ee_pos + ee_delta[:3].reshape(1, 3)
            # For orientation: apply small rotation to current orientation estimate
            dw = ee_delta[3:6]  # angular velocity-like
            half_dw = 0.5 * dw
            dq_orient = np.array([[1.0, half_dw[0], half_dw[1], half_dw[2]]])  # wxyz
            norm_dq = np.linalg.norm(dq_orient)
            if norm_dq > 1e-8:
                dq_orient /= norm_dq
            ee_orient = quat_mul_wxyz(dq_orient, ee_orient)
            norm_o = np.linalg.norm(ee_orient)
            if norm_o > 1e-8:
                ee_orient /= norm_o

        # Clamp total delta from initial joint state
        initial_q = current_dofs[0, : self._num_arm_dofs]
        total_delta = q - initial_q
        if self._max_joint_step_rad > 0.0:
            total_delta = np.clip(total_delta, -self._max_joint_step_rad, self._max_joint_step_rad)
        q = initial_q + total_delta

        self._reachable = last_error_norm < 0.5

        return q

    def reset(self) -> None:
        """Clear the target pose, filter state, and reachability flag."""
        self._raw_position = None
        self._raw_orientation = None
        self._filtered_position = None
        self._filtered_orientation = None
        self._reachable = True
