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

"""Velocity-based IK controller - Jacobian inverse methods with gain.

Computes joint velocities via ``dq = gain * J† * x_error`` (method-selectable),
then integrates
them over dt to produce **absolute joint positions**.  The gain controls how
aggressively the end effector tracks the target (higher = faster convergence,
lower = smoother but slower).

Drop-in replacement for ``PositionBasedIKController`` - matches the same
three-method interface::

    set_target(position, orientation)
    compute() -> np.ndarray | None
    reset()
"""

from __future__ import annotations

import time
from collections.abc import Sequence

import carb
import numpy as np
from isaacsim.core.experimental.prims import Articulation, RigidPrim

from ._utils import FloatingBaseArticulationError, quat_error_wxyz, slice_ee_jacobian, xyzw_to_wxyz


class VelocityBasedIKController:
    """Per-arm velocity-based IK solver using selectable Jacobian methods.

    The solve loop each frame:
        1. Read current joint positions, EE pose, and Jacobian from PhysX.
        2. Compute 6D task-space error (position + orientation).
        3. Compute joint velocities: ``dq = gain * J† * x_error``.
        4. Integrate: ``q_new = q_current + dq * dt``.
        5. Clamp per-step joint change to ``max_joint_step_rad``.

    The ``gain`` parameter controls response speed:
        - gain ~ 1.0-5.0: smooth, conservative tracking.
        - gain ~ 10.0-20.0: fast, aggressive tracking.
        - gain > 30.0: may overshoot or oscillate.

    Implements the same interface as ``PositionBasedIKController``::

        set_target(position, orientation)
        compute() -> np.ndarray | None
        reset()

    The articulation must have a fixed base: the Jacobian is indexed for the
    fixed-base layout ``(num_links - 1, 6, num_dofs)``.

    Args:
        robot: Value for robot.
        ee_link: Value for ee link.
        ee_link_index: Value for ee link index.
        num_arm_dofs: Number of controlled joints. Without explicit indices, controls the first N DOFs.
        arm_dof_indices: Optional articulation DOF indices of the controlled arm joints,
            in ascending order.
        method: Value for method.
        damping: Value for damping.
        min_singular_value: Value for min singular value.
        gain: Value for gain.
        max_joint_step_rad: Value for max joint step rad.

    Raises:
        ValueError: If ``arm_dof_indices`` is empty.
    """

    def __init__(
        self,
        robot: Articulation,
        ee_link: RigidPrim,
        ee_link_index: int,
        num_arm_dofs: int,
        method: str = "damped-least-squares",
        damping: float = 0.05,
        min_singular_value: float = 1e-5,
        gain: float = 5.0,
        max_joint_step_rad: float = 0.0,
        *,
        arm_dof_indices: Sequence[int] | None = None,
    ) -> None:
        if arm_dof_indices is None:
            arm_dof_indices = range(num_arm_dofs)
        if len(arm_dof_indices) != num_arm_dofs:
            raise ValueError("num_arm_dofs must match the number of arm_dof_indices")
        if len(arm_dof_indices) == 0:
            raise ValueError("arm_dof_indices must name at least one articulation DOF")
        self._robot = robot
        self._ee_link = ee_link
        self._ee_link_index = ee_link_index
        self._arm_dof_indices = np.asarray(arm_dof_indices, dtype=np.int64)
        self._num_arm_dofs = len(self._arm_dof_indices)
        self._method = method
        self._damping = max(1e-6, damping)
        self._min_singular_value = max(1e-8, min_singular_value)
        self._gain = max(0.01, gain)
        self._max_joint_step_rad = max(0.0, max_joint_step_rad)

        self._goal_position: np.ndarray | None = None  # (1, 3)
        self._goal_orientation: np.ndarray | None = None  # (1, 4) wxyz
        self._last_time: float = 0.0
        self._reachable: bool = True
        self._floating_base_reported: bool = False

    @property
    def reachable(self) -> bool:
        """Whether the last compute() produced a valid solution.

        Returns:
            The requested value.
        """
        return self._reachable

    @property
    def gain(self) -> float:
        """Return the velocity IK proportional gain.

        Returns:
            The requested value.
        """
        return self._gain

    @gain.setter
    def gain(self, value: float) -> None:
        """Set the velocity IK proportional gain.

        Args:
            value: Value for value.
        """
        self._gain = max(0.01, value)

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

    @property
    def method(self) -> str:
        """Return the differential IK method name.

        Returns:
            The requested value.
        """
        return self._method

    @method.setter
    def method(self, value: str) -> None:
        """Set the differential IK method by name.

        Args:
            value: Value for value.
        """
        allowed = {
            "damped-least-squares",
            "pseudoinverse",
            "transpose",
            "singular-value-decomposition",
        }
        if value not in allowed:
            raise ValueError(f"Unknown IK method: {value}")
        self._method = value

    @property
    def damping(self) -> float:
        """Return the damping factor for the DLS method.

        Returns:
            The requested value.
        """
        return float(self._damping)

    @damping.setter
    def damping(self, value: float) -> None:
        """Set the damping factor for DLS method.

        Args:
            value: Value for value.
        """
        self._damping = max(1e-6, value)

    @property
    def vr_target_filter(self) -> float:
        """Return the VR target filter strength (always 0 for velocity IK).

        Returns:
            The requested value.
        """
        return 0.0

    @vr_target_filter.setter
    def vr_target_filter(self, value: float) -> None:
        """No-op; velocity IK does not support VR target filtering.

        Args:
            value: Value for value.
        """

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
        self._goal_position = np.array([list(position)], dtype=np.float64)
        if orientation is not None:
            self._goal_orientation = xyzw_to_wxyz(orientation)
        else:
            self._goal_orientation = None

    def compute(self) -> np.ndarray | None:
        """Compute one IK step via pseudoinverse Jacobian with velocity integration.

        Returns:
            The requested value.
        """
        if self._goal_position is None:
            return None

        now = time.monotonic()
        dt = now - self._last_time if self._last_time > 0 else 1.0 / 60.0
        dt = min(dt, 0.1)
        self._last_time = now

        try:
            current_dofs = self._robot.get_dof_positions().numpy()
            ee_pos, ee_orient = (arr.numpy() for arr in self._ee_link.get_world_poses())
            jacobians = self._robot.get_jacobian_matrices().numpy()
            jacobian_ee = slice_ee_jacobian(jacobians, self._robot.num_dofs, self._ee_link_index, self._arm_dof_indices)
        except FloatingBaseArticulationError as exc:
            self._report_floating_base(exc)
            return None
        except (AssertionError, RuntimeError):
            return None

        # 6D task-space error: position (3) + orientation (3)
        pos_error = self._goal_position - ee_pos
        if self._goal_orientation is not None:
            orient_error = quat_error_wxyz(self._goal_orientation, ee_orient)
        else:
            orient_error = np.zeros((1, 3), dtype=np.float64)

        x_error = np.concatenate([pos_error, orient_error], axis=-1)
        x_error = np.expand_dims(x_error, axis=2)

        dq_vel = self._compute_joint_velocity(jacobian_ee, x_error)

        delta = dq_vel * dt
        if self._max_joint_step_rad > 0.0:
            delta = np.clip(delta, -self._max_joint_step_rad, self._max_joint_step_rad)
        new_positions = current_dofs[:, self._arm_dof_indices] + delta

        pos_err_norm = float(np.linalg.norm(pos_error))
        self._reachable = pos_err_norm < 0.5

        return new_positions.squeeze(0)

    def reset(self) -> None:
        """Clear the target pose and timing state."""
        self._goal_position = None
        self._goal_orientation = None
        self._last_time = 0.0
        self._reachable = True

    def _report_floating_base(self, error: FloatingBaseArticulationError) -> None:
        """Report a refused floating-base articulation once per solver.

        Args:
            error: The refusal raised while slicing the Jacobian.
        """
        self._reachable = False
        if self._floating_base_reported:
            return
        self._floating_base_reported = True
        carb.log_warn(f"[Teleop][IK] {error}")

    def _compute_joint_velocity(self, jacobian_ee: np.ndarray, x_error: np.ndarray) -> np.ndarray:
        """Compute velocity-space joint update with selectable Jacobian method.

        Args:
            jacobian_ee: Value for jacobian ee.
            x_error: Value for x error.

        Returns:
            The requested value.
        """
        batch_size = jacobian_ee.shape[0]
        dofs = jacobian_ee.shape[2]
        dq = np.zeros((batch_size, dofs), dtype=np.float64)

        for i in range(batch_size):
            J = jacobian_ee[i]
            e = x_error[i, :, 0]
            if self._method == "singular-value-decomposition":
                U, S, Vh = np.linalg.svd(J)
                # Filtered singular values are dropped rather than inverted. A
                # degenerate configuration puts exact zeros in `S`, and inverting
                # the whole array before selecting would divide by them.
                inv_s = np.divide(1.0, S, out=np.zeros_like(S), where=np.greater(S, self._min_singular_value))
                J_pinv = Vh.T @ np.diag(inv_s) @ U.T
                dq[i] = self._gain * (J_pinv @ e)
            elif self._method == "pseudoinverse":
                J_pinv = np.linalg.pinv(J)
                dq[i] = self._gain * (J_pinv @ e)
            elif self._method == "transpose":
                dq[i] = self._gain * (J.T @ e)
            elif self._method == "damped-least-squares":
                JT = J.T
                lmbda = np.eye(J.shape[0], dtype=np.float64) * (self._damping**2)
                try:
                    dq[i] = self._gain * (JT @ np.linalg.solve(J @ JT + lmbda, e))
                except np.linalg.LinAlgError:
                    dq[i] = self._gain * (JT @ e)
            else:
                raise ValueError(f"Unknown IK method: {self._method}")

        return dq
