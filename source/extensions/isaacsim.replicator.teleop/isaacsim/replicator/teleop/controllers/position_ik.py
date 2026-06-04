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

"""Position-based IK controller - differential IK with multiple solver methods.

Computes joint position deltas from the full task-space error each frame
using one of several numerical methods (DLS, pseudoinverse, transpose, SVD).

Drop-in compatible with ``VelocityBasedIKController`` - matches the same
three-method interface::

    set_target(position, orientation)
    compute() -> np.ndarray | None
    reset()
"""

from __future__ import annotations

import numpy as np
from isaacsim.core.experimental.prims import Articulation, RigidPrim

from ._utils import ema_blend, quat_conjugate_wxyz, quat_mul_wxyz, xyzw_to_wxyz

# ---------------------------------------------------------------------------
# Differential IK math
# ---------------------------------------------------------------------------


def _differential_ik(
    jacobian_ee: np.ndarray,
    current_pos: np.ndarray,
    current_orient: np.ndarray,
    goal_pos: np.ndarray,
    goal_orient: np.ndarray | None = None,
    method: str = "damped-least-squares",
    scale: float = 1.0,
    damping: float = 0.05,
    min_singular_value: float = 1e-5,
) -> np.ndarray:
    """Compute delta-DOF positions via differential IK.

    Args:
        jacobian_ee: End-effector Jacobian (N, 6, num_arm_dofs).
        current_pos: Current EE position (N, 3).
        current_orient: Current EE orientation wxyz (N, 4).
        goal_pos: Goal EE position (N, 3).
        goal_orient: Goal EE orientation wxyz (N, 4), or None (position only).
        method: IK method name.
        scale: Step scale factor.
        damping: Damping factor for DLS.
        min_singular_value: Threshold for SVD method.

    Returns:
        Delta DOF positions (N, num_arm_dofs).
    """
    goal_orient = current_orient if goal_orient is None else goal_orient
    q = quat_mul_wxyz(goal_orient, quat_conjugate_wxyz(current_orient))
    error = np.expand_dims(np.concatenate([goal_pos - current_pos, q[:, 1:] * np.sign(q[:, [0]])], axis=-1), axis=2)

    if method == "singular-value-decomposition":
        U, S, Vh = np.linalg.svd(jacobian_ee)
        inv_s = np.where(min_singular_value < S, 1.0 / S, np.zeros_like(S))
        inv_s_diag = np.zeros((*inv_s.shape, inv_s.shape[-1]), dtype=inv_s.dtype)
        np.einsum("...ii->...i", inv_s_diag)[...] = inv_s
        K = inv_s.shape[-1]
        pseudoinverse = np.swapaxes(Vh, 1, 2)[:, :, :K] @ inv_s_diag @ np.swapaxes(U, 1, 2)[:, :K, :]
        return (scale * pseudoinverse @ error).squeeze(-1)
    elif method == "pseudoinverse":
        pseudoinverse = np.linalg.pinv(jacobian_ee)
        return (scale * pseudoinverse @ error).squeeze(-1)
    elif method == "transpose":
        transpose = np.swapaxes(jacobian_ee, 1, 2)
        return (scale * transpose @ error).squeeze(-1)
    elif method == "damped-least-squares":
        transpose = np.swapaxes(jacobian_ee, 1, 2)
        lmbda = np.eye(jacobian_ee.shape[1]) * (damping**2)
        try:
            return (scale * transpose @ np.linalg.solve(jacobian_ee @ transpose + lmbda, error)).squeeze(-1)
        except np.linalg.LinAlgError:
            return (scale * transpose @ error).squeeze(-1)
    else:
        raise ValueError(f"Unknown IK method: {method}")


# ---------------------------------------------------------------------------
# Per-arm IK solver
# ---------------------------------------------------------------------------


class PositionBasedIKController:
    """Per-arm position-based IK solver (Jacobian with multiple methods).

    Computes joint position deltas from the full task-space error each frame.
    Minimal interface - any alternative IK backend can be swapped in by
    matching these three methods::

        set_target(position, orientation)
        compute() -> np.ndarray | None
        reset()

    Built-in safeguards for VR teleop:
    - **VR target filtering**: EMA low-pass filter on the raw VR target
      (``vr_target_filter``, 0.0 = none, ~0.9 = heavy filtering).
    - **Joint-step clamp**: caps max joint change per step
      (``max_joint_step_rad``, radians).
    - **Manipulability check**: if the Jacobian's minimum singular value
      drops below ``min_manipulability``, the arm is near a singularity or
      workspace boundary - the solver freezes instead of shaking.
    - **Error-proportional scaling**: when EE-to-target distance exceeds
      ``error_scale_distance``, the step is scaled down proportionally so
      the arm slows to a stop at its reachable limit.
    """

    def __init__(
        self,
        robot: Articulation,
        ee_link: RigidPrim,
        ee_link_index: int,
        num_arm_dofs: int,
        method: str = "damped-least-squares",
        scale: float = 1.0,
        damping: float = 0.05,
        vr_target_filter: float = 0.0,
        max_joint_step_rad: float = 0.0,
        min_manipulability: float = 0.001,
        error_scale_distance: float = 0.5,
    ) -> None:
        self._robot = robot
        self._ee_link = ee_link
        self._ee_link_index = ee_link_index
        self._num_arm_dofs = num_arm_dofs
        self._method = method
        self._scale = scale
        self._damping = damping
        self._vr_target_filter = np.clip(vr_target_filter, 0.0, 0.99)
        self._max_joint_step_rad = max(0.0, max_joint_step_rad)
        self._min_manipulability = max(0.0, min_manipulability)
        self._error_scale_distance = max(0.01, error_scale_distance)

        self._raw_position: np.ndarray | None = None
        self._raw_orientation: np.ndarray | None = None  # wxyz (1,4)
        self._filtered_position: np.ndarray | None = None
        self._filtered_orientation: np.ndarray | None = None
        self._reachable: bool = True

    @property
    def reachable(self) -> bool:
        """Whether the last compute() produced a valid solution."""
        return self._reachable

    @property
    def vr_target_filter(self) -> float:
        """Return the EMA low-pass filter strength for VR targets."""
        return float(self._vr_target_filter)

    @vr_target_filter.setter
    def vr_target_filter(self, value: float) -> None:
        """Set the EMA low-pass filter strength for VR targets."""
        self._vr_target_filter = np.clip(value, 0.0, 0.99)

    @property
    def max_joint_step_rad(self) -> float:
        """Return the maximum allowed joint change per step in radians."""
        return float(self._max_joint_step_rad)

    @max_joint_step_rad.setter
    def max_joint_step_rad(self, value: float) -> None:
        """Set the maximum allowed joint change per step in radians."""
        self._max_joint_step_rad = max(0.0, value)

    @property
    def method(self) -> str:
        """Return the differential IK method name."""
        return self._method

    @method.setter
    def method(self, value: str) -> None:
        """Set the differential IK method by name."""
        allowed = {
            "damped-least-squares",
            "pseudoinverse",
            "transpose",
            "singular-value-decomposition",
        }
        if value not in allowed:
            raise ValueError(f"Unknown IK method: {value}")
        self._method = value

    def set_target(
        self,
        position: tuple[float, float, float],
        orientation: tuple[float, float, float, float] | None,
    ) -> None:
        """Set the 6DOF goal pose (sim coordinates, xyzw quaternion)."""
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
        """Compute one differential IK step.

        Returns:
            Absolute joint positions for the first ``num_arm_dofs`` DOFs
            as a 1-D numpy array, or None if no target / physics not ready.
        """
        if self._filtered_position is None:
            return None

        try:
            current_dofs = self._robot.get_dof_positions().numpy()
            ee_pos_np, ee_orient_np = (arr.numpy() for arr in self._ee_link.get_world_poses())
            jacobians = self._robot.get_jacobian_matrices().numpy()
            jacobian_ee = jacobians[:, self._ee_link_index - 1, :, : self._num_arm_dofs]
        except (AssertionError, RuntimeError):
            return None

        # Soft manipulability scaling - reduce step near singularities
        singular_values = np.linalg.svd(jacobian_ee[0], compute_uv=False)
        min_sv = float(singular_values[-1])
        if min_sv < self._min_manipulability:
            manip_scale = min_sv / self._min_manipulability
            self._reachable = False
        else:
            manip_scale = 1.0
            self._reachable = True

        # Error-proportional scaling - reduce step when target is far away
        pos_error = float(np.linalg.norm(self._filtered_position - ee_pos_np))
        if pos_error > self._error_scale_distance:
            error_scale = self._error_scale_distance / pos_error
        else:
            error_scale = 1.0

        combined_scale = self._scale * error_scale * manip_scale

        delta = _differential_ik(
            jacobian_ee=jacobian_ee,
            current_pos=ee_pos_np,
            current_orient=ee_orient_np,
            goal_pos=self._filtered_position,
            goal_orient=self._filtered_orientation,
            method=self._method,
            scale=combined_scale,
            damping=self._damping,
        )

        if self._max_joint_step_rad > 0.0:
            delta = np.clip(delta, -self._max_joint_step_rad, self._max_joint_step_rad)
        new_positions = current_dofs[:, : self._num_arm_dofs] + delta

        return new_positions.squeeze(0)

    def reset(self) -> None:
        """Clear the target pose, filter state, and reachability flag."""
        self._raw_position = None
        self._raw_orientation = None
        self._filtered_position = None
        self._filtered_orientation = None
        self._reachable = True
