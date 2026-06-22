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

"""Levenberg-Marquardt IK solver implementation."""

from __future__ import annotations

from typing import Any

import numpy as np

from .ik_solver import IKSolver, IKSolverRegistry, pose_error  # noqa: F401 -- re-exports pose_error
from .kinematic_chain import KinematicChain
from .math import Transform, VecN

# ---------- IK ----------


def ik_lm(
    chain: KinematicChain,
    q0: VecN,
    target: Transform,
    lam: float = 1e-3,
    iters: int = 30,
    tol: float = 1e-6,
    w_rot: float = 1.0,
    w_pos: float = 1.0,
    max_step: float = 0.5,
    base_frame: Transform | None = None,
    null_space_bias: float = 0.05,
    joint_fixed: list[bool] | np.ndarray | None = None,
) -> VecN:
    """Solve IK using Levenberg-Marquardt with optional null-space bias.

    When a base_frame is provided the caller's target is in robot-base frame
    but the FK chain is relative to base_frame (the start-site zero-config
    pose). Express the target in chain-local coordinates so the solver
    compares like with like.

    Args:
        chain: Kinematic chain providing joints and FK computation.
        q0: Initial joint configuration.
        target: Desired end-effector pose in chain-local coordinates.
        lam: Levenberg-Marquardt damping factor.
        iters: Maximum iterations.
        tol: Convergence tolerance on weighted cost.
        w_rot: Rotation weight in cost (x3 for rot components).
        w_pos: Position weight in cost (x3 for pos components).
        max_step: Maximum joint step per iteration.
        base_frame: If set, target is expressed in this frame.
        null_space_bias: Bias toward joint mid-range in null space.
        joint_fixed: Mask of fixed (locked) joints.

    Returns:
        Joint values that (approximately) achieve the target.

    """
    if base_frame is not None:
        target = base_frame.inv() @ target

    joints = chain.joints
    n = len(joints)
    q = q0.astype(float).copy()
    lo = np.array([j.lower for j in joints])
    hi = np.array([j.upper for j in joints])
    q = np.clip(q, lo, hi)

    # Locked DOFs: set Jacobian columns to zero so those joints don't move.
    fixed_mask = np.asarray(joint_fixed, dtype=bool) if joint_fixed is not None else np.zeros(n, dtype=bool)
    if fixed_mask.shape != (n,):
        fixed_mask = np.zeros(n, dtype=bool)

    # Pre-allocate constants used every iteration
    I_n = np.eye(n)
    W = np.array([w_rot, w_rot, w_rot, w_pos, w_pos, w_pos], dtype=float)
    tol_sq = tol * tol

    # Null-space joint centering: for joints with finite limits, bias the
    # regularisation toward mid-range instead of toward zero-step.  When a
    # Jacobian column vanishes (gimbal lock / singularity) the damping term
    # dominates and pushes the joint toward centre, preventing it from
    # getting stuck.  For unbounded joints the bias is zero (no centre).
    finite_mask = np.isfinite(lo) & np.isfinite(hi)
    # Compute midpoint only on finite entries; for unbounded joints lo+hi would
    # be -inf + +inf = NaN and emit a RuntimeWarning even though np.where would
    # discard the value.
    q_center = np.zeros_like(lo)
    q_center[finite_mask] = 0.5 * (lo[finite_mask] + hi[finite_mask])
    use_null_bias = null_space_bias > 0.0 and np.any(finite_mask)

    # Initial evaluation (fused FK + Jacobian in one pass)
    T, J = chain.compute_fk_and_jacobian(q)
    J[:, fixed_mask] = 0.0
    e = pose_error(target, T)
    ew = W * e
    cost = ew @ ew

    for _ in range(iters):
        if cost < tol_sq:
            break

        # Weighted Jacobian: broadcast W column-wise (avoids 6x6 diag matmul)
        Jw = W[:, None] * J
        JtJw = Jw.T @ Jw
        Jtew = Jw.T @ ew

        # Null-space bias: regularise toward (q_center - q) instead of zero.
        # In directions where J is rank-deficient the damping term dominates,
        # yielding dq ≈ null_space_bias * (q_center - q) which moves the
        # joint toward mid-range and out of singular configurations.
        if use_null_bias:
            rhs = Jtew + lam * null_space_bias * (q_center - q)
        else:
            rhs = Jtew

        dq = np.linalg.solve(JtJw + lam * I_n, rhs)
        dq[fixed_mask] = 0.0

        # Clamp step magnitude to prevent wild jumps far from the solution.
        # Squared comparison avoids sqrt; actual norm computed only when clamping.
        step_sq = dq @ dq
        if step_sq > max_step * max_step:
            dq *= max_step / np.sqrt(step_sq)

        q_new = np.clip(q + dq, lo, hi)

        # Trial evaluation (fused FK + Jacobian so we don't recompute on accept)
        T_new, J_new = chain.compute_fk_and_jacobian(q_new)
        J_new[:, fixed_mask] = 0.0
        e_new = pose_error(target, T_new)
        ew_new = W * e_new
        cost_new = ew_new @ ew_new

        # Adaptive damping: shrink toward Gauss-Newton on progress, grow on overshoot
        if cost_new < cost:
            q, J, e, ew, cost = q_new, J_new, e_new, ew_new, cost_new
            lam = max(lam * 0.5, 1e-6)
        else:
            lam = min(lam * 2.0, 1e2)

    return q


# ---------------------------------------------------------------------------
# IKSolver interface implementation
# ---------------------------------------------------------------------------


class IKSolverLM(IKSolver):
    """Levenberg-Marquardt IK solver implementing the :class:`IKSolver` interface.

    Solver-specific keyword arguments accepted by :meth:`solve`:

    * ``lam`` (float): LM damping factor (default 1e-3).
    * ``iters`` (int): maximum iterations (default 30).
    * ``tol`` (float): internal convergence tolerance (default 1e-6).
    * ``w_rot`` (float): rotation error weight (default 1.0).
    * ``w_pos`` (float): position error weight (default 1.0).
    * ``max_step`` (float): maximum step magnitude (default 0.5).
    * ``null_space_bias`` (float): null-space joint centering (default 0.05).
    * ``joint_fixed`` (array-like of bool, optional): mask of chain length; True
      locks that DOF (Jacobian column set to zero and step zeroed).
    """

    def solve(
        self,
        chain: KinematicChain,
        target: Transform,
        q0: VecN | None = None,
        **kwargs: Any,
    ) -> VecN:
        """Solve IK for the given kinematic chain. See class docstring for kwargs.

        Args:
            chain: Kinematic chain providing joints and FK computation.
            target: Desired end-effector pose in chain-local coordinates.
            q0: Initial joint configuration, or None for zero.
            **kwargs: Solver options (lam, iters, tol, w_rot, w_pos, etc.).

        Returns:
            Joint values that achieve the target.

        """
        if q0 is None:
            q0 = np.zeros(len(chain.joints))
        # Target is already in chain-local frame; do not pass base_frame.
        return ik_lm(chain, q0, target, base_frame=None, **kwargs)


# Register as the default IK solver.
IKSolverRegistry.register("lm", IKSolverLM, default=True)
