# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

"""Nominal-model controller feedforward for Newton articulations.

Numpy forward kinematics plus a world-frame Newton-Euler sweep over a Newton
``Model`` (world 0), shared by the differentiable Featherstone bridge and the
sim-sim validation harness. The computation is deliberately independent of the
identified parameters: it mirrors the fixed internal model of a real
feedforward controller (gravity compensation or computed torque).
"""

from __future__ import annotations

import hashlib
from dataclasses import dataclass
from typing import TYPE_CHECKING, Any

import numpy as np

if TYPE_CHECKING:
    from .trajectory_csv import TrajectoryDataset

__all__ = [
    "FF_CACHE_MAX_ENTRIES",
    "FeedforwardCacheEntry",
    "compute_feedforward",
    "feedforward_content_hash",
    "filtered_acceleration",
]

# Feedforward cache bound: multiple-shooting runs alternate over a handful of
# chunk trajectories; entries are small (steps x num_dof float arrays), so the
# cap only guards against unbounded growth over very long sessions.
FF_CACHE_MAX_ENTRIES = 16


@dataclass
class FeedforwardCacheEntry:
    """Computed controller feedforward for one trajectory.

    Keyed by trajectory object identity and verified by a content hash: the
    stored trajectory reference keeps ``id()`` valid for the entry's lifetime,
    and the hash catches both in-place mutation and id reuse after garbage
    collection. Configured feedforward is fail-closed, so a cache entry always
    contains finite feedforward and velocity-target arrays.
    """

    key: int
    trajectory: Any
    content_hash: int
    steps: int
    feedforward: np.ndarray  # (steps, num_dof)
    velocity_targets: np.ndarray  # (steps, num_dof) float32


def feedforward_content_hash(trajectory: TrajectoryDataset) -> int:
    """Cheap fingerprint of every trajectory channel the feedforward reads.

    Args:
        trajectory: Telemetry trajectory used by the operation.

    Returns:
        Result produced by the operation.
    """
    digest = hashlib.blake2b(digest_size=16)
    for name in ("times", "positions", "velocities", "commands"):
        value = getattr(trajectory, name, None)
        if value is None:
            digest.update(name.encode("utf-8"))
            digest.update(b"\0")
            continue
        arr = np.ascontiguousarray(value)
        digest.update(name.encode("utf-8"))
        digest.update(str(arr.dtype).encode("ascii"))
        digest.update(np.asarray(arr.shape, dtype=np.int64).tobytes())
        digest.update(arr.tobytes())
    return int.from_bytes(digest.digest(), byteorder="big", signed=False)


def filtered_acceleration(velocities: np.ndarray, times: np.ndarray | None) -> np.ndarray:
    """Central-difference acceleration with a zero-phase moving-average smooth.

    Telemetry velocities are differentiated once, so the result is noisy at
    typical logging rates (30 Hz); the excitation is band-limited well below
    Nyquist, so a short symmetric moving average removes differentiation noise
    without phase distortion.

    Args:
        velocities: Value supplied for ``velocities``.
        times: Value supplied for ``times``.

    Returns:
        Result produced by the operation.
    """
    velocities = np.asarray(velocities, dtype=np.float64)
    steps = velocities.shape[0]
    if steps < 3:
        return np.zeros_like(velocities)
    if times is not None and times.shape[0] >= steps:
        accel = np.gradient(velocities, times[:steps], axis=0)
    else:
        accel = np.gradient(velocities, axis=0)
    window = min(5, steps if steps % 2 == 1 else steps - 1)
    if window >= 3:
        kernel = np.ones(window, dtype=np.float64) / window
        pad = window // 2
        padded = np.pad(accel, ((pad, pad), (0, 0)), mode="edge")
        accel = np.stack([np.convolve(padded[:, j], kernel, mode="valid") for j in range(accel.shape[1])], axis=1)
    return accel


def _quat_mul(q1: np.ndarray, q2: np.ndarray) -> np.ndarray:
    """Hamilton product of xyzw quaternion arrays (broadcasts over leading dims).

    Args:
        q1: Value supplied for ``q1``.
        q2: Value supplied for ``q2``.

    Returns:
        Result produced by the operation.
    """
    x1, y1, z1, w1 = q1[..., 0], q1[..., 1], q1[..., 2], q1[..., 3]
    x2, y2, z2, w2 = q2[..., 0], q2[..., 1], q2[..., 2], q2[..., 3]
    return np.stack(
        (
            w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
            w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
            w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
            w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
        ),
        axis=-1,
    )


def _quat_rotate(q: np.ndarray, v: np.ndarray) -> np.ndarray:
    """Rotate vectors by xyzw quaternions (broadcasts over leading dims).

    Args:
        q: Value supplied for ``q``.
        v: Value supplied for ``v``.

    Returns:
        Result produced by the operation.
    """
    qv = q[..., :3]
    qw = q[..., 3:4]
    t = 2.0 * np.cross(qv, v)
    return v + qw * t + np.cross(qv, t)


def _quat_conj(q: np.ndarray) -> np.ndarray:
    out = np.array(q, copy=True)
    out[..., :3] *= -1.0
    return out


def _transform_mul(pa: np.ndarray, qa: np.ndarray, pb: np.ndarray, qb: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    """Compose rigid transforms (pa, qa) ∘ (pb, qb) with xyzw quaternions.

    Args:
        pa: Value supplied for ``pa``.
        qa: Value supplied for ``qa``.
        pb: Value supplied for ``pb``.
        qb: Value supplied for ``qb``.

    Returns:
        Result produced by the operation.
    """
    return pa + _quat_rotate(qa, pb), _quat_mul(qa, qb)


def _quat_from_axis_angle(axis: np.ndarray, angle: np.ndarray) -> np.ndarray:
    half = 0.5 * np.asarray(angle, dtype=np.float64)
    sin_half = np.sin(half)
    axis = np.asarray(axis, dtype=np.float64).reshape(3)
    return np.stack(
        (
            axis[0] * sin_half,
            axis[1] * sin_half,
            axis[2] * sin_half,
            np.cos(half),
        ),
        axis=-1,
    )


def _quat_to_matrix(q: np.ndarray) -> np.ndarray:
    """Rotation matrices (T, 3, 3) from xyzw quaternions (T, 4).

    Args:
        q: Value supplied for ``q``.

    Returns:
        Result produced by the operation.
    """
    x, y, z, w = q[..., 0], q[..., 1], q[..., 2], q[..., 3]
    out = np.empty(q.shape[:-1] + (3, 3), dtype=np.float64)
    out[..., 0, 0] = 1.0 - 2.0 * (y * y + z * z)
    out[..., 0, 1] = 2.0 * (x * y - z * w)
    out[..., 0, 2] = 2.0 * (x * z + y * w)
    out[..., 1, 0] = 2.0 * (x * y + z * w)
    out[..., 1, 1] = 1.0 - 2.0 * (x * x + z * z)
    out[..., 1, 2] = 2.0 * (y * z - x * w)
    out[..., 2, 0] = 2.0 * (x * z - y * w)
    out[..., 2, 1] = 2.0 * (y * z + x * w)
    out[..., 2, 2] = 1.0 - 2.0 * (x * x + y * y)
    return out


def compute_feedforward(
    newton_module,  # noqa: ANN001
    model,  # noqa: ANN001
    *,
    world_count: int,
    coords_per_world: int,
    dofs_per_world: int,
    coord_map_world0: np.ndarray,
    dof_map_world0: np.ndarray,
    default_joint_q_world0: np.ndarray,
    baseline_mass: np.ndarray,
    baseline_com: np.ndarray,
    baseline_inertia: np.ndarray,
    q_steps: np.ndarray,
    qd_steps: np.ndarray | None = None,
    qdd_steps: np.ndarray | None = None,
) -> np.ndarray:
    """Nominal-model inverse-dynamics torque per step for the mapped DOFs (world 0).

    Runs a numpy forward-kinematics pass over the world-0 articulation at each
    telemetry configuration, then a world-frame Newton-Euler sweep (gravity folded
    in via a D'Alembert base acceleration of ``-g``), and projects the interbody
    wrenches onto the joint axes. With ``qd_steps``/``qdd_steps`` omitted or zero
    this reduces exactly to static gravity compensation. Masses, CoMs, and
    inertias come from the supplied baselines — the NOMINAL model captured at context
    build, before any theta writes — mirroring the fixed internal model a real
    feedforward controller uses. Held joints stay at their default configuration
    with zero rates, so the hand/gripper load is included in the arm joints'
    subtree wrenches.

    Args:
        newton_module: Value supplied for ``newton_module``.
        model: Value supplied for ``model``.
        world_count: Value supplied for ``world_count``.
        coords_per_world: Value supplied for ``coords_per_world``.
        dofs_per_world: Value supplied for ``dofs_per_world``.
        coord_map_world0: Value supplied for ``coord_map_world0``.
        dof_map_world0: Value supplied for ``dof_map_world0``.
        default_joint_q_world0: Value supplied for ``default_joint_q_world0``.
        baseline_mass: Value supplied for ``baseline_mass``.
        baseline_com: Value supplied for ``baseline_com``.
        baseline_inertia: Value supplied for ``baseline_inertia``.
        q_steps: Value supplied for ``q_steps``.
        qd_steps: Value supplied for ``qd_steps``.
        qdd_steps: Value supplied for ``qdd_steps``.

    Returns:
        Result produced by the operation.
    """  # noqa: DOC107
    JointType = newton_module.JointType
    steps = int(q_steps.shape[0])
    num_dof = int(q_steps.shape[1])

    joints_per_world = int(model.joint_count) // world_count
    joint_type = model.joint_type.numpy()[:joints_per_world]
    joint_parent = model.joint_parent.numpy()[:joints_per_world]
    joint_child = model.joint_child.numpy()[:joints_per_world]
    joint_q_start = model.joint_q_start.numpy()
    joint_qd_start = model.joint_qd_start.numpy()
    joint_x_p = np.asarray(model.joint_X_p.numpy())[:joints_per_world].reshape(joints_per_world, 7)
    joint_x_c = np.asarray(model.joint_X_c.numpy())[:joints_per_world].reshape(joints_per_world, 7)
    joint_axis = np.asarray(model.joint_axis.numpy())
    gravity = np.asarray(model.gravity.numpy(), dtype=np.float64).reshape(-1, 3)[0]

    masses = np.asarray(baseline_mass, dtype=np.float64).reshape(-1)
    coms = np.asarray(baseline_com, dtype=np.float64).reshape(-1, 3)
    inertias = np.asarray(baseline_inertia, dtype=np.float64).reshape(-1, 3, 3)

    # Full world-0 configuration and rates per step: measured values on mapped
    # coords/DOFs, defaults (held pose, zero rates) elsewhere.
    q_full = np.tile(np.asarray(default_joint_q_world0[:coords_per_world], dtype=np.float64), (steps, 1))
    q_full[:, np.asarray(coord_map_world0[:num_dof], dtype=np.int64)] = q_steps.astype(np.float64)
    qd_full = np.zeros((steps, dofs_per_world), dtype=np.float64)
    qdd_full = np.zeros((steps, dofs_per_world), dtype=np.float64)
    if qd_steps is not None:
        qd_full[:, np.asarray(dof_map_world0[:num_dof], dtype=np.int64)] = qd_steps.astype(np.float64)
    if qdd_steps is not None:
        qdd_full[:, np.asarray(dof_map_world0[:num_dof], dtype=np.int64)] = qdd_steps.astype(np.float64)

    identity_q = np.zeros((steps, 4), dtype=np.float64)
    identity_q[:, 3] = 1.0
    body_p: dict[int, np.ndarray] = {}
    body_q: dict[int, np.ndarray] = {}
    anchor_p = np.zeros((joints_per_world, steps, 3), dtype=np.float64)
    anchor_q = np.zeros((joints_per_world, steps, 4), dtype=np.float64)

    for joint in range(joints_per_world):
        parent = int(joint_parent[joint])
        if parent >= 0 and parent in body_p:
            parent_p, parent_q = body_p[parent], body_q[parent]
        else:
            parent_p = np.zeros((steps, 3), dtype=np.float64)
            parent_q = identity_q
        x_p = joint_x_p[joint]
        a_p, a_q = _transform_mul(parent_p, parent_q, x_p[:3][None, :], x_p[3:][None, :])
        anchor_p[joint], anchor_q[joint] = a_p, a_q

        jtype = int(joint_type[joint])
        coord = int(joint_q_start[joint])
        dof = int(joint_qd_start[joint])
        coord_count = int(joint_q_start[joint + 1]) - coord
        if jtype == int(JointType.REVOLUTE):
            j_p = np.zeros((steps, 3), dtype=np.float64)
            j_q = _quat_from_axis_angle(joint_axis[dof], q_full[:, coord])
        elif jtype == int(JointType.PRISMATIC):
            j_p = q_full[:, coord, None] * joint_axis[dof][None, :]
            j_q = identity_q
        # A generic USD joint with every axis locked — the usual way a fixed base
        # is authored — imports as a D6 joint carrying no coordinates, which is
        # kinematically a FIXED joint.
        elif jtype == int(JointType.FIXED) or (jtype == int(JointType.D6) and coord_count == 0):
            j_p = np.zeros((steps, 3), dtype=np.float64)
            j_q = identity_q
        elif jtype in (int(JointType.FREE), int(JointType.DISTANCE)):
            j_p = q_full[:, coord : coord + 3]
            j_q = q_full[:, coord + 3 : coord + 7]
        else:
            raise ValueError(
                f"Gravity feedforward does not support joint type {JointType(jtype).name} "
                f"({jtype}, joint {joint}). Set simulation.newton.feedforward='none' for this articulation."
            )
        c_p, c_q = _transform_mul(a_p, a_q, j_p, j_q)
        x_c = joint_x_c[joint]
        inv_q = _quat_conj(x_c[3:])[None, :]
        inv_p = -_quat_rotate(inv_q, x_c[:3][None, :])
        body_p[int(joint_child[joint])], body_q[int(joint_child[joint])] = _transform_mul(c_p, c_q, inv_p, inv_q)

    # World CoM positions.
    com_world: dict[int, np.ndarray] = {
        body: body_p[body] + _quat_rotate(body_q[body], coms[body][None, :])
        for body in body_p
        if body < masses.shape[0]
    }

    # Kinematic sweep: world-frame angular velocity/acceleration and CoM
    # acceleration per body. Gravity enters as a D'Alembert base acceleration of
    # -g, so the backward sweep needs no separate gravity term and the zero-rate
    # case reduces exactly to static gravity compensation.
    minus_g_row = np.broadcast_to(-gravity, (steps, 3))
    zeros3 = np.zeros((steps, 3), dtype=np.float64)
    omega: dict[int, np.ndarray] = {}
    alpha: dict[int, np.ndarray] = {}
    acom: dict[int, np.ndarray] = {}
    for joint in range(joints_per_world):
        parent = int(joint_parent[joint])
        child = int(joint_child[joint])
        if child not in com_world:
            continue
        jtype = int(joint_type[joint])
        if jtype in (int(JointType.FREE), int(JointType.DISTANCE)):
            raise ValueError(
                "Feedforward torque requires a fixed-base articulation; found a FREE/DISTANCE joint. "
                "Set simulation.newton.feedforward='none' for floating-base robots."
            )
        o_j = anchor_p[joint]
        if parent >= 0 and parent in acom:
            w_p, al_p = omega[parent], alpha[parent]
            r_a = o_j - com_world[parent]
            a_anchor = acom[parent] + np.cross(al_p, r_a) + np.cross(w_p, np.cross(w_p, r_a))
        else:
            w_p, al_p = zeros3, zeros3
            a_anchor = minus_g_row
        r_c = com_world[child] - o_j
        dof = int(joint_qd_start[joint])
        if jtype == int(JointType.REVOLUTE):
            axis_w = _quat_rotate(anchor_q[joint], joint_axis[dof][None, :])
            w_rel = axis_w * qd_full[:, dof, None]
            w_c = w_p + w_rel
            al_c = al_p + axis_w * qdd_full[:, dof, None] + np.cross(w_p, w_rel)
            a_c = a_anchor + np.cross(al_c, r_c) + np.cross(w_c, np.cross(w_c, r_c))
        elif jtype == int(JointType.PRISMATIC):
            axis_w = _quat_rotate(anchor_q[joint], joint_axis[dof][None, :])
            w_c, al_c = w_p, al_p
            v_rel = axis_w * qd_full[:, dof, None]
            a_rel = axis_w * qdd_full[:, dof, None]
            a_c = (
                a_anchor + a_rel + 2.0 * np.cross(w_p, v_rel) + np.cross(al_c, r_c) + np.cross(w_c, np.cross(w_c, r_c))
            )
        else:  # FIXED, or a fully locked generic joint
            w_c, al_c = w_p, al_p
            a_c = a_anchor + np.cross(al_c, r_c) + np.cross(w_c, np.cross(w_c, r_c))
        omega[child], alpha[child], acom[child] = w_c, al_c, a_c

    # Dynamic backward sweep: accumulate interbody wrenches up the tree (reverse
    # topological order) and project each mapped joint's wrench onto its axis.
    dof_to_joint = {}
    for joint in range(joints_per_world):
        for dof in range(int(joint_qd_start[joint]), int(joint_qd_start[joint + 1])):
            dof_to_joint[dof] = joint
    f_acc: dict[int, np.ndarray] = {}
    n_acc: dict[int, np.ndarray] = {}
    joint_tau: dict[int, np.ndarray] = {}
    for joint in reversed(range(joints_per_world)):
        child = int(joint_child[joint])
        if child not in com_world:
            continue
        rot = _quat_to_matrix(body_q[child])
        inertia_w = rot @ inertias[child] @ np.transpose(rot, (0, 2, 1))
        w_b = omega[child]
        torque_b = np.einsum("tij,tj->ti", inertia_w, alpha[child]) + np.cross(
            w_b, np.einsum("tij,tj->ti", inertia_w, w_b)
        )
        f_total = masses[child] * acom[child] + f_acc.pop(child, 0.0)
        n_total = torque_b + n_acc.pop(child, 0.0)
        jtype = int(joint_type[joint])
        dof = int(joint_qd_start[joint])
        if jtype == int(JointType.REVOLUTE):
            axis_w = _quat_rotate(anchor_q[joint], joint_axis[dof][None, :])
            lever = com_world[child] - anchor_p[joint]
            joint_tau[joint] = np.einsum("ti,ti->t", axis_w, n_total + np.cross(lever, f_total))
        elif jtype == int(JointType.PRISMATIC):
            axis_w = _quat_rotate(anchor_q[joint], joint_axis[dof][None, :])
            joint_tau[joint] = np.einsum("ti,ti->t", axis_w, f_total)
        parent = int(joint_parent[joint])
        if parent >= 0 and parent in com_world:
            f_acc[parent] = f_acc.get(parent, 0.0) + f_total
            n_acc[parent] = n_acc.get(parent, 0.0) + n_total + np.cross(com_world[child] - com_world[parent], f_total)

    feedforward = np.zeros((steps, num_dof), dtype=np.float64)
    for column, dof in enumerate(np.asarray(dof_map_world0[:num_dof], dtype=np.int64)):
        joint = dof_to_joint[int(dof)]
        if joint in joint_tau:
            feedforward[:, column] = joint_tau[joint]
    return feedforward.astype(np.float32)
