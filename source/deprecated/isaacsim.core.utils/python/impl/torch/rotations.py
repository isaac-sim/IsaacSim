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

"""Provides PyTorch-based utilities for 3D rotations, quaternions, and coordinate transformations."""

from __future__ import annotations

import numpy as np
from isaacsim.core.deprecation_manager import import_module
from isaacsim.core.utils.torch.maths import copysign, normalize  # noqa: F401
from pxr import Gf
from scipy.spatial.transform import Rotation

torch = import_module("torch")


def gf_quat_to_tensor(orientation: Gf.Quatd | Gf.Quatf | Gf.Quaternion, device: object = None) -> torch.Tensor:
    """Converts a pxr Quaternion type to a torch array (scalar first).

    Args:
        orientation: Input quaternion from USD.
        device: Device to place the tensor on.

    Returns:
        Quaternion as torch tensor in [w, x, y, z] format.
    """
    quat = torch.zeros(4, dtype=torch.float32, device=device)
    quat[1:] = torch.tensor(orientation.GetImaginary(), dtype=torch.float32, device=device)
    quat[0] = orientation.GetReal()
    return quat


def euler_angles_to_quats(
    euler_angles: torch.Tensor, degrees: bool = False, extrinsic: bool = True, device: object = None
) -> torch.Tensor:
    """Vectorized version of converting euler angles to quaternion (scalar first).

    Args:
        euler_angles: euler angles with shape (N, 3)
        degrees: True if degrees, False if radians.
        extrinsic: True if the euler angles follows the extrinsic angles
            convention (equivalent to ZYX ordering but returned in the reverse) and False if it follows
            the intrinsic angles conventions (equivalent to XYZ ordering).
        device: Device to place the tensor on.

    Returns:
        quaternions representation of the angles (N, 4) - scalar first.
    """
    if extrinsic:
        order = "xyz"
    else:
        order = "XYZ"
    # TODO: implement a torch version
    rot = Rotation.from_euler(order, euler_angles.cpu().numpy(), degrees=degrees)
    result = rot.as_quat()
    if len(result.shape) == 1:
        result = result[[3, 0, 1, 2]]
    else:
        result = result[:, [3, 0, 1, 2]]
    result = torch.from_numpy(np.asarray(result, dtype=np.float32)).float().to(device)
    return result


def rot_matrices_to_quats(rotation_matrices: torch.Tensor, device: object = None) -> torch.Tensor:
    """Vectorized version of converting rotation matrices to quaternions.

    Args:
        rotation_matrices: N Rotation matrices with shape (N, 3, 3) or (3, 3)
        device: Device to place the tensor on.

    Returns:
        quaternion representation of the rotation matrices (N, 4) or (4,) - scalar first
    """
    rot = Rotation.from_matrix(rotation_matrices.cpu().numpy())
    result = rot.as_quat()
    if len(result.shape) == 1:
        result = result[[3, 0, 1, 2]]
    else:
        result = result[:, [3, 0, 1, 2]]
    result = torch.from_numpy(np.asarray(result, dtype=np.float32)).float().to(device)
    return result


def rad2deg(radian_value: torch.Tensor, device: object = None) -> torch.Tensor:
    """Converts radians to degrees.

    Args:
        radian_value: Angle value in radians.
        device: Device to place the tensor on.

    Returns:
        Angle value in degrees.
    """
    return torch.rad2deg(radian_value).float().to(device)


def deg2rad(degree_value: float, device: object = None) -> torch.Tensor:
    """Converts degrees to radians.

    Args:
        degree_value: Angle value in degrees.
        device: Device to place the tensor on.

    Returns:
        Angle value in radians.
    """
    return torch.deg2rad(degree_value).float().to(device)


@torch.jit.script
def quat_mul(a: torch.Tensor, b: torch.Tensor) -> torch.Tensor:
    """Multiply two quaternion tensors (scalar first).

    Args:
        a: First quaternion tensor of shape (..., 4) in [w, x, y, z] format.
        b: Second quaternion tensor of shape (..., 4) in [w, x, y, z] format.

    Returns:
        Product quaternion tensor of same shape as inputs.
    """
    assert a.shape == b.shape
    shape = a.shape
    a = a.reshape(-1, 4)
    b = b.reshape(-1, 4)

    w1, x1, y1, z1 = a[:, 0], a[:, 1], a[:, 2], a[:, 3]
    w2, x2, y2, z2 = b[:, 0], b[:, 1], b[:, 2], b[:, 3]
    ww = (z1 + x1) * (x2 + y2)
    yy = (w1 - y1) * (w2 + z2)
    zz = (w1 + y1) * (w2 - z2)
    xx = ww + yy + zz
    qq = 0.5 * (xx + (z1 - x1) * (x2 - y2))
    w = qq - ww + (z1 - y1) * (y2 - z2)
    x = qq - xx + (x1 + w1) * (x2 + w2)
    y = qq - yy + (w1 - x1) * (y2 + z2)
    z = qq - zz + (z1 + y1) * (w2 - x2)

    quat = torch.stack([w, x, y, z], dim=-1).view(shape)

    return quat


@torch.jit.script
def quat_conjugate(a: torch.Tensor) -> torch.Tensor:
    """Compute the conjugate of a quaternion tensor (scalar first).

    Args:
        a: Quaternion tensor of shape (..., 4) in [w, x, y, z] format.

    Returns:
        Conjugate quaternion tensor of same shape.
    """
    shape = a.shape
    a = a.reshape(-1, 4)
    return torch.cat((a[:, 0:1], -a[:, 1:]), dim=-1).view(shape)


@torch.jit.script
def quat_apply(a: torch.Tensor, b: torch.Tensor) -> torch.Tensor:
    """Apply quaternion rotation to a 3D vector tensor.

    Args:
        a: Quaternion tensor of shape (..., 4) in [w, x, y, z] format.
        b: Vector tensor of shape (..., 3) to rotate.

    Returns:
        Rotated vector tensor of same shape as b.
    """
    shape = b.shape
    a = a.reshape(-1, 4)
    b = b.reshape(-1, 3)
    xyz = a[:, 1:]
    t = xyz.cross(b, dim=-1) * 2
    return (b + a[:, 0:1] * t + xyz.cross(t, dim=-1)).view(shape)


@torch.jit.script
def quat_rotate(q: torch.Tensor, v: torch.Tensor) -> torch.Tensor:
    """Rotate a vector by a quaternion.

    Args:
        q: Quaternion tensor of shape (N, 4) in [w, x, y, z] format.
        v: Vector tensor of shape (N, 3) to rotate.

    Returns:
        Rotated vector tensor of shape (N, 3).
    """
    shape = q.shape
    q_w = q[:, 0]
    q_vec = q[:, 1:]
    a = v * (2.0 * q_w**2 - 1.0).unsqueeze(-1)
    b = torch.cross(q_vec, v, dim=-1) * q_w.unsqueeze(-1) * 2.0
    c = q_vec * torch.bmm(q_vec.view(shape[0], 1, 3), v.view(shape[0], 3, 1)).squeeze(-1) * 2.0
    return a + b + c


@torch.jit.script
def quat_rotate_inverse(q: torch.Tensor, v: torch.Tensor) -> torch.Tensor:
    """Rotate a vector by the inverse of a quaternion.

    Args:
        q: Quaternion tensor of shape (N, 4) in [w, x, y, z] format.
        v: Vector tensor of shape (N, 3) to rotate.

    Returns:
        Rotated vector tensor of shape (N, 3).
    """
    shape = q.shape
    q_w = q[:, 0]
    q_vec = q[:, 1:]
    a = v * (2.0 * q_w**2 - 1.0).unsqueeze(-1)
    b = torch.cross(q_vec, v, dim=-1) * q_w.unsqueeze(-1) * 2.0
    c = q_vec * torch.bmm(q_vec.view(shape[0], 1, 3), v.view(shape[0], 3, 1)).squeeze(-1) * 2.0
    return a - b + c


@torch.jit.script
def quat_unit(a: torch.Tensor) -> torch.Tensor:
    """Normalize a quaternion to unit length.

    Args:
        a: Quaternion tensor to normalize.

    Returns:
        Unit quaternion tensor.
    """
    return normalize(a)


@torch.jit.script
def quat_from_angle_axis(angle: torch.Tensor, axis: torch.Tensor) -> torch.Tensor:
    """Create a quaternion from an angle and axis of rotation.

    Args:
        angle: Rotation angle tensor in radians.
        axis: Rotation axis tensor of shape (N, 3).

    Returns:
        Unit quaternion tensor of shape (N, 4) in [w, x, y, z] format.
    """
    theta = (angle / 2).unsqueeze(-1)
    xyz = normalize(axis) * theta.sin()
    w = theta.cos()
    return quat_unit(torch.cat([w, xyz], dim=-1))


@torch.jit.script
def quat_axis(q: torch.Tensor, axis: int = 0) -> torch.Tensor:
    """Get a basis vector rotated by the given quaternion.

    Args:
        q: Quaternion tensor of shape (N, 4) in [w, x, y, z] format.
        axis: Index of the basis vector (0, 1, or 2 for x, y, z).

    Returns:
        Rotated basis vector tensor of shape (N, 3).
    """
    basis_vec = torch.zeros(q.shape[0], 3, device=q.device)
    basis_vec[:, axis] = 1
    return quat_rotate(q, basis_vec)


@torch.jit.script
def normalize_angle(x: torch.Tensor) -> torch.Tensor:
    """Normalize an angle to the range [-pi, pi].

    Args:
        x: Input angle tensor in radians.

    Returns:
        Normalized angle tensor in the range [-pi, pi].
    """
    return torch.atan2(torch.sin(x), torch.cos(x))


@torch.jit.script
def get_basis_vector(q: torch.Tensor, v: torch.Tensor) -> torch.Tensor:
    """Get a basis vector rotated by the given quaternion.

    Args:
        q: Quaternion tensor of shape (N, 4) in [w, x, y, z] format.
        v: Basis vector tensor of shape (N, 3) to rotate.

    Returns:
        Rotated basis vector tensor of shape (N, 3).
    """
    return quat_rotate(q, v)


@torch.jit.script
def quats_to_rot_matrices(quats: torch.Tensor) -> torch.Tensor:
    """Convert quaternions (scalar first) to rotation matrices.

    Args:
        quats: Quaternion tensor of shape (N, 4) or (4,) in [w, x, y, z] format.

    Returns:
        Rotation matrix tensor of shape (N, 3, 3) or (3, 3).
    """
    squeeze_flag = False
    if quats.dim() == 1:
        squeeze_flag = True
        quats = torch.unsqueeze(quats, 0)
    nq = torch.linalg.vecdot(quats, quats, dim=1)
    singularities = nq < 1e-10
    result = torch.zeros(quats.shape[0], 3, 3, device=quats.device)
    result[singularities] = torch.eye(3, device=quats.device).reshape((1, 3, 3)).repeat(sum(singularities), 1, 1)
    non_singular = quats[torch.logical_not(singularities)] * torch.sqrt(2.0 / nq).reshape((-1, 1)).repeat(1, 4)
    non_singular = torch.einsum("bi,bj->bij", non_singular, non_singular)
    result[torch.logical_not(singularities), 0, 0] = 1.0 - non_singular[:, 2, 2] - non_singular[:, 3, 3]
    result[torch.logical_not(singularities), 0, 1] = non_singular[:, 1, 2] - non_singular[:, 3, 0]
    result[torch.logical_not(singularities), 0, 2] = non_singular[:, 1, 3] + non_singular[:, 2, 0]
    result[torch.logical_not(singularities), 1, 0] = non_singular[:, 1, 2] + non_singular[:, 3, 0]
    result[torch.logical_not(singularities), 1, 1] = 1.0 - non_singular[:, 1, 1] - non_singular[:, 3, 3]
    result[torch.logical_not(singularities), 1, 2] = non_singular[:, 2, 3] - non_singular[:, 1, 0]
    result[torch.logical_not(singularities), 2, 0] = non_singular[:, 1, 3] - non_singular[:, 2, 0]
    result[torch.logical_not(singularities), 2, 1] = non_singular[:, 2, 3] + non_singular[:, 1, 0]
    result[torch.logical_not(singularities), 2, 2] = 1.0 - non_singular[:, 1, 1] - non_singular[:, 2, 2]
    if squeeze_flag:
        result = torch.squeeze(result)
    return result


@torch.jit.script
def matrices_to_euler_angles(mat: torch.Tensor, extrinsic: bool = True) -> torch.Tensor:
    """Convert rotation matrices to Euler angles (XYZ convention).

    Args:
        mat: Rotation matrix tensor of shape (N, 3, 3).
        extrinsic: If True, uses extrinsic XYZ convention; otherwise intrinsic.

    Returns:
        Euler angles tensor of shape (N, 3) in radians.
    """
    _POLE_LIMIT = 1.0 - 1e-6
    if extrinsic:
        north_pole = mat[:, 2, 0] > _POLE_LIMIT
        south_pole = mat[:, 2, 0] < -_POLE_LIMIT
        result = torch.zeros(mat.shape[0], 3, device=mat.device)
        result[north_pole, 0] = 0.0
        result[north_pole, 1] = -np.pi / 2
        result[north_pole, 2] = torch.arctan2(mat[north_pole, 0, 1], mat[north_pole, 0, 2])
        result[south_pole, 0] = 0.0
        result[south_pole, 1] = np.pi / 2
        result[south_pole, 2] = torch.arctan2(mat[south_pole, 0, 1], mat[south_pole, 0, 2])
        result[torch.logical_not(torch.logical_or(south_pole, north_pole)), 0] = torch.arctan2(
            mat[torch.logical_not(torch.logical_or(south_pole, north_pole)), 2, 1],
            mat[torch.logical_not(torch.logical_or(south_pole, north_pole)), 2, 2],
        )
        result[torch.logical_not(torch.logical_or(south_pole, north_pole)), 1] = -torch.arcsin(
            mat[torch.logical_not(torch.logical_or(south_pole, north_pole)), 2, 0]
        )
        result[torch.logical_not(torch.logical_or(south_pole, north_pole)), 2] = torch.arctan2(
            mat[torch.logical_not(torch.logical_or(south_pole, north_pole)), 1, 0],
            mat[torch.logical_not(torch.logical_or(south_pole, north_pole)), 0, 0],
        )
    else:
        north_pole = mat[:, 2, 0] > _POLE_LIMIT
        south_pole = mat[:, 2, 0] < -_POLE_LIMIT
        result = torch.zeros(mat.shape[0], 3, device=mat.device)
        result[north_pole, 0] = torch.arctan2(mat[north_pole, 1, 0], mat[north_pole, 1, 1])
        result[north_pole, 1] = np.pi / 2
        result[north_pole, 2] = 0.0
        result[south_pole, 0] = torch.arctan2(mat[south_pole, 1, 0], mat[south_pole, 1, 1])
        result[south_pole, 1] = -np.pi / 2
        result[south_pole, 2] = 0.0
        result[torch.logical_not(torch.logical_or(south_pole, north_pole)), 0] = -torch.arctan2(
            mat[torch.logical_not(torch.logical_or(south_pole, north_pole)), 1, 2],
            mat[torch.logical_not(torch.logical_or(south_pole, north_pole)), 2, 2],
        )
        result[torch.logical_not(torch.logical_or(south_pole, north_pole)), 1] = torch.arcsin(
            mat[torch.logical_not(torch.logical_or(south_pole, north_pole)), 0, 2]
        )
        result[torch.logical_not(torch.logical_or(south_pole, north_pole)), 2] = -torch.arctan2(
            mat[torch.logical_not(torch.logical_or(south_pole, north_pole)), 0, 1],
            mat[torch.logical_not(torch.logical_or(south_pole, north_pole)), 0, 0],
        )
    return result


@torch.jit.script
def get_euler_xyz(q: torch.Tensor, extrinsic: bool = True) -> tuple[torch.Tensor, torch.Tensor, torch.Tensor]:
    """Get Euler XYZ angles from quaternions (scalar first).

    Args:
        q: Quaternion tensor of shape (N, 4) in [w, x, y, z] format.
        extrinsic: If True, uses extrinsic XYZ convention; otherwise intrinsic.

    Returns:
        Tuple of (roll, pitch, yaw) tensors each of shape (N,) in radians.
    """
    if extrinsic:
        qw, qx, qy, qz = 0, 1, 2, 3
        # roll (x-axis rotation)
        sinr_cosp = 2.0 * (q[:, qw] * q[:, qx] + q[:, qy] * q[:, qz])
        cosr_cosp = q[:, qw] * q[:, qw] - q[:, qx] * q[:, qx] - q[:, qy] * q[:, qy] + q[:, qz] * q[:, qz]
        roll = torch.atan2(sinr_cosp, cosr_cosp)

        # pitch (y-axis rotation)
        sinp = 2.0 * (q[:, qw] * q[:, qy] - q[:, qz] * q[:, qx])
        pitch = torch.where(torch.abs(sinp) >= 1, copysign(np.pi / 2.0, sinp), torch.asin(sinp))

        # yaw (z-axis rotation)
        siny_cosp = 2.0 * (q[:, qw] * q[:, qz] + q[:, qx] * q[:, qy])
        cosy_cosp = q[:, qw] * q[:, qw] + q[:, qx] * q[:, qx] - q[:, qy] * q[:, qy] - q[:, qz] * q[:, qz]
        yaw = torch.atan2(siny_cosp, cosy_cosp)

        return roll % (2 * np.pi), pitch % (2 * np.pi), yaw % (2 * np.pi)
    else:
        result = matrices_to_euler_angles(quats_to_rot_matrices(q), extrinsic=False)
        return result[:, 0], result[:, 1], result[:, 2]


@torch.jit.script
def quat_from_euler_xyz(
    roll: torch.Tensor, pitch: torch.Tensor, yaw: torch.Tensor, extrinsic: bool = True
) -> torch.Tensor:
    """Create quaternions from roll, pitch, and yaw angles.

    Args:
        roll: Roll angle tensor in radians.
        pitch: Pitch angle tensor in radians.
        yaw: Yaw angle tensor in radians.
        extrinsic: If True, uses extrinsic XYZ convention; otherwise intrinsic.

    Returns:
        Quaternion tensor of shape (N, 4) in [w, x, y, z] format.
    """
    cy = torch.cos(yaw * 0.5)
    sy = torch.sin(yaw * 0.5)
    cr = torch.cos(roll * 0.5)
    sr = torch.sin(roll * 0.5)
    cp = torch.cos(pitch * 0.5)
    sp = torch.sin(pitch * 0.5)

    if extrinsic:
        qw = cy * cr * cp + sy * sr * sp
        qx = cy * sr * cp - sy * cr * sp
        qy = cy * cr * sp + sy * sr * cp
        qz = sy * cr * cp - cy * sr * sp
    else:
        qw = -sr * sp * sy + cr * cp * cy
        qx = sr * cp * cy + sp * sy * cr
        qy = -sr * sy * cp + sp * cr * cy
        qz = sr * sp * cy + sy * cr * cp

    return torch.stack([qw, qx, qy, qz], dim=-1)


@torch.jit.script
def quat_diff_rad(a: torch.Tensor, b: torch.Tensor) -> torch.Tensor:
    """Get the difference in radians between two quaternions.

    Args:
        a: first quaternion, shape (N, 4)
        b: second quaternion, shape (N, 4)

    Returns:
        Difference in radians, shape (N,)
    """
    b_conj = quat_conjugate(b)
    mul = quat_mul(a, b_conj)
    # 2 * torch.acos(torch.abs(mul[:, -1]))
    return 2.0 * torch.asin(torch.clamp(torch.norm(mul[:, 1:], p=2, dim=-1), max=1.0))


# NB: do not make this function jit, since it is passed around as an argument.
def normalise_quat_in_pose(pose: object) -> object:  # noqa: N802
    """Takes a pose and normalises the quaternion portion of it.

    Args:
        pose: shape N, 7

    Returns:
        Pose with normalised quat. Shape N, 7
    """
    pos = pose[:, 0:3]
    quat = pose[:, 3:7]
    quat /= torch.norm(quat, dim=-1, p=2).reshape(-1, 1)
    return torch.cat([pos, quat], dim=-1)


@torch.jit.script
def compute_heading_and_up(
    torso_rotation: torch.Tensor,
    inv_start_rot: torch.Tensor,
    to_target: torch.Tensor,
    vec0: torch.Tensor,
    vec1: torch.Tensor,
    up_idx: int,
) -> tuple[torch.Tensor, torch.Tensor, torch.Tensor, torch.Tensor, torch.Tensor]:
    """Compute heading and up vectors from torso rotation and target direction.

    Args:
        torso_rotation: Torso rotation quaternion tensor of shape (N, 4).
        inv_start_rot: Inverse start rotation quaternion tensor of shape (N, 4).
        to_target: Direction vector to target of shape (N, 3).
        vec0: Heading basis vector of shape (N, 3).
        vec1: Up basis vector of shape (N, 3).
        up_idx: Index of the up axis (0, 1, or 2).

    Returns:
        Tuple of (torso_quat, up_proj, heading_proj, up_vec, heading_vec).
    """
    num_envs = torso_rotation.shape[0]
    target_dirs = normalize(to_target)

    torso_quat = quat_mul(torso_rotation, inv_start_rot)
    up_vec = get_basis_vector(torso_quat, vec1).view(num_envs, 3)
    heading_vec = get_basis_vector(torso_quat, vec0).view(num_envs, 3)
    up_proj = up_vec[:, up_idx]
    heading_proj = torch.bmm(heading_vec.view(num_envs, 1, 3), target_dirs.view(num_envs, 3, 1)).view(num_envs)

    return torso_quat, up_proj, heading_proj, up_vec, heading_vec


@torch.jit.script
def compute_rot(
    torso_quat: torch.Tensor,
    velocity: torch.Tensor,
    ang_velocity: torch.Tensor,
    targets: torch.Tensor,
    torso_positions: torch.Tensor,
    extrinsic: bool = True,
) -> tuple[torch.Tensor, torch.Tensor, torch.Tensor, torch.Tensor, torch.Tensor, torch.Tensor]:
    """Compute local velocities and angles to target from torso rotation.

    Args:
        torso_quat: Torso rotation quaternion tensor of shape (N, 4).
        velocity: Linear velocity tensor of shape (N, 3).
        ang_velocity: Angular velocity tensor of shape (N, 3).
        targets: Target position tensor of shape (N, 3).
        torso_positions: Torso position tensor of shape (N, 3).
        extrinsic: If True, uses extrinsic XYZ convention; otherwise intrinsic.

    Returns:
        Tuple of (vel_loc, angvel_loc, roll, pitch, yaw, angle_to_target).
    """
    vel_loc = quat_rotate_inverse(torso_quat, velocity)
    angvel_loc = quat_rotate_inverse(torso_quat, ang_velocity)

    roll, pitch, yaw = get_euler_xyz(torso_quat, extrinsic=extrinsic)

    walk_target_angle = torch.atan2(targets[:, 2] - torso_positions[:, 2], targets[:, 0] - torso_positions[:, 0])
    angle_to_target = walk_target_angle - yaw

    return vel_loc, angvel_loc, roll, pitch, yaw, angle_to_target


def xyzw2wxyz(q: object) -> object:
    """Converts quaternion from [x, y, z, w] to [w, x, y, z] format.

    Args:
        q: Quaternion tensor in [x, y, z, w] format.

    Returns:
        Quaternion tensor in [w, x, y, z] format.
    """
    return torch.roll(q, 1, -1)


def wxyz2xyzw(q: object) -> object:
    """Converts quaternion from [w, x, y, z] to [x, y, z, w] format.

    Args:
        q: Quaternion tensor in [w, x, y, z] format.

    Returns:
        Quaternion tensor in [x, y, z, w] format.
    """
    return torch.roll(q, -1, -1)
