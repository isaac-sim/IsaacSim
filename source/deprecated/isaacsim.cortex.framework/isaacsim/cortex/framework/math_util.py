# SPDX-FileCopyrightText: Copyright (c) 2020-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

"""Module containing a collection of math utilities.

Conventions:
- Quaternion has elements (w,x,y,z) compatible with the core API.
- Function names with "T" are referring to a homogeneous transform matrix.
- Functions with pq in their name are referencing the "position, quaternion" rotation
  representation. For instance, T2pq(T) takes a transform represented as a homogeneous transform
  matrix T and converts it to a pair (p,q) where p is the position, and q is the quaternion
  rotation. Note: "2" is shorthand for "to".
"""

from __future__ import annotations

import copy
from collections.abc import Sequence
from typing import Optional

import numpy as np
from isaacsim.core.utils.math import normalized
from isaacsim.core.utils.rotations import (
    quat_to_rot_matrix,
    rot_matrix_to_quat,
)
from isaacsim.core.utils.stage import get_stage_units
from numpy.linalg import norm


def transform_dist(T1: np.ndarray, T2: np.ndarray, position_scalar: float, rotation_matrix_scalar: float) -> float:
    """Measures the distance between the provided transforms.

    Generally translation and rotational distances are not composable into a unique distance metric
    since transform distance between two geometric objects in a scene is really a function of the
    geometry of the objects (e.g. distance between surface points) rather than an object-independent
    function of translation and rotation.

    However, a reasonable proxy is to measure the simple Euclidean distance between the position
    components and rotation matrices. Those two components won't be comparable, but they can be
    weighed together into a good metric.

    Note that a rotation matrix is defined by placing the frame's (normalized) axes into the columns
    of a matrix. Specifically, if the frame is defined by {o, ax, ay, az}, where o is the origin
    (position) and a{x,y,z} are the three 3D axes, the rotation matrix is R = [ax, ay, az].
    Therefore, the Euclidean distance between two rotation matrices R1 and R2 is simply the distance
    between end-points of the axes of the two frames extending from a common origin.

    Equation:

      dist = position_scalar * |o1 - o2| + rotation_matrix_scalar * |R1 - R2|

    Args:
        T1: The first transform to be compared as a 4x4 homogeneous transform matrix.
        T2: The second transform to be compared as a 4x4 homogeneous transform matrix.
        position_scalar: The scalar weight on the position distance.
        rotation_matrix_scalar: the scalar weight on the rotation matrix distance.

    Returns:
        The distance between T1 and T2 as given by the above equation.
    """
    R1, p1 = unpack_T(T1)
    R2, p2 = unpack_T(T2)
    n = np.linalg.norm
    return position_scalar * n(p2 - p1) + rotation_matrix_scalar * n(R2 - R1)


def transforms_are_close(
    T1: np.ndarray, T2: np.ndarray, p_thresh: float, R_thresh: float, verbose: Optional[bool] = False
) -> bool:
    """Measures whether the two provided transforms T1 and T2 are close to each other.

    T1, T2 should both be 4x4 homogeneous matrices. p_thresh is the "close" threshold for the
    position difference, and R_thresh is the "close" threshold for the average rotation difference
    of the axes.

    Formula:

      close = |p1-p2| <= p_thresh and |R1-R2|/3 <= R_thresh

    Note that the rotation matrix columns are the frame axes. See the comment in transform_dist()
    for a more detailed description of the intuition behind Euclidean distance between two rotation
    matrices.

    Args:
        T1: The first transform being compared as a 4x4 homogeneous transform matrix.
        T2: The second transform being compared as a 4x4 homogeneous transform matrix.
        p_thresh: The positional threshold defining "close" in position space.
        R_thresh: The rotational threshold defining "close" in rotation space.
        verbose: An optional flag to turn on diagnostic prints.

    Returns:
        True if the T1 and T2 are close per the thresholds {p,R}_thresh. False otherwise.
    """
    Te = T1 - T2
    Re, pe = unpack_T(Te)

    npe = np.linalg.norm(pe)
    nRe = np.linalg.norm(Re)

    # Since there are three axes, we look at the average rotational error to make the units
    # comparable.
    thresh_met = npe <= p_thresh and nRe / 3 <= R_thresh
    if verbose:
        print(f"npe: {npe} vs p_thresh: {p_thresh}; nRe: {nRe} vs R_thresh: {R_thresh}")
    return thresh_met


def matrix_to_quat(mat: np.ndarray) -> np.ndarray:
    """Converts the provided rotation matrix into a quaternion in (w, x, y, z) order.

    Args:
        mat: A 3x3 rotation matrix.

    Returns:
        The quaternion corresponding to the provided rotation matrix.
    """
    return rot_matrix_to_quat(mat)


class Quaternion:
    """A convenience class for abstracting quaternions.

    Args:
        vals: The underlying quaternion data in the order [w,x,y,z]
    """

    def __init__(self, vals: Sequence[float]) -> None:
        self.vals = vals

    def __mul__(self, other: Quaternion) -> Quaternion:
        """An implementation of quaternion right multiplication.

        If this quaternion is q and the other quaternion is q_other, computes and returns

            q_prod = q * q_other

        Args:
            other: The other quaternion multiplying this on the right.

        Returns:
            A Quaternion abstracting the resulting product quaternion.
        """
        w0, x0, y0, z0 = self.vals
        w1, x1, y1, z1 = other.vals

        w = w0 * w1 - x0 * x1 - y0 * y1 - z0 * z1
        x = w0 * x1 + x0 * w1 + y0 * z1 - z0 * y1
        y = w0 * y1 - x0 * z1 + y0 * w1 + z0 * x1
        z = w0 * z1 + x0 * y1 - y0 * x1 + z0 * w1

        return Quaternion([w, x, y, z])


def reorder_q_xyzw2wxyz(q: np.ndarray) -> np.ndarray:
    """Reorders the given quaternion from (x,y,z,w) order (ROS convention) to (w,x,y,z) order.

    (Isaac Sim core API convention).

    Args:
        q: The quaternion in (x,y,z,w) order.

    Returns:
        The quaternion in (w,x,y,z) order.
    """
    return np.array([q[3], q[0], q[1], q[2]])


def reorder_q_wxyz2xyzw(q: np.ndarray) -> np.ndarray:
    """Reorders the given quaternion from (w,x,y,z) order (Isaac Sim core API convention) to.

    (x,y,z,w) order (ROS convention).

    Args:
        q: The quaternion in (w,x,y,z) order.

    Returns:
        The quaternion in (x,y,z,w) order.
    """
    return np.array([q[1], q[2], q[3], q[0]])


def to_homogeneous_vec(v: np.ndarray) -> np.ndarray:
    """Converts the provided 3D vector into a 4D homogeneous vector padded with 1 in the final.

    dimension.

    Args:
        v: The 3d vector to convert.

    Returns:
        A 4d vector with the first 3 components containing v and a 1 in the final component.
    """
    hv = np.ones(4)
    hv[:3] = v
    return hv


def apply_T(T: np.ndarray, v: np.ndarray) -> np.ndarray:  # noqa: N802
    """Applies the 4x4 homogeneous transform matrix T to the provided 3D vector v. Returns the.

    transformed 3D vector.

    Args:
        T: The 4x4 homogeneous transform matrix to apply to the vector v.
        v: The 3d vector v which will be transformed.

    Returns:
        A 3d vector representing the first 3 components of T * [v;1].
    """
    return T.dot(to_homogeneous_vec(v))[:3]


def T2pq(T: np.ndarray) -> tuple[np.ndarray, np.ndarray]:  # noqa: N802
    """Converts a 4x4 homogeneous matrix to a position-quaternion representation.

    Args:
        T: A 4x4 homogeneous transform matrix.

    Returns:
        A tuple (p,q) where p is a 3D position vector and q is a 4D quaternion vector.
    """
    R, p = unpack_T(T)
    return p, matrix_to_quat(R)


def pq2T(p: np.ndarray, q: np.ndarray) -> np.ndarray:  # noqa: N802
    """Converts a pose given as (<position>,<quaternion>) to a 4x4 homogeneous transform matrix.

    Args:
        p: 3D position vector
        q: 4D quaternion vector

    Returns:
        A 4x4 homogeneous transform matrix.
    """
    return pack_Rp(quat_to_rot_matrix(q), p)


def R2T(R: np.ndarray) -> np.ndarray:  # noqa: N802
    """Expands a rotation matrix to be a 4x4 homogeneous matrix by padding it with a zero position.

    vector.

    Args:
        R: A 3x3 rotation matrix.

    Returns:
        A zero padded 4x4 homogeneous transform matrix T = [R, 0; 0, 1].
    """
    T = np.eye(4)
    T[:3, :3] = R
    return T


def proj_orth(
    v1: np.ndarray, v2: np.ndarray, normalize_res: Optional[bool] = False, eps: Optional[float] = 1e-5
) -> np.ndarray:
    """Projects v1 orthogonal to v2. If v2 is zero (within eps), v1 is returned unchanged. If.

    normalize_res is true, normalizes the result before returning.

    Args:
        v1: The vector to be projected.
        v2: The vector defining the desired orthogonal space.
        normalize_res: If True, the resulting projected vector is normalized before being returned.
        eps: If the norm of v2 is smaller than this, we consider it to be the zero vector and simply
            return v1. (It considers all vectors to be already orthogonal to zero.)

    Returns:
        The projected (and potentially normalized) copy of v1.
    """
    v2_norm = norm(v2)
    if v2_norm < eps:
        return v1

    v2n = v2 / v2_norm
    v1 = v1 - np.dot(v1, v2n) * v2n
    if normalize_res:
        return normalized(v1)
    else:
        return v1


def unpack_T(T: np.ndarray) -> tuple[np.ndarray, np.ndarray]:  # noqa: N802
    """Unpack the rotation matrix and translation separately from a 4x4 homogeneous transform.

    matrix.

    Args:
        T: The 4x4 homogeneous transform matrix to split.

    Returns:
        A tuple (R, p) where R is the 3x3 rotation matrix and p is the 3d position vector.
    """
    return T[:3, :3], T[:3, 3]


def unpack_R(R: np.ndarray) -> tuple[np.ndarray, np.ndarray, np.ndarray]:  # noqa: N802
    """Unpack the individual axes (columns) of the rotation matrix.

    Args:
        R: A 3x3 rotation matrix to be split.

    Returns:
        A tuple (ax, ay, az) where R = [ax, ay, az]. Each vector is an axis of the frame represented by R.
    """
    return R[:3, 0], R[:3, 1], R[:3, 2]


def pack_R(ax: np.ndarray, ay: np.ndarray, az: np.ndarray, as_homogeneous: bool = False) -> np.ndarray:  # noqa: N802
    """Pack a rotation matrix with the supplied axis columns.

    Args:
        ax: The x-axis of a rotation matrix (column 1).
        ay: The y-axis of a rotation matrix (column 2).
        az: The z-axis of a rotation matrix (column 3).
        as_homogeneous: If True, returns a 4x4 homogeneous matrix instead of 3x3.

    Returns:
        A rotation matrix R = [ax, ay, az] with the axes as columns.
    """
    if as_homogeneous:
        R = np.eye(4)
    else:
        R = np.eye(3)
    R[:3, 0] = ax
    R[:3, 1] = ay
    R[:3, 2] = az
    return R


def pack_Rp(R: np.ndarray, p: np.ndarray) -> np.ndarray:  # noqa: N802
    """Packs the provided rotation matrix (R) and position (p) into a 4x4 homogeneous transform.

    matrix.

    Args:
        R: A 3x3 rotation matrix.
        p: a 3D position vector.

    Returns:
        A 4x4 homogeneous transform matrix T = [R, p; 0, 1] formed from the frame (R,p).
    """
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = p
    return T


def invert_T(T: np.ndarray) -> np.ndarray:  # noqa: N802
    """Inverts the provided transform matrix using the explicit formula leveraging the.

    orthogonality of R and the sparsity of the transform.

    Specifically, denote T = h(R, t) where h(.,.) is a function mapping the rotation R and
    translation t to a homogeneous matrix defined by those parameters. Then

      inv(T) = inv(h(R,t)) = h(R', -R't).

    Args:
        T: A 4x4 homogeneous transform matrix.

    Returns:
        inv(T) using the computation outlined above.
    """
    R, t = unpack_T(T)
    R_trans = R.T
    return pack_Rp(R_trans, -R_trans.dot(t))


class ExpAvg(object):
    """Computes the exponential weighted average of a stream of values.

    Represents the weighted average defined recursively by

      avg_val_t = gamma avg_val_{t-1} + (1-gamma) val_t

    Expanding recursively, this gives the following weighed average

      avg_val_t = w_1 val_1 + ... + w_t val_t

    with w_t = (1-gamma) gamma^t. These weights are geometric so the sum is

      w_1 + ... + w_t = 1-gamma^t

    which converges exponentially to 1 as t increases. So the running average converges
    exponentially on a running geometrically weighted average, updating continually with each
    incoming value.

    Intuitively, gammas closer to 1 correspond to larger effective window size, while gammas closer
    to 0 correspond to smaller window size. Values closer to 1 weigh the previous values more and
    the latest value less.

    If a ballpark estimate of the average is available, that can be provided as prior_avg to reduce
    bias of the initial estimates. Providing a prior_avg will set the initial value of self.val_avg
    to the prior_avg, and that prior value will be available immediately via self.val_avg.

    The latest average value can be accessed by self.val_avg.

    Args:
        gamma: The gamma value for averaging. This is how strongly to weigh the previous weighted
            average.
        prior_avg: An optional seed value for the val_avg. If it's not provided, the val_avg field
            is seeded with the first value.
    """

    def __init__(self, gamma: float, prior_avg: Optional[float] = None) -> None:
        self.gamma = gamma
        self.prior_avg = prior_avg
        self.reset()

    def reset(self) -> None:
        """Reset this exponential average, clearing the previous value."""
        self.val_avg = self.prior_avg

    def is_ready(self) -> bool:
        """Query if at least one value has been consumed, and false otherwise.

        Returns:
            True if at least one value has been consumed (after construction or the latest reset).
        """
        return self.val_avg is not None

    def update(self, val: float) -> None:
        """Update the average with the provided value. When the first value is consumed, the.

        average is set to that value explicitly is no prior value is specified.

        Args:
            val: The value being averaged in.
        """
        if self.val_avg is None:
            self.val_avg = val
            return

        self.val_avg = self.gamma * self.val_avg + (1.0 - self.gamma) * val


def proj_R(R: np.ndarray) -> np.ndarray:  # noqa: N802
    """Projects a rotational matrix to make it a valid rotation.

    The projection is performed by first converting the rotation matrix components into a
    quaternion, thennormalizing the quaternion and converting it back to a rotation matrix.

    Args:
        R: The 3x3 matrix representing an approximate rotation matrix.

    Returns:
        The projected version of R.
    """
    q = matrix_to_quat(R)
    q /= np.linalg.norm(q)
    R = quat_to_rot_matrix(q)
    return R


def proj_T(T: np.ndarray) -> np.ndarray:  # noqa: N802
    """Projects the rotational matrix portion of the provide homogeneous transform matrix to make.

    it a valid rotation.

    The projection is performed by first converting the rotation matrix components into a
    quaternion followed by normalizing the quaternion.

    The modification is not performed inline, so a copy of the transform is created for returning
    and the parameter T is left untouched.


    Args:
        T: The unprojected 4x4 homogeneous transform matrix. The rotation portion need not be an
            exact rotation matrix. It will be projected.

    Returns:
        The projected copy of the 4x4 homogeneous transform matrix.
    """
    T = copy.deepcopy(T)
    T[:3, :3] = proj_R(T[:3, :3])
    return T


def make_rotation_matrix(az_dominant: np.ndarray, ax_suggestion: np.ndarray) -> np.ndarray:
    """Constructs a rotation matrix with the z-axis given by az_dominant (normalized), and the.

    x-axis given by a orthogonally projected version of ax_suggestion. The y-axis is formed via the
    right hand rule.

    Args:
        az_dominant: The z-axis vector to be used as the dominant z axis. This axis vector won't
            change direction, but it will be normalized.
        ax_suggestion: A x-axis suggestion vector. This axis will be projected to be orthogonal to
            the az_dominant axis, then normalized.

    Returns:
        A 3x3 rotation matrix constructed from the args as described above.
    """
    az = normalized(az_dominant)
    ax = proj_orth(ax_suggestion, az)
    ay = np.cross(az, ax)
    return pack_R(ax, ay, az)


def to_meters(p_stage: np.ndarray) -> np.ndarray:
    """Convert the position p_stage from stage units to meters.

    By default, a stage uses meters, so this method does nothing. But if the world is constructed
    with different units, this method will convert those units to meters.

    Args:
        p_stage: The position vector in stage units.

    Returns:
        The position vector in meters.
    """
    return p_stage * get_stage_units()


def T_to_meters(T_stage: np.ndarray) -> np.ndarray:  # noqa: N802
    """Convert the homogeneous transform to meters. This method simply makes a copy of T_stage and.

    converts the translation components to meters using a call to to_meters().

    Args:
        T_stage: A 4x4 homogeneous transform matrix with position components in stage units.

    Returns:
        A copy of the 4x4 homogeneous transform matrix with position components in meters.
    """
    T_meters = copy.deepcopy(T_stage)
    T_meters[:3, 3] = to_meters(T_meters[:3, 3])
    return T_meters


def to_stage_units(p_meters: np.ndarray) -> np.ndarray:
    """Convert the position p_meters from meters to stage units.

    Args:
        p_meters: A position vector in meters.

    Returns:
        The position vector in stage units.
    """
    return p_meters / get_stage_units()
