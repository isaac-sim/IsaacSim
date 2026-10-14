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
"""Inertia tensor reconstruction from USD MassAPI principal axes + diagonal inertia."""

from __future__ import annotations

from dataclasses import dataclass

from pxr import Gf, Usd, UsdPhysics

_NEWTON_INERTIA_ATTR = "newton:inertia"


@dataclass
class InertiaData:
    """URDF inertial element data."""

    mass: float = 0.0
    origin_xyz: tuple[float, float, float] = (0.0, 0.0, 0.0)
    origin_rpy: tuple[float, float, float] = (0.0, 0.0, 0.0)
    ixx: float = 0.0
    ixy: float = 0.0
    ixz: float = 0.0
    iyy: float = 0.0
    iyz: float = 0.0
    izz: float = 0.0


def reconstruct_inertia_tensor(
    principal_axes: Gf.Quatf, diagonal_inertia: Gf.Vec3f
) -> tuple[float, float, float, float, float, float]:
    """Reconstruct the 3x3 inertia tensor from eigenvalue decomposition.

    I = R^T * diag(D) * R

    Args:
        principal_axes: Quaternion encoding the principal axes rotation.
        diagonal_inertia: Principal moments of inertia (eigenvalues).

    Returns:
        (ixx, ixy, ixz, iyy, iyz, izz) the 6 unique inertia tensor values.
    """
    R = Gf.Matrix3d(Gf.Rotation(Gf.Quatd(principal_axes)))
    D = Gf.Matrix3d()
    D.SetDiagonal(Gf.Vec3d(diagonal_inertia[0], diagonal_inertia[1], diagonal_inertia[2]))

    I = R.GetTranspose() * D * R

    return (I[0][0], I[0][1], I[0][2], I[1][1], I[1][2], I[2][2])


def read_newton_inertia(prim: Usd.Prim) -> tuple[float, float, float, float, float, float] | None:
    """Read the NewtonMassAPI ``newton:inertia`` tensor if authored.

    The attribute holds the symmetric 3x3 inertia tensor as 6 elements
    ``[Ixx, Iyy, Izz, Ixy, Ixz, Iyz]`` in the body's local frame, in double
    precision. When authored it overrides ``physics:diagonalInertia`` and
    ``physics:principalAxes``.

    Args:
        prim: USD prim to inspect.

    Returns:
        (ixx, iyy, izz, ixy, ixz, iyz) or None if not authored or malformed.
    """
    attr = prim.GetAttribute(_NEWTON_INERTIA_ATTR)
    if not attr or not attr.HasAuthoredValue():
        return None
    values = attr.Get()
    if values is None or len(values) != 6:
        return None
    return tuple(float(v) for v in values)


def read_inertial_from_prim(prim: Usd.Prim) -> InertiaData | None:
    """Read inertial data from a prim's UsdPhysicsMassAPI.

    Args:
        prim: USD prim with MassAPI applied.

    Returns:
        InertiaData or None if no mass data is available.
    """
    if not prim.HasAPI(UsdPhysics.MassAPI):
        return None

    mass_api = UsdPhysics.MassAPI(prim)

    mass_attr = mass_api.GetMassAttr()
    mass = mass_attr.Get() if mass_attr and mass_attr.HasAuthoredValue() else None
    if mass is None or mass <= 0.0:
        return None

    data = InertiaData(mass=float(mass))

    com_attr = mass_api.GetCenterOfMassAttr()
    if com_attr and com_attr.HasAuthoredValue():
        com = com_attr.Get()
        if com is not None:
            data.origin_xyz = (float(com[0]), float(com[1]), float(com[2]))

    diag_attr = mass_api.GetDiagonalInertiaAttr()
    axes_attr = mass_api.GetPrincipalAxesAttr()

    diag = diag_attr.Get() if diag_attr and diag_attr.HasAuthoredValue() else None
    axes = axes_attr.Get() if axes_attr and axes_attr.HasAuthoredValue() else None

    newton_inertia = read_newton_inertia(prim)

    if newton_inertia is not None:
        ixx, iyy, izz, ixy, ixz, iyz = newton_inertia
        data.ixx = ixx
        data.iyy = iyy
        data.izz = izz
        data.ixy = ixy
        data.ixz = ixz
        data.iyz = iyz
    elif diag is not None and axes is not None:
        ixx, ixy, ixz, iyy, iyz, izz = reconstruct_inertia_tensor(axes, diag)
        data.ixx = ixx
        data.ixy = ixy
        data.ixz = ixz
        data.iyy = iyy
        data.iyz = iyz
        data.izz = izz
    elif diag is not None:
        data.ixx = float(diag[0])
        data.iyy = float(diag[1])
        data.izz = float(diag[2])

    return data
