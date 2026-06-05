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

"""Provides utility functions for manipulating USD prim transformations and xform operations."""

from collections.abc import Sequence

import numpy as np
from pxr import Gf, Usd, UsdGeom


def prim_get_xform_op_order(prim: Usd.Prim) -> list[str]:
    """Returns the order of Xform ops on a given prim.

    Args:
        prim: The prim to get xform op order from.

    Returns:
        List of xform op names in order, or empty list if no ops exist.
    """
    x = UsdGeom.Xformable(prim)
    op_order = x.GetXformOpOrderAttr().Get()
    if op_order is not None:
        op_order = list(op_order)
        return op_order
    else:
        return []


def prim_set_xform_op_order(prim: Usd.Prim, op_order: Sequence[str]) -> Usd.Prim:
    """Sets the order of Xform ops on a given prim.

    Args:
        prim: The prim to set xform op order on.
        op_order: Sequence of xform op names in desired order.

    Returns:
        The modified prim.
    """
    x = UsdGeom.Xformable(prim)
    x.GetXformOpOrderAttr().Set(op_order)
    return prim


def prim_xform_op_move_end_to_front(prim: Usd.Prim) -> Usd.Prim:
    """Pops the last xform op on a given prim and adds it to the front.

    Args:
        prim: The USD prim to modify.

    Returns:
        The modified pform.
    """
    order = prim_get_xform_op_order(prim)
    end = order.pop(-1)
    order.insert(0, end)
    prim_set_xform_op_order(prim, order)
    return prim


def prim_get_num_xform_ops(prim: Usd.Prim) -> int:
    """Returns the number of xform ops on a given prim.

    Args:
        prim: The prim to count xform ops for.

    Returns:
        The number of xform ops.
    """
    return len(prim_get_xform_op_order(prim))


def prim_translate(prim: Usd.Prim, offset: tuple[float, float, float]) -> Usd.Prim:
    """Translates a prim along the (x, y, z) dimensions.

    Args:
        prim: The USD prim to translate.
        offset: The offsets for the (x, y, z) dimensions.

    Returns:
        The translated prim.
    """
    x = UsdGeom.Xformable(prim)
    x.AddTranslateOp(opSuffix=f"num_{prim_get_num_xform_ops(prim)}").Set(offset)
    prim_xform_op_move_end_to_front(prim)
    return prim


def prim_rotate_x(prim: Usd.Prim, angle: float) -> Usd.Prim:
    """Rotates a prim around the X axis.

    Args:
        prim: The USD prim to rotate.
        angle: The rotation angle in degrees.

    Returns:
        The rotated prim.
    """
    x = UsdGeom.Xformable(prim)
    x.AddRotateXOp(opSuffix=f"num_{prim_get_num_xform_ops(prim)}").Set(angle)
    prim_xform_op_move_end_to_front(prim)
    return prim


def prim_rotate_y(prim: Usd.Prim, angle: float) -> Usd.Prim:
    """Rotates a prim around the Y axis.

    Args:
        prim: The USD prim to rotate.
        angle: The rotation angle in degrees.

    Returns:
        The rotated prim.
    """
    x = UsdGeom.Xformable(prim)
    x.AddRotateYOp(opSuffix=f"num_{prim_get_num_xform_ops(prim)}").Set(angle)
    prim_xform_op_move_end_to_front(prim)
    return prim


def prim_rotate_z(prim: Usd.Prim, angle: float) -> Usd.Prim:
    """Rotates a prim around the Z axis.

    Args:
        prim: The USD prim to rotate.
        angle: The rotation angle in degrees.

    Returns:
        The rotated prim.
    """
    x = UsdGeom.Xformable(prim)
    x.AddRotateZOp(opSuffix=f"num_{prim_get_num_xform_ops(prim)}").Set(angle)
    prim_xform_op_move_end_to_front(prim)
    return prim


def _translation_to_np(t: Gf.Vec3d) -> np.ndarray:
    """Convert a Gf.Vec3d translation to a numpy array.

    Args:
        t: The translation vector to convert.

    Returns:
        Translation as numpy array.
    """
    return np.array(t)


def _rotation_to_np_quat(r: Gf.Rotation) -> np.ndarray:
    """Convert a Gf.Rotation to a numpy quaternion array.

    Args:
        r: The rotation to convert.

    Returns:
        Quaternion as numpy array [w, x, y, z] where w is the real component.
    """
    quat = r.GetQuaternion()
    real = quat.GetReal()
    imag: Gf.Vec3d = quat.GetImaginary()
    return np.array([real, imag[0], imag[1], imag[2]])


def prim_get_local_transform(prim: Usd.Prim) -> tuple[np.ndarray, np.ndarray]:
    """From: https://docs.omniverse.nvidia.com/dev-guide/latest/programmer_ref/usd/transforms/get-local-transforms.html.

    Get the local transformation of a prim using Xformable.
    See https://openusd.org/release/api/class_usd_geom_xformable.html

    Args:
        prim: The prim to calculate the local transformation.

    Returns:
        A tuple of:
        - Translation vector.
        - Rotation quaternion, i.e. 3d vector plus angle.
    """
    xform = UsdGeom.Xformable(prim)
    local_transformation: Gf.Matrix4d = xform.GetLocalTransformation()
    translation: Gf.Vec3d = local_transformation.ExtractTranslation()
    rotation: Gf.Rotation = local_transformation.ExtractRotation()
    return _translation_to_np(translation), _rotation_to_np_quat(rotation)


def prim_get_world_transform(prim: Usd.Prim) -> tuple[np.ndarray, np.ndarray]:
    """From: https://docs.omniverse.nvidia.com/dev-guide/latest/programmer_ref/usd/transforms/get-world-transforms.html.

    Get the world transformation of a prim using Xformable.
    See https://openusd.org/release/api/class_usd_geom_xformable.html

    Args:
        prim: The prim to calculate the world transformation.

    Returns:
        A tuple of:
        - Translation vector.
        - Rotation quaternion, i.e. 3d vector plus angle.
    """
    xform = UsdGeom.Xformable(prim)
    time = Usd.TimeCode.Default()  # The time at which we compute the bounding box
    world_transform: Gf.Matrix4d = xform.ComputeLocalToWorldTransform(time)
    translation: Gf.Vec3d = world_transform.ExtractTranslation()
    rotation: Gf.Rotation = world_transform.ExtractRotation()
    return _translation_to_np(translation), _rotation_to_np_quat(rotation)
