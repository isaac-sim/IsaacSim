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

"""Deprecated xform operation utility functions."""

import numpy as np
import usdrt
from isaacsim.core.utils.prims import (
    get_prim_at_path,
    get_prim_parent,
    get_prim_path,
    is_prim_path_valid,
)
from pxr import Gf, Usd, UsdGeom
from scipy.spatial.transform import Rotation


def clear_xform_ops(prim: Usd.Prim) -> None:
    """Remove all xform ops from input prim.

    Args:
        prim: The input USD prim.
    """
    xformable = UsdGeom.Xformable(prim)
    xformable.ClearXformOpOrder()
    # Remove any authored transform properties
    authored_prop_names = prim.GetAuthoredPropertyNames()
    for prop_name in authored_prop_names:
        if prop_name.startswith("xformOp:"):
            prim.RemoveProperty(prop_name)


def reset_and_set_xform_ops(
    prim: Usd.Prim, translation: Gf.Vec3d, orientation: Gf.Quatd, scale: Gf.Vec3d = Gf.Vec3d([1.0, 1.0, 1.0])
) -> None:
    """Reset xform ops to isaac sim defaults, and set their values.

    Args:
        prim: Prim to reset
        translation: translation to set
        orientation: orientation to set
        scale: scale to set
    """
    xformable = UsdGeom.Xformable(prim)
    clear_xform_ops(prim)

    xform_op_scale = xformable.AddXformOp(UsdGeom.XformOp.TypeScale, UsdGeom.XformOp.PrecisionDouble, "")
    xform_op_scale.Set(scale)

    xform_op_tranlsate = xformable.AddXformOp(UsdGeom.XformOp.TypeTranslate, UsdGeom.XformOp.PrecisionDouble, "")
    xform_op_tranlsate.Set(translation)

    xform_op_rot = xformable.AddXformOp(UsdGeom.XformOp.TypeOrient, UsdGeom.XformOp.PrecisionDouble, "")
    xform_op_rot.Set(orientation)

    xformable.SetXformOpOrder([xform_op_tranlsate, xform_op_rot, xform_op_scale])


def reset_xform_ops(prim: Usd.Prim) -> None:
    """Reset xform ops for a prim to isaac sim defaults.

    Args:
        prim: Prim to reset xform ops on
    """
    xformable = UsdGeom.Xformable(prim)
    # get current position and orientation
    T_p_w = xformable.ComputeParentToWorldTransform(Usd.TimeCode.Default())
    T_l_w = xformable.ComputeLocalToWorldTransform(Usd.TimeCode.Default())
    T_l_p = Gf.Transform()
    T_l_p.SetMatrix(Gf.Matrix4d(np.matmul(T_l_w, np.linalg.inv(T_p_w)).tolist()))
    current_translation = T_l_p.GetTranslation()
    current_orientation = T_l_p.GetRotation().GetQuat()

    reset_and_set_xform_ops(prim, current_translation, current_orientation)


def _get_world_pose_transform_w_scale(prim_path: str, fabric: bool = False) -> usdrt.Gf.Matrix4d:
    """Get the world transformation matrix including scale for a prim.

    Args:
        prim_path: Path to the prim.
        fabric: Whether to use fabric API for retrieving the transformation.

    Raises:
        Exception: If the prim path is not valid.

    Returns:
        World transformation matrix including scale for the prim.
    """
    # This will return a transformation matrix with translation as the last row and scale included
    if not is_prim_path_valid(prim_path, fabric=False):
        raise Exception("Prim path is not valid")

    def _get_from_usd_prim(prim_path: str) -> usdrt.Gf.Matrix4d:
        chain = []
        current_path = prim_path
        while current_path:
            usd_prim = get_prim_at_path(prim_path=current_path, fabric=False)
            local_transform = usdrt.Gf.Matrix4d(
                UsdGeom.Xformable(usd_prim).GetLocalTransformation(Usd.TimeCode.Default())
            )
            chain.append(local_transform)
            parent_prim = get_prim_parent(usd_prim)
            if parent_prim:
                current_path = get_prim_path(parent_prim)
            else:
                break
        result = usdrt.Gf.Matrix4d(1.0)
        for transform in reversed(chain):
            result = transform * result
        return result

    if not fabric:
        return _get_from_usd_prim(prim_path)

    fabric_prim = get_prim_at_path(prim_path=prim_path, fabric=True)
    xformable_prim = usdrt.Rt.Xformable(fabric_prim)
    if xformable_prim.HasWorldXform():
        world_pos_attr = xformable_prim.GetWorldPositionAttr()
        if not world_pos_attr.IsValid():
            world_pos = usdrt.Gf.Vec3d(0)
        else:
            world_pos = world_pos_attr.Get(usdrt.Usd.TimeCode.Default())
        world_orientation_attr = xformable_prim.GetWorldOrientationAttr()
        if not world_orientation_attr.IsValid():
            world_orientation = usdrt.Gf.Quatf(1)
        else:
            world_orientation = world_orientation_attr.Get(usdrt.Usd.TimeCode.Default())
        world_scale_attr = xformable_prim.GetWorldScaleAttr()
        if not world_scale_attr.IsValid():
            world_scale = usdrt.Gf.Vec3d(1)
        else:
            world_scale = world_scale_attr.Get(usdrt.Usd.TimeCode.Default())
        scale = usdrt.Gf.Matrix4d()
        rot = usdrt.Gf.Matrix4d()
        scale.SetScale(usdrt.Gf.Vec3d(world_scale))
        rot.SetRotate(usdrt.Gf.Quatd(world_orientation))
        result = scale * rot
        result.SetTranslateOnly(world_pos)
        return result
    elif xformable_prim.HasLocalXform():
        local_transform = xformable_prim.GetLocalMatrixAttr().Get(usdrt.Usd.TimeCode.Default())
        parent_prim = get_prim_parent(get_prim_at_path(prim_path=prim_path, fabric=False))
        parent_world_transform = usdrt.Gf.Matrix4d(1.0)
        if parent_prim:
            parent_world_transform = _get_world_pose_transform_w_scale(get_prim_path(parent_prim))
        return local_transform * parent_world_transform
    else:
        return _get_from_usd_prim(prim_path)


def get_local_pose(prim_path: str) -> tuple:
    """Get the local pose of a prim relative to its parent.

    Args:
        prim_path: Path to the prim.

    Returns:
        A tuple containing the local translation (as numpy array) and orientation (as quaternion in w,x,y,z format).

    Raises:
        Exception: If the prim path is not valid.
    """
    if not is_prim_path_valid(prim_path, fabric=False):
        raise Exception("Prim path is not valid")
    fabric_prim = get_prim_at_path(prim_path=prim_path, fabric=True)
    xformable_prim = usdrt.Rt.Xformable(fabric_prim)
    if xformable_prim.HasWorldXform():
        world_pos = xformable_prim.GetWorldPositionAttr().Get(usdrt.Usd.TimeCode.Default())
        world_orientation = xformable_prim.GetWorldOrientationAttr().Get(usdrt.Usd.TimeCode.Default())
        prim_l_w_transform = usdrt.Gf.Matrix4d()
        prim_l_w_transform.SetRotate(usdrt.Gf.Quatd(world_orientation))
        prim_l_w_transform.SetTranslateOnly(world_pos)
        parent_prim = get_prim_parent(get_prim_at_path(prim_path=prim_path, fabric=False))
        parent_l_w_transform = usdrt.Gf.Matrix4d(1.0)
        if parent_prim:
            parent_l_w_transform = _get_world_pose_transform_w_scale(get_prim_path(parent_prim))
            parent_l_w_transform.Orthonormalize()
            parent_l_w_transform = np.array(parent_l_w_transform)
        result_transform = np.matmul(prim_l_w_transform, np.linalg.inv(parent_l_w_transform))
        result_transform = np.transpose(result_transform)
        r = Rotation.from_matrix(result_transform[:3, :3])
        return result_transform[:3, 3], r.as_quat()[[3, 0, 1, 2]]

    elif xformable_prim.HasLocalXform():
        local_transform = xformable_prim.GetLocalMatrixAttr().Get(usdrt.Usd.TimeCode.Default())
        local_transform.Orthonormalize()
        return (
            np.array(local_transform.ExtractTranslation()),
            np.array(local_transform.ExtractRotationQuat())[[3, 0, 1, 2]],
        )
    else:
        usd_prim = get_prim_at_path(prim_path=prim_path, fabric=False)
        local_transform = usdrt.Gf.Matrix4d(UsdGeom.Xformable(usd_prim).GetLocalTransformation(Usd.TimeCode.Default()))
        local_transform.Orthonormalize()
        return (
            np.array(local_transform.ExtractTranslation()),
            np.array(local_transform.ExtractRotationQuat())[[3, 0, 1, 2]],
        )


def get_world_pose(prim_path: str, fabric: bool = False) -> tuple:
    """Get the world pose of a prim.

    Args:
        prim_path: Path to the prim.
        fabric: Whether to use fabric API for retrieving the pose.

    Returns:
        A tuple containing the world translation (as numpy array) and orientation (as quaternion in w,x,y,z format).
    """
    result_transform = _get_world_pose_transform_w_scale(prim_path, fabric)
    result_transform.Orthonormalize()
    result_transform = np.transpose(result_transform)
    r = Rotation.from_matrix(result_transform[:3, :3])
    return result_transform[:3, 3], r.as_quat()[[3, 0, 1, 2]]
