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

"""Utility functions for transforming USD prim locations, orientations, scales, and coordinate spaces."""

from __future__ import annotations

from pxr import Gf, Sdf, Usd, UsdGeom


def set_location(prim: Usd.Prim, location: Gf.Vec3d) -> None:
    """Set the location of the prim, handling translate xformOps.

    Args:
        prim: The USD prim to modify.
        location: The new location for the prim.
    """
    xformable = UsdGeom.Xformable(prim)

    # Retrieve existing translate ops and set the value to the first one, or add a new one
    translate_ops = [op for op in xformable.GetOrderedXformOps() if op.GetOpType() == UsdGeom.XformOp.TypeTranslate]
    translate_op = translate_ops[0] if translate_ops else xformable.AddTranslateOp()
    translate_op.Set(location)

    # Get the current xformOpOrder if it exists or create a new one
    op_order_attr = prim.GetAttribute("xformOpOrder")
    op_order = list(op_order_attr.Get()) if op_order_attr and op_order_attr.Get() else []

    if "xformOp:translate" not in op_order:
        # Add translate op to order if not present
        op_order.append("xformOp:translate")
        if op_order_attr:
            # Update existing xformOpOrder attribute if it exists
            op_order_attr.Set(op_order)
        else:
            # Create xformOpOrder attribute if it doesn't exist
            op_order_attr = prim.CreateAttribute("xformOpOrder", Sdf.ValueTypeNames.TokenArray)
            op_order_attr.Set(op_order)


def set_orientation(prim: Usd.Prim, orientation: Gf.Quatf | Gf.Quatd) -> None:
    """Set the orientation of the prim, handling orient xformOps.

    Args:
        prim: The USD prim to modify.
        orientation: The new orientation as a quaternion.

    Raises:
        TypeError: If orientation is not Gf.Quatf or Gf.Quatd.
    """
    xformable = UsdGeom.Xformable(prim)

    # Determine precision based on the type of orientation
    if isinstance(orientation, Gf.Quatf):
        precision = UsdGeom.XformOp.PrecisionFloat
    elif isinstance(orientation, Gf.Quatd):
        precision = UsdGeom.XformOp.PrecisionDouble
    else:
        raise TypeError("Orientation must be Gf.Quatf or Gf.Quatd")  # Raise error for invalid type

    # Get the current xformOpOrder if it exists or create a new one
    op_order_attr = prim.GetAttribute("xformOpOrder")
    op_order = list(op_order_attr.Get()) if op_order_attr and op_order_attr.Get() else []
    orient_op_name = "xformOp:orient"

    if orient_op_name in op_order:
        # Get existing orient op and its type
        orient_op = prim.GetAttribute(orient_op_name)
        existing_type = orient_op.GetTypeName()

        # Convert orientation to match existing op type
        if existing_type == Sdf.ValueTypeNames.Quatd and isinstance(orientation, Gf.Quatf):
            orientation = Gf.Quatd(orientation)
        elif existing_type == Sdf.ValueTypeNames.Quatf and isinstance(orientation, Gf.Quatd):
            orientation = Gf.Quatf(orientation)

        # Update the existing orient op if it exists in the xformOpOrder
        orient_op.Set(orientation)
    else:
        # Add a new orient op with determined precision and append it to xformOpOrder
        orient_op = xformable.AddOrientOp(precision=precision)
        orient_op.Set(orientation)
        op_order.append(orient_op_name)

    # Ensure 'xformOp:orient' is in xformOpOrder
    if "xformOp:orient" not in op_order:
        # Add orient op to order if missing
        op_order.append("xformOp:orient")
        if op_order_attr:
            # Update existing xformOpOrder
            op_order_attr.Set(op_order)
        else:
            # Create xformOpOrder attribute if it doesn't exist
            op_order_attr = prim.CreateAttribute("xformOpOrder", Sdf.ValueTypeNames.TokenArray)
            op_order_attr.Set(op_order)


def set_rotation(prim: Usd.Prim, rotation: Gf.Vec3f | Gf.Vec3d | Gf.Rotation) -> None:
    """Set the rotation of the prim, handling rotateXYZ xformOps.

    Args:
        prim: The USD prim to modify.
        rotation: The rotation as Euler angles or a Gf.Rotation object.

    Raises:
        TypeError: If rotation is not Gf.Vec3f, Gf.Vec3d, or Gf.Rotation.
    """
    xformable = UsdGeom.Xformable(prim)

    # Convert Gf.Rotation to Gf.Vec3f euler angles
    if isinstance(rotation, Gf.Rotation):
        # Decompose rotation into angles, XYZ rotation use reversed axis ZYX to decompose
        angles_deg_zyx = rotation.Decompose(Gf.Vec3d.ZAxis(), Gf.Vec3d.YAxis(), Gf.Vec3d.XAxis())
        # Reverse the order to XYZ
        rotation_vec = Gf.Vec3f(angles_deg_zyx[2], angles_deg_zyx[1], angles_deg_zyx[0])
    elif isinstance(rotation, (Gf.Vec3f, Gf.Vec3d)):
        rotation_vec = Gf.Vec3f(rotation)  # Convert to Gf.Vec3f
    else:
        raise TypeError("Rotation must be Gf.Vec3f, Gf.Vec3d, or Gf.Rotation")

    # Retrieve existing rotateXYZ op from xformOpOrder or add a new one
    op_order_attr = prim.GetAttribute("xformOpOrder")
    op_order = list(op_order_attr.Get()) if op_order_attr and op_order_attr.Get() else []
    rotate_op_name = "xformOp:rotateXYZ"

    if rotate_op_name in op_order:
        # Get existing rotateXYZ op and set the rotation value
        rotate_op = prim.GetAttribute(rotate_op_name)
        rotate_op.Set(rotation_vec)
    else:
        # Add a new rotateXYZ op with the rotation value, and append it to xformOpOrder
        rotate_op = xformable.AddRotateXYZOp()
        rotate_op.Set(rotation_vec)
        op_order.append(rotate_op_name)

    # Ensure 'xformOp:rotateXYZ' is in xformOpOrder
    if "xformOp:rotateXYZ" not in op_order:
        # Add rotateXYZ op to order if missing
        op_order.append("xformOp:rotateXYZ")
        if op_order_attr:
            # Update existing xformOpOrder
            op_order_attr.Set(op_order)
        else:
            # Create xformOpOrder attribute if it doesn't exist
            op_order_attr = prim.CreateAttribute("xformOpOrder", Sdf.ValueTypeNames.TokenArray)
            op_order_attr.Set(op_order)


def set_scale(prim: Usd.Prim, scale: Gf.Vec3f) -> None:
    """Set the scale of the prim, handling scale xformOps.

    Args:
        prim: The USD prim to modify.
        scale: The scale values for each axis.
    """
    xformable = UsdGeom.Xformable(prim)

    # Retrieve existing scale ops and set the value to the first one, or add a new one
    scale_ops = [op for op in xformable.GetOrderedXformOps() if op.GetOpType() == UsdGeom.XformOp.TypeScale]
    scale_op = scale_ops[0] if scale_ops else xformable.AddScaleOp()
    scale_op.Set(scale)

    # Update xformOpOrder
    op_order_attr = prim.GetAttribute("xformOpOrder")
    op_order = list(op_order_attr.Get()) if op_order_attr and op_order_attr.Get() else []

    # Ensure 'xformOp:scale' is in xformOpOrder
    if "xformOp:scale" not in op_order:
        # Add scale op to order if not present
        op_order.append("xformOp:scale")
        if op_order_attr:
            # Update existing xformOpOrder
            op_order_attr.Set(op_order)
        else:
            # Create xformOpOrder attribute if it doesn't exist
            op_order_attr = prim.CreateAttribute("xformOpOrder", Sdf.ValueTypeNames.TokenArray)
            op_order_attr.Set(op_order)


def set_transform_matrix(prim: Usd.Prim, transform: Gf.Matrix4d) -> None:
    """Set the transformation matrix of the prim using a transform op.

    Args:
        prim: The USD prim to modify.
        transform: The 4x4 transformation matrix to apply.
    """
    xformable = UsdGeom.Xformable(prim)

    # Retrieve existing transform ops and set the value to the first one, or add a new one
    transform_ops = [op for op in xformable.GetOrderedXformOps() if op.GetOpType() == UsdGeom.XformOp.TypeTransform]
    transform_op = transform_ops[0] if transform_ops else xformable.AddTransformOp()
    transform_op.Set(transform)  # Set the transformation matrix

    # Update xformOpOrder
    op_order_attr = prim.GetAttribute("xformOpOrder")
    op_order = list(op_order_attr.Get()) if op_order_attr and op_order_attr.Get() else []

    # Ensure 'xformOp:transform' is in xformOpOrder
    if "xformOp:transform" not in op_order:
        # Add transform op to order if not present
        op_order.append("xformOp:transform")
        if op_order_attr:
            # Update existing xformOpOrder
            op_order_attr.Set(op_order)
        else:
            # Create xformOpOrder attribute if it doesn't exist
            op_order_attr = prim.CreateAttribute("xformOpOrder", Sdf.ValueTypeNames.TokenArray)
            op_order_attr.Set(op_order)


def set_transform_attributes(
    prim: Usd.Prim,
    location: Gf.Vec3d | None = None,
    orientation: Gf.Quatf | Gf.Quatd | None = None,
    rotation: Gf.Vec3f | Gf.Vec3d | Gf.Rotation | None = None,
    scale: Gf.Vec3f | None = None,
    transform: Gf.Matrix4d | None = None,
) -> None:
    """Set or update the transform attributes of a prim.

    Args:
        prim: The USD prim to modify.
        location: The location to set.
        orientation: The orientation quaternion to set.
        rotation: The rotation to set as Euler angles or Gf.Rotation.
        scale: The scale values to set.
        transform: The transformation matrix to set.
    """
    if transform is not None:
        set_transform_matrix(prim, transform)

    if location is not None:
        set_location(prim, location)

    if orientation is not None:
        set_orientation(prim, orientation)

    if rotation is not None:
        set_rotation(prim, rotation)

    if scale is not None:
        set_scale(prim, scale)


def get_prim_world_pose(prim: Usd.Prim) -> tuple[Gf.Vec3d, Gf.Quatd] | None:
    """Return the world pose of a USD prim as (Gf.Vec3d, Gf.Quatd).

    Args:
        prim: The USD prim to get the world pose from.

    Returns:
        A tuple containing the world position and orientation as (location, orientation).
    """
    xform_cache = UsdGeom.XformCache()
    world_matrix = xform_cache.GetLocalToWorldTransform(prim).RemoveScaleShear()
    location = world_matrix.ExtractTranslation()
    orientation = world_matrix.ExtractRotationQuat()
    return location, orientation


def transform_local_poses_to_world(
    poses: list[tuple[Gf.Vec3d, Gf.Quatd]], prim: Usd.Prim
) -> list[tuple[Gf.Vec3d, Gf.Quatd]]:
    """Transform a list of local grasp poses to world poses using the prim's transform.

    Args:
        poses: List of local poses as (location, orientation) tuples.
        prim: The USD prim providing the transformation reference.

    Returns:
        List of world poses as (location, orientation) tuples.
    """
    xform_cache = UsdGeom.XformCache()
    transform_matrix = xform_cache.GetLocalToWorldTransform(prim).RemoveScaleShear()
    world_rotation = transform_matrix.ExtractRotationQuat()

    world_poses = []
    for loc, quat in poses:
        world_location = transform_matrix.Transform(loc)
        world_orientation = world_rotation * quat
        world_poses.append((world_location, world_orientation))
    return world_poses


def transform_local_pose_to_world(pose: tuple[Gf.Vec3d, Gf.Quatd], prim: Usd.Prim) -> tuple[Gf.Vec3d, Gf.Quatd]:
    """Transform a local pose to world pose using the prim's transform.

    Args:
        pose: The local pose as (location, orientation) tuple.
        prim: The USD prim providing the transformation reference.

    Returns:
        The world pose as (location, orientation) tuple.
    """
    xform_cache = UsdGeom.XformCache()
    transform_matrix = xform_cache.GetLocalToWorldTransform(prim).RemoveScaleShear()
    world_rotation = transform_matrix.ExtractRotationQuat()
    world_location = transform_matrix.Transform(pose[0])
    world_orientation = world_rotation * pose[1]
    return world_location, world_orientation
