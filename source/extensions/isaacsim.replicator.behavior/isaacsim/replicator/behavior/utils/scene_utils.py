# SPDX-FileCopyrightText: Copyright (c) 2024-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

"""Utility functions for USD scene manipulation used by behavior scripts."""

from __future__ import annotations

import carb
import carb.settings
import omni.kit.app
import omni.kit.commands
import omni.physics.core
import omni.physx
import omni.usd
from pxr import Gf, PhysxSchema, Sdf, Usd, UsdGeom, UsdPhysics, UsdShade


def get_world_location(prim: Usd.Prim, xform_cache: UsdGeom.XformCache | None = None) -> Gf.Vec3d:
    """Get the absolute location of the prim.

    Args:
        prim: The USD prim to get the location of.
        xform_cache: Optional transform cache for performance.

    Returns:
        The world-space location of the prim.
    """
    if xform_cache is None:
        xform_cache = UsdGeom.XformCache()
    return xform_cache.GetLocalToWorldTransform(prim).ExtractTranslation()


def get_world_rotation(prim: Usd.Prim, xform_cache: UsdGeom.XformCache | None = None) -> Gf.Rotation:
    """Get the absolute rotation of the prim.

    Args:
        prim: The USD prim to get the rotation of.
        xform_cache: Optional transform cache for performance.

    Returns:
        The world-space rotation of the prim.
    """
    if xform_cache is None:
        xform_cache = UsdGeom.XformCache()
    # Wrap the matrix in a Gf.Transform to avoid any scale or shear issues
    return Gf.Transform(xform_cache.GetLocalToWorldTransform(prim)).GetRotation()


def get_rotation_op_and_value(
    prim: Usd.Prim,
) -> tuple[str, float | Gf.Quatf | Gf.Quatd | Gf.Rotation | Gf.Vec3d | Gf.Vec3f]:
    """Get the rotation of the prim, creating an xformOp:orient if it doesn't exist.

    Args:
        prim: The USD prim to get the rotation from.

    Returns:
        A tuple of the rotation op name and the rotation value.
    """
    xformable = UsdGeom.Xformable(prim)
    xform_ops = xformable.GetOrderedXformOps()

    for op in xform_ops:
        rotation_op = op.GetOpName()
        if rotation_op.startswith("xformOp:rotate") or rotation_op == "xformOp:orient":
            rotation_val = op.Get()
            return (rotation_op, rotation_val)
        elif rotation_op == "xformOp:transform":
            transform_matrix = op.Get()
            transform = Gf.Transform(transform_matrix)
            rotation_val = transform.GetRotation()
            return (rotation_op, rotation_val)

    orient_op = xformable.AddXformOp(UsdGeom.XformOp.TypeOrient, UsdGeom.XformOp.PrecisionDouble)
    identity_quat = Gf.Quatd(1.0, 0.0, 0.0, 0.0)
    orient_op.Set(identity_quat)
    return ("xformOp:orient", identity_quat)


def set_rotation_op_and_value(
    prim: Usd.Prim,
    rotation_op: str,
    rotation_val: float | Gf.Rotation | Gf.Quatf | Gf.Quatd | Gf.Vec3d | Gf.Vec3f,
) -> None:
    """Set the rotation of the prim, the ops should already exist.

    Args:
        prim: The USD prim to set the rotation on.
        rotation_op: The name of the rotation xformOp.
        rotation_val: The rotation value to set.
    """
    xformable = UsdGeom.Xformable(prim)
    xform_ops = xformable.GetOrderedXformOps()

    for op in xform_ops:
        if op.GetOpName() == rotation_op:
            if rotation_op == "xformOp:transform":
                transform_matrix = op.Get()
                transform = Gf.Transform(transform_matrix)
                transform.SetRotation(rotation_val)
                op.Set(transform.GetMatrix())
            else:
                op.Set(rotation_val)
            return


def set_rotation_with_ops(prim: Usd.Prim, rotation: Gf.Rotation) -> None:
    """Set the rotation using the first valid op from orient, rotate, transform, if none found create new orient op.

    Args:
        prim: The USD prim to set the rotation on.
        rotation: The rotation to apply.
    """
    xformable = UsdGeom.Xformable(prim)
    xform_ops = xformable.GetOrderedXformOps()

    for op in xform_ops:
        op_name = op.GetOpName()

        if op_name == "xformOp:orient":
            op_type = op.GetTypeName()
            if op_type == Sdf.ValueTypeNames.Quatf:
                quat = Gf.Quatf(rotation.GetQuat())
            elif op_type == Sdf.ValueTypeNames.Quatd:
                quat = Gf.Quatd(rotation.GetQuat())
            op.Set(quat)
            return

        elif op_name.startswith("xformOp:rotate"):
            rotation_order = op_name[len("xformOp:rotate") :]
            try:
                rotation_decomp = decompose_rotation(rotation, rotation_order)
            except ValueError as error:
                carb.log_warn(f"[{prim.GetPath()}] {error}")
                return
            op.Set(rotation_decomp)
            return

        elif op_name == "xformOp:transform":
            transform = Gf.Transform(op.Get())
            transform.SetRotation(rotation)
            op.Set(transform.GetMatrix())
            return

    # Create a new orient op if none found
    orient_op = xformable.AddXformOp(UsdGeom.XformOp.TypeOrient, UsdGeom.XformOp.PrecisionDouble)
    orient_op.Set(Gf.Quatd(rotation.GetQuat()))


def decompose_rotation(rotation: Gf.Rotation, rotation_order: str) -> float | Gf.Vec3f:
    """Helper function to decompose Gf.Rotation based on rotation order.

    Args:
        rotation: The rotation to decompose.
        rotation_order: The rotation order string, such as "XYZ", "ZYX", "X", "Y", or "Z".

    Returns:
        The decomposed rotation as euler angles or a single-axis angle.

    Raises:
        ValueError: If the rotation order is not supported.
    """
    if rotation_order in ("XYZ", "X", "Y", "Z"):
        rotation_decomp = rotation.Decompose(Gf.Vec3d.ZAxis(), Gf.Vec3d.YAxis(), Gf.Vec3d.XAxis())
        rotation_vec = Gf.Vec3f(rotation_decomp[2], rotation_decomp[1], rotation_decomp[0])
        if rotation_order == "X":
            return rotation_vec[0]
        if rotation_order == "Y":
            return rotation_vec[1]
        if rotation_order == "Z":
            return rotation_vec[2]
        return rotation_vec
    elif rotation_order == "XZY":
        rotation_decomp = rotation.Decompose(Gf.Vec3d.YAxis(), Gf.Vec3d.ZAxis(), Gf.Vec3d.XAxis())
        return Gf.Vec3f(rotation_decomp[2], rotation_decomp[0], rotation_decomp[1])
    elif rotation_order == "YXZ":
        rotation_decomp = rotation.Decompose(Gf.Vec3d.ZAxis(), Gf.Vec3d.XAxis(), Gf.Vec3d.YAxis())
        return Gf.Vec3f(rotation_decomp[1], rotation_decomp[2], rotation_decomp[0])
    elif rotation_order == "YZX":
        rotation_decomp = rotation.Decompose(Gf.Vec3d.XAxis(), Gf.Vec3d.ZAxis(), Gf.Vec3d.YAxis())
        return Gf.Vec3f(rotation_decomp[0], rotation_decomp[2], rotation_decomp[1])
    elif rotation_order == "ZXY":
        rotation_decomp = rotation.Decompose(Gf.Vec3d.YAxis(), Gf.Vec3d.XAxis(), Gf.Vec3d.ZAxis())
        return Gf.Vec3f(rotation_decomp[1], rotation_decomp[0], rotation_decomp[2])
    elif rotation_order == "ZYX":
        rotation_decomp = rotation.Decompose(Gf.Vec3d.XAxis(), Gf.Vec3d.YAxis(), Gf.Vec3d.ZAxis())
        return Gf.Vec3f(rotation_decomp[0], rotation_decomp[1], rotation_decomp[2])
    raise ValueError(f"Unsupported rotation_order: {rotation_order!r}")


def calculate_look_at_rotation(
    eye: Gf.Vec3d, target_location: Gf.Vec3d, up_axis: Gf.Vec3d, epsilon: float = 1e-5
) -> Gf.Rotation:
    """Calculate a look-at rotation matrix from 'eye' to 'target_location' and adjusts for collinearity if needed.

    Args:
        eye: The position of the observer.
        target_location: The position to look at.
        up_axis: The up direction vector.
        epsilon: Small threshold for collinearity detection.

    Returns:
        The look-at rotation.
    """
    # Check for collinearity between direction and up axis, and adjust if needed to avoid undefined rotations.
    direction = target_location - eye
    cross_product = direction.GetCross(up_axis)

    # Adjust the eye position slightly if direction and up axis are collinear
    if cross_product.GetLength() < epsilon:
        if abs(up_axis[0]) < epsilon:
            orthogonal_direction = Gf.Vec3d(1, 0, 0)
        elif abs(up_axis[1]) < epsilon:
            orthogonal_direction = Gf.Vec3d(0, 1, 0)
        else:
            orthogonal_direction = Gf.Vec3d(0, 0, 1)
        eye = eye + orthogonal_direction * epsilon

    # Create a look-at transformation matrix from 'eye' to 'target_location' with specified 'up' direction
    # and invert it to orient the prim to the target
    look_at_matrix = Gf.Matrix4d().SetLookAt(eye, target_location, up_axis)
    look_at_matrix = look_at_matrix.GetInverse()

    # Extract and return the rotation part of the matrix
    return look_at_matrix.ExtractRotation()


def set_location(prim: Usd.Prim, location: Gf.Vec3d) -> None:
    """Set the location of the prim, handling translate xformOps.

    Args:
        prim: The USD prim to set the location on.
        location: The location to set.
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
        prim: The USD prim to set the orientation on.
        orientation: The orientation quaternion to set.
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
        prim: The USD prim to set the rotation on.
        rotation: The rotation to set.
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
        prim: The USD prim to set the scale on.
        scale: The scale to set.
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
        prim: The USD prim to set the transform on.
        transform: The transformation matrix to set.
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
        prim: The USD prim to set the transform attributes on.
        location: Optional location to set.
        orientation: Optional orientation quaternion to set.
        rotation: Optional rotation to set.
        scale: Optional scale to set.
        transform: Optional full transformation matrix to set.
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


def create_asset(stage: Usd.Stage, asset_url: str, prim_path: str) -> Usd.Prim:
    """Create a new Xform prim with the provided asset URL reference at path with or without colliders.

    Args:
        stage: The USD stage to create the prim on.
        asset_url: The URL of the asset to reference.
        prim_path: The path for the new prim.

    Returns:
        The created USD prim.
    """
    prim = stage.DefinePrim(prim_path, "Xform")
    prim.GetReferences().AddReference(asset_url)
    return prim


def create_physics_material(
    stage: Usd.Stage, prim_path: str, restitution: float, static_friction: float, dynamic_friction: float
) -> UsdShade.Material:
    """Create a new physics material with the specified friction and restitution properties.

    Args:
        stage: The USD stage to create the material on.
        prim_path: The path for the new material prim.
        restitution: The restitution coefficient.
        static_friction: The static friction coefficient.
        dynamic_friction: The dynamic friction coefficient.

    Returns:
        The created USD material.
    """
    material = UsdShade.Material.Define(stage, prim_path)
    physics_material = UsdPhysics.MaterialAPI.Apply(material.GetPrim())
    physics_material.CreateRestitutionAttr().Set(restitution)
    physics_material.CreateStaticFrictionAttr().Set(static_friction)
    physics_material.CreateDynamicFrictionAttr().Set(dynamic_friction)
    return material


def create_mdl_material(mtl_url: str, mtl_name: str, mtl_path: str) -> UsdShade.Material:
    """Create an MDL material at the given path with the specified URL and name.

    Args:
        mtl_url: The URL of the MDL material.
        mtl_name: The name of the material.
        mtl_path: The path for the material prim.

    Returns:
        The created USD material.
    """
    omni.kit.commands.execute("CreateMdlMaterialPrim", mtl_url=mtl_url, mtl_name=mtl_name, mtl_path=mtl_path)
    stage = omni.usd.get_context().get_stage()
    mtl_prim = stage.GetPrimAtPath(mtl_path)
    shader = UsdShade.Shader(omni.usd.get_shader_from_material(mtl_prim, get_prim=True))

    # Create various material inputs
    shader.CreateInput("diffuse_texture", Sdf.ValueTypeNames.Asset)
    shader.CreateInput("project_uvw", Sdf.ValueTypeNames.Bool)
    shader.CreateInput("texture_scale", Sdf.ValueTypeNames.Float2)
    shader.CreateInput("texture_rotate", Sdf.ValueTypeNames.Float)

    return UsdShade.Material(mtl_prim)


def add_colliders(
    prim: Usd.Prim, approximation_type: str = "convexHull", physics_scene: UsdPhysics.Scene | None = None
) -> None:
    """Add colliders to the prim and its descendants (without rigid body dynamics the asset will be static).

    Args:
        prim: The USD prim to add colliders to.
        approximation_type: The collision approximation type.
        physics_scene: Optional physics scene to associate colliders with.
    """
    for desc_prim in Usd.PrimRange(prim):
        # Add collision properties to all geometry types
        if desc_prim.IsA(UsdGeom.Gprim):
            if not desc_prim.HasAPI(UsdPhysics.CollisionAPI):
                collision_api = UsdPhysics.CollisionAPI.Apply(desc_prim)
            else:
                collision_api = UsdPhysics.CollisionAPI(desc_prim)
            collision_api.CreateCollisionEnabledAttr(True)

            # Add collider to specific physics scene if provided
            if physics_scene:
                collision_api.GetSimulationOwnerRel().AddTarget(physics_scene.GetPath())

        # Add mesh collision properties to mesh geometry
        if desc_prim.IsA(UsdGeom.Mesh):
            if not desc_prim.HasAPI(UsdPhysics.MeshCollisionAPI):
                mesh_collision_api = UsdPhysics.MeshCollisionAPI.Apply(desc_prim)
            else:
                mesh_collision_api = UsdPhysics.MeshCollisionAPI(desc_prim)
            mesh_collision_api.CreateApproximationAttr().Set(approximation_type)


def disable_colliders(prim: Usd.Prim, include_descendants: bool = True) -> None:
    """Disable the colliders of the prim and its descendants.

    Args:
        prim: The USD prim to disable colliders on.
        include_descendants: Whether to include descendant prims.
    """
    if include_descendants:
        prims = Usd.PrimRange(prim)
    else:
        prims = [prim]
    for p in prims:
        if p.HasAPI(UsdPhysics.CollisionAPI):
            collision_api = UsdPhysics.CollisionAPI(p)
            collision_api.CreateCollisionEnabledAttr(False)


def add_rigid_body_dynamics(
    prim: Usd.Prim,
    physics_scene: UsdPhysics.Scene | None = None,
    disable_gravity: bool = False,
    angular_damping: float | None = None,
    linear_damping: float | None = None,
) -> None:
    """Add rigid body dynamics properties to the prim if it has colliders.

    Args:
        prim: The USD prim to add rigid body dynamics to.
        physics_scene: Optional physics scene to associate the rigid body with.
        disable_gravity: Whether to disable gravity for this body.
        angular_damping: Optional angular damping coefficient.
        linear_damping: Optional linear damping coefficient.
    """
    # Physics
    if not prim.HasAPI(UsdPhysics.RigidBodyAPI):
        rigid_body_api = UsdPhysics.RigidBodyAPI.Apply(prim)
    else:
        rigid_body_api = UsdPhysics.RigidBodyAPI(prim)
    rigid_body_api.CreateRigidBodyEnabledAttr(True)
    # Add collider to specific physics scene if provided
    if physics_scene:
        rigid_body_api.GetSimulationOwnerRel().AddTarget(physics_scene.GetPath())
    # PhysX
    if not prim.HasAPI(PhysxSchema.PhysxRigidBodyAPI):
        physx_rigid_body_api = PhysxSchema.PhysxRigidBodyAPI.Apply(prim)
    else:
        physx_rigid_body_api = PhysxSchema.PhysxRigidBodyAPI(prim)
    physx_rigid_body_api.GetDisableGravityAttr().Set(disable_gravity)
    if angular_damping is not None:
        physx_rigid_body_api.CreateAngularDampingAttr().Set(angular_damping)
    if linear_damping is not None:
        physx_rigid_body_api.CreateLinearDampingAttr().Set(linear_damping)


def disable_rigid_body_dynamics(prim: Usd.Prim, include_descendants: bool = True) -> None:
    """Disable the rigid body dynamics properties of the prim and its descendants.

    Args:
        prim: The USD prim to disable rigid body dynamics on.
        include_descendants: Whether to include descendant prims.
    """
    if include_descendants:
        prims = Usd.PrimRange(prim)
    else:
        prims = [prim]
    for p in prims:
        if p.HasAPI(UsdPhysics.RigidBodyAPI):
            rigid_body_api = UsdPhysics.RigidBodyAPI(p)
            rigid_body_api.CreateRigidBodyEnabledAttr(False)


def create_collision_walls(
    stage: Usd.Stage,
    prim: Usd.Prim,
    prim_path: str,
    height: float,
    thickness: float = 0.4,
    physics_scene: UsdPhysics.Scene | None = None,
    physics_material: UsdShade.Material | None = None,
    bbox_cache: UsdGeom.BBoxCache | None = None,
    visible: bool = False,
) -> list[Usd.Prim]:
    """Create collision walls around the top surface of the prim with the given height and thickness.

    Args:
        stage: The USD stage to create walls on.
        prim: The USD prim to create walls around.
        prim_path: The base path for the wall prims.
        height: The height of the walls.
        thickness: The thickness of the walls.
        physics_scene: Optional physics scene to associate walls with.
        physics_material: Optional physics material for the walls.
        bbox_cache: Optional bounding box cache for performance.
        visible: Whether the walls should be visible.

    Returns:
        List of created wall prims.
    """
    # Init BBoxCache and XformCache if not provided
    if bbox_cache is None:
        bbox_cache = UsdGeom.BBoxCache(Usd.TimeCode.Default(), includedPurposes=[UsdGeom.Tokens.default_])

    # Get prim world transform, scale, and untransformed bounding box size
    world_bound = bbox_cache.ComputeWorldBound(prim)
    world_tf_mat = world_bound.GetMatrix()
    world_scale = Gf.Transform(world_tf_mat).GetScale()

    # Compute the world-space scaled bounding box of the prim (GetRange() returns untransformed size)
    bbox_range_untransformed = world_bound.GetRange()
    bbox_width, bbox_depth, bbox_height = bbox_range_untransformed.GetSize()
    bbox_width *= world_scale[0]
    bbox_depth *= world_scale[1]
    bbox_height *= world_scale[2]

    # Use the specified height, thickness and the prim bounding box size to calculate the wall sizes
    floor_ceiling_size = (bbox_width, bbox_depth, thickness)
    side_wall_size = (thickness, bbox_depth, height)
    front_back_wall_size = (bbox_width, thickness, height)

    # Compute the top center of the prim bounding box (scaled midpoint + half height offset)
    mid_point = bbox_range_untransformed.GetMidpoint()
    mid_point[0] *= world_scale[0]
    mid_point[1] *= world_scale[1]
    mid_point[2] *= world_scale[2]
    top_center = mid_point + Gf.Vec3d(0, 0, bbox_height / 2.0)

    # Pre-compute offsets for positioning the walls
    half_thickness = thickness / 2.0  # Offset for floor and ceiling along the z-axis
    wall_center_z = top_center[2] + (height / 2.0)  # Center z-coordinate for side walls
    half_width_thickness = (bbox_width + thickness) / 2.0  # Offset for side walls along the x-axis
    half_depth_thickness = (bbox_depth + thickness) / 2.0  # Offset for front/back walls along the y-axis

    # Define the walls (name, location, size) with the specified thickness added externally to the surface and height
    walls = [
        ("floor", (top_center[0], top_center[1], top_center[2] - half_thickness), floor_ceiling_size),
        ("ceiling", (top_center[0], top_center[1], top_center[2] + height + half_thickness), floor_ceiling_size),
        ("left_wall", (top_center[0] - half_width_thickness, top_center[1], wall_center_z), side_wall_size),
        ("right_wall", (top_center[0] + half_width_thickness, top_center[1], wall_center_z), side_wall_size),
        ("front_wall", (top_center[0], top_center[1] + half_depth_thickness, wall_center_z), front_back_wall_size),
        ("back_wall", (top_center[0], top_center[1] - half_depth_thickness, wall_center_z), front_back_wall_size),
    ]

    # Create a root prim for the walls to avoid inheriting parent transformations
    walls_root_path = omni.usd.get_stage_next_free_path(stage, f"{prim_path}/CollisionWalls", False)
    walls_root_prim = stage.DefinePrim(walls_root_path, "Xform")

    # Apply the world location and rotation transform (remove scale, already applied) to the walls root prim
    UsdGeom.Xformable(walls_root_prim).AddTransformOp().Set(world_tf_mat.RemoveScaleShear())

    # Create wall prims as children of the walls root prim xform
    collision_walls = []
    for wall_name, position, size in walls:
        wall_prim = stage.DefinePrim(f"{walls_root_prim.GetPath()}/{wall_name}", "Cube")
        # Scale by half the size since Cube has a default size of 2 units
        wall_scale = (size[0] / 2.0, size[1] / 2.0, size[2] / 2.0)
        set_transform_attributes(wall_prim, location=position, scale=wall_scale)
        add_colliders(wall_prim, physics_scene=physics_scene)
        if not visible:
            UsdGeom.Imageable(wall_prim).MakeInvisible()
        if physics_material is not None:
            mat_binding_api = UsdShade.MaterialBindingAPI.Apply(wall_prim)
            mat_binding_api.Bind(physics_material, UsdShade.Tokens.weakerThanDescendants, "physics")
        collision_walls.append(wall_prim)

    return collision_walls


async def run_simulation_async(sim_steps: int, physx_dt: float, render: bool = True) -> None:
    """Run the simulation for the specified number of steps. Optionally render the simulation by advancing the app.

    Args:
        sim_steps: Number of simulation steps to run.
        physx_dt: Physics time step duration.
        render: Whether to render each simulation step.
    """
    physx_sim_interface = omni.physx.get_physx_simulation_interface()
    for _ in range(sim_steps):
        physx_sim_interface.simulate(physx_dt, 0)
        physx_sim_interface.fetch_results()
        if render:
            await omni.kit.app.get_app().next_update_async()


async def apply_forces_and_simulate_async(
    stage_id: int,
    body_ids: list[int],
    forces: list[tuple[float, float, float]],
    positions: list[tuple[float, float, float]],
    sim_steps: int,
    physx_dt: float,
    render: bool = True,
) -> None:
    """Apply forces to assets at the specified positions, and simulate for the given number of steps.

    Args:
        stage_id: The stage identifier.
        body_ids: List of body identifiers to apply forces to.
        forces: List of force vectors to apply.
        positions: List of positions where forces are applied.
        sim_steps: Number of simulation steps to run.
        physx_dt: Physics time step duration.
        render: Whether to render each simulation step.
    """
    physx_sim_interface = omni.physx.get_physx_simulation_interface()

    # Apply the forces
    for body_id, force, position in zip(body_ids, forces, positions):
        physx_sim_interface.apply_force_at_pos(stage_id, body_id, carb.Float3(*force), carb.Float3(*position))

    # Run the simulation for the specified number of steps
    for _ in range(sim_steps):
        physx_sim_interface.simulate(physx_dt, 0)
        physx_sim_interface.fetch_results()
        if render:
            await omni.kit.app.get_app().next_update_async()


def disable_simulation_reset_on_stop() -> None:
    """Disable the simulation reset on stop setting. Needed to preserve the simulation state after play+stop."""
    carb.settings.get_settings().set(omni.physx.bindings._physx.SETTING_RESET_ON_STOP, False)


def reset_simulation_and_enable_reset_on_stop() -> None:
    """Reset the simulation and enable the simulation reset on stop setting. Needed to reset the simulation state after play+stop."""
    if carb.settings.get_settings().get(omni.physx.bindings._physx.SETTING_RESET_ON_STOP):
        carb.log_warn(
            "Expected 'omni.physx.bindings._physx.SETTING_RESET_ON_STOP' to be False, skipping reset_simulation"
        )
        return
    physics_stage_update_interface = omni.physics.core.get_physics_stage_update_interface()
    physics_stage_update_interface.reset_simulation()
    carb.settings.get_settings().set(omni.physx.bindings._physx.SETTING_RESET_ON_STOP, True)
