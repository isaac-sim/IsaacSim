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

"""Utilities for converting URDF/PhysX joint data into MJCF-compatible data."""

from __future__ import annotations

import logging

from pxr import Sdf, Usd, UsdGeom, UsdPhysics

from .physx_types import PhysxAttr, PhysxSchema

_logger = logging.getLogger(__name__)


def convert_joints_attributes(stage: Usd.Stage) -> None:
    """Convert all joint attributes to MJCF attributes.

    Args:
        stage: USD stage to update with MJCF attributes.
    """
    default_prim_path = stage.GetDefaultPrim().GetPath()
    scope_path = default_prim_path.AppendChild("Physics")

    if not stage.GetPrimAtPath(scope_path).IsValid():
        UsdGeom.Scope.Define(stage, scope_path)

    for prim in stage.Traverse():
        if prim.IsA(UsdPhysics.RevoluteJoint) or prim.IsA(UsdPhysics.PrismaticJoint):
            convert_urdf_to_physx(prim)
            create_mjc_actuator_from_physics(prim, stage, scope_path)
            convert_physx_to_mjc(prim)


def convert_urdf_to_physx(joint: Usd.Prim) -> None:
    """Convert URDF attributes to PhysX attributes.

    Args:
        joint: Joint prim.

    Raises:
        ValueError: If the input joint prim is invalid.
    """
    if not joint.IsValid():
        raise ValueError(f"URDF joint prim not found at path: {joint.GetPath()}")

    joint_type: str | None = None
    if joint.IsA(UsdPhysics.RevoluteJoint):
        joint_type = "angular"
    elif joint.IsA(UsdPhysics.PrismaticJoint):
        joint_type = "linear"
    else:
        return

    if joint.HasAPI(UsdPhysics.DriveAPI, joint_type):
        drive_api = UsdPhysics.DriveAPI(joint, joint_type)
    else:
        drive_api = UsdPhysics.DriveAPI.Apply(joint, joint_type)

    # Set joint limits
    joint_limits = (
        joint.GetAttribute("urdf:limit:effort").Get() if joint.GetAttribute("urdf:limit:effort").IsValid() else None
    )

    if joint_limits:
        if joint_limits < 0:
            _logger.warning(
                f"Invalid joint limits {joint_limits} for joint {joint.GetPath()}, will be set to 0 (unrestricted force)"
            )
            joint_limits = 0
        drive_api.CreateMaxForceAttr().Set(joint_limits)

    joint_max_velocity = (
        joint.GetAttribute("urdf:limit:velocity").Get() if joint.GetAttribute("urdf:limit:velocity").IsValid() else None
    )

    if joint_max_velocity:
        if not joint.HasAPI(PhysxSchema.JOINT_API):
            joint.ApplyAPI(PhysxSchema.JOINT_API)
        if joint_max_velocity < 0:
            _logger.warning(
                f"Invalid joint max velocity {joint_max_velocity} for joint {joint.GetPath()}, will be set to 0 (unrestricted velocity)"
            )
            joint_max_velocity = 0
        joint_max_velocity_deg = joint_max_velocity * 180 / 3.1415926
        joint.CreateAttribute(PhysxAttr.JOINT_MAX_VELOCITY.name, PhysxAttr.JOINT_MAX_VELOCITY.type).Set(
            joint_max_velocity_deg
        )

    damping = (
        joint.GetAttribute("urdf:dynamics:damping").Get()
        if joint.GetAttribute("urdf:dynamics:damping").IsValid()
        else None
    )

    if damping:
        drive_api.CreateDampingAttr().Set(damping)

    friction = (
        joint.GetAttribute("urdf:dynamics:friction").Get()
        if joint.GetAttribute("urdf:dynamics:friction").IsValid()
        else None
    )

    if friction:
        if not joint.HasAPI(PhysxSchema.JOINT_API):
            joint.ApplyAPI(PhysxSchema.JOINT_API)
        joint.CreateAttribute(PhysxAttr.JOINT_FRICTION.name, PhysxAttr.JOINT_FRICTION.type).Set(friction)

    target_position = (
        joint.GetAttribute("urdf:calibration:reference_position").Get()
        if joint.GetAttribute("urdf:calibration:reference_position").IsValid()
        else None
    )

    if target_position:
        drive_api.CreateTargetPositionAttr().Set(target_position)


def create_mjc_actuator_from_physics(joint: Usd.Prim, stage: Usd.Stage, path: str) -> Usd.Prim | None:
    """Create an MJCF actuator for a joint.

    Args:
        joint: URDF joint prim.
        stage: USD stage to update with MJCF attributes.
        path: Path to the MJCF actuator scope.

    Returns:
        The created MJCF actuator prim.

    Raises:
        ValueError: If the input joint prim is invalid.
    """
    if not joint.IsValid():
        raise ValueError(f"URDF joint prim not found at path: {joint.GetPath()}")

    joint_type: str | None = None
    if joint.IsA(UsdPhysics.RevoluteJoint):
        joint_type = "angular"
    elif joint.IsA(UsdPhysics.PrismaticJoint):
        joint_type = "linear"
    else:
        return None

    mjc_actuator = stage.DefinePrim(f"{path}/{joint.GetName()}_actuator", "MjcActuator")
    mjc_actuator.CreateRelationship("mjc:target", custom=False).SetTargets([joint.GetPath()])

    if joint.HasAPI(UsdPhysics.DriveAPI, joint_type):
        drive_api = UsdPhysics.DriveAPI(joint, joint_type)
    else:
        drive_api = UsdPhysics.DriveAPI.Apply(joint, joint_type)

    max_force = drive_api.GetMaxForceAttr().Get() if drive_api.GetMaxForceAttr().IsValid() else None
    if max_force:
        mjc_actuator.CreateAttribute("mjc:forceRange:max", Sdf.ValueTypeNames.Float).Set(max_force)
        mjc_actuator.CreateAttribute("mjc:forceRange:min", Sdf.ValueTypeNames.Float).Set(-max_force)

    stiffness = drive_api.GetStiffnessAttr().Get() if drive_api.GetStiffnessAttr().IsValid() else 0
    damping = drive_api.GetDampingAttr().Get() if drive_api.GetDampingAttr().IsValid() else 0

    # position control
    # "gainprm" = [kp, 0, 0, 0, 0, 0, 0, 0, 0, 0]
    # "biasprm" = [0, -kp, -kd, 0, 0, 0, 0, 0, 0, 0]
    # stiffness = kp
    # damping = kd

    if stiffness > 0 and damping > 0:
        gain_prm = [stiffness, 0, 0, 0, 0, 0, 0, 0, 0, 0]
        bias_prm = [0, -stiffness, -damping, 0, 0, 0, 0, 0, 0, 0]
        mjc_actuator.CreateAttribute("mjc:gainPrm", Sdf.ValueTypeNames.FloatArray).Set(gain_prm)
        mjc_actuator.CreateAttribute("mjc:biasPrm", Sdf.ValueTypeNames.FloatArray).Set(bias_prm)
        mjc_actuator.CreateAttribute("mjc:gainType", Sdf.ValueTypeNames.String).Set("fixed")
        mjc_actuator.CreateAttribute("mjc:biasType", Sdf.ValueTypeNames.String).Set("affine")

    # velocity control
    # "gainprm" = [kd, 0, 0, 0, 0, 0, 0, 0, 0, 0]
    # "biasprm" = [0, 0, -kd, 0, 0, 0, 0, 0, 0, 0]
    # stiffness = 0
    # damping = kd

    elif damping > 0 and stiffness == 0:
        gain_prm = [damping, 0, 0, 0, 0, 0, 0, 0, 0, 0]
        bias_prm = [0, 0, -damping, 0, 0, 0, 0, 0, 0, 0]
        mjc_actuator.CreateAttribute("mjc:gainPrm", Sdf.ValueTypeNames.FloatArray).Set(gain_prm)
        mjc_actuator.CreateAttribute("mjc:biasPrm", Sdf.ValueTypeNames.FloatArray).Set(bias_prm)
        mjc_actuator.CreateAttribute("mjc:gainType", Sdf.ValueTypeNames.String).Set("fixed")
        mjc_actuator.CreateAttribute("mjc:biasType", Sdf.ValueTypeNames.String).Set("affine")

    else:
        _logger.warning(
            f"Stiffness and damping not available joint {joint.GetPath()}, actuator will be created without gain parameters"
        )

    return mjc_actuator


def convert_physx_to_mjc(joint: Usd.Prim) -> None:
    """Convert a PhysX joint to an MJCF joint.

    Args:
        joint: PhysX joint prim.

    Raises:
        ValueError: If the input joint prim is invalid.
    """
    if not joint.IsValid():
        raise ValueError(f"PhysX joint prim not found at path: {joint.GetPath()}")

    if joint.IsA(UsdPhysics.RevoluteJoint):
        joint_type = "angular"
    elif joint.IsA(UsdPhysics.PrismaticJoint):
        joint_type = "linear"
    else:
        return

    if joint.HasAPI(UsdPhysics.DriveAPI, joint_type):
        drive_api = UsdPhysics.DriveAPI(joint, joint_type)

        target_position = (
            drive_api.GetTargetPositionAttr().Get() if drive_api.GetTargetPositionAttr().IsValid() else None
        )
        if target_position:
            joint.CreateAttribute("mjc:ref", Sdf.ValueTypeNames.Float).Set(target_position)

    if joint.HasAPI(PhysxSchema.JOINT_API):
        friction_attr = joint.GetAttribute(PhysxAttr.JOINT_FRICTION.name)
        joint_friction = friction_attr.Get() if friction_attr and friction_attr.IsValid() else None
        if joint_friction:
            joint.CreateAttribute("mjc:frictionloss", Sdf.ValueTypeNames.Float).Set(joint_friction)

        armature_attr = joint.GetAttribute(PhysxAttr.JOINT_ARMATURE.name)
        armature = armature_attr.Get() if armature_attr and armature_attr.IsValid() else None
        if armature:
            joint.CreateAttribute("mjc:armature", Sdf.ValueTypeNames.Float).Set(armature)
