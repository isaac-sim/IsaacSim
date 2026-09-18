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

import importlib
import importlib.util
import logging
from pathlib import Path

from pxr import Plug, Sdf, Usd, UsdGeom, UsdPhysics

from .physx_types import PhysxAttr, PhysxSchema

_logger = logging.getLogger(__name__)


def _ensure_mjc_schemas_registered(required: bool = False) -> bool:
    """Register Newton and MuJoCo codeless USD schemas before they are queried.

    ``mujoco-usd-converter`` ships the ``mjcPhysics`` plugin but does not load it
    on import. Its schemas include Newton APIs, so import ``newton_usd_schemas``
    first to register the ``newton`` plugin, then register ``mjcPhysics``. URDF
    conversion applies ``MjcJointAPI`` by default, so both plugins must be loaded
    before USD constructs its process-wide schema registry.

    Args:
        required: Raise when the converter package or schema resources are absent.

    Returns:
        Whether the ``mjcPhysics`` plugin is registered.
    """
    registry = Plug.Registry()

    if not registry.GetPluginWithName("newton"):
        try:
            importlib.import_module("newton_usd_schemas")
        except ImportError as error:
            if required:
                raise RuntimeError(
                    "Newton USD schemas are required for URDF joint conversion, but the "
                    "newton-usd-schemas package is not installed."
                ) from error
    if required and not registry.GetPluginWithName("newton"):
        raise RuntimeError("Failed to register the 'newton' USD schema plugin.")

    if registry.GetPluginWithName("mjcPhysics"):
        return True

    spec = importlib.util.find_spec("mujoco_usd_converter")
    package_paths = spec.submodule_search_locations if spec is not None else None
    if not package_paths:
        if required:
            raise RuntimeError(
                "MuJoCo USD schemas are required for URDF joint conversion, but the "
                "mujoco-usd-converter package is not installed."
            )
        return False

    plugins_dir = Path(next(iter(package_paths))) / "plugins"
    if not plugins_dir.is_dir():
        if required:
            raise RuntimeError(f"MuJoCo USD schema plugin directory not found: {plugins_dir}")
        return False

    registered = registry.RegisterPlugins(str(plugins_dir))
    if not registry.GetPluginWithName("mjcPhysics"):
        if required:
            raise RuntimeError(
                f"Failed to register the 'mjcPhysics' USD schema plugin from {plugins_dir}. "
                f"Registered plugins: {[plugin.name for plugin in registered]}"
            )
        return False
    return True


_ensure_mjc_schemas_registered()


def _resolve_actuator_path(stage: Usd.Stage, scope_path: Sdf.Path | str, joint: Usd.Prim) -> Sdf.Path:
    """Return a collision-free actuator prim path for ``joint`` under ``scope_path``.

    Actuators live in a flat scope, but joint leaf names are not unique across a
    hierarchy (for example ``/robot/arm/joint1`` and ``/robot/leg/joint1``). Prefer
    ``<joint_leaf>_actuator``; if that path already belongs to a different joint,
    fall back to a name derived from the joint's full path (and warn) so no actuator
    is silently dropped or overwritten. Reusing the path of this joint's own
    actuator keeps re-runs idempotent.

    Args:
        stage: Stage used by the test.
        scope_path: Path to the USD scope.
        joint: Joint prim to configure.

    Returns:
        The resulting value.
    """
    scope = scope_path if isinstance(scope_path, Sdf.Path) else Sdf.Path(str(scope_path))

    def _targets_joint(prim: Usd.Prim) -> bool:
        rel = prim.GetRelationship("mjc:target")
        return bool(rel) and joint.GetPath() in rel.GetTargets()

    preferred = scope.AppendChild(f"{joint.GetName()}_actuator")
    existing = stage.GetPrimAtPath(preferred)
    if not existing.IsValid() or _targets_joint(existing):
        return preferred

    unique_leaf = joint.GetPath().pathString.strip("/").replace("/", "_") + "_actuator"
    candidate = scope.AppendChild(unique_leaf)
    suffix = 1
    while True:
        prim = stage.GetPrimAtPath(candidate)
        if not prim.IsValid():
            break
        if _targets_joint(prim):
            # This joint already owns the uniquified actuator: reuse it silently.
            return candidate
        candidate = scope.AppendChild(f"{unique_leaf}_{suffix}")
        suffix += 1
    _logger.warning(
        f"MjcActuator name collision at {preferred} for joint {joint.GetPath()}; "
        f"using unique name {candidate} to avoid overwriting another joint's actuator"
    )
    return candidate


def convert_joints_attributes(stage: Usd.Stage) -> None:
    """Convert all joint attributes to MJCF attributes.

    Args:
        stage: USD stage to update with MJCF attributes.
    """
    _ensure_mjc_schemas_registered(required=True)

    default_prim_path = stage.GetDefaultPrim().GetPath()
    scope_path = default_prim_path.AppendChild("Physics")

    if not stage.GetPrimAtPath(scope_path).IsValid():
        UsdGeom.Scope.Define(stage, scope_path)

    for prim in stage.Traverse():
        if prim.IsA(UsdPhysics.RevoluteJoint) or prim.IsA(UsdPhysics.PrismaticJoint):
            # Apply MjcJointAPI schema to the joint
            if "MjcJointAPI" not in prim.GetAppliedSchemas():
                prim.ApplyAPI("MjcJointAPI")
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
                f"Invalid joint limits {joint_limits} for joint {joint.GetPath()}, will be set to 0 "
                "(unrestricted force)"
            )
            joint_limits = 0
        drive_api.CreateMaxForceAttr().Set(joint_limits)

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


def create_mjc_actuator_from_physics(joint: Usd.Prim, stage: Usd.Stage, path: Sdf.Path | str) -> Usd.Prim | None:
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

    mjc_actuator = stage.DefinePrim(_resolve_actuator_path(stage, path, joint), "MjcActuator")
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
            f"Stiffness and damping not available joint {joint.GetPath()}, actuator will be created without gain "
            "parameters"
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
