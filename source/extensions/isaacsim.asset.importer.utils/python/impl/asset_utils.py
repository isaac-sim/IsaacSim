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

"""Utility helpers for post-import USD asset modifications (fix-base, density, joint drives)."""

from __future__ import annotations

import logging
import math
import re
from collections.abc import Callable

from pxr import Sdf, Usd, UsdPhysics, Vt

__all__ = [
    "apply_fix_base",
    "apply_floating_base",
    "fix_articulation_root_for_fixed_base",
    "apply_link_density",
    "apply_joint_drives",
    "apply_mjc_actuator_gains",
]

_logger = logging.getLogger(__name__)


def _get_joint_body(joint_prim: Usd.Prim, body_index: int) -> Sdf.Path | None:
    """Get the body relationship target for a joint.

    Args:
        joint_prim: The USD joint prim.
        body_index: Body slot index (0 or 1) for body0 or body1.

    Returns:
        Path to the linked body prim, or ``None`` if not applicable.
    """
    joint = UsdPhysics.Joint(joint_prim)
    if not joint:
        return None
    exclude_attr = joint.GetExcludeFromArticulationAttr()
    if exclude_attr and exclude_attr.Get():
        return None
    rel = joint.GetBody0Rel() if body_index == 0 else joint.GetBody1Rel()
    if rel:
        targets = rel.GetTargets()
        if targets:
            return targets[0]
    return None


def _detect_fixed_base(stage: Usd.Stage, root_link: Usd.Prim, joints: list[Usd.Prim]) -> bool:
    """Determine if the robot has a fixed base.

    A robot is fixed-base if there is a FixedJoint connecting the root link to:
    - A body0 with no target (world)
    - A body0 targeting a non-RigidBody prim (e.g. the default prim / scene root)

    Args:
        stage: The USD stage containing the joints and links.
        root_link: The root rigid-body link prim.
        joints: Joint prims to inspect (typically all physics joints on *stage*).

    Returns:
        ``True`` if a fixed-base configuration is detected, else ``False``.
    """
    root_path = str(root_link.GetPath())

    for j in joints:
        if not j.IsA(UsdPhysics.FixedJoint):
            continue

        body0 = _get_joint_body(j, 0)
        body1 = _get_joint_body(j, 1)

        target_is_root = False
        other_body = None

        if body1 and str(body1) == root_path:
            target_is_root = True
            other_body = body0
        elif body0 and str(body0) == root_path:
            target_is_root = True
            other_body = body1

        if not target_is_root:
            continue

        if other_body is None:
            return True

        other_prim = stage.GetPrimAtPath(other_body)
        if other_prim is None or not other_prim.IsValid():
            return True
        if not other_prim.HasAPI(UsdPhysics.RigidBodyAPI):
            return True

    return False


def _find_articulation_root_link(stage: Usd.Stage) -> Usd.Prim | None:
    """Find the root rigid-body link of the articulation.

    The root is the rigid body that is never the child (``body1``) of a
    non-fixed joint whose other side is also a rigid body.

    Args:
        stage: The USD stage to inspect.

    Returns:
        The articulation root prim, or ``None`` if no rigid body exists.
    """
    rigid_bodies: list[Usd.Prim] = [p for p in stage.Traverse() if p.HasAPI(UsdPhysics.RigidBodyAPI)]
    if not rigid_bodies:
        return None

    rigid_body_paths = {str(p.GetPath()) for p in rigid_bodies}

    child_paths: set[str] = set()
    for prim in stage.Traverse():
        if not prim.IsA(UsdPhysics.Joint) or prim.IsA(UsdPhysics.FixedJoint):
            continue
        body0 = _get_joint_body(prim, 0)
        body1 = _get_joint_body(prim, 1)
        if body0 and body1 and str(body0) in rigid_body_paths and str(body1) in rigid_body_paths:
            child_paths.add(str(body1))

    candidates = [p for p in rigid_bodies if str(p.GetPath()) not in child_paths]
    if not candidates:
        _logger.warning(
            "Could not determine articulation root - every rigid body is a joint child. " "Falling back to %s",
            rigid_bodies[0].GetPath(),
        )
        return rigid_bodies[0]
    if len(candidates) > 1:
        _logger.info(
            "Multiple articulation root candidates (%d); using %s",
            len(candidates),
            candidates[0].GetPath(),
        )
    return candidates[0]


def apply_fix_base(stage: Usd.Stage) -> None:
    """Add a fixed joint from the world to the articulation root link.

    Also relocates any ``ArticulationRootAPI`` off the root rigid body.

    Args:
        stage: The USD stage to modify.
    """
    default_prim = stage.GetDefaultPrim()
    if not default_prim or not default_prim.IsValid():
        _logger.warning("Cannot apply fix_base - no default prim found.")
        return

    root_link = _find_articulation_root_link(stage)
    if root_link is None:
        _logger.warning("Cannot apply fix_base - no rigid body link found.")
        return

    joints = [prim for prim in stage.Traverse() if prim.IsA(UsdPhysics.Joint)]

    if _detect_fixed_base(stage, root_link, joints):
        _logger.info("Fixed base already present on %s - skipping fix_base.", root_link.GetPath())
    else:
        _logger.info("Applying fix_base to %s.", root_link.GetPath())
        joint_path = default_prim.GetPath().AppendChild("fix_base_joint")
        fixed_joint = UsdPhysics.FixedJoint.Define(stage, joint_path)
        fixed_joint.CreateBody1Rel().SetTargets([root_link.GetPath()])

    # PhysX requires ArticulationRootAPI on the parent of the root rigid body
    # for proper reduced-coordinate fixed-base articulations.
    relocated = fix_articulation_root_for_fixed_base(stage)
    if relocated:
        _logger.info("Relocated ArticulationRootAPI on %d rigid body(ies).", relocated)


def apply_floating_base(stage: Usd.Stage) -> None:
    """Remove any fixed joint anchoring the articulation root link to the world.

    The inverse of :func:`apply_fix_base`. Drops every ``FixedJoint`` whose
    body pair is ``(world | non-rigid prim, articulation root link)`` (in
    either order), turning a fixed-base robot into a floating-base one.
    Internal fixed joints between two rigid bodies are left alone.

    Args:
        stage: The USD stage to modify.
    """
    root_link = _find_articulation_root_link(stage)
    if root_link is None:
        _logger.warning("Cannot apply floating_base - no rigid body link found.")
        return

    root_path = str(root_link.GetPath())
    joints_to_remove: list[Sdf.Path] = []

    for joint in [p for p in stage.Traverse() if p.IsA(UsdPhysics.FixedJoint)]:
        body0 = _get_joint_body(joint, 0)
        body1 = _get_joint_body(joint, 1)

        if body1 and str(body1) == root_path:
            other_body = body0
        elif body0 and str(body0) == root_path:
            other_body = body1
        else:
            continue

        if other_body is None:
            joints_to_remove.append(joint.GetPath())
            continue
        other_prim = stage.GetPrimAtPath(other_body)
        if other_prim is None or not other_prim.IsValid() or not other_prim.HasAPI(UsdPhysics.RigidBodyAPI):
            joints_to_remove.append(joint.GetPath())

    for path in joints_to_remove:
        _logger.info("Removing world-anchoring fixed joint %s for floating base.", path)
        stage.RemovePrim(path)

    if not joints_to_remove:
        _logger.info("No world-anchoring fixed joint found on %s - already floating-base.", root_path)


def fix_articulation_root_for_fixed_base(stage: Usd.Stage) -> int:
    """Move ArticulationRootAPI from rigid bodies to their parent prims.

    After the asset transformer, ArticulationRootAPI ends up on the root rigid
    body.  When combined with a FixedJoint on that same body, PhysX treats the
    articulation as a floating-base + external constraint (maximal coordinate
    tree) rather than a proper fixed-base reduced-coordinate articulation.

    Moving ArticulationRootAPI to the parent of the root rigid body resolves
    this.  All matching rigid bodies in the stage are processed.

    Args:
        stage: The USD stage to modify.

    Returns:
        Number of articulation roots that were relocated.
    """
    root_body_prims = [
        prim
        for prim in stage.Traverse()
        if prim.HasAPI(UsdPhysics.ArticulationRootAPI) and prim.HasAPI(UsdPhysics.RigidBodyAPI)
    ]

    if not root_body_prims:
        return 0

    if len(root_body_prims) > 1:
        paths = [str(p.GetPath()) for p in root_body_prims]
        _logger.warning(
            f"Multiple articulation roots found on rigid bodies ({len(root_body_prims)}): {paths}. "
            "Relocating all of them to their respective parent prims."
        )

    relocated = 0
    for root_body_prim in root_body_prims:
        parent_prim = root_body_prim.GetParent()
        if not parent_prim or not parent_prim.IsValid():
            _logger.warning(
                f"Rigid body {root_body_prim.GetPath()} has no valid parent prim" " - skipping ArticulationRootAPI fix."
            )
            continue

        articulation_api_names = [
            name
            for name in root_body_prim.GetAppliedSchemas()
            if "ArticulationRoot" in name or name == "PhysxArticulationAPI"
        ]

        if parent_prim.HasAPI(UsdPhysics.ArticulationRootAPI):
            _logger.warning(
                f"Parent prim {parent_prim.GetPath()} already has ArticulationRootAPI."
                f" Removing duplicate from rigid body {root_body_prim.GetPath()}."
            )
            _remove_articulation_schemas(root_body_prim, articulation_api_names)
            relocated += 1
            continue

        UsdPhysics.ArticulationRootAPI.Apply(parent_prim)
        already_on_parent = set(parent_prim.GetAppliedSchemas())
        for name in articulation_api_names:
            if name != "PhysicsArticulationRootAPI" and name not in already_on_parent:
                parent_prim.AddAppliedSchema(name)

        _copy_articulation_attrs(root_body_prim, parent_prim)
        _remove_articulation_schemas(root_body_prim, articulation_api_names)

        relocated += 1

    return relocated


def _copy_articulation_attrs(src: Usd.Prim, dst: Usd.Prim) -> None:
    """Copy ArticulationRootAPI and physxArticulation attributes from *src* to *dst*.

    Args:
        src: Source prim carrying articulation attributes.
        dst: Destination prim to receive copied attributes.
    """
    usd_art_api = UsdPhysics.ArticulationRootAPI(src)
    for attr_name in usd_art_api.GetSchemaAttributeNames():
        attr = src.GetAttribute(attr_name)
        val = attr.Get() if attr else None
        if val is not None:
            dst_attr = dst.GetAttribute(attr_name)
            if not dst_attr:
                dst_attr = dst.CreateAttribute(attr_name, attr.GetTypeName())
            dst_attr.Set(val)

    for attr in src.GetAttributes():
        aname = attr.GetName()
        if aname.startswith("physxArticulation:"):
            val = attr.Get()
            if val is not None:
                dst_attr = dst.GetAttribute(aname)
                if not dst_attr:
                    dst_attr = dst.CreateAttribute(aname, attr.GetTypeName())
                dst_attr.Set(val)


def _remove_articulation_schemas(prim: Usd.Prim, api_names: list[str]) -> None:
    """Remove all articulation-related schemas from *prim*.

    Args:
        prim: Prim whose articulation schemas should be stripped.
        api_names: Applied schema names collected from the source (articulation-related).
    """
    prim.RemoveAppliedSchema("PhysxArticulationAPI")
    prim.RemoveAPI(UsdPhysics.ArticulationRootAPI)
    for name in api_names:
        if name not in ("PhysicsArticulationRootAPI", "PhysxArticulationAPI"):
            prim.RemoveAppliedSchema(name)


def apply_link_density(stage: Usd.Stage, density: float) -> None:
    """Set default density on rigid body links that have no explicit mass.

    Args:
        stage: The USD stage to modify.
        density: The density value in kg/m^3.
    """
    for prim in stage.Traverse():
        if not (prim.HasAPI(UsdPhysics.RigidBodyAPI) or prim.HasAPI("PhysicsRigidBodyAPI")):
            continue
        if not prim.HasAPI(UsdPhysics.MassAPI):
            UsdPhysics.MassAPI.Apply(prim)
        mass_api = UsdPhysics.MassAPI(prim)
        mass_attr = mass_api.GetMassAttr()
        if mass_attr and mass_attr.HasValue() and mass_attr.Get() > 0.0:
            continue
        density_attr = mass_api.GetDensityAttr()
        if not density_attr:
            density_attr = mass_api.CreateDensityAttr()
        density_attr.Set(density)


def _collect_joints(stage: Usd.Stage) -> dict[str, tuple]:
    """Collect all revolute/prismatic joints from *stage*.

    Args:
        stage: The USD stage to traverse for joint prims.

    Returns:
        Mapping of joint name to ``(prim, is_revolute, instance_name)``.
    """
    joints: dict[str, tuple] = {}
    for prim in stage.Traverse():
        if not (prim.IsA(UsdPhysics.RevoluteJoint) or prim.IsA(UsdPhysics.PrismaticJoint)):
            continue
        is_revolute = prim.IsA(UsdPhysics.RevoluteJoint)
        instance_name = "angular" if is_revolute else "linear"
        joints[prim.GetName()] = (prim, is_revolute, instance_name)
    return joints


def apply_joint_drives(
    stage: Usd.Stage,
    drive_type: str | dict[str, str] | None = None,
    target_type: str | dict[str, str] | None = None,
    stiffness: float | dict[str, float] | None = None,
    damping: float | dict[str, float] | None = None,
) -> None:
    """Set joint drive properties (type, target, gains) on USD joints.

    Each parameter accepts either a single value (applied to all joints) or a
    ``dict`` mapping regex patterns to per-joint values.

    Args:
        stage: The USD stage to modify.
        drive_type: Drive type string (``"force"`` or ``"acceleration"``), or a
            dict mapping joint-name regex patterns to drive type strings.
        target_type: Target type string (``"none"``, ``"position"``, or
            ``"velocity"``), or a dict of patterns.
        stiffness: Stiffness in Nm/rad (revolute) or N/m (prismatic), or a
            dict of patterns.  Revolute values are converted to the USD
            Nm/deg convention internally.
        damping: Damping in Nm*s/rad (revolute) or N*s/m (prismatic), or a
            dict of patterns.  Same unit conversion as *stiffness*.
    """
    joints = _collect_joints(stage)
    if not joints:
        return

    if drive_type is not None:
        _set_drive_type_on_joints(joints, drive_type)
    if target_type is not None:
        _set_target_type_on_joints(joints, target_type)
    if stiffness is not None:
        _set_stiffness_on_joints(joints, stiffness)
    if damping is not None:
        _set_damping_on_joints(joints, damping)


def _set_drive_type_on_joints(
    joints: dict[str, tuple],
    drive_type: str | dict[str, str],
) -> None:
    """Set the drive type (force or acceleration) on joint prims.

    Args:
        joints: Mapping of joint name to ``(prim, is_revolute, instance_name)``.
        drive_type: A single type string or a dict of regex-pattern to type.
    """

    def _apply(prim: Usd.Prim, instance_name: str, value: str) -> None:
        drive = UsdPhysics.DriveAPI.Get(prim, instance_name)
        type_attr = drive.GetTypeAttr()
        if not type_attr:
            type_attr = drive.CreateTypeAttr()
        type_attr.Set(value)

    _apply_to_joints(joints, drive_type, _apply)


def _set_target_type_on_joints(
    joints: dict[str, tuple],
    target_type: str | dict[str, str],
) -> None:
    """Set the target type (none, effort, position, velocity) on joint prims.

    For ``"none"`` or ``"effort"``, both stiffness and damping are zeroed out.
    For ``"velocity"``, stiffness is zeroed out and damping is set to a non-zero value.

    Args:
        joints: Mapping of joint name to ``(prim, is_revolute, instance_name)``.
        target_type: A single type string or a dict of regex-pattern to type.
    """

    def _apply(prim: Usd.Prim, instance_name: str, value: str) -> None:
        drive = UsdPhysics.DriveAPI.Get(prim, instance_name)
        if value == "none" or value == "effort":
            if drive.GetStiffnessAttr():
                drive.GetStiffnessAttr().Set(0.0)
            else:
                drive.CreateStiffnessAttr().Set(0.0)
            if drive.GetDampingAttr():
                drive.GetDampingAttr().Set(0.0)
            else:
                drive.CreateDampingAttr().Set(0.0)

        elif value == "velocity":
            if drive.GetStiffnessAttr():
                drive.GetStiffnessAttr().Set(0.0)
            else:
                drive.CreateStiffnessAttr().Set(0.0)

    _apply_to_joints(joints, target_type, _apply)


def _set_stiffness_on_joints(
    joints: dict[str, tuple],
    stiffness: float | dict[str, float],
) -> None:
    """Set stiffness on joint drive APIs.

    For revolute joints values (Nm/rad) are converted to USD (Nm/deg).

    Args:
        joints: Mapping of joint name to ``(prim, is_revolute, instance_name)``.
        stiffness: A single value or a dict of regex-pattern to value.
    """

    def _apply(prim: Usd.Prim, instance_name: str, value: float, *, is_revolute: bool = False) -> None:
        drive = UsdPhysics.DriveAPI.Get(prim, instance_name)
        usd_value = value * math.pi / 180.0 if is_revolute else value
        attr = drive.GetStiffnessAttr()
        if not attr:
            attr = drive.CreateStiffnessAttr()
        attr.Set(usd_value)

    _apply_to_joints(joints, stiffness, _apply, pass_is_revolute=True)


def _set_damping_on_joints(
    joints: dict[str, tuple],
    damping: float | dict[str, float],
) -> None:
    """Set damping on joint drive APIs.

    For revolute joints values (Nm*s/rad) are converted to USD (Nm*s/deg).

    Args:
        joints: Mapping of joint name to ``(prim, is_revolute, instance_name)``.
        damping: A single value or a dict of regex-pattern to value.
    """

    def _apply(prim: Usd.Prim, instance_name: str, value: float, *, is_revolute: bool = False) -> None:
        drive = UsdPhysics.DriveAPI.Get(prim, instance_name)
        usd_value = value * math.pi / 180.0 if is_revolute else value
        attr = drive.GetDampingAttr()
        if not attr:
            attr = drive.CreateDampingAttr()
        attr.Set(usd_value)

    _apply_to_joints(joints, damping, _apply, pass_is_revolute=True)


def apply_mjc_actuator_gains(
    stage: Usd.Stage,
    gain_type: str | None,
    bias_type: str | None,
    gain_prm: list[float] | None,
    bias_prm: list[float] | None,
) -> int:
    """Set MJCF actuator gain parameters on all MjcActuator prims.

    Finds every ``MjcActuator`` prim in the stage and writes the given
    ``gainType``, ``biasType``, ``gainPrm``, and ``biasPrm`` attributes,
    following the encoding used by ``create_mjc_actuator_from_physics`` in
    ``urdf_to_mjc_physx_conversion_utils`` and read back by
    ``convert_mjc_actuator_to_physics`` in ``mjc_to_physx_conversion_utils``.

    Common configurations:

    * **Position control** (kp + kd):
      ``gain_type="fixed"``, ``bias_type="affine"``,
      ``gain_prm=[kp, 0, ...]``, ``bias_prm=[0, -kp, -kd, 0, ...]``

    * **Velocity control** (kd only):
      ``gain_type="fixed"``, ``bias_type="affine"``,
      ``gain_prm=[kd, 0, ...]``, ``bias_prm=[0, 0, -kd, 0, ...]``

    Args:
        stage: The USD stage containing MjcActuator prims.
        gain_type: MuJoCo gain type string (e.g. ``"fixed"``), or ``None`` to skip.
        bias_type: MuJoCo bias type string (e.g. ``"affine"``), or ``None`` to skip.
        gain_prm: Gain parameter array (10 floats), or ``None`` to skip.
        bias_prm: Bias parameter array (10 floats), or ``None`` to skip.

    Returns:
        Number of MjcActuator prims updated.
    """
    updated = 0
    for prim in stage.Traverse():
        if prim.GetTypeName() != "MjcActuator":
            continue

        if gain_prm is not None:
            _set_or_create_float_array(prim, "mjc:gainPrm", gain_prm)
        if bias_prm is not None:
            _set_or_create_float_array(prim, "mjc:biasPrm", bias_prm)
        if gain_type is not None:
            _set_or_create_string(prim, "mjc:gainType", gain_type)
        if bias_type is not None:
            _set_or_create_string(prim, "mjc:biasType", bias_type)
        updated += 1

    _logger.info(f"Updated gain parameters on {updated} MjcActuator prims.")
    return updated


def _set_or_create_float_array(prim: Usd.Prim, name: str, values: list[float]) -> None:
    """Set or create a FloatArray attribute on *prim*.

    Args:
        prim: Prim owning the attribute.
        name: Fully qualified attribute name.
        values: Float values to assign to the array attribute.
    """
    attr = prim.GetAttribute(name)
    if not attr or not attr.IsValid():
        attr = prim.CreateAttribute(name, Sdf.ValueTypeNames.FloatArray)
    attr.Set(Vt.FloatArray(values))


def _set_or_create_string(prim: Usd.Prim, name: str, value: str) -> None:
    """Set or create a String attribute on *prim*.

    Args:
        prim: Prim owning the attribute.
        name: Fully qualified attribute name.
        value: String value to assign.
    """
    attr = prim.GetAttribute(name)
    if not attr or not attr.IsValid():
        attr = prim.CreateAttribute(name, Sdf.ValueTypeNames.String)
    attr.Set(value)


def _apply_to_joints(
    joints: dict[str, tuple],
    spec: object,
    fn: Callable[..., None],
    *,
    pass_is_revolute: bool = False,
) -> None:
    """Dispatch *fn* across joints for a scalar or pattern-dict *spec*.

    Args:
        joints: Mapping of joint name to ``(prim, is_revolute, instance_name)``.
        spec: A scalar value applied to all joints, or a dict mapping regex
            patterns to per-match values.
        fn: Callable ``(prim, instance_name, value, **kw)`` to invoke.
        pass_is_revolute: If ``True``, forward ``is_revolute`` as a keyword
            argument to *fn*.
    """
    if isinstance(spec, dict):
        for pattern, value in spec.items():
            matches = [n for n in joints if re.search(pattern, n)]
            if not matches:
                raise ValueError(
                    f"Joint name pattern '{pattern}' matched no joints." f" Available joints: {list(joints.keys())}"
                )
            for name in matches:
                prim, is_rev, inst = joints[name]
                kw = {"is_revolute": is_rev} if pass_is_revolute else {}
                fn(prim, inst, value, **kw)
    else:
        for _name, (prim, is_rev, inst) in joints.items():
            kw = {"is_revolute": is_rev} if pass_is_revolute else {}
            fn(prim, inst, spec, **kw)
