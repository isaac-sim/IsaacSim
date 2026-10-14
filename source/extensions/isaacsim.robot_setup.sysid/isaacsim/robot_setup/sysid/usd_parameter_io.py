# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

"""USD attribute read/write for SysID parameters."""

from __future__ import annotations

from dataclasses import dataclass
from typing import TYPE_CHECKING, Any, Optional

import numpy as np

if TYPE_CHECKING:
    from pxr import Usd, UsdPhysics


def _physx_schema() -> Any:
    """Return the PhysX schema module when it is available.

    Pixar's usd-core wheel intentionally does not ship Omniverse PhysX schemas.
    Standalone Newton still needs to read ordinary USD physics and any
    already-authored PhysX attributes by name.

    Returns:
        The ``pxr.PhysxSchema`` module, or ``None`` when it is not installed.
    """
    try:
        from pxr import PhysxSchema
    except ImportError:
        return None
    return PhysxSchema


@dataclass(frozen=True)
class LinkUsdSnapshot:
    """PhysX/USD link properties relevant to system identification."""

    path: str
    mass: float
    com_offset: np.ndarray
    diagonal_inertia: np.ndarray
    contact_offset: float
    principal_axes_quat: np.ndarray | None = None
    inertia_matrix: np.ndarray | None = None


@dataclass(frozen=True)
class JointUsdSnapshot:
    """Joint drive, friction, and limit snapshot from USD."""

    path: str
    lower_limit: float
    upper_limit: float
    is_angular: bool
    friction: float = 0.0
    static_friction: float = 0.0
    dynamic_friction: float = 0.0
    viscous_friction: float = 0.0
    stiffness: float = 0.0
    damping: float = 0.0
    armature: float = 0.0
    max_force: float = float("inf")
    dof_index: int = -1
    uses_newton_friction: bool = False
    # Always False: drive damping is read from and written to the drive attribute on every backend,
    # because Newton applies ``newton:damping`` as passive damping on top of the drive gain.
    uses_newton_damping: bool = False
    uses_newton_armature: bool = False


def _mass_api(prim: Usd.Prim, *, apply: bool = False) -> UsdPhysics.MassAPI | None:
    """Return a link's mass API, only authoring the schema when writing.

    Args:
        prim: Link prim to inspect.
        apply: Whether an absent ``MassAPI`` may be applied to the prim.

    Returns:
        The mass API, or ``None`` when the prim has none and ``apply`` is false.
    """
    from pxr import UsdPhysics

    if prim.HasAPI(UsdPhysics.MassAPI):
        return UsdPhysics.MassAPI(prim)
    # Reading must not dirty the stage: preflight, Check, and prepare all read
    # link snapshots, and applying a schema there would author into the user's
    # layer before any solve has been accepted.
    return UsdPhysics.MassAPI.Apply(prim) if apply else None


def _normalize_physics_backend(physics_backend: str) -> str:
    """Validate a physics backend name used to pick solver-specific joint properties.

    Args:
        physics_backend: Backend name, or an empty string or ``"auto"`` when unspecified.

    Returns:
        The lowercase backend name, or an empty string when unspecified.

    Raises:
        ValueError: If the backend is not empty, ``"auto"``, ``"newton"``, or ``"physx"``.
    """
    backend = str(physics_backend).strip().lower()
    if backend == "auto":
        return ""
    if backend not in {"", "newton", "physx"}:
        raise ValueError(f"Unsupported physics backend for USD joint properties: {physics_backend!r}")
    return backend


def _uses_newton_attr(newton_attr: Usd.Attribute, physx_attr: Usd.Attribute | None, backend: str) -> bool:
    """Decide whether an authored ``newton:*`` joint property supplies the snapshot value.

    Args:
        newton_attr: Newton-namespaced joint attribute.
        physx_attr: Matching PhysX attribute, if any.
        backend: Normalized physics backend name.

    Returns:
        True when the Newton value is authored and either the backend is Newton, or the backend is
        unspecified and the matching PhysX value is unauthored.
    """
    if backend == "physx" or not (newton_attr.IsValid() and newton_attr.HasAuthoredValueOpinion()):
        return False
    if backend == "newton":
        return True
    # Newton writeback leaves the source PhysX value in place, so only a Newton
    # read may prefer the Newton value when both are authored.
    return not (physx_attr is not None and physx_attr.IsValid() and physx_attr.HasAuthoredValueOpinion())


def _drive_instance_for_joint(prim: Usd.Prim) -> str | None:
    from pxr import UsdPhysics

    if prim.IsA(UsdPhysics.RevoluteJoint):
        return "angular"
    if prim.IsA(UsdPhysics.PrismaticJoint):
        return "linear"
    return None


def _drive_api(prim: Usd.Prim, *, apply: bool = False) -> UsdPhysics.DriveAPI | None:
    from pxr import UsdPhysics

    instance = _drive_instance_for_joint(prim)
    if instance is None:
        return None
    if not prim.HasAPI(UsdPhysics.DriveAPI, instance):
        if apply:
            return UsdPhysics.DriveAPI.Apply(prim, instance)
        return None
    return UsdPhysics.DriveAPI(prim, instance)


def _float_attr(attr, default: float = 0.0) -> float:  # noqa: ANN001
    if attr is None or not attr.IsValid():
        return default
    value = attr.Get()
    return default if value is None else float(value)


def _attr_value_or_default(attr, default):  # noqa: ANN001, ANN202
    if attr is None or not attr.IsValid():
        return default
    value = attr.Get()
    return default if value is None else value


def _authored_attr_value_or_default(attr, default):  # noqa: ANN001, ANN202
    if attr is None or not attr.IsValid() or not attr.HasAuthoredValueOpinion():
        return default
    value = attr.Get()
    return default if value is None else value


def _set_float_attr(prim: Usd.Prim, name: str, value: float, *, custom: bool = True) -> None:
    from pxr import Sdf

    attr = prim.GetAttribute(name)
    if attr is None or not attr.IsValid():
        attr = prim.CreateAttribute(name, Sdf.ValueTypeNames.Float, custom=custom)
    attr.Set(float(value))


def _api_prim(api) -> Usd.Prim | None:  # noqa: ANN001
    getter = getattr(api, "GetPrim", None)
    if not callable(getter):
        return None
    prim = getter()
    return prim if prim is not None and prim.IsValid() else None


def _schema_attr(api, getter_name: str, attr_name: str, type_name, *, create: bool = False):  # noqa: ANN001, ANN202
    getter = getattr(api, getter_name, None)
    if callable(getter):
        attr = getter()
        if attr is not None and attr.IsValid():
            return attr
        if not create:
            return attr
    prim = _api_prim(api)
    if prim is None:
        return None
    attr = prim.GetAttribute(attr_name)
    if (attr is None or not attr.IsValid()) and create:
        attr = prim.CreateAttribute(attr_name, type_name, custom=False)
    return attr


def _drive_max_force_attr(  # noqa: ANN202
    drive: UsdPhysics.DriveAPI | None,
    prim: Usd.Prim,
    drive_type: str | None,
    *,
    create: bool = False,
):
    from pxr import Sdf

    if drive is None or drive_type is None:
        return None
    getter = getattr(drive, "GetMaxForceAttr", None)
    if callable(getter):
        attr = getter()
        if attr is not None and attr.IsValid():
            return attr
        if not create:
            return attr
    creator = getattr(drive, "CreateMaxForceAttr", None)
    if create and callable(creator):
        return creator()
    name = f"drive:{drive_type}:physics:maxForce"
    attr = prim.GetAttribute(name)
    if (attr is None or not attr.IsValid()) and create:
        attr = prim.CreateAttribute(name, Sdf.ValueTypeNames.Float, custom=False)
    return attr


def _read_joint_axis_friction(prim: Usd.Prim, drive_type: str | None) -> tuple[float, float, float]:
    if drive_type is None:
        return 0.0, 0.0, 0.0
    static = _float_attr(prim.GetAttribute(f"physxJointAxis:{drive_type}:staticFrictionEffort"))
    dynamic = _float_attr(prim.GetAttribute(f"physxJointAxis:{drive_type}:dynamicFrictionEffort"))
    viscous = _float_attr(prim.GetAttribute(f"physxJointAxis:{drive_type}:viscousFrictionCoefficient"))
    if drive_type == "angular" and viscous:
        viscous = float(np.rad2deg(viscous))
    return static, dynamic, viscous


def _write_joint_axis_friction(
    prim: Usd.Prim,
    drive_type: str | None,
    *,
    static_friction: float,
    dynamic_friction: float,
    viscous_friction: float,
) -> None:
    if drive_type is None:
        return
    physx_schema = _physx_schema()
    joint_axis_api = getattr(physx_schema, "PhysxJointAxisAPI", None) if physx_schema is not None else None
    if joint_axis_api is not None:
        try:
            joint_axis_api.Apply(prim, drive_type)
        except Exception:
            prim.ApplyAPI("PhysxJointAxisAPI", drive_type)
    _set_float_attr(prim, f"physxJointAxis:{drive_type}:staticFrictionEffort", max(static_friction, 0.0), custom=False)
    _set_float_attr(
        prim, f"physxJointAxis:{drive_type}:dynamicFrictionEffort", max(dynamic_friction, 0.0), custom=False
    )
    if drive_type == "angular" and viscous_friction:
        viscous_friction = float(np.deg2rad(viscous_friction))
    _set_float_attr(
        prim, f"physxJointAxis:{drive_type}:viscousFrictionCoefficient", max(viscous_friction, 0.0), custom=False
    )


def _drive_attr_to_effective(value: float, drive_type: str | None) -> float:
    if drive_type == "angular" and value:
        return float(1.0 / np.deg2rad(1.0 / value))
    return value


def _effective_to_drive_attr(value: float, drive_type: str | None) -> float:
    if drive_type == "angular" and value:
        return float(1.0 / np.rad2deg(1.0 / value))
    return value


def read_link_usd_snapshot(stage: Usd.Stage, link_path: str) -> LinkUsdSnapshot:
    """Read mass, COM, diagonal inertia, and contact offset from a link prim.

    Args:
        stage: USD stage used by the operation.
        link_path: Value supplied for ``link_path``.

    Returns:
        Result produced by the operation.
    """
    prim = stage.GetPrimAtPath(link_path)
    if not prim.IsValid():
        raise ValueError(f"Link prim not found: {link_path}")

    mass_api = _mass_api(prim)
    mass = 1.0 if mass_api is None else float(_attr_value_or_default(mass_api.GetMassAttr(), 1.0))
    if not np.isfinite(mass) or mass <= 0.0:
        mass = 1.0
    com_attr = None if mass_api is None else mass_api.GetCenterOfMassAttr().Get()
    if com_attr is None:
        com = np.zeros(3, dtype=np.float64)
    else:
        com = np.array([com_attr[0], com_attr[1], com_attr[2]], dtype=np.float64)
        if not np.all(np.isfinite(com)):
            com = np.zeros(3, dtype=np.float64)

    diag_attr = None if mass_api is None else mass_api.GetDiagonalInertiaAttr().Get()
    if diag_attr is None:
        inertia = np.ones(3, dtype=np.float64)
    else:
        inertia = np.array([diag_attr[0], diag_attr[1], diag_attr[2]], dtype=np.float64)
        if not np.all(np.isfinite(inertia)) or np.any(inertia <= 0.0):
            inertia = np.ones(3, dtype=np.float64)
    principal_axes_quat = None if mass_api is None else _read_principal_axes_quat(mass_api)
    inertia_matrix = _inertia_matrix_from_principal_axes(inertia, principal_axes_quat)

    contact_offset = _float_attr(prim.GetAttribute("physxCollision:contactOffset"))
    physx_schema = _physx_schema()
    if physx_schema is not None and prim.HasAPI(physx_schema.PhysxCollisionAPI):
        val = physx_schema.PhysxCollisionAPI(prim).GetContactOffsetAttr().Get()
        if val is not None:
            contact_offset = float(val)

    return LinkUsdSnapshot(
        path=link_path,
        mass=mass,
        com_offset=com,
        diagonal_inertia=inertia,
        contact_offset=contact_offset,
        principal_axes_quat=principal_axes_quat,
        inertia_matrix=inertia_matrix,
    )


def write_link_usd_snapshot(
    stage: Usd.Stage,
    snapshot: LinkUsdSnapshot,
    *,
    write_mass: bool = True,
    write_com: bool = True,
    write_inertia: bool = True,
    write_contact: bool = False,
) -> None:
    """Write link mass/COM/inertia back to USD, optionally including contact offset.

    Args:
        stage: USD stage used by the operation.
        snapshot: Value supplied for ``snapshot``.
        write_mass: Value supplied for ``write_mass``.
        write_com: Value supplied for ``write_com``.
        write_inertia: Value supplied for ``write_inertia``.
        write_contact: Whether to author the snapshot's contact offset.
    """
    from pxr import Gf, Sdf

    prim = stage.GetPrimAtPath(snapshot.path)
    if not prim.IsValid():
        raise ValueError(f"Link prim not found: {snapshot.path}")

    mass_api = _mass_api(prim, apply=True)
    if write_mass:
        mass_api.GetMassAttr().Set(float(snapshot.mass))
    com = snapshot.com_offset.reshape(3)
    if write_com:
        mass_api.GetCenterOfMassAttr().Set(Gf.Vec3f(float(com[0]), float(com[1]), float(com[2])))
    diag = snapshot.diagonal_inertia.reshape(3)
    principal_axes = snapshot.principal_axes_quat
    if snapshot.inertia_matrix is not None:
        diag, principal_axes = _principal_axes_from_inertia_matrix(snapshot.inertia_matrix)
    if write_inertia:
        mass_api.GetDiagonalInertiaAttr().Set(Gf.Vec3f(float(diag[0]), float(diag[1]), float(diag[2])))
    if write_inertia and principal_axes is not None:
        quat = np.asarray(principal_axes, dtype=np.float64).reshape(4)
        attr = _schema_attr(
            mass_api,
            "GetPrincipalAxesAttr",
            "physics:principalAxes",
            Sdf.ValueTypeNames.Quatf,
            create=True,
        )
        if attr is not None and attr.IsValid():
            attr.Set(Gf.Quatf(float(quat[0]), Gf.Vec3f(float(quat[1]), float(quat[2]), float(quat[3]))))

    if not write_contact:
        return
    physx_schema = _physx_schema()
    if physx_schema is not None and prim.HasAPI(physx_schema.PhysxCollisionAPI):
        physx_schema.PhysxCollisionAPI(prim).GetContactOffsetAttr().Set(float(snapshot.contact_offset))
    elif physx_schema is not None and snapshot.contact_offset != 0.0:
        physx_schema.PhysxCollisionAPI.Apply(prim).GetContactOffsetAttr().Set(float(snapshot.contact_offset))
    elif snapshot.contact_offset != 0.0 or prim.GetAttribute("physxCollision:contactOffset").IsValid():
        _set_float_attr(prim, "physxCollision:contactOffset", snapshot.contact_offset, custom=False)


def read_joint_usd_snapshot(
    stage: Usd.Stage,
    dof_path: str,
    *,
    dof_index: int = -1,
    physics_backend: str = "",
) -> Optional[JointUsdSnapshot]:
    """Read revolute/prismatic joint limits, drive gains, and friction from a DOF prim path.

    When ``physics_backend`` is ``"newton"``, authored ``newton:friction`` and ``newton:armature``
    values take precedence over their PhysX counterparts. When it is ``"physx"``, Newton values are
    ignored. When it is unspecified, a Newton value is used only when the matching PhysX value is
    unauthored. A Newton-sourced friction reports zero static, dynamic, and viscous friction because
    Newton does not apply the PhysX per-axis friction. Drive damping is always read from the drive.

    Args:
        stage: USD stage used by the operation.
        dof_path: Value supplied for ``dof_path``.
        dof_index: Value supplied for ``dof_index``.
        physics_backend: Backend whose solver-specific joint properties are read.

    Returns:
        Result produced by the operation.

    Raises:
        ValueError: If ``physics_backend`` is not empty, ``"auto"``, ``"newton"``, or ``"physx"``.
    """
    from pxr import UsdPhysics

    backend = _normalize_physics_backend(physics_backend)
    prim = stage.GetPrimAtPath(dof_path)
    if not prim.IsValid():
        return None

    physx_friction_attr = prim.GetAttribute("physxJoint:jointFriction")
    physx_armature_attr = prim.GetAttribute("physxJoint:armature")
    friction = _float_attr(physx_friction_attr)
    armature = _float_attr(physx_armature_attr)
    physx_schema = _physx_schema()
    if physx_schema is not None and prim.HasAPI(physx_schema.PhysxJointAPI):
        physx_joint = physx_schema.PhysxJointAPI(prim)
        physx_friction_attr = physx_joint.GetJointFrictionAttr()
        physx_armature_attr = physx_joint.GetArmatureAttr()
        friction = _float_attr(physx_friction_attr)
        armature = _float_attr(physx_armature_attr)

    newton_friction_attr = prim.GetAttribute("newton:friction")
    newton_armature_attr = prim.GetAttribute("newton:armature")
    uses_newton_friction = _uses_newton_attr(newton_friction_attr, physx_friction_attr, backend)
    uses_newton_armature = _uses_newton_attr(newton_armature_attr, physx_armature_attr, backend)
    if uses_newton_friction:
        friction = _float_attr(newton_friction_attr)
    if uses_newton_armature:
        armature = _float_attr(newton_armature_attr)

    drive_type = _drive_instance_for_joint(prim)
    static_friction, dynamic_friction, viscous_friction = (
        (0.0, 0.0, 0.0) if uses_newton_friction else _read_joint_axis_friction(prim, drive_type)
    )
    # Keep ``friction`` as the authored physxJoint:jointFriction and preserve the
    # per-axis dynamic friction separately. Overwriting one with the other here
    # silently dropped a real baseline; write_usd_from_baselines already
    # cross-fills between the two when one is absent.
    drive = _drive_api(prim)
    stiffness = _drive_attr_to_effective(
        _float_attr(drive.GetStiffnessAttr()) if drive is not None else 0.0,
        drive_type,
    )
    damping = _drive_attr_to_effective(
        _float_attr(drive.GetDampingAttr()) if drive is not None else 0.0,
        drive_type,
    )
    max_force = (
        _float_attr(_drive_max_force_attr(drive, prim, drive_type), float("inf")) if drive is not None else float("inf")
    )

    if prim.IsA(UsdPhysics.RevoluteJoint):
        joint = UsdPhysics.RevoluteJoint(prim)
        lower = float(_authored_attr_value_or_default(joint.GetLowerLimitAttr(), -180.0))
        upper = float(_authored_attr_value_or_default(joint.GetUpperLimitAttr(), 180.0))
        return JointUsdSnapshot(
            path=dof_path,
            lower_limit=np.deg2rad(lower),
            upper_limit=np.deg2rad(upper),
            is_angular=True,
            friction=friction,
            static_friction=static_friction,
            dynamic_friction=dynamic_friction,
            viscous_friction=viscous_friction,
            stiffness=stiffness,
            damping=damping,
            armature=armature,
            max_force=max_force,
            dof_index=dof_index,
            uses_newton_friction=uses_newton_friction,
            uses_newton_armature=uses_newton_armature,
        )

    if prim.IsA(UsdPhysics.PrismaticJoint):
        joint = UsdPhysics.PrismaticJoint(prim)
        lower = float(_authored_attr_value_or_default(joint.GetLowerLimitAttr(), -1.0))
        upper = float(_authored_attr_value_or_default(joint.GetUpperLimitAttr(), 1.0))
        return JointUsdSnapshot(
            path=dof_path,
            lower_limit=lower,
            upper_limit=upper,
            is_angular=False,
            friction=friction,
            static_friction=static_friction,
            dynamic_friction=dynamic_friction,
            viscous_friction=viscous_friction,
            stiffness=stiffness,
            damping=damping,
            armature=armature,
            max_force=max_force,
            dof_index=dof_index,
            uses_newton_friction=uses_newton_friction,
            uses_newton_armature=uses_newton_armature,
        )

    return None


def write_joint_usd_snapshot(
    stage: Usd.Stage,
    snapshot: JointUsdSnapshot,
    *,
    physics_backend: str = "",
    write_stiffness: bool = True,
    write_damping: bool = True,
    write_limits: bool = True,
    write_friction: bool = True,
    write_armature: bool = True,
) -> None:
    """Write joint drive gains, friction, and limits to USD.

    Args:
        stage: USD stage used by the operation.
        snapshot: Value supplied for ``snapshot``.
        physics_backend: Selected backend that owns solver-specific properties.
        write_stiffness: Value supplied for ``write_stiffness``.
        write_damping: Value supplied for ``write_damping``.
        write_limits: Value supplied for ``write_limits``.
        write_friction: Value supplied for ``write_friction``.
        write_armature: Value supplied for ``write_armature``.
    """
    from pxr import UsdPhysics

    prim = stage.GetPrimAtPath(snapshot.path)
    if not prim.IsValid():
        raise ValueError(f"Joint prim not found: {snapshot.path}")

    if write_limits and prim.IsA(UsdPhysics.RevoluteJoint):
        joint = UsdPhysics.RevoluteJoint(prim)
        lower = np.rad2deg(snapshot.lower_limit) if snapshot.is_angular else snapshot.lower_limit
        upper = np.rad2deg(snapshot.upper_limit) if snapshot.is_angular else snapshot.upper_limit
        joint.GetLowerLimitAttr().Set(float(lower))
        joint.GetUpperLimitAttr().Set(float(upper))
    elif write_limits and prim.IsA(UsdPhysics.PrismaticJoint):
        joint = UsdPhysics.PrismaticJoint(prim)
        joint.GetLowerLimitAttr().Set(float(snapshot.lower_limit))
        joint.GetUpperLimitAttr().Set(float(snapshot.upper_limit))
    elif write_limits:
        raise ValueError(f"Unsupported joint prim for limit write: {snapshot.path}")

    backend = _normalize_physics_backend(physics_backend)
    use_newton_friction = snapshot.uses_newton_friction if not backend else backend == "newton"
    use_newton_armature = snapshot.uses_newton_armature if not backend else backend == "newton"

    write_drive = bool(write_stiffness or write_damping)
    drive = _drive_api(prim, apply=write_drive) if write_drive else None
    drive_type = _drive_instance_for_joint(prim)
    if drive is not None:
        if write_stiffness:
            drive.CreateStiffnessAttr().Set(float(max(_effective_to_drive_attr(snapshot.stiffness, drive_type), 0.0)))
        if write_damping:
            drive.CreateDampingAttr().Set(float(max(_effective_to_drive_attr(snapshot.damping, drive_type), 0.0)))
        if np.isfinite(snapshot.max_force):
            attr = _drive_max_force_attr(drive, prim, drive_type, create=True)
            if attr is not None and attr.IsValid():
                attr.Set(float(max(snapshot.max_force, 0.0)))

    static_friction = snapshot.static_friction
    dynamic_friction = snapshot.dynamic_friction
    if static_friction == 0.0 and snapshot.friction != 0.0:
        static_friction = snapshot.friction
    if dynamic_friction == 0.0 and snapshot.friction != 0.0:
        dynamic_friction = snapshot.friction
    writes_newton = bool((write_friction and use_newton_friction) or (write_armature and use_newton_armature))
    if writes_newton and "NewtonJointAPI" not in prim.GetAppliedSchemas():
        if not prim.ApplyAPI("NewtonJointAPI"):
            raise RuntimeError(f"Could not apply NewtonJointAPI to joint prim: {snapshot.path}")

    write_physx_friction = bool(write_friction and not use_newton_friction)
    write_physx_armature = bool(write_armature and not use_newton_armature)
    if write_friction and use_newton_friction:
        _set_float_attr(prim, "newton:friction", max(snapshot.friction, 0.0), custom=False)
    if write_armature and use_newton_armature:
        _set_float_attr(prim, "newton:armature", max(snapshot.armature, 0.0), custom=False)
    if write_physx_friction:
        _write_joint_axis_friction(
            prim,
            drive_type,
            static_friction=static_friction,
            dynamic_friction=dynamic_friction,
            viscous_friction=snapshot.viscous_friction,
        )
    physx_schema = _physx_schema()
    if (
        (write_physx_friction or write_physx_armature)
        and physx_schema is not None
        and (prim.HasAPI(physx_schema.PhysxJointAPI) or snapshot.friction != 0.0 or snapshot.armature != 0.0)
    ):
        physx_joint = (
            physx_schema.PhysxJointAPI(prim)
            if prim.HasAPI(physx_schema.PhysxJointAPI)
            else physx_schema.PhysxJointAPI.Apply(prim)
        )
        if write_physx_friction:
            physx_joint.CreateJointFrictionAttr().Set(float(max(snapshot.friction, 0.0)))
        if write_physx_armature:
            physx_joint.CreateArmatureAttr().Set(float(max(snapshot.armature, 0.0)))
    elif write_physx_friction and (snapshot.friction != 0.0 or prim.GetAttribute("physxJoint:jointFriction").IsValid()):
        _set_float_attr(prim, "physxJoint:jointFriction", max(snapshot.friction, 0.0), custom=False)
    if (
        write_physx_armature
        and physx_schema is None
        and (snapshot.armature != 0.0 or prim.GetAttribute("physxJoint:armature").IsValid())
    ):
        _set_float_attr(prim, "physxJoint:armature", max(snapshot.armature, 0.0), custom=False)


def read_articulation_usd_snapshots(
    stage: Usd.Stage,
    link_paths: list[str],
    dof_paths: list[str],
    *,
    physics_backend: str = "",
) -> tuple[list[LinkUsdSnapshot], list[Optional[JointUsdSnapshot]]]:
    """Bulk-read link and joint USD snapshots for an articulation.

    Args:
        stage: USD stage used by the operation.
        link_paths: Value supplied for ``link_paths``.
        dof_paths: Value supplied for ``dof_paths``.
        physics_backend: Backend whose solver-specific joint properties are read.

    Returns:
        Result produced by the operation.
    """
    links = [read_link_usd_snapshot(stage, path) for path in link_paths]
    joints = [
        read_joint_usd_snapshot(stage, path, dof_index=index, physics_backend=physics_backend)
        for index, path in enumerate(dof_paths)
    ]
    return links, joints


def _read_principal_axes_quat(mass_api: UsdPhysics.MassAPI) -> np.ndarray | None:
    from pxr import Sdf

    attr = _schema_attr(
        mass_api,
        "GetPrincipalAxesAttr",
        "physics:principalAxes",
        Sdf.ValueTypeNames.Quatf,
    )
    if attr is None or not attr.IsValid():
        return None
    value = attr.Get()
    if value is None:
        return None
    try:
        imag = value.GetImaginary()
        quat = np.array([value.GetReal(), imag[0], imag[1], imag[2]], dtype=np.float64)
    except Exception:
        return None
    norm = float(np.linalg.norm(quat))
    if not np.isfinite(norm) or norm <= 0.0:
        return None
    return quat / norm


def _inertia_matrix_from_principal_axes(diag: np.ndarray, quat: np.ndarray | None) -> np.ndarray:
    diagonal = np.asarray(diag, dtype=np.float64).reshape(3)
    if quat is None:
        return np.diag(diagonal)
    rotation = _quat_to_matrix(quat)
    return rotation @ np.diag(diagonal) @ rotation.T


def _principal_axes_from_inertia_matrix(matrix: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    full = np.asarray(matrix, dtype=np.float64).reshape(3, 3)
    full = 0.5 * (full + full.T)
    try:
        eigvals, eigvecs = np.linalg.eigh(full)
    except np.linalg.LinAlgError:
        return np.ones(3, dtype=np.float64), np.array([1.0, 0.0, 0.0, 0.0], dtype=np.float64)
    eigvals = np.maximum(eigvals, 1e-9)
    if np.linalg.det(eigvecs) < 0.0:
        eigvecs[:, 0] *= -1.0
    return eigvals.astype(np.float64), _matrix_to_quat(eigvecs)


def _quat_to_matrix(quat: np.ndarray) -> np.ndarray:
    w, x, y, z = np.asarray(quat, dtype=np.float64).reshape(4)
    norm = max(float(np.linalg.norm([w, x, y, z])), 1e-12)
    w, x, y, z = w / norm, x / norm, y / norm, z / norm
    return np.array(
        [
            [1.0 - 2.0 * (y * y + z * z), 2.0 * (x * y - z * w), 2.0 * (x * z + y * w)],
            [2.0 * (x * y + z * w), 1.0 - 2.0 * (x * x + z * z), 2.0 * (y * z - x * w)],
            [2.0 * (x * z - y * w), 2.0 * (y * z + x * w), 1.0 - 2.0 * (x * x + y * y)],
        ],
        dtype=np.float64,
    )


def _matrix_to_quat(matrix: np.ndarray) -> np.ndarray:
    m = np.asarray(matrix, dtype=np.float64).reshape(3, 3)
    trace = float(np.trace(m))
    if trace > 0.0:
        s = np.sqrt(trace + 1.0) * 2.0
        quat = np.array([0.25 * s, (m[2, 1] - m[1, 2]) / s, (m[0, 2] - m[2, 0]) / s, (m[1, 0] - m[0, 1]) / s])
    else:
        i = int(np.argmax(np.diag(m)))
        if i == 0:
            s = np.sqrt(1.0 + m[0, 0] - m[1, 1] - m[2, 2]) * 2.0
            quat = np.array([(m[2, 1] - m[1, 2]) / s, 0.25 * s, (m[0, 1] + m[1, 0]) / s, (m[0, 2] + m[2, 0]) / s])
        elif i == 1:
            s = np.sqrt(1.0 + m[1, 1] - m[0, 0] - m[2, 2]) * 2.0
            quat = np.array([(m[0, 2] - m[2, 0]) / s, (m[0, 1] + m[1, 0]) / s, 0.25 * s, (m[1, 2] + m[2, 1]) / s])
        else:
            s = np.sqrt(1.0 + m[2, 2] - m[0, 0] - m[1, 1]) * 2.0
            quat = np.array([(m[1, 0] - m[0, 1]) / s, (m[0, 2] + m[2, 0]) / s, (m[1, 2] + m[2, 1]) / s, 0.25 * s])
    norm = max(float(np.linalg.norm(quat)), 1e-12)
    return (quat / norm).astype(np.float64)
